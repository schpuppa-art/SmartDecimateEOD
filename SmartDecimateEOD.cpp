///////////////////////////////////////////////////////////////////////////////
//  SmartDecimateEOD.cpp — content-aware decimator for animation
//
//  MIT License — Copyright (c) 2026 Shurik Pronkin
//  https://github.com/schpuppa-art
//
//  Unlike classical decimators (srestore/TDecimate) which enforce a fixed
//  per-chunk ratio, SmartDecimateEOD uses a GLOBAL budget: the total output
//  frame count is fixed by fps_out (CFR, audio-safe), but the distribution
//  of kept frames across scenes is content-adaptive.
//
//  Rationale for animation:
//    - On-twos scenes (O D D D D D D D) contribute surplus duplicates
//    - On-ones scenes (O O O O O O O O) contribute zero duplicates
//    - Classical fixed-ratio decimation cuts originals in on-ones scenes,
//      destroying real drawn motion
//    - Smart decimation uses duplicates from on-twos scenes as "credit"
//      to preserve all originals in on-ones scenes, at fixed total length
//
//  Pipeline ordering (recommended):
//      src → SmartDecimateEOD(scenes_file=..., cache=...) → RIFE
//      (or legacy: src → SCDetectEOD(...) → SmartDecimateEOD(sc=..., cache=...))
//
//  Boundary source (scenes_file wins if both given):
//    scenes_file — direct read of SCDetectEOD's scenes.txt, no sc clip needed.
//                  meanY is computed per-frame from thumbnails in Pass 1A, so
//                  fade detection works.
//    sc clip     — boundaries + _scd_meanY via frame-props (legacy).
//
//  Two-pass eager preanalysis in constructor (stateless, AvsPmod-safe):
//    PASS 1 — single decode per scene: SSIM thumbnail + downscaled
//              registration buffer in one sweep; classify ORIG/DUP per scene;
//              unimodal-variance guard forces all-ORIG on full on-ones scenes;
//              monotonic meanY drift forces all-ORIG on fades
//    PASS 2  — global budget: keep all ORIG, fill remaining budget with the
//              first DUP after each ORIG across scenes (preserves local
//              on-twos rhythm); error-diffusion across scenes for exact total
//
//  Props written (debug=true):
//    _sdec_sceneId   (int)      scene index of the source frame this output came from
//    _sdec_isDup     (int 0/1)  classification of source frame
//    _sdec_isFade    (int 0/1)  1 if scene was marked fade/dissolve
//    _sdec_srcFrame  (int)      source frame index
//
//  Build: cl /LD /EHsc /O2 SmartDecimateEOD.cpp /link /OUT:SmartDecimateEOD.dll
//  MSVC required — AviSynth+ ABI incompatible with MinGW.
///////////////////////////////////////////////////////////////////////////////

#define NOMINMAX
#include <windows.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "avisynth.h"

// Cyrillic/UTF-8 safe fopen: AviSynth passes paths as UTF-8; CRT fopen on
// ru-Windows interprets them as cp1251 and fails on Cyrillic. Convert to
// wide and use _wfopen.
static FILE* fopen_utf8(const char* path, const wchar_t* mode) {
    if (!path || !*path) return nullptr;
    int wl = MultiByteToWideChar(CP_UTF8, 0, path, -1, nullptr, 0);
    if (wl <= 0) return nullptr;
    std::vector<wchar_t> wpath(wl);
    MultiByteToWideChar(CP_UTF8, 0, path, -1, wpath.data(), wl);
    return _wfopen(wpath.data(), mode);
}

static const uint32_t SDEC_MAGIC   = 0x43454453; // 'SDEC' little-endian
static const uint32_t SDEC_VERSION = 3;         // v3: + out_boundary (decimated scene cuts)

static uint64_t fnv1a(const void* data, size_t len, uint64_t seed = 0xcbf29ce484222325ULL) {
    const uint8_t* p = (const uint8_t*)data;
    uint64_t h = seed;
    for (size_t i = 0; i < len; ++i) { h ^= p[i]; h *= 0x100000001b3ULL; }
    return h;
}

// Recursively create directories for a file path.
static void ensure_dir_for_file(const char* path) {
    std::string s(path);
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '/' || s[i] == '\\') {
            std::string dir = s.substr(0, i);
            if (!dir.empty()) CreateDirectoryA(dir.c_str(), NULL);
        }
    }
}

class SmartDecimateEOD : public GenericVideoFilter {
    PClip sc_clip;

    // Params
    double fps_out;
    double q_low_pct;          // low quantile percentile on SSIM (inverted: high = dup)
    int    th_ssim_dup;        // SSIM guard: q80 < this → scene has no dups (skip registration)
    double residual_ceiling;   // abs residual cap for percentile guard
    double residual_mult;      // threshold = q_low * mult
    double th_ratio;           // block max/mean ratio threshold
    int    reg_scale;          // downscale factor for registration SAD (1=full, 2=half, 4=quarter)
    bool   full_range;
    int    thumb_w;            // thumbnail width for SSIM (default 480)
    int    thumb_h;            // thumbnail height for SSIM (default 360)
    double bypass_alt;         // alternation threshold for RIFE eligibility
    int    bypass_min_len;     // minimum scene length for RIFE eligibility
    int    bypass_window;      // half-window for local alternation (frames)
    std::string mode;          // "cfr" (budgeted) or "content" (drop all dups)
    int    fade_ratio;           // percent 0..100
    int    fade_mean_drift;      // minimum |meanY[end]-meanY[start]| for fade
    bool   protect_fade;
    bool   allow_original_cut;
    bool   debug;
    bool   analyze_only;
    std::string cache_path;
    std::string log_path;
    std::string analyze_log_path;
    std::string scuts_out_path;
    std::string scenes_file_path;   // direct boundary source; wins over sc_clip

    // Diagnostic arrays
    std::vector<int>     d_ssim;         // size N  SSIM×1000 (thumbnail)
    std::vector<double>  d_residual;     // size N  registered SAD residual
    std::vector<double>  d_ratio;        // size N  block max/mean ratio
    std::vector<double>  d_otsu;         // size N  scene threshold (broadcast)
    std::vector<uint8_t> d_unimodal;
    std::vector<int32_t> d_sceneLen;
    std::vector<int32_t> d_sceneOrig;
    std::vector<int32_t> d_sceneDup;
    std::vector<int32_t> d_sceneStart;
    std::vector<double>  d_alternation;  // per-scene O/D alternation rate
    std::vector<double>  d_local_alt;    // per-frame local alternation (sliding window)

    // Preanalysis output
    std::vector<uint8_t> keep_mask;    // size N  (1 = kept)
    std::vector<int32_t> out_to_src;   // size T  (source index per output frame)
    std::vector<uint8_t> out_boundary; // size T  (1 = scene boundary on decimated timeline)
    std::vector<int32_t> scene_of;     // size N  (scene id per source frame, debug)
    std::vector<uint8_t> is_dup;       // size N  (debug)
    std::vector<uint8_t> is_fade_src;  // size N  (debug, per-frame copy of scene flag)

    int N_in;
    int N_out;
    int fps_num_out;
    int fps_den_out;

    // ─── helpers ───────────────────────────────────────────────────────────

    static double scene_median(std::vector<double>& v) {
        if (v.empty()) return 0.0;
        std::vector<double> tmp = v;
        std::sort(tmp.begin(), tmp.end());
        return tmp[tmp.size() / 2];
    }

    // Otsu's method on a list of doubles, normalized to 256-bin histogram.
    // Returns a threshold in original units.
    static double otsu_threshold(const std::vector<double>& diffs) {
        if (diffs.size() < 4) return 0.0;
        double lo = *std::min_element(diffs.begin(), diffs.end());
        double hi = *std::max_element(diffs.begin(), diffs.end());
        if (hi - lo < 1e-6) return lo;
        int hist[256] = {0};
        for (double d : diffs) {
            int b = (int)((d - lo) / (hi - lo) * 255.0);
            if (b < 0) b = 0; if (b > 255) b = 255;
            hist[b]++;
        }
        int total = (int)diffs.size();
        double sum = 0;
        for (int i = 0; i < 256; ++i) sum += (double)i * hist[i];
        double sumB = 0, wB = 0, maxVar = -1;
        int thr = 0;
        for (int i = 0; i < 256; ++i) {
            wB += hist[i]; if (wB == 0) continue;
            double wF = total - wB; if (wF == 0) break;
            sumB += (double)i * hist[i];
            double mB = sumB / wB;
            double mF = (sum - sumB) / wF;
            double var = wB * wF * (mB - mF) * (mB - mF);
            if (var > maxVar) { maxVar = var; thr = i; }
        }
        return lo + (hi - lo) * thr / 255.0;
    }

    // ─── SSIM-based dup detection (ported from SCDetectEOD v2) ────────
    // Thumbnail: box-average Y to thumb_w×thumb_h + studio→full + 3×3 blur.
    // SSIM on thumbnail: 0..1000.  High = similar = duplicate.

    void make_thumb(PVideoFrame& f, uint8_t* thumb, bool full_range_src) {
        const uint8_t* yp = f->GetReadPtr(PLANAR_Y);
        int pitch = f->GetPitch(PLANAR_Y);
        int W = vi.width, H = vi.height;
        int tw = thumb_w, th = thumb_h;
        for (int dy = 0; dy < th; dy++) {
            int sy0 = dy * H / th;
            int sy1 = (dy + 1) * H / th;
            if (sy1 <= sy0) sy1 = sy0 + 1;
            for (int dx = 0; dx < tw; dx++) {
                int sx0 = dx * W / tw;
                int sx1 = (dx + 1) * W / tw;
                if (sx1 <= sx0) sx1 = sx0 + 1;
                int acc = 0, cnt = 0;
                for (int y = sy0; y < sy1 && y < H; y++) {
                    const uint8_t* row = yp + y * pitch;
                    for (int x = sx0; x < sx1 && x < W; x++) {
                        acc += row[x]; cnt++;
                    }
                }
                int avg = cnt > 0 ? acc / cnt : 0;
                if (!full_range_src) {
                    avg = (avg - 16) * 255 / 219;
                    if (avg < 0) avg = 0;
                    if (avg > 255) avg = 255;
                }
                thumb[dy * tw + dx] = (uint8_t)avg;
            }
        }
        // 3×3 box blur
        std::vector<uint8_t> tmp(tw * th);
        std::memcpy(tmp.data(), thumb, tw * th);
        for (int y = 1; y < th - 1; y++) {
            for (int x = 1; x < tw - 1; x++) {
                int sum9 = 0;
                for (int dy2 = -1; dy2 <= 1; dy2++)
                    for (int dx2 = -1; dx2 <= 1; dx2++)
                        sum9 += tmp[(y + dy2) * tw + (x + dx2)];
                thumb[y * tw + x] = (uint8_t)(sum9 / 9);
            }
        }
    }

    // Mean luma of a built thumbnail (full-range Y, 0..255). Matches the scale
    // of SCDetectEOD's _scd_meanY, so fade drift uses the same numbers whether
    // meanY came from sc_clip props or was computed here in scenes_file mode.
    static int thumb_mean(const uint8_t* thumb, int n) {
        if (n <= 0) return 0;
        long acc = 0;
        for (int i = 0; i < n; ++i) acc += thumb[i];
        return (int)(acc / n);
    }

    static int compute_ssim(const uint8_t* a, const uint8_t* b, int n) {
        double sum_a = 0, sum_b = 0, sum_a2 = 0, sum_b2 = 0, sum_ab = 0;
        for (int i = 0; i < n; i++) {
            double va = a[i], vb = b[i];
            sum_a += va; sum_b += vb;
            sum_a2 += va * va; sum_b2 += vb * vb;
            sum_ab += va * vb;
        }
        double mu_a = sum_a / n, mu_b = sum_b / n;
        double var_a = sum_a2 / n - mu_a * mu_a;
        double var_b = sum_b2 / n - mu_b * mu_b;
        double cov = sum_ab / n - mu_a * mu_b;
        const double C1 = 6.5025, C2 = 58.5225;
        double num = (2.0 * mu_a * mu_b + C1) * (2.0 * cov + C2);
        double den = (mu_a * mu_a + mu_b * mu_b + C1) * (var_a + var_b + C2);
        double ssim = (den > 0) ? (num / den) : 0.0;
        if (ssim < 0.0) ssim = 0.0;
        if (ssim > 1.0) ssim = 1.0;
        return (int)(ssim * 1000);
    }

    // ─── Registration + SAD + block ratio (from v3.5) ──────────────────
    // Operates on a pre-downscaled DENSE Y buffer (W_s × H_s) instead of the
    // full-res frame. The downscale (box-average by reg_scale) is done once
    // per frame; the 49-pass grid search then runs on the small contiguous
    // buffer — cache-friendly and reg_scale^2 fewer pixels. On 4K this is the
    // dominant cost, so it makes Pass 1 fast.
    int rs_w() const { int rs = reg_scale < 1 ? 1 : reg_scale; return vi.width / rs; }
    int rs_h() const { int rs = reg_scale < 1 ? 1 : reg_scale; return vi.height / rs; }

    void downscale_into(PVideoFrame& f, uint8_t* out) {
        const uint8_t* yp = f->GetReadPtr(PLANAR_Y);
        int pitch = f->GetPitch(PLANAR_Y);
        int W = vi.width, H = vi.height;
        int rs = reg_scale < 1 ? 1 : reg_scale;
        int ow = W / rs, oh = H / rs;
        if (rs == 1) {
            for (int y = 0; y < oh; ++y)
                std::memcpy(out + (size_t)y * ow, yp + (size_t)y * pitch, ow);
            return;
        }
        for (int y = 0; y < oh; ++y) {
            for (int x = 0; x < ow; ++x) {
                int acc = 0;
                for (int dy = 0; dy < rs; ++dy) {
                    const uint8_t* row = yp + (size_t)(y * rs + dy) * pitch + x * rs;
                    for (int dx = 0; dx < rs; ++dx) acc += row[dx];
                }
                out[(size_t)y * ow + x] = (uint8_t)(acc / (rs * rs));
            }
        }
    }

    void downscale_y(PVideoFrame& f, std::vector<uint8_t>& out) {
        out.resize((size_t)rs_w() * rs_h());
        downscale_into(f, out.data());
    }

    static double sad_buf(const uint8_t* p0, const uint8_t* p1,
                          int W, int H, int dx, int dy, int stride = 1) {
        int margin = 4;
        int64_t sad = 0, cnt = 0;
        for (int y = margin; y < H - margin; y += stride) {
            int y1 = y + dy;
            if (y1 < 0 || y1 >= H) continue;
            const uint8_t* r0 = p0 + (size_t)y * W;
            const uint8_t* r1 = p1 + (size_t)y1 * W;
            for (int x = margin; x < W - margin; x += stride) {
                int x1 = x + dx;
                if (x1 < 0 || x1 >= W) continue;
                sad += std::abs((int)r0[x] - (int)r1[x1]);
                cnt++;
            }
        }
        return cnt ? (double)sad / (double)cnt : 0.0;
    }

    // prev_buf and cur_buf are pre-downscaled dense Y buffers (rs_w × rs_h).
    void compute_pair_buf(const uint8_t* p0, const uint8_t* p1,
                          double& residual, double& ratio) {
        int W = rs_w(), H = rs_h();
        // Coarse grid search ±3 on a strided subsample (every 4th pixel) —
        // finding the shift doesn't need every pixel. Final residual at the
        // chosen shift is computed densely.
        int gstride = 4;
        int best_dx = 0, best_dy = 0; double best = 1e30;
        for (int dy = -3; dy <= 3; ++dy)
            for (int dx = -3; dx <= 3; ++dx) {
                double s = sad_buf(p0, p1, W, H, dx, dy, gstride);
                if (s < best) { best = s; best_dx = dx; best_dy = dy; }
            }
        residual = sad_buf(p0, p1, W, H, best_dx, best_dy, 2);
        // Block ratio
        int BS = 16;
        int nbx = (W - 8) / BS, nby = (H - 8) / BS;
        if (nbx < 1 || nby < 1) { ratio = 1.0; return; }
        std::vector<double> blocks;
        blocks.reserve((size_t)nbx * nby);
        for (int by = 0; by < nby; ++by)
            for (int bx = 0; bx < nbx; ++bx) {
                int x0 = 4 + bx * BS, y0 = 4 + by * BS;
                int64_t sad2 = 0, cnt2 = 0;
                for (int j = 0; j < BS; ++j) {
                    int y1 = y0 + j + best_dy;
                    if (y1 < 0 || y1 >= H) continue;
                    const uint8_t* r0 = p0 + (size_t)(y0 + j) * W;
                    const uint8_t* r1 = p1 + (size_t)y1 * W;
                    for (int i = 0; i < BS; ++i) {
                        int x1 = x0 + i + best_dx;
                        if (x1 < 0 || x1 >= W) continue;
                        sad2 += std::abs((int)r0[x0 + i] - (int)r1[x1]);
                        cnt2++;
                    }
                }
                if (cnt2) blocks.push_back((double)sad2 / (double)cnt2);
            }
        if (blocks.empty()) { ratio = 1.0; return; }
        double mx = 0, sm = 0;
        for (double v : blocks) { if (v > mx) mx = v; sm += v; }
        double mn = sm / blocks.size();
        ratio = mn > 0.01 ? mx / mn : 1.0;
    }

    // Deterministic params hash: serialize each field into a packed buffer
    // (no struct padding) so identical settings always hash identically.
    uint64_t params_hash_for(int mode_id) const {
        uint8_t buf[128];
        size_t o = 0;
        auto putd = [&](double v){ std::memcpy(buf+o,&v,sizeof(v)); o+=sizeof(v); };
        auto puti = [&](int v){ std::memcpy(buf+o,&v,sizeof(v)); o+=sizeof(v); };
        auto putb = [&](uint8_t v){ buf[o++]=v; };
        putd(fps_out); putd(bypass_alt); putd(residual_ceiling);
        putd(residual_mult); putd(th_ratio);
        puti(th_ssim_dup); puti((int)full_range); puti(mode_id);
        puti(fade_ratio); puti(fade_mean_drift);
        puti(thumb_w); puti(thumb_h); puti(reg_scale);
        putb((uint8_t)bypass_min_len); putb((uint8_t)bypass_window);
        putb((uint8_t)(protect_fade?1:0)); putb((uint8_t)(allow_original_cut?1:0));
        return fnv1a(buf, o);
    }

    bool try_load_cache(uint64_t params_hash) {
        if (cache_path.empty()) return false;
        FILE* f = fopen(cache_path.c_str(), "rb");
        if (!f) return false;
        uint32_t magic = 0, ver = 0, n = 0, t = 0, fn = 0, fd = 0;
        uint64_t ph = 0;
        bool ok = fread(&magic,sizeof(magic),1,f)==1 &&
                  fread(&ver,sizeof(ver),1,f)==1 &&
                  fread(&n,sizeof(n),1,f)==1 &&
                  fread(&t,sizeof(t),1,f)==1 &&
                  fread(&fn,sizeof(fn),1,f)==1 &&
                  fread(&fd,sizeof(fd),1,f)==1 &&
                  fread(&ph,sizeof(ph),1,f)==1;
        if (!ok || magic != SDEC_MAGIC || ver != SDEC_VERSION ||
            (int)n != N_in || ph != params_hash) {
            fprintf(stderr, "[SmartDecimateEOD] cache reject: ok=%d magic=%s ver=%u(want %u) "
                    "n=%u(want %d) hash=%llu(want %llu)\n",
                    (int)ok, (magic==SDEC_MAGIC?"ok":"BAD"), ver, SDEC_VERSION,
                    n, N_in, (unsigned long long)ph, (unsigned long long)params_hash);
            fflush(stderr);
            fclose(f); return false;
        }
        keep_mask.assign(n, 0);
        out_to_src.assign(t, 0);
        ok = fread(keep_mask.data(), 1, n, f) == n &&
             fread(out_to_src.data(), sizeof(int32_t), t, f) == t;
        if (!ok) { fclose(f); return false; }
        out_boundary.assign(t, 0);
        if (fread(out_boundary.data(), 1, t, f) != t) out_boundary.assign(t, 0);
        N_out = (int)t;
        fps_num_out = (int)fn;
        fps_den_out = (int)fd;
        // v2: try loading diagnostic arrays (optional, for mark_only reuse)
        is_dup.assign(n, 0);
        scene_of.assign(n, 0);
        is_fade_src.assign(n, 0);
        d_unimodal.assign(n, 0);
        d_alternation.assign(n, 0.0);
        d_local_alt.assign(n, 0.0);
        d_sceneLen.assign(n, 0);
        d_sceneOrig.assign(n, 0);
        d_sceneDup.assign(n, 0);
        d_sceneStart.assign(n, 0);
        d_ssim.assign(n, 0);
        d_residual.assign(n, 0.0);
        d_ratio.assign(n, 1.0);
        d_otsu.assign(n, 0.0);
        bool diag_ok =
            fread(is_dup.data(), 1, n, f) == n &&
            fread(scene_of.data(), sizeof(int32_t), n, f) == n &&
            fread(is_fade_src.data(), 1, n, f) == n &&
            fread(d_unimodal.data(), 1, n, f) == n &&
            fread(d_alternation.data(), sizeof(double), n, f) == n &&
            fread(d_local_alt.data(), sizeof(double), n, f) == n &&
            fread(d_sceneLen.data(), sizeof(int32_t), n, f) == n &&
            fread(d_sceneOrig.data(), sizeof(int32_t), n, f) == n &&
            fread(d_sceneDup.data(), sizeof(int32_t), n, f) == n &&
            fread(d_sceneStart.data(), sizeof(int32_t), n, f) == n &&
            fread(d_ssim.data(), sizeof(int), n, f) == n &&
            fread(d_residual.data(), sizeof(double), n, f) == n &&
            fread(d_ratio.data(), sizeof(double), n, f) == n &&
            fread(d_otsu.data(), sizeof(double), n, f) == n;
        if (!diag_ok) {
            // v1 cache without diagnostics — clear arrays
            is_dup.assign(n, 0); scene_of.assign(n, 0);
            is_fade_src.assign(n, 0); d_unimodal.assign(n, 0);
            d_alternation.assign(n, 0.0); d_local_alt.assign(n, 0.0);
        }
        fclose(f);
        return true;
    }

    void build_out_boundary() {
        out_boundary.assign(N_out, 0);
        if (scene_of.empty()) return;
        for (int j = 0; j < N_out; ++j) {
            int sc_cur  = scene_of[out_to_src[j]];
            int sc_prev = (j > 0) ? scene_of[out_to_src[j - 1]] : -1;
            out_boundary[j] = (sc_cur != sc_prev) ? 1 : 0;
        }
    }

    void save_cache(uint64_t params_hash) const {
        if (cache_path.empty()) return;
        ensure_dir_for_file(cache_path.c_str());
        FILE* f = fopen(cache_path.c_str(), "wb");
        if (!f) return;
        uint32_t magic = SDEC_MAGIC, ver = SDEC_VERSION;
        uint32_t n = (uint32_t)N_in, t = (uint32_t)N_out;
        uint32_t fn = (uint32_t)fps_num_out, fd = (uint32_t)fps_den_out;
        fwrite(&magic,sizeof(magic),1,f);
        fwrite(&ver,sizeof(ver),1,f);
        fwrite(&n,sizeof(n),1,f);
        fwrite(&t,sizeof(t),1,f);
        fwrite(&fn,sizeof(fn),1,f);
        fwrite(&fd,sizeof(fd),1,f);
        fwrite(&params_hash,sizeof(params_hash),1,f);
        fwrite(keep_mask.data(), 1, keep_mask.size(), f);
        fwrite(out_to_src.data(), sizeof(int32_t), out_to_src.size(), f);
        // out_boundary: decimated-timeline scene cuts (size N_out)
        if ((int)out_boundary.size() == N_out)
            fwrite(out_boundary.data(), 1, N_out, f);
        else {
            std::vector<uint8_t> zero(N_out, 0);
            fwrite(zero.data(), 1, N_out, f);
        }
        // v2: diagnostic arrays for mark_only reuse
        if (!is_dup.empty()) {
            fwrite(is_dup.data(), 1, N_in, f);
            fwrite(scene_of.data(), sizeof(int32_t), N_in, f);
            fwrite(is_fade_src.data(), 1, N_in, f);
            fwrite(d_unimodal.data(), 1, N_in, f);
            fwrite(d_alternation.data(), sizeof(double), N_in, f);
            fwrite(d_local_alt.data(), sizeof(double), N_in, f);
            fwrite(d_sceneLen.data(), sizeof(int32_t), N_in, f);
            fwrite(d_sceneOrig.data(), sizeof(int32_t), N_in, f);
            fwrite(d_sceneDup.data(), sizeof(int32_t), N_in, f);
            fwrite(d_sceneStart.data(), sizeof(int32_t), N_in, f);
            fwrite(d_ssim.data(), sizeof(int), N_in, f);
            fwrite(d_residual.data(), sizeof(double), N_in, f);
            fwrite(d_ratio.data(), sizeof(double), N_in, f);
            fwrite(d_otsu.data(), sizeof(double), N_in, f);
        }
        fclose(f);
    }

public:
    SmartDecimateEOD(PClip _child, PClip _sc, double _fps_out,
                     int _th_ssim_dup, double _res_ceil, double _res_mult,
                     double _th_ratio, int _reg_scale, bool _full_range,
                     int _thumb_w, int _thumb_h,
                     double _bypass_alt, int _bypass_min_len,
                     int _bypass_window,
                     const char* _mode,
                     int _fade_ratio, int _fade_drift,
                     bool _protect_fade, bool _allow_cut,
                     const char* _cache, const char* _log, bool _debug,
                     bool _analyze_only, const char* _analyze_log,
                     const char* _scuts_out,
                     const char* _scenes_file,
                     IScriptEnvironment* env)
        : GenericVideoFilter(_child), sc_clip(_sc),
          fps_out(_fps_out),
          q_low_pct(80.0), th_ssim_dup(_th_ssim_dup),
          residual_ceiling(_res_ceil), residual_mult(_res_mult),
          th_ratio(_th_ratio), reg_scale(_reg_scale), full_range(_full_range),
          thumb_w(_thumb_w), thumb_h(_thumb_h),
          bypass_alt(_bypass_alt), bypass_min_len(_bypass_min_len),
          bypass_window(_bypass_window),
          mode(_mode ? _mode : "cfr"),
          fade_ratio(_fade_ratio), fade_mean_drift(_fade_drift),
          protect_fade(_protect_fade), allow_original_cut(_allow_cut),
          debug(_debug), analyze_only(_analyze_only),
          cache_path(_cache ? _cache : ""), log_path(_log ? _log : ""),
          analyze_log_path(_analyze_log ? _analyze_log : ""),
          scuts_out_path(_scuts_out ? _scuts_out : ""),
          scenes_file_path(_scenes_file ? _scenes_file : "")
    {
        if (!vi.IsYV12())
            env->ThrowError("SmartDecimateEOD: YV12 input required");
        if (fps_out <= 0 || fps_out > 60)
            env->ThrowError("SmartDecimateEOD: fps_out out of range");

        N_in = vi.num_frames;
        double fps_in = (double)vi.fps_numerator / (double)vi.fps_denominator;
        // Target frame count from source length ratio (keeps duration exact).
        N_out = (int)((double)N_in * fps_out / fps_in + 0.5);
        if (N_out < 1) N_out = 1;
        if (N_out > N_in) N_out = N_in;

        // Represent fps_out as rational with denominator 1001 or 1 as best fits.
        // Simple approach: numerator = round(fps_out * 1000), denom = 1000.
        fps_num_out = (int)(fps_out * 1000.0 + 0.5);
        fps_den_out = 1000;

        // Params hash. IMPORTANT: hash the fields explicitly, never the raw
        // struct memory — a mixed double/int/uint8_t struct has padding bytes
        // that are uninitialized, so hashing sizeof(struct) gives different
        // results for identical field values (nondeterministic cache misses).
        int mode_id = (mode == "content") ? 1 : (mode == "mark_only" ? 2 : 0);
        uint64_t params_hash = params_hash_for(mode_id);

        if (analyze_only) {
            // Always re-run Pass 1.
            preanalyze(env);
            N_out = N_in;
            out_to_src.clear();
            out_to_src.reserve(N_in);
            for (int i = 0; i < N_in; ++i) out_to_src.push_back(i);
        } else if (mode == "mark_only") {
            // Try loading diagnostic arrays from cfr cache (mode_id=0).
            // This avoids a full preanalyze — mark_only becomes instant.
            uint64_t cfr_hash = params_hash_for(0);
            if (try_load_cache(cfr_hash) && !is_dup.empty() && !d_alternation.empty()) {
                // Diagnostics loaded from cfr cache — set up mark_only output.
                N_out = N_in;
                fps_num_out = vi.fps_numerator;
                fps_den_out = vi.fps_denominator;
                keep_mask.assign(N_in, 1);
                out_to_src.clear();
                out_to_src.reserve(N_in);
                for (int i = 0; i < N_in; ++i) out_to_src.push_back(i);
            } else {
                // No cfr cache — fall back to full preanalyze.
                preanalyze(env);
            }
        } else if (try_load_cache(params_hash)) {
            // cfr mode, cache hit — all arrays loaded including diagnostics.
            fprintf(stderr, "[SmartDecimateEOD] CACHE HIT: loaded from %s (N_in=%d N_out=%d)\n",
                    cache_path.c_str(), N_in, N_out); fflush(stderr);
        } else {
            fprintf(stderr, "[SmartDecimateEOD] CACHE MISS: recomputing Pass 1. "
                    "cache='%s' N_in=%d hash=%llu mode=%s reg_scale=%d fps_out=%.3f\n",
                    cache_path.c_str(), N_in, (unsigned long long)params_hash,
                    mode.c_str(), reg_scale, fps_out); fflush(stderr);
            preanalyze(env);
            save_cache(params_hash);
        }

        // Advertise new clip length & fps.
        vi.num_frames = N_out;
        vi.fps_numerator   = fps_num_out;
        vi.fps_denominator = fps_den_out;
    }

    // ─── two-pass preanalysis ──────────────────────────────────────────────

    void preanalyze(IScriptEnvironment* env) {
        // Diagnostic buffers.
        d_ssim.assign(N_in, 0);
        d_residual.assign(N_in, 0.0);
        d_ratio.assign(N_in, 1.0);
        d_otsu.assign(N_in, 0.0);
        d_unimodal.assign(N_in, 0);
        d_sceneLen.assign(N_in, 0);
        d_sceneOrig.assign(N_in, 0);
        d_sceneDup.assign(N_in, 0);
        d_sceneStart.assign(N_in, 0);
        d_alternation.assign(N_in, 0.0);

        // ── Read scene boundaries + meanY from sc_clip. In READ mode
        //    (scenes_file) this is zero pixel decode of child — only
        //    frame-prop traffic on sc_clip. Cheap.
        std::vector<int>    ssim_val(N_in, 0);
        std::vector<double> resid(N_in, 0.0);
        std::vector<double> ratio_v(N_in, 1.0);
        std::vector<int>    meanY(N_in, 0);
        std::vector<int>    boundaries;
        boundaries.push_back(0);

        bool has_meanY = false;
        if (!scenes_file_path.empty()) {
            // ── Direct read of scenes.txt (SCDetectEOD format): one integer
            //    per line, optional "# ..." comment. Zero frame-prop traffic,
            //    no sc_clip needed. meanY is not in the file, but it is computed
            //    per-frame from thumbnails in Pass 1A below, so fade detection
            //    still works in this mode.
            FILE* f = fopen_utf8(scenes_file_path.c_str(), L"rb");
            if (!f)
                env->ThrowError("SmartDecimateEOD: cannot open scenes_file '%s'",
                                scenes_file_path.c_str());
            char line[512];
            int last = 0;
            while (fgets(line, sizeof(line), f)) {
                char* p = line;
                while (*p == ' ' || *p == '\t') ++p;
                if (*p == '#' || *p == '\r' || *p == '\n' || *p == '\0') continue;
                char* end = nullptr;
                long v = strtol(p, &end, 10);
                if (end == p) continue;            // no digits on this line
                int b = (int)v;
                if (b <= 0 || b >= N_in) continue; // 0 and out-of-range ignored
                if (b <= last) continue;           // keep strictly ascending
                boundaries.push_back(b);
                last = b;
            }
            fclose(f);
            // meanY stays all -1 (already initialized); has_meanY=false.
        } else {
            // ── Legacy: read scene boundaries + meanY from sc_clip frame-props.
            //    In READ mode (SCDetectEOD reading a file) this is zero pixel
            //    decode of child — only frame-prop traffic on sc_clip.
            PVideoFrame sf = sc_clip->GetFrame(0, env);
            auto props = env->getFramePropsRO(sf);
            int err = 0;
            meanY[0] = (int)env->propGetInt(props, "_scd_meanY", 0, &err);
            if (err == 0) has_meanY = true; else meanY[0] = -1;
            for (int i = 1; i < N_in; ++i) {
                PVideoFrame sf2 = sc_clip->GetFrame(i, env);
                auto props2 = env->getFramePropsRO(sf2);
                int e1 = 0;
                int b = (int)env->propGetInt(props2, "_scd_boundary", 0, &e1);
                int e2 = 0;
                meanY[i] = (int)env->propGetInt(props2, "_scd_meanY", 0, &e2);
                if (e2 != 0) meanY[i] = -1;
                if (b == 1) boundaries.push_back(i);
            }
        }
        boundaries.push_back(N_in);

        int S = (int)boundaries.size() - 1;
        scene_of.assign(N_in, 0);
        is_dup.assign(N_in, 0);
        is_fade_src.assign(N_in, 0);
        std::vector<uint8_t> is_fade(S, 0);

        int tn = thumb_w * thumb_h;
        std::vector<uint8_t> thumb_prev(tn), thumb_cur(tn);

        // ── PASS 1A: SSIM sweep over ALL frames (cheap thumbnails), one
        //    decode per frame.
        int bw = rs_w(), bh = rs_h();
        size_t bsz = (size_t)bw * bh;
        {
            PVideoFrame f0 = child->GetFrame(0, env);
            make_thumb(f0, thumb_prev.data(), full_range);
            if (!has_meanY) meanY[0] = thumb_mean(thumb_prev.data(), tn);
        }
        for (int i = 1; i < N_in; ++i) {
            PVideoFrame cf = child->GetFrame(i, env);
            make_thumb(cf, thumb_cur.data(), full_range);
            if (!has_meanY) meanY[i] = thumb_mean(thumb_cur.data(), tn);
            ssim_val[i] = compute_ssim(thumb_prev.data(), thumb_cur.data(), tn);
            d_ssim[i] = ssim_val[i];
            std::memcpy(thumb_prev.data(), thumb_cur.data(), tn);
        }
        // scenes_file mode: meanY is now filled per-frame from thumbnails, so
        // fade detection can run. (sc_clip mode already set has_meanY above.)
        if (!scenes_file_path.empty()) has_meanY = true;

        // ── PASS 1B: per-scene. Fade + SSIM guard are cheap (use ssim_val).
        //    Registration runs ONLY for scenes that pass the guard (deferred);
        //    guard-rejected scenes (static / on-ones / fast motion / fades)
        //    skip it. This deferral is what keeps Pass 1 fast on mixed material.
        for (int s = 0; s < S; ++s) {
            int a = boundaries[s];
            int b = boundaries[s + 1];
            int len = b - a;
            for (int i = a; i < b; ++i) {
                scene_of[i] = s;
                d_sceneLen[i]   = len;
                d_sceneStart[i] = a;
            }
            if (len < 2) continue;

            // Fade detection: monotonic meanY drift with sufficient magnitude.
            // meanY is per-frame in both modes: from sc_clip props, or computed
            // from thumbnails in scenes_file mode (Pass 1A above).
            if (protect_fade && has_meanY) {
                int drift = std::abs(meanY[b - 1] - meanY[a]);
                if (drift >= fade_mean_drift) {
                    int sign = (meanY[b - 1] > meanY[a]) ? 1 : -1;
                    int violations = 0;
                    for (int i = a + 1; i < b; ++i) {
                        int d = (meanY[i] - meanY[i - 1]) * sign;
                        if (d < -1) violations++;
                    }
                    if (violations * 100 < (len - 1) * (100 - fade_ratio)) {
                        is_fade[s] = 1;
                        for (int i = a; i < b; ++i) is_fade_src[i] = 1;
                        continue; // all-ORIG
                    }
                }
            }

            // SSIM guard: if q80(ssim) < th_ssim_dup, scene has no real dups.
            std::vector<int> ss;
            ss.reserve(len - 1);
            for (int i = a + 1; i < b; ++i) ss.push_back(ssim_val[i]);
            std::vector<int> ss_sorted = ss;
            std::sort(ss_sorted.begin(), ss_sorted.end());
            int q80_idx = (int)(ss_sorted.size() * 0.80);
            if (q80_idx >= (int)ss_sorted.size()) q80_idx = (int)ss_sorted.size() - 1;
            int q80 = ss_sorted[q80_idx];
            if (q80 < th_ssim_dup) {
                for (int i = a; i < b; ++i) {
                    d_otsu[i] = (double)th_ssim_dup;
                    d_unimodal[i] = 1;
                }
                continue; // all ORIG — SSIM says no dups, skip registration
            }

            // Deferred registration: only guard-passing scenes reach here.
            // Rolling 2-frame downscaled buffer; grid search on the small buffer.
            std::vector<uint8_t> buf_prev(bsz), buf_cur(bsz);
            {
                PVideoFrame pf = child->GetFrame(a, env);
                downscale_into(pf, buf_prev.data());
            }
            for (int i = a + 1; i < b; ++i) {
                PVideoFrame cf = child->GetFrame(i, env);
                downscale_into(cf, buf_cur.data());
                double r, rt;
                compute_pair_buf(buf_prev.data(), buf_cur.data(), r, rt);
                resid[i] = r; ratio_v[i] = rt;
                d_residual[i] = r; d_ratio[i] = rt;
                buf_prev.swap(buf_cur);
            }

            // Residual percentile classification.
            std::vector<double> sd;
            sd.reserve(len - 1);
            for (int i = a + 1; i < b; ++i) sd.push_back(resid[i]);
            std::vector<double> sd_sorted = sd;
            std::sort(sd_sorted.begin(), sd_sorted.end());
            int q_idx = (int)(sd_sorted.size() * 0.20);
            if (q_idx < 0) q_idx = 0;
            if (q_idx >= (int)sd_sorted.size()) q_idx = (int)sd_sorted.size() - 1;
            double q_low = sd_sorted[q_idx];
            if (q_low > residual_ceiling) {
                for (int i = a; i < b; ++i) {
                    d_otsu[i] = q_low;
                    d_unimodal[i] = 1;
                }
                continue; // all ORIG — residual too high for dups
            }
            double thr_res = q_low * residual_mult;
            if (thr_res < 1.0) thr_res = 1.0;
            for (int i = a; i < b; ++i) d_otsu[i] = thr_res;
            for (int i = a + 1; i < b; ++i) {
                bool sig1 = resid[i] < thr_res;
                bool sig2 = ratio_v[i] < th_ratio;
                if (sig1 && sig2) is_dup[i] = 1;
            }
        }

        // Per-scene counts for diagnostics (broadcast to all frames in scene).
        for (int s = 0; s < S; ++s) {
            int a = boundaries[s], b = boundaries[s + 1];
            int no = 0, nd = 0;
            for (int i = a; i < b; ++i) {
                if (is_dup[i]) nd++; else no++;
            }
            // Alternation rate: how often classification flips between
            // consecutive frames. 1.0 = perfect O-D-O-D, 0.0 = monotonic.
            int trans = 0;
            for (int i = a + 1; i < b; ++i) {
                if (is_dup[i] != is_dup[i - 1]) trans++;
            }
            double alt = (b - a > 1) ? (double)trans / (double)(b - a - 1) : 0.0;
            for (int i = a; i < b; ++i) {
                d_sceneOrig[i] = no;
                d_sceneDup[i] = nd;
                d_alternation[i] = alt;
            }
        }

        // Per-frame local alternation (sliding window within scene bounds).
        // This enables sub-scene bypass: parts of a scene with on-twos
        // pattern go through RIFE, parts with irregular pattern go from source.
        d_local_alt.assign(N_in, 0.0);
        for (int i = 0; i < N_in; ++i) {
            // Find scene bounds for this frame
            int s = scene_of[i];
            int sa = d_sceneStart[i];
            int sb = sa + d_sceneLen[i];
            // Window within scene
            int wa = std::max(sa + 1, i - bypass_window);
            int wb = std::min(sb, i + bypass_window + 1);
            if (wb - wa < 2) { d_local_alt[i] = d_alternation[i]; continue; }
            int trans = 0;
            for (int k = wa; k < wb; ++k) {
                if (k > sa && is_dup[k] != is_dup[k - 1]) trans++;
            }
            d_local_alt[i] = (double)trans / (double)(wb - wa);
        }

        // CSV analysis log (optional, independent of log_path).
        if (!analyze_log_path.empty()) {
                ensure_dir_for_file(analyze_log_path.c_str());
            FILE* f = fopen(analyze_log_path.c_str(), "w");
            if (f) {
                fprintf(f, "src,scene,ssim,residual,ratio,thr,is_dup,is_fade,unimodal,"
                           "scene_len,scene_orig,scene_dup,scene_start,alternation,local_alt\n");
                for (int i = 0; i < N_in; ++i) {
                    fprintf(f, "%d,%d,%d,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f\n",
                        i, scene_of[i], d_ssim[i], d_residual[i], d_ratio[i], d_otsu[i],
                        (int)is_dup[i], (int)is_fade_src[i], (int)d_unimodal[i],
                        d_sceneLen[i], d_sceneOrig[i], d_sceneDup[i],
                        d_sceneStart[i], d_alternation[i], d_local_alt[i]);
                }
                fclose(f);
            }
        }

        // PASS 2: global budget (skipped in analyze_only mode).
        if (analyze_only) return;

        // ─── MARK_ONLY mode: pass source through, add _sdec_bypass ────
        // Output: same length, same fps as source. Frame property
        // _sdec_bypass=1 marks frames belonging to scenes where
        // classification says "this scene should bypass RIFE processing".
        if (mode == "mark_only") {
            keep_mask.assign(N_in, 1);
            N_out = N_in;
            fps_num_out = vi.fps_numerator;
            fps_den_out = vi.fps_denominator;
            out_to_src.clear();
            out_to_src.reserve(N_in);
            for (int i = 0; i < N_in; ++i) out_to_src.push_back(i);
            return;
        }

        // ─── CFR per-scene mode: each scene keeps round(len*fps_out/fps_in)
        // frames, picking ORIGs first (by priority), then DUPs.
        // If scene must shed ORIGs, shed by smallest score (least motion).
        // Global length is deterministic: sum of round(len*ratio).
        if (mode == "cfr" || mode == "cfr_per_scene") {
            double fps_in = (double)vi.fps_numerator / (double)vi.fps_denominator;
            double ratio = fps_out / fps_in;
            keep_mask.assign(N_in, 0);

            // Walk scenes from scene_of[]
            int i = 0;
            int drift_sign = 1; // alternate round-up/round-down for drift zeroing
            while (i < N_in) {
                int s = scene_of[i];
                int a = i;
                while (i < N_in && scene_of[i] == s) ++i;
                int b = i;
                int len = b - a;
                double target_f = (double)len * ratio;
                int target = (int)std::floor(target_f);
                // Distribute fractional part alternately to keep global drift near 0.
                double frac = target_f - (double)target;
                if (frac >= 0.5 && drift_sign > 0) { target++; drift_sign = -1; }
                else if (frac > 0.0 && frac < 0.5 && drift_sign < 0) { target++; drift_sign = 1; }
                else if (frac >= 0.5) drift_sign = 1;
                if (target < 1) target = 1;
                if (target > len) target = len;

                // Count scene ORIGs.
                int no = 0;
                for (int k = a; k < b; ++k) if (!is_dup[k]) no++;

                if (no <= target) {
                    // All ORIGs fit; fill remaining slots with DUPs having highest
                    // score (most distinct — least redundant).
                    for (int k = a; k < b; ++k) if (!is_dup[k]) keep_mask[k] = 1;
                    int need = target - no;
                    if (need > 0) {
                        std::vector<std::pair<double,int>> dups;
                        for (int k = a; k < b; ++k) {
                            if (is_dup[k]) {
                                double da = (k > a)     ? resid[k]     : 0.0;
                                double db = (k + 1 < b) ? resid[k + 1] : 0.0;
                                dups.push_back({std::max(da, db), k});
                            }
                        }
                        // largest first
                        std::sort(dups.begin(), dups.end(),
                                  [](const std::pair<double,int>& x,
                                     const std::pair<double,int>& y){ return x.first > y.first; });
                        for (int t = 0; t < need && t < (int)dups.size(); ++t)
                            keep_mask[dups[t].second] = 1;
                    }
                } else {
                    // Must shed ORIGs: keep those with highest score (most motion).
                    std::vector<std::pair<double,int>> origs;
                    for (int k = a; k < b; ++k) {
                        if (!is_dup[k]) {
                            double da = (k > a)     ? resid[k]     : 0.0;
                            double db = (k + 1 < b) ? resid[k + 1] : 0.0;
                            origs.push_back({std::max(da, db), k});
                        }
                    }
                    std::sort(origs.begin(), origs.end(),
                              [](const std::pair<double,int>& x,
                                 const std::pair<double,int>& y){ return x.first > y.first; });
                    for (int t = 0; t < target && t < (int)origs.size(); ++t)
                        keep_mask[origs[t].second] = 1;
                }
            }

            // Build out_to_src
            int kept = 0;
            out_to_src.clear();
            for (int k = 0; k < N_in; ++k) if (keep_mask[k]) { out_to_src.push_back(k); kept++; }
            N_out = kept;
            fps_num_out = (int)(fps_out * 1000.0 + 0.5);
            fps_den_out = 1000;

            // Materialize decimated-timeline scene boundaries from the verified
            // scuts.txt scenes (via scene_of) remapped through out_to_src.
            // Output frame j is a boundary if its source scene differs from the
            // previous kept frame's source scene. This is the exact remap of the
            // hand-verified cuts onto the decimated clip — no re-detection.
            build_out_boundary();

            // Write decimated-timeline scene cuts as a scenes.txt for
            // SCSignalBorder (or any scenes_file consumer) on the decimated
            // clip. These are the hand-verified scuts.txt cuts remapped — no
            // re-detection on the decimated picture.
            if (!scuts_out_path.empty()) {
                ensure_dir_for_file(scuts_out_path.c_str());
                FILE* sf = fopen(scuts_out_path.c_str(), "w");
                if (sf) {
                    for (int j = 1; j < N_out; ++j)
                        if (out_boundary[j]) fprintf(sf, "%d\n", j);
                    fclose(sf);
                }
            }

            if (!log_path.empty()) {
                    ensure_dir_for_file(log_path.c_str());
                FILE* f = fopen(log_path.c_str(), "w");
                if (f) {
                    fprintf(f, "# SmartDecimateEOD (per-scene CFR)\n");
                    fprintf(f, "# N_in=%d N_out=%d fps_in=%.4f fps_out=%.4f\n",
                            N_in, N_out, fps_in, fps_out);
                    fprintf(f, "# scene start end len n_orig n_dup n_kept is_bypass\n");
                    int ss = 0, j = 0;
                    while (j < N_in) {
                        int s2 = scene_of[j];
                        int aa = j;
                        while (j < N_in && scene_of[j] == s2) ++j;
                        int bb = j;
                        int no = 0, nd = 0, nk = 0;
                        for (int k = aa; k < bb; ++k) {
                            if (is_dup[k]) nd++; else no++;
                            if (keep_mask[k]) nk++;
                        }
                        bool bypass = true;
                        if ((bb - aa) >= bypass_min_len && d_alternation[aa] >= bypass_alt) bypass = false;
                        if (is_fade_src[aa] != 0) bypass = true;
                        fprintf(f, "%d %d %d %d %d %d %d %d\n",
                                ss++, aa, bb, bb - aa, no, nd, nk, bypass ? 1 : 0);
                    }
                    fclose(f);
                }
            }
            return;
        }

        // ─── Legacy content mode (broken, kept for compat) ────────────
        if (mode == "content") {
            keep_mask.assign(N_in, 0);
            int kept = 0;
            for (int i = 0; i < N_in; ++i) {
                if (!is_dup[i]) { keep_mask[i] = 1; kept++; }
            }
            N_out = kept;
            // Exact rational: fps_out = fps_in_num * N_out / (fps_in_den * N_in)
            // Use int64 then GCD-reduce to avoid overflow in vi.fps fields.
            int64_t num = (int64_t)vi.fps_numerator * (int64_t)N_out;
            int64_t den = (int64_t)vi.fps_denominator * (int64_t)N_in;
            int64_t g = num; int64_t b = den;
            while (b) { int64_t t = b; b = g % b; g = t; }
            if (g > 0) { num /= g; den /= g; }
            // Clamp to int32 (x264 and AviSynth+ store as int).
            while (num > 0x7fffffffLL || den > 0x7fffffffLL) { num >>= 1; den >>= 1; }
            fps_num_out = (int)num;
            fps_den_out = (int)den;

            out_to_src.clear();
            out_to_src.reserve(N_out);
            for (int i = 0; i < N_in; ++i) if (keep_mask[i]) out_to_src.push_back(i);

            // Log per-scene for diagnostics.
            if (!log_path.empty()) {
                    ensure_dir_for_file(log_path.c_str());
                FILE* f = fopen(log_path.c_str(), "w");
                if (f) {
                    double fps_actual = (double)fps_num_out / (double)fps_den_out;
                    fprintf(f, "# SmartDecimateEOD (content mode)\n");
                    fprintf(f, "# N_in=%d N_out=%d fps_in=%.4f fps_out=%.4f (%d/%d)\n",
                            N_in, N_out,
                            (double)vi.fps_numerator / (double)vi.fps_denominator,
                            fps_actual, fps_num_out, fps_den_out);
                    fprintf(f, "# scene start end len n_orig n_dup n_kept is_fade\n");
                    // reconstruct scenes from scene_of
                    int s = 0;
                    for (int i = 0; i < N_in; ) {
                        int j = i;
                        while (j < N_in && scene_of[j] == scene_of[i]) ++j;
                        int no = 0, nd = 0, nk = 0;
                        for (int k = i; k < j; ++k) {
                            if (is_dup[k]) nd++; else no++;
                            if (keep_mask[k]) nk++;
                        }
                        fprintf(f, "%d %d %d %d %d %d %d %d\n",
                                s++, i, j, j - i, no, nd, nk, (int)is_fade_src[i]);
                        i = j;
                    }
                    fclose(f);
                }
            }
            return;
        }

        // ─── CFR mode (original budgeted logic) ───────────────────────
        int n_orig = 0;
        for (int i = 0; i < N_in; ++i) if (!is_dup[i]) n_orig++;

        if (n_orig > N_out && !allow_original_cut) {
            double fps_in = (double)vi.fps_numerator / (double)vi.fps_denominator;
            double rec = (double)n_orig * fps_in / (double)N_in;
            char msg[256];
            _snprintf_s(msg, sizeof(msg), _TRUNCATE,
                "SmartDecimateEOD: source has %d originals but budget is %d. "
                "Recommend fps_out >= %.3f, or pass allow_original_cut=true.",
                n_orig, N_out, rec);
            env->ThrowError("%s", msg);
        }

        // Build keep_mask.
        keep_mask.assign(N_in, 0);
        // First: mark all ORIG as keep.
        for (int i = 0; i < N_in; ++i) if (!is_dup[i]) keep_mask[i] = 1;

        int kept = n_orig;
        int budget_dup = N_out - n_orig;

        if (budget_dup > 0) {
            // Distribute dups: walk the film; after each ORIG take the
            // immediately-following DUP(s) up to one per ORIG per pass.
            // Multi-pass until budget exhausted or no more candidates.
            // This preserves on-twos rhythm (O D O D ...) in static scenes
            // and naturally skips on-ones scenes (no dups to take).
            bool progress = true;
            while (budget_dup > 0 && progress) {
                progress = false;
                for (int i = 0; i < N_in - 1 && budget_dup > 0; ++i) {
                    if (keep_mask[i] && is_dup[i + 1] && !keep_mask[i + 1]) {
                        // Don't cross scene boundary
                        if (scene_of[i] != scene_of[i + 1]) continue;
                        keep_mask[i + 1] = 1;
                        budget_dup--;
                        kept++;
                        progress = true;
                        // Skip the one we just took so next iteration of inner
                        // loop looks for the next ORIG's dup, not a chain.
                        i++;
                    }
                }
            }
        } else if (budget_dup < 0 && allow_original_cut) {
            // Soft-budget mode: drop originals with lowest score.
            // Score: min(diff[i-1,i], diff[i,i+1]); smaller = more dispensable.
            std::vector<std::pair<double,int>> scored;
            for (int i = 0; i < N_in; ++i) {
                if (!keep_mask[i]) continue;
                double a = (i > 0)        ? resid[i]     : 1e30;
                double b = (i + 1 < N_in) ? resid[i + 1] : 1e30;
                if (scene_of[i - (i>0?1:0)] != scene_of[i]) a = 1e30;
                if (i + 1 < N_in && scene_of[i] != scene_of[i+1]) b = 1e30;
                double sc = std::min(a, b);
                scored.push_back({sc, i});
            }
            std::sort(scored.begin(), scored.end());
            int to_drop = -budget_dup;
            for (int k = 0; k < to_drop && k < (int)scored.size(); ++k) {
                keep_mask[scored[k].second] = 0;
                kept--;
            }
        }

        // If we ended up off-target due to scene-boundary refusals, adjust:
        // drop trailing dups (lowest-diff kept frames that are dups) or, if
        // still short, take any remaining dup regardless of ORIG adjacency.
        while (kept > N_out) {
            // Drop the kept DUP with smallest diff (most redundant).
            int victim = -1; double best = 1e30;
            for (int i = 0; i < N_in; ++i) {
                if (keep_mask[i] && is_dup[i] && !is_fade_src[i]) {
                    if (resid[i] < best) { best = resid[i]; victim = i; }
                }
            }
            if (victim < 0) break;
            keep_mask[victim] = 0;
            kept--;
        }
        while (kept < N_out) {
            int victim = -1; double best = 1e30;
            for (int i = 0; i < N_in; ++i) {
                if (!keep_mask[i] && is_dup[i]) {
                    if (resid[i] < best) { best = resid[i]; victim = i; }
                }
            }
            if (victim < 0) break;
            keep_mask[victim] = 1;
            kept++;
        }
        N_out = kept; // final

        // Build out_to_src.
        out_to_src.clear();
        out_to_src.reserve(N_out);
        for (int i = 0; i < N_in; ++i) if (keep_mask[i]) out_to_src.push_back(i);

        // Optional log
        if (!log_path.empty()) {
                    ensure_dir_for_file(log_path.c_str());
            FILE* f = fopen(log_path.c_str(), "w");
            if (f) {
                fprintf(f, "# SmartDecimateEOD log\n");
                fprintf(f, "# N_in=%d N_out=%d n_orig=%d\n", N_in, N_out, n_orig);
                fprintf(f, "# scene start end len n_orig n_dup n_kept is_fade\n");
                for (int s = 0; s < S; ++s) {
                    int a = boundaries[s], b = boundaries[s+1];
                    int no=0,nd=0,nk=0;
                    for (int i = a; i < b; ++i) {
                        if (is_dup[i]) nd++; else no++;
                        if (keep_mask[i]) nk++;
                    }
                    fprintf(f, "%d %d %d %d %d %d %d %d\n",
                            s, a, b, b-a, no, nd, nk, (int)is_fade[s]);
                }
                fclose(f);
            }
        }
    }

    // ─── GetFrame: pure remap ──────────────────────────────────────────────

    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override {
        if (n < 0) n = 0;
        if (n >= N_out) n = N_out - 1;
        int src_n = out_to_src[n];
        PVideoFrame f = child->GetFrame(src_n, env);

        // Emit _scd_boundary on the decimated timeline from the materialized
        // out_boundary array (verified scuts.txt cuts remapped through
        // out_to_src in Pass 1). No second SCDetectEOD, no re-detection.
        if ((mode != "mark_only") && (int)out_boundary.size() == N_out) {
            env->MakeWritable(&f);
            auto props = env->getFramePropsRW(f);
            env->propSetInt(props, "_scd_boundary", out_boundary[n], 0);
        }

        if (debug || analyze_only || mode == "mark_only") {
            env->MakeWritable(&f);
            auto props = env->getFramePropsRW(f);
            env->propSetInt(props, "_sdec_srcFrame", src_n, 0);
            if (!scene_of.empty())
                env->propSetInt(props, "_sdec_sceneId", scene_of[src_n], 0);
            if (!is_dup.empty())
                env->propSetInt(props, "_sdec_isDup", is_dup[src_n], 0);
            if (!is_fade_src.empty())
                env->propSetInt(props, "_sdec_isFade", is_fade_src[src_n], 0);
            if (!d_ssim.empty()) {
                env->propSetInt(props, "_sdec_ssim",       d_ssim[src_n], 0);
                env->propSetFloat(props, "_sdec_residual", d_residual[src_n], 0);
                env->propSetFloat(props, "_sdec_ratio",    d_ratio[src_n], 0);
                env->propSetFloat(props, "_sdec_otsu",     d_otsu[src_n], 0);
                env->propSetInt(props, "_sdec_unimodal",   d_unimodal[src_n], 0);
                env->propSetInt(props, "_sdec_sceneLen",   d_sceneLen[src_n], 0);
                env->propSetInt(props, "_sdec_sceneOrig",  d_sceneOrig[src_n], 0);
                env->propSetInt(props, "_sdec_sceneDup",   d_sceneDup[src_n], 0);
                env->propSetInt(props, "_sdec_sceneStart", d_sceneStart[src_n], 0);
                if (!d_alternation.empty())
                    env->propSetFloat(props, "_sdec_alternation", d_alternation[src_n], 0);
                if (!d_local_alt.empty())
                    env->propSetFloat(props, "_sdec_local_alt", d_local_alt[src_n], 0);
            }
            // Bypass flag: per-scene decision using scene-wide alternation.
            // Sub-scene switching creates visible seams at bypass boundaries
            // (source vs RIFE frames don't match), so bypass is scene-level.
            // d_local_alt is computed and exposed as diagnostic prop only.
            int bypass = 1;  // default: bypass (source)
            if (!d_alternation.empty() && d_alternation[src_n] >= bypass_alt &&
                d_sceneLen[src_n] >= bypass_min_len) {
                bypass = 0;  // scene-wide on-twos pattern → RIFE
            }
            // fade flag overrides — short fades may have insufficient frames
            if (!is_fade_src.empty() && is_fade_src[src_n]) bypass = 1;
            env->propSetInt(props, "_sdec_bypass", bypass, 0);
        }
        return f;
    }

    int __stdcall SetCacheHints(int cachehints, int) override {
        return cachehints == CACHE_GET_MTMODE ? MT_SERIALIZED : 0;
    }

    static AVSValue __cdecl Create(AVSValue args, void*, IScriptEnvironment* env) {
        PClip sc = args[1].IsClip() ? args[1].AsClip() : nullptr;
        const char* scenes_file = args[25].AsString("");
        if (!sc && (!scenes_file || !*scenes_file))
            env->ThrowError("SmartDecimateEOD: need either sc clip (2nd arg) "
                            "or scenes_file=");
        return new SmartDecimateEOD(
            args[0].AsClip(),
            sc,
            args[2].AsFloat(12.5f),      // fps_out
            args[3].AsInt(990),          // th_ssim_dup (SSIM guard)
            args[4].AsFloat(5.0f),       // residual_ceiling
            args[5].AsFloat(2.5f),       // residual_mult
            args[6].AsFloat(10.0f),      // th_ratio
            args[7].AsInt(1),            // reg_scale (1=full, 2=half, 4=quarter)
            args[8].AsBool(false),       // full_range
            args[9].AsInt(160),          // thumb_w
            args[10].AsInt(120),         // thumb_h
            args[11].AsFloat(0.80f),     // bypass_alt
            args[12].AsInt(4),           // bypass_min_len
            args[13].AsInt(12),          // bypass_window
            args[14].AsString("cfr"),    // mode
            args[15].AsInt(60),          // fade_ratio
            args[16].AsInt(20),          // fade_mean_drift
            args[17].AsBool(true),       // protect_fade
            args[18].AsBool(false),      // allow_original_cut
            args[19].AsString(""),       // cache
            args[20].AsString(""),       // log
            args[21].AsBool(false),      // debug
            args[22].AsBool(false),      // analyze_only
            args[23].AsString(""),       // analyze_log
            args[24].AsString(""),       // scuts_out
            scenes_file,                 // scenes_file (direct boundary source)
            env);
    }
};

const AVS_Linkage* AVS_linkage = nullptr;

extern "C" __declspec(dllexport)
const char* __stdcall AvisynthPluginInit3(IScriptEnvironment* env, const AVS_Linkage* const vectors) {
    AVS_linkage = vectors;
    env->AddFunction("SmartDecimateEOD",
        "c[sc]c[fps_out]f[th_ssim_dup]i[residual_ceiling]f[residual_mult]f[th_ratio]f"
        "[reg_scale]i[full_range]b[thumb_w]i[thumb_h]i"
        "[bypass_alt]f[bypass_min_len]i[bypass_window]i[mode]s"
        "[fade_ratio]i[fade_mean_drift]i"
        "[protect_fade]b[allow_original_cut]b[cache]s[log]s[debug]b"
        "[analyze_only]b[analyze_log]s[scuts_out]s[scenes_file]s",
        SmartDecimateEOD::Create, nullptr);
    return "SmartDecimateEOD v6.0 — direct scenes_file read (sc optional, fade detection via thumbnail meanY)";
}
