# SmartDecimateEOD

Content-aware frame decimator for hand-drawn animation, for AviSynth+ (x64).

SmartDecimateEOD removes **only duplicate frames** from animation — never originals — so that frame interpolation (RIFE) can work on a clean set of unique drawings without interpolating across doubled frames. It is scene-aware: each scene is classified by its own motion pattern, and audio sync is preserved by a per-scene constant-frame-rate budget.

## The problem

Classical animation is drawn "on twos": the artist makes ~12 unique drawings per second, and each drawing is held for two frames to reach 24/25 fps. The resulting frame stream is irregular:

- **Static scenes** — long runs of duplicates: `O D D D D D D …`
- **Normal motion** — strict on-twos: `O D O D O D …`
- **Fast motion / pans** — drawn "on ones", every frame unique: `O O O O …`

Fixed-ratio decimators (srestore and similar) assume a uniform cadence. On animation they cut genuine unique frames in on-ones passages and leave duplicates in static ones, which produces judder and breaks any subsequent interpolation. SmartDecimateEOD instead measures each scene and decides per frame whether it is an original or a held duplicate.

## How detection works

For every adjacent frame pair the plugin computes two independent signals and combines them with a fast guard:

1. **SSIM guard (thumbnail).** Structural similarity on a small box-averaged thumbnail. SSIM is excellent at telling "these two frames belong to different content" (a real cut, a pan, an on-ones scene) but it saturates in the 990–1000 range and **cannot** by itself separate a held duplicate from a slightly redrawn original. It is therefore used only as a *guard*: if the 80th-percentile SSIM inside a scene is below `th_ssim_dup`, the scene has no duplicates at all (every frame differs structurally) and the expensive step below is skipped entirely.

2. **Registration + residual SAD + block ratio (full-res).** For scenes that pass the guard, the plugin performs a ±3 integer motion search at a coarse scale, then measures the registered full-resolution SAD residual and a per-block max/mean ratio. A held duplicate, after registration, has a near-zero residual and an evenly distributed (low-ratio) error; a redrawn original does not. Classification is the AND of both signals.

This two-stage design is what makes it both accurate and fast: registration — the heavy part — runs only on scenes that actually contain duplicates. Fades, zooms, on-ones and static-without-dup scenes are dismissed by the SSIM guard at thumbnail cost.

## Sync model

Output frame count per scene is fixed arithmetically: each scene keeps `round(len · fps_out / fps_in)` frames. Originals have priority; duplicates fill any remaining budget by score; if a scene must shed frames it sheds the lowest-scoring ones. Because the budget is computed per scene, audio stays in sync across the whole film without a global drift term.

## Hybrid RIFE-bypass pipeline

Interpolating across a scene that was drawn on ones — or across a fade or a zoom — produces artifacts. SmartDecimateEOD classifies each scene by its **alternation rate** (how often the original/duplicate label flips between adjacent frames):

- `alternation ≥ bypass_alt` (clean on-twos) → the scene is decimated to `fps_out` and sent through RIFE ×2 back to source rate.
- otherwise → the scene passes through from the **source untouched**, skipping RIFE, preserving the original discrete poses.

The switch is performed per scene (never mid-scene, which would create a visible seam between source and interpolated frames). Two plugin instances cooperate:

- `mode="cfr_per_scene"` produces the decimated clip for the RIFE branch.
- `mode="mark_only"` writes a per-frame `_sdec_bypass` property; the companion `SmartDecimateEOD_Splice` switches each output frame between the source and the RIFE result based on that flag. Both clips share the same frame count and rate, so sync holds.

## Build

MSVC x64 only. MinGW DLLs are ABI-incompatible with MSVC AviSynth+ (vtable layout). From the *x64 Native Tools Command Prompt for VS 2022*:

```
cl /LD /EHsc /O2 /I"C:\Program Files (x86)\AviSynth+\FilterSDK\include" SmartDecimateEOD.cpp /link /OUT:SmartDecimateEOD.dll
```

Delete any older `SmartDecimateEOD.dll` from `plugins64` before installing the new one — a stale DLL silently shadows the new build.

## Input

YV12 only. The two clips passed in (the video and the scene-detector clip) must share the same frame numbering.

## Two-pass workflow

The detection pass is expensive on HD/UHD material but its result is cached, so it is run once. The cache stores the full classification, and both `cfr_per_scene` and `mark_only` read from the **same** cache file — `mark_only` recomputes its hash as if it were `cfr_per_scene`, so it loads instantly instead of re-analyzing.

**Pass 1 — preparation (writes the cache):**

```avs
src = LWLibavVideoSource("film.mkv").Crop(220, 0, -222, -0).RemoveGrain(20)
sc  = src.SCDetectEOD(scenes_file="film.scenes.txt")
SmartDecimateEOD(src, sc, fps_out=12.5, mode="cfr_per_scene", \
                 bypass_alt=0.7, reg_scale=2, \
                 cache="film.sdec.bin", log="film.sdec.log")
```

Run a full frame pass to populate the cache (the constructor does the work, so the clip must actually be processed end to end):

```
avs2pipemod64.exe -benchmark prep.avs
```

Verify `film.sdec.bin` exists and is non-trivial in size.

**Pass 2 — production (encode):**

```avs
src = LWLibavVideoSource("film.mkv").Crop(220, 0, -222, -0).RemoveGrain(20).Prefetch(2)
sc  = src.SCDetectEOD(scenes_file="film.scenes.txt")

dec = SmartDecimateEOD(src, sc, fps_out=12.5, mode="cfr_per_scene", \
                       bypass_alt=0.7, reg_scale=2, cache="film.sdec.bin")

# Re-detect scene boundaries on the decimated timeline for the RIFE signal.
# (scene numbers from the source timeline do NOT map onto the decimated clip.)
global sd_dec = dec.SCDetectEOD(mono=true)
SC_BORDER_H = 64
global decBlack = sd_dec.AddBorders(0, SC_BORDER_H, 0, 0, color=$000000)
global decWhite = sd_dec.AddBorders(0, SC_BORDER_H, 0, 0, color=$FFFFFF)
global SC_state = 0
dec_signaled = ScriptClip(decBlack, """
    b = sd_dec.propGetInt("_scd_boundary")
    global SC_state = (b == 1) ? (1 - SC_state) : SC_state
    SC_state == 1 ? decWhite : decBlack
""", local=false)

dec_rgb = z_ConvertFormat(dec_signaled, pixel_type="RGBPS", \
                          colorspace_op="709:709:709:l=>rgb:709:709:f")
rife_out = Rife(dec_rgb, gpu_thread=1, model=47, sc=true, sc_threshold=0.16, \
                factor_num=2, factor_den=1, uhd=true, skip=true).Prefetch(2)
rife_yuv = z_ConvertFormat(rife_out, pixel_type="YUV420P8", \
                           colorspace_op="rgb:709:709:f=>709:709:709:l")
rife_cropped = rife_yuv.Crop(0, SC_BORDER_H, -0, -0)

mask = SmartDecimateEOD(src, sc, mode="mark_only", \
                        bypass_alt=0.7, reg_scale=2, cache="film.sdec.bin")

SmartDecimateEOD_Splice(src, rife_cropped, mask)
```

**Rules for the cache to hit:**

- detection parameters must be identical between the prep and production calls (`fps_out`, `bypass_alt`, `reg_scale`, `th_ssim_dup`, `residual_ceiling`, `residual_mult`, `th_ratio`, …);
- the same `cache=` path in all calls;
- the clip fed to SmartDecimateEOD must be the one the cache was built on (same crop/denoise, no resize between prep and production).

## Calibration

```avs
src = LWLibavVideoSource("film.mkv").Crop(220, 0, -222, -0).RemoveGrain(20)
sc  = src.SCDetectEOD(scenes_file="film.scenes.txt")
ana = SmartDecimateEOD(src, sc, reg_scale=2, analyze_only=true, \
                       analyze_log="sdec_dbg.csv")
SmartDecimateEOD_Overlay(ana)
```

`SmartDecimateEOD_Overlay` (from `SmartDecimateEOD_debug.avsi`) draws, per frame: scene id and range, scene length / originals / duplicates, local alternation and the bypass decision, the SSIM / residual / ratio metrics with the active threshold, and a large ORIG/DUP verdict. The same metrics are written to the CSV for offline inspection.

## Parameters

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `clip` | clip | — | source video, YV12 (positional) |
| `sc_clip` | clip | — | scene-detector clip carrying `_scd_boundary` (positional) |
| `fps_out` | float | 12.5 | target frame rate after decimation |
| `th_ssim_dup` | int | 990 | SSIM guard: if a scene's 80th-percentile SSIM is below this, the scene is treated as all-original and registration is skipped |
| `residual_ceiling` | float | 5.0 | if a scene's low-quantile registered residual exceeds this, the scene is all-original (rejects dark/noisy scenes) |
| `residual_mult` | float | 2.5 | per-scene duplicate threshold = low-quantile residual × this |
| `th_ratio` | float | 10.0 | block max/mean ratio threshold (second classification signal) |
| `reg_scale` | int | 1 | internal downscale for the registration step: 1 = full, 2 = half, 4 = quarter. Cheaper and faster than an external resize, since no intermediate clip is allocated |
| `full_range` | bool | false | set true if source Y is already full-range 0–255 |
| `thumb_w` | int | 160 | SSIM-guard thumbnail width |
| `thumb_h` | int | 120 | SSIM-guard thumbnail height |
| `bypass_alt` | float | 0.80 | alternation threshold; scenes at or above go through RIFE, below pass through from source |
| `bypass_min_len` | int | 4 | minimum scene length to be eligible for RIFE |
| `bypass_window` | int | 12 | half-window for the per-frame local-alternation diagnostic |
| `mode` | string | "cfr" | "cfr_per_scene" (decimate), "mark_only" (write `_sdec_bypass`), "content" |
| `fade_ratio` | int | 60 | fade-detection sensitivity |
| `fade_mean_drift` | int | 20 | fade-detection mean-luma drift threshold |
| `protect_fade` | bool | true | never decimate inside a detected fade (auto-disabled when mean-luma data is unavailable, e.g. SCDetectEOD READ mode) |
| `allow_original_cut` | bool | false | permit shedding originals when a scene is over budget |
| `cache` | string | "" | path to the binary cache; shared between `cfr_per_scene` and `mark_only` |
| `log` | string | "" | path to a human-readable per-scene log |
| `debug` | bool | false | expose per-frame diagnostic frame properties |
| `analyze_only` | bool | false | pass-through at source length, populate diagnostics only (for calibration) |
| `analyze_log` | string | "" | path to a per-frame metrics CSV |

Parent directories for `cache`, `log` and `analyze_log` are created automatically.

## Frame properties

In `debug`, `analyze_only` or `mark_only` mode the plugin attaches, per frame: `_sdec_ssim`, `_sdec_residual`, `_sdec_ratio`, `_sdec_otsu` (active threshold), `_sdec_unimodal`, `_sdec_bypass`, `_sdec_srcFrame`, `_sdec_sceneId`, `_sdec_sceneStart`, `_sdec_sceneLen`, `_sdec_sceneOrig`, `_sdec_sceneDup`, `_sdec_alternation`, `_sdec_local_alt`.

## Companion files

- **SmartDecimateEOD_hybrid.avsi** — `SmartDecimateEOD_Splice(src, rife, mask)`, the per-frame switch between source and RIFE driven by `_sdec_bypass`.
- **SmartDecimateEOD_debug.avsi** — `SmartDecimateEOD_Overlay(clip)`, the calibration overlay.

## Scene detection

SmartDecimateEOD consumes scene boundaries through the `_scd_boundary` frame property, produced by [SCDetectEOD](https://github.com/schpuppa-art/SCDetectEOD-Avisynth-). Any source of that property works; the scene-boundary file is the universal coordination protocol across the pipeline.

## License

MIT © 2026 Shurik Pronkin
