# SmartDecimateEOD

A content-aware decimator for animation, as an AviSynth+ x64 plugin. It drops
duplicate frames so the stream's total length becomes exactly `fps_out` (CFR,
audio-safe), without destroying genuinely hand-drawn motion. Input is **YV12
only**.

Classic decimators cut by a fixed ratio per chunk ("drop every Nth frame").
On animation that is destructive: scenes drawn *on ones* (`O O O O O` — every
frame new) lose real motion because the decimator cannot tell a duplicate from
an original. SmartDecimateEOD holds a **global budget** — total output count is
fixed by `fps_out`, but the distribution of kept frames across scenes is
content-adaptive. Duplicates from *on-twos* scenes (`O D D D D`) act as credit:
dropping them frees budget that is spent preserving every original in on-ones
scenes. Output length is fixed; only redundancy is cut.

## How it works

All analysis runs **once in the constructor** (stateless, survives AvsPmod graph
rebuilds). `GetFrame` afterwards is a lookup table with no decoder access.

**Pass 1 — classification (heavy).** Decodes every frame and decides "original
or duplicate" per frame. Two phases:
- *Pass 1A* — one decode pass: an SSIM thumbnail (160×120, box-averaged Y,
  studio→full, 3×3 blur) against the previous frame. In `scenes_file` mode, the
  frame's mean luma is also computed here from the thumbnail (used for fade
  detection).
- *Pass 1B* — per scene: SSIM guard + registration (SAD ±3) + block ratio.
  Registration is **deferred** — it runs only for scenes that pass the cheap
  guard; static / on-ones / fast-motion / fade scenes skip it. This is what keeps
  Pass 1 fast on mixed material (×2–4 speedup).

**Pass 2 — budget distribution (light).** Touches no frames. Takes Pass 1's
arrays and decides which frames to keep to hit `fps_out` exactly: keeps every
ORIG, fills the remaining budget with the first DUP after each ORIG (preserving
the local on-twos rhythm), error-diffuses the rounding across scenes for an exact
total. Pass 2 is a **consumer** of Pass 1's arrays, not repeated work — the two
cannot be merged into one frame loop because the budget is global (needs the sum
of ORIG/DUP across all scenes) while classification is local (frame vs neighbor).

## Boundary source (`scenes_file` wins if both are given)

Scene boundaries are a hard requirement (the per-scene CFR budget and fade
detection depend on them). Two sources, strict priority:

1. **`scenes_file`** — direct read of `scenes.txt` (SCDetectEOD format: an
   integer per line, optional `# comment`). Zero frame-prop traffic, no second
   clip needed. Read via a UTF-8/wide path helper, so Cyrillic paths work.
2. **`sc` clip** — boundaries via the `_scd_boundary` frame property on every
   frame (legacy); mean luma via `_scd_meanY`.

If both are set, `scenes_file` wins and `sc` is ignored. If neither is set, the
plugin errors out — boundaries are mandatory.

## Fade detection (both modes)

A fade (gradual darken/brighten) is a run where mean luma drifts **monotonically**.
The detector measures the drift `|meanY[scene end] − meanY[scene start]|` and the
fraction of monotonicity violations **inside** the scene; if it passes
(`fade_mean_drift`, `fade_ratio`), the scene is forced all-ORIG (`protect_fade`)
so the fade is not thinned.

This needs mean luma on **every frame** of a scene, not just on boundaries. In
`sc`-clip mode that comes per-frame from `_scd_meanY`. In `scenes_file` mode the
file carries no per-frame mean luma (it is a sparse boundary list), so mean luma
is computed from the thumbnail in Pass 1A — the frame is decoded for SSIM anyway,
so `mean(thumbnail)` is free, and its full-range Y scale matches `_scd_meanY`, so
the fade thresholds carry over unchanged.

## Cache and modes

Analysis is cached to `cache=` (a binary file with a header and all diagnostic
arrays). Two-pass workflow: a prep script populates the cache, a production
script loads it instantly.

Modes (`mode`) and their `mode_id` used in the cache key:
- **`cfr` / `cfr_per_scene`** → `mode_id=0`. Per-scene CFR: each scene keeps
  `round(len × fps_out / fps_in)` frames. Both strings are equivalent for caching.
- **`content`** → `mode_id=1`. Drops all duplicates, no budget.
- **`mark_only`** → `mode_id=2`. Cuts no frames (`N_out=N_in`); only writes
  diagnostic frame properties (`_sdec_bypass` etc.) for a hybrid RIFE pipeline.

The **cache key is a parameter hash** computed field-by-field (never over raw
struct memory, whose padding bytes are uninitialized and would cause
nondeterministic misses).

> **`mark_only` loads the cache written by `cfr` / `cfr_per_scene`** (it looks up
> `mode_id=0`, not its own `mode_id=2`). So the hashed parameters of both calls
> must match. In particular, **`fps_out` is part of the hash**: if the prep call
> sets `fps_out=12` and the `mark_only` call omits it, the default `12.5` is used,
> the hashes differ, and `mark_only` falls back to a full Pass 1. Always pass
> `fps_out` to `mark_only`, even though it does not change length there.

Not hashed (need not be duplicated between calls): `cache`, `log`, `scuts_out`,
`scenes_file`, `analyze_log`.

## Parameters

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `clip` | clip | — | YV12 source (positional) |
| `sc` | clip | — | boundary source via `_scd_boundary` (optional; needed if no `scenes_file`) |
| `scenes_file` | string | "" | path to boundary list; **wins over** `sc` |
| `fps_out` | float | 12.5 | target rate; **part of the cache hash** |
| `mode` | string | "cfr" | "cfr" / "cfr_per_scene" / "content" / "mark_only" |
| `reg_scale` | int | 1 | registration downscale (1=full, 2=half, 4=quarter); hashed |
| `bypass_alt` | float | 0.80 | alternation threshold for RIFE eligibility; hashed |
| `bypass_min_len` | int | 4 | min scene length for RIFE; hashed |
| `bypass_window` | int | 12 | half-window for local alternation |
| `th_ssim_dup` | int | 990 | SSIM guard: q80 < this → scene has no dups (skips registration); hashed |
| `residual_ceiling` | float | 5.0 | abs residual cap for percentile guard; hashed |
| `residual_mult` | float | 2.5 | threshold = q_low × mult; hashed |
| `th_ratio` | float | 10.0 | block max/mean ratio threshold; hashed |
| `full_range` | bool | false | full-range Y source; hashed |
| `thumb_w` | int | 160 | thumbnail width for SSIM / mean luma |
| `thumb_h` | int | 120 | thumbnail height |
| `fade_ratio` | int | 60 | percent 0..100, allowed non-monotonic fraction of a fade |
| `fade_mean_drift` | int | 20 | min `\|meanY[end]−meanY[start]\|` for a fade |
| `protect_fade` | bool | true | force all-ORIG on fades |
| `allow_original_cut` | bool | false | allow cutting originals (if `fps_out` is too low) |
| `cache` | string | "" | path to the binary analysis cache |
| `log` | string | "" | path to per-scene text log |
| `scuts_out` | string | "" | path to boundaries on the decimated timeline |
| `analyze_only` | bool | false | always re-run Pass 1 (no cache), diagnostics |
| `analyze_log` | string | "" | path to the analyze log |
| `debug` | bool | false | per-frame diagnostics to stderr |

## Frame properties

In `mark_only` mode (and under `debug`), on every frame:

| Prop | Meaning |
|---|---|
| `_sdec_bypass` | 0 = RIFE-eligible (on-twos), 1 = source (everything else + fades) |
| `_sdec_sceneId` | scene index |
| `_sdec_isDup` | 1 if the frame is classified as a duplicate |
| `_sdec_isFade` | 1 if the frame is in a fade scene |
| `_sdec_srcFrame` | original frame number |

On the decimated output (`cfr` modes), `_scd_boundary` is set on the new
timeline's scene boundaries, for downstream scene-aware filters (RIFE with
`sc=true`, SCSignalBorder).

## Examples

v6.0, direct file read (no SCDetectEOD):
```
src = LWLibavVideoSource("film.dgi-2.avi")
dec = SmartDecimateEOD(src, fps_out=12, mode="cfr_per_scene", bypass_alt=0.7, \
        reg_scale=2, scenes_file="scuts.avs.scenes.txt", \
        cache="sdec.bin", log="sdec.log", scuts_out="scuts_dec.txt")
```

`mark_only` for a hybrid RIFE pipeline (hashed parameters match `dec`, including
`fps_out`):
```
mask = SmartDecimateEOD(src, mode="mark_only", fps_out=12, bypass_alt=0.7, \
        reg_scale=2, scenes_file="scuts.avs.scenes.txt", cache="sdec.bin")
```

Legacy via an `sc` clip (works as before):
```
sc  = src.SCDetectEOD(scenes_file="scuts.avs.scenes.txt")
dec = SmartDecimateEOD(src, sc, fps_out=12, mode="cfr_per_scene", cache="sdec.bin")
```

## Build

```
cl /LD /EHsc /O2 SmartDecimateEOD.cpp
```
From the x64 Native Tools Command Prompt for VS 2022. Delete the old DLL from
`plugins64` before replacing it. MSVC is required throughout — MinGW-built DLLs
are ABI-incompatible with an MSVC-compiled AviSynth+.

### Files

```
SmartDecimateEOD.cpp    plugin source
SmartDecimateEOD.avsi   thin wrapper / autoload helper
LICENSE                 MIT
README.md               this file
```

You also need `avisynth.h` (the AviSynth+ C++ API header) on the include path at
build time; it is not redistributed here.

## Notes / limits

- Do **not** reuse the source `scenes_file` on the decimated clip — frame numbers
  shift. For detection on the decimated output, `scuts_out` gives the
  recalculated boundaries.
- Prep and production scripts must use identical hashed parameters, or the cache
  misses.
- `cfr` and `cfr_per_scene` are equivalent for the cache (one `mode_id`).
- If `fps_out` is too low (budget smaller than the number of originals) the
  plugin errors, recommending a higher `fps_out` or `allow_original_cut=true`.

## Compatibility

Default `mode="cfr"`, `sc` optional, `scenes_file` named at the end of the
signature. v5.x scripts with two clips work byte-for-byte: the second clip is
read as `sc`, `scenes_file` is empty → legacy path.

## License

MIT — see [LICENSE](LICENSE). © 2026 Shurik Pronkin.
