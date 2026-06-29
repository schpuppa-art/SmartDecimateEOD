# SmartDecimateEOD.avsi
#
# Content-aware decimator for animation (AviSynth+ x64).
# The function is registered by SmartDecimateEOD.dll itself — this file is only
# an autoload helper carrying the signature reference.
# Do NOT redefine SmartDecimateEOD here; that would clash with the C++ plugin.
#
# Source / issues: https://github.com/schpuppa-art/SmartDecimateEOD
# License: MIT, (c) 2026 Shurik Pronkin
#
# Signature (named args; scenes_file wins over the optional sc clip):
#
#   SmartDecimateEOD(clip, [sc]clip,
#       float  fps_out=12.5,
#       int    th_ssim_dup=990,
#       float  residual_ceiling=5.0,
#       float  residual_mult=2.5,
#       float  th_ratio=10.0,
#       int    reg_scale=1,
#       bool   full_range=false,
#       int    thumb_w=160,
#       int    thumb_h=120,
#       float  bypass_alt=0.80,
#       int    bypass_min_len=4,
#       int    bypass_window=12,
#       string mode="cfr",          # cfr | cfr_per_scene | content | mark_only
#       int    fade_ratio=60,
#       int    fade_mean_drift=20,
#       bool   protect_fade=true,
#       bool   allow_original_cut=false,
#       string cache="",
#       string log="",
#       bool   debug=false,
#       bool   analyze_only=false,
#       string analyze_log="",
#       string scuts_out="",
#       string scenes_file="")      # path to scenes.txt; wins over sc clip
#
# Minimal use (v6.0, no SCDetectEOD clip):
#
#   src = LWLibavVideoSource("film.dgi-2.avi")
#   SmartDecimateEOD(src, fps_out=12, mode="cfr_per_scene", \
#       scenes_file="scuts.avs.scenes.txt", cache="sdec.bin", \
#       scuts_out="scuts_dec.txt")
#
# mark_only must repeat the SAME hashed params (incl. fps_out) as the cfr call
# that filled the cache, or it re-runs a full analysis pass:
#
#   SmartDecimateEOD(src, mode="mark_only", fps_out=12, \
#       scenes_file="scuts.avs.scenes.txt", cache="sdec.bin")
