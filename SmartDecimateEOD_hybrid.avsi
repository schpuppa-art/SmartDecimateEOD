###############################################################################
#  SmartDecimateEOD_hybrid.avsi — per-scene splicer for decimate+RIFE pipeline
#
#  Philosophy:
#    - Non-bypass scenes (on-twos, static, mixed): decimate to fps_out CFR,
#      then RIFE ×2 back to source fps. RIFE handles motion interpolation.
#    - Bypass scenes (unimodal on-ones, fade, zoom, pure motion): pass source
#      through unchanged, skip RIFE entirely — preserving art's discrete poses.
#
#  Sync invariant: every scene keeps its source duration exactly.
#    bypass scene: len frames @ fps_in → same len frames @ fps_in
#    non-bypass: len frames @ fps_in → round(len*ratio) @ fps_out 
#                → RIFE ×factor → 2*round(len*ratio) @ fps_out*2
#    For fps_out = fps_in/2 and RIFE ×2: output ≈ len frames (±1 rounding)
#
#  Usage:
#      src = LWLibavVideoSource("film.vob")
#      result = SmartDecimateEOD_Hybrid(src, 
#                   scd_args=..., dec_args=..., rife_args=...)
#
#  Simplest form with inline construction:
#      src = LWLibavVideoSource("film.vob")
#      sc = src.SCDetectEOD()
#      dec = SmartDecimateEOD(src, sc, fps_out=12.5, mode="cfr_per_scene",
#                             cache="sdec.bin")
#      dec_rgb = z_ConvertFormat(dec, pixel_type="RGBPS", colorspace_op=...)
#      rife_out = Rife(dec_rgb, model=73, factor_num=2, factor_den=1, ...)
#      rife_yuv = z_ConvertFormat(rife_out, pixel_type="YUV420P8", ...)
#      mask = SmartDecimateEOD(src, sc, mode="mark_only")
#      result = SmartDecimateEOD_Splice(src, rife_yuv, mask)
###############################################################################

# Splice function: takes source, rife_processed, and mask clips.
# All three MUST have the same frame count (source length).
# Returns a clip where each frame = source if mask._sdec_bypass=1, else rife.
function SmartDecimateEOD_Splice(clip src, clip rife, clip mask) {
    Assert(src.FrameCount == rife.FrameCount, "Splice: src/rife frame count mismatch")
    Assert(src.FrameCount == mask.FrameCount, "Splice: src/mask frame count mismatch")
    global _sdec_src = src
    global _sdec_rife = rife
    ScriptClip(mask, """
        b = propGetInt("_sdec_bypass")
        b == 1 ? _sdec_src : _sdec_rife
    """)
}
