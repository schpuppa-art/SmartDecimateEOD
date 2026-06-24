###############################################################################
#  SmartDecimateEOD_debug.avsi — overlay helpers for calibration
#
#  Usage:
#      sc = SCDetectEOD(cache="scd.bin")
#      ana = SmartDecimateEOD(sc, analyze_only=true, analyze_log="sdec.csv")
#      SmartDecimateEOD_Overlay(ana)
#  Result: every source frame is kept, with on-screen labels:
#      top-left : frame #, scene id, scene range/len, orig/dup counts
#      middle   : big ORIG / DUP / FADE / UNIMODAL tag in color
#      bottom-left: diff value vs. otsu threshold
###############################################################################

function SmartDecimateEOD_Overlay(clip c) {
    c

    # Big centered verdict.
    ScriptClip("""
        sid  = propGetInt("_sdec_sceneId")
        isd  = propGetInt("_sdec_isDup")
        isf  = propGetInt("_sdec_isFade")
        uni  = propGetInt("_sdec_unimodal")
        tag  = isf==1 ? "FADE-ORIG" : \
               uni==1 ? "UNIMODAL-ORIG" : \
               isd==1 ? "DUP" : "ORIG"
        col  = isf==1 ? $0080FF :
\              uni==1 ? $00FF80 :
\              isd==1 ? $FF4040 : $40FF40
        Subtitle(tag, align=5, size=48, text_color=col, halo_color=$000000)
    """)

    # Top-left: scene info.
    ScriptClip("""
        src  = propGetInt("_sdec_srcFrame")
        sid  = propGetInt("_sdec_sceneId")
        sst  = propGetInt("_sdec_sceneStart")
        sln  = propGetInt("_sdec_sceneLen")
        sor  = propGetInt("_sdec_sceneOrig")
        sdu  = propGetInt("_sdec_sceneDup")
        la   = propGetFloat("_sdec_local_alt")
        bp   = propGetInt("_sdec_bypass")
        t = "src="+String(src) + "  scene="+String(sid) + \
            "  ["+String(sst)+".."+String(sst+sln-1)+"] len="+String(sln) + \
            "  O="+String(sor)+" D="+String(sdu) + \
            "  la="+String(la,"%.2f")+" bp="+String(bp)
        Subtitle(t, x=8, y=8, size=18, text_color=$FFFFFF, halo_color=$000000)
    """)

    # Bottom-left: hybrid metrics.
    ScriptClip("""
        s  = propGetInt("_sdec_ssim")
        r  = propGetFloat("_sdec_residual")
        rt = propGetFloat("_sdec_ratio")
        o  = propGetFloat("_sdec_otsu")
        t = "ssim="+String(s) + " res="+String(r,"%.2f") + " rat="+String(rt,"%.1f") + " thr="+String(o,"%.2f")
        Subtitle(t, x=8, y=Height-28, size=18, \
                 text_color=$FFFFFF, halo_color=$000000)
    """)
}
