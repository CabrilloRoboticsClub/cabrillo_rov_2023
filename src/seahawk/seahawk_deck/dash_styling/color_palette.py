from os import path

PATH = path.dirname(__file__)

DARK_MODE = {
    "MAIN_WIN_BKG"      : "#171717",
    "INACTIVE_TAB_BKG"  : "#1a1a1a",
    "INACTIVE_TAB_BD"   : "#2f2f2f",
    "ACTIVE_TAB_BKG"    : "#1a1a1a",
    "SURFACE_DRK"       : "#212121",
    "SURFACE_BRIGHT"    : "#2f2f2f",
    "ACCENT_LIGHT"      : "#ff5900",
    "PRIMARY_TEXT"      : "#fffbf0",
    "SECONDARY_TEXT"    : "#ded3b6",
    "WARNING"           : "#fc3019",
    "LINEAR_CURVE"      : f"{PATH}/thrt_crv_linear_drk.svg",
    "CUBIC_CURVE"       : f"{PATH}/thrt_crv_cubed_drk.svg",
    "FIFTH_DEG_CURVE"   : f"{PATH}/thrt_crv_fifth_deg_drk.svg",
    "DOWN_ARROW"        : f"{PATH}/countdown_down.svg",
    "UP_ARROW"          : f"{PATH}/countdown_up.svg",
    "MAP_IMG"           : f"{PATH}/product_demo_map_drk.png"
}

# thrt_crv_fifth_deg_drk

LIGHT_MODE = {
    "MAIN_WIN_BKG"      : "#ececee",
    "INACTIVE_TAB_BKG"  : "#d0d0d6",
    "INACTIVE_TAB_BD"   : "#2d2938",
    "ACTIVE_TAB_BKG"    : "#d9d9de",
    "SURFACE_DRK"       : "#d0d0d6",
    "SURFACE_BRIGHT"    : "#e3e3e4",
    "ACCENT_LIGHT"      : "#ff5900",
    "ACCENT_DRK"        : "#5c4ba3",
    "PRIMARY_TEXT"      : "#0c0c0f",
    "SECONDARY_TEXT"    : "#2f2f2f",
    "WARNING"           : "#fc3019",
    "LINEAR_CURVE"      : f"{PATH}/thrt_crv_linear_lite.svg",
    "CUBIC_CURVE"       : f"{PATH}/thrt_crv_cubed_lite.svg",
    "FIFTH_DEG_CURVE"   : f"{PATH}/thrt_crv_fifth_deg_lite.svg",
    "DOWN_ARROW"        : f"{PATH}/countdown_down.svg",
    "UP_ARROW"          : f"{PATH}/countdown_up.svg",
    "MAP_IMG"           : f"{PATH}/product_demo_map_lite.png"
}