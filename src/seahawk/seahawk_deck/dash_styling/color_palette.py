from os import path

PATH = path.dirname(__file__)

DARK_MODE = {
    "MAIN_WIN_BKG"      : "#171717",
    "TAB_BKG"           : "#1a1a1a",
    "INACTIVE_TAB_BD"   : "#2f2f2f",
    "SURFACE_PRIMARY"   : "#212121",
    "SURFACE_SECONDARY" : "#2f2f2f",
    "ACCENT"            : "#ff5900",
    "TEXT"              : "#ded3b6",
    "TEXT_EMPH"         : "#fffbf0",
    "WARNING"           : "#fc3019",
    "LINEAR_CURVE"      : f"{PATH}/thrt_crv_linear_drk.svg",
    "CUBIC_CURVE"       : f"{PATH}/thrt_crv_cubed_drk.svg",
    "FIFTH_DEG_CURVE"   : f"{PATH}/thrt_crv_fifth_deg_drk.svg",
    "DOWN_ARROW"        : f"{PATH}/countdown_down.svg",
    "UP_ARROW"          : f"{PATH}/countdown_up.svg",
    "MAP_IMG"           : f"{PATH}/product_demo_map_drk.png"
}

LIGHT_MODE = {
    "MAIN_WIN_BKG"      : "#ececee",
    "TAB_BKG"           : "#fc3019",
    "INACTIVE_TAB_BD"   : "#2d2938",
    "SURFACE_PRIMARY"   : "#d0d0d6",
    "SURFACE_SECONDARY" : "#e3e3e4",
    "ACCENT"            : "#ff5900",
    "TEXT"              : "#2f2f2f",
    "TEXT_EMPH"         : "#0c0c0f",
    "WARNING"           : "#fc3019",
    "LINEAR_CURVE"      : f"{PATH}/thrt_crv_linear_lite.svg",
    "CUBIC_CURVE"       : f"{PATH}/thrt_crv_cubed_lite.svg",
    "FIFTH_DEG_CURVE"   : f"{PATH}/thrt_crv_fifth_deg_lite.svg",
    "DOWN_ARROW"        : f"{PATH}/countdown_down.svg",
    "UP_ARROW"          : f"{PATH}/countdown_up.svg",
    "MAP_IMG"           : f"{PATH}/product_demo_map_lite.png"
}