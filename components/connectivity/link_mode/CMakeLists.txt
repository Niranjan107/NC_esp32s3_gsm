idf_component_register(
    SRCS "link_mode.c"
    INCLUDE_DIRS "include"
    REQUIRES common config
    PRIV_REQUIRES gsm wifi_sta cmd_parser json
)
