idf_component_register(SRCS "thermometer.c"
                    INCLUDE_DIRS "include"
                    REQUIRES common screen
                    PRIV_REQUIRES driver)
