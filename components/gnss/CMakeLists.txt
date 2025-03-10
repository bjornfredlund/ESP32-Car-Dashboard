idf_component_register(SRCS "gnss.c" "minmea.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver common screen)
set_source_files_properties(minmea.c PROPERTIES COMPILE_OPTIONS "-Dtimegm=mktime")