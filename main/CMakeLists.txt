idf_component_register(SRCS "bt_car_telemetry.c"
INCLUDE_DIRS "."
REQUIRES common gnss led led_blink_task screen thermometer accelerometer)
