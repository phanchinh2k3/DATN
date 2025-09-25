idf_component_register(
    SRCS "ina219.c"
    INCLUDE_DIRS "."
    REQUIRES i2cdev driver log
)
