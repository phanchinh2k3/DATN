set(req driver freertos esp_idf_lib_helpers)

if(CONFIG_I2CDEV_USE_LEGACY_DRIVER)
    set(USE_LEGACY_DRIVER TRUE)
elseif(NOT DEFINED IDF_VERSION_MAJOR)
    set(USE_LEGACY_DRIVER TRUE)
elseif(IDF_VERSION_MAJOR LESS 5)
    set(USE_LEGACY_DRIVER TRUE)
elseif(IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR LESS 3)
    set(USE_LEGACY_DRIVER TRUE)
else()
    set(USE_LEGACY_DRIVER FALSE)
endif()

if(USE_LEGACY_DRIVER)
    set(SRCS "i2cdev_legacy.c")
else()
    set(SRCS "i2cdev.c")
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES ${req}
)
