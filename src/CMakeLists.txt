idf_component_register(SRCS "main.cpp"
    INCLUDE_DIRS "." "../lib/App" "../lib/Platform"
    REQUIRES esp_lcd esp_pm esp_system esp_driver_ledc espcoredump)
