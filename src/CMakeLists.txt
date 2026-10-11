idf_component_register(SRCS "main.cpp"
    INCLUDE_DIRS "." "../lib/App" "../lib/Platform"
    REQUIRES framework-arduinoespressif32 esp_lcd esp_pm esp_system esp_driver_ledc espcoredump)
