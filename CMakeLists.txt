if(CONFIG_CROSSES_INPUT_PROCESSOR_ROTATE)
  zephyr_library()
  zephyr_library_sources(src/input_processor_rotate.c)
endif()
