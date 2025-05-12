find_package(Python3 REQUIRED COMPONENTS Interpreter)

if(Python3_VERSION VERSION_LESS 3.5 AND (NOT DEFINED ENABLE_tfg OR ENABLE_tfg))
    message(WARNING "Python interpreter not found or unsupported version (only 3.5+), disabling tfg program")
endif()

cmake_dependent_option(ENABLE_tfg "Enable/disable tfg program" ON
                       "Python3_FOUND;NOT Python3_VERSION VERSION_LESS 3.5" OFF)

if(ENABLE_tfg)
    configure_file("${CMAKE_SOURCE_DIR}/cmake/templates/setup.py.in"
                   "${CMAKE_BINARY_DIR}/setup.py")

    install(CODE "EXECUTE_PROCESS(COMMAND \"${Python3_EXECUTABLE}\" \"${CMAKE_BINARY_DIR}/setup.py\" install)")
else()
    set(ENABLE_tfg OFF CACHE BOOL "Enable/disable tfg program" FORCE)
endif()
