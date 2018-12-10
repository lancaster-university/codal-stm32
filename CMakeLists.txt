project(codal-stm32)

string(FIND "${device.processor}" "STM32F1" pos)

if (NOT "${pos}" STREQUAL "-1")
    set(DEVICE_DIR "stm32f1xx")
else ()
    set(DEVICE_DIR "stm32f4xx")
endif()

include("${CODAL_UTILS_LOCATION}")

# find sources and headers
RECURSIVE_FIND_DIR(TOP_LEVEL_INCLUDE_DIRS0 "${CMAKE_CURRENT_SOURCE_DIR}/stm32cube/${DEVICE_DIR}" "*.h")
RECURSIVE_FIND_DIR(TOP_LEVEL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/inc" "*.h")

RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE0 "${CMAKE_CURRENT_SOURCE_DIR}/src" "*.cpp")
RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE1 "${CMAKE_CURRENT_SOURCE_DIR}/src" "*.c")
RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/stm32cube/${DEVICE_DIR}" "*.[cs]")
LIST(APPEND TOP_LEVEL_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/asm/CortexContextSwitch.s")

# add them
include_directories(${TOP_LEVEL_INCLUDE_DIRS0} ${TOP_LEVEL_INCLUDE_DIRS} ${INCLUDE_DIRS})

# create our target
add_library(codal-stm32 ${TOP_LEVEL_SOURCE} ${TOP_LEVEL_SOURCE0} ${TOP_LEVEL_SOURCE1})

target_link_libraries(codal-stm32 codal-core ${OBJECT_FILES} ${ARCHIVE_FILES})

#set_target_properties(codal-stm32 PROPERTIES LINKER_LANGUAGE CXX)

# expose it to parent cmake.
target_include_directories(codal-stm32 PUBLIC ${TOP_LEVEL_INCLUDE_DIRS} ${TOP_LEVEL_INCLUDE_DIRS0} ${INCLUDE_DIRS})
