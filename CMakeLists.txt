project(codal-stm32)

include("${CODAL_UTILS_LOCATION}")

# find sources and headers
RECURSIVE_FIND_DIR(TOP_LEVEL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/inc" "*.h")
RECURSIVE_FIND_DIR(TOP_LEVEL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/stm32cube/stm32f4xx" "*.h")
RECURSIVE_FIND_DIR(INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/${CODAL_OUTPUT_NAME}/inc" "*.h")

RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/source" "*.c??")
RECURSIVE_FIND_FILE(TOP_LEVEL_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/stm32cube/stm32f4xx" "*.c")

# add them
include_directories(${TOP_LEVEL_INCLUDE_DIRS} ${INCLUDE_DIRS})

# create our target
add_library(codal-stm32 ${TOP_LEVEL_SOURCE})

target_link_libraries(codal-stm32 codal-core ${OBJECT_FILES} ${ARCHIVE_FILES})

#set_target_properties(codal-stm32 PROPERTIES LINKER_LANGUAGE CXX)

# expose it to parent cmake.
target_include_directories(codal-stm32 PUBLIC ${TOP_LEVEL_INCLUDE_DIRS} ${INCLUDE_DIRS})