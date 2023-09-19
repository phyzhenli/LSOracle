project(dagp)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/common COM_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/recBisection REC_FILES)

add_library(${PROJECT_NAME} STATIC ${COM_FILES} ${REC_FILES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/common ${CMAKE_CURRENT_SOURCE_DIR}/src/recBisection)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../metis/include)

add_definitions(-DdagP_METIS)