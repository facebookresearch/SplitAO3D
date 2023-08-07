file(GLOB SRC *.cpp)
file(GLOB SRC_HEADERS include/research_framework/*.h)

add_library(research_framework ${SRC} ${SRC_HEADERS})

target_include_directories(research_framework INTERFACE include)

target_link_libraries(research_framework PRIVATE glog rlr_streaming)
