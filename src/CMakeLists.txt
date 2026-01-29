file(GLOB SRC_FILES CONFIGURE_DEPENDS *.cpp *.h)
target_sources(graphics PRIVATE ${SRC_FILES})
target_include_directories(graphics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})