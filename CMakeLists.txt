cmake_minimum_required (VERSION 3.1)

project (TrackRender C)

find_package(PNG REQUIRED)
find_package(ZLIB REQUIRED)

include_directories(${PNG_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS} ../libIsoRender/src)
link_directories(${PNG_LIBRARY_DIRS} ${ZLIB_LIBRARY_DIRS} ../libIsoRender)

add_executable(maketrack src/main.c src/track.c src/track_sections.c src/sprites.c)
set_property(TARGET maketrack PROPERTY C_STANDARD 99)
#target_link_libraries(maketrack ${PNG_LIBRARY} ${ZLIB_LIBRARY})
target_link_libraries (maketrack m IsoRender ${PNG_LIBRARY} ${ZLIB_LIBRARY} assimp embree3 jansson)

add_executable(subposition src/subposition.c src/track_sections.c)
set_property(TARGET subposition PROPERTY C_STANDARD 99)
target_link_libraries (subposition m IsoRender)
