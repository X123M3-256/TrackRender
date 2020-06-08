cmake_minimum_required (VERSION 3.1)

project (TrackRender C)

add_subdirectory(libIsoRender)
include_directories(libIsoRender/src libIsoRender/libImage/src /usr/local/Cellar/jansson/2.12/include)
link_directories(/usr/local/Cellar/jansson/2.12/lib/)

if(MSVC) # assumes vcpkg
      find_package(jansson CONFIG REQUIRED)
      find_package(PNG REQUIRED)
      find_package(embree 3 REQUIRED)
      find_package(assimp CONFIG REQUIRED)
      link_directories(${EMBREE_ROOT_DIR}/lib)
      set(JANSSON jansson::jansson)
      add_definitions(/D_USE_MATH_DEFINES /DNOMINMAX /wd4305 /wd4244 /wd4996)
else()
      set(JANSSON jansson)
endif()

add_executable(TrackRender src/main.c src/track.c src/track_sections.c src/sprites.c)
set_property(TARGET TrackRender PROPERTY C_STANDARD 99)
target_link_libraries (TrackRender Image IsoRender ${JANSSON})
