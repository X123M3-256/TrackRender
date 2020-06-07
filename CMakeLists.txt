cmake_minimum_required (VERSION 3.1)

project (TrackRender C)

add_subdirectory(libIsoRender)
include_directories(libIsoRender/src libIsoRender/libImage/src /usr/local/Cellar/jansson/2.12/include)
link_directories(/usr/local/Cellar/jansson/2.12/lib/)

if(MSVC) # assumes vcpkg
      find_package(jansson CONFIG REQUIRED)
      set(JANSSON jansson::jansson)
else()
      set(JANSSON jansson)
endif()

add_executable(TrackRender src/main.c src/track.c src/track_sections.c src/sprites.c)
set_property(TARGET TrackRender PROPERTY C_STANDARD 99)
target_link_libraries (TrackRender Image IsoRender ${JANSSON})
