cmake_minimum_required (VERSION 3.1)
include_directories(../libIsoRender/src ../libImage/src)
link_directories(../libIsoRender ../libImage)

project (TrackRender C)
add_executable(TrackRender src/main.c src/track.c src/track_sections.c src/objLoader/list.c src/objLoader/obj_parser.c src/objLoader/string_extra.c)
set_property(TARGET TrackRender PROPERTY C_STANDARD 99)
target_link_libraries (TrackRender Image IsoRender png embree3 assimp jansson m)
