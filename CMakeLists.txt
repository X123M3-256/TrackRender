cmake_minimum_required (VERSION 3.1)
include_directories(../libIsoRender ../libRCT2/src)
link_directories(../libIsoRender ../libRCT2)

project (TrackRender C)
add_executable(TrackRender src/main.c src/track.c src/track_sections.c src/png.c src/objLoader/list.c src/objLoader/obj_parser.c src/objLoader/string_extra.c)
set_property(TARGET TrackRender PROPERTY C_STANDARD 99)
target_link_libraries (TrackRender IsoRender RCT2 png jansson m)
