#include <renderer.h>
#include <model.h>

#define TILE_SIZE 3.67423461417477

typedef struct
{
mesh_t mesh;
float length;
}track_type_t;

typedef struct
{
vector3_t position;
vector3_t normal;
vector3_t tangent;
vector3_t binormal;
}track_point_t;

typedef struct
{
int num_rects;
int offset_x;
int offset_y;
rect_t* rects;
}mask_t;


typedef struct
{
int num_sprites;
mask_t* masks;
}view_t;

typedef struct
{
track_point_t (*curve)(float distance);
float length;
view_t views[4];
}track_section_t;


void framebuffer_render_track_section(framebuffer_t* framebuffer,context_t* context,track_section_t* track_section,track_type_t* track_type);


extern track_section_t flat;
extern track_section_t flat_to_gentle_up;
extern track_section_t flat_to_gentle_down;
extern track_section_t gentle_up_to_flat;
extern track_section_t gentle_down_to_flat;
extern track_section_t gentle;
extern track_section_t gentle_to_steep_up;
extern track_section_t gentle_to_steep_down;
extern track_section_t steep_to_gentle_up;
extern track_section_t steep_to_gentle_down;
extern track_section_t steep;
extern track_section_t steep_to_vertical_up;
extern track_section_t steep_to_vertical_down;
extern track_section_t vertical;
extern track_section_t small_turn_left;
extern track_section_t medium_turn_left;
extern track_section_t large_turn_left_to_diag;
extern track_section_t large_turn_right_to_diag;
extern track_section_t flat_diag;
extern track_section_t flat_to_gentle_up_diag;
extern track_section_t flat_to_gentle_down_diag;
extern track_section_t gentle_up_to_flat_diag;
extern track_section_t gentle_down_to_flat_diag;
extern track_section_t gentle_diag;
extern track_section_t gentle_to_steep_up_diag;
extern track_section_t gentle_to_steep_down_diag;
extern track_section_t steep_to_gentle_up_diag;
extern track_section_t steep_to_gentle_down_diag;
extern track_section_t steep_diag;

extern track_section_t vertical_twist_left;
extern track_section_t vertical_twist_right;
extern track_section_t heartline_roll_left;

