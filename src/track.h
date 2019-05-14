#include <renderer.h>
#include <model.h>

#define TILE_SIZE 3.67423461417477

enum {
TRACK_DIAGONAL=1,
TRACK_VERTICAL=2,
TRACK_EXTRUDE_BEHIND=4,
};

enum {
VIEW_NEEDS_TRACK_MASK=1,
VIEW_ENFORCE_NON_OVERLAPPING=2
};

enum {
TRACK_MASK_NONE=0,
TRACK_MASK_DIFFERENCE,
TRACK_MASK_INTERSECT,
TRACK_MASK_UNION
};

typedef struct
{
mesh_t mesh;
mesh_t mask;
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
int track_mask_op;
int num_rects;
int x_offset;
int y_offset;
rect_t* rects;
}mask_t;


typedef struct
{
int flags;
int num_sprites;
mask_t* masks;
}view_t;



typedef struct
{
unsigned int flags;
track_point_t (*curve)(float distance);
float length;
view_t views[4];
}track_section_t;


void framebuffer_render_track_section(framebuffer_t* framebuffer,context_t* context,track_section_t* track_section,track_type_t* track_type,int extrude_behind,int track_mask);


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
extern track_section_t vertical_to_steep_up;
extern track_section_t vertical;
extern track_section_t small_turn_left;
extern track_section_t medium_turn_left;
extern track_section_t large_turn_left_to_diag;
extern track_section_t large_turn_right_to_diag;
extern track_section_t flat_diag;
extern track_section_t flat_to_gentle_up_diag;
extern track_section_t flat_to_gentle_down_diag;
extern track_section_t gentle_up_to_flat_diag;
extern track_section_t gentle_to_flat_up_diag;
extern track_section_t gentle_diag;
extern track_section_t gentle_to_steep_up_diag;
extern track_section_t gentle_to_steep_down_diag;
extern track_section_t steep_to_gentle_up_diag;
extern track_section_t steep_to_gentle_down_diag;
extern track_section_t steep_diag;
extern track_section_t flat_to_left_bank;
extern track_section_t flat_to_right_bank;
extern track_section_t left_bank_to_gentle_up;
extern track_section_t right_bank_to_gentle_up;
extern track_section_t gentle_up_to_left_bank;
extern track_section_t gentle_up_to_right_bank;
extern track_section_t left_bank;
extern track_section_t flat_to_left_bank_diag;
extern track_section_t flat_to_right_bank_diag;
extern track_section_t left_bank_to_gentle_up_diag;
extern track_section_t right_bank_to_gentle_up_diag;
extern track_section_t gentle_up_to_left_bank_diag;
extern track_section_t gentle_up_to_right_bank_diag;
extern track_section_t left_bank_diag;
extern track_section_t small_turn_left_bank;
extern track_section_t medium_turn_left_bank;
extern track_section_t large_turn_left_to_diag_bank;
extern track_section_t large_turn_right_to_diag_bank;
extern track_section_t small_turn_left_gentle_up;
extern track_section_t small_turn_right_gentle_up;
extern track_section_t medium_turn_left_gentle_up;
extern track_section_t medium_turn_right_gentle_up;
extern track_section_t very_small_turn_left_steep_up;
extern track_section_t very_small_turn_right_steep_up;
extern track_section_t vertical_twist_left_up;
extern track_section_t vertical_twist_right_up;

extern track_section_t gentle_up_to_gentle_up_left_bank;
extern track_section_t gentle_up_to_gentle_up_right_bank;
extern track_section_t gentle_up_left_bank_to_gentle_up;
extern track_section_t gentle_up_right_bank_to_gentle_up;

extern track_section_t left_bank_to_gentle_up_left_bank;
extern track_section_t gentle_up_left_bank_to_left_bank;
extern track_section_t right_bank_to_gentle_up_right_bank;
extern track_section_t gentle_up_right_bank_to_right_bank;

extern track_section_t gentle_up_left_bank;
extern track_section_t gentle_up_right_bank;

extern track_section_t flat_to_gentle_up_left_bank;
extern track_section_t flat_to_gentle_up_right_bank;
extern track_section_t gentle_up_left_bank_to_flat;
extern track_section_t gentle_up_right_bank_to_flat;

extern track_section_t small_turn_left_bank_gentle_up;
extern track_section_t small_turn_right_bank_gentle_up;
extern track_section_t medium_turn_left_bank_gentle_up;
extern track_section_t medium_turn_right_bank_gentle_up;

extern track_section_t s_bend_left;
extern track_section_t s_bend_right;

extern track_section_t small_helix_left_up;
extern track_section_t small_helix_right_up;
extern track_section_t medium_helix_left_up;
extern track_section_t medium_helix_right_up;


extern track_section_t barrel_roll_left;
extern track_section_t barrel_roll_right;
extern track_section_t half_loop;

extern track_section_t flat_to_steep_up;
extern track_section_t steep_to_flat_up;
