#include <renderer.h>
#include <model.h>

#define TILE_SIZE 3.67423461417477

enum {
TRACK_DIAGONAL=1,
TRACK_VERTICAL=2,
TRACK_EXTRUDE_BEHIND=4,
TRACK_ENTRY_BANK_LEFT=8,
TRACK_ENTRY_BANK_RIGHT=16,
TRACK_EXIT_BANK_LEFT=32,
TRACK_EXIT_BANK_RIGHT=64,
TRACK_NO_SUPPORTS=128,
TRACK_SPECIAL_STEEP_TO_VERTICAL=0x01000000,
TRACK_SPECIAL_VERTICAL_TO_STEEP=0x02000000,
TRACK_SPECIAL_VERTICAL=0x03000000,
TRACK_SPECIAL_VERTICAL_TWIST_LEFT=0x04000000,
TRACK_SPECIAL_VERTICAL_TWIST_RIGHT=0x05000000,
TRACK_SPECIAL_BARREL_ROLL_LEFT=0x06000000,
TRACK_SPECIAL_BARREL_ROLL_RIGHT=0x07000000,
TRACK_SPECIAL_HALF_LOOP=0x08000000,
TRACK_SPECIAL_QUARTER_LOOP=0x09000000,
TRACK_SPECIAL_MASK=0xFF000000
};

#define TRACK_BANK_LEFT (TRACK_ENTRY_BANK_LEFT|TRACK_EXIT_BANK_LEFT)
#define TRACK_BANK_RIGHT (TRACK_ENTRY_BANK_RIGHT|TRACK_EXIT_BANK_RIGHT)

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

enum track_flags
{
TRACK_HAS_SUPPORTS=1
};

enum supports
{
SUPPORT_FLAT=0,
SUPPORT_BANK_HALF=1,
SUPPORT_BANK=2,
SUPPORT_BASE=3,
SUPPORT_SPECIAL_STEEP_TO_VERTICAL=4,
SUPPORT_SPECIAL_VERTICAL_TO_STEEP=5,
SUPPORT_SPECIAL_VERTICAL=6,
SUPPORT_SPECIAL_VERTICAL_TWIST=7,
SUPPORT_SPECIAL_BARREL_ROLL=8,
SUPPORT_SPECIAL_HALF_LOOP=10,
SUPPORT_SPECIAL_QUARTER_LOOP=11
};
#define NUM_SUPPORT_MODELS 10
#define SUPPORT_SPECIAL_START SUPPORT_SPECIAL_STEEP_TO_VERTICAL

typedef struct
{
uint32_t flags;
mesh_t mesh;
mesh_t mask;
mesh_t supports[NUM_SUPPORT_MODELS];
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


void render_track_section(context_t* context,track_section_t* track_section,track_type_t* track_type,int extrude_behind,int track_mask,int views,image_t* image);


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

extern track_section_t quarter_loop_up;
