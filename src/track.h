#include <jansson.h>
#include <renderer.h>
#include <model.h>


#define TILE_SIZE 3.3//3.67423461417477
#define CLEARANCE_HEIGHT (0.20412414523*TILE_SIZE)


enum {
TRACK_DIAGONAL=1,
TRACK_VERTICAL=2,
TRACK_EXTRUDE_BEHIND=4,
TRACK_ENTRY_BANK_LEFT=8,
TRACK_ENTRY_BANK_RIGHT=16,
TRACK_EXIT_BANK_LEFT=32,
TRACK_EXIT_BANK_RIGHT=64,
TRACK_NO_SUPPORTS=128,
TRACK_OFFSET_SPRITE_MASK=256,
TRACK_SPECIAL_STEEP_TO_VERTICAL=0x01000000,
TRACK_SPECIAL_VERTICAL_TO_STEEP=0x02000000,
TRACK_SPECIAL_VERTICAL=0x03000000,
TRACK_SPECIAL_VERTICAL_TWIST_LEFT=0x04000000,
TRACK_SPECIAL_VERTICAL_TWIST_RIGHT=0x05000000,
TRACK_SPECIAL_BARREL_ROLL_LEFT=0x06000000,
TRACK_SPECIAL_BARREL_ROLL_RIGHT=0x07000000,
TRACK_SPECIAL_HALF_LOOP=0x08000000,
TRACK_SPECIAL_QUARTER_LOOP=0x09000000,
TRACK_SPECIAL_BRAKE=0x0A000000,
TRACK_SPECIAL_BLOCK_BRAKE=0x0B000000,
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
TRACK_HAS_LIFT=1,
TRACK_HAS_SUPPORTS=2
};

enum track_subtypes
{
TRACK_SUBTYPE_DEFAULT=0,
TRACK_SUBTYPE_LIFT=1,
TRACK_SUBTYPE_BRAKE=2,
TRACK_SUBTYPE_BLOCK_BRAKE=3
};

enum track_groups
{
TRACK_GROUP_BRAKES=1,
TRACK_GROUP_GENTLE_SLOPES=2,
TRACK_GROUP_STEEP_SLOPES=4,
TRACK_GROUP_VERTICAL_SLOPES=8,
TRACK_GROUP_TURNS=16,
TRACK_GROUP_SLOPED_TURNS=32,
TRACK_GROUP_DIAGONALS=64,
TRACK_GROUP_BANKED_TURNS=128,
TRACK_GROUP_BANKED_SLOPED_TURNS=256,
TRACK_GROUP_S_BENDS=512,
TRACK_GROUP_HELICES=1024,
TRACK_GROUP_LARGE_SLOPE_TRANSITIONS=2048,
TRACK_GROUP_BARREL_ROLLS=4096,
TRACK_GROUP_HALF_LOOPS=8192,
TRACK_GROUP_QUARTER_LOOPS=16384,
TRACK_GROUP_BOOSTERS=32768,
TRACK_GROUP_LAUNCHED_LIFTS=65536
};



enum models
{
MODEL_FLAT,
MODEL_BANK_HALF,
MODEL_BANK,
MODEL_BASE,
MODEL_SPECIAL_BRAKE,
MODEL_SPECIAL_BLOCK_BRAKE,
MODEL_SPECIAL_STEEP_TO_VERTICAL,
MODEL_SPECIAL_VERTICAL_TO_STEEP,
MODEL_SPECIAL_VERTICAL,
MODEL_SPECIAL_VERTICAL_TWIST,
MODEL_SPECIAL_BARREL_ROLL,
MODEL_SPECIAL_HALF_LOOP,
MODEL_SPECIAL_QUARTER_LOOP
};
#define NUM_MODELS 13
#define SUPPORT_SPECIAL_START SUPPORT_SPECIAL_STEEP_TO_VERTICAL

typedef struct
{
uint32_t flags;
uint32_t groups;
uint32_t lift_groups;
uint32_t models_loaded;
mesh_t mesh;
mesh_t lift_mesh;
mesh_t mask;
mesh_t models[NUM_MODELS];
float length;
float z_offset;
float support_spacing;
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


int write_track_type(context_t* context,track_type_t* track_type,json_t* sprites,const char* base_dir,const char* output_dir);

extern track_section_t flat;
extern track_section_t flat_asymmetric;
extern track_section_t brake;
extern track_section_t block_brake;
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


extern track_section_t semi_split_left_bank;
extern track_section_t semi_split_left_bank_diag;
extern track_section_t semi_split_small_turn_left_bank;
extern track_section_t semi_split_medium_turn_left_bank;
extern track_section_t semi_split_large_turn_left_to_diag_bank;
extern track_section_t semi_split_large_turn_right_to_diag_bank;

extern track_section_t semi_split_left_bank_to_gentle_up_left_bank;
extern track_section_t semi_split_right_bank_to_gentle_up_right_bank;
extern track_section_t semi_split_gentle_up_left_bank_to_left_bank;
extern track_section_t semi_split_gentle_up_right_bank_to_right_bank;

extern track_section_t semi_split_small_turn_left_bank_gentle_up;
extern track_section_t semi_split_small_turn_right_bank_gentle_up;
extern track_section_t semi_split_medium_turn_left_bank_gentle_up;
extern track_section_t semi_split_medium_turn_right_bank_gentle_up;

extern track_section_t semi_split_small_helix_left_up;
extern track_section_t semi_split_small_helix_right_up;
extern track_section_t semi_split_medium_helix_left_up;
extern track_section_t semi_split_medium_helix_right_up;

extern track_section_t semi_split_quarter_loop_up;
