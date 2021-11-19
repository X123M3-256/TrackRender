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
TRACK_SUPPORT_BASE=512,
TRACK_SPECIAL_STEEP_TO_VERTICAL=0x01000000,
TRACK_SPECIAL_VERTICAL_TO_STEEP=0x02000000,
TRACK_SPECIAL_VERTICAL=0x03000000,
TRACK_SPECIAL_VERTICAL_TWIST_LEFT=0x04000000,
TRACK_SPECIAL_VERTICAL_TWIST_RIGHT=0x05000000,
TRACK_SPECIAL_BARREL_ROLL_LEFT=0x06000000,
TRACK_SPECIAL_BARREL_ROLL_RIGHT=0x07000000,
TRACK_SPECIAL_HALF_LOOP=0x08000000,
TRACK_SPECIAL_QUARTER_LOOP=0x09000000,
TRACK_SPECIAL_CORKSCREW_LEFT=0x0A000000,
TRACK_SPECIAL_CORKSCREW_RIGHT=0x0B000000,
TRACK_SPECIAL_BRAKE=0x0C000000,
TRACK_SPECIAL_BLOCK_BRAKE=0x0D000000,
TRACK_SPECIAL_BOOSTER=0x0E000000,
TRACK_SPECIAL_LAUNCHED_LIFT=0x0F000000,
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
TRACK_MASK_UNION,
TRACK_MASK_TRANSFER_NEXT=8
};

enum track_flags
{
TRACK_HAS_LIFT=1,
TRACK_HAS_SUPPORTS=2,
TRACK_SEMI_SPLIT=4
};

enum track_subtypes
{
TRACK_SUBTYPE_DEFAULT=0,
TRACK_SUBTYPE_LIFT=1,
};

enum track_groups
{
TRACK_GROUP_FLAT=0x1,
TRACK_GROUP_BRAKES=0x2,
TRACK_GROUP_GENTLE_SLOPES=0x4,
TRACK_GROUP_STEEP_SLOPES=0x8,
TRACK_GROUP_VERTICAL_SLOPES=0x10,
TRACK_GROUP_TURNS=0x20,
TRACK_GROUP_SLOPED_TURNS=0x40,
TRACK_GROUP_DIAGONALS=0x80,
TRACK_GROUP_BANKED_TURNS=0x100,
TRACK_GROUP_BANKED_SLOPED_TURNS=0x200,
TRACK_GROUP_S_BENDS=0x400,
TRACK_GROUP_HELICES=0x800,
TRACK_GROUP_LARGE_SLOPE_TRANSITIONS=0x1000,
TRACK_GROUP_BARREL_ROLLS=0x2000,
TRACK_GROUP_INLINE_TWISTS=0x4000,
TRACK_GROUP_HALF_LOOPS=0x8000,
TRACK_GROUP_QUARTER_LOOPS=0x10000,
TRACK_GROUP_CORKSCREWS=0x20000,
TRACK_GROUP_LARGE_CORKSCREWS=0x40000,
TRACK_GROUP_BOOSTERS=0x80000,
TRACK_GROUP_LAUNCHED_LIFTS=0x100000,
TRACK_GROUP_TURN_BANK_TRANSITIONS=0x200000,
TRACK_GROUP_MEDIUM_HALF_LOOPS=0x400000,
TRACK_GROUP_LARGE_HALF_LOOPS=0x800000,
TRACK_GROUP_ZERO_G_ROLLS=0x1000000,
TRACK_GROUP_SMALL_SLOPE_TRANSITIONS=0x2000000
};



enum models
{
MODEL_FLAT,
MODEL_BANK_SIXTH,
MODEL_BANK_THIRD,
MODEL_BANK_HALF,
MODEL_BANK_TWO_THIRDS,
MODEL_BANK_FIVE_SIXTHS,
MODEL_BANK,
MODEL_BASE,
MODEL_SPECIAL_BRAKE,
MODEL_SPECIAL_BLOCK_BRAKE,
MODEL_SPECIAL_BOOSTER,
MODEL_SPECIAL_STEEP_TO_VERTICAL,
MODEL_SPECIAL_VERTICAL_TO_STEEP,
MODEL_SPECIAL_VERTICAL,
MODEL_SPECIAL_VERTICAL_TWIST,
MODEL_SPECIAL_BARREL_ROLL,
MODEL_SPECIAL_HALF_LOOP,
MODEL_SPECIAL_QUARTER_LOOP,
MODEL_SPECIAL_CORKSCREW
};
#define NUM_MODELS 19
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

void generate_subposition_data(track_section_t* track_section,char* name,int reverse);

typedef struct
	{
	track_section_t flat;
	track_section_t flat_asymmetric;
	track_section_t brake;
	track_section_t block_brake;
	track_section_t booster;
	track_section_t flat_to_gentle_up;
	track_section_t gentle_up_to_flat;
	track_section_t gentle;
	track_section_t gentle_to_steep_up;
	track_section_t steep_to_gentle_up;
	track_section_t steep;
	track_section_t steep_to_vertical_up;
	track_section_t vertical_to_steep_up;
	track_section_t vertical;
	track_section_t small_turn_left;
	track_section_t medium_turn_left;
	track_section_t large_turn_left_to_diag;
	track_section_t large_turn_right_to_diag;
	track_section_t flat_diag;
	track_section_t flat_to_gentle_up_diag;
	track_section_t gentle_to_flat_up_diag;
	track_section_t gentle_diag;
	track_section_t gentle_to_steep_up_diag;
	track_section_t steep_to_gentle_up_diag;
	track_section_t steep_diag;
	track_section_t flat_to_left_bank;
	track_section_t flat_to_right_bank;
	track_section_t left_bank_to_gentle_up;
	track_section_t right_bank_to_gentle_up;
	track_section_t gentle_up_to_left_bank;
	track_section_t gentle_up_to_right_bank;
	track_section_t left_bank;
	track_section_t flat_to_left_bank_diag;
	track_section_t flat_to_right_bank_diag;
	track_section_t left_bank_to_gentle_up_diag;
	track_section_t right_bank_to_gentle_up_diag;
	track_section_t gentle_up_to_left_bank_diag;
	track_section_t gentle_up_to_right_bank_diag;
	track_section_t left_bank_diag;
	track_section_t small_turn_left_bank;
	track_section_t medium_turn_left_bank;
	track_section_t large_turn_left_to_diag_bank;
	track_section_t large_turn_right_to_diag_bank;
	track_section_t small_turn_left_gentle_up;
	track_section_t small_turn_right_gentle_up;
	track_section_t medium_turn_left_gentle_up;
	track_section_t medium_turn_right_gentle_up;
	track_section_t very_small_turn_left_steep_up;
	track_section_t very_small_turn_right_steep_up;
	track_section_t vertical_twist_left_up;
	track_section_t vertical_twist_right_up;
	track_section_t gentle_up_to_gentle_up_left_bank;
	track_section_t gentle_up_to_gentle_up_right_bank;
	track_section_t gentle_up_left_bank_to_gentle_up;
	track_section_t gentle_up_right_bank_to_gentle_up;
	track_section_t left_bank_to_gentle_up_left_bank;
	track_section_t gentle_up_left_bank_to_left_bank;
	track_section_t right_bank_to_gentle_up_right_bank;
	track_section_t gentle_up_right_bank_to_right_bank;
	track_section_t gentle_up_left_bank;
	track_section_t gentle_up_right_bank;
	track_section_t flat_to_gentle_up_left_bank;
	track_section_t flat_to_gentle_up_right_bank;
	track_section_t gentle_up_left_bank_to_flat;
	track_section_t gentle_up_right_bank_to_flat;
	track_section_t small_turn_left_bank_gentle_up;
	track_section_t small_turn_right_bank_gentle_up;
	track_section_t medium_turn_left_bank_gentle_up;
	track_section_t medium_turn_right_bank_gentle_up;
	track_section_t s_bend_left;
	track_section_t s_bend_right;
	track_section_t small_helix_left_up;
	track_section_t small_helix_right_up;
	track_section_t medium_helix_left_up;
	track_section_t medium_helix_right_up;
	track_section_t barrel_roll_left;
	track_section_t barrel_roll_right;
	track_section_t inline_twist_left;
	track_section_t inline_twist_right;
	track_section_t half_loop;
	track_section_t medium_half_loop_left;
	track_section_t medium_half_loop_right;
	track_section_t large_half_loop_left;
	track_section_t large_half_loop_right;
	track_section_t flat_to_steep_up;
	track_section_t steep_to_flat_up;
	track_section_t small_flat_to_steep_up;
	track_section_t small_steep_to_flat_up;
	track_section_t small_flat_to_steep_up_diag;
	track_section_t small_steep_to_flat_up_diag;
	track_section_t quarter_loop_up;
	track_section_t corkscrew_left;
	track_section_t corkscrew_right;
	track_section_t large_corkscrew_left;
	track_section_t large_corkscrew_right;
	track_section_t zero_g_roll_left;
	track_section_t zero_g_roll_right;
	track_section_t large_zero_g_roll_left;
	track_section_t large_zero_g_roll_right;
	track_section_t small_turn_left_bank_to_gentle_up;
	track_section_t small_turn_right_bank_to_gentle_up;
	track_section_t launched_lift;
	}track_list_t;


extern track_list_t track_list_default;
extern track_list_t track_list_semi_split;
extern track_list_t track_list_split;

