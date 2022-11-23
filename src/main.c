#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include "track.h"

context_t get_context(light_t* lights,uint32_t num_lights)
{
context_t context;
context_init(&context,lights,num_lights,palette_rct2(),TILE_SIZE);
//context.palette.colors[0].r=0;
//context.palette.colors[0].g=0;
//context.palette.colors[0].b=0;
return context;
}


int load_model(mesh_t* model,json_t* json,const char* name)
{
json_t* mesh=json_object_get(json,name);
	if(mesh!=NULL)
	{
		if(json_is_string(mesh))
		{
		if(mesh_load_transform(model,json_string_value(mesh),rotate_y(-0.5*M_PI)))
			{
			printf("Failed to load model \"%s\"\n",json_string_value(mesh));
			return 1;
			}
		return 0;
		}
	printf("Error: Property \"%s\" not found or is not an object\n",name);
	return 1;
	}
return 2;
}

int load_groups(json_t* json,uint32_t* out)
{
//Load track sections
uint32_t groups=0;
	for(int i=0;i<json_array_size(json);i++)
	{
	json_t* group_name=json_array_get(json,i);
	assert(group_name!=NULL);
		if(!json_is_string(group_name))
		{
		printf("Error: Array \"sections\" contains non-string value\n");
		return 1;
		}
		if(strcmp(json_string_value(group_name),"flat")==0)groups|=TRACK_GROUP_FLAT;
		else if(strcmp(json_string_value(group_name),"brakes")==0)groups|=TRACK_GROUP_BRAKES;
		else if(strcmp(json_string_value(group_name),"turns")==0)groups|=TRACK_GROUP_TURNS;
		else if(strcmp(json_string_value(group_name),"gentle_slopes")==0)groups|=TRACK_GROUP_GENTLE_SLOPES;
		else if(strcmp(json_string_value(group_name),"steep_slopes")==0)groups|=TRACK_GROUP_STEEP_SLOPES;
		else if(strcmp(json_string_value(group_name),"vertical_slopes")==0)groups|=TRACK_GROUP_VERTICAL_SLOPES;
		else if(strcmp(json_string_value(group_name),"diagonals")==0)groups|=TRACK_GROUP_DIAGONALS;
		else if(strcmp(json_string_value(group_name),"sloped_turns")==0)groups|=TRACK_GROUP_SLOPED_TURNS|TRACK_GROUP_STEEP_SLOPED_TURNS;
		else if(strcmp(json_string_value(group_name),"gentle_sloped_turns")==0)groups|=TRACK_GROUP_SLOPED_TURNS;
		else if(strcmp(json_string_value(group_name),"banked_turns")==0)groups|=TRACK_GROUP_BANKED_TURNS;
		else if(strcmp(json_string_value(group_name),"banked_sloped_turns")==0)groups|=TRACK_GROUP_BANKED_SLOPED_TURNS;
		else if(strcmp(json_string_value(group_name),"s_bends")==0)groups|=TRACK_GROUP_S_BENDS;
		else if(strcmp(json_string_value(group_name),"helices")==0)groups|=TRACK_GROUP_HELICES;
		else if(strcmp(json_string_value(group_name),"small_slope_transitions")==0)groups|=TRACK_GROUP_SMALL_SLOPE_TRANSITIONS;
		else if(strcmp(json_string_value(group_name),"large_slope_transitions")==0)groups|=TRACK_GROUP_LARGE_SLOPE_TRANSITIONS;
		else if(strcmp(json_string_value(group_name),"barrel_rolls")==0)groups|=TRACK_GROUP_BARREL_ROLLS;
		else if(strcmp(json_string_value(group_name),"inline_twists")==0)groups|=TRACK_GROUP_INLINE_TWISTS;
		else if(strcmp(json_string_value(group_name),"quarter_loops")==0)groups|=TRACK_GROUP_QUARTER_LOOPS;
		else if(strcmp(json_string_value(group_name),"corkscrews")==0)groups|=TRACK_GROUP_CORKSCREWS;
		else if(strcmp(json_string_value(group_name),"large_corkscrews")==0)groups|=TRACK_GROUP_LARGE_CORKSCREWS;
		else if(strcmp(json_string_value(group_name),"half_loops")==0)groups|=TRACK_GROUP_HALF_LOOPS;
		else if(strcmp(json_string_value(group_name),"medium_half_loops")==0)groups|=TRACK_GROUP_MEDIUM_HALF_LOOPS;
		else if(strcmp(json_string_value(group_name),"large_half_loops")==0)groups|=TRACK_GROUP_LARGE_HALF_LOOPS;
		else if(strcmp(json_string_value(group_name),"zero_g_rolls")==0)groups|=TRACK_GROUP_ZERO_G_ROLLS;
		else if(strcmp(json_string_value(group_name),"boosters")==0)groups|=TRACK_GROUP_BOOSTERS;
		else if(strcmp(json_string_value(group_name),"launched_lifts")==0)groups|=TRACK_GROUP_LAUNCHED_LIFTS;
		else if(strcmp(json_string_value(group_name),"turn_bank_transitions")==0)groups|=TRACK_GROUP_TURN_BANK_TRANSITIONS;
		else if(strcmp(json_string_value(group_name),"vertical_boosters")==0)groups|=TRACK_GROUP_VERTICAL_BOOSTERS;
		else
		{
		printf("Error: Unrecognized section group \"%s\"\n",json_string_value(group_name));
		return 1;
		}
	}
*out=groups;
return 0;
}

int load_track_type(track_type_t* track_type,json_t* json)
{
//Load track flags
track_type->flags=0;
json_t* flags=json_object_get(json,"flags");
	if(flags!=NULL)
	{
		if(!json_is_array(flags))
		{
		printf("Error: Property \"flags\" is not an array\n");
		return 1;
		}
		for(int i=0;i<json_array_size(flags);i++)
		{
		json_t* flag_name=json_array_get(flags,i);
		assert(flag_name!=NULL);
			if(!json_is_string(flag_name))
			{
			printf("Error: Array \"flags\" contains non-string value\n");
			return 1;
			}
			if(strcmp(json_string_value(flag_name),"has_lift")==0)track_type->flags|=TRACK_HAS_LIFT;
			else if(strcmp(json_string_value(flag_name),"has_supports")==0)track_type->flags|=TRACK_HAS_SUPPORTS;
			else if(strcmp(json_string_value(flag_name),"semi_split")==0)track_type->flags|=TRACK_SEMI_SPLIT;
			else if(strcmp(json_string_value(flag_name),"split")==0)track_type->flags|=TRACK_SPLIT;
			else if(strcmp(json_string_value(flag_name),"no_lift_sprite")==0)track_type->flags|=TRACK_NO_LIFT_SPRITE;
			else if(strcmp(json_string_value(flag_name),"separate_tie")==0)track_type->flags|=TRACK_SEPARATE_TIE;
			else if(strcmp(json_string_value(flag_name),"tie_at_boundary")==0)track_type->flags|=TRACK_SEPARATE_TIE|TRACK_TIE_AT_BOUNDARY;
			else
			{
			printf("Error: Unrecognized flag \"%s\"\n",json_string_value(flag_name));
			return 1;
			}
		}
	}

json_t* groups=json_object_get(json,"sections");
	if(groups!=NULL)
	{
		if(!json_is_array(groups))
		{
		printf("Error: Property \"sections\" is not an array\n");
		return 1;
		}
		if(load_groups(groups,&(track_type->groups)))return 1;
	}
	
	if(track_type->flags&TRACK_HAS_LIFT)
	{
	json_t* groups=json_object_get(json,"lift_sections");
		if(groups!=NULL)
		{
			if(!json_is_array(groups))
			{
			printf("Error: Property \"lift_sections\" is not an array\n");
			return 1;
			}
			if(load_groups(groups,&(track_type->lift_groups)))return 1;
		}
	json_t* offset=json_object_get(json,"lift_offset");
		if(offset)
		{
			if(!json_is_integer(offset))
			{
			printf("Error: Property \"lift_sections\" is not an array\n");
			return 1;
			}
		track_type->lift_offset=json_integer_value(offset);
		}
		else track_type->lift_offset=13;
	}


//Load length
json_t* length=json_object_get(json,"length");
	if(length!=NULL&&json_is_number(length))track_type->length=json_number_value(length)*TILE_SIZE;
	else
	{
	printf("Error: Property \"length\" not found or is not a number\n");
	return 1;
	}


//Load tie length
	if(track_type->flags&TRACK_TIE_AT_BOUNDARY)
	{
	json_t* tie_length=json_object_get(json,"tie_length");
		if(tie_length!=NULL&&json_is_number(tie_length))track_type->tie_length=json_number_value(tie_length)*TILE_SIZE;
		else
		{
		printf("Error: Property \"tie_length\" not found or is not a number\n");
		return 1;
		}
	}


//Load Z offset
json_t* z_offset=json_object_get(json,"z_offset");
	if(z_offset!=NULL&&json_is_number(z_offset))track_type->z_offset=json_number_value(z_offset);
	else
	{
	printf("Error: Property \"z_offset\" not found or is not a number\n");
	return 1;
	}

//Load support_spacing
json_t* support_spacing=json_object_get(json,"support_spacing");
	if(support_spacing!=NULL)
	{
		if(json_is_number(support_spacing))track_type->support_spacing=json_number_value(support_spacing)*TILE_SIZE;
		else
		{
		printf("Error: Property \"support_spacing\" not found or is not a number\n");
		return 1;
		}
	} else track_type->support_spacing=TILE_SIZE;

//Load pivot
json_t* pivot=json_object_get(json,"pivot");
	if(pivot!=NULL)
	{
		if(json_is_number(pivot))track_type->pivot=json_number_value(pivot)*TILE_SIZE;
		else
		{
		printf("Error: Property \"pivot\" not found or is not a number\n");
		return 1;
		}
	}else track_type->pivot=0;

//Load models
json_t* models=json_object_get(json,"models");
	if(models==NULL||!json_is_object(models))
	{
	printf("Error: Property \"models\" not found or is not an object\n");
	return 1;
	}
	
	if(load_model(&(track_type->mesh),models,"track"))return 1;
	if(load_model(&(track_type->mask),models,"mask"))
	{
	mesh_destroy(&(track_type->mesh));
	return 1;
	}

	if(track_type->flags&TRACK_HAS_LIFT)
	{
		if(load_model(&(track_type->lift_mesh),models,"lift"))
		{
		mesh_destroy(&(track_type->mesh));
		mesh_destroy(&(track_type->mask));
		return 1;
		}
	}

	if(track_type->flags&TRACK_SEPARATE_TIE)
	{
		if(load_model(&(track_type->tie_mesh),models,"tie"))
		{
		mesh_destroy(&(track_type->mesh));
		mesh_destroy(&(track_type->mask));
			if(track_type->flags&TRACK_HAS_LIFT)mesh_destroy(&(track_type->lift_mesh));
		return 1;
		}
	}

const char* support_model_names[NUM_MODELS]={"track_alt","track_tie","support_flat","support_bank_sixth","support_bank_third","support_bank_half","support_bank_two_thirds","support_bank_five_sixths","support_bank","support_base","brake","block_brake","booster","support_steep_to_vertical","support_vertical_to_steep","support_vertical","support_vertical_twist","support_barrel_roll","support_half_loop","support_quarter_loop","support_corkscrew","support_zero_g_roll","support_large_zero_g_roll"};

track_type->models_loaded=0;
	for(int i=0;i<NUM_MODELS;i++)
	{
	int result=load_model(&(track_type->models[i]),models,support_model_names[i]);
		if(result==0)track_type->models_loaded|=1<<i;
		else if(result==1)
		{
		mesh_destroy(&(track_type->mesh));
		mesh_destroy(&(track_type->mask));
			if(track_type->flags&TRACK_HAS_LIFT)mesh_destroy(&(track_type->lift_mesh));
			for(int j=0;j<i;j++)mesh_destroy(&(track_type->models[j]));
		return 1;
		}
	}

return 0;
}

int load_vector(vector3_t* vector,json_t* array)
{
int size=json_array_size(array);
	if(size!=3)
	{
	printf("Vector must have 3 components\n");
	return 1;
	}

json_t* x=json_array_get(array,0);
json_t* y=json_array_get(array,1);
json_t* z=json_array_get(array,2);

	if(!json_is_number(x)||!json_is_number(y)||!json_is_number(z))
	{
	printf("Vector components must be numeric\n");
	return 1;
	}
vector->x=json_number_value(x);
vector->y=json_number_value(y);
vector->z=json_number_value(z);
return 0;
}

int load_lights(light_t* lights,int* lights_count,json_t* json)
{
int num_lights=json_array_size(json);
	for(int i=0;i<num_lights;i++)
	{
	json_t* light=json_array_get(json,i);
	assert(light!=NULL);
		if(!json_is_object(light))
		{
		printf("Warning: Light array contains an element which is not an object - ignoring\n");
		continue;
		}
	
	json_t* type=json_object_get(light,"type");
		if(type==NULL||!json_is_string(type))
		{
		printf("Error: Property \"type\" not found or is not a string\n");
		return 1;
		}
	
	const char* type_value=json_string_value(type);
		if(strcmp(type_value,"diffuse")==0)lights[i].type=LIGHT_DIFFUSE;
		else if(strcmp(type_value,"specular")==0)lights[i].type=LIGHT_SPECULAR;
		else
		{
		printf("Unrecognized light type \"%s\"\n",type);
		free(lights);
		}

	json_t* shadow=json_object_get(light,"shadow");
		if(shadow==NULL||!json_is_boolean(shadow))
		{
		printf("Error: Property \"shadow\" not found or is not a boolean\n");
		return 1;
		}
		if(json_boolean_value(shadow))lights[i].shadow=1;
		else lights[i].shadow=0;

	json_t* direction=json_object_get(light,"direction");
		if(direction==NULL||!json_is_array(direction))
		{
		printf("Error: Property \"direction\" not found or is not a direction\n");
		return 1;
		}
		if(load_vector(&(lights[i].direction),direction))return 1;
	lights[i].direction=vector3_normalize(lights[i].direction);
	
	json_t* strength=json_object_get(light,"strength");
		if(strength==NULL||!json_is_number(strength))
		{
		printf("Error: Property \"strength\" not found or is not a number\n");
		return 1;
		}
	lights[i].intensity=json_number_value(strength);
	}
*lights_count=num_lights;
return 0;
}


int main(int argc,char** argv)
{

	if(argc!=2)
	{
	printf("Usage: TrackRender <file>\n");
	return 1;
	}
	
json_error_t error;
json_t* track=json_load_file(argv[1],0,&error);
	if(track==NULL)
	{
	printf("Error: %s at line %d column %d\n",error.text,error.line,error.column);
	return 1;
	}

const char* base_dir=NULL;
json_t* json_base_dir=json_object_get(track,"base_directory");
	if(json_base_dir!=NULL&&json_is_string(json_base_dir))base_dir=json_string_value(json_base_dir);
	else printf("Error: No property \"base_directory\" found\n");

const char* sprite_dir=NULL;
json_t* json_sprite_dir=json_object_get(track,"sprite_directory");
	if(json_sprite_dir!=NULL&&json_is_string(json_sprite_dir))sprite_dir=json_string_value(json_sprite_dir);
	else printf("Error: No property \"sprite_directory\" found\n");

const char* spritefile_in=NULL;
json_t* json_spritefile_in=json_object_get(track,"spritefile_in");
	if(json_spritefile_in!=NULL&&json_is_string(json_spritefile_in))spritefile_in=json_string_value(json_spritefile_in);
	else printf("Error: No property \"spritefile_in\" found\n");

const char* spritefile_out=NULL;
json_t* json_spritefile_out=json_object_get(track,"spritefile_out");
	if(json_spritefile_out!=NULL&&json_is_string(json_spritefile_out))spritefile_out=json_string_value(json_spritefile_out);
	else printf("Error: No property \"spritefile_out\" found\n");

int num_lights=9;
light_t lights[16]={
{LIGHT_DIFFUSE,0,vector3_normalize(vector3(0.0,-1.0,0.0)),0.25},
{LIGHT_DIFFUSE,0,vector3_normalize(vector3(1.0,0.3,0.0)),0.32},
{LIGHT_SPECULAR,0,vector3_normalize(vector3(1,1,-1)),1.0},
{LIGHT_DIFFUSE,0,vector3_normalize(vector3(1,0.65,-1)),0.8},
{LIGHT_DIFFUSE,0,vector3(0.0,1.0,0.0),0.174},
{LIGHT_DIFFUSE,0,vector3_normalize(vector3(-1.0,0.0,0.0)),0.15},
{LIGHT_DIFFUSE,0,vector3_normalize(vector3(0.0,1.0,1.0)),0.2},
{LIGHT_DIFFUSE,0,vector3_normalize(vector3(0.65,0.816,-0.65000000)),0.25},
{LIGHT_DIFFUSE,0,vector3_normalize(vector3(-1.0,0.0,-1.0)),0.25},
{0,0,{0,0,0},0},
{0,0,{0,0,0},0},
{0,0,{0,0,0},0},
{0,0,{0,0,0},0},
{0,0,{0,0,0},0},
{0,0,{0,0,0},0},
{0,0,{0,0,0},0}
};

json_t* light_array=json_object_get(track,"lights");
	if(light_array!=NULL)
	{
		if(!json_is_array(light_array))
		{
		printf("Error: Property \"lights\" is not an array\n");
		return 1;
		}
	if(load_lights(lights,&num_lights,light_array))return 1;
	}

track_type_t track_type;
	if(load_track_type(&track_type,track))return 1;

char full_path[256];
snprintf(full_path,256,"%s%s",base_dir,spritefile_in);
json_t* sprites=json_load_file(full_path,0,&error);
	if(sprites==NULL)
	{
	printf("Error: %s in file %s line %d column %d\n",error.text,error.source,error.line,error.column);
	return 1;
	}


context_t context=get_context(lights,num_lights);

write_track_type(&context,&track_type,sprites,base_dir,sprite_dir);

snprintf(full_path,256,"%s%s",base_dir,spritefile_out);
json_dump_file(sprites,full_path,JSON_INDENT(4));
context_destroy(&context);

return 0;
}

