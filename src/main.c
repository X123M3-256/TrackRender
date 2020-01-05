#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <jansson.h>
#include <renderer.h>
#include <model.h>
#include <image.h>
#include <vectormath.h>
#include "track.h"
#include "objLoader/obj_parser.h"


int load_file(const char* filename,uint8_t** data,uint32_t* length)
{
FILE* file=fopen(filename,"r");
	if(file==NULL)return 1;
fseek(file,0,SEEK_END);
*length=ftell(file);
fseek(file,0,SEEK_SET);
*data=malloc(*length);
	if(fread(*data,1,*length,file)!=*length)return 2;
fclose(file);
return 0;
}

context_t get_context()
{
light_t lights[6]={{LIGHT_HEMI,vector3(0.0,1.0,0.0),0.176},{LIGHT_DIFFUSE,vector3(0.671641,0.38733586252,-0.631561),0.9},{LIGHT_SPECULAR,vector3(0.671641,0.38733586252,-0.631561),0.3},{LIGHT_DIFFUSE,vector3(0.259037,0.03967,-0.965052),0.4},{LIGHT_DIFFUSE,vector3(-0.904066,0.412439,-0.112069),0.15},{LIGHT_DIFFUSE,vector3(0.50718,0.517623,0.689083),0.1}};

context_t context;
context_init(&context,lights,6,palette_rct2(),TILE_SIZE);
context.palette.colors[0].r=0;
context.palette.colors[0].g=0;
context.palette.colors[0].b=0;
return context;
}


int is_in_mask(int x,int y,mask_t* mask)
{
	for(int i=0;i<mask->num_rects;i++)
	{
		if(x>=mask->rects[i].x_lower&&x<mask->rects[i].x_upper&&y>=mask->rects[i].y_lower&&y<mask->rects[i].y_upper)return 1;
	}
return 0;
}
/*
void render_track_sprite(image_t* image,int angle,int sprite,track_section_t* track_section,track_type_t* track_type)
{
context_t context=get_context();
	for(int i=0;i<angle;i++)context_rotate(&context);

view_t* view=track_section->views+angle;
framebuffer_t framebuffer;
framebuffer_render_track_section(&framebuffer,&context,track_section,track_type,(track_section->flags&TRACK_EXTRUDE_BEHIND)&&(angle%2==0));
image_from_framebuffer(image,&framebuffer,&(context.palette));

	if(view->masks!=NULL)
	{
	printf("%d %d\n",image->x_offset,image->y_offset);
		for(int x=0;x<image->width;x++)
		for(int y=0;y<image->height;y++)
		{
			if(!is_in_mask(x+image->x_offset,y+image->y_offset,view->masks+sprite))
			{
			image->pixels[x+image->width*y]=0;
			}
		}
	image->x_offset+=view->masks[sprite].x_offset;
	image->y_offset+=view->masks[sprite].y_offset;
	}	
}
*/

void write_track_section(track_section_t* track_section,track_type_t* track_type,const char* filename,json_t* sprites)
{
context_t context=get_context();

	for(int angle=0;angle<4;angle++)
	{
		if(track_section->views[angle].num_sprites==0)continue;

	view_t* view=track_section->views+angle;

	image_t full_sprite;
	framebuffer_t framebuffer;
	framebuffer_render_track_section(&framebuffer,&context,track_section,track_type,(track_section->flags&TRACK_EXTRUDE_BEHIND)&&(angle%2==0),0);
	image_t track_mask;
		if(view->flags&VIEW_NEEDS_TRACK_MASK)
		{
		framebuffer_t mask_framebuffer;
		framebuffer_render_track_section(&mask_framebuffer,&context,track_section,track_type,0,1);
		image_from_framebuffer(&track_mask,&mask_framebuffer,&(context.palette));
		}
	image_from_framebuffer(&full_sprite,&framebuffer,&(context.palette));

		for(int sprite=0;sprite<view->num_sprites;sprite++)
		{
		char final_filename[255];
		char relative_filename[255];
			if(track_section->views[angle].num_sprites==1)sprintf(relative_filename,"%s_%d.png",filename,angle+1);
			else sprintf(relative_filename,"%s_%d_%d.png",filename,angle+1,sprite+1);
		sprintf(final_filename,"/home/edward/Programming/RCT2/OpenRCT2/resources/g2/%s",relative_filename);
		printf("%s\n",final_filename);

		image_t part_sprite;
		image_copy(&full_sprite,&part_sprite);

			if(view->masks!=NULL)
			{
				for(int x=0;x<full_sprite.width;x++)
				for(int y=0;y<full_sprite.height;y++)
				{
				int in_mask=is_in_mask(x+full_sprite.x_offset,y+full_sprite.y_offset,view->masks+sprite);

					if(view->masks[sprite].track_mask_op!=TRACK_MASK_NONE)
					{
					int mask_x=(x+full_sprite.x_offset)-track_mask.x_offset;
					int mask_y=(y+full_sprite.y_offset)-track_mask.y_offset;
					int in_track_mask=mask_x>=0&&mask_y>=0&&mask_x<track_mask.width&&mask_y<track_mask.height&&track_mask.pixels[mask_x+mask_y*track_mask.width]!=0;
						if(in_track_mask)part_sprite.pixels[x+part_sprite.width*y]=50;
						switch(view->masks[sprite].track_mask_op)
						{
						case TRACK_MASK_DIFFERENCE:
						in_mask=in_mask&&!in_track_mask;
						break;
						case TRACK_MASK_INTERSECT:
						in_mask=in_mask&&in_track_mask;
						break;
						case TRACK_MASK_UNION:
						in_mask=in_mask||in_track_mask;
						break;
						}
					}
					if(view->flags&VIEW_ENFORCE_NON_OVERLAPPING)
					{
						for(int i=0;i<sprite;i++)
						{
							if(is_in_mask(x+full_sprite.x_offset,y+full_sprite.y_offset,view->masks+i))in_mask=0;
						}
					}

					if(in_mask)
					{
					part_sprite.pixels[x+part_sprite.width*y]=full_sprite.pixels[x+full_sprite.width*y];
					}
					else
					{
					part_sprite.pixels[x+part_sprite.width*y]=0;
					}
				}
			part_sprite.x_offset+=view->masks[sprite].x_offset;
			part_sprite.y_offset+=view->masks[sprite].y_offset;
			}

	
		FILE* file=fopen(final_filename,"w");
			if(file==NULL)
			{
			printf("Error: could not open %s for writing\n",final_filename);
			exit(1);
			}
		image_write_png(&part_sprite,file);
		fclose(file);	

		json_t* sprite_entry=json_object();
		json_object_set(sprite_entry,"path",json_string(relative_filename));
		json_object_set(sprite_entry,"x_offset",json_integer(part_sprite.x_offset));
		json_object_set(sprite_entry,"y_offset",json_integer(part_sprite.y_offset));
		json_array_append(sprites,sprite_entry);
		image_destroy(&part_sprite);
		}

		if(view->flags&VIEW_NEEDS_TRACK_MASK)image_destroy(&track_mask);
	image_destroy(&full_sprite);
	context_rotate(&context);
	}
}



#define SQRT_2 1.4142135623731
#define SQRT1_2 0.707106781
#define SQRT_3 1.73205080757
#define SQRT_6 2.44948974278
/*
float srgb2linear2(float x)
{
	if (x<=0.04045)return x/12.92;
	else return pow((x+0.055)/1.055,2.4);
}
float luma_from_rgb(int r,int g,int b)
{
return 0.299*srgb2linear2(r/255.0)+0.587*srgb2linear2(g/255.0)+0.114*srgb2linear2(b/255.0);
}


float get_shaded_luma(float color,float ambient,float diffuse,float specular_intensity,float specular_exponent,vector3_t normal)
{
float projection[9]=
	{2.0	,0.0	 ,-2.0,
	 -1.0	,-SQRT_6 ,-1.0,
	 SQRT_3	,-SQRT_2 ,SQRT_3};
vector3_t light_direction=vector3_normalize(vector3(1.0,-atan(3.141592658979/9.0),0.0));

normal=vector3_normalize(matrix_vector(projection,normal));

float ambient_factor=ambient;
float diffuse_factor=diffuse*fmax(vector3_dot(normal,light_direction),0.0);	
//vector3_t reflected_light_direction=vector3_sub(vector3_mult(normal,2.0*vector3_dot(light_direction,normal)),light_direction);
//float specular_factor=specular_intensity*powf(fmax(0.0,diffuse_factor>0.0?-reflected_light_direction.z:0.0),specular_exponent);
return color*(ambient_factor+diffuse_factor);//+specular_factor;
}

float get_error(float color,float ambient,float diffuse,float specular_intensity,float specular_exponent)
{
float a=get_shaded_luma(color,ambient,diffuse,specular_intensity,specular_exponent,vector3(0,1.0,0))-luma_from_rgb(31,123,0);//Top
float b=get_shaded_luma(color,ambient,diffuse,specular_intensity,specular_exponent,vector3(0,0,-1.0))-luma_from_rgb(71,175,39);//Side
float c=get_shaded_luma(color,ambient,diffuse,specular_intensity,specular_exponent,vector3(-1.0,0,0))-luma_from_rgb(31,123,0);//Back
return a*a+b*b+c*c;
}
*/
int main(int argc,char** argv)
{
/*
track_type_t intamindouble;
mesh_load_obj(&(intamindouble.mesh),"models/intamindouble/intamindouble.obj");
mesh_load_obj(&(intamindouble.mask),"models/intamindouble/intamindouble_mask.obj");
intamindouble.length=TILE_SIZE*0.5;

track_type_t intamindouble_lift;
mesh_load_obj(&(intamindouble_lift.mesh),"models/intamindouble/intamindouble_lift.obj");
mesh_load_obj(&(intamindouble_lift.mask),"models/intamindouble/intamindouble_mask.obj");
intamindouble_lift.length=TILE_SIZE*0.5;
*/
/*
track_type_t intamin;
mesh_load_obj(&(intamin.mesh),"models/intamin.obj");
mesh_load_obj(&(intamin.mask),"models/intamindouble/intamindouble_mask.obj");
intamin.length=TILE_SIZE*0.5;
*/
/*
track_type_t rmc;
mesh_load_obj(&(rmc.mesh),"models/rmc/rmc.obj");
mesh_load_obj(&(rmc.mask),"models/rmc/rmc_mask.obj");
rmc.length=TILE_SIZE*0.5;

track_type_t rmc_lift;
mesh_load_obj(&(rmc_lift.mesh),"models/rmc/rmc_lift.obj");
mesh_load_obj(&(rmc_lift.mask),"models/rmc/rmc_mask.obj");
rmc_lift.length=TILE_SIZE*0.5;
*/

track_type_t raptor;
mesh_load_obj(&(raptor.mesh),"models/raptor/raptor.obj");
mesh_load_obj(&(raptor.mask),"models/raptor/raptor_mask.obj");
raptor.length=TILE_SIZE;

track_type_t raptor_lift;
mesh_load_obj(&(raptor_lift.mesh),"models/raptor/raptor_lift.obj");
mesh_load_obj(&(raptor_lift.mask),"models/raptor/raptor_mask.obj");
raptor_lift.length=TILE_SIZE;



json_t* sprites=json_load_file("/home/edward/Programming/RCT2/OpenRCT2/resources/g2/sprites_working.json",0,NULL);

/*write_track_section(&steep_to_vertical_up,&intamin,"track/intamin/steep_to_vertical_up",sprites);
write_track_section(&steep_to_vertical_down,&intamin,"track/intamin/steep_to_vertical_down",sprites);
write_track_section(&vertical,&intamin,"track/intamin/vertical",sprites);
write_track_section(&vertical_twist_left_up,&intamin,"track/intamin/vertical_twist_left_up",sprites);
write_track_section(&vertical_twist_right_up,&intamin,"track/intamin/vertical_twist_right_up",sprites);*/


//Flat
write_track_section(&flat,&raptor,"track/raptor/flat",sprites);
write_track_section(&flat,&raptor,"track/raptor/brake",sprites);//TODO actual sprites for these

//Slopes
write_track_section(&flat_to_gentle_up,&raptor,"track/raptor/flat_to_gentle_up",sprites);
write_track_section(&gentle_up_to_flat,&raptor,"track/raptor/gentle_to_flat_up",sprites);
write_track_section(&gentle,&raptor,"track/raptor/gentle",sprites);
write_track_section(&gentle_to_steep_up,&raptor,"track/raptor/gentle_to_steep_up",sprites);
write_track_section(&steep_to_gentle_up,&raptor,"track/raptor/steep_to_gentle_up",sprites);
write_track_section(&steep,&raptor,"track/raptor/steep",sprites);
write_track_section(&steep_to_vertical_up,&raptor,"track/raptor/steep_to_vertical_up",sprites);
write_track_section(&vertical_to_steep_up,&raptor,"track/raptor/vertical_to_steep_up",sprites);
write_track_section(&vertical,&raptor,"track/raptor/vertical",sprites);

//Turns
write_track_section(&small_turn_left,&raptor,"track/raptor/small_turn_left",sprites);
write_track_section(&medium_turn_left,&raptor,"track/raptor/medium_turn_left",sprites);
write_track_section(&large_turn_left_to_diag,&raptor,"track/raptor/large_turn_left_to_diag",sprites);
write_track_section(&large_turn_right_to_diag,&raptor,"track/raptor/large_turn_right_to_diag",sprites);

//Diagonals
write_track_section(&flat_diag,&raptor,"track/raptor/flat_diag",sprites);
write_track_section(&flat_to_gentle_up_diag,&raptor,"track/raptor/flat_to_gentle_up_diag",sprites);
write_track_section(&gentle_to_flat_up_diag,&raptor,"track/raptor/gentle_to_flat_up_diag",sprites);
write_track_section(&gentle_diag,&raptor,"track/raptor/gentle_diag",sprites);
write_track_section(&gentle_to_steep_up_diag,&raptor,"track/raptor/gentle_to_steep_up_diag",sprites);
write_track_section(&steep_to_gentle_up_diag,&raptor,"track/raptor/steep_to_gentle_up_diag",sprites);
write_track_section(&steep_diag,&raptor,"track/raptor/steep_diag",sprites);

//Banked turns
write_track_section(&flat_to_left_bank,&raptor,"track/raptor/flat_to_left_bank",sprites);
write_track_section(&flat_to_right_bank,&raptor,"track/raptor/flat_to_right_bank",sprites);
write_track_section(&left_bank_to_gentle_up,&raptor,"track/raptor/left_bank_to_gentle_up",sprites);
write_track_section(&right_bank_to_gentle_up,&raptor,"track/raptor/right_bank_to_gentle_up",sprites);
write_track_section(&gentle_up_to_left_bank,&raptor,"track/raptor/gentle_up_to_left_bank",sprites);
write_track_section(&gentle_up_to_right_bank,&raptor,"track/raptor/gentle_up_to_right_bank",sprites);
write_track_section(&left_bank,&raptor,"track/raptor/left_bank",sprites);
write_track_section(&flat_to_left_bank_diag,&raptor,"track/raptor/flat_to_left_bank_diag",sprites);
write_track_section(&flat_to_right_bank_diag,&raptor,"track/raptor/flat_to_right_bank_diag",sprites);
write_track_section(&left_bank_to_gentle_up_diag,&raptor,"track/raptor/left_bank_to_gentle_up_diag",sprites);
write_track_section(&right_bank_to_gentle_up_diag,&raptor,"track/raptor/right_bank_to_gentle_up_diag",sprites);
write_track_section(&gentle_up_to_left_bank_diag,&raptor,"track/raptor/gentle_up_to_left_bank_diag",sprites);
write_track_section(&gentle_up_to_right_bank_diag,&raptor,"track/raptor/gentle_up_to_right_bank_diag",sprites);
write_track_section(&left_bank_diag,&raptor,"track/raptor/left_bank_diag",sprites);
write_track_section(&small_turn_left_bank,&raptor,"track/raptor/small_turn_left_bank",sprites);
write_track_section(&medium_turn_left_bank,&raptor,"track/raptor/medium_turn_left_bank",sprites);
write_track_section(&large_turn_left_to_diag_bank,&raptor,"track/raptor/large_turn_left_to_diag_bank",sprites);
write_track_section(&large_turn_right_to_diag_bank,&raptor,"track/raptor/large_turn_right_to_diag_bank",sprites);


//Sloped turns
write_track_section(&small_turn_left_gentle_up,&raptor,"track/raptor/small_turn_left_gentle_up",sprites);
write_track_section(&small_turn_right_gentle_up,&raptor,"track/raptor/small_turn_right_gentle_up",sprites);
write_track_section(&medium_turn_left_gentle_up,&raptor,"track/raptor/medium_turn_left_gentle_up",sprites);
write_track_section(&medium_turn_right_gentle_up,&raptor,"track/raptor/medium_turn_right_gentle_up",sprites);
write_track_section(&very_small_turn_left_steep_up,&raptor,"track/raptor/very_small_turn_left_steep_up",sprites);
write_track_section(&very_small_turn_right_steep_up,&raptor,"track/raptor/very_small_turn_right_steep_up",sprites);
write_track_section(&vertical_twist_left_up,&raptor,"track/raptor/vertical_twist_left_up",sprites);
write_track_section(&vertical_twist_right_up,&raptor,"track/raptor/vertical_twist_right_up",sprites);


//Sloped banked turns
write_track_section(&gentle_up_to_gentle_up_left_bank,&raptor,"track/raptor/gentle_up_to_gentle_up_left_bank",sprites);
write_track_section(&gentle_up_to_gentle_up_right_bank,&raptor,"track/raptor/gentle_up_to_gentle_up_right_bank",sprites);
write_track_section(&gentle_up_left_bank_to_gentle_up,&raptor,"track/raptor/gentle_up_left_bank_to_gentle_up",sprites);
write_track_section(&gentle_up_right_bank_to_gentle_up,&raptor,"track/raptor/gentle_up_right_bank_to_gentle_up",sprites);
write_track_section(&left_bank_to_gentle_up_left_bank,&raptor,"track/raptor/left_bank_to_gentle_up_left_bank",sprites);
write_track_section(&right_bank_to_gentle_up_right_bank,&raptor,"track/raptor/right_bank_to_gentle_up_right_bank",sprites);
write_track_section(&gentle_up_left_bank_to_left_bank,&raptor,"track/raptor/gentle_up_left_bank_to_left_bank",sprites);
write_track_section(&gentle_up_right_bank_to_right_bank,&raptor,"track/raptor/gentle_up_right_bank_to_right_bank",sprites);
write_track_section(&gentle_up_left_bank,&raptor,"track/raptor/gentle_up_left_bank",sprites);
write_track_section(&gentle_up_right_bank,&raptor,"track/raptor/gentle_up_right_bank",sprites);
write_track_section(&flat_to_gentle_up_left_bank,&raptor,"track/raptor/flat_to_gentle_up_left_bank",sprites);
write_track_section(&flat_to_gentle_up_right_bank,&raptor,"track/raptor/flat_to_gentle_up_right_bank",sprites);
write_track_section(&gentle_up_left_bank_to_flat,&raptor,"track/raptor/gentle_up_left_bank_to_flat",sprites);
write_track_section(&gentle_up_right_bank_to_flat,&raptor,"track/raptor/gentle_up_right_bank_to_flat",sprites);
write_track_section(&small_turn_left_bank_gentle_up,&raptor,"track/raptor/small_turn_left_bank_gentle_up",sprites);
write_track_section(&small_turn_right_bank_gentle_up,&raptor,"track/raptor/small_turn_right_bank_gentle_up",sprites);
write_track_section(&medium_turn_left_bank_gentle_up,&raptor,"track/raptor/medium_turn_left_bank_gentle_up",sprites);
write_track_section(&medium_turn_right_bank_gentle_up,&raptor,"track/raptor/medium_turn_right_bank_gentle_up",sprites);

//Miscellaneous
write_track_section(&s_bend_left,&raptor,"track/raptor/s_bend_left",sprites);
write_track_section(&s_bend_right,&raptor,"track/raptor/s_bend_right",sprites);
write_track_section(&small_helix_left_up,&raptor,"track/raptor/small_helix_left_up",sprites);
write_track_section(&small_helix_right_up,&raptor,"track/raptor/small_helix_right_up",sprites);
write_track_section(&medium_helix_left_up,&raptor,"track/raptor/medium_helix_left_up",sprites);
write_track_section(&medium_helix_right_up,&raptor,"track/raptor/medium_helix_right_up",sprites);


//Lift pieces

write_track_section(&flat,&raptor_lift,"track/raptor/flat_lift",sprites); 
write_track_section(&flat_to_gentle_up,&raptor_lift,"track/raptor/flat_to_gentle_up_lift",sprites);
write_track_section(&gentle_up_to_flat,&raptor_lift,"track/raptor/gentle_to_flat_up_lift",sprites);
write_track_section(&gentle,&raptor_lift,"track/raptor/gentle_lift",sprites);
write_track_section(&gentle_to_steep_up,&raptor_lift,"track/raptor/gentle_to_steep_up_lift",sprites);
write_track_section(&steep_to_gentle_up,&raptor_lift,"track/raptor/steep_to_gentle_up_lift",sprites);
write_track_section(&steep,&raptor_lift,"track/raptor/steep_lift",sprites);

write_track_section(&flat_diag,&raptor_lift,"track/raptor/flat_diag_lift",sprites);
write_track_section(&flat_to_gentle_up_diag,&raptor_lift,"track/raptor/flat_to_gentle_up_diag_lift",sprites);
write_track_section(&gentle_to_flat_up_diag,&raptor_lift,"track/raptor/gentle_to_flat_up_diag_lift",sprites);
write_track_section(&gentle_diag,&raptor_lift,"track/raptor/gentle_diag_lift",sprites);
write_track_section(&gentle_to_steep_up_diag,&raptor_lift,"track/raptor/gentle_to_steep_up_diag_lift",sprites);
write_track_section(&steep_to_gentle_up_diag,&raptor_lift,"track/raptor/steep_to_gentle_up_diag_lift",sprites);
write_track_section(&steep_diag,&raptor_lift,"track/raptor/steep_diag_lift",sprites);

//Inversions
write_track_section(&barrel_roll_left,&raptor,"track/raptor/barrel_roll_left",sprites);
write_track_section(&barrel_roll_right,&raptor,"track/raptor/barrel_roll_right",sprites);
write_track_section(&half_loop,&raptor,"track/raptor/half_loop",sprites);

write_track_section(&flat_to_steep_up,&raptor,"track/raptor/flat_to_steep_up",sprites);
write_track_section(&steep_to_flat_up,&raptor,"track/raptor/steep_to_flat_up",sprites);
write_track_section(&quarter_loop_up,&raptor,"track/raptor/quarter_loop_up",sprites);

json_dump_file(sprites,"/home/edward/Programming/RCT2/OpenRCT2/resources/g2/sprites.json",JSON_INDENT(4));

return 0;
}


