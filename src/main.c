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


context_t get_context(light_t* lights,uint32_t num_lights)
{
context_t context;
context_init(&context,lights,num_lights,palette_rct2(),1.5*sqrt(6));
//context.palette.colors[0].r=0;
//context.palette.colors[0].g=0;
//context.palette.colors[0].b=0;
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

void write_track_section(context_t* context,track_section_t* track_section,track_type_t* track_type,const char* filename,json_t* sprites)
{
image_t full_sprites[4];
	if(track_section->flags&TRACK_EXTRUDE_BEHIND)
	{
	printf("Test\n");
	render_track_section(context,track_section,track_type,1,0,0x5,full_sprites);
	render_track_section(context,track_section,track_type,0,0,0xA,full_sprites);
	}
	else render_track_section(context,track_section,track_type,0,0,0xF,full_sprites);

image_t track_masks[4];
int track_mask_views=0;
	for(int i=0;i<4;i++)track_mask_views|=(track_section->views[i].flags&VIEW_NEEDS_TRACK_MASK?1:0)<<i;
	if(track_mask_views!=0)render_track_section(context,track_section,track_type,0,1,track_mask_views,track_masks);

	for(int angle=0;angle<4;angle++)
	{
		if(track_section->views[angle].num_sprites==0)continue;

	view_t* view=track_section->views+angle;

		
		for(int sprite=0;sprite<view->num_sprites;sprite++)
		{
		char final_filename[255];
		char relative_filename[255];
			if(view->num_sprites==1)sprintf(relative_filename,"%s_%d.png",filename,angle+1);
			else sprintf(relative_filename,"%s_%d_%d.png",filename,angle+1,sprite+1);
		sprintf(final_filename,"/home/edward/RCT2Prog/OpenRCT2/resources/g2/%s",relative_filename);
		printf("%s\n",final_filename);

		image_t part_sprite;
		image_copy(full_sprites+angle,&part_sprite);

			if(view->masks!=NULL)
			{
				for(int x=0;x<full_sprites[angle].width;x++)
				for(int y=0;y<full_sprites[angle].height;y++)
				{
				int in_mask=is_in_mask(x+full_sprites[angle].x_offset,y+full_sprites[angle].y_offset,view->masks+sprite);

					if(view->masks[sprite].track_mask_op!=TRACK_MASK_NONE)
					{
					int mask_x=(x+full_sprites[angle].x_offset)-track_masks[angle].x_offset;
					int mask_y=(y+full_sprites[angle].y_offset)-track_masks[angle].y_offset;
					int in_track_mask=mask_x>=0&&mask_y>=0&&mask_x<track_masks[angle].width&&mask_y<track_masks[angle].height&&track_masks[angle].pixels[mask_x+mask_y*track_masks[angle].width]!=0;
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
							if(is_in_mask(x+full_sprites[angle].x_offset,y+full_sprites[angle].y_offset,view->masks+i))in_mask=0;
						}
					}

					if(in_mask)
					{
					part_sprite.pixels[x+part_sprite.width*y]=full_sprites[angle].pixels[x+full_sprites[angle].width*y];
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

		if(view->flags&VIEW_NEEDS_TRACK_MASK)image_destroy(track_masks+angle);
	image_destroy(full_sprites+angle);
	}
}




int main(int argc,char** argv)
{
light_t lights[6]={
{LIGHT_SPECULAR,vector3(0.671641,0.38733586252,-0.631561),0.35},
{LIGHT_DIFFUSE,vector3(0.0,1.0,0.0),0.044},
{LIGHT_DIFFUSE,vector3_normalize(vector3(-1.0,1.0,0.0)),0.19},
{LIGHT_DIFFUSE,vector3_normalize(vector3(0.0,1.0,1.0)),0.04},
{LIGHT_DIFFUSE,vector3_normalize(vector3(0.5,0.816,-0.5000000)),0.17},
{LIGHT_DIFFUSE,vector3_normalize(vector3(-1.0,1.0,-1.0)),0.045},
};

//light_t lights[6]={{LIGHT_HEMI,{0.0,1.0,0.0},0.176},{LIGHT_DIFFUSE,{0.671641,0.38733586252,-0.631561},0.9},{LIGHT_SPECULAR,{0.671641,0.38733586252,-0.631561},0.3},{LIGHT_DIFFUSE,{0.259037,0.03967,-0.965052},0.4},{LIGHT_DIFFUSE,{-0.904066,0.412439,-0.112069},0.15},{LIGHT_DIFFUSE,{0.50718,0.517623,0.689083},0.1}};
context_t context=get_context(lights,6);

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
track_type_t rmc;
rmc.flags=TRACK_HAS_SUPPORTS;
mesh_load(&(rmc.mesh),"rmc.obj");
mesh_load(&(rmc.mask),"rmc_mask.obj");
mesh_load(&(rmc.supports[SUPPORT_FLAT]),"rmc_support.obj");
mesh_load(&(rmc.supports[SUPPORT_BANK_HALF]),"rmc_support_bank_half.obj");
mesh_load(&(rmc.supports[SUPPORT_BANK]),"rmc_support_bank.obj");
mesh_load(&(rmc.supports[SUPPORT_BASE]),"rmc_support_base.obj");
mesh_load(&(rmc.supports[SUPPORT_SPECIAL_VERTICAL]),"rmc_support_vertical.obj");
mesh_load(&(rmc.supports[SUPPORT_SPECIAL_STEEP_TO_VERTICAL]),"rmc_support_steep_to_vertical.obj");
mesh_load(&(rmc.supports[SUPPORT_SPECIAL_VERTICAL_TO_STEEP]),"rmc_support_vertical_to_steep.obj");
mesh_load(&(rmc.supports[SUPPORT_SPECIAL_VERTICAL_TWIST]),"rmc_support_vertical_twist.obj");
rmc.length=TILE_SIZE*0.5;

track_type_t rmc_lift;
rmc_lift.flags=TRACK_HAS_SUPPORTS;
mesh_load(&(rmc_lift.mesh),"rmc_lift.obj");
mesh_load(&(rmc_lift.mask),"rmc_mask.obj");
mesh_load(&(rmc_lift.supports[SUPPORT_FLAT]),"rmc_support.obj");
mesh_load(&(rmc_lift.supports[SUPPORT_BANK_HALF]),"rmc_support_bank_half.obj");
mesh_load(&(rmc_lift.supports[SUPPORT_BANK]),"rmc_support_bank.obj");
mesh_load(&(rmc_lift.supports[SUPPORT_BASE]),"rmc_support_base.obj");
rmc_lift.length=TILE_SIZE*0.5;
/*
track_type_t raptor;
mesh_load(&(raptor.mesh),"raptor.obj");
mesh_load(&(raptor.mask),"raptor_mask.obj");
raptor.length=0.5*TILE_SIZE;

track_type_t raptor_lift;
mesh_load(&(raptor_lift.mesh),"raptor_lift.obj");
mesh_load(&(raptor_lift.mask),"raptor_mask.obj");
raptor_lift.length=0.5*TILE_SIZE;
*/

json_t* sprites=json_load_file("/home/edward/RCT2Prog/OpenRCT2/resources/g2/sprites_old.json",0,NULL);

/*write_track_section(&steep_to_vertical_up,&intamin,"track/intamin/steep_to_vertical_up",sprites);
write_track_section(&steep_to_vertical_down,&intamin,"track/intamin/steep_to_vertical_down",sprites);
write_track_section(&vertical,&intamin,"track/intamin/vertical",sprites);
write_track_section(&vertical_twist_left_up,&intamin,"track/intamin/vertical_twist_left_up",sprites);
write_track_section(&vertical_twist_right_up,&intamin,"track/intamin/vertical_twist_right_up",sprites);*/

//Flat
write_track_section(&context,&flat,&rmc,"track/rmc/flat",sprites);
write_track_section(&context,&flat,&rmc_lift,"track/rmc/brake",sprites);//TODO actual sprites for these

//Slopes
write_track_section(&context,&flat_to_gentle_up,&rmc,"track/rmc/flat_to_gentle_up",sprites);
write_track_section(&context,&gentle_up_to_flat,&rmc,"track/rmc/gentle_to_flat_up",sprites);
write_track_section(&context,&gentle,&rmc,"track/rmc/gentle",sprites);
write_track_section(&context,&gentle_to_steep_up,&rmc,"track/rmc/gentle_to_steep_up",sprites);
write_track_section(&context,&steep_to_gentle_up,&rmc,"track/rmc/steep_to_gentle_up",sprites);

write_track_section(&context,&steep,&rmc,"track/rmc/steep",sprites);
write_track_section(&context,&steep_to_vertical_up,&rmc,"track/rmc/steep_to_vertical_up",sprites);
write_track_section(&context,&vertical_to_steep_up,&rmc,"track/rmc/vertical_to_steep_up",sprites);
write_track_section(&context,&vertical,&rmc,"track/rmc/vertical",sprites);

//Turns
write_track_section(&context,&small_turn_left,&rmc,"track/rmc/small_turn_left",sprites);
write_track_section(&context,&medium_turn_left,&rmc,"track/rmc/medium_turn_left",sprites);
write_track_section(&context,&large_turn_left_to_diag,&rmc,"track/rmc/large_turn_left_to_diag",sprites);
write_track_section(&context,&large_turn_right_to_diag,&rmc,"track/rmc/large_turn_right_to_diag",sprites);

//Diagonals
write_track_section(&context,&flat_diag,&rmc,"track/rmc/flat_diag",sprites);
write_track_section(&context,&flat_to_gentle_up_diag,&rmc,"track/rmc/flat_to_gentle_up_diag",sprites);
write_track_section(&context,&gentle_to_flat_up_diag,&rmc,"track/rmc/gentle_to_flat_up_diag",sprites);
write_track_section(&context,&gentle_diag,&rmc,"track/rmc/gentle_diag",sprites);
write_track_section(&context,&gentle_to_steep_up_diag,&rmc,"track/rmc/gentle_to_steep_up_diag",sprites);
write_track_section(&context,&steep_to_gentle_up_diag,&rmc,"track/rmc/steep_to_gentle_up_diag",sprites);
write_track_section(&context,&steep_diag,&rmc,"track/rmc/steep_diag",sprites);

//Banked turns
write_track_section(&context,&flat_to_left_bank,&rmc,"track/rmc/flat_to_left_bank",sprites);
write_track_section(&context,&flat_to_right_bank,&rmc,"track/rmc/flat_to_right_bank",sprites);
write_track_section(&context,&left_bank_to_gentle_up,&rmc,"track/rmc/left_bank_to_gentle_up",sprites);
write_track_section(&context,&right_bank_to_gentle_up,&rmc,"track/rmc/right_bank_to_gentle_up",sprites);
write_track_section(&context,&gentle_up_to_left_bank,&rmc,"track/rmc/gentle_up_to_left_bank",sprites);
write_track_section(&context,&gentle_up_to_right_bank,&rmc,"track/rmc/gentle_up_to_right_bank",sprites);
write_track_section(&context,&left_bank,&rmc,"track/rmc/left_bank",sprites);
write_track_section(&context,&flat_to_left_bank_diag,&rmc,"track/rmc/flat_to_left_bank_diag",sprites);
write_track_section(&context,&flat_to_right_bank_diag,&rmc,"track/rmc/flat_to_right_bank_diag",sprites);
write_track_section(&context,&left_bank_to_gentle_up_diag,&rmc,"track/rmc/left_bank_to_gentle_up_diag",sprites);
write_track_section(&context,&right_bank_to_gentle_up_diag,&rmc,"track/rmc/right_bank_to_gentle_up_diag",sprites);
write_track_section(&context,&gentle_up_to_left_bank_diag,&rmc,"track/rmc/gentle_up_to_left_bank_diag",sprites);
write_track_section(&context,&gentle_up_to_right_bank_diag,&rmc,"track/rmc/gentle_up_to_right_bank_diag",sprites);
write_track_section(&context,&left_bank_diag,&rmc,"track/rmc/left_bank_diag",sprites);
write_track_section(&context,&small_turn_left_bank,&rmc,"track/rmc/small_turn_left_bank",sprites);
write_track_section(&context,&medium_turn_left_bank,&rmc,"track/rmc/medium_turn_left_bank",sprites);
write_track_section(&context,&large_turn_left_to_diag_bank,&rmc,"track/rmc/large_turn_left_to_diag_bank",sprites);
write_track_section(&context,&large_turn_right_to_diag_bank,&rmc,"track/rmc/large_turn_right_to_diag_bank",sprites);

//Sloped turns
write_track_section(&context,&small_turn_left_gentle_up,&rmc,"track/rmc/small_turn_left_gentle_up",sprites);
write_track_section(&context,&small_turn_right_gentle_up,&rmc,"track/rmc/small_turn_right_gentle_up",sprites);
write_track_section(&context,&medium_turn_left_gentle_up,&rmc,"track/rmc/medium_turn_left_gentle_up",sprites);
write_track_section(&context,&medium_turn_right_gentle_up,&rmc,"track/rmc/medium_turn_right_gentle_up",sprites);
write_track_section(&context,&very_small_turn_left_steep_up,&rmc,"track/rmc/very_small_turn_left_steep_up",sprites);
write_track_section(&context,&very_small_turn_right_steep_up,&rmc,"track/rmc/very_small_turn_right_steep_up",sprites);
write_track_section(&context,&vertical_twist_left_up,&rmc,"track/rmc/vertical_twist_left_up",sprites);
write_track_section(&context,&vertical_twist_right_up,&rmc,"track/rmc/vertical_twist_right_up",sprites);


//Sloped banked turns
write_track_section(&context,&gentle_up_to_gentle_up_left_bank,&rmc,"track/rmc/gentle_up_to_gentle_up_left_bank",sprites);
write_track_section(&context,&gentle_up_to_gentle_up_right_bank,&rmc,"track/rmc/gentle_up_to_gentle_up_right_bank",sprites);
write_track_section(&context,&gentle_up_left_bank_to_gentle_up,&rmc,"track/rmc/gentle_up_left_bank_to_gentle_up",sprites);
write_track_section(&context,&gentle_up_right_bank_to_gentle_up,&rmc,"track/rmc/gentle_up_right_bank_to_gentle_up",sprites);
write_track_section(&context,&left_bank_to_gentle_up_left_bank,&rmc,"track/rmc/left_bank_to_gentle_up_left_bank",sprites);
write_track_section(&context,&right_bank_to_gentle_up_right_bank,&rmc,"track/rmc/right_bank_to_gentle_up_right_bank",sprites);
write_track_section(&context,&gentle_up_left_bank_to_left_bank,&rmc,"track/rmc/gentle_up_left_bank_to_left_bank",sprites);
write_track_section(&context,&gentle_up_right_bank_to_right_bank,&rmc,"track/rmc/gentle_up_right_bank_to_right_bank",sprites);
write_track_section(&context,&gentle_up_left_bank,&rmc,"track/rmc/gentle_up_left_bank",sprites);
write_track_section(&context,&gentle_up_right_bank,&rmc,"track/rmc/gentle_up_right_bank",sprites);
write_track_section(&context,&flat_to_gentle_up_left_bank,&rmc,"track/rmc/flat_to_gentle_up_left_bank",sprites);
write_track_section(&context,&flat_to_gentle_up_right_bank,&rmc,"track/rmc/flat_to_gentle_up_right_bank",sprites);
write_track_section(&context,&gentle_up_left_bank_to_flat,&rmc,"track/rmc/gentle_up_left_bank_to_flat",sprites);
write_track_section(&context,&gentle_up_right_bank_to_flat,&rmc,"track/rmc/gentle_up_right_bank_to_flat",sprites);
write_track_section(&context,&small_turn_left_bank_gentle_up,&rmc,"track/rmc/small_turn_left_bank_gentle_up",sprites);
write_track_section(&context,&small_turn_right_bank_gentle_up,&rmc,"track/rmc/small_turn_right_bank_gentle_up",sprites);
write_track_section(&context,&medium_turn_left_bank_gentle_up,&rmc,"track/rmc/medium_turn_left_bank_gentle_up",sprites);
write_track_section(&context,&medium_turn_right_bank_gentle_up,&rmc,"track/rmc/medium_turn_right_bank_gentle_up",sprites);

//Miscellaneous
write_track_section(&context,&s_bend_left,&rmc,"track/rmc/s_bend_left",sprites);
write_track_section(&context,&s_bend_right,&rmc,"track/rmc/s_bend_right",sprites);
write_track_section(&context,&small_helix_left_up,&rmc,"track/rmc/small_helix_left_up",sprites);
write_track_section(&context,&small_helix_right_up,&rmc,"track/rmc/small_helix_right_up",sprites);
write_track_section(&context,&medium_helix_left_up,&rmc,"track/rmc/medium_helix_left_up",sprites);
write_track_section(&context,&medium_helix_right_up,&rmc,"track/rmc/medium_helix_right_up",sprites);


//Lift pieces
write_track_section(&context,&flat,&rmc_lift,"track/rmc/flat_lift",sprites); 
write_track_section(&context,&flat_to_gentle_up,&rmc_lift,"track/rmc/flat_to_gentle_up_lift",sprites);
write_track_section(&context,&gentle_up_to_flat,&rmc_lift,"track/rmc/gentle_to_flat_up_lift",sprites);
write_track_section(&context,&gentle,&rmc_lift,"track/rmc/gentle_lift",sprites);
write_track_section(&context,&gentle_to_steep_up,&rmc_lift,"track/rmc/gentle_to_steep_up_lift",sprites);
write_track_section(&context,&steep_to_gentle_up,&rmc_lift,"track/rmc/steep_to_gentle_up_lift",sprites);
write_track_section(&context,&steep,&rmc_lift,"track/rmc/steep_lift",sprites);

write_track_section(&context,&flat_diag,&rmc_lift,"track/rmc/flat_diag_lift",sprites);
write_track_section(&context,&flat_to_gentle_up_diag,&rmc_lift,"track/rmc/flat_to_gentle_up_diag_lift",sprites);
write_track_section(&context,&gentle_to_flat_up_diag,&rmc_lift,"track/rmc/gentle_to_flat_up_diag_lift",sprites);
write_track_section(&context,&gentle_diag,&rmc_lift,"track/rmc/gentle_diag_lift",sprites);
write_track_section(&context,&gentle_to_steep_up_diag,&rmc_lift,"track/rmc/gentle_to_steep_up_diag_lift",sprites);
write_track_section(&context,&steep_to_gentle_up_diag,&rmc_lift,"track/rmc/steep_to_gentle_up_diag_lift",sprites);
write_track_section(&context,&steep_diag,&rmc_lift,"track/rmc/steep_diag_lift",sprites);

//Inversions
write_track_section(&context,&barrel_roll_left,&rmc,"track/rmc/barrel_roll_left",sprites);
write_track_section(&context,&barrel_roll_right,&rmc,"track/rmc/barrel_roll_right",sprites);
write_track_section(&context,&half_loop,&rmc,"track/rmc/half_loop",sprites);

write_track_section(&context,&flat_to_steep_up,&rmc,"track/rmc/flat_to_steep_up",sprites);
write_track_section(&context,&steep_to_flat_up,&rmc,"track/rmc/steep_to_flat_up",sprites);
write_track_section(&context,&quarter_loop_up,&rmc,"track/rmc/quarter_loop_up",sprites);

json_dump_file(sprites,"/home/edward/RCT2Prog/OpenRCT2/resources/g2/sprites.json",JSON_INDENT(4));

context_destroy(&context);
return 0;
}

