#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <jansson.h>
#include <object.h>
#include <renderer.h>
#include <model.h>
#include <vectormath.h>
#include "track.h"
#include "objLoader/obj_parser.h"

int image_write_png(image_t* image,FILE* file);


void mesh_load_obj(mesh_t* mesh,char* filename)
{
    int i, j;
    obj_scene_data obj_data;
    if (!parse_obj_scene(&obj_data, filename))
    {
printf("Failed to load mesh\n");
exit(0);
}
    
    // Count vertices
    mesh->num_vertices = obj_data.vertex_count;
    mesh->num_normals = obj_data.vertex_normal_count;
    mesh->num_faces=obj_data.face_count;
	mesh->num_uvs=0;
	mesh->num_materials=3;

    // Allocate arrays
    mesh->vertices = malloc(mesh->num_vertices * sizeof(vector3_t));
    mesh->normals = malloc(mesh->num_normals * sizeof(vector3_t));
    mesh->faces = malloc(mesh->num_faces * sizeof(face_t));

	mesh->materials=malloc(3*sizeof(material_t));
	mesh->materials[1].flags=MATERIAL_IS_REMAPPABLE;
	mesh->materials[1].region=1;
	mesh->materials[1].specular_intensity=0.0;
	mesh->materials[1].specular_exponent=0.0;
	mesh->materials[1].color=vector3(0.6,0.6,0.6);
	mesh->materials[0].flags=MATERIAL_IS_REMAPPABLE;
	mesh->materials[0].region=2;
	mesh->materials[0].specular_intensity=0.0;
	mesh->materials[0].specular_exponent=0.0;
	mesh->materials[0].color=vector3(1.0,1.0,1.0);



    // Load vertices
    for (i = 0; i < mesh->num_vertices; i++) {
        mesh->vertices[i].x = obj_data.vertex_list[i]->e[0];
        mesh->vertices[i].y = obj_data.vertex_list[i]->e[1];
        mesh->vertices[i].z = obj_data.vertex_list[i]->e[2];
    }
    // Load vertices
    for (i = 0; i < mesh->num_normals; i++) {
        mesh->normals[i].x = obj_data.vertex_normal_list[i]->e[0];
        mesh->normals[i].y = obj_data.vertex_normal_list[i]->e[1];
        mesh->normals[i].z = obj_data.vertex_normal_list[i]->e[2];
    }
    // Load faces
    for (i = 0; i < obj_data.face_count; i++) 
	{
	mesh->faces[i].material= obj_data.face_list[i]->material_index;
  	    //obj_material* material = obj_data.material_list[obj_data.face_list[i]->material_index];
	//	mesh-
           // if (material->diff[0] < 0.3 && material->diff[1] > 0.7 && material->diff[2] < 0.3)

              	for (j = 0; j < 3; j++) 
		{
          	mesh->faces[i].vertices[j] = obj_data.face_list[i]->vertex_index[j];
            	mesh->faces[i].normals[j] = obj_data.face_list[i]->normal_index[j];
        	}
        } 

    delete_obj_data(&obj_data);
}

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
light_t light={vector3_normalize(vector3(1.0,-atan(3.141592658979/9.0),0.0)),0.25,0.72};
context_t context;
context_init(&context,light,palette_rct2(),TILE_SIZE);
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


void write_track_section(track_section_t* track_section,track_type_t* track_type,const char* filename,json_t* sprites)
{
context_t context=get_context();

	for(int angle=0;angle<4;angle++)
	{
	view_t* view=track_section->views+angle;
		for(int sprite=0;sprite<view->num_sprites;sprite++)
		{
		char final_filename[255];
		char relative_filename[255];
			if(track_section->views[angle].num_sprites==1)sprintf(relative_filename,"%s_%d.png",filename,angle+1);
			else sprintf(relative_filename,"%s_%d_%d.png",filename,angle+1,sprite+1);
		sprintf(final_filename,"/home/edward/Programming/OpenRCT2/resources/g2/%s",relative_filename);
		//sprintf(final_filename,"/home/edward/RCT2/Programming/TrackRender/%s",relative_filename);
		printf("%s\n",final_filename);
		image_t image;
		framebuffer_t framebuffer;
		framebuffer_render_track_section(&framebuffer,&context,track_section,track_type);
		image_from_framebuffer(&image,&framebuffer,&(context.palette));

			if(view->masks!=NULL)
			{
			printf("%d %d\n",image.offset_x,image.offset_y);
				for(int x=0;x<image.width;x++)
				for(int y=0;y<image.height;y++)
				{
					if(!is_in_mask(x+image.offset_x,y+image.offset_y,view->masks+sprite))
					{
					image.pixels[x+image.width*y]=0;
					}
				}
			image.offset_x+=view->masks[sprite].offset_x;
			image.offset_y+=view->masks[sprite].offset_y;
			}	
		FILE* file=fopen(final_filename,"w");
		image_write_png(&image,file);
	
		json_t* sprite_entry=json_object();
		json_object_set(sprite_entry,"path",json_string(relative_filename));
		json_object_set(sprite_entry,"x_offset",json_integer(image.offset_x));
		json_object_set(sprite_entry,"y_offset",json_integer(image.offset_y));
		json_array_append(sprites,sprite_entry);
		}
	context_rotate(&context);
	}
}






#define SQRT_2 1.4142135623731
#define SQRT1_2 0.707106781
#define SQRT_3 1.73205080757
#define SQRT_6 2.44948974278

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

int main(int argc,char** argv)
{
uint8_t* g1_data;
uint32_t g1_length;
	if(load_file("/home/edward/Programming/OpenRCT2/data/g1.dat",&g1_data,&g1_length))return 3;
image_list_t image_list;
error_t error=image_list_decode(&image_list,g1_data,g1_length);
	if(error!=ERROR_NONE)return 4;

track_type_t rmc;
mesh_load_obj(&(rmc.mesh),"rmc.obj");
rmc.length=TILE_SIZE*0.5;

track_type_t rmc_lift;
mesh_load_obj(&(rmc_lift.mesh),"rmc_lift.obj");
rmc_lift.length=TILE_SIZE*0.5;


json_t* sprites=json_load_file("/home/edward/Programming/OpenRCT2/resources/g2/sprites.json",0,NULL);


//Flat
write_track_section(&flat,&rmc,"track/rmc/flat",sprites);
write_track_section(&flat,&rmc_lift,"track/rmc/flat_lift",sprites);
write_track_section(&flat,&rmc,"track/rmc/brake",sprites);//TODO actual sprites for these
//Slopes
write_track_section(&flat_to_gentle_up,&rmc,"track/rmc/flat_to_gentle_up",sprites);
write_track_section(&flat_to_gentle_up,&rmc_lift,"track/rmc/flat_to_gentle_up_lift",sprites);
write_track_section(&gentle_up_to_flat,&rmc,"track/rmc/gentle_up_to_flat",sprites);
write_track_section(&gentle_up_to_flat,&rmc_lift,"track/rmc/gentle_up_to_flat_lift",sprites);
write_track_section(&gentle,&rmc,"track/rmc/gentle",sprites);
write_track_section(&gentle,&rmc_lift,"track/rmc/gentle_lift",sprites);
write_track_section(&gentle_to_steep_up,&rmc,"track/rmc/gentle_to_steep_up",sprites);
write_track_section(&gentle_to_steep_up,&rmc_lift,"track/rmc/gentle_to_steep_up_lift",sprites);
write_track_section(&steep_to_gentle_up,&rmc,"track/rmc/steep_to_gentle_up",sprites);
write_track_section(&steep_to_gentle_up,&rmc_lift,"track/rmc/steep_to_gentle_up_lift",sprites);
write_track_section(&steep,&rmc,"track/rmc/steep",sprites);
write_track_section(&steep,&rmc_lift,"track/rmc/steep_lift",sprites);

write_track_section(&steep_to_vertical_up,&rmc,"track/rmc/steep_to_vertical_up",sprites);
write_track_section(&steep_to_vertical_down,&rmc,"track/rmc/steep_to_vertical_down",sprites);
write_track_section(&vertical,&rmc,"track/rmc/vertical",sprites);



//Turns
write_track_section(&small_turn_left,&rmc,"track/rmc/small_turn_left",sprites);
write_track_section(&medium_turn_left,&rmc,"track/rmc/medium_turn_left",sprites);
write_track_section(&large_turn_left_to_diag,&rmc,"track/rmc/large_turn_left_to_diag",sprites);
write_track_section(&large_turn_right_to_diag,&rmc,"track/rmc/large_turn_right_to_diag",sprites);



//Banked turns

//Sloped turns

//write_track_section(&vertical_twist_left,&rmc,"track/rmc/vertical_twist_left",sprites);
//write_track_section(&vertical_twist_right,&rmc,"track/rmc/vertical_twist_right",sprites);

//Sloped banked turns


//Inversions
//write_track_section(&heartline_roll_left,&rmc,"track/rmc/heartline_roll",sprites);


json_dump_file(sprites,"/home/edward/Programming/OpenRCT2/resources/g2/sprites.json",JSON_INDENT(4));
return 0;

}


