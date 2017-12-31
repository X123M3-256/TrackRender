#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "track.h"

vector3_t change_coordinates(vector3_t a)
{
vector3_t result={a.z,a.y,a.x};
return result;
}

void track_transform_primitives(track_point_t (*track_curve)(float distance),primitive_t* primitives,uint32_t num_primitives,int flags)
{
	for(int i=0;i<num_primitives;i++)
	{
		for(int j=0;j<3;j++)
		{
		track_point_t track_point=track_curve(primitives[i].vertices[j].z);
			if(flags&TRACK_DIAGONAL)track_point.position.x+=0.75*sqrt(6);
		track_point.position.y-=0.75;
			if(!(flags&TRACK_VERTICAL))track_point.position.z-=0.75*sqrt(6);
		primitives[i].vertices[j]=change_coordinates(vector3_add(track_point.position,vector3_add(vector3_mult(track_point.normal,primitives[i].vertices[j].y),vector3_mult(track_point.binormal,primitives[i].vertices[j].x))));
		primitives[i].normals[j]=change_coordinates(vector3_add(vector3_mult(track_point.tangent,primitives[i].normals[j].z),vector3_add(vector3_mult(track_point.normal,primitives[i].normals[j].y),vector3_mult(track_point.binormal,primitives[i].normals[j].x))));
		}
	}
}



void framebuffer_render_track_section(framebuffer_t* framebuffer,context_t* context,track_section_t* track_section,track_type_t* track_type,int extrude_behind,int track_mask)
{
float num_meshes=(int)floor(0.5+track_section->length/track_type->length);
float scale=track_section->length/(num_meshes*track_type->length);
float length=scale*track_type->length;

	if(extrude_behind)num_meshes++;//TODO make extruded section straight

uint32_t primitives_per_mesh=mesh_count_primitives(&(track_type->mesh));

	if(track_mask)primitives_per_mesh+=16;

material_t blank_mat;
blank_mat.flags=0;
blank_mat.region=0;
blank_mat.specular_intensity=0.0;
blank_mat.specular_exponent=0.0;
blank_mat.color=vector3(0.6,0.6,0.6);


primitive_t* primitives=malloc(((size_t)num_meshes)*primitives_per_mesh*sizeof(primitive_t));
	for(int i=0;i<num_meshes;i++)
	{
	transform_t transform={{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,scale},{0.0,0.0,(i-(extrude_behind?1:0))*length}};

	mesh_get_primitives(&(track_type->mesh),primitives+i*primitives_per_mesh);
	
		if(track_mask)
		{
		int base=i*primitives_per_mesh+(primitives_per_mesh-16);
			for(int j=0;j<8;j++)
			{
			primitives[base+2*j].vertices[0]=vector3(-track_type->rail_offset_x,track_type->rail_offset_y,j*0.125*track_type->length);
			primitives[base+2*j].vertices[1]=vector3(track_type->rail_offset_x,track_type->rail_offset_y,j*0.125*track_type->length);
			primitives[base+2*j].vertices[2]=vector3(track_type->rail_offset_x,track_type->rail_offset_y,(j+1)*0.125*track_type->length);
			primitives[base+2*j].normals[0]=vector3(0.0,1.0,0.0);
			primitives[base+2*j].normals[1]=vector3(0.0,1.0,0.0);
			primitives[base+2*j].normals[2]=vector3(0.0,1.0,0.0);
			primitives[base+2*j].material=&blank_mat;

			primitives[base+2*j+1].vertices[0]=vector3(track_type->rail_offset_x,track_type->rail_offset_y,(j+1)*0.125*track_type->length);
			primitives[base+2*j+1].vertices[1]=vector3(-track_type->rail_offset_x,track_type->rail_offset_y,(j+1)*0.125*track_type->length);
			primitives[base+2*j+1].vertices[2]=vector3(-track_type->rail_offset_x,track_type->rail_offset_y,j*0.125*track_type->length);
			primitives[base+2*j+1].normals[0]=vector3(0.0,1.0,0.0);
			primitives[base+2*j+1].normals[1]=vector3(0.0,1.0,0.0);
			primitives[base+2*j+1].normals[2]=vector3(0.0,1.0,0.0);
			primitives[base+2*j+1].material=&blank_mat;
			}
			for(int j=0;j<primitives_per_mesh-16;j++)primitives[i*primitives_per_mesh+j].material=NULL;
		}
	transform_primitives(transform,primitives+i*primitives_per_mesh,primitives_per_mesh);
	}
track_transform_primitives(track_section->curve,primitives,((uint32_t)num_meshes)*primitives_per_mesh,track_section->flags);
transform_primitives(context->projection,primitives,((uint32_t)num_meshes)*primitives_per_mesh);
framebuffer_from_primitives(framebuffer,context,primitives,((uint32_t)num_meshes)*primitives_per_mesh);
free(primitives);
}
