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
float min=10000000.0;
float max=-10000000.0;
	for(int i=0;i<num_primitives;i++)
	{
		for(int j=0;j<3;j++)
		{
		min=(primitives[i].vertices[j].z/4.848872)<min?(primitives[i].vertices[j].z/4.848872):min;
		max=(primitives[i].vertices[j].z/4.848872)>max?(primitives[i].vertices[j].z/4.848872):max;
		track_point_t track_point;
			if(primitives[i].vertices[j].z>0)
			{
			track_point=track_curve(primitives[i].vertices[j].z);
			}
			else
			{
			track_point=track_curve(0);
			track_point.position=vector3_add(track_point.position,vector3_mult(track_point.tangent,primitives[i].vertices[j].z));
			}
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
int num_meshes=(int)floor(0.5+track_section->length/track_type->length);
float scale=track_section->length/(num_meshes*track_type->length);

float length=scale*track_type->length;
	if(extrude_behind)num_meshes++;//TODO make extruded section straight

uint32_t primitives_per_mesh=mesh_count_primitives(&(track_type->mesh));
uint32_t primitives_per_mask=0;	
	if(track_mask)primitives_per_mask=mesh_count_primitives(&(track_type->mask));

primitive_t* primitives=malloc(num_meshes*(primitives_per_mesh+primitives_per_mask)*sizeof(primitive_t));
primitive_t* mask_primitives=primitives+(primitives_per_mesh*num_meshes);
	for(int i=0;i<num_meshes;i++)
	{
	transform_t transform={{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,scale},{0.0,0.0,(i-(extrude_behind?1:0))*length}};

	mesh_get_primitives(&(track_type->mesh),primitives+i*primitives_per_mesh);
	transform_primitives(transform,primitives+i*primitives_per_mesh,primitives_per_mesh);
	
		if(track_mask)
		{
		mesh_get_primitives(&(track_type->mask),mask_primitives+i*primitives_per_mask);
		transform_primitives(transform,mask_primitives+i*primitives_per_mask,primitives_per_mask);
		}
	}
	if(track_mask)
	{
		for(int j=0;j<num_meshes*primitives_per_mesh;j++)primitives[j].material=NULL;
	}
track_transform_primitives(track_section->curve,primitives,num_meshes*(primitives_per_mesh+primitives_per_mask),track_section->flags);
transform_primitives(context->projection,primitives,num_meshes*(primitives_per_mesh+primitives_per_mask));
framebuffer_from_primitives(framebuffer,context,primitives,num_meshes*(primitives_per_mesh+primitives_per_mask));
free(primitives);
}
