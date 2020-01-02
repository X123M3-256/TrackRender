#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "track.h"


#define Z_OFFSET ((12.0/8.0)*0.75)

vector3_t change_coordinates(vector3_t a)
{
vector3_t result={a.z,a.y,a.x};
return result;
}

typedef struct
	{
	track_point_t (*track_curve)(float);
	float scale;
	float offset;
	int flags;
	}track_transform_args_t;

track_point_t get_track_point(track_point_t (*curve)(float distance),int flags,float u)
{
track_point_t track_point;
	if(u>=0)
	{
	track_point=curve(u);
	}
	else
	{
	track_point=curve(0);
	track_point.position=vector3_add(track_point.position,vector3_mult(track_point.tangent,u));
	}

	if(flags&TRACK_DIAGONAL)track_point.position.x+=0.75*sqrt(6);
track_point.position.y+=Z_OFFSET-1.5;
	if(!(flags&TRACK_VERTICAL))track_point.position.z-=0.75*sqrt(6);
return track_point;
}

track_point_t only_yaw(track_point_t input)
{
track_point_t output;
output.position=input.position;
output.normal=vector3(0,1,0);
output.binormal=vector3_normalize(vector3_cross(output.normal,change_coordinates(input.tangent)));
output.tangent=vector3_normalize(vector3_cross(output.normal,output.binormal));
return output;
}

vertex_t track_transform(vector3_t vertex,vector3_t normal,void* data)
{
track_transform_args_t args=*((track_transform_args_t*)data);

vertex.z=args.scale*vertex.z+args.offset;

track_point_t track_point=get_track_point(args.track_curve,args.flags,vertex.z);	

vertex_t out;
out.vertex=change_coordinates(vector3_add(track_point.position,vector3_add(vector3_mult(track_point.normal,vertex.y),vector3_mult(track_point.binormal,vertex.x))));
out.normal=change_coordinates(vector3_add(vector3_mult(track_point.tangent,normal.z),vector3_add(vector3_mult(track_point.normal,normal.y),vector3_mult(track_point.binormal,normal.x))));
return out;
}

vertex_t base_transform(vector3_t vertex,vector3_t normal,void* data)
{
track_transform_args_t args=*((track_transform_args_t*)data);

vertex.z=args.scale*vertex.z+args.offset;

track_point_t track_point=get_track_point(args.track_curve,args.flags,vertex.z);	

track_point.binormal=vector3_normalize(vector3_cross(vector3(0,1,0),track_point.tangent));
track_point.normal=vector3_normalize(vector3_cross(track_point.tangent,track_point.binormal));


vertex_t out;
out.vertex=change_coordinates(vector3_add(track_point.position,vector3_add(vector3_mult(vector3(0,1,0),vertex.y),vector3_mult(track_point.binormal,vertex.x))));
out.normal=change_coordinates(vector3_add(vector3_mult(track_point.tangent,normal.z),vector3_add(vector3_mult(track_point.normal,normal.y),vector3_mult(track_point.binormal,normal.x))));
return out;
}


int get_support_index(int bank)
{
	switch(bank)
	{
	case 0:
	return SUPPORT_FLAT;
	break;
	case -1:
	case 1:
	return SUPPORT_BANK_HALF;
	break;
	case -2:
	case 2:
	return SUPPORT_BANK;
	break;
	}
}

int get_special_index(int flags)
{
	switch(flags&TRACK_SPECIAL_MASK)
	{
	case TRACK_SPECIAL_STEEP_TO_VERTICAL:
		return SUPPORT_SPECIAL_STEEP_TO_VERTICAL;
	break;
	case TRACK_SPECIAL_VERTICAL_TO_STEEP:
		return SUPPORT_SPECIAL_VERTICAL_TO_STEEP;
	break;
	case TRACK_SPECIAL_VERTICAL:
		return SUPPORT_SPECIAL_VERTICAL;
	break;
	case TRACK_SPECIAL_VERTICAL_TWIST_LEFT:
	case TRACK_SPECIAL_VERTICAL_TWIST_RIGHT:
		return SUPPORT_SPECIAL_VERTICAL_TWIST;
	break;
	case TRACK_SPECIAL_BARREL_ROLL_LEFT:
	case TRACK_SPECIAL_BARREL_ROLL_RIGHT:
		return SUPPORT_SPECIAL_BARREL_ROLL;
	break;
	case TRACK_SPECIAL_HALF_LOOP:
		return SUPPORT_SPECIAL_HALF_LOOP;
	break;
	case TRACK_SPECIAL_QUARTER_LOOP:
		return SUPPORT_SPECIAL_QUARTER_LOOP;
	break;
	}
}

void render_track_section(context_t* context,track_section_t* track_section,track_type_t* track_type,int extrude_behind,int track_mask,int rendered_views,image_t* images)
{
int num_meshes=(int)floor(0.5+track_section->length/track_type->length);
float scale=track_section->length/(num_meshes*track_type->length);

float length=scale*track_type->length;
	if(extrude_behind)num_meshes++;//TODO make extruded section straight


#define DENOM 2

context_begin_render(context);
	for(int i=0;i<num_meshes;i++)
	{
	track_transform_args_t args;
	args.scale=scale;
	args.offset=(i-(extrude_behind?1:0))*length;
	args.track_curve=track_section->curve;
	args.flags=track_section->flags;
     		if(track_mask)context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);
	context_add_model_transformed(context,&(track_type->mesh),track_transform,&args,track_mask); 
		if((track_type->flags&TRACK_HAS_SUPPORTS)&&!(track_section->flags&TRACK_NO_SUPPORTS))context_add_model_transformed(context,&(track_type->supports[SUPPORT_BASE]),base_transform,&args,track_mask); 
	}

	if(track_section->flags&TRACK_SPECIAL_MASK)
	{
	int index=get_special_index(track_section->flags);
		if(index==SUPPORT_SPECIAL_VERTICAL||index==SUPPORT_SPECIAL_STEEP_TO_VERTICAL||index==SUPPORT_SPECIAL_VERTICAL_TO_STEEP||index==SUPPORT_SPECIAL_VERTICAL_TWIST)
		{
		matrix_t mat=views[3];
			if((track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_VERTICAL_TWIST_RIGHT)
			{
			mat.entries[6]*=-1;
			mat.entries[7]*=-1;
			mat.entries[8]*=-1;
			}
		context_add_model(context,&(track_type->supports[index]),transform(mat,vector3(0.0,Z_OFFSET-1.5,0.0)),track_mask); 
		}
	}

	if((track_type->flags&TRACK_HAS_SUPPORTS)&&!(track_section->flags&TRACK_NO_SUPPORTS))
	{
	float support_spacing=0.8*TILE_SIZE;
	int num_supports=(int)floor(0.5+track_section->length/support_spacing);
	float support_step=track_section->length/num_supports;
	int entry=0;
	int exit=0;
		if(track_section->flags&TRACK_ENTRY_BANK_LEFT)entry=DENOM;
		else if(track_section->flags&TRACK_ENTRY_BANK_RIGHT)entry=-DENOM;
		else entry=0;

		if(track_section->flags&TRACK_EXIT_BANK_LEFT)exit=DENOM;
		else if(track_section->flags&TRACK_EXIT_BANK_RIGHT)exit=-DENOM;
		else exit=0;
		
		for(int i=0;i<num_supports+1;i++)
		{
		int u=(i*DENOM)/num_supports;
		int bank_angle=(entry*(DENOM-u)+(exit*u))/DENOM;

		track_point_t track_point=only_yaw(get_track_point(track_section->curve,track_section->flags,i*support_step));
		matrix_t rotation=matrix(track_point.binormal.x,track_point.normal.x,track_point.tangent.x,track_point.binormal.y,track_point.normal.y,track_point.tangent.y,track_point.binormal.z,track_point.normal.z,track_point.tangent.z);
			if(bank_angle<0)rotation=matrix_mult(views[2],rotation);
		context_add_model(context,&(track_type->supports[get_support_index(bank_angle)]),transform(rotation,change_coordinates(track_point.position)),track_mask); 
		}
	}




context_finalize_render(context);
	for(int i=0;i<4;i++)
	{
		if(rendered_views&(1<<i))
		{
			if(track_mask)context_render_silhouette(context,rotate_y(0.5*i*M_PI),images+i);
			else context_render_view(context,rotate_y(0.5*i*M_PI),images+i);
		}
	}
context_end_render(context);
}
