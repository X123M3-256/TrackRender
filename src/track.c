#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "track.h"
#include "sprites.h"

float start_offset_x=0.0;
float start_offset_y=0.0;
float end_offset_x=0.0;
float end_offset_y=0.0;


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
	float z_offset;
	float length;
	int flags;
	}track_transform_args_t;

track_point_t get_track_point(track_point_t (*curve)(float distance),int flags,float z_offset,float length,float u)
{
track_point_t track_point;
	if(u<0)
	{
	track_point=curve(0);
	track_point.position=vector3_add(track_point.position,vector3_mult(track_point.tangent,u));
	}
	else if(u>length)
	{
	track_point=curve(length);
	track_point.position=vector3_add(track_point.position,vector3_mult(track_point.tangent,(u-length)));
	}
	else
	{
	track_point=curve(u);
	}

	if(flags&TRACK_DIAGONAL)track_point.position.x+=0.5*TILE_SIZE;
	if(flags&TRACK_DIAGONAL_2)track_point.position.z+=0.5*TILE_SIZE;
track_point.position.y+=z_offset-2*CLEARANCE_HEIGHT;
	if(!(flags&TRACK_VERTICAL))track_point.position.z-=0.5*TILE_SIZE;

float v=u/length;
	if(v<0)v=0;
	else if(v>1)v=1;

track_point.position.z+=(start_offset_x*TILE_SIZE/32.0)*(2*v*v*v-3*v*v+1);
track_point.position.y+=(start_offset_y*CLEARANCE_HEIGHT/8.0)*(2*v*v*v-3*v*v+1);

track_point.position.z+=(end_offset_x*TILE_SIZE/32.0)*(-2*v*v*v+3*v*v);
track_point.position.y+=(end_offset_y*CLEARANCE_HEIGHT/8.0)*(-2*v*v*v+3*v*v);

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

track_point_t track_point=get_track_point(args.track_curve,args.flags,args.z_offset,args.length,vertex.z);	

vertex_t out;
out.vertex=change_coordinates(vector3_add(track_point.position,vector3_add(vector3_mult(track_point.normal,vertex.y),vector3_mult(track_point.binormal,vertex.x))));
out.normal=change_coordinates(vector3_add(vector3_mult(track_point.tangent,normal.z),vector3_add(vector3_mult(track_point.normal,normal.y),vector3_mult(track_point.binormal,normal.x))));
return out;
}

vertex_t base_transform(vector3_t vertex,vector3_t normal,void* data)
{
track_transform_args_t args=*((track_transform_args_t*)data);

vertex.z=args.scale*vertex.z+args.offset;

track_point_t track_point=get_track_point(args.track_curve,args.flags,args.z_offset,args.length,vertex.z);	

track_point.binormal=vector3_normalize(vector3_cross(vector3(0,1,0),track_point.tangent));
track_point.normal=vector3_normalize(vector3_cross(track_point.tangent,track_point.binormal));


vertex_t out;
out.vertex=change_coordinates(vector3_add(track_point.position,vector3_add(vector3_mult(vector3(0,1,0),vertex.y),vector3_mult(track_point.binormal,vertex.x))));
out.normal=change_coordinates(vector3_add(vector3_mult(track_point.tangent,normal.z),vector3_add(vector3_mult(track_point.normal,normal.y),vector3_mult(track_point.binormal,normal.x))));
return out;
}

#define DENOM 6

int get_support_index(int bank)
{
	switch(bank)
	{
	case 0:
	return MODEL_FLAT;
	break;
	case -1:
	case 1:
	return MODEL_BANK_SIXTH;
	break;
	case -2:
	case 2:
	return MODEL_BANK_THIRD;
	case -3:
	case 3:
	return MODEL_BANK_HALF;
	break;
	case -4:
	case 4:
	return MODEL_BANK_TWO_THIRDS;
	break;
	case -5:
	case 5:
	return MODEL_BANK_FIVE_SIXTHS;
	case -6:
	case 6:
	return MODEL_BANK;
	break;
	}
return MODEL_FLAT;
}

int get_special_index(int flags)
{
	switch(flags&TRACK_SPECIAL_MASK)
	{
	case TRACK_SPECIAL_STEEP_TO_VERTICAL:
		return MODEL_SPECIAL_STEEP_TO_VERTICAL;
	break;
	case TRACK_SPECIAL_VERTICAL_TO_STEEP:
		return MODEL_SPECIAL_VERTICAL_TO_STEEP;
	break;
	case TRACK_SPECIAL_VERTICAL:
		return MODEL_SPECIAL_VERTICAL;
	break;
	case TRACK_SPECIAL_VERTICAL_TWIST_LEFT:
	case TRACK_SPECIAL_VERTICAL_TWIST_RIGHT:
		return MODEL_SPECIAL_VERTICAL_TWIST;
	break;
	case TRACK_SPECIAL_BARREL_ROLL_LEFT:
	case TRACK_SPECIAL_BARREL_ROLL_RIGHT:
		return MODEL_SPECIAL_BARREL_ROLL;
	break;
	case TRACK_SPECIAL_HALF_LOOP:
		return MODEL_SPECIAL_HALF_LOOP;
	break;
	case TRACK_SPECIAL_QUARTER_LOOP:
		return MODEL_SPECIAL_QUARTER_LOOP;
	break;
	case TRACK_SPECIAL_CORKSCREW_LEFT:
	case TRACK_SPECIAL_CORKSCREW_RIGHT:
		return MODEL_SPECIAL_CORKSCREW;
	break;
	case TRACK_SPECIAL_ZERO_G_ROLL_LEFT:
	case TRACK_SPECIAL_ZERO_G_ROLL_RIGHT:
		return MODEL_SPECIAL_ZERO_G_ROLL;
	break;
	case TRACK_SPECIAL_LARGE_ZERO_G_ROLL_LEFT:
	case TRACK_SPECIAL_LARGE_ZERO_G_ROLL_RIGHT:
		return MODEL_SPECIAL_LARGE_ZERO_G_ROLL;
	break;
	case TRACK_SPECIAL_BRAKE:
		return MODEL_SPECIAL_BRAKE;
	break;
	case TRACK_SPECIAL_BLOCK_BRAKE:
		return MODEL_SPECIAL_BLOCK_BRAKE;
	break;
	case TRACK_SPECIAL_BOOSTER:
	case TRACK_SPECIAL_LAUNCHED_LIFT:
	case TRACK_SPECIAL_VERTICAL_BOOSTER:
		return MODEL_SPECIAL_BOOSTER;
	break;
	}
assert(0);
return 0;
}

void render_track_section(context_t* context,track_section_t* track_section,track_type_t* track_type,int extrude_behind,int track_mask,int rendered_views,image_t* images,int subtype)
{
int num_meshes=(int)floor(0.5+track_section->length/track_type->length);
float scale=track_section->length/(num_meshes*track_type->length);

//If alternating track meshes are used, we would prefer to have an even number of meshes as long as it doesn't cause too much distortion
	if(track_type->models_loaded&(1<<MODEL_TRACK_ALT))
	{
	int num_meshes_even=2*(int)floor(0.5+track_section->length/(2*track_type->length));
		if(track_section->flags&TRACK_ALT_PREFER_ODD)num_meshes_even=2*(int)floor(track_section->length/(2*track_type->length))+1;
	float scale_even=track_section->length/(num_meshes_even*track_type->length);
		if(scale_even>0.9&&scale_even<1.11111)	
		{
		num_meshes=num_meshes_even;
		scale=scale_even;
		}
	}


float length=scale*track_type->length;
float z_offset=((track_type->z_offset/8.0)*CLEARANCE_HEIGHT);


mesh_t* mesh;
		switch(subtype)
		{
		case TRACK_SUBTYPE_DEFAULT:
			mesh=&(track_type->mesh);
		break;	
		case TRACK_SUBTYPE_LIFT:
			mesh=&(track_type->lift_mesh);
		break;
		default:
			assert(0);
		break;
		}


context_begin_render(context);

//Add ghost models/track masks at start and end
track_transform_args_t args;
args.scale=scale;
args.offset=-length;
args.z_offset=z_offset;
args.track_curve=track_section->curve;
args.flags=track_section->flags;
args.length=track_section->length;
	if(track_mask)context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);//);
	else if(!extrude_behind)context_add_model_transformed(context,mesh,track_transform,&args,MESH_GHOST);
args.offset=track_section->length;
	if(track_mask)context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);//track_mask?0:MESH_GHOST);
	else context_add_model_transformed(context,mesh,track_transform,&args,MESH_GHOST);


	if(track_type->flags&TRACK_TIE_AT_BOUNDARY)
	{
	//Determine start angle
	int start_angle=3;
		if(rendered_views&1)start_angle=0;
		else if(rendered_views&2)start_angle=1;
		else if(rendered_views&4)start_angle=2;

	int end_angle=start_angle;
		if(track_section->flags&TRACK_EXIT_90_DEG_LEFT)end_angle-=1;
		else if(track_section->flags&TRACK_EXIT_90_DEG_RIGHT)end_angle+=1;
		else if(track_section->flags&TRACK_EXIT_180_DEG)end_angle+=2;
		if(end_angle<0)end_angle+=4;
		if(end_angle>3)end_angle-=4;

	int start_tie=start_angle<=1;
	int end_tie=end_angle>1;

	//Calculate corrected scale
	double corrected_length=num_meshes*track_type->length;
		if(!start_tie)corrected_length-=track_type->tie_length;
		if(end_tie)corrected_length+=track_type->tie_length;
	double corrected_scale=track_section->length/corrected_length;

	double tie_length=corrected_scale*track_type->tie_length;
	double inter_length=corrected_scale*(track_type->length-track_type->tie_length);

	double offset=0;

		if(extrude_behind)
		{
		num_meshes++;
		offset-=(extrude_behind?1:0)*corrected_scale*track_type->length;
		}
		for(int i=0;i<2*num_meshes+1;i++)
		{
		track_transform_args_t args;
		args.scale=corrected_scale;
		args.offset=offset;
		args.z_offset=z_offset;
		args.track_curve=track_section->curve;
		args.flags=track_section->flags;
		args.length=track_section->length;
			if((!(i&1))&&(i!=0||start_tie)&&(i!=2*num_meshes||end_tie))
			{
			track_point_t track_point=get_track_point(track_section->curve,track_section->flags,z_offset,args.length,args.offset+track_type->tie_length/2);	
			context_add_model(context,&(track_type->tie_mesh),transform(matrix(track_point.binormal.z,track_point.normal.z,track_point.tangent.z,track_point.binormal.y,track_point.normal.y,track_point.tangent.y,track_point.binormal.x,track_point.normal.x,track_point.tangent.x),change_coordinates(track_point.position)),track_mask);
				if(track_type->models_loaded&(1<<MODEL_TRACK_TIE))context_add_model_transformed(context,&(track_type->models[MODEL_TRACK_TIE]),track_transform,&args,track_mask);
			offset+=tie_length;
			}
			else if(i&1)
			{
			int use_alt=(track_type->models_loaded&(1<<MODEL_TRACK_ALT))&&(i&2);
				if(track_section->flags&TRACK_ALT_INVERT)use_alt=!use_alt;
			//Add track model
				if(!(track_type->models_loaded&(1<<MODEL_TRACK_TIE))&&start_tie)args.offset=offset-tie_length;
				if(use_alt)context_add_model_transformed(context,&(track_type->models[MODEL_TRACK_ALT]),track_transform,&args,track_mask);
				else context_add_model_transformed(context,mesh,track_transform,&args,track_mask);
			//Add track mask
				if(track_mask)
				{
					if(start_tie)args.offset=offset-tie_length;
				context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);
				}
			offset+=inter_length;
			}
		}

	}
	else
	{
		if(extrude_behind)num_meshes++;
		for(int i=0;i<num_meshes;i++)
		{
		track_transform_args_t args;
		args.scale=scale;
		args.offset=(i-(extrude_behind?1:0))*length;
		args.z_offset=z_offset;
		args.track_curve=track_section->curve;
		args.flags=track_section->flags;
		args.length=track_section->length;
		
		int alt_available=track_type->models_loaded&(1<<MODEL_TRACK_ALT);
		int use_alt=alt_available&&(i&1);
			if(alt_available&&(track_section->flags&TRACK_ALT_INVERT))use_alt=!use_alt;

			if(track_mask)context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);
			if(use_alt)context_add_model_transformed(context,&(track_type->models[MODEL_TRACK_ALT]),track_transform,&args,track_mask);
			else context_add_model_transformed(context,mesh,track_transform,&args,track_mask);

			if((track_type->models_loaded&(1<<MODEL_BASE))&&(track_type->flags&TRACK_HAS_SUPPORTS)&&!(track_section->flags&TRACK_NO_SUPPORTS))context_add_model_transformed(context,&(track_type->models[MODEL_BASE]),base_transform,&args,track_mask); 
			if(track_type->flags&TRACK_SEPARATE_TIE)
			{
			track_point_t track_point=get_track_point(track_section->curve,track_section->flags,z_offset,args.length,args.offset+0.5*length);	
			context_add_model(context,&(track_type->tie_mesh),transform(matrix(track_point.binormal.z,track_point.normal.z,track_point.tangent.z,track_point.binormal.y,track_point.normal.y,track_point.tangent.y,track_point.binormal.x,track_point.normal.x,track_point.tangent.x),change_coordinates(track_point.position)),track_mask);
			}
		}
	}
	


	if(track_section->flags&TRACK_SPECIAL_MASK)
	{
	int index=get_special_index(track_section->flags);
		if(track_type->models_loaded&(1<<index))
		{
		matrix_t mat=views[1];
			if((track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_VERTICAL_TWIST_RIGHT&&(track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_BARREL_ROLL_RIGHT&&(track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_CORKSCREW_RIGHT&&(track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_ZERO_G_ROLL_RIGHT&&(track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_LARGE_ZERO_G_ROLL_RIGHT)
			{
			mat.entries[6]*=-1;
			mat.entries[7]*=-1;
			mat.entries[8]*=-1;
			}
			if((track_section->flags&TRACK_SPECIAL_MASK)==TRACK_SPECIAL_LAUNCHED_LIFT)
			{
			mat=matrix_mult(mat,rotate_x(0.387596687));
			mat=matrix_mult(mat,matrix(1,0,0,0,1,0,0,0,1.080123));
			}
			else if((track_section->flags&TRACK_SPECIAL_MASK)==TRACK_SPECIAL_VERTICAL_BOOSTER)
			{
			mat=matrix_mult(mat,rotate_x(-0.5*M_PI));
			mat=matrix_mult(mat,matrix(1,0,0,0,1,0,0,0,0.816496580928));
			}
		context_add_model(context,&(track_type->models[index]),transform(mat,vector3(!(track_section->flags&TRACK_VERTICAL)?-0.5*TILE_SIZE:0,z_offset-2*CLEARANCE_HEIGHT,0)),track_mask); 
		}
	}

	if((track_type->flags&TRACK_HAS_SUPPORTS)&&!(track_section->flags&TRACK_NO_SUPPORTS))
	{
	int num_supports=(int)floor(0.5+track_section->length/track_type->support_spacing);
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

		track_point_t track_point=get_track_point(track_section->curve,track_section->flags,z_offset,track_section->length,i*support_step);

		track_point_t support_point=only_yaw(track_point);

		matrix_t rotation=matrix(support_point.binormal.x,support_point.normal.x,support_point.tangent.x,support_point.binormal.y,support_point.normal.y,support_point.tangent.y,support_point.binormal.z,support_point.normal.z,support_point.tangent.z);
			if(bank_angle<0)rotation=matrix_mult(views[2],rotation);

		vector3_t translation=change_coordinates(support_point.position);
		translation.y-=track_type->pivot/sqrt(track_point.tangent.x*track_point.tangent.x+track_point.tangent.z*track_point.tangent.z)-track_type->pivot;

		context_add_model(context,&(track_type->models[get_support_index(bank_angle)]),transform(rotation,translation),track_mask); 
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


int is_in_mask(int x,int y,mask_t* mask)
{
	for(int i=0;i<mask->num_rects;i++)
	{
		if(x>=mask->rects[i].x_lower&&x<mask->rects[i].x_upper&&y>=mask->rects[i].y_lower&&y<mask->rects[i].y_upper)return 1;
	}
return 0;
}

int offset_table_index(track_point_t track)
{
	if(vector3_norm(vector3_sub(track.normal,vector3_normalize(vector3(0,1,0))))<0.1&&vector3_norm(vector3_sub(track.tangent,vector3_normalize(vector3(0,0,1))))<0.1)return 0;
	else if(vector3_norm(vector3_sub(track.tangent,vector3_normalize(vector3(0,2*CLEARANCE_HEIGHT,TILE_SIZE))))<0.1)return 0x01;
	else if(vector3_norm(vector3_sub(track.tangent,vector3_normalize(vector3(TILE_SIZE,2*CLEARANCE_HEIGHT,0))))<0.1)return 0x31;
	else if(vector3_norm(vector3_sub(track.tangent,vector3_normalize(vector3(-TILE_SIZE,2*CLEARANCE_HEIGHT,0))))<0.1)return 0x11;
	else if(vector3_norm(vector3_sub(track.tangent,vector3_normalize(vector3(0,8*CLEARANCE_HEIGHT,TILE_SIZE))))<0.1)return 2;
	else if(vector3_norm(vector3_sub(track.normal,vector3_normalize(vector3(1,1,0))))<0.1)return 3;
	else if(vector3_norm(vector3_sub(track.normal,vector3_normalize(vector3(-1,1,0))))<0.1)return 3;
	else if(vector3_norm(vector3_sub(track.normal,vector3_normalize(vector3(0,-1,0))))<0.1)
	{
		if(vector3_norm(vector3_sub(track.tangent,vector3_normalize(vector3(1,0,0))))<0.1)return 0x34;
		else if(vector3_norm(vector3_sub(track.tangent,vector3_normalize(vector3(-1,0,0))))<0.1)return 0x14;
		else if(track.tangent.z>0.8)return 0x04;
		else if(track.tangent.z<0.8)return 0x24;
	return 5;
	}
return 5;
}

void set_offset(int view_angle,track_section_t* track_section)
{
int start_table=offset_table_index(track_section->curve(0));
int end_table=offset_table_index(track_section->curve(track_section->length));

float offset_tables[6][8]={
	{0,0.5,0,0,0,0.5,0,0},
	{0,1.0,0,0,0,0,0,0},//Gentle
	{-2.25,0,-2.0,0,-0.75,0,-1.5,-1.0},//Steep
	{0,1.0,0,1.0,0,1.0,0,0},//Bank
	{0,-0.5,0,0,0,-0.5,0,0},//Inverted
	{0,0,0,0,0,0,0,0},//Other
	};

start_offset_x=offset_tables[start_table&0xF][2*view_angle];
start_offset_y=offset_tables[start_table&0xF][2*view_angle+1];

view_angle=(view_angle+(end_table>>4))%4;
end_table&=0xF;

end_offset_x=offset_tables[end_table&0xF][2*view_angle];
end_offset_y=offset_tables[end_table&0xF][2*view_angle+1];
}

void render_track_sections(context_t* context,track_section_t* track_section,track_type_t* track_type,int track_mask,int subtype,int views,image_t* sprites)
{
/*
set_offset(0,track_section);
	if(views&0x1)render_track_section(context,track_section,track_type,0,track_mask,0x1,sprites,subtype);
set_offset(1,track_section);
	if(views&0x2)render_track_section(context,track_section,track_type,0,track_mask,0x2,sprites,subtype);
set_offset(2,track_section);
	if(views&0x4)render_track_section(context,track_section,track_type,0,track_mask,0x4,sprites,subtype);
set_offset(3,track_section);
	if(views&0x8)render_track_section(context,track_section,track_type,0,track_mask,0x8,sprites,subtype);
return;
*/
	if(track_section->flags&TRACK_EXTRUDE_BEHIND)
	{
		if(track_type->flags&TRACK_SEPARATE_TIE)
		{
			if(views&0x1)render_track_section(context,track_section,track_type,1,track_mask,0x1,sprites,subtype);
			if(views&0x2)render_track_section(context,track_section,track_type,0,track_mask,0x2,sprites,subtype);
			if(views&0x4)render_track_section(context,track_section,track_type,1,track_mask,0x4,sprites,subtype);
			if(views&0x8)render_track_section(context,track_section,track_type,0,track_mask,0x8,sprites,subtype);
		}
		else
		{
			if(views&0x5)render_track_section(context,track_section,track_type,1,track_mask,views&0x5,sprites,subtype);
			if(views&0xA)render_track_section(context,track_section,track_type,0,track_mask,views&0xA,sprites,subtype);
		}
	}
	else
	{
		if((track_type->flags&TRACK_SEPARATE_TIE)&&(track_section->flags&TRACK_EXIT_90_DEG))
		{
			if(views&0x1)render_track_section(context,track_section,track_type,0,track_mask,0x1,sprites,subtype);
			if(views&0x2)render_track_section(context,track_section,track_type,0,track_mask,0x2,sprites,subtype);
			if(views&0x4)render_track_section(context,track_section,track_type,0,track_mask,0x4,sprites,subtype);
			if(views&0x8)render_track_section(context,track_section,track_type,0,track_mask,0x8,sprites,subtype);
		}
		else if((track_type->flags&TRACK_SEPARATE_TIE))
		{
			if(views&0x3)render_track_section(context,track_section,track_type,0,track_mask,views&0x3,sprites,subtype);
			if(views&0xC)render_track_section(context,track_section,track_type,0,track_mask,views&0xC,sprites,subtype);
		}
		else
		{
		render_track_section(context,track_section,track_type,0,track_mask,views,sprites,subtype);
		}
	}
}

void write_track_section(context_t* context,track_section_t* track_section,track_type_t* track_type,const char* base_directory,const char* filename,json_t* sprites,int subtype,image_t* overlay)
{
int z_offset=(int)(track_type->z_offset+0.499999);
image_t full_sprites[4];
render_track_sections(context,track_section,track_type,0,subtype,0xF,full_sprites);

	if(overlay!=NULL&&!(track_type->flags & TRACK_NO_LIFT_SPRITE))
	{
		for(int i=0;i<4;i++)image_blit(full_sprites+i,overlay+i,0,track_type->lift_offset-z_offset);
	}

image_t track_masks[4];
int track_mask_views=0;
	for(int i=0;i<4;i++)track_mask_views|=(track_section->views[i].flags&VIEW_NEEDS_TRACK_MASK?1:0)<<i;
	if(track_mask_views!=0)render_track_sections(context,track_section,track_type,1,subtype,track_mask_views,track_masks);

	for(int angle=0;angle<4;angle++)
	{
		if(track_section->views[angle].num_sprites==0)continue;

	view_t* view=track_section->views+angle;

	char final_filename[512];
	char relative_filename[512];
	snprintf(relative_filename,512,"%s_%d.png",filename,angle+1);
		
		for(int sprite=0;sprite<view->num_sprites;sprite++)
		{
		char final_filename[512];
		char relative_filename[512];
			if(view->num_sprites==1)snprintf(relative_filename,512,"%s_%d.png",filename,angle+1);
			else snprintf(relative_filename,512,"%s_%d_%d.png",filename,angle+1,sprite+1);
		snprintf(final_filename,512,"%s%s",base_directory,relative_filename);
//y		snprintf(final_filename,512,"../ImageEncode/%s",relative_filename);
		printf("%s\n",final_filename);

		image_t part_sprite;
		image_copy(full_sprites+angle,&part_sprite);

			if(view->masks!=NULL)
			{
				for(int x=0;x<full_sprites[angle].width;x++)
				for(int y=0;y<full_sprites[angle].height;y++)
				{
				int in_mask=is_in_mask(x+full_sprites[angle].x_offset,y+full_sprites[angle].y_offset+((track_section->flags&TRACK_OFFSET_SPRITE_MASK)?(z_offset-8):0),view->masks+sprite);

					if(view->masks[sprite].track_mask_op!=TRACK_MASK_NONE)
					{
					int mask_x=(x+full_sprites[angle].x_offset)-track_masks[angle].x_offset;
					int mask_y=(y+full_sprites[angle].y_offset)-track_masks[angle].y_offset;
					int in_track_mask=mask_x>=0&&mask_y>=0&&mask_x<track_masks[angle].width&&mask_y<track_masks[angle].height&&track_masks[angle].pixels[mask_x+mask_y*track_masks[angle].width]!=0;

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

						if(sprite<view->num_sprites-1&&(view->masks[sprite].track_mask_op&TRACK_MASK_TRANSFER_NEXT)&&in_track_mask&&is_in_mask(x+full_sprites[angle].x_offset,y+full_sprites[angle].y_offset+((track_section->flags&TRACK_OFFSET_SPRITE_MASK)?(z_offset-8):0),view->masks+sprite+1))in_mask=1;
						
					}

					if(view->flags&VIEW_ENFORCE_NON_OVERLAPPING)
					{
						for(int i=0;i<sprite;i++)
						{
							if(is_in_mask(x+full_sprites[angle].x_offset,y+full_sprites[angle].y_offset+((track_section->flags&TRACK_OFFSET_SPRITE_MASK)?(z_offset-8):0),view->masks+i))in_mask=0;//Note z offset untested
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
		//if(view->flags&VIEW_NEEDS_TRACK_MASK)image_write_png(&(track_masks[angle]),file);
		image_crop(&part_sprite);
		image_write_png(&part_sprite,file);
		//image_write_png(full_sprites+angle,file);
		fclose(file);	

		json_t* sprite_entry=json_object();
		json_object_set(sprite_entry,"path",json_string(relative_filename));
		json_object_set(sprite_entry,"x_offset",json_integer(part_sprite.x_offset));
		json_object_set(sprite_entry,"y_offset",json_integer(part_sprite.y_offset));
		json_object_set(sprite_entry,"palette",json_string("keep"));
		json_array_append(sprites,sprite_entry);
		image_destroy(&part_sprite);
		}

		if(view->flags&VIEW_NEEDS_TRACK_MASK)image_destroy(track_masks+angle);
	image_destroy(full_sprites+angle);
	}
}

int write_track_subtype(context_t* context,track_type_t* track_type,track_list_t track_list,json_t* sprites,const char* base_dir,const char* output_dir,int subtype)
{
char output_path[300];
const char* suffix="";

int groups=0;
	switch(subtype)
	{
	case TRACK_SUBTYPE_DEFAULT:
	groups=track_type->groups;
	suffix="";
	break;
	case TRACK_SUBTYPE_LIFT:
	groups=track_type->lift_groups;
	suffix="_lift";
	break;
	}

//Flat
	if(groups&TRACK_GROUP_FLAT)
	{
	sprintf(output_path,"%.255sflat%s",output_dir,suffix);
	if(subtype==TRACK_SUBTYPE_LIFT)write_track_section(context,&(track_list.flat_asymmetric),track_type,base_dir,output_path,sprites,subtype,flat_chain);
	else write_track_section(context,&(track_list.flat),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_BRAKES)
	{
	sprintf(output_path,"%.255sbrake%s",output_dir,suffix);
	write_track_section(context,&(track_list.brake),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sblock_brake%s",output_dir,suffix);
	write_track_section(context,&(track_list.block_brake),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_BOOSTERS)
	{
	sprintf(output_path,"%.255sbooster%s",output_dir,suffix);
	write_track_section(context,&(track_list.booster),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_VERTICAL_BOOSTERS)
	{
	sprintf(output_path,"%.255svertical_booster%s",output_dir,suffix);
	write_track_section(context,&(track_list.vertical_booster),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Slopes
	if(groups&TRACK_GROUP_GENTLE_SLOPES)
	{
	sprintf(output_path,"%.255sflat_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.flat_to_gentle_up),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?flat_to_gentle_up_chain:NULL);
	sprintf(output_path,"%.255sgentle_up_to_flat%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_to_flat),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_up_to_flat_chain:NULL);
	sprintf(output_path,"%.255sgentle%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_chain:NULL);
	}

	if(groups&TRACK_GROUP_STEEP_SLOPES)
	{
	sprintf(output_path,"%.255sgentle_to_steep_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_to_steep_up),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_to_steep_up_chain:NULL);
	sprintf(output_path,"%.255ssteep_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.steep_to_gentle_up),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?steep_to_gentle_up_chain:NULL);
	sprintf(output_path,"%.255ssteep%s",output_dir,suffix);
	write_track_section(context,&(track_list.steep),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?steep_chain:NULL);
	}
	if(groups&TRACK_GROUP_VERTICAL_SLOPES)
	{
	sprintf(output_path,"%.255ssteep_to_vertical_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.steep_to_vertical_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255svertical_to_steep_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.vertical_to_steep_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255svertical%s",output_dir,suffix);
	write_track_section(context,&(track_list.vertical),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Turns
	if(groups&TRACK_GROUP_TURNS)
	{
	sprintf(output_path,"%.255ssmall_turn_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_turn_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_turn_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_left_to_diag%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_turn_left_to_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_right_to_diag%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_turn_right_to_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Diagonals
	if(groups&TRACK_GROUP_DIAGONALS)
	{
	sprintf(output_path,"%.255sflat_diag%s",output_dir,suffix);
	write_track_section(context,&(track_list.flat_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?flat_diag_chain:NULL);
		if(groups&TRACK_GROUP_FLAT)
		{
		sprintf(output_path,"%.255sflat_to_gentle_up_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.flat_to_gentle_up_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?flat_to_gentle_up_diag_chain:NULL);
		sprintf(output_path,"%.255sgentle_to_flat_up_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.gentle_to_flat_up_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_to_flat_up_diag_chain:NULL);
		sprintf(output_path,"%.255sgentle_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.gentle_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_diag_chain:NULL);
		}
		if(groups&TRACK_GROUP_STEEP_SLOPES)
		{
		sprintf(output_path,"%.255sgentle_to_steep_up_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.gentle_to_steep_up_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_to_steep_up_diag_chain:NULL);
		sprintf(output_path,"%.255ssteep_to_gentle_up_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.steep_to_gentle_up_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?steep_to_gentle_up_diag_chain:NULL);
		sprintf(output_path,"%.255ssteep_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.steep_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?steep_diag_chain:NULL);
		}
	}

//Banked turns
	if(groups&TRACK_GROUP_BANKED_TURNS)
	{
	sprintf(output_path,"%.255sflat_to_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.flat_to_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sflat_to_right_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.flat_to_right_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sleft_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.left_bank_to_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sright_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.right_bank_to_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_to_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_to_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_to_right_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_to_right_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	
	sprintf(output_path,"%.255sleft_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);

		if(groups&TRACK_GROUP_DIAGONALS)
		{
		sprintf(output_path,"%.255sflat_to_left_bank_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.flat_to_left_bank_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sflat_to_right_bank_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.flat_to_right_bank_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sleft_bank_to_gentle_up_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.left_bank_to_gentle_up_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sright_bank_to_gentle_up_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.right_bank_to_gentle_up_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sgentle_up_to_left_bank_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.gentle_up_to_left_bank_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sgentle_up_to_right_bank_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.gentle_up_to_right_bank_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sleft_bank_diag%s",output_dir,suffix);
		write_track_section(context,&(track_list.left_bank_diag),track_type,base_dir,output_path,sprites,subtype,NULL);
		}

	sprintf(output_path,"%.255ssmall_turn_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_turn_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_turn_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_left_to_diag_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_turn_left_to_diag_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_right_to_diag_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_turn_right_to_diag_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Sloped turns
	if(groups&TRACK_GROUP_SLOPED_TURNS)
	{
	sprintf(output_path,"%.255ssmall_turn_left_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_turn_left_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssmall_turn_right_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_turn_right_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_left_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_turn_left_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_right_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_turn_right_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if((groups&TRACK_GROUP_STEEP_SLOPED_TURNS)&&(groups&TRACK_GROUP_STEEP_SLOPES))
	{
	sprintf(output_path,"%.255svery_small_turn_left_steep_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.very_small_turn_left_steep_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255svery_small_turn_right_steep_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.very_small_turn_right_steep_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if((groups&TRACK_GROUP_SLOPED_TURNS)&&(groups&TRACK_GROUP_VERTICAL_SLOPES))
	{
	sprintf(output_path,"%.255svertical_twist_left_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.vertical_twist_left_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255svertical_twist_right_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.vertical_twist_right_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Sloped banked turns

	if(groups&TRACK_GROUP_BANKED_SLOPED_TURNS)
	{
	sprintf(output_path,"%.255sgentle_up_to_gentle_up_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_to_gentle_up_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_to_gentle_up_right_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_to_gentle_up_right_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_left_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_left_bank_to_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_right_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_right_bank_to_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sleft_bank_to_gentle_up_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.left_bank_to_gentle_up_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sright_bank_to_gentle_up_right_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.right_bank_to_gentle_up_right_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_left_bank_to_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_left_bank_to_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_right_bank_to_right_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_right_bank_to_right_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_right_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_right_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sflat_to_gentle_up_left_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.flat_to_gentle_up_left_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sflat_to_gentle_up_right_bank%s",output_dir,suffix);
	write_track_section(context,&(track_list.flat_to_gentle_up_right_bank),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_left_bank_to_flat%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_left_bank_to_flat),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_right_bank_to_flat%s",output_dir,suffix);
	write_track_section(context,&(track_list.gentle_up_right_bank_to_flat),track_type,base_dir,output_path,sprites,subtype,NULL);
	
	sprintf(output_path,"%.255ssmall_turn_left_bank_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_turn_left_bank_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssmall_turn_right_bank_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_turn_right_bank_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_left_bank_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_turn_left_bank_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_right_bank_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_turn_right_bank_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Miscellaneous
	if(groups&TRACK_GROUP_S_BENDS)
	{
	sprintf(output_path,"%.255ss_bend_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.s_bend_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ss_bend_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.s_bend_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

	if(groups&TRACK_GROUP_HELICES)
	{
	sprintf(output_path,"%.255ssmall_helix_left_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_helix_left_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssmall_helix_right_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_helix_right_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_helix_left_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_helix_left_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_helix_right_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_helix_right_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Inversions
	if(groups&TRACK_GROUP_BARREL_ROLLS)
	{
	sprintf(output_path,"%.255sbarrel_roll_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.barrel_roll_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sbarrel_roll_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.barrel_roll_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_INLINE_TWISTS)
	{
	sprintf(output_path,"%.255sinline_twist_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.inline_twist_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sinline_twist_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.inline_twist_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_HALF_LOOPS)
	{
	sprintf(output_path,"%.255shalf_loop%s",output_dir,suffix);
	write_track_section(context,&(track_list.half_loop),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_LARGE_SLOPE_TRANSITIONS)
	{
	sprintf(output_path,"%.255sflat_to_steep_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.flat_to_steep_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssteep_to_flat_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.steep_to_flat_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_QUARTER_LOOPS)
	{
	sprintf(output_path,"%.255squarter_loop_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.quarter_loop_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_CORKSCREWS)
	{
	sprintf(output_path,"%.255scorkscrew_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.corkscrew_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255scorkscrew_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.corkscrew_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_LARGE_CORKSCREWS)
	{
	sprintf(output_path,"%.255slarge_corkscrew_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_corkscrew_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_corkscrew_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_corkscrew_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_TURN_BANK_TRANSITIONS)
	{
	sprintf(output_path,"%.255ssmall_turn_left_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_turn_left_bank_to_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssmall_turn_right_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_turn_right_bank_to_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	
	if(groups&TRACK_GROUP_MEDIUM_HALF_LOOPS)
	{
	sprintf(output_path,"%.255smedium_half_loop_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_half_loop_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_half_loop_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.medium_half_loop_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_LARGE_HALF_LOOPS)
	{
	sprintf(output_path,"%.255slarge_half_loop_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_half_loop_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_half_loop_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_half_loop_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_ZERO_G_ROLLS)
	{
	sprintf(output_path,"%.255szero_g_roll_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.zero_g_roll_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255szero_g_roll_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.zero_g_roll_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_zero_g_roll_left%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_zero_g_roll_left),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_zero_g_roll_right%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_zero_g_roll_right),track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	
	if(groups&TRACK_GROUP_SMALL_SLOPE_TRANSITIONS)
	{
	sprintf(output_path,"%.255ssmall_flat_to_steep_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_flat_to_steep_up),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?small_flat_to_steep_up_chain:NULL);
	sprintf(output_path,"%.255ssmall_steep_to_flat_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_steep_to_flat_up),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?small_steep_to_flat_up_chain:NULL);
	sprintf(output_path,"%.255ssmall_flat_to_steep_up_diag%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_flat_to_steep_up_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?small_flat_to_steep_up_diag_chain:NULL);
	sprintf(output_path,"%.255ssmall_steep_to_flat_up_diag%s",output_dir,suffix);
	write_track_section(context,&(track_list.small_steep_to_flat_up_diag),track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?small_steep_to_flat_up_diag_chain:NULL);
	}

	if(groups&TRACK_GROUP_LARGE_SLOPED_TURNS)
	{
	sprintf(output_path,"%.255slarge_turn_left_to_diag_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_turn_left_to_diag_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_right_to_diag_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_turn_right_to_diag_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_left_to_orthogonal_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_turn_left_to_orthogonal_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_right_to_orthogonal_gentle_up%s",output_dir,suffix);
	write_track_section(context,&(track_list.large_turn_right_to_orthogonal_gentle_up),track_type,base_dir,output_path,sprites,subtype,NULL);

	}

//Launched lift
	if(groups&TRACK_GROUP_LAUNCHED_LIFTS)
	{
	sprintf(output_path,"%.255spowered_lift%s",output_dir,suffix);
	write_track_section(context,&(track_list.launched_lift),track_type,base_dir,output_path,sprites,subtype,NULL);
	}

return 0;
}

int write_track_type(context_t* context,track_type_t* track_type,json_t* sprites,const char* base_dir,const char* output_dir)
{
track_list_t track_list=track_list_default;
	if(track_type->flags&TRACK_SEMI_SPLIT)track_list=track_list_semi_split;
	else if(track_type->flags&TRACK_SPLIT)track_list=track_list_split;

write_track_subtype(context,track_type,track_list,sprites,base_dir,output_dir,TRACK_SUBTYPE_DEFAULT);
	if(track_type->flags&TRACK_HAS_LIFT)
	{
	write_track_subtype(context,track_type,track_list,sprites,base_dir,output_dir,TRACK_SUBTYPE_LIFT);
	}
return 0;
}


