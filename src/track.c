#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "track.h"
#include "sprites.h"

#define track_base (12.375*0.0254)
#define support_top (18*0.0254)
const float pivot=0.5*(track_base+support_top);

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
track_point.position.y+=z_offset-2*CLEARANCE_HEIGHT;
	if(!(flags&TRACK_VERTICAL))track_point.position.z-=0.5*TILE_SIZE;
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
	case TRACK_SPECIAL_BRAKE:
		return MODEL_SPECIAL_BRAKE;
	break;
	case TRACK_SPECIAL_BLOCK_BRAKE:
		return MODEL_SPECIAL_BLOCK_BRAKE;
	break;
	case TRACK_SPECIAL_BOOSTER:
	case TRACK_SPECIAL_LAUNCHED_LIFT:
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

float length=scale*track_type->length;
	if(extrude_behind)num_meshes++;//TODO make extruded section straight
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
args.offset=-length-1e-5;
args.z_offset=z_offset;
args.track_curve=track_section->curve;
args.flags=track_section->flags;
args.length=track_section->length;
	if(track_mask)context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);//);
	else if(!extrude_behind)context_add_model_transformed(context,mesh,track_transform,&args,MESH_GHOST);
args.offset=track_section->length+1e-5;
	if(track_mask)context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);//track_mask?0:MESH_GHOST);
	else context_add_model_transformed(context,mesh,track_transform,&args,MESH_GHOST);

	for(int i=0;i<num_meshes;i++)
	{
	track_transform_args_t args;
	args.scale=scale;
	args.offset=(i-(extrude_behind?1:0))*length;
	args.z_offset=z_offset;
	args.track_curve=track_section->curve;
	args.flags=track_section->flags;
	args.length=track_section->length;
		

     		if(track_mask)context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);
	context_add_model_transformed(context,mesh,track_transform,&args,track_mask); 
		if((track_type->models_loaded&(1<<MODEL_BASE))&&(track_type->flags&TRACK_HAS_SUPPORTS)&&!(track_section->flags&TRACK_NO_SUPPORTS))context_add_model_transformed(context,&(track_type->models[MODEL_BASE]),base_transform,&args,track_mask); 
	}

	


	if(track_section->flags&TRACK_SPECIAL_MASK)
	{
	int index=get_special_index(track_section->flags);
		if(track_type->models_loaded&(1<<index))
		{
		matrix_t mat=views[1];
		mat.entries[6]*=-1; //Flip so faces point the right way - TODO sort out the coordinate system
			if((track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_VERTICAL_TWIST_RIGHT&&(track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_BARREL_ROLL_RIGHT&&(track_section->flags&TRACK_SPECIAL_MASK)!=TRACK_SPECIAL_CORKSCREW_RIGHT)
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
		context_add_model(context,&(track_type->models[index]),transform(mat,vector3(!(track_section->flags&TRACK_VERTICAL)?-0.5*TILE_SIZE:0,z_offset-2*CLEARANCE_HEIGHT,0)),track_mask); 
		}
	}

	if((track_type->flags&TRACK_HAS_SUPPORTS)&&!(track_section->flags&TRACK_NO_SUPPORTS))
	{
	//float support_spacing=0.8*TILE_SIZE;
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
		translation.y-=pivot/sqrt(track_point.tangent.x*track_point.tangent.x+track_point.tangent.z*track_point.tangent.z)-pivot;

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

void write_track_section(context_t* context,track_section_t* track_section,track_type_t* track_type,const char* base_directory,const char* filename,json_t* sprites,int subtype,image_t* overlay)
{
int z_offset=(int)(track_type->z_offset+0.499999);
image_t full_sprites[4];
	if(track_section->flags&TRACK_EXTRUDE_BEHIND)
	{
	render_track_section(context,track_section,track_type,1,0,0x5,full_sprites,subtype);
	render_track_section(context,track_section,track_type,0,0,0xA,full_sprites,subtype);
	}
	else render_track_section(context,track_section,track_type,0,0,0xF,full_sprites,subtype);

	if(overlay!=NULL)
	{
		for(int i=0;i<4;i++)image_blit(full_sprites+i,overlay+i,0,13-z_offset);//RMC was 14, make this a parameter
	}

image_t track_masks[4];
int track_mask_views=0;
	for(int i=0;i<4;i++)track_mask_views|=(track_section->views[i].flags&VIEW_NEEDS_TRACK_MASK?1:0)<<i;
	if(track_mask_views!=0)render_track_section(context,track_section,track_type,0,1,track_mask_views,track_masks,subtype);




	for(int angle=0;angle<4;angle++)
	{
		if(track_section->views[angle].num_sprites==0)continue;

	view_t* view=track_section->views+angle;

		
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
	if((groups&TRACK_GROUP_SLOPED_TURNS)&&(groups&TRACK_GROUP_STEEP_SLOPES))
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
write_track_subtype(context,track_type,track_list_default,sprites,base_dir,output_dir,TRACK_SUBTYPE_DEFAULT);
	if(track_type->flags&TRACK_HAS_LIFT)
	{
	write_track_subtype(context,track_type,track_list_default,sprites,base_dir,output_dir,TRACK_SUBTYPE_LIFT);
	}
return 0;
}



//       y
// z     .     x
//    .  .  .
//       .
//


matrix_t track_point_get_rotation(track_point_t point)
{
return matrix(
point.tangent.z,point.normal.z,-point.binormal.z,
point.tangent.y,point.normal.y,-point.binormal.y,
point.tangent.x,point.normal.x,-point.binormal.x);

}


typedef struct
{
int yaw_sprite;
int pitch_sprite;
int bank_sprite;
float yaw;
float pitch;
float roll;
}sprite_rotation_t;

#define Y(i) (M_PI*i/16.0)

#define M_PI_6 (M_PI/6.0)
#define M_PI_12 (M_PI/12.0)

#define CORKSCREW_ANGLE_1 2.0 * M_PI_12
#define CORKSCREW_ANGLE_2 4.0 * M_PI_12
#define CORKSCREW_ANGLE_3 M_PI_2
#define CORKSCREW_ANGLE_4 8.0 * M_PI_12
#define CORKSCREW_ANGLE_5 10.0 * M_PI_12
#define G atan(1/sqrt(6))
#define S atan(4/sqrt(6))
#define V (0.5*M_PI)
#define FG (0.5*G)
#define GS (0.5*(G+S))
#define SV (0.5*(V+S))

#define CRY(angle) (atan2(0.5*(1-cos(angle)),1-0.5*(1-cos(angle))))
#define CRP(angle) (-asin(-sin(angle)/sqrt(2.0)))
#define CRR(angle) (-atan2(sin(angle)/sqrt(2.0),cos(angle)))

#define CLY(angle) (-CRY(angle))
#define CLP(angle) (-CRP(-angle))
#define CLR(angle) (-CRR(angle))


sprite_rotation_t sprite_rotations[]=
	{
	{ 0,0,0,Y( 0),0.0,0.0},
	{ 1,0,0,Y( 1),0.0,0.0},
	{ 2,0,0,Y( 2),0.0,0.0},
	{ 3,0,0,Y( 3),0.0,0.0},
	{ 4,0,0,Y( 4),0.0,0.0},
	{ 5,0,0,Y( 5),0.0,0.0},
	{ 6,0,0,Y( 6),0.0,0.0},
	{ 7,0,0,Y( 7),0.0,0.0},
	{ 8,0,0,Y( 8),0.0,0.0},
	{ 9,0,0,Y( 9),0.0,0.0},
	{10,0,0,Y(10),0.0,0.0},
	{11,0,0,Y(11),0.0,0.0},
	{12,0,0,Y(12),0.0,0.0},
	{13,0,0,Y(13),0.0,0.0},
	{14,0,0,Y(14),0.0,0.0},
	{15,0,0,Y(15),0.0,0.0},
	{16,0,0,Y(16),0.0,0.0},
	{17,0,0,Y(17),0.0,0.0},
	{18,0,0,Y(18),0.0,0.0},
	{19,0,0,Y(19),0.0,0.0},
	{20,0,0,Y(20),0.0,0.0},
	{21,0,0,Y(21),0.0,0.0},
	{22,0,0,Y(22),0.0,0.0},
	{23,0,0,Y(23),0.0,0.0},
	{24,0,0,Y(24),0.0,0.0},
	{25,0,0,Y(25),0.0,0.0},
	{26,0,0,Y(26),0.0,0.0},
	{27,0,0,Y(27),0.0,0.0},
	{28,0,0,Y(28),0.0,0.0},
	{29,0,0,Y(29),0.0,0.0},
	{30,0,0,Y(30),0.0,0.0},
	{31,0,0,Y(31),0.0,0.0},
	{ 0,1,0,Y( 0), FG,0.0},
	{ 8,1,0,Y( 8), FG,0.0},
	{16,1,0,Y(16), FG,0.0},
	{24,1,0,Y(24), FG,0.0},
	{ 0,5,0,Y( 0),-FG,0.0},
	{ 8,5,0,Y( 8),-FG,0.0},
	{16,5,0,Y(16),-FG,0.0},
	{24,5,0,Y(24),-FG,0.0},
	{ 0,2,0,Y( 0),G,0.0},
	{ 1,2,0,Y( 1),G,0.0},
	{ 2,2,0,Y( 2),G,0.0},
	{ 3,2,0,Y( 3),G,0.0},
	{ 4,2,0,Y( 4),G,0.0},
	{ 5,2,0,Y( 5),G,0.0},
	{ 6,2,0,Y( 6),G,0.0},
	{ 7,2,0,Y( 7),G,0.0},
	{ 8,2,0,Y( 8),G,0.0},
	{ 9,2,0,Y( 9),G,0.0},
	{10,2,0,Y(10),G,0.0},
	{11,2,0,Y(11),G,0.0},
	{12,2,0,Y(12),G,0.0},
	{13,2,0,Y(13),G,0.0},
	{14,2,0,Y(14),G,0.0},
	{15,2,0,Y(15),G,0.0},
	{16,2,0,Y(16),G,0.0},
	{17,2,0,Y(17),G,0.0},
	{18,2,0,Y(18),G,0.0},
	{19,2,0,Y(19),G,0.0},
	{20,2,0,Y(20),G,0.0},
	{21,2,0,Y(21),G,0.0},
	{22,2,0,Y(22),G,0.0},
	{23,2,0,Y(23),G,0.0},
	{24,2,0,Y(24),G,0.0},
	{25,2,0,Y(25),G,0.0},
	{26,2,0,Y(26),G,0.0},
	{27,2,0,Y(27),G,0.0},
	{28,2,0,Y(28),G,0.0},
	{29,2,0,Y(29),G,0.0},
	{30,2,0,Y(30),G,0.0},
	{31,2,0,Y(31),G,0.0},
	{ 0,6,0,Y( 0),-G,0.0},
	{ 1,6,0,Y( 1),-G,0.0},
	{ 2,6,0,Y( 2),-G,0.0},
	{ 3,6,0,Y( 3),-G,0.0},
	{ 4,6,0,Y( 4),-G,0.0},
	{ 5,6,0,Y( 5),-G,0.0},
	{ 6,6,0,Y( 6),-G,0.0},
	{ 7,6,0,Y( 7),-G,0.0},
	{ 8,6,0,Y( 8),-G,0.0},
	{ 9,6,0,Y( 9),-G,0.0},
	{10,6,0,Y(10),-G,0.0},
	{11,6,0,Y(11),-G,0.0},
	{12,6,0,Y(12),-G,0.0},
	{13,6,0,Y(13),-G,0.0},
	{14,6,0,Y(14),-G,0.0},
	{15,6,0,Y(15),-G,0.0},
	{16,6,0,Y(16),-G,0.0},
	{17,6,0,Y(17),-G,0.0},
	{18,6,0,Y(18),-G,0.0},
	{19,6,0,Y(19),-G,0.0},
	{20,6,0,Y(20),-G,0.0},
	{21,6,0,Y(21),-G,0.0},
	{22,6,0,Y(22),-G,0.0},
	{23,6,0,Y(23),-G,0.0},
	{24,6,0,Y(24),-G,0.0},
	{25,6,0,Y(25),-G,0.0},
	{26,6,0,Y(26),-G,0.0},
	{27,6,0,Y(27),-G,0.0},
	{28,6,0,Y(28),-G,0.0},
	{29,6,0,Y(29),-G,0.0},
	{30,6,0,Y(30),-G,0.0},
	{31,6,0,Y(31),-G,0.0},
	{ 0,3,0,Y( 0), GS,0.0},
	{ 8,3,0,Y( 8), GS,0.0},
	{16,3,0,Y(16), GS,0.0},
	{24,3,0,Y(24), GS,0.0},
	{ 0,7,0,Y( 0),-GS,0.0},
	{ 8,7,0,Y( 8),-GS,0.0},
	{16,7,0,Y(16),-GS,0.0},
	{24,7,0,Y(24),-GS,0.0},
	{ 0,4,0,Y( 0),S,0.0},
	{ 1,4,0,Y( 1),S,0.0},
	{ 2,4,0,Y( 2),S,0.0},
	{ 3,4,0,Y( 3),S,0.0},
	{ 4,4,0,Y( 4),S,0.0},
	{ 5,4,0,Y( 5),S,0.0},
	{ 6,4,0,Y( 6),S,0.0},
	{ 7,4,0,Y( 7),S,0.0},
	{ 8,4,0,Y( 8),S,0.0},
	{ 9,4,0,Y( 9),S,0.0},
	{10,4,0,Y(10),S,0.0},
	{11,4,0,Y(11),S,0.0},
	{12,4,0,Y(12),S,0.0},
	{13,4,0,Y(13),S,0.0},
	{14,4,0,Y(14),S,0.0},
	{15,4,0,Y(15),S,0.0},
	{16,4,0,Y(16),S,0.0},
	{17,4,0,Y(17),S,0.0},
	{18,4,0,Y(18),S,0.0},
	{19,4,0,Y(19),S,0.0},
	{20,4,0,Y(20),S,0.0},
	{21,4,0,Y(21),S,0.0},
	{22,4,0,Y(22),S,0.0},
	{23,4,0,Y(23),S,0.0},
	{24,4,0,Y(24),S,0.0},
	{25,4,0,Y(25),S,0.0},
	{26,4,0,Y(26),S,0.0},
	{27,4,0,Y(27),S,0.0},
	{28,4,0,Y(28),S,0.0},
	{29,4,0,Y(29),S,0.0},
	{30,4,0,Y(30),S,0.0},
	{31,4,0,Y(31),S,0.0},
	{ 0,8,0,Y( 0),-S,0.0},
	{ 1,8,0,Y( 1),-S,0.0},
	{ 2,8,0,Y( 2),-S,0.0},
	{ 3,8,0,Y( 3),-S,0.0},
	{ 4,8,0,Y( 4),-S,0.0},
	{ 5,8,0,Y( 5),-S,0.0},
	{ 6,8,0,Y( 6),-S,0.0},
	{ 7,8,0,Y( 7),-S,0.0},
	{ 8,8,0,Y( 8),-S,0.0},
	{ 9,8,0,Y( 9),-S,0.0},
	{10,8,0,Y(10),-S,0.0},
	{11,8,0,Y(11),-S,0.0},
	{12,8,0,Y(12),-S,0.0},
	{13,8,0,Y(13),-S,0.0},
	{14,8,0,Y(14),-S,0.0},
	{15,8,0,Y(15),-S,0.0},
	{16,8,0,Y(16),-S,0.0},
	{17,8,0,Y(17),-S,0.0},
	{18,8,0,Y(18),-S,0.0},
	{19,8,0,Y(19),-S,0.0},
	{20,8,0,Y(20),-S,0.0},
	{21,8,0,Y(21),-S,0.0},
	{22,8,0,Y(22),-S,0.0},
	{23,8,0,Y(23),-S,0.0},
	{24,8,0,Y(24),-S,0.0},
	{25,8,0,Y(25),-S,0.0},
	{26,8,0,Y(26),-S,0.0},
	{27,8,0,Y(27),-S,0.0},
	{28,8,0,Y(28),-S,0.0},
	{29,8,0,Y(29),-S,0.0},
	{30,8,0,Y(30),-S,0.0},
	{31,8,0,Y(31),-S,0.0},
	{ 0,9,0,Y( 0), SV,0.0},
	{ 8,9,0,Y( 8), SV,0.0},
	{16,9,0,Y(16), SV,0.0},
	{24,9,0,Y(24), SV,0.0},
	{ 0,17,0,Y( 0),-SV,0.0},
	{ 8,17,0,Y( 8),-SV,0.0},
	{16,17,0,Y(16),-SV,0.0},
	{24,17,0,Y(24),-SV,0.0},
	{ 0,10,0,Y( 0),V,0.0},
	{ 1,10,0,Y( 1),V,0.0},
	{ 2,10,0,Y( 2),V,0.0},
	{ 3,10,0,Y( 3),V,0.0},
	{ 4,10,0,Y( 4),V,0.0},
	{ 5,10,0,Y( 5),V,0.0},
	{ 6,10,0,Y( 6),V,0.0},
	{ 7,10,0,Y( 7),V,0.0},
	{ 8,10,0,Y( 8),V,0.0},
	{ 9,10,0,Y( 9),V,0.0},
	{10,10,0,Y(10),V,0.0},
	{11,10,0,Y(11),V,0.0},
	{12,10,0,Y(12),V,0.0},
	{13,10,0,Y(13),V,0.0},
	{14,10,0,Y(14),V,0.0},
	{15,10,0,Y(15),V,0.0},
	{16,10,0,Y(16),V,0.0},
	{17,10,0,Y(17),V,0.0},
	{18,10,0,Y(18),V,0.0},
	{19,10,0,Y(19),V,0.0},
	{20,10,0,Y(20),V,0.0},
	{21,10,0,Y(21),V,0.0},
	{22,10,0,Y(22),V,0.0},
	{23,10,0,Y(23),V,0.0},
	{24,10,0,Y(24),V,0.0},
	{25,10,0,Y(25),V,0.0},
	{26,10,0,Y(26),V,0.0},
	{27,10,0,Y(27),V,0.0},
	{28,10,0,Y(28),V,0.0},
	{29,10,0,Y(29),V,0.0},
	{30,10,0,Y(30),V,0.0},
	{31,10,0,Y(31),V,0.0},
	{ 0,18,0,Y( 0),-V,0.0},
	{ 1,18,0,Y( 1),-V,0.0},
	{ 2,18,0,Y( 2),-V,0.0},
	{ 3,18,0,Y( 3),-V,0.0},
	{ 4,18,0,Y( 4),-V,0.0},
	{ 5,18,0,Y( 5),-V,0.0},
	{ 6,18,0,Y( 6),-V,0.0},
	{ 7,18,0,Y( 7),-V,0.0},
	{ 8,18,0,Y( 8),-V,0.0},
	{ 9,18,0,Y( 9),-V,0.0},
	{10,18,0,Y(10),-V,0.0},
	{11,18,0,Y(11),-V,0.0},
	{12,18,0,Y(12),-V,0.0},
	{13,18,0,Y(13),-V,0.0},
	{14,18,0,Y(14),-V,0.0},
	{15,18,0,Y(15),-V,0.0},
	{16,18,0,Y(16),-V,0.0},
	{17,18,0,Y(17),-V,0.0},
	{18,18,0,Y(18),-V,0.0},
	{19,18,0,Y(19),-V,0.0},
	{20,18,0,Y(20),-V,0.0},
	{21,18,0,Y(21),-V,0.0},
	{22,18,0,Y(22),-V,0.0},
	{23,18,0,Y(23),-V,0.0},
	{24,18,0,Y(24),-V,0.0},
	{25,18,0,Y(25),-V,0.0},
	{26,18,0,Y(26),-V,0.0},
	{27,18,0,Y(27),-V,0.0},
	{28,18,0,Y(28),-V,0.0},
	{29,18,0,Y(29),-V,0.0},
	{30,18,0,Y(30),-V,0.0},
	{31,18,0,Y(31),-V,0.0},
	{ 0,11,0,Y( 0),V+1*M_PI_12,0.0},
	{ 8,11,0,Y( 8),V+1*M_PI_12,0.0},
	{16,11,0,Y(16),V+1*M_PI_12,0.0},
	{24,11,0,Y(24),V+1*M_PI_12,0.0},
	{ 0,12,0,Y( 0),V+2*M_PI_12,0.0},
	{ 8,12,0,Y( 8),V+2*M_PI_12,0.0},
	{16,12,0,Y(16),V+2*M_PI_12,0.0},
	{24,12,0,Y(24),V+2*M_PI_12,0.0},
	{ 0,13,0,Y( 0),V+3*M_PI_12,0.0},
	{ 8,13,0,Y( 8),V+3*M_PI_12,0.0},
	{16,13,0,Y(16),V+3*M_PI_12,0.0},
	{24,13,0,Y(24),V+3*M_PI_12,0.0},
	{ 0,14,0,Y( 0),V+4*M_PI_12,0.0},
	{ 8,14,0,Y( 8),V+4*M_PI_12,0.0},
	{16,14,0,Y(16),V+4*M_PI_12,0.0},
	{24,14,0,Y(24),V+4*M_PI_12,0.0},
	{ 0,15,0,Y( 0),V+5*M_PI_12,0.0},
	{ 8,15,0,Y( 8),V+5*M_PI_12,0.0},
	{16,15,0,Y(16),V+5*M_PI_12,0.0},
	{24,15,0,Y(24),V+5*M_PI_12,0.0},
	{ 0,16,0,Y( 0),V+6*M_PI_12,0.0},
	{ 8,16,0,Y( 8),V+6*M_PI_12,0.0},
	{16,16,0,Y(16),V+6*M_PI_12,0.0},
	{24,16,0,Y(24),V+6*M_PI_12,0.0},
	{ 0,19,0,Y( 0),-V-1*M_PI_12,0.0},
	{ 8,19,0,Y( 8),-V-1*M_PI_12,0.0},
	{16,19,0,Y(16),-V-1*M_PI_12,0.0},
	{24,19,0,Y(24),-V-1*M_PI_12,0.0},
	{ 0,20,0,Y( 0),-V-2*M_PI_12,0.0},
	{ 8,20,0,Y( 8),-V-2*M_PI_12,0.0},
	{16,20,0,Y(16),-V-2*M_PI_12,0.0},
	{24,20,0,Y(24),-V-2*M_PI_12,0.0},
	{ 0,21,0,Y( 0),-V-3*M_PI_12,0.0},
	{ 8,21,0,Y( 8),-V-3*M_PI_12,0.0},
	{16,21,0,Y(16),-V-3*M_PI_12,0.0},
	{24,21,0,Y(24),-V-3*M_PI_12,0.0},
	{ 0,22,0,Y( 0),-V-4*M_PI_12,0.0},
	{ 8,22,0,Y( 8),-V-4*M_PI_12,0.0},
	{16,22,0,Y(16),-V-4*M_PI_12,0.0},
	{24,22,0,Y(24),-V-4*M_PI_12,0.0},
	{ 0,23,0,Y( 0),-V-5*M_PI_12,0.0},
	{ 8,23,0,Y( 8),-V-5*M_PI_12,0.0},
	{16,23,0,Y(16),-V-5*M_PI_12,0.0},
	{24,23,0,Y(24),-V-5*M_PI_12,0.0},
	{ 0,34,0,Y( 0)+CRY(1*M_PI_6),CRP(1*M_PI_6),CRR(1*M_PI_6)},
	{ 8,34,0,Y( 8)+CRY(1*M_PI_6),CRP(1*M_PI_6),CRR(1*M_PI_6)},
	{16,34,0,Y(16)+CRY(1*M_PI_6),CRP(1*M_PI_6),CRR(1*M_PI_6)},
	{24,34,0,Y(24)+CRY(1*M_PI_6),CRP(1*M_PI_6),CRR(1*M_PI_6)},
	{ 0,35,0,Y( 0)+CRY(2*M_PI_6),CRP(2*M_PI_6),CRR(2*M_PI_6)},
	{ 8,35,0,Y( 8)+CRY(2*M_PI_6),CRP(2*M_PI_6),CRR(2*M_PI_6)},
	{16,35,0,Y(16)+CRY(2*M_PI_6),CRP(2*M_PI_6),CRR(2*M_PI_6)},
	{24,35,0,Y(24)+CRY(2*M_PI_6),CRP(2*M_PI_6),CRR(2*M_PI_6)},
	{ 0,36,0,Y( 0)+CRY(3*M_PI_6),CRP(3*M_PI_6),CRR(3*M_PI_6)},
	{ 8,36,0,Y( 8)+CRY(3*M_PI_6),CRP(3*M_PI_6),CRR(3*M_PI_6)},
	{16,36,0,Y(16)+CRY(3*M_PI_6),CRP(3*M_PI_6),CRR(3*M_PI_6)},
	{24,36,0,Y(24)+CRY(3*M_PI_6),CRP(3*M_PI_6),CRR(3*M_PI_6)},
	{ 0,37,0,Y( 0)+CRY(4*M_PI_6),CRP(4*M_PI_6),CRR(4*M_PI_6)},
	{ 8,37,0,Y( 8)+CRY(4*M_PI_6),CRP(4*M_PI_6),CRR(4*M_PI_6)},
	{16,37,0,Y(16)+CRY(4*M_PI_6),CRP(4*M_PI_6),CRR(4*M_PI_6)},
	{24,37,0,Y(24)+CRY(4*M_PI_6),CRP(4*M_PI_6),CRR(4*M_PI_6)},
	{ 0,38,0,Y( 0)+CRY(5*M_PI_6),CRP(5*M_PI_6),CRR(5*M_PI_6)},
	{ 8,38,0,Y( 8)+CRY(5*M_PI_6),CRP(5*M_PI_6),CRR(5*M_PI_6)},
	{16,38,0,Y(16)+CRY(5*M_PI_6),CRP(5*M_PI_6),CRR(5*M_PI_6)},
	{24,38,0,Y(24)+CRY(5*M_PI_6),CRP(5*M_PI_6),CRR(5*M_PI_6)},
	{ 0,24,0,Y( 0)+CLY(1*M_PI_6),CLP(1*M_PI_6),CLR(1*M_PI_6)},
	{ 8,24,0,Y( 8)+CLY(1*M_PI_6),CLP(1*M_PI_6),CLR(1*M_PI_6)},
	{16,24,0,Y(16)+CLY(1*M_PI_6),CLP(1*M_PI_6),CLR(1*M_PI_6)},
	{24,24,0,Y(24)+CLY(1*M_PI_6),CLP(1*M_PI_6),CLR(1*M_PI_6)},
	{ 0,25,0,Y( 0)+CLY(2*M_PI_6),CLP(2*M_PI_6),CLR(2*M_PI_6)},
	{ 8,25,0,Y( 8)+CLY(2*M_PI_6),CLP(2*M_PI_6),CLR(2*M_PI_6)},
	{16,25,0,Y(16)+CLY(2*M_PI_6),CLP(2*M_PI_6),CLR(2*M_PI_6)},
	{24,25,0,Y(24)+CLY(2*M_PI_6),CLP(2*M_PI_6),CLR(2*M_PI_6)},
	{ 0,26,0,Y( 0)+CLY(3*M_PI_6),CLP(3*M_PI_6),CLR(3*M_PI_6)},
	{ 8,26,0,Y( 8)+CLY(3*M_PI_6),CLP(3*M_PI_6),CLR(3*M_PI_6)},
	{16,26,0,Y(16)+CLY(3*M_PI_6),CLP(3*M_PI_6),CLR(3*M_PI_6)},
	{24,26,0,Y(24)+CLY(3*M_PI_6),CLP(3*M_PI_6),CLR(3*M_PI_6)},
	{ 0,27,0,Y( 0)+CLY(4*M_PI_6),CLP(4*M_PI_6),CLR(4*M_PI_6)},
	{ 8,27,0,Y( 8)+CLY(4*M_PI_6),CLP(4*M_PI_6),CLR(4*M_PI_6)},
	{16,27,0,Y(16)+CLY(4*M_PI_6),CLP(4*M_PI_6),CLR(4*M_PI_6)},
	{24,27,0,Y(24)+CLY(4*M_PI_6),CLP(4*M_PI_6),CLR(4*M_PI_6)},
	{ 0,28,0,Y( 0)+CLY(5*M_PI_6),CLP(5*M_PI_6),CLR(5*M_PI_6)},
	{ 8,28,0,Y( 8)+CLY(5*M_PI_6),CLP(5*M_PI_6),CLR(5*M_PI_6)},
	{16,28,0,Y(16)+CLY(5*M_PI_6),CLP(5*M_PI_6),CLR(5*M_PI_6)},
	{24,28,0,Y(24)+CLY(5*M_PI_6),CLP(5*M_PI_6),CLR(5*M_PI_6)},
	{ 0,39,0,Y( 0)+CRY(-1*M_PI_6),CRP(-1*M_PI_6),CRR(-1*M_PI_6)},
	{ 8,39,0,Y( 8)+CRY(-1*M_PI_6),CRP(-1*M_PI_6),CRR(-1*M_PI_6)},
	{16,39,0,Y(16)+CRY(-1*M_PI_6),CRP(-1*M_PI_6),CRR(-1*M_PI_6)},
	{24,39,0,Y(24)+CRY(-1*M_PI_6),CRP(-1*M_PI_6),CRR(-1*M_PI_6)},
	{ 0,40,0,Y( 0)+CRY(-2*M_PI_6),CRP(-2*M_PI_6),CRR(-2*M_PI_6)},
	{ 8,40,0,Y( 8)+CRY(-2*M_PI_6),CRP(-2*M_PI_6),CRR(-2*M_PI_6)},
	{16,40,0,Y(16)+CRY(-2*M_PI_6),CRP(-2*M_PI_6),CRR(-2*M_PI_6)},
	{24,40,0,Y(24)+CRY(-2*M_PI_6),CRP(-2*M_PI_6),CRR(-2*M_PI_6)},
	{ 0,41,0,Y( 0)+CRY(-3*M_PI_6),CRP(-3*M_PI_6),CRR(-3*M_PI_6)},
	{ 8,41,0,Y( 8)+CRY(-3*M_PI_6),CRP(-3*M_PI_6),CRR(-3*M_PI_6)},
	{16,41,0,Y(16)+CRY(-3*M_PI_6),CRP(-3*M_PI_6),CRR(-3*M_PI_6)},
	{24,41,0,Y(24)+CRY(-3*M_PI_6),CRP(-3*M_PI_6),CRR(-3*M_PI_6)},
	{ 0,42,0,Y( 0)+CRY(-4*M_PI_6),CRP(-4*M_PI_6),CRR(-4*M_PI_6)},
	{ 8,42,0,Y( 8)+CRY(-4*M_PI_6),CRP(-4*M_PI_6),CRR(-4*M_PI_6)},
	{16,42,0,Y(16)+CRY(-4*M_PI_6),CRP(-4*M_PI_6),CRR(-4*M_PI_6)},
	{24,42,0,Y(24)+CRY(-4*M_PI_6),CRP(-4*M_PI_6),CRR(-4*M_PI_6)},
	{ 0,43,0,Y( 0)+CRY(-5*M_PI_6),CRP(-5*M_PI_6),CRR(-5*M_PI_6)},
	{ 8,43,0,Y( 8)+CRY(-5*M_PI_6),CRP(-5*M_PI_6),CRR(-5*M_PI_6)},
	{16,43,0,Y(16)+CRY(-5*M_PI_6),CRP(-5*M_PI_6),CRR(-5*M_PI_6)},
	{24,43,0,Y(24)+CRY(-5*M_PI_6),CRP(-5*M_PI_6),CRR(-5*M_PI_6)},
	{ 0,29,0,Y( 0)+CLY(-1*M_PI_6),CLP(-1*M_PI_6),CLR(-1*M_PI_6)},
	{ 8,29,0,Y( 8)+CLY(-1*M_PI_6),CLP(-1*M_PI_6),CLR(-1*M_PI_6)},
	{16,29,0,Y(16)+CLY(-1*M_PI_6),CLP(-1*M_PI_6),CLR(-1*M_PI_6)},
	{24,29,0,Y(24)+CLY(-1*M_PI_6),CLP(-1*M_PI_6),CLR(-1*M_PI_6)},
	{ 0,30,0,Y( 0)+CLY(-2*M_PI_6),CLP(-2*M_PI_6),CLR(-2*M_PI_6)},
	{ 8,30,0,Y( 8)+CLY(-2*M_PI_6),CLP(-2*M_PI_6),CLR(-2*M_PI_6)},
	{16,30,0,Y(16)+CLY(-2*M_PI_6),CLP(-2*M_PI_6),CLR(-2*M_PI_6)},
	{24,30,0,Y(24)+CLY(-2*M_PI_6),CLP(-2*M_PI_6),CLR(-2*M_PI_6)},
	{ 0,31,0,Y( 0)+CLY(-3*M_PI_6),CLP(-3*M_PI_6),CLR(-3*M_PI_6)},
	{ 8,31,0,Y( 8)+CLY(-3*M_PI_6),CLP(-3*M_PI_6),CLR(-3*M_PI_6)},
	{16,31,0,Y(16)+CLY(-3*M_PI_6),CLP(-3*M_PI_6),CLR(-3*M_PI_6)},
	{24,31,0,Y(24)+CLY(-3*M_PI_6),CLP(-3*M_PI_6),CLR(-3*M_PI_6)},
	{ 0,32,0,Y( 0)+CLY(-4*M_PI_6),CLP(-4*M_PI_6),CLR(-4*M_PI_6)},
	{ 8,32,0,Y( 8)+CLY(-4*M_PI_6),CLP(-4*M_PI_6),CLR(-4*M_PI_6)},
	{16,32,0,Y(16)+CLY(-4*M_PI_6),CLP(-4*M_PI_6),CLR(-4*M_PI_6)},
	{24,32,0,Y(24)+CLY(-4*M_PI_6),CLP(-4*M_PI_6),CLR(-4*M_PI_6)},
	{ 0,33,0,Y( 0)+CLY(-5*M_PI_6),CLP(-5*M_PI_6),CLR(-5*M_PI_6)},
	{ 8,33,0,Y( 8)+CLY(-5*M_PI_6),CLP(-5*M_PI_6),CLR(-5*M_PI_6)},
	{16,33,0,Y(16)+CLY(-5*M_PI_6),CLP(-5*M_PI_6),CLR(-5*M_PI_6)},
	{24,33,0,Y(24)+CLY(-5*M_PI_6),CLP(-5*M_PI_6),CLR(-5*M_PI_6)},
};

float get_rotation_distance(matrix_t a,matrix_t b)
{
matrix_t diff=matrix_mult(a,matrix_transpose(b));
return acos((diff.entries[0]+diff.entries[4]+diff.entries[8]-1.0)/2.0);
}

sprite_rotation_t get_closest_rotation(matrix_t rotation)
{
float min_dist=1000.0;
int min_index=0;
	for(int i=0;i<374;i++)
	{
	matrix_t candidate_rotation=matrix_mult(matrix_mult(rotate_y(-sprite_rotations[i].yaw),rotate_z(sprite_rotations[i].pitch)),rotate_x(-sprite_rotations[i].roll));
	float dist=get_rotation_distance(rotation,candidate_rotation);
		if(dist<min_dist)
		{
		min_dist=dist;
		min_index=i;
		}
	}
return sprite_rotations[min_index];
}





//Generate the subposition data for a track piece
void generate_view_subposition_data(track_section_t* track_section,char* name,int view,int reverse)
{
int length=(int)floor(0.5+32.0*(track_section->length/TILE_SIZE));


track_point_t end=track_section->curve(track_section->length);
float finish_angle=roundf(2.0*atan2(-end.tangent.x,-end.tangent.z)/M_PI);
//printf("Angle %f\n",0.5*finish_angle*M_PI);
matrix_t reverse_transform=rotate_y(finish_angle);
vector3_t reverse_offset=end.position;

printf("CREATE_VEHICLE_INFO(TrackVehicleInfo%s%d, {\n",name,view);
	for(int i=0;i<length;i+=1)
	{
	track_point_t point;
		if(!reverse)point=track_section->curve(TILE_SIZE*(i+(view==0||view==3))/32.0);
		else
		{
		point=track_section->curve(TILE_SIZE*(length-(i+(view==0||view==3)))/32.0);
		point.position=matrix_vector(reverse_transform,vector3_sub(point.position,reverse_offset));
		point.tangent=vector3_mult(matrix_vector(reverse_transform,point.tangent),-1.0);
		point.normal=matrix_vector(reverse_transform,point.normal);
		point.binormal=vector3_mult(matrix_vector(reverse_transform,point.binormal),-1.0);
		}
	int x=(int)floor(0.5+32.0*point.position.z/TILE_SIZE);
	int y=(int)floor(0.5+32.0*point.position.x/TILE_SIZE);
	int z=(int)floor(0.5+(16*sqrt(6))*point.position.y/TILE_SIZE)+reverse;
	//matrix_t r=rotate_y(0.5*M_PI);
	//printf("%.2f\t%.2f %.2f\n",r.entries[0],r.entries[1],r.entries[2]);
	//printf("%.2f\t%.2f %.2f\n",r.entries[3],r.entries[4],r.entries[5]);
	//printf("%.2f\t%.2f %.2f\n\n",r.entries[6],r.entries[7],r.entries[8]);
	//r=track_point_get_rotation(point);
	//printf("%.2f\t%.2f %.2f\n",r.entries[0],r.entries[1],r.entries[2]);
	//printf("%.2f\t%.2f %.2f\n",r.entries[3],r.entries[4],r.entries[5]);
	//printf("%.2f\t%.2f %.2f\n\n",r.entries[6],r.entries[7],r.entries[8]);

	sprite_rotation_t rotation=get_closest_rotation(track_point_get_rotation(point));

	
	
	int t;		
		switch(view)
		{
		case 0:
			x=(32-x);
			y=(16-y);
		break;
		case 1:
			t=x;
			x=(16-y);
			y=t;
		break;
		case 2:	
			y=(16+y);
		break;
		case 3:
			t=x;
			x=(16+y);
			y=(32-t);
		break;
		}
		if(i%5==0)printf("    ");
		//TODO the reverse rotation seems to only apply to corkscrews
	printf("{%d, %d, %d, %d, %d, %d}, ",x,y,z,(8*view+rotation.yaw_sprite+(reverse?0:0))%32,rotation.pitch_sprite,rotation.bank_sprite);
		if(i%5==4||i==length-1)putchar('\n');
	}
puts("})\n");
}

void generate_subposition_data(track_section_t* track_section,char* name,int reverse)
{
	for(int i=0;i<4;i++)
	{
	generate_view_subposition_data(track_section,name,i,reverse);
	}
}
