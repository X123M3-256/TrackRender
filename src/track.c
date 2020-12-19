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


     	if(track_mask)
	{
	track_transform_args_t args;
	args.scale=scale;
	args.offset=-length;
	args.z_offset=z_offset;
	args.track_curve=track_section->curve;
	args.flags=track_section->flags;
	args.length=track_section->length;
	context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);
	//context_add_model_transformed(context,mesh,track_transform,&args,0);
	args.offset=track_section->length;
	context_add_model_transformed(context,&(track_type->mask),track_transform,&args,0);
	//context_add_model_transformed(context,mesh,track_transform,&args,0);
	}

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
		matrix_t mat=views[3];
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

int write_track_subtype(context_t* context,track_type_t* track_type,json_t* sprites,const char* base_dir,const char* output_dir,int subtype)
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

int semi_split=track_type->flags&TRACK_SEMI_SPLIT;

//Flat
	if(groups&TRACK_GROUP_FLAT)
	{
	sprintf(output_path,"%.255sflat%s",output_dir,suffix);
	if(subtype==TRACK_SUBTYPE_LIFT)write_track_section(context,&flat_asymmetric,track_type,base_dir,output_path,sprites,subtype,flat_chain);
	else write_track_section(context,&flat,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_BRAKES)
	{
	sprintf(output_path,"%.255sbrake%s",output_dir,suffix);
	write_track_section(context,&brake,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sblock_brake%s",output_dir,suffix);
	write_track_section(context,&block_brake,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_BOOSTERS)
	{
	sprintf(output_path,"%.255sbooster%s",output_dir,suffix);
	write_track_section(context,&booster,track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Slopes
	if(groups&TRACK_GROUP_GENTLE_SLOPES)
	{
	sprintf(output_path,"%.255sflat_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&flat_to_gentle_up,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?flat_to_gentle_up_chain:NULL);
	sprintf(output_path,"%.255sgentle_up_to_flat%s",output_dir,suffix);
	write_track_section(context,&gentle_up_to_flat,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_up_to_flat_chain:NULL);
	sprintf(output_path,"%.255sgentle%s",output_dir,suffix);
	write_track_section(context,&gentle,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_chain:NULL);
	}

	if(groups&TRACK_GROUP_STEEP_SLOPES)
	{
	sprintf(output_path,"%.255sgentle_to_steep_up%s",output_dir,suffix);
	write_track_section(context,&gentle_to_steep_up,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_to_steep_up_chain:NULL);
	sprintf(output_path,"%.255ssteep_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&steep_to_gentle_up,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?steep_to_gentle_up_chain:NULL);
	sprintf(output_path,"%.255ssteep%s",output_dir,suffix);
	write_track_section(context,&steep,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?steep_chain:NULL);
	}
	if(groups&TRACK_GROUP_VERTICAL_SLOPES)
	{
	sprintf(output_path,"%.255ssteep_to_vertical_up%s",output_dir,suffix);
	write_track_section(context,&steep_to_vertical_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255svertical_to_steep_up%s",output_dir,suffix);
	write_track_section(context,&vertical_to_steep_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255svertical%s",output_dir,suffix);
	write_track_section(context,&vertical,track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Turns
	if(groups&TRACK_GROUP_TURNS)
	{
	sprintf(output_path,"%.255ssmall_turn_left%s",output_dir,suffix);
	write_track_section(context,&small_turn_left,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_left%s",output_dir,suffix);
	write_track_section(context,&medium_turn_left,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_left_to_diag%s",output_dir,suffix);
	write_track_section(context,&large_turn_left_to_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_right_to_diag%s",output_dir,suffix);
	write_track_section(context,&large_turn_right_to_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Diagonals
	if(groups&TRACK_GROUP_DIAGONALS)
	{
	sprintf(output_path,"%.255sflat_diag%s",output_dir,suffix);
	write_track_section(context,&flat_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?flat_diag_chain:NULL);
	sprintf(output_path,"%.255sflat_to_gentle_up_diag%s",output_dir,suffix);
	write_track_section(context,&flat_to_gentle_up_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?flat_to_gentle_up_diag_chain:NULL);
	sprintf(output_path,"%.255sgentle_to_flat_up_diag%s",output_dir,suffix);
	write_track_section(context,&gentle_to_flat_up_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_to_flat_up_diag_chain:NULL);
	sprintf(output_path,"%.255sgentle_diag%s",output_dir,suffix);
	write_track_section(context,&gentle_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_diag_chain:NULL);
	sprintf(output_path,"%.255sgentle_to_steep_up_diag%s",output_dir,suffix);
	write_track_section(context,&gentle_to_steep_up_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?gentle_to_steep_up_diag_chain:NULL);
	sprintf(output_path,"%.255ssteep_to_gentle_up_diag%s",output_dir,suffix);
	write_track_section(context,&steep_to_gentle_up_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?steep_to_gentle_up_diag_chain:NULL);
	sprintf(output_path,"%.255ssteep_diag%s",output_dir,suffix);
	write_track_section(context,&steep_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?steep_diag_chain:NULL);
	}

//Banked turns
	if(groups&TRACK_GROUP_BANKED_TURNS)
	{
	sprintf(output_path,"%.255sflat_to_left_bank%s",output_dir,suffix);
	write_track_section(context,&flat_to_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sflat_to_right_bank%s",output_dir,suffix);
	write_track_section(context,&flat_to_right_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sleft_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&left_bank_to_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sright_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&right_bank_to_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_to_left_bank%s",output_dir,suffix);
	write_track_section(context,&gentle_up_to_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_to_right_bank%s",output_dir,suffix);
	write_track_section(context,&gentle_up_to_right_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	
	sprintf(output_path,"%.255sleft_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_left_bank:&left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);

		if(groups&TRACK_GROUP_DIAGONALS)
		{
		sprintf(output_path,"%.255sflat_to_left_bank_diag%s",output_dir,suffix);
		write_track_section(context,&flat_to_left_bank_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sflat_to_right_bank_diag%s",output_dir,suffix);
		write_track_section(context,&flat_to_right_bank_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sleft_bank_to_gentle_up_diag%s",output_dir,suffix);
		write_track_section(context,&left_bank_to_gentle_up_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sright_bank_to_gentle_up_diag%s",output_dir,suffix);
		write_track_section(context,&right_bank_to_gentle_up_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sgentle_up_to_left_bank_diag%s",output_dir,suffix);
		write_track_section(context,&gentle_up_to_left_bank_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sgentle_up_to_right_bank_diag%s",output_dir,suffix);
		write_track_section(context,&gentle_up_to_right_bank_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
		sprintf(output_path,"%.255sleft_bank_diag%s",output_dir,suffix);
		write_track_section(context,semi_split?&semi_split_left_bank_diag:&left_bank_diag,track_type,base_dir,output_path,sprites,subtype,NULL);
		}

	sprintf(output_path,"%.255ssmall_turn_left_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_small_turn_left_bank:&small_turn_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_left_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_medium_turn_left_bank:&medium_turn_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_left_to_diag_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_large_turn_left_to_diag_bank:&large_turn_left_to_diag_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_turn_right_to_diag_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_large_turn_right_to_diag_bank:&large_turn_right_to_diag_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Sloped turns
	if(groups&TRACK_GROUP_SLOPED_TURNS)
	{
	sprintf(output_path,"%.255ssmall_turn_left_gentle_up%s",output_dir,suffix);
	write_track_section(context,&small_turn_left_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssmall_turn_right_gentle_up%s",output_dir,suffix);
	write_track_section(context,&small_turn_right_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_left_gentle_up%s",output_dir,suffix);
	write_track_section(context,&medium_turn_left_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_right_gentle_up%s",output_dir,suffix);
	write_track_section(context,&medium_turn_right_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if((groups&TRACK_GROUP_SLOPED_TURNS)&&(groups&TRACK_GROUP_STEEP_SLOPES))
	{
	sprintf(output_path,"%.255svery_small_turn_left_steep_up%s",output_dir,suffix);
	write_track_section(context,&very_small_turn_left_steep_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255svery_small_turn_right_steep_up%s",output_dir,suffix);
	write_track_section(context,&very_small_turn_right_steep_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if((groups&TRACK_GROUP_SLOPED_TURNS)&&(groups&TRACK_GROUP_VERTICAL_SLOPES))
	{
	sprintf(output_path,"%.255svertical_twist_left_up%s",output_dir,suffix);
	write_track_section(context,&vertical_twist_left_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255svertical_twist_right_up%s",output_dir,suffix);
	write_track_section(context,&vertical_twist_right_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Sloped banked turns

	if(groups&TRACK_GROUP_BANKED_SLOPED_TURNS)
	{
	sprintf(output_path,"%.255sgentle_up_to_gentle_up_left_bank%s",output_dir,suffix);
	write_track_section(context,&gentle_up_to_gentle_up_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_to_gentle_up_right_bank%s",output_dir,suffix);
	write_track_section(context,&gentle_up_to_gentle_up_right_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_left_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&gentle_up_left_bank_to_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_right_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&gentle_up_right_bank_to_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sleft_bank_to_gentle_up_left_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_left_bank_to_gentle_up_left_bank:&left_bank_to_gentle_up_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sright_bank_to_gentle_up_right_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_right_bank_to_gentle_up_right_bank:&right_bank_to_gentle_up_right_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_left_bank_to_left_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_gentle_up_left_bank_to_left_bank:&gentle_up_left_bank_to_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_right_bank_to_right_bank%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_gentle_up_right_bank_to_right_bank:&gentle_up_right_bank_to_right_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_left_bank%s",output_dir,suffix);
	write_track_section(context,&gentle_up_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_right_bank%s",output_dir,suffix);
	write_track_section(context,&gentle_up_right_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sflat_to_gentle_up_left_bank%s",output_dir,suffix);
	write_track_section(context,&flat_to_gentle_up_left_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sflat_to_gentle_up_right_bank%s",output_dir,suffix);
	write_track_section(context,&flat_to_gentle_up_right_bank,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_left_bank_to_flat%s",output_dir,suffix);
	write_track_section(context,&gentle_up_left_bank_to_flat,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sgentle_up_right_bank_to_flat%s",output_dir,suffix);
	write_track_section(context,&gentle_up_right_bank_to_flat,track_type,base_dir,output_path,sprites,subtype,NULL);
	
	sprintf(output_path,"%.255ssmall_turn_left_bank_gentle_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_small_turn_left_bank_gentle_up:&small_turn_left_bank_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssmall_turn_right_bank_gentle_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_small_turn_right_bank_gentle_up:&small_turn_right_bank_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_left_bank_gentle_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_medium_turn_left_bank_gentle_up:&medium_turn_left_bank_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_turn_right_bank_gentle_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_medium_turn_right_bank_gentle_up:&medium_turn_right_bank_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Miscellaneous
	if(groups&TRACK_GROUP_S_BENDS)
	{
	sprintf(output_path,"%.255ss_bend_left%s",output_dir,suffix);
	write_track_section(context,&s_bend_left,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ss_bend_right%s",output_dir,suffix);
	write_track_section(context,&s_bend_right,track_type,base_dir,output_path,sprites,subtype,NULL);
	}

	if(groups&TRACK_GROUP_HELICES)
	{
	sprintf(output_path,"%.255ssmall_helix_left_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_small_helix_left_up:&small_helix_left_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssmall_helix_right_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_small_helix_right_up:&small_helix_right_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_helix_left_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_medium_helix_left_up:&medium_helix_left_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255smedium_helix_right_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_medium_helix_right_up:&medium_helix_right_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	}

//Inversions
	if(groups&TRACK_GROUP_BARREL_ROLLS)
	{
	sprintf(output_path,"%.255sbarrel_roll_left%s",output_dir,suffix);
	write_track_section(context,&barrel_roll_left,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255sbarrel_roll_right%s",output_dir,suffix);
	write_track_section(context,&barrel_roll_right,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_HALF_LOOPS)
	{
	sprintf(output_path,"%.255shalf_loop%s",output_dir,suffix);
	write_track_section(context,&half_loop,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_LARGE_SLOPE_TRANSITIONS)
	{
	sprintf(output_path,"%.255sflat_to_steep_up%s",output_dir,suffix);
	write_track_section(context,&flat_to_steep_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssteep_to_flat_up%s",output_dir,suffix);
	write_track_section(context,&steep_to_flat_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_QUARTER_LOOPS)
	{
	sprintf(output_path,"%.255squarter_loop_up%s",output_dir,suffix);
	write_track_section(context,semi_split?&semi_split_quarter_loop_up:&quarter_loop_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_CORKSCREWS)
	{
	sprintf(output_path,"%.255scorkscrew_left%s",output_dir,suffix);
	write_track_section(context,&corkscrew_left,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255scorkscrew_right%s",output_dir,suffix);
	write_track_section(context,&corkscrew_right,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	if(groups&TRACK_GROUP_TURN_BANK_TRANSITIONS)
	{
	sprintf(output_path,"%.255ssmall_turn_left_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&small_turn_left_bank_to_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255ssmall_turn_right_bank_to_gentle_up%s",output_dir,suffix);
	write_track_section(context,&small_turn_right_bank_to_gentle_up,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	
	if(groups&TRACK_GROUP_LARGE_HALF_LOOPS)
	{
	sprintf(output_path,"%.255slarge_half_loop_left%s",output_dir,suffix);
	write_track_section(context,&large_half_loop_left,track_type,base_dir,output_path,sprites,subtype,NULL);
	sprintf(output_path,"%.255slarge_half_loop_right%s",output_dir,suffix);
	write_track_section(context,&large_half_loop_right,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
	
	if(groups&TRACK_GROUP_SMALL_SLOPE_TRANSITIONS)
	{
	sprintf(output_path,"%.255ssmall_flat_to_steep_up%s",output_dir,suffix);
	write_track_section(context,&small_flat_to_steep_up,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?small_flat_to_steep_up_chain:NULL);
	sprintf(output_path,"%.255ssmall_steep_to_flat_up%s",output_dir,suffix);
	write_track_section(context,&small_steep_to_flat_up,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?small_steep_to_flat_up_chain:NULL);
	sprintf(output_path,"%.255ssmall_flat_to_steep_up_diag%s",output_dir,suffix);
	write_track_section(context,&small_flat_to_steep_up_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?small_flat_to_steep_up_diag_chain:NULL);
	sprintf(output_path,"%.255ssmall_steep_to_flat_up_diag%s",output_dir,suffix);
	write_track_section(context,&small_steep_to_flat_up_diag,track_type,base_dir,output_path,sprites,subtype,subtype==TRACK_SUBTYPE_LIFT?small_steep_to_flat_up_diag_chain:NULL);
	}

//Launched lift
	if(groups&TRACK_GROUP_LAUNCHED_LIFTS)
	{
	sprintf(output_path,"%.255spowered_lift%s",output_dir,suffix);
	write_track_section(context,&launched_lift,track_type,base_dir,output_path,sprites,subtype,NULL);
	}
return 0;
}

int write_track_type(context_t* context,track_type_t* track_type,json_t* sprites,const char* base_dir,const char* output_dir)
{
write_track_subtype(context,track_type,sprites,base_dir,output_dir,TRACK_SUBTYPE_DEFAULT);
	if(track_type->flags&TRACK_HAS_LIFT)
	{
	write_track_subtype(context,track_type,sprites,base_dir,output_dir,TRACK_SUBTYPE_LIFT);
	}
return 0;
}
