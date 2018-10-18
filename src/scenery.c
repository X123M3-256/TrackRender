

void rct2_image_from_image(rct2_image_t* rct2_image,image_t* image)
{
rct2_image->width=image->width;
rct2_image->height=image->height;
rct2_image->x_offset=image->offset_x;
rct2_image->y_offset=image->offset_y;
rct2_image->data=image->pixels;
}

typedef struct
{
track_section_t* section;
uint32_t x,y;
uint32_t sprites[4];
}map_element_info_t;

void write_scenery(object_t* object,track_type_t* track_type,const char* filename)
{
uint32_t num_elements=4;
map_element_info_t elements[]={{&flat,0,0,{0,0,0,0}},{&flat_to_gentle_up,0,1,{0,0,0,0}},{&gentle_up_to_flat,0,2,{0,0,0,0}},{&gentle,0,3,{0,0,0,0}}};



large_scenery_t scenery;

scenery.cursor_sel=20;
scenery.flags=0;
scenery.build_fee=10;
scenery.remove_fee=0;
scenery.scroll=0xFF;
scenery.unknown=NULL;

scenery.tile_info.num_tiles=num_elements;
scenery.tile_info.tiles=malloc(num_elements*sizeof(large_scenery_tile_t));
	for(uint32_t i=0;i<num_elements;i++)
	{
	scenery.tile_info.tiles[i].x=elements[i].x*32;
	scenery.tile_info.tiles[i].y=elements[i].y*32;
	scenery.tile_info.tiles[i].flags=0;
	scenery.tile_info.tiles[i].quadrants=0xF;
	scenery.tile_info.tiles[i].walls=0xF;
	scenery.tile_info.tiles[i].base_height=0;
	scenery.tile_info.tiles[i].clearance_height=0;
	}

memset(scenery.name.strings,0,sizeof(char*)*NUM_LANGUAGES);
scenery.name.strings[LANGUAGE_ENGLISH_US]=malloc(strlen("Intamin double spine track")+1);
strcpy(scenery.name.strings[LANGUAGE_ENGLISH_US],"Intamin double spine track");

scenery.group_info.flags=0;
memcpy(scenery.group_info.name,"        ",8);

//TODO the sprites

scenery.sprites.num_images=4+num_elements*4;
scenery.sprites.images=malloc(scenery.sprites.num_images*sizeof(rct2_image_t));
image_t image;
render_track_sprite(&image,0,0,&flat,track_type);
rct2_image_from_image(scenery.sprites.images+0,&image);
render_track_sprite(&image,1,0,&flat,track_type);
rct2_image_from_image(scenery.sprites.images+1,&image);
render_track_sprite(&image,2,0,&flat,track_type);
rct2_image_from_image(scenery.sprites.images+2,&image);
render_track_sprite(&image,3,0,&flat,track_type);
rct2_image_from_image(scenery.sprites.images+3,&image);


	for(uint32_t i=0;i<num_elements;i++)
	for(uint32_t angle=0;angle<4;angle++)
	{
	render_track_sprite(&image,angle,elements[i].sprites[angle],elements[i].section,track_type);
	rct2_image_from_image(scenery.sprites.images+(i*4+angle+4),&image);
	printf("%d\n",(i*4+angle+4));
	}



object->flags=OBJECT_LARGE_SCENERY;
memcpy(object->name,filename,8);
object->checksum=0;

uint8_t* data;uint32_t length;
error_t error=large_scenery_encode(&scenery,ENCODING_NONE,&(object->chunk));

    if(error!=ERROR_NONE)
    {
    printf("Error: %s\n",error_string(error));
    }
large_scenery_destroy(&scenery);
}

//In main()

object_t object;
write_scenery(&object,&intamindouble,"INTDSPN ");

FILE* file=fopen("/home/edward/.config/OpenRCT2/object/INTDSPN.DAT","w");
    if(file==NULL)
    {
    printf("Could not open file for writing\n");
    return 1;
    }
error=object_write(&object,file);
    if(error!=ERROR_NONE)
    {
    printf("Could not write file\n");
    object_destroy(&object);
    return 1;
    }

object_destroy(&object);
