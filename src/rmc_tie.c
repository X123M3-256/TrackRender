#include "rmc_tie.h"

#define TILE_SIZE (3.67423461417)

material_t wood={MATERIAL_IS_REMAPPABLE,1,0.5,1,.color={0.8,0.8,0.8}};
material_t steel={0,0,0.5,1,.color={0.8,0.8,0.8}};


void primitive(primitive_t* primitive,vector3_t a,vector3_t b,vector3_t c,vector3_t normal,material_t* mat)
{
primitive->vertices[0]=a;
primitive->vertices[1]=b;
primitive->vertices[2]=c;
primitive->normals[0]=normal;
primitive->normals[1]=normal;
primitive->normals[2]=normal;
primitive->material=mat;
}

void wood_column(primitive_t* primitives,int side,float height)
{
float wood_beam_width=TILE_SIZE/16;
float wood_beam_base=-0.75;
float wood_beam_top=height;
float offset=side==0?0.5*(wood_beam_width-TILE_SIZE):0.5*(TILE_SIZE-wood_beam_width);

primitive(primitives+0,vector3(-wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(0,-1,0),&wood);
primitive(primitives+1,vector3(wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(0,-1,0),&wood);


primitive(primitives+2,vector3(-wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_top,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(0,1,0),&wood);
primitive(primitives+3,vector3(wood_beam_width/2,wood_beam_top,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_top,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(0,1,0),&wood);

primitive(primitives+4,vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(1,0,0),&wood);
primitive(primitives+5,vector3(wood_beam_width/2,wood_beam_top,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(1,0,0),&wood);

primitive(primitives+6,vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_top,offset+wood_beam_width/2),vector3(0,0,1),&wood);
primitive(primitives+7,vector3(wood_beam_width/2,wood_beam_top,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_top,offset+wood_beam_width/2),vector3(0,0,1),&wood);

primitive(primitives+8,vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(-1,0,0),&wood);
primitive(primitives+9,vector3(-wood_beam_width/2,wood_beam_top,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(-1,0,0),&wood);

primitive(primitives+10,vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(0,0,-1),&wood);
primitive(primitives+11,vector3(wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_top,offset-wood_beam_width/2),vector3(0,0,-1),&wood);

primitive(primitives+12,vector3(wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(1,0,0),&wood);
primitive(primitives+13,vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(1,0,0),&wood);

	if(side)
	{
	primitive(primitives+14,vector3(-wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(0,0,1),&wood);
	primitive(primitives+15,vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(0,0,1),&wood);
	}

primitive(primitives+16,vector3(-wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(-1,0,0),&wood);
primitive(primitives+17,vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base,offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(-1,0,0),&wood);

	if(!side)
	{
	primitive(primitives+18,vector3(-wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(0,0,-1),&wood);
	primitive(primitives+19,vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(0,0,-1),&wood);
	}

//Base

primitive(primitives+20,vector3(-wood_beam_width/2,wood_beam_base,-offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,-offset+wood_beam_width/2),vector3(0,-1,0),&wood);
primitive(primitives+21,vector3(wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,-offset+wood_beam_width/2),vector3(0,-1,0),&wood);


primitive(primitives+22,vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,-offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,-offset+wood_beam_width/2),vector3(0,1,0),&wood);
primitive(primitives+23,vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,-offset+wood_beam_width/2),vector3(0,1,0),&wood);

primitive(primitives+24,vector3(wood_beam_width/2,wood_beam_base,-offset+wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,-offset+wood_beam_width/2),vector3(1,0,0),&wood);
primitive(primitives+25,vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(wood_beam_width/2,wood_beam_base+wood_beam_width,-offset+wood_beam_width/2),vector3(1,0,0),&wood);

primitive(primitives+26,vector3(-wood_beam_width/2,wood_beam_base,-offset+wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,-offset+wood_beam_width/2),vector3(-1,0,0),&wood);
primitive(primitives+27,vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base,offset-wood_beam_width/2),vector3(-wood_beam_width/2,wood_beam_base+wood_beam_width,-offset+wood_beam_width/2),vector3(-1,0,0),&wood);
}

void steel_beam(primitive_t* primitive)
{

}

void rmc_tie_get_primitives(primitive_t* primitives)
{
wood_column(primitives+0,0,0);
wood_column(primitives+28,1,0);

}


