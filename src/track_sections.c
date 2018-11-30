#include <stdint.h>
#include <math.h>
#include "track.h"
#define NORM(x,y) (sqrt((x)*(x)+(y)*(y)))

#define HALF_LOOP_LENGTH (7.1888786411859895+9.877636849070576+1.9843134832984428)

float cubic(float a,float b, float c,float d,float x)
{
return x*(x*(x*a+b)+c)+d;
}

float cubic_derivative(float a,float b,float c,float x)
{
return (x*(3.0*x*a+2.0*b)+c);
}

track_point_t plane_curve_vertical(vector3_t position,vector3_t tangent)
{
track_point_t point;
point.position=position;
point.tangent=tangent;
point.normal=vector3(0.0,tangent.z,-tangent.y);
point.binormal=vector3(1.0,0.0,0.0);
return point;
}
track_point_t plane_curve_vertical_diagonal(vector3_t position,vector3_t tangent)
{
track_point_t point;
point.position=position;
point.tangent=tangent;
point.normal=vector3(tangent.y/sqrt(2),tangent.z*sqrt(2),-tangent.y/sqrt(2));
point.binormal=vector3(sqrt(0.5),0.0,sqrt(0.5)); 
return point;
}
track_point_t plane_curve_horizontal(vector3_t position,vector3_t tangent)
{
track_point_t point;
point.position=position;
point.tangent=tangent;
point.normal=vector3(0.0,1.0,0.0);
point.binormal=vector3(tangent.z,0.0,-tangent.x);
return point;
}

track_point_t cubic_curve_vertical(float xa,float xb,float xc,float xd,float ya,float yb,float yc,float yd,float distance)
{
return plane_curve_vertical(vector3(0.0,cubic(ya,yb,yc,yd,distance),cubic(xa,xb,xc,xd,distance)),vector3_normalize(vector3(0.0,cubic_derivative(ya,yb,yc,distance),cubic_derivative(xa,xb,xc,distance))));
}
track_point_t cubic_curve_vertical_diagonal(float xa,float xb,float xc,float xd,float ya,float yb,float yc,float yd,float distance)
{
float x=cubic(xa,xb,xc,xd,distance);
float y=cubic(ya,yb,yc,yd,distance);
float dx=cubic_derivative(xa,xb,xc,distance);
float dy=cubic_derivative(ya,yb,yc,distance);
return plane_curve_vertical_diagonal(vector3(-x/sqrt(2),y,x/sqrt(2)),vector3_normalize(vector3(-dx/sqrt(2),dy,dx/sqrt(2))));
}
track_point_t cubic_curve_horizontal(float xa,float xb,float xc,float xd,float ya,float yb,float yc,float yd,float distance)
{
return plane_curve_horizontal(vector3(cubic(ya,yb,yc,yd,distance),0.0,cubic(xa,xb,xc,xd,distance)),vector3_normalize(vector3(cubic_derivative(ya,yb,yc,distance),0.0,cubic_derivative(xa,xb,xc,distance))));
}

track_point_t banked_curve(track_point_t unbanked_curve,float angle)
{
track_point_t point;
point.position=unbanked_curve.position;
point.tangent=unbanked_curve.tangent;
point.normal=vector3_add(vector3_mult(unbanked_curve.normal,cos(angle)),vector3_mult(unbanked_curve.binormal,sin(angle)));
point.binormal=vector3_add(vector3_mult(unbanked_curve.normal,-sin(angle)),vector3_mult(unbanked_curve.binormal,cos(angle)));
return point;
}

track_point_t flat_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,0.0,distance),vector3(0.0,0.0,1.0));
}


track_point_t flat_to_gentle_up_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,(1.0/18.0)*distance*distance,distance),vector3_normalize(vector3(0.0,(1.0/9.0)*distance,1.0)));
}
track_point_t gentle_to_flat_up_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,(1.0/18.0)*distance*distance,distance),vector3_normalize(vector3(0.0,(1.0/9.0)*distance,1.0)));
}
track_point_t flat_to_gentle_down_curve(float distance) //Unconfirmed
{
return plane_curve_vertical(vector3(0.0,-(1.0/18.0)*distance*distance,distance),vector3_normalize(vector3(0.0,-(1.0/9.0)*distance,1.0)));
}
track_point_t gentle_up_to_flat_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,-(1.0/18.0)*(TILE_SIZE-distance)*(TILE_SIZE-distance)+0.75,distance),vector3_normalize(vector3(0.0,(1.0/9.0)*(TILE_SIZE-distance),1.0)));
}
track_point_t gentle_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,1.5*distance/TILE_SIZE,(distance)),vector3_normalize(vector3(0.0,1.5/TILE_SIZE,1.0)));
}
track_point_t gentle_to_steep_up_curve(float distance)
{
return cubic_curve_vertical(0,0,1.5*sqrt(6),0.0,0,1.5,1.5,0,distance/3.0);
}
track_point_t steep_to_gentle_up_curve(float distance)
{
return cubic_curve_vertical(0,0,1.5*sqrt(6),0,0,-1.5,4.5,0,distance/3.0);
}
track_point_t steep_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,distance,distance*TILE_SIZE/6.0),vector3_normalize(vector3(0.0,6.0/TILE_SIZE,1.0)));
}
track_point_t vertical_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,distance,0.0),vector3(0.0,1.0,0.0));
}
track_point_t steep_to_vertical_up_curve(float distance)
{
return cubic_curve_vertical(-0.612,-0.612,3.062,-1.837,-0.5,0.75,5.0,0,distance/5.25);
}
track_point_t steep_to_vertical_down_curve(float distance)
{
return cubic_curve_vertical(-0.612,2.449,0.0,0.0,-0.5,0.75,5.0,0,distance/5.25);
}
track_point_t small_turn_left_curve(float distance)
{
float angle=distance/(1.5*TILE_SIZE);
track_point_t point;
point.position=vector3(1.5*TILE_SIZE*(1.0-cos(angle)),0,1.5*TILE_SIZE*sin(angle));
point.tangent=vector3(sin(angle),0.0,cos(angle));
point.normal=vector3(0.0,1.0,0.0);
point.binormal=vector3_cross(point.normal,point.tangent);
return point;
}
track_point_t small_turn_right_curve(float distance)
{
float angle=distance/(1.5*TILE_SIZE);
track_point_t point;
point.position=vector3(1.5*TILE_SIZE*(cos(angle)-1.0),0,1.5*TILE_SIZE*sin(angle));
point.tangent=vector3(-sin(angle),0.0,cos(angle));
point.normal=vector3(0.0,1.0,0.0);
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}
track_point_t medium_turn_left_curve(float distance)
{
float angle=distance/(2.5*TILE_SIZE);
track_point_t point;
point.position=vector3(2.5*TILE_SIZE*(1.0-cos(angle)),0,2.5*TILE_SIZE*sin(angle));
point.tangent=vector3(sin(angle),0.0,cos(angle));
point.normal=vector3(0.0,1.0,0.0);
point.binormal=vector3_cross(point.normal,point.tangent);
return point;
}
track_point_t large_turn_left_to_diag_curve(float distance)
{
return cubic_curve_horizontal(-1.3711730708738337,-0.4432403936892513,11,0.0,-1.3484692283495336,5.022703842524301,0,0.0,distance/(0.875*TILE_SIZE*3.1415926));
}
track_point_t large_turn_right_to_diag_curve(float distance)
{
return cubic_curve_horizontal(-1.3711730708738337,-0.4432403936892513,11,0.0,1.3484692283495336,-5.022703842524301,0.0,0.0,distance/(0.875*TILE_SIZE*3.1415926));
}
track_point_t flat_diag_curve(float distance)
{
return plane_curve_horizontal(vector3(-distance/sqrt(2),0.0,distance/sqrt(2)),vector3(-sqrt(0.5),0.0,sqrt(0.5)));
}
track_point_t flat_to_gentle_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(0,0,sqrt(2),0,0,1.0/18.0,0,0,distance/sqrt(2));
}
track_point_t flat_to_gentle_down_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(0,0,sqrt(2),0,0,-1.0/18.0,0,0.75,distance/sqrt(2));
}
track_point_t gentle_to_flat_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(0,0,sqrt(2),0,0,-1.0/18.0,1.5/TILE_SIZE,0.0,distance/sqrt(2));//TODO confirm this is correct geometry
}
track_point_t gentle_diag_curve(float distance)
{
return plane_curve_vertical_diagonal(vector3(-distance/sqrt(2),1.5*distance/(sqrt(2)*TILE_SIZE),distance/sqrt(2)),vector3_normalize(vector3(-1.0/sqrt(2),1.5/(sqrt(2)*TILE_SIZE),1.0/sqrt(2))));
}
track_point_t gentle_to_steep_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(0,0,sqrt(2)*TILE_SIZE,0.0,0,1.5,1.5,0,distance/3.0);
}
track_point_t steep_to_gentle_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(0,0,sqrt(2)*TILE_SIZE,0,0,-1.5,4.5,0,distance/3.0);
}
track_point_t steep_diag_curve(float distance)
{
return plane_curve_vertical_diagonal(vector3(-distance*TILE_SIZE/6.0,distance,distance*TILE_SIZE/6.0),vector3_normalize(vector3(-1.0,6.0/TILE_SIZE,1.0)));
}
track_point_t flat_to_left_bank_curve(float distance)
{
return banked_curve(flat_curve(distance),0.25*M_PI*distance/TILE_SIZE);
}
track_point_t flat_to_right_bank_curve(float distance)
{
return banked_curve(flat_curve(distance),-0.25*M_PI*distance/TILE_SIZE);
}
track_point_t left_bank_to_gentle_up_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),0.25*M_PI*(1.0-distance/TILE_SIZE));
}
track_point_t right_bank_to_gentle_up_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),-0.25*M_PI*(1.0-distance/TILE_SIZE));
}
track_point_t gentle_up_to_left_bank_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),0.25*M_PI*distance/TILE_SIZE);
}
track_point_t gentle_up_to_right_bank_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),-0.25*M_PI*distance/TILE_SIZE);
}
track_point_t left_bank_curve(float distance)
{
return banked_curve(flat_curve(distance),0.25*M_PI);
}
track_point_t flat_to_left_bank_diag_curve(float distance)
{
return banked_curve(flat_diag_curve(distance),0.25*M_PI*distance/(sqrt(2)*TILE_SIZE));
}
track_point_t flat_to_right_bank_diag_curve(float distance)
{
return banked_curve(flat_diag_curve(distance),-0.25*M_PI*distance/(sqrt(2)*TILE_SIZE));
}
track_point_t left_bank_to_gentle_up_diag_curve(float distance)
{
return banked_curve(flat_to_gentle_up_diag_curve(distance),0.25*M_PI*(1.0-distance/(sqrt(2)*TILE_SIZE)));
}
track_point_t right_bank_to_gentle_up_diag_curve(float distance)
{
return banked_curve(flat_to_gentle_up_diag_curve(distance),-0.25*M_PI*(1.0-distance/(sqrt(2)*TILE_SIZE)));
}
track_point_t gentle_up_to_left_bank_diag_curve(float distance)
{
return banked_curve(gentle_to_flat_up_diag_curve(distance),0.25*M_PI*distance/(sqrt(2)*TILE_SIZE));
}
track_point_t gentle_up_to_right_bank_diag_curve(float distance)
{
return banked_curve(gentle_to_flat_up_diag_curve(distance),-0.25*M_PI*distance/(sqrt(2)*TILE_SIZE));
}
track_point_t left_bank_diag_curve(float distance)
{
return banked_curve(flat_diag_curve(distance),0.25*M_PI);
}
track_point_t small_turn_left_bank_curve(float distance)
{
return banked_curve(small_turn_left_curve(distance),0.25*M_PI);
}
track_point_t medium_turn_left_bank_curve(float distance)
{
return banked_curve(medium_turn_left_curve(distance),0.25*M_PI);
}
track_point_t large_turn_left_to_diag_bank_curve(float distance)
{
return banked_curve(large_turn_left_to_diag_curve(distance),0.25*M_PI);
}
track_point_t large_turn_right_to_diag_bank_curve(float distance)
{
return banked_curve(large_turn_right_to_diag_curve(distance),-0.25*M_PI);
}


track_point_t sloped_turn_left_curve(float radius,float gradient,float distance)
{
float arclength=radius*sqrt(1.0+gradient*gradient);
float angle=distance/arclength;
track_point_t point;
point.position=vector3(radius*(1.0-cos(angle)),angle*radius*gradient,radius*sin(angle));
float tangent_z=1.0/sqrt(1+gradient*gradient);
float tangent_y=gradient/sqrt(1+gradient*gradient);
point.tangent=vector3_normalize(vector3(tangent_z*sin(angle),tangent_y,tangent_z*cos(angle)));
point.normal=vector3_normalize(vector3(-tangent_y*sin(angle),tangent_z,-tangent_y*cos(angle)));
point.binormal=vector3_cross(point.normal,point.tangent);
return point;
}
track_point_t sloped_turn_right_curve(float radius,float gradient,float distance)
{
float arclength=radius*sqrt(1.0+gradient*gradient);
float angle=distance/arclength;
track_point_t point;
point.position=vector3(radius*(cos(angle)-1.0),angle*radius*gradient,radius*sin(angle));
float tangent_z=1.0/sqrt(1+gradient*gradient);
float tangent_y=gradient/sqrt(1+gradient*gradient);
point.tangent=vector3_normalize(vector3(-tangent_z*sin(angle),tangent_y,tangent_z*cos(angle)));
point.normal=vector3_normalize(vector3(tangent_y*sin(angle),tangent_z,-tangent_y*cos(angle)));
point.binormal=vector3_cross(point.normal,point.tangent);
return point;
}
track_point_t small_turn_left_gentle_up_curve(float distance)
{
return sloped_turn_left_curve(1.5*TILE_SIZE,3.0/(0.75*M_PI*TILE_SIZE),distance);
}
track_point_t small_turn_right_gentle_up_curve(float distance)
{
return sloped_turn_right_curve(1.5*TILE_SIZE,3.0/(0.75*M_PI*TILE_SIZE),distance);
}
track_point_t medium_turn_left_gentle_up_curve(float distance)
{
return sloped_turn_left_curve(2.5*TILE_SIZE,6.0/(1.25*M_PI*TILE_SIZE),distance);
}
track_point_t medium_turn_right_gentle_up_curve(float distance)
{
return sloped_turn_right_curve(2.5*TILE_SIZE,6.0/(1.25*M_PI*TILE_SIZE),distance);
}
track_point_t very_small_turn_left_steep_up_curve(float distance)
{
return sloped_turn_left_curve(0.5*TILE_SIZE,6.0/(0.25*M_PI*TILE_SIZE),distance);
}
track_point_t very_small_turn_right_steep_up_curve(float distance)
{
return sloped_turn_right_curve(0.5*TILE_SIZE,6.0/(0.25*M_PI*TILE_SIZE),distance);
}


track_point_t gentle_up_to_gentle_up_left_bank_curve(float distance)
{
return banked_curve(gentle_curve(distance),0.25*M_PI*distance/TILE_SIZE);
}
track_point_t gentle_up_to_gentle_up_right_bank_curve(float distance)
{
return banked_curve(gentle_curve(distance),-0.25*M_PI*distance/TILE_SIZE);
}
track_point_t gentle_up_left_bank_to_gentle_up_curve(float distance)
{
return banked_curve(gentle_curve(distance),0.25*M_PI*(1.0-distance/TILE_SIZE));
}
track_point_t gentle_up_right_bank_to_gentle_up_curve(float distance)
{
return banked_curve(gentle_curve(distance),-0.25*M_PI*(1.0-distance/TILE_SIZE));
}
track_point_t left_bank_to_gentle_up_left_bank_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),0.25*M_PI);
}
track_point_t right_bank_to_gentle_up_right_bank_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),-0.25*M_PI);
}
track_point_t gentle_up_left_bank_to_left_bank_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),0.25*M_PI);
}
track_point_t gentle_up_right_bank_to_right_bank_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),-0.25*M_PI);
}
track_point_t gentle_up_left_bank_curve(float distance)
{
return banked_curve(gentle_curve(distance),0.25*M_PI);
}
track_point_t gentle_up_right_bank_curve(float distance)
{
return banked_curve(gentle_curve(distance),-0.25*M_PI);
}
track_point_t flat_to_gentle_up_left_bank_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),0.25*M_PI*distance/TILE_SIZE);
}
track_point_t flat_to_gentle_up_right_bank_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),-0.25*M_PI*distance/TILE_SIZE);
}
track_point_t gentle_up_left_bank_to_flat_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),0.25*M_PI*(1.0-distance/TILE_SIZE));
}
track_point_t gentle_up_right_bank_to_flat_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),-0.25*M_PI*(1.0-distance/TILE_SIZE));
}
track_point_t small_turn_left_bank_gentle_up_curve(float distance)
{
return banked_curve(small_turn_left_gentle_up_curve(distance),0.25*M_PI);
}
track_point_t small_turn_right_bank_gentle_up_curve(float distance)
{
return banked_curve(small_turn_right_gentle_up_curve(distance),-0.25*M_PI);
}
track_point_t medium_turn_left_bank_gentle_up_curve(float distance)
{
return banked_curve(medium_turn_left_gentle_up_curve(distance),0.25*M_PI);
}
track_point_t medium_turn_right_bank_gentle_up_curve(float distance)
{
return banked_curve(medium_turn_right_gentle_up_curve(distance),-0.25*M_PI);
}
track_point_t vertical_twist_left_up_curve(float distance)
{
track_point_t point;
point.position=vector3(0.0,distance,0.0);
point.tangent=vector3(0.0,1.0,0.0);
point.normal=vector3(-sin(distance*3.141562654/18.0),0.0,-cos(distance*3.141562654/18.0));
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}
track_point_t vertical_twist_right_up_curve(float distance)
{
track_point_t point;
point.position=vector3(0.0,distance,0.0);
point.tangent=vector3(0.0,1.0,0.0);
point.normal=vector3(sin(distance*3.141562654/18.0),0.0,-cos(distance*3.141562654/18.0));
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}


track_point_t s_bend_left_curve(float distance)
{
return cubic_curve_horizontal(15.954592314951398,-23.931888472427097,19,0,-7.348469228349534,11.022703842524301,0,0,distance/NORM(3.0*TILE_SIZE,1.0*TILE_SIZE));
}
track_point_t s_bend_right_curve(float distance)
{
return cubic_curve_horizontal(15.954592314951398,-23.931888472427097,19,0,7.348469228349534,-11.022703842524301,0,0,distance/NORM(3.0*TILE_SIZE,1.0*TILE_SIZE));
}
track_point_t small_helix_left_up_curve(float distance)
{
return banked_curve(sloped_turn_left_curve(1.5*TILE_SIZE,0.75/(0.75*M_PI*TILE_SIZE),distance),0.25*M_PI);
}
track_point_t small_helix_right_up_curve(float distance)
{
return banked_curve(sloped_turn_right_curve(1.5*TILE_SIZE,0.75/(0.75*M_PI*TILE_SIZE),distance),-0.25*M_PI);
}
track_point_t medium_helix_left_up_curve(float distance)
{
return banked_curve(sloped_turn_left_curve(2.5*TILE_SIZE,0.75/(1.25*M_PI*TILE_SIZE),distance),0.25*M_PI);
}
track_point_t medium_helix_right_up_curve(float distance)
{
return banked_curve(sloped_turn_right_curve(2.5*TILE_SIZE,0.75/(1.25*M_PI*TILE_SIZE),distance),-0.25*M_PI);
}

track_point_t barrel_roll_left_curve(float x)
{
track_point_t point;
float length=4.5*sqrt(6)/3.1415926589;
point.position=vector3(-0.875*sin(x/length),0.875*(1-cos(x/length)),x);
point.tangent=vector3_normalize(vector3(-0.875*cos(x/length)/length,0.875*sin(x/length)/length,1.0));
point.normal=vector3(sin(x/length),cos(x/length),0.0);
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}
track_point_t barrel_roll_right_curve(float x)
{
track_point_t left_point=barrel_roll_left_curve(x);
track_point_t point;
point.position=vector3(-left_point.position.x,left_point.position.y,left_point.position.z);
point.tangent=vector3(-left_point.tangent.x,left_point.tangent.y,left_point.tangent.z);
point.normal=vector3(-left_point.normal.x,left_point.normal.y,left_point.normal.z);
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}
track_point_t half_loop_curve(float distance)
{
float bezier_start=1.9843134832984428;

#define HALF_LOOP_LENGTH (7.1888786411859895+9.877636849070576+1.9843134832984428)


	if(distance<bezier_start)return plane_curve_vertical(vector3(0.0,0.75*(distance/bezier_start),0.5*TILE_SIZE*(distance/bezier_start)),vector3_normalize(vector3(0.0,1.5/TILE_SIZE,1.0)));

float bezier_distance=distance-bezier_start;
	if(bezier_distance<9.877636849070576)return cubic_curve_vertical(6.696938456699067,-19.230994220485517,18.371173070873834,1.8371173070873834,-7,6.75,7.5,0.75,cubic(1.6,-1.2,0.6,0,bezier_distance/9.877636849070576));
	else return cubic_curve_vertical(0,-4,0,4.0+1.5*sqrt(6),-2.0,-1.0,8.0,8.0,cubic(-0.45,0.77,0.68,0,(bezier_distance-9.877636849070576)/7.1888786411859895));
}

//Slopes
track_section_t flat={0,flat_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t flat_to_gentle_up={0,flat_to_gentle_up_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t flat_to_gentle_down={0,flat_to_gentle_down_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t gentle_up_to_flat={0,gentle_up_to_flat_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t gentle={0,gentle_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
rect_t all={INT32_MIN,INT32_MIN,INT32_MAX,INT32_MAX};
mask_t gentle_to_steep_up_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all},{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t gentle_to_steep_up={0,gentle_to_steep_up_curve,3.0,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_to_steep_up_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_to_steep_up_masks+2},{0,1,NULL}}};
rect_t steep_to_gentle_up_rects[]={{9,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,9,INT32_MAX},{INT32_MIN,INT32_MIN,-12,INT32_MAX},{-12,INT32_MIN,INT32_MAX,INT32_MAX}};
mask_t steep_to_gentle_up_masks[]={{TRACK_MASK_UNION,1,0,0,steep_to_gentle_up_rects},{TRACK_MASK_DIFFERENCE,1,0,0,steep_to_gentle_up_rects+1},{TRACK_MASK_UNION,1,0,0,steep_to_gentle_up_rects+2},{TRACK_MASK_DIFFERENCE,1,0,0,steep_to_gentle_up_rects+3}};
track_section_t steep_to_gentle_up={0,steep_to_gentle_up_curve,3.0,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,steep_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,2,steep_to_gentle_up_masks+2},{0,1,NULL}}};
track_section_t steep={0,steep_curve,6.0,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t steep_to_vertical_up={TRACK_VERTICAL,steep_to_vertical_up_curve,5.25,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t steep_to_vertical_down={TRACK_VERTICAL,steep_to_vertical_down_curve,5.25,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t vertical={TRACK_VERTICAL,vertical_curve,3.0,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};

//Turns
rect_t small_turn_rects[165]={
	{INT32_MIN,-1,INT32_MAX,INT32_MAX},{INT32_MIN,-2,-2,INT32_MAX},{INT32_MIN,-3,-4,INT32_MAX},{INT32_MIN,-4,-6,INT32_MAX},{INT32_MIN,-5,-8,INT32_MAX},{INT32_MIN,-6,-10,INT32_MAX},{INT32_MIN,-7,-12,INT32_MAX},{INT32_MIN,-8,-14,INT32_MAX},{INT32_MIN,-9,-16,INT32_MAX},{INT32_MIN,-10,-18,INT32_MAX},{INT32_MIN,-11,-20,INT32_MAX},{INT32_MIN,-12,-22,INT32_MAX},{INT32_MIN,-13,-24,INT32_MAX},{0,-12,INT32_MAX,1},{2,-13,INT32_MAX,1},{4,-14,INT32_MAX,1},{6,-15,INT32_MAX,1},{8,-16,INT32_MAX,1},{10,-17,INT32_MAX,1},{12,-18,INT32_MAX,1},{14,-19,INT32_MAX,1},{16,-20,INT32_MAX,1},{18,-21,INT32_MAX,1},{20,-22,INT32_MAX,1},{22,-23,INT32_MAX,1},{24,-24,INT32_MAX,1},{INT32_MIN,INT32_MIN,-24,-13},{-24,INT32_MIN,-22,-12},{-22,INT32_MIN,-20,-11},{-20,INT32_MIN,-18,-10},{-18,INT32_MIN,-16,-9},{-16,INT32_MIN,-14,-8},{-14,INT32_MIN,-12,-7},{-12,INT32_MIN,-10,-6},{-10,INT32_MIN,-8,-5},{-8,INT32_MIN,-6,-4},{-6,INT32_MIN,-4,-3},{-4,INT32_MIN,-2,-2},{-2,INT32_MIN,0,-1},{0,INT32_MIN,2,-12},{2,INT32_MIN,4,-13},{4,INT32_MIN,6,-14},{6,INT32_MIN,8,-15},{8,INT32_MIN,10,-16},{10,INT32_MIN,12,-17},{12,INT32_MIN,14,-18},{14,INT32_MIN,16,-19},{16,INT32_MIN,18,-20},{18,INT32_MIN,20,-21},{20,INT32_MIN,22,-22},{22,INT32_MIN,24,-23},{24,INT32_MIN,INT32_MAX,-24},{INT32_MIN,INT32_MIN,0,INT32_MAX},{0,INT32_MIN,2,31},{2,INT32_MIN,4,30},{4,INT32_MIN,6,29},{6,INT32_MIN,8,28},{8,INT32_MIN,10,27},{10,INT32_MIN,12,26},{12,INT32_MIN,14,25},{14,INT32_MIN,16,24},{16,INT32_MIN,18,23},{18,INT32_MIN,20,22},{20,INT32_MIN,22,21},{22,INT32_MIN,24,20},{24,INT32_MIN,26,19},{26,INT32_MIN,28,18},{28,INT32_MIN,30,17},{30,INT32_MIN,32,16},{30,16,34,INT32_MAX},{28,17,36,INT32_MAX},{26,18,38,INT32_MAX},{24,19,40,INT32_MAX},{22,20,42,INT32_MAX},{20,21,44,INT32_MAX},{18,22,46,INT32_MAX},{16,23,48,INT32_MAX},{14,24,50,INT32_MAX},{12,25,52,INT32_MAX},{10,26,54,INT32_MAX},{8,27,56,INT32_MAX},{6,28,58,INT32_MAX},{4,29,60,INT32_MAX},{2,30,62,INT32_MAX},{0,31,64,INT32_MAX},{32,INT32_MIN,34,16},{34,INT32_MIN,36,17},{36,INT32_MIN,38,18},{38,INT32_MIN,40,19},{40,INT32_MIN,42,20},{42,INT32_MIN,44,21},{44,INT32_MIN,46,22},{46,INT32_MIN,48,23},{48,INT32_MIN,50,24},{50,INT32_MIN,52,25},{52,INT32_MIN,54,26},{54,INT32_MIN,56,27},{56,INT32_MIN,58,28},{58,INT32_MIN,60,29},{60,INT32_MIN,62,30},{62,INT32_MIN,64,31},{64,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,7},{-24,INT32_MIN,-22,8},{-22,INT32_MIN,-20,9},{-20,INT32_MIN,-18,10},{-18,INT32_MIN,-16,11},{-16,INT32_MIN,-14,12},{-14,INT32_MIN,-12,13},{-12,INT32_MIN,-10,14},{-10,INT32_MIN,-8,15},{-8,INT32_MIN,-6,16},{-6,INT32_MIN,-4,17},{-4,INT32_MIN,-2,18},{-2,INT32_MIN,0,19},{0,INT32_MIN,2,31},{2,INT32_MIN,4,30},{4,INT32_MIN,6,29},{6,INT32_MIN,8,28},{8,INT32_MIN,10,27},{10,INT32_MIN,12,26},{12,INT32_MIN,14,25},{14,INT32_MIN,16,24},{16,INT32_MIN,18,23},{18,INT32_MIN,20,22},{20,INT32_MIN,22,21},{22,INT32_MIN,24,20},{24,INT32_MIN,INT32_MAX,19},{INT32_MIN,19,0,31},{INT32_MIN,18,-2,31},{INT32_MIN,17,-4,31},{INT32_MIN,16,-6,31},{INT32_MIN,15,-8,31},{INT32_MIN,14,-10,31},{INT32_MIN,13,-12,31},{INT32_MIN,12,-14,31},{INT32_MIN,11,-16,31},{INT32_MIN,10,-18,31},{INT32_MIN,9,-20,31},{INT32_MIN,8,-22,31},{INT32_MIN,7,-24,31},{INT32_MIN,31,INT32_MAX,INT32_MAX},{2,30,INT32_MAX,INT32_MAX},{4,29,INT32_MAX,INT32_MAX},{6,28,INT32_MAX,INT32_MAX},{8,27,INT32_MAX,INT32_MAX},{10,26,INT32_MAX,INT32_MAX},{12,25,INT32_MAX,INT32_MAX},{14,24,INT32_MAX,INT32_MAX},{16,23,INT32_MAX,INT32_MAX},{18,22,INT32_MAX,INT32_MAX},{20,21,INT32_MAX,INT32_MAX},{22,20,INT32_MAX,INT32_MAX},{24,19,INT32_MAX,INT32_MAX},{-26,INT32_MIN,INT32_MAX,INT32_MAX},{-28,13,-26,INT32_MAX},{-30,14,-28,INT32_MAX},{-32,15,-30,INT32_MAX},{-38,INT32_MIN,-26,13},{-39,INT32_MIN,-25,14},{-40,INT32_MIN,-24,15},{INT32_MIN,INT32_MIN,26-64,INT32_MAX},{26-64,13,28-64,INT32_MAX},{28-64,14,30-64,INT32_MAX},{30-64,15,32-64,INT32_MAX}
};
mask_t small_turn_masks[12]={
	{0,13,0,0,small_turn_rects},{0,13,-32,16,small_turn_rects+13},{0,26,0,32,small_turn_rects+26},
	{0,17,0,0,small_turn_rects+52},{0,16,-32,-16,small_turn_rects+69},{0,17,-64,0,small_turn_rects+85},
	{0,26,0,0,small_turn_rects+102},{0,13,32,-16,small_turn_rects+128},{0,13,0,-32,small_turn_rects+141},
	{0,4,0,0,small_turn_rects+154},{0,3,32,16,small_turn_rects+158},{0,4,64,0,small_turn_rects+161}
};
track_section_t small_turn_left={0,small_turn_left_curve,0.75*TILE_SIZE*3.1415926,{{0,3,small_turn_masks},{0,3,small_turn_masks+3},{0,3,small_turn_masks+6},{0,3,small_turn_masks+9}}};
rect_t medium_turn_rects[]={
//First angle
{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{2,0,INT32_MAX,1},{4,1,INT32_MAX,2},{6,2,INT32_MAX,3},{8,3,INT32_MAX,4},{10,4,INT32_MAX,5},{12,5,INT32_MAX,6},{14,6,INT32_MAX,7},{16,7,INT32_MAX,8},{18,8,INT32_MAX,9},{20,9,INT32_MAX,10},{22,10,INT32_MAX,11},{24,11,INT32_MAX,12},{26,12,INT32_MAX,13},{28,13,INT32_MAX,14},{30,14,INT32_MAX,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{32,-49,INT32_MAX,-48},{30,-48,INT32_MAX,-47},{28,-47,INT32_MAX,-46},{26,-46,INT32_MAX,-45},{24,-45,INT32_MAX,-44},{22,-44,INT32_MAX,-43},{20,-43,INT32_MAX,-42},{18,-42,INT32_MAX,-41},{16,-41,INT32_MAX,-40},{14,-40,INT32_MAX,-39},{12,-39,INT32_MAX,-38},{10,-38,INT32_MAX,-37},{8,-37,INT32_MAX,-36},{6,-36,INT32_MAX,-35},{4,-35,INT32_MAX,-34},{2,-34,INT32_MAX,-33},{0,-33,INT32_MAX,-32},{2,-32,INT32_MAX,-31},{4,-31,INT32_MAX,-30},{6,-30,INT32_MAX,-29},{8,-29,INT32_MAX,-28},{10,-28,INT32_MAX,-27},{12,-27,INT32_MAX,-26},{14,-26,INT32_MAX,-25},{16,-25,INT32_MAX,-24},{18,-24,INT32_MAX,-23},{20,-23,INT32_MAX,-22},{22,-22,INT32_MAX,-21},{24,-21,INT32_MAX,-20},{26,-20,INT32_MAX,-19},{28,-19,INT32_MAX,-18},{30,-18,INT32_MAX,-17},
{INT32_MIN,-18,-30,-17},{INT32_MIN,-19,-28,-18},{INT32_MIN,-20,-26,-19},{INT32_MIN,-21,-24,-20},{INT32_MIN,-22,-22,-21},{INT32_MIN,-23,-20,-22},{INT32_MIN,-24,-18,-23},{INT32_MIN,-25,-16,-24},{INT32_MIN,-26,-14,-25},{INT32_MIN,-27,-12,-26},{INT32_MIN,-28,-10,-27},{INT32_MIN,-29,-8,-28},{INT32_MIN,-30,-6,-29},{INT32_MIN,-31,-4,-30},{INT32_MIN,-32,-2,-31},{INT32_MIN,-33,0,-32},{INT32_MIN,-34,2,-33},{INT32_MIN,-35,4,-34},{INT32_MIN,-36,6,-35},{INT32_MIN,-37,8,-36},{INT32_MIN,-38,10,-37},{INT32_MIN,-39,12,-38},{INT32_MIN,-40,14,-39},{INT32_MIN,-41,16,-40},{INT32_MIN,-42,18,-41},{INT32_MIN,-43,20,-42},{INT32_MIN,-44,22,-43},{INT32_MIN,-45,24,-44},{INT32_MIN,-46,26,-45},{INT32_MIN,-47,28,-46},{INT32_MIN,-48,30,-47},{INT32_MIN,INT32_MIN,INT32_MAX,-48},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,INT32_MAX},{30,16,34,17},{28,17,36,18},{26,18,38,19},{24,19,40,20},{22,20,42,21},{20,21,44,22},{18,22,46,23},{16,23,48,24},{14,24,50,25},{12,25,52,26},{10,26,54,27},{8,27,56,28},{6,28,58,29},{4,29,60,30},{2,30,62,31},{0,31,64,32},{2,32,62,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},{32,INT32_MIN,96,16},{34,16,94,17},{36,17,92,18},{38,18,90,19},{40,19,88,20},{42,20,86,21},{44,21,84,22},{46,22,82,23},{48,23,80,24},{50,24,78,25},{52,25,76,26},{54,26,74,27},{56,27,72,28},{58,28,70,29},{60,29,68,30},{62,30,66,31},{62,32,66,33},{60,33,68,34},{58,34,70,35},{56,35,72,36},{54,36,74,37},{52,37,76,38},{50,38,78,39},{48,39,80,40},{46,40,82,41},{44,41,84,42},{42,42,86,43},{40,43,88,44},{38,44,90,45},{36,45,92,46},{34,46,94,47},{32,47,96,INT32_MAX},{94,16,98,17},{92,17,100,18},{90,18,102,19},{88,19,104,20},{86,20,106,21},{84,21,108,22},{82,22,110,23},{80,23,112,24},{78,24,114,25},{76,25,116,26},{74,26,118,27},{72,27,120,28},{70,28,122,29},{68,29,124,30},{66,30,126,31},{64,31,128,32},{66,32,126,33},{68,33,124,34},{70,34,122,35},{72,35,120,36},{74,36,118,37},{76,37,116,38},{78,38,114,39},{80,39,112,40},{82,40,110,41},{84,41,108,42},{86,42,106,43},{88,43,104,44},{90,44,102,45},{92,45,100,46},{94,46,98,47},{96,INT32_MIN,INT32_MAX,16},{98,16,INT32_MAX,17},{100,17,INT32_MAX,18},{102,18,INT32_MAX,19},{104,19,INT32_MAX,20},{106,20,INT32_MAX,21},{108,21,INT32_MAX,22},{110,22,INT32_MAX,23},{112,23,INT32_MAX,24},{114,24,INT32_MAX,25},{116,25,INT32_MAX,26},{118,26,INT32_MAX,27},{120,27,INT32_MAX,28},{122,28,INT32_MAX,29},{124,29,INT32_MAX,30},{126,30,INT32_MAX,31},{128,31,INT32_MAX,32},{128,31,INT32_MAX,32},{126,32,INT32_MAX,33},{124,33,INT32_MAX,34},{122,34,INT32_MAX,35},{120,35,INT32_MAX,36},{118,36,INT32_MAX,37},{116,37,INT32_MAX,38},{114,38,INT32_MAX,39},{112,39,INT32_MAX,40},{110,40,INT32_MAX,41},{108,41,INT32_MAX,42},{106,42,INT32_MAX,43},{104,43,INT32_MAX,44},{102,44,INT32_MAX,45},{100,45,INT32_MAX,46},{96,47,INT32_MAX,INT32_MAX},
//Third angle
{32,47,INT32_MAX,48},{30,46,INT32_MAX,47},{28,45,INT32_MAX,46},{26,44,INT32_MAX,45},{24,43,INT32_MAX,44},{22,42,INT32_MAX,43},{20,41,INT32_MAX,42},{18,40,INT32_MAX,41},{16,39,INT32_MAX,40},{14,38,INT32_MAX,39},{12,37,INT32_MAX,38},{10,36,INT32_MAX,37},{8,35,INT32_MAX,36},{6,34,INT32_MAX,35},{4,33,INT32_MAX,34},{2,32,INT32_MAX,33},{0,31,INT32_MAX,32},{-2,30,INT32_MAX,31},{-4,29,INT32_MAX,30},{-6,28,INT32_MAX,29},{-8,27,INT32_MAX,28},{-10,26,INT32_MAX,27},{-12,25,INT32_MAX,26},{-14,24,INT32_MAX,25},{-16,23,INT32_MAX,24},{-18,22,INT32_MAX,23},{-20,21,INT32_MAX,22},{-22,20,INT32_MAX,21},{-24,19,INT32_MAX,20},{-26,18,INT32_MAX,19},{-28,17,INT32_MAX,18},{-30,16,INT32_MAX,17},{INT32_MIN,INT32_MIN,INT32_MAX,16},{INT32_MIN,47,-32,48},{INT32_MIN,46,-30,47},{INT32_MIN,45,-28,46},{INT32_MIN,44,-26,45},{INT32_MIN,43,-24,44},{INT32_MIN,42,-22,43},{INT32_MIN,41,-20,42},{INT32_MIN,40,-18,41},{INT32_MIN,39,-16,40},{INT32_MIN,38,-14,39},{INT32_MIN,37,-12,38},{INT32_MIN,36,-10,37},{INT32_MIN,35,-8,36},{INT32_MIN,34,-6,35},{INT32_MIN,33,-4,34},{INT32_MIN,32,-2,33},{INT32_MIN,31,0,32},{INT32_MIN,30,-2,31},{INT32_MIN,29,-4,30},{INT32_MIN,28,-6,29},{INT32_MIN,27,-8,28},{INT32_MIN,26,-10,27},{INT32_MIN,25,-12,26},{INT32_MIN,24,-14,25},{INT32_MIN,23,-16,24},{INT32_MIN,22,-18,23},{INT32_MIN,21,-20,22},{INT32_MIN,20,-22,21},{INT32_MIN,19,-24,20},{INT32_MIN,18,-26,19},{INT32_MIN,17,-28,18},{INT32_MIN,16,-30,17},{-2,62,2,63},{-4,61,4,62},{-6,60,6,61},{-8,59,8,60},{-10,58,10,59},{-12,57,12,58},{-14,56,14,57},{-16,55,16,56},{-18,54,18,55},{-20,53,20,54},{-22,52,22,53},{-24,51,24,52},{-26,50,26,51},{-28,49,28,50},{-30,48,30,49},{-32,47,32,48},{-30,46,30,47},{-28,45,28,46},{-26,44,26,45},{-24,43,24,44},{-22,42,22,43},{-20,41,20,42},{-18,40,18,41},{-16,39,16,40},{-14,38,14,39},{-12,37,12,38},{-10,36,10,37},{-8,35,8,36},{-6,34,6,35},{-4,33,4,34},{-2,32,2,33},{INT32_MIN,79,-32,80},{INT32_MIN,78,-30,79},{INT32_MIN,77,-28,78},{INT32_MIN,76,-26,77},{INT32_MIN,75,-24,76},{INT32_MIN,74,-22,75},{INT32_MIN,73,-20,74},{INT32_MIN,72,-18,73},{INT32_MIN,71,-16,72},{INT32_MIN,70,-14,71},{INT32_MIN,69,-12,70},{INT32_MIN,68,-10,69},{INT32_MIN,67,-8,68},{INT32_MIN,66,-6,67},{INT32_MIN,65,-4,66},{INT32_MIN,64,-2,65},{INT32_MIN,63,0,64},{INT32_MIN,62,-2,63},{INT32_MIN,61,-4,62},{INT32_MIN,60,-6,61},{INT32_MIN,59,-8,60},{INT32_MIN,58,-10,59},{INT32_MIN,57,-12,58},{INT32_MIN,56,-14,57},{INT32_MIN,55,-16,56},{INT32_MIN,54,-18,55},{INT32_MIN,53,-20,54},{INT32_MIN,52,-22,53},{INT32_MIN,51,-24,52},{INT32_MIN,50,-26,51},{INT32_MIN,49,-28,50},{INT32_MIN,48,-30,49},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{-2,64,INT32_MAX,65},{-4,65,INT32_MAX,66},{-6,66,INT32_MAX,67},{-8,67,INT32_MAX,68},{-10,68,INT32_MAX,69},{-12,69,INT32_MAX,70},{-14,70,INT32_MAX,71},{-16,71,INT32_MAX,72},{-18,72,INT32_MAX,73},{-20,73,INT32_MAX,74},{-22,74,INT32_MAX,75},{-24,75,INT32_MAX,76},{-26,76,INT32_MAX,77},{-28,77,INT32_MAX,78},{-30,78,INT32_MAX,79},{INT32_MIN,79,INT32_MAX,INT32_MAX},
//Fourth angle
{-32,15,INT32_MAX,INT32_MAX},{-30,14,INT32_MAX,15},{-28,13,INT32_MAX,14},{-26,12,INT32_MAX,13},{-24,11,INT32_MAX,12},{-22,10,INT32_MAX,11},{-20,9,INT32_MAX,10},{-18,8,INT32_MAX,9},{-16,7,INT32_MAX,8},{-14,6,INT32_MAX,7},{-12,5,INT32_MAX,6},{-10,4,INT32_MAX,5},{-8,3,INT32_MAX,4},{-6,2,INT32_MAX,3},{-4,1,INT32_MAX,2},{-2,0,INT32_MAX,1},{0,-1,INT32_MAX,0},{-2,-2,INT32_MAX,-1},{-4,-3,INT32_MAX,-2},{-6,-4,INT32_MAX,-3},{-8,-5,INT32_MAX,-4},{-10,-6,INT32_MAX,-5},{-12,-7,INT32_MAX,-6},{-14,-8,INT32_MAX,-7},{-16,-9,INT32_MAX,-8},{-18,-10,INT32_MAX,-9},{-20,-11,INT32_MAX,-10},{-22,-12,INT32_MAX,-11},{-24,-13,INT32_MAX,-12},{-26,-14,INT32_MAX,-13},{-28,-15,INT32_MAX,-14},{-30,-16,INT32_MAX,-15},{-32,INT32_MIN,INT32_MAX,-16},{-34,14,-30,15},{-36,13,-28,14},{-38,12,-26,13},{-40,11,-24,12},{-42,10,-22,11},{-44,9,-20,10},{-46,8,-18,9},{-48,7,-16,8},{-50,6,-14,7},{-52,5,-12,6},{-54,4,-10,5},{-56,3,-8,4},{-58,2,-6,3},{-60,1,-4,2},{-62,0,-2,1},{-64,-1,0,0},{-62,-2,-2,-1},{-60,-3,-4,-2},{-58,-4,-6,-3},{-56,-5,-8,-4},{-54,-6,-10,-5},{-52,-7,-12,-6},{-50,-8,-14,-7},{-48,-9,-16,-8},{-46,-10,-18,-9},{-44,-11,-20,-10},{-42,-12,-22,-11},{-40,-13,-24,-12},{-38,-14,-26,-13},{-36,-15,-28,-14},{-34,-16,-30,-15},{-96,15,-32,INT32_MAX},{-94,14,-34,15},{-92,13,-36,14},{-90,12,-38,13},{-88,11,-40,12},{-86,10,-42,11},{-84,9,-44,10},{-82,8,-46,9},{-80,7,-48,8},{-78,6,-50,7},{-76,5,-52,6},{-74,4,-54,5},{-72,3,-56,4},{-70,2,-58,3},{-68,1,-60,2},{-66,0,-62,1},{-66,-2,-62,-1},{-68,-3,-60,-2},{-70,-4,-58,-3},{-72,-5,-56,-4},{-74,-6,-54,-5},{-76,-7,-52,-6},{-78,-8,-50,-7},{-80,-9,-48,-8},{-82,-10,-46,-9},{-84,-11,-44,-10},{-86,-12,-42,-11},{-88,-13,-40,-12},{-90,-14,-38,-13},{-92,-15,-36,-14},{-94,-16,-34,-15},{-96,INT32_MIN,-32,-16},{-98,14,-94,15},{-100,13,-92,14},{-102,12,-90,13},{-104,11,-88,12},{-106,10,-86,11},{-108,9,-84,10},{-110,8,-82,9},{-112,7,-80,8},{-114,6,-78,7},{-116,5,-76,6},{-118,4,-74,5},{-120,3,-72,4},{-122,2,-70,3},{-124,1,-68,2},{-126,0,-66,1},{-128,-1,-64,0},{-126,-2,-66,-1},{-124,-3,-68,-2},{-122,-4,-70,-3},{-120,-5,-72,-4},{-118,-6,-74,-5},{-116,-7,-76,-6},{-114,-8,-78,-7},{-112,-9,-80,-8},{-110,-10,-82,-9},{-108,-11,-84,-10},{-106,-12,-86,-11},{-104,-13,-88,-12},{-102,-14,-90,-13},{-100,-15,-92,-14},{-98,-16,-94,-15},{INT32_MIN,15,-96,INT32_MAX},{INT32_MIN,14,-98,15},{INT32_MIN,13,-100,14},{INT32_MIN,12,-102,13},{INT32_MIN,11,-104,12},{INT32_MIN,10,-106,11},{INT32_MIN,9,-108,10},{INT32_MIN,8,-110,9},{INT32_MIN,7,-112,8},{INT32_MIN,6,-114,7},{INT32_MIN,5,-116,6},{INT32_MIN,4,-118,5},{INT32_MIN,3,-120,4},{INT32_MIN,2,-122,3},{INT32_MIN,1,-124,2},{INT32_MIN,0,-126,1},{INT32_MIN,-1,-128,0},{INT32_MIN,-1,-128,0},{INT32_MIN,-2,-126,-1},{INT32_MIN,-3,-124,-2},{INT32_MIN,-4,-122,-3},{INT32_MIN,-5,-120,-4},{INT32_MIN,-6,-118,-5},{INT32_MIN,-7,-116,-6},{INT32_MIN,-8,-114,-7},{INT32_MIN,-9,-112,-8},{INT32_MIN,-10,-110,-9},{INT32_MIN,-11,-108,-10},{INT32_MIN,-12,-106,-11},{INT32_MIN,-13,-104,-12},{INT32_MIN,-14,-102,-13},{INT32_MIN,-15,-100,-14},{INT32_MIN,INT32_MIN,-96,-16}
};
mask_t medium_turn_masks[]={
{0,33,0,0,medium_turn_rects},{0,32,-32,16,medium_turn_rects+33},{0,31,0,32,medium_turn_rects+65},{0,32,-32,48,medium_turn_rects+96},{0,32,0,64,medium_turn_rects+128},
{0,33,0,0,medium_turn_rects+160},{0,31,-32,-16,medium_turn_rects+193},{0,32,-64,0,medium_turn_rects+224},{0,31,-96,-16,medium_turn_rects+256},{0,33,-128,0,medium_turn_rects+287},
{0,33,0,0,medium_turn_rects+320},{0,32,32,-16,medium_turn_rects+353},{0,31,0,-32,medium_turn_rects+384},{0,32,32,-48,medium_turn_rects+416},{0,32,0,-64,medium_turn_rects+448},
{0,33,0,0,medium_turn_rects+480},{0,31,32,16,medium_turn_rects+513},{0,32,64,0,medium_turn_rects+544},{0,31,96,16,medium_turn_rects+576},{0,33,128,0,medium_turn_rects+607}
};
track_section_t medium_turn_left={0,medium_turn_left_curve,1.25*TILE_SIZE*3.1415926,{{0,5,medium_turn_masks},{0,5,medium_turn_masks+5},{0,5,medium_turn_masks+10},{0,5,medium_turn_masks+15}}};
rect_t large_turn_left_to_diag_rects[]={
//First angle
{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{62,-32,INT32_MAX,-31},{60,-31,INT32_MAX,-30},{58,-30,INT32_MAX,-29},{56,-29,INT32_MAX,-28},{54,-28,INT32_MAX,-27},{52,-27,INT32_MAX,-26},{50,-26,INT32_MAX,-25},{48,-25,INT32_MAX,-24},{46,-24,INT32_MAX,-23},{44,-23,INT32_MAX,-22},{42,-22,INT32_MAX,-21},{40,-21,INT32_MAX,-20},{38,-20,INT32_MAX,-19},{36,-19,INT32_MAX,-18},{34,-18,INT32_MAX,-17},{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{2,0,INT32_MAX,1},{4,1,INT32_MAX,2},{6,2,INT32_MAX,3},{8,3,INT32_MAX,4},{10,4,INT32_MAX,5},{12,5,INT32_MAX,6},{14,6,INT32_MAX,7},{16,7,INT32_MAX,8},{18,8,INT32_MAX,9},{20,9,INT32_MAX,10},{22,10,INT32_MAX,11},{24,11,INT32_MAX,12},{26,12,INT32_MAX,13},{28,13,INT32_MAX,14},{30,14,INT32_MAX,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,62,-31},{INT32_MIN,-31,-4,-30},{4,-31,60,-30},{INT32_MIN,-30,-6,-29},{6,-30,58,-29},{INT32_MIN,-29,-8,-28},{8,-29,56,-28},{INT32_MIN,-28,-10,-27},{10,-28,54,-27},{INT32_MIN,-27,-12,-26},{12,-27,52,-26},{INT32_MIN,-26,-14,-25},{14,-26,50,-25},{INT32_MIN,-25,-16,-24},{16,-25,48,-24},{INT32_MIN,-24,-18,-23},{18,-24,46,-23},{INT32_MIN,-23,-20,-22},{20,-23,44,-22},{INT32_MIN,-22,-22,-21},{22,-22,42,-21},{INT32_MIN,-21,-24,-20},{24,-21,40,-20},{INT32_MIN,-20,-26,-19},{26,-20,38,-19},{INT32_MIN,-19,-28,-18},{28,-19,36,-18},{INT32_MIN,-18,-30,-17},{30,-18,34,-17},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,INT32_MAX},
{30,16,34,17},{28,17,36,18},{26,18,38,19},{24,19,40,20},{22,20,42,21},{20,21,44,22},{18,22,46,23},{16,23,48,24},{14,24,50,25},{12,25,52,26},{10,26,54,27},{8,27,56,28},{6,28,58,29},{4,29,60,30},{2,30,62,31},{0,31,64,32},{2,32,62,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},
{32,INT32_MIN,96,16},{34,16,94,17},{36,17,92,18},{38,18,90,19},{40,19,88,20},{42,20,86,21},{44,21,84,22},{46,22,82,23},{48,23,80,24},{50,24,78,25},{52,25,76,26},{54,26,74,27},{56,27,72,28},{58,28,70,29},{60,29,68,30},{62,30,66,31},{62,32,66,33},{60,33,68,34},{58,34,70,35},{56,35,72,36},{54,36,74,37},{52,37,76,38},{50,38,78,39},{48,39,80,40},{46,40,82,41},{44,41,84,42},{42,42,86,43},{40,43,88,44},{38,44,90,45},{36,45,92,46},{34,46,94,47},{32,47,96,INT32_MAX},
{96,INT32_MIN,INT32_MAX,16},{94,16,INT32_MAX,17},{92,17,INT32_MAX,18},{90,18,INT32_MAX,19},{88,19,INT32_MAX,20},{86,20,INT32_MAX,21},{84,21,INT32_MAX,22},{82,22,INT32_MAX,23},{80,23,INT32_MAX,24},{78,24,INT32_MAX,25},{76,25,INT32_MAX,26},{74,26,INT32_MAX,27},{72,27,INT32_MAX,28},{70,28,INT32_MAX,29},{68,29,INT32_MAX,30},{66,30,INT32_MAX,31},{64,31,INT32_MAX,32},{66,32,INT32_MAX,33},{68,33,INT32_MAX,34},{70,34,INT32_MAX,35},{72,35,INT32_MAX,36},{74,36,INT32_MAX,37},{76,37,INT32_MAX,38},{78,38,INT32_MAX,39},{80,39,INT32_MAX,40},{82,40,INT32_MAX,41},{84,41,INT32_MAX,42},{86,42,INT32_MAX,43},{88,43,INT32_MAX,44},{90,44,INT32_MAX,45},{92,45,INT32_MAX,46},{94,46,INT32_MAX,47},{96,47,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},
{INT32_MIN,INT32_MIN,-32,16},{INT32_MIN,16,-30,17},{INT32_MIN,17,-28,18},{INT32_MIN,18,-26,19},{INT32_MIN,19,-24,20},{INT32_MIN,20,-22,21},{INT32_MIN,21,-20,22},{INT32_MIN,22,-18,23},{INT32_MIN,23,-16,24},{INT32_MIN,24,-14,25},{INT32_MIN,25,-12,26},{INT32_MIN,26,-10,27},{INT32_MIN,27,-8,28},{INT32_MIN,28,-6,29},{INT32_MIN,29,-4,30},{INT32_MIN,30,-2,31},{-64,31,0,32},{-62,32,-2,33},{-60,33,-4,34},{-58,34,-6,35},{-56,35,-8,36},{-54,36,-10,37},{-52,37,-12,38},{-50,38,-14,39},{-48,39,-16,40},{-46,40,-18,41},{-44,41,-20,42},{-42,42,-22,43},{-40,43,-24,44},{-38,44,-26,45},{-36,45,-28,46},{-34,46,-30,47},
{0,31,INT32_MAX,32},{-2,32,INT32_MAX,33},{-4,33,INT32_MAX,34},{-6,34,INT32_MAX,35},{-8,35,INT32_MAX,36},{-10,36,INT32_MAX,37},{-12,37,INT32_MAX,38},{-14,38,INT32_MAX,39},{-16,39,INT32_MAX,40},{-18,40,INT32_MAX,41},{-20,41,INT32_MAX,42},{-22,42,INT32_MAX,43},{-24,43,INT32_MAX,44},{-26,44,INT32_MAX,45},{-28,45,INT32_MAX,46},{-30,46,INT32_MAX,47},{-32,47,INT32_MAX,48},{-30,48,INT32_MAX,49},{-28,49,INT32_MAX,50},{-26,50,INT32_MAX,51},{-24,51,INT32_MAX,52},{-22,52,INT32_MAX,53},{-20,53,INT32_MAX,54},{-18,54,INT32_MAX,55},{-16,55,INT32_MAX,56},{-14,56,INT32_MAX,57},{-12,57,INT32_MAX,58},{-10,58,INT32_MAX,59},{-8,59,INT32_MAX,60},{-6,60,INT32_MAX,61},{-4,61,INT32_MAX,62},{-2,62,INT32_MAX,63},{0,63,INT32_MAX,INT32_MAX},
{INT32_MIN,31,-64,32},{INT32_MIN,32,-62,33},{INT32_MIN,33,-60,34},{INT32_MIN,34,-58,35},{INT32_MIN,35,-56,36},{INT32_MIN,36,-54,37},{INT32_MIN,37,-52,38},{INT32_MIN,38,-50,39},{INT32_MIN,39,-48,40},{INT32_MIN,40,-46,41},{INT32_MIN,41,-44,42},{INT32_MIN,42,-42,43},{INT32_MIN,43,-40,44},{INT32_MIN,44,-38,45},{INT32_MIN,45,-36,46},{INT32_MIN,46,-34,47},{INT32_MIN,47,-32,48},{INT32_MIN,48,-30,49},{INT32_MIN,49,-28,50},{INT32_MIN,50,-26,51},{INT32_MIN,51,-24,52},{INT32_MIN,52,-22,53},{INT32_MIN,53,-20,54},{INT32_MIN,54,-18,55},{INT32_MIN,55,-16,56},{INT32_MIN,56,-14,57},{INT32_MIN,57,-12,58},{INT32_MIN,58,-10,59},{INT32_MIN,59,-8,60},{INT32_MIN,60,-6,61},{INT32_MIN,61,-4,62},{INT32_MIN,62,-2,63},{INT32_MIN,63,0,INT32_MAX},
//Fourth angle
{0,INT32_MIN,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{-32,15,INT32_MAX,INT32_MAX},
{-64,INT32_MIN,0,0},{-62,0,-2,1},{-60,1,-4,2},{-58,2,-6,3},{-56,3,-8,4},{-54,4,-10,5},{-52,5,-12,6},{-50,6,-14,7},{-48,7,-16,8},{-46,8,-18,9},{-44,9,-20,10},{-42,10,-22,11},{-40,11,-24,12},{-38,12,-26,13},{-36,13,-28,14},{-34,14,-30,15},
{-66,0,-62,1},{-68,1,-60,2},{-70,2,-58,3},{-72,3,-56,4},{-74,4,-54,5},{-76,5,-52,6},{-78,6,-50,7},{-80,7,-48,8},{-82,8,-46,9},{-84,9,-44,10},{-86,10,-42,11},{-88,11,-40,12},{-90,12,-38,13},{-92,13,-36,14},{-94,14,-34,15},{-96,15,-32,INT32_MAX},
{INT32_MIN,INT32_MIN,-64,0},{INT32_MIN,0,-66,1},{INT32_MIN,1,-68,2},{INT32_MIN,2,-70,3},{INT32_MIN,3,-72,4},{INT32_MIN,4,-74,5},{INT32_MIN,5,-76,6},{INT32_MIN,6,-78,7},{INT32_MIN,7,-80,8},{INT32_MIN,8,-82,9},{INT32_MIN,9,-84,10},{INT32_MIN,10,-86,11},{INT32_MIN,11,-88,12},{INT32_MIN,12,-90,13},{INT32_MIN,13,-92,14},{INT32_MIN,14,-94,15},{INT32_MIN,15,-96,INT32_MAX},
};
mask_t large_turn_left_to_diag_masks[]={
{0,33,0,0,large_turn_left_to_diag_rects},{0,47,-32,16,large_turn_left_to_diag_rects+33},{0,31,0,32,large_turn_left_to_diag_rects+80},{0,31,-32,48,large_turn_left_to_diag_rects+111},
{0,33,0,0,large_turn_left_to_diag_rects+142},{0,31,-32,-16,large_turn_left_to_diag_rects+175},{0,32,-64,0,large_turn_left_to_diag_rects+206},{0,33,-96,-16,large_turn_left_to_diag_rects+238},
{0,16,0,0,large_turn_left_to_diag_rects+271},{0,32,32,-16,large_turn_left_to_diag_rects+287},{0,33,0,-32,large_turn_left_to_diag_rects+319},{0,33,32,-48,large_turn_left_to_diag_rects+352},
{0,17,0,0,large_turn_left_to_diag_rects+385},{0,16,32,16,large_turn_left_to_diag_rects+402},{0,16,64,0,large_turn_left_to_diag_rects+418},{0,17,96,16,large_turn_left_to_diag_rects+434},
};
track_section_t large_turn_left_to_diag={0,large_turn_left_to_diag_curve,0.875*TILE_SIZE*3.1415926,{{0,4,large_turn_left_to_diag_masks},{0,4,large_turn_left_to_diag_masks+4},{0,4,large_turn_left_to_diag_masks+8},{0,4,large_turn_left_to_diag_masks+12}}};
rect_t large_turn_right_to_diag_rects[]={
//First angle
{INT32_MIN,INT32_MIN,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,16},{96,15,INT32_MAX,16},{INT32_MIN,16,34,17},{94,16,INT32_MAX,17},{INT32_MIN,17,36,18},{92,17,INT32_MAX,18},{INT32_MIN,18,38,19},{90,18,INT32_MAX,19},{INT32_MIN,19,40,20},{88,19,INT32_MAX,20},{INT32_MIN,20,42,21},{86,20,INT32_MAX,21},{INT32_MIN,21,44,22},{84,21,INT32_MAX,22},{INT32_MIN,22,46,23},{82,22,INT32_MAX,23},{INT32_MIN,23,48,24},{80,23,INT32_MAX,24},{INT32_MIN,24,50,25},{78,24,INT32_MAX,25},{INT32_MIN,25,52,26},{76,25,INT32_MAX,26},{INT32_MIN,26,54,27},{74,26,INT32_MAX,27},{INT32_MIN,27,56,28},{72,27,INT32_MAX,28},{INT32_MIN,28,58,29},{70,28,INT32_MAX,29},{INT32_MIN,29,60,30},{68,29,INT32_MAX,30},{INT32_MIN,30,62,31},{66,30,INT32_MAX,31},{INT32_MIN,31,INT32_MAX,INT32_MAX},
{0,INT32_MIN,64,0},{2,0,62,1},{4,1,60,2},{6,2,58,3},{8,3,56,4},{10,4,54,5},{12,5,52,6},{14,6,50,7},{16,7,48,8},{18,8,46,9},{20,9,44,10},{22,10,42,11},{24,11,40,12},{26,12,38,13},{28,13,36,14},{30,14,34,15},
{62,0,66,1},{60,1,68,2},{58,2,70,3},{56,3,72,4},{54,4,74,5},{52,5,76,6},{50,6,78,7},{48,7,80,8},{46,8,82,9},{44,9,84,10},{42,10,86,11},{40,11,88,12},{38,12,90,13},{36,13,92,14},{34,14,94,15},{32,15,96,16},{34,16,94,17},{36,17,92,18},{38,18,90,19},{40,19,88,20},{42,20,86,21},{44,21,84,22},{46,22,82,23},{48,23,80,24},{50,24,78,25},{52,25,76,26},{54,26,74,27},{56,27,72,28},{58,28,70,29},{60,29,68,30},{62,30,66,31},
{64,INT32_MIN,INT32_MAX,0},{66,0,INT32_MAX,1},{68,1,INT32_MAX,2},{70,2,INT32_MAX,3},{72,3,INT32_MAX,4},{74,4,INT32_MAX,5},{76,5,INT32_MAX,6},{78,6,INT32_MAX,7},{80,7,INT32_MAX,8},{82,8,INT32_MAX,9},{84,9,INT32_MAX,10},{86,10,INT32_MAX,11},{88,11,INT32_MAX,12},{90,12,INT32_MAX,13},{92,13,INT32_MAX,14},{94,14,INT32_MAX,15},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},
{32,INT32_MIN,INT32_MAX,16},{30,16,INT32_MAX,17},{28,17,INT32_MAX,18},{26,18,INT32_MAX,19},{24,19,INT32_MAX,20},{22,20,INT32_MAX,21},{20,21,INT32_MAX,22},{18,22,INT32_MAX,23},{16,23,INT32_MAX,24},{14,24,INT32_MAX,25},{12,25,INT32_MAX,26},{10,26,INT32_MAX,27},{8,27,INT32_MAX,28},{6,28,INT32_MAX,29},{4,29,INT32_MAX,30},{2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{2,32,62,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},
{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,48},{INT32_MIN,48,30,49},{INT32_MIN,49,28,50},{INT32_MIN,50,26,51},{INT32_MIN,51,24,52},{INT32_MIN,52,22,53},{INT32_MIN,53,20,54},{INT32_MIN,54,18,55},{INT32_MIN,55,16,56},{INT32_MIN,56,14,57},{INT32_MIN,57,12,58},{INT32_MIN,58,10,59},{INT32_MIN,59,8,60},{INT32_MIN,60,6,61},{INT32_MIN,61,4,62},{INT32_MIN,62,2,63},{INT32_MIN,63,0,64},{INT32_MIN,64,-2,INT32_MAX},
{62,32,INT32_MAX,33},{60,33,INT32_MAX,34},{58,34,INT32_MAX,35},{56,35,INT32_MAX,36},{54,36,INT32_MAX,37},{52,37,INT32_MAX,38},{50,38,INT32_MAX,39},{48,39,INT32_MAX,40},{46,40,INT32_MAX,41},{44,41,INT32_MAX,42},{42,42,INT32_MAX,43},{40,43,INT32_MAX,44},{38,44,INT32_MAX,45},{36,45,INT32_MAX,46},{34,46,INT32_MAX,47},{32,47,INT32_MAX,48},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{-2,64,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},
{-34,16,-30,17},{-36,17,-28,18},{-38,18,-26,19},{-40,19,-24,20},{-42,20,-22,21},{-44,21,-20,22},{-46,22,-18,23},{-48,23,-16,24},{-50,24,-14,25},{-52,25,-12,26},{-54,26,-10,27},{-56,27,-8,28},{-58,28,-6,29},{-60,29,-4,30},{-62,30,-2,31},{-64,31,INT32_MAX,32},{-62,32,INT32_MAX,33},{-60,33,INT32_MAX,34},{-58,34,INT32_MAX,35},{-56,35,INT32_MAX,36},{-54,36,INT32_MAX,37},{-52,37,INT32_MAX,38},{-50,38,INT32_MAX,39},{-48,39,INT32_MAX,40},{-46,40,INT32_MAX,41},{-44,41,INT32_MAX,42},{-42,42,INT32_MAX,43},{-40,43,INT32_MAX,44},{-38,44,INT32_MAX,45},{-36,45,INT32_MAX,46},{-34,46,INT32_MAX,47},{-32,47,INT32_MAX,INT32_MAX},
{-96,INT32_MIN,-32,16},{-94,16,-34,17},{-92,17,-36,18},{-90,18,-38,19},{-88,19,-40,20},{-86,20,-42,21},{-84,21,-44,22},{-82,22,-46,23},{-80,23,-48,24},{-78,24,-50,25},{-76,25,-52,26},{-74,26,-54,27},{-72,27,-56,28},{-70,28,-58,29},{-68,29,-60,30},{-66,30,-62,31},{-66,32,-62,33},{-68,33,-60,34},{-70,34,-58,35},{-72,35,-56,36},{-74,36,-54,37},{-76,37,-52,38},{-78,38,-50,39},{-80,39,-48,40},{-82,40,-46,41},{-84,41,-44,42},{-86,42,-42,43},{-88,43,-40,44},{-90,44,-38,45},{-92,45,-36,46},{-94,46,-34,47},{-96,47,-32,INT32_MAX},
{INT32_MIN,INT32_MIN,-96,16},{INT32_MIN,16,-94,17},{INT32_MIN,17,-92,18},{INT32_MIN,18,-90,19},{INT32_MIN,19,-88,20},{INT32_MIN,20,-86,21},{INT32_MIN,21,-84,22},{INT32_MIN,22,-82,23},{INT32_MIN,23,-80,24},{INT32_MIN,24,-78,25},{INT32_MIN,25,-76,26},{INT32_MIN,26,-74,27},{INT32_MIN,27,-72,28},{INT32_MIN,28,-70,29},{INT32_MIN,29,-68,30},{INT32_MIN,30,-66,31},{INT32_MIN,31,-64,32},{INT32_MIN,32,-66,33},{INT32_MIN,33,-68,34},{INT32_MIN,34,-70,35},{INT32_MIN,35,-72,36},{INT32_MIN,36,-74,37},{INT32_MIN,37,-76,38},{INT32_MIN,38,-78,39},{INT32_MIN,39,-80,40},{INT32_MIN,40,-82,41},{INT32_MIN,41,-84,42},{INT32_MIN,42,-86,43},{INT32_MIN,43,-88,44},{INT32_MIN,44,-90,45},{INT32_MIN,45,-92,46},{INT32_MIN,46,-94,47},{INT32_MIN,47,-96,INT32_MAX},
//Fourth angle
{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{INT32_MIN,-26,-50,-25},{INT32_MIN,-25,-48,-24},{INT32_MIN,-24,-46,-23},{INT32_MIN,-23,-44,-22},{INT32_MIN,-22,-42,-21},{INT32_MIN,-21,-40,-20},{INT32_MIN,-20,-38,-19},{INT32_MIN,-19,-36,-18},{INT32_MIN,-18,-34,-17},{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,-2,1},{INT32_MIN,1,-4,2},{INT32_MIN,2,-6,3},{INT32_MIN,3,-8,4},{INT32_MIN,4,-10,5},{INT32_MIN,5,-12,6},{INT32_MIN,6,-14,7},{INT32_MIN,7,-16,8},{INT32_MIN,8,-18,9},{INT32_MIN,9,-20,10},{INT32_MIN,10,-22,11},{INT32_MIN,11,-24,12},{INT32_MIN,12,-26,13},{INT32_MIN,13,-28,14},{INT32_MIN,14,-30,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,INT32_MAX,-31},{INT32_MIN,-31,-4,-30},{4,-31,INT32_MAX,-30},{INT32_MIN,-30,-6,-29},{6,-30,INT32_MAX,-29},{INT32_MIN,-29,-8,-28},{8,-29,INT32_MAX,-28},{INT32_MIN,-28,-10,-27},{10,-28,INT32_MAX,-27},{INT32_MIN,-27,-12,-26},{12,-27,INT32_MAX,-26},{-50,-26,-14,-25},{14,-26,INT32_MAX,-25},{-48,-25,-16,-24},{16,-25,INT32_MAX,-24},{-46,-24,-18,-23},{18,-24,INT32_MAX,-23},{-44,-23,-20,-22},{20,-23,INT32_MAX,-22},{-42,-22,-22,-21},{22,-22,INT32_MAX,-21},{-40,-21,-24,-20},{24,-21,INT32_MAX,-20},{-38,-20,-26,-19},{26,-20,INT32_MAX,-19},{-36,-19,-28,-18},{28,-19,INT32_MAX,-18},{-34,-18,-30,-17},{30,-18,INT32_MAX,-17}
};
mask_t large_turn_right_to_diag_masks[]={
{0,49,0,0,large_turn_right_to_diag_rects},{0,16,-32,16,large_turn_right_to_diag_rects+49},{0,31,-64,0,large_turn_right_to_diag_rects+65},{0,16,-96,16,large_turn_right_to_diag_rects+96},
{0,16,0,0,large_turn_right_to_diag_rects+112},{0,32,-32,-16,large_turn_right_to_diag_rects+128},{0,34,0,-32,large_turn_right_to_diag_rects+160},{0,33,-32,-48,large_turn_right_to_diag_rects+194},
{0,16,0,0,large_turn_right_to_diag_rects+227},{0,32,32,-16,large_turn_right_to_diag_rects+243},{0,32,64,0,large_turn_right_to_diag_rects+275},{0,33,96,-16,large_turn_right_to_diag_rects+307},
{0,33,0,0,large_turn_right_to_diag_rects+340},{0,41,32,16,large_turn_right_to_diag_rects+373},{0,31,0,32,large_turn_right_to_diag_rects+414},{0,31,32,48,large_turn_right_to_diag_rects+445},
};
track_section_t large_turn_right_to_diag={0,large_turn_right_to_diag_curve,0.875*TILE_SIZE*3.1415926,{{0,4,large_turn_right_to_diag_masks},{0,4,large_turn_right_to_diag_masks+4},{0,4,large_turn_right_to_diag_masks+8},{0,4,large_turn_right_to_diag_masks+12}}};

//Diagonals
rect_t diag_slope_rect={-32,INT32_MIN,32,INT32_MAX};
mask_t diag_slope_mask={0,1,0,0,&diag_slope_rect};
track_section_t flat_diag={TRACK_DIAGONAL,flat_diag_curve,sqrt(2)*TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t flat_to_gentle_up_diag={TRACK_DIAGONAL,flat_to_gentle_up_diag_curve,sqrt(2)*TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t gentle_to_flat_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,gentle_to_flat_up_diag_curve,sqrt(2)*TILE_SIZE,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
track_section_t gentle_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,gentle_diag_curve,sqrt(2)*TILE_SIZE,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
track_section_t gentle_to_steep_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,gentle_to_steep_up_diag_curve,3.0,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
track_section_t steep_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,steep_to_gentle_up_diag_curve,3.0,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
track_section_t steep_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,steep_diag_curve,6.0,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};

//Banked turns
rect_t flat_to_left_bank_rects[]={{INT32_MIN,INT32_MIN,-1,INT32_MAX},{-1,INT32_MIN,INT32_MAX,INT32_MAX}};
mask_t flat_to_left_bank_masks[]={{TRACK_MASK_UNION,1,0,0,flat_to_left_bank_rects},{TRACK_MASK_DIFFERENCE,1,0,0,flat_to_left_bank_rects+1},{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t flat_to_left_bank={0,flat_to_left_bank_curve,TILE_SIZE,{{VIEW_NEEDS_TRACK_MASK,2,flat_to_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,flat_to_left_bank_masks+2},{0,1,NULL},{0,1,NULL}}};
rect_t flat_to_right_bank_rects[]={{-1,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-1,INT32_MAX}};
mask_t flat_to_right_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all},{TRACK_MASK_UNION,1,0,0,flat_to_right_bank_rects},{TRACK_MASK_DIFFERENCE,1,0,0,flat_to_right_bank_rects+1}};
track_section_t flat_to_right_bank={0,flat_to_right_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,flat_to_right_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,flat_to_right_bank_masks+2}}};
rect_t left_bank_to_gentle_up_rects[]={{6,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,6,INT32_MAX}};
mask_t left_bank_to_gentle_up_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all},{TRACK_MASK_UNION,1,0,0,left_bank_to_gentle_up_rects},{TRACK_MASK_DIFFERENCE,1,0,0,left_bank_to_gentle_up_rects+1}};
track_section_t left_bank_to_gentle_up={0,left_bank_to_gentle_up_curve,TILE_SIZE,{{VIEW_NEEDS_TRACK_MASK,2,left_bank_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,2,left_bank_to_gentle_up_masks+2},{0,1,NULL},{0,1,NULL}}};
rect_t right_bank_to_gentle_up_rects[]={{INT32_MIN,INT32_MIN,-5,INT32_MAX},{-5,INT32_MIN,INT32_MAX,INT32_MAX}};
mask_t right_bank_to_gentle_up_masks[]={{TRACK_MASK_UNION,1,0,0,right_bank_to_gentle_up_rects},{TRACK_MASK_DIFFERENCE,1,0,0,right_bank_to_gentle_up_rects+1},{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t right_bank_to_gentle_up={0,right_bank_to_gentle_up_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,right_bank_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,2,right_bank_to_gentle_up_masks+2}}};
rect_t gentle_up_to_left_bank_rects[]={{INT32_MIN,INT32_MIN,-2,INT32_MAX},{-2,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-16,INT32_MAX},{-16,INT32_MIN,INT32_MAX,INT32_MAX}};
mask_t gentle_up_to_left_bank_masks[]={{TRACK_MASK_UNION,1,0,0,gentle_up_to_left_bank_rects},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_to_left_bank_rects+1},{TRACK_MASK_UNION,1,0,0,gentle_up_to_left_bank_rects+2},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_to_left_bank_rects+3}};
track_section_t gentle_up_to_left_bank={0,gentle_up_to_left_bank_curve,TILE_SIZE,{{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_left_bank_masks+2},{0,1,NULL},{0,1,NULL}}};
rect_t gentle_up_to_right_bank_rects[]={{13,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,13,INT32_MAX},{-2,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-2,INT32_MAX}};
mask_t gentle_up_to_right_bank_masks[]={{TRACK_MASK_UNION,1,0,0,gentle_up_to_right_bank_rects},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_to_right_bank_rects+1},{TRACK_MASK_UNION,1,0,0,gentle_up_to_right_bank_rects+2},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_to_right_bank_rects+3}};
track_section_t gentle_up_to_right_bank={0,gentle_up_to_right_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_right_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_right_bank_masks+2}}};
track_section_t left_bank={0,left_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
//Note this is not the splitting used for the existing sprites, but it works better
mask_t diagonal_transition_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&diag_slope_rect},{TRACK_MASK_DIFFERENCE,1,0,0,&diag_slope_rect}};
track_section_t flat_to_left_bank_diag={TRACK_DIAGONAL,flat_to_left_bank_diag_curve,sqrt(2)*TILE_SIZE,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t flat_to_right_bank_diag={TRACK_DIAGONAL,flat_to_right_bank_diag_curve,sqrt(2)*TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,NULL}}};
track_section_t left_bank_to_gentle_up_diag={TRACK_DIAGONAL,left_bank_to_gentle_up_diag_curve,sqrt(2)*TILE_SIZE,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t right_bank_to_gentle_up_diag={TRACK_DIAGONAL,right_bank_to_gentle_up_diag_curve,sqrt(2)*TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,NULL}}};
track_section_t gentle_up_to_left_bank_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,gentle_up_to_left_bank_diag_curve,sqrt(2)*TILE_SIZE,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
track_section_t gentle_up_to_right_bank_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,gentle_up_to_right_bank_diag_curve,sqrt(2)*TILE_SIZE,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,&diag_slope_mask}}};
track_section_t left_bank_diag={TRACK_DIAGONAL,left_bank_diag_curve,sqrt(2)*TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
mask_t small_turn_left_bank_masks[]={
	{TRACK_MASK_INTERSECT,13,0,0,small_turn_rects},{TRACK_MASK_DIFFERENCE,13,0,0,small_turn_rects},{0,13,-32,16,small_turn_rects+13},{0,26,0,32,small_turn_rects+26},
	{0,17,0,0,small_turn_rects+52},{0,16,-32,-16,small_turn_rects+69},{0,17,-64,0,small_turn_rects+85},
	{0,26,0,0,small_turn_rects+102},{0,13,32,-16,small_turn_rects+128},{TRACK_MASK_INTERSECT,13,0,-32,small_turn_rects+141},{TRACK_MASK_DIFFERENCE,13,0,-32,small_turn_rects+141},
	{0,4,0,0,small_turn_rects+154},{0,3,32,16,small_turn_rects+158},{0,4,64,0,small_turn_rects+161}

};
track_section_t small_turn_left_bank={0,small_turn_left_bank_curve,0.75*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,4,small_turn_left_bank_masks},{0,3,small_turn_left_bank_masks+4},{VIEW_NEEDS_TRACK_MASK,4,small_turn_left_bank_masks+7},{0,3,small_turn_left_bank_masks+11}}};
mask_t medium_turn_left_bank_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_rects},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_rects},{0,32,-32,16,medium_turn_rects+33},{0,31,0,32,medium_turn_rects+65},{0,32,-32,48,medium_turn_rects+96},{0,32,0,64,medium_turn_rects+128},//TODO proper splitting
{0,33,0,0,medium_turn_rects+160},{0,31,-32,-16,medium_turn_rects+193},{0,32,-64,0,medium_turn_rects+224},{0,31,-96,-16,medium_turn_rects+256},{0,33,-128,0,medium_turn_rects+287},
{0,33,0,0,medium_turn_rects+320},{0,32,32,-16,medium_turn_rects+353},{0,31,0,-32,medium_turn_rects+384},{0,32,32,-48,medium_turn_rects+416},{TRACK_MASK_INTERSECT,32,0,-64,medium_turn_rects+448},{TRACK_MASK_DIFFERENCE,32,0,-64,medium_turn_rects+448},//TODO proper splitting
{0,33,0,0,medium_turn_rects+480},{0,31,32,16,medium_turn_rects+513},{0,32,64,0,medium_turn_rects+544},{0,31,96,16,medium_turn_rects+576},{0,33,128,0,medium_turn_rects+607}
};
track_section_t medium_turn_left_bank={0,medium_turn_left_bank_curve,1.25*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,6,medium_turn_left_bank_masks},{0,5,medium_turn_left_bank_masks+6},{VIEW_NEEDS_TRACK_MASK,6,medium_turn_left_bank_masks+11},{0,5,medium_turn_left_bank_masks+17}}};
rect_t large_turn_left_to_diag_bank_rects[]={
//First view
{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{62,-32,INT32_MAX,-31},{60,-31,INT32_MAX,-30},{58,-30,INT32_MAX,-29},{56,-29,INT32_MAX,-28},{54,-28,INT32_MAX,-27},{52,-27,INT32_MAX,-26},{50,-26,INT32_MAX,-25},{48,-25,INT32_MAX,-24},{46,-24,INT32_MAX,-23},{44,-23,INT32_MAX,-22},{42,-22,INT32_MAX,-21},{40,-21,INT32_MAX,-20},{38,-20,INT32_MAX,-19},{36,-19,INT32_MAX,-18},{34,-18,INT32_MAX,-17},{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{2,0,INT32_MAX,1},{4,1,INT32_MAX,2},{6,2,INT32_MAX,3},{8,3,INT32_MAX,4},{10,4,INT32_MAX,5},{12,5,INT32_MAX,6},{14,6,INT32_MAX,7},{16,7,INT32_MAX,8},{18,8,INT32_MAX,9},{20,9,INT32_MAX,10},{22,10,INT32_MAX,11},{24,11,INT32_MAX,12},{26,12,INT32_MAX,13},{28,13,INT32_MAX,14},{30,14,INT32_MAX,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,62,-31},{INT32_MIN,-31,-4,-30},{4,-31,60,-30},{INT32_MIN,-30,-6,-29},{6,-30,58,-29},{INT32_MIN,-29,-8,-28},{8,-29,56,-28},{INT32_MIN,-28,-10,-27},{10,-28,54,-27},{INT32_MIN,-27,-12,-26},{12,-27,52,-26},{INT32_MIN,-26,-14,-25},{14,-26,50,-25},{INT32_MIN,-25,-16,-24},{16,-25,48,-24},{INT32_MIN,-24,-18,-23},{18,-24,46,-23},{INT32_MIN,-23,-20,-22},{20,-23,44,-22},{INT32_MIN,-22,-22,-21},{22,-22,42,-21},{INT32_MIN,-21,-24,-20},{24,-21,40,-20},{INT32_MIN,-20,-26,-19},{26,-20,38,-19},{INT32_MIN,-19,-28,-18},{28,-19,36,-18},{INT32_MIN,-18,-30,-17},{30,-18,34,-17},
//Second view
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,INT32_MAX},
{32,INT32_MIN,64,16},{30,16,64,17},{28,17,64,18},{26,18,64,19},{24,19,64,20},{22,20,64,21},{20,21,64,22},{18,22,64,23},{16,23,64,24},{14,24,64,25},{12,25,64,26},{10,26,64,27},{8,27,64,28},{6,28,64,29},{4,29,64,30},{2,30,64,31},{0,31,64,32},{2,32,64,33},{4,33,64,34},{6,34,64,35},{8,35,64,36},{10,36,64,37},{12,37,64,38},{14,38,64,39},{16,39,64,40},{18,40,64,41},{20,41,64,42},{22,42,64,43},{24,43,64,44},{26,44,64,45},{28,45,64,46},{30,46,64,47},{32,47,64,INT32_MAX},
{64,INT32_MIN,64,INT32_MAX},
{64,INT32_MIN,INT32_MAX,INT32_MAX},
//Third view
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},
{INT32_MIN,INT32_MIN,-32,16},{INT32_MIN,16,-30,17},{INT32_MIN,17,-28,18},{INT32_MIN,18,-26,19},{INT32_MIN,19,-24,20},{INT32_MIN,20,-22,21},{INT32_MIN,21,-20,22},{INT32_MIN,22,-18,23},{INT32_MIN,23,-16,24},{INT32_MIN,24,-14,25},{INT32_MIN,25,-12,26},{INT32_MIN,26,-10,27},{INT32_MIN,27,-8,28},{INT32_MIN,28,-6,29},{INT32_MIN,29,-4,30},{INT32_MIN,30,-2,31},{-64,31,0,32},{-62,32,-2,33},{-60,33,-4,34},{-58,34,-6,35},{-56,35,-8,36},{-54,36,-10,37},{-52,37,-12,38},{-50,38,-14,39},{-48,39,-16,40},{-46,40,-18,41},{-44,41,-20,42},{-42,42,-22,43},{-40,43,-24,44},{-38,44,-26,45},{-36,45,-28,46},{-34,46,-30,47},
{0,31,INT32_MAX,32},{-2,32,INT32_MAX,33},{-4,33,INT32_MAX,34},{-6,34,INT32_MAX,35},{-8,35,INT32_MAX,36},{-10,36,INT32_MAX,37},{-12,37,INT32_MAX,38},{-14,38,INT32_MAX,39},{-16,39,INT32_MAX,40},{-18,40,INT32_MAX,41},{-20,41,INT32_MAX,42},{-22,42,INT32_MAX,43},{-24,43,INT32_MAX,44},{-26,44,INT32_MAX,45},{-28,45,INT32_MAX,46},{-30,46,INT32_MAX,47},{-32,47,INT32_MAX,48},{-30,48,INT32_MAX,49},{-28,49,INT32_MAX,50},{-26,50,INT32_MAX,51},{-24,51,INT32_MAX,52},{-22,52,INT32_MAX,53},{-20,53,INT32_MAX,54},{-18,54,INT32_MAX,55},{-16,55,INT32_MAX,56},{-14,56,INT32_MAX,57},{-12,57,INT32_MAX,58},{-10,58,INT32_MAX,59},{-8,59,INT32_MAX,60},{-6,60,INT32_MAX,61},{-4,61,INT32_MAX,62},{-2,62,INT32_MAX,63},{0,63,INT32_MAX,INT32_MAX},
{INT32_MIN,31,-64,32},{INT32_MIN,32,-62,33},{INT32_MIN,33,-60,34},{INT32_MIN,34,-58,35},{INT32_MIN,35,-56,36},{INT32_MIN,36,-54,37},{INT32_MIN,37,-52,38},{INT32_MIN,38,-50,39},{INT32_MIN,39,-48,40},{INT32_MIN,40,-46,41},{INT32_MIN,41,-44,42},{INT32_MIN,42,-42,43},{INT32_MIN,43,-40,44},{INT32_MIN,44,-38,45},{INT32_MIN,45,-36,46},{INT32_MIN,46,-34,47},{INT32_MIN,47,-32,48},{INT32_MIN,48,-30,49},{INT32_MIN,49,-28,50},{INT32_MIN,50,-26,51},{INT32_MIN,51,-24,52},{INT32_MIN,52,-22,53},{INT32_MIN,53,-20,54},{INT32_MIN,54,-18,55},{INT32_MIN,55,-16,56},{INT32_MIN,56,-14,57},{INT32_MIN,57,-12,58},{INT32_MIN,58,-10,59},{INT32_MIN,59,-8,60},{INT32_MIN,60,-6,61},{INT32_MIN,61,-4,62},{INT32_MIN,62,-2,63},{INT32_MIN,63,0,INT32_MAX},
//Fourth view
{0,INT32_MIN,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{-32,15,INT32_MAX,INT32_MAX},
{-64,INT32_MIN,0,0},{-62,0,-2,1},{-60,1,-4,2},{-58,2,-6,3},{-56,3,-8,4},{-54,4,-10,5},{-52,5,-12,6},{-50,6,-14,7},{-48,7,-16,8},{-46,8,-18,9},{-44,9,-20,10},{-42,10,-22,11},{-40,11,-24,12},{-38,12,-26,13},{-36,13,-28,14},{-34,14,-30,15},
{-66,0,-62,1},{-68,1,-60,2},{-70,2,-58,3},{-72,3,-56,4},{-74,4,-54,5},{-76,5,-52,6},{-78,6,-50,7},{-80,7,-48,8},{-82,8,-46,9},{-84,9,-44,10},{-86,10,-42,11},{-88,11,-40,12},{-90,12,-38,13},{-92,13,-36,14},{-94,14,-34,15},{-96,15,-32,INT32_MAX},
{INT32_MIN,INT32_MIN,-64,0},{INT32_MIN,0,-66,1},{INT32_MIN,1,-68,2},{INT32_MIN,2,-70,3},{INT32_MIN,3,-72,4},{INT32_MIN,4,-74,5},{INT32_MIN,5,-76,6},{INT32_MIN,6,-78,7},{INT32_MIN,7,-80,8},{INT32_MIN,8,-82,9},{INT32_MIN,9,-84,10},{INT32_MIN,10,-86,11},{INT32_MIN,11,-88,12},{INT32_MIN,12,-90,13},{INT32_MIN,13,-92,14},{INT32_MIN,14,-94,15},{INT32_MIN,15,-96,INT32_MAX}
};
mask_t large_turn_left_to_diag_bank_masks[]={
{0,33,0,0,large_turn_left_to_diag_bank_rects},{0,47,-32,16,large_turn_left_to_diag_bank_rects+33},{0,31,0,32,large_turn_left_to_diag_bank_rects+80},{0,31*2,-32,48,large_turn_left_to_diag_bank_rects+111-31},
{0,33,0,0,large_turn_left_to_diag_bank_rects+142},{0,33,-32,-16,large_turn_left_to_diag_bank_rects+175},{0,1,-64,0,large_turn_left_to_diag_bank_rects+208},{0,1,-96,-16,large_turn_left_to_diag_bank_rects+209},
{0,16,0,0,large_turn_left_to_diag_bank_rects+210},{0,32,32,-16,large_turn_left_to_diag_bank_rects+226},{0,33,0,-32,large_turn_left_to_diag_bank_rects+258},{0,33,32,-48,large_turn_left_to_diag_bank_rects+291},
{0,17,0,0,large_turn_left_to_diag_bank_rects+324},{0,16,32,16,large_turn_left_to_diag_bank_rects+341},{0,16,64,0,large_turn_left_to_diag_bank_rects+357},{0,17,96,16,large_turn_left_to_diag_bank_rects+373},
};
track_section_t large_turn_left_to_diag_bank={0,large_turn_left_to_diag_bank_curve,0.875*TILE_SIZE*3.1415926,{{0,4,large_turn_left_to_diag_bank_masks},{0,4,large_turn_left_to_diag_bank_masks+4},{0,4,large_turn_left_to_diag_bank_masks+8},{0,4,large_turn_left_to_diag_bank_masks+12}}};
rect_t large_turn_right_to_diag_bank_rects[]={
//First angle
{INT32_MIN,INT32_MIN,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,16},{96,15,INT32_MAX,16},{INT32_MIN,16,34,17},{94,16,INT32_MAX,17},{INT32_MIN,17,36,18},{92,17,INT32_MAX,18},{INT32_MIN,18,38,19},{90,18,INT32_MAX,19},{INT32_MIN,19,40,20},{88,19,INT32_MAX,20},{INT32_MIN,20,42,21},{86,20,INT32_MAX,21},{INT32_MIN,21,44,22},{84,21,INT32_MAX,22},{INT32_MIN,22,46,23},{82,22,INT32_MAX,23},{INT32_MIN,23,48,24},{80,23,INT32_MAX,24},{INT32_MIN,24,50,25},{78,24,INT32_MAX,25},{INT32_MIN,25,52,26},{76,25,INT32_MAX,26},{INT32_MIN,26,54,27},{74,26,INT32_MAX,27},{INT32_MIN,27,56,28},{72,27,INT32_MAX,28},{INT32_MIN,28,58,29},{70,28,INT32_MAX,29},{INT32_MIN,29,60,30},{68,29,INT32_MAX,30},{INT32_MIN,30,62,31},{66,30,INT32_MAX,31},{INT32_MIN,31,INT32_MAX,INT32_MAX},
{0,INT32_MIN,64,0},{2,0,62,1},{4,1,60,2},{6,2,58,3},{8,3,56,4},{10,4,54,5},{12,5,52,6},{14,6,50,7},{16,7,48,8},{18,8,46,9},{20,9,44,10},{22,10,42,11},{24,11,40,12},{26,12,38,13},{28,13,36,14},{30,14,34,15},
{62,0,66,1},{60,1,68,2},{58,2,70,3},{56,3,72,4},{54,4,74,5},{52,5,76,6},{50,6,78,7},{48,7,80,8},{46,8,82,9},{44,9,84,10},{42,10,86,11},{40,11,88,12},{38,12,90,13},{36,13,92,14},{34,14,94,15},{32,15,96,16},{34,16,94,17},{36,17,92,18},{38,18,90,19},{40,19,88,20},{42,20,86,21},{44,21,84,22},{46,22,82,23},{48,23,80,24},{50,24,78,25},{52,25,76,26},{54,26,74,27},{56,27,72,28},{58,28,70,29},{60,29,68,30},{62,30,66,31},
{64,INT32_MIN,INT32_MAX,0},{66,0,INT32_MAX,1},{68,1,INT32_MAX,2},{70,2,INT32_MAX,3},{72,3,INT32_MAX,4},{74,4,INT32_MAX,5},{76,5,INT32_MAX,6},{78,6,INT32_MAX,7},{80,7,INT32_MAX,8},{82,8,INT32_MAX,9},{84,9,INT32_MAX,10},{86,10,INT32_MAX,11},{88,11,INT32_MAX,12},{90,12,INT32_MAX,13},{92,13,INT32_MAX,14},{94,14,INT32_MAX,15},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},
{32,INT32_MIN,INT32_MAX,16},{30,16,INT32_MAX,17},{28,17,INT32_MAX,18},{26,18,INT32_MAX,19},{24,19,INT32_MAX,20},{22,20,INT32_MAX,21},{20,21,INT32_MAX,22},{18,22,INT32_MAX,23},{16,23,INT32_MAX,24},{14,24,INT32_MAX,25},{12,25,INT32_MAX,26},{10,26,INT32_MAX,27},{8,27,INT32_MAX,28},{6,28,INT32_MAX,29},{4,29,INT32_MAX,30},{2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{2,32,62,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},
{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,48},{INT32_MIN,48,30,49},{INT32_MIN,49,28,50},{INT32_MIN,50,26,51},{INT32_MIN,51,24,52},{INT32_MIN,52,22,53},{INT32_MIN,53,20,54},{INT32_MIN,54,18,55},{INT32_MIN,55,16,56},{INT32_MIN,56,14,57},{INT32_MIN,57,12,58},{INT32_MIN,58,10,59},{INT32_MIN,59,8,60},{INT32_MIN,60,6,61},{INT32_MIN,61,4,62},{INT32_MIN,62,2,63},{INT32_MIN,63,0,64},{INT32_MIN,64,-2,INT32_MAX},
{62,32,INT32_MAX,33},{60,33,INT32_MAX,34},{58,34,INT32_MAX,35},{56,35,INT32_MAX,36},{54,36,INT32_MAX,37},{52,37,INT32_MAX,38},{50,38,INT32_MAX,39},{48,39,INT32_MAX,40},{46,40,INT32_MAX,41},{44,41,INT32_MAX,42},{42,42,INT32_MAX,43},{40,43,INT32_MAX,44},{38,44,INT32_MAX,45},{36,45,INT32_MAX,46},{34,46,INT32_MAX,47},{32,47,INT32_MAX,48},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{-2,64,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},
{-64,INT32_MIN,-32,16},{-64,16,-30,17},{-64,17,-28,18},{-64,18,-26,19},{-64,19,-24,20},{-64,20,-22,21},{-64,21,-20,22},{-64,22,-18,23},{-64,23,-16,24},{-64,24,-14,25},{-64,25,-12,26},{-64,26,-10,27},{-64,27,-8,28},{-64,28,-6,29},{-64,29,-4,30},{-64,30,-2,31},{-64,31,INT32_MAX,INT32_MAX},
{-64,INT32_MIN,-64,INT32_MAX},
{INT32_MIN,INT32_MIN,-64,INT32_MAX},
//Fourth angle
{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{-32,-10,-18,-9},{18,-10,INT32_MAX,-9},{-32,-9,-16,-8},{16,-9,INT32_MAX,-8},{-32,-8,-14,-7},{14,-8,INT32_MAX,-7},{-32,-7,-12,-6},{12,-7,INT32_MAX,-6},{-32,-6,-10,-5},{10,-6,INT32_MAX,-5},{-32,-5,-8,-4},{8,-5,INT32_MAX,-4},{-32,-4,-6,-3},{6,-4,INT32_MAX,-3},{-32,-3,-4,-2},{4,-3,INT32_MAX,-2},{-32,-2,-2,-1},{2,-2,INT32_MAX,-1},{-32,-1,INT32_MAX,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},

{INT32_MIN,-26,-50,-25},{INT32_MIN,-25,-48,-24},{INT32_MIN,-24,-46,-23},{INT32_MIN,-23,-44,-22},{INT32_MIN,-22,-42,-21},{INT32_MIN,-21,-40,-20},{INT32_MIN,-20,-38,-19},{INT32_MIN,-19,-36,-18},{INT32_MIN,-18,-34,-17},{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,-2,1},{INT32_MIN,1,-4,2},{INT32_MIN,2,-6,3},{INT32_MIN,3,-8,4},{INT32_MIN,4,-10,5},{INT32_MIN,5,-12,6},{INT32_MIN,6,-14,7},{INT32_MIN,7,-16,8},{INT32_MIN,8,-18,9},{INT32_MIN,9,-20,10},{INT32_MIN,10,-22,11},{INT32_MIN,11,-24,12},{INT32_MIN,12,-26,13},{INT32_MIN,13,-28,14},{INT32_MIN,14,-30,15},

{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},

{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,INT32_MAX,-31},{INT32_MIN,-31,-4,-30},{4,-31,INT32_MAX,-30},{INT32_MIN,-30,-6,-29},{6,-30,INT32_MAX,-29},{INT32_MIN,-29,-8,-28},{8,-29,INT32_MAX,-28},{INT32_MIN,-28,-10,-27},{10,-28,INT32_MAX,-27},{INT32_MIN,-27,-12,-26},{12,-27,INT32_MAX,-26},{-50,-26,-14,-25},{14,-26,INT32_MAX,-25},{-48,-25,-16,-24},{16,-25,INT32_MAX,-24},{-46,-24,-18,-23},{18,-24,INT32_MAX,-23},{-44,-23,-20,-22},{20,-23,INT32_MAX,-22},{-42,-22,-22,-21},{22,-22,INT32_MAX,-21},{-40,-21,-24,-20},{24,-21,INT32_MAX,-20},{-38,-20,-26,-19},{26,-20,INT32_MAX,-19},{-36,-19,-28,-18},{28,-19,INT32_MAX,-18},{-34,-18,-30,-17},{30,-18,INT32_MAX,-17},

};
mask_t large_turn_right_to_diag_bank_masks[]={
{0,49,0,0,large_turn_right_to_diag_bank_rects},{0,16,-32,16,large_turn_right_to_diag_bank_rects+49},{0,31,-64,0,large_turn_right_to_diag_bank_rects+65},{0,16,-96,16,large_turn_right_to_diag_bank_rects+96},
{0,16,0,0,large_turn_right_to_diag_bank_rects+112},{0,32,-32,-16,large_turn_right_to_diag_bank_rects+128},{0,34,0,-32,large_turn_right_to_diag_bank_rects+160},{0,33,-32,-48,large_turn_right_to_diag_bank_rects+194},
{0,16,0,0,large_turn_right_to_diag_bank_rects+227},{0,17,32,-16,large_turn_right_to_diag_bank_rects+243},{0,1,64,0,large_turn_right_to_diag_bank_rects+260},{0,1,96,-16,large_turn_right_to_diag_bank_rects+261},
{TRACK_MASK_DIFFERENCE,27,0,0,large_turn_right_to_diag_bank_rects+262},{0,41,32,16,large_turn_right_to_diag_bank_rects+289},{0,31,0,32,large_turn_right_to_diag_bank_rects+330},{0,31,32,48,large_turn_right_to_diag_bank_rects+361},
};
track_section_t large_turn_right_to_diag_bank={0,large_turn_right_to_diag_bank_curve,0.875*TILE_SIZE*3.1415926,{{0,4,large_turn_right_to_diag_bank_masks},{0,4,large_turn_right_to_diag_bank_masks+4},{0,4,large_turn_right_to_diag_bank_masks+8},{VIEW_NEEDS_TRACK_MASK,4,large_turn_right_to_diag_bank_masks+12}}};


//Sloped turns
//TODO fix these rects
rect_t small_turn_left_gentle_up_rects[]={
{INT32_MIN,-32,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,-32},
{INT32_MIN,INT32_MIN,32,INT32_MAX},{32,INT32_MIN,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,9},{INT32_MIN,9,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-32,INT32_MAX},
};
mask_t small_turn_left_gentle_up_masks[]={
{0,1,-6,-3,small_turn_left_gentle_up_rects},{0,1,6,45,small_turn_left_gentle_up_rects+1},
{0,1,6,-3,small_turn_left_gentle_up_rects+2},{0,1,-70,13,small_turn_left_gentle_up_rects+3},
{0,1,-6,-3,small_turn_left_gentle_up_rects+4},{0,1,6,-19,small_turn_left_gentle_up_rects+5},
{0,1,6,-3,small_turn_left_gentle_up_rects+6},{0,1,58,13,small_turn_left_gentle_up_rects+7}
};
track_section_t small_turn_left_gentle_up={0,small_turn_left_gentle_up_curve,NORM(0.75*M_PI*TILE_SIZE,3.0),{{0,2,small_turn_left_gentle_up_masks},{0,2,small_turn_left_gentle_up_masks+2},{0,2,small_turn_left_gentle_up_masks+4},{0,2,small_turn_left_gentle_up_masks+6}}};
rect_t small_turn_right_gentle_up_rects[]={
{INT32_MIN,INT32_MIN,32,INT32_MAX},{32,INT32_MIN,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,9},{INT32_MIN,9,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-32,INT32_MAX},
{INT32_MIN,-25,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,-25},
};
mask_t small_turn_right_gentle_up_masks[]={
{0,1,-6,-3,small_turn_right_gentle_up_rects},{0,1,-58,13,small_turn_right_gentle_up_rects+1},
{0,1,6,-3,small_turn_right_gentle_up_rects+2},{0,1,-6,-19,small_turn_right_gentle_up_rects+3},
{0,1,-6,-3,small_turn_right_gentle_up_rects+4},{0,1,70,13,small_turn_right_gentle_up_rects+5},
{0,1,6,-3,small_turn_right_gentle_up_rects+6},{0,1,-6,45,small_turn_right_gentle_up_rects+7}
};
track_section_t small_turn_right_gentle_up={0,small_turn_right_gentle_up_curve,NORM(0.75*M_PI*TILE_SIZE,3.0),{{0,2,small_turn_right_gentle_up_masks},{0,2,small_turn_right_gentle_up_masks+2},{0,2,small_turn_right_gentle_up_masks+4},{0,2,small_turn_right_gentle_up_masks+6}}};
rect_t medium_turn_left_gentle_up_rects[]={
//First angle
{INT32_MIN,-46,0,-18},{INT32_MIN,-18,INT32_MAX,INT32_MAX},
{0,-46,INT32_MAX,-18},
{0,0,0,0},
{0,-91,INT32_MAX,-46},
{INT32_MIN,INT32_MIN,INT32_MAX,-91},{INT32_MIN,-91,0,-46},
//Second angle
{INT32_MIN,INT32_MIN,32,INT32_MAX},
{32,INT32_MIN,64,INT32_MAX},
{64,-55,65,-54},{64,-54,66,-52},{64,-52,67,-51},{64,-51,68,-49},{64,-49,69,-48},{64,-48,70,-46},{64,-46,71,-45},{64,-45,72,-43},{64,-43,73,-42},{64,-42,74,-40},{64,-40,75,-39},{64,-39,76,-37},{64,-37,77,-36},{64,-36,78,-34},{64,-34,79,-33},{64,-33,80,-31},{64,-31,81,-30},{64,-30,82,-28},{64,-28,83,-27},{64,-27,84,-25},{64,-25,85,-24},{64,-24,86,-22},{64,-22,87,-21},{64,-21,88,-19},{64,-19,89,-18},{64,-18,90,-16},{64,-16,91,-15},{64,-15,92,-13},{64,-13,93,-12},{64,-12,94,-10},{64,-10,95,-9},{64,-9,96,INT32_MAX},
{64,-101,65,-100},{64,-100,66,-98},{64,-98,67,-97},{64,-97,68,-96},{64,-96,69,-94},{64,-94,70,-93},{64,-93,71,-92},{64,-92,72,-91},{64,-91,73,-89},{64,-89,74,-88},{64,-88,75,-87},{64,-87,76,-85},{64,-85,77,-84},{64,-84,78,-83},{64,-83,79,-82},{64,-82,80,-80},{64,-80,81,-79},{64,-79,82,-78},{64,-78,83,-76},{64,-76,84,-75},{64,-75,85,-74},{64,-74,86,-73},{64,-73,87,-71},{64,-71,88,-70},{64,-70,89,-69},{64,-69,90,-67},{64,-67,91,-66},{64,-66,92,-65},{64,-65,93,-64},{64,-64,94,-62},{64,-62,95,-61},{64,-61,96,-60},{64,-60,97,-58},{64,-58,98,-57},{64,-57,99,-56},{64,-56,100,-55},{65,-55,101,-54},{66,-54,101,-53},{66,-53,102,-52},{67,-52,103,-51},{68,-51,104,-49},{69,-49,105,-48},{70,-48,106,-47},{70,-47,107,-46},{71,-46,108,-45},{72,-45,108,-44},{72,-44,109,-43},{73,-43,110,-42},{74,-42,111,-40},{75,-40,112,-39},{76,-39,113,-38},{76,-38,114,-37},{77,-37,115,-36},{78,-36,115,-35},{78,-35,116,-34},{79,-34,117,-33},{80,-33,118,-31},{81,-31,119,-30},{82,-30,120,-29},{82,-29,121,-28},{83,-28,122,-27},{84,-27,123,-25},{85,-25,124,-24},{86,-24,125,-23},{86,-23,126,-22},{87,-22,127,-21},{88,-21,127,-20},{88,-20,128,-19},{89,-19,128,-18},{90,-18,128,-16},{91,-16,128,-15},{92,-15,128,-13},{93,-13,128,-12},{94,-12,128,-10},{95,-10,128,-9},{96,-9,128,INT32_MAX},
{64,INT32_MIN,INT32_MAX,-101},{65,-101,INT32_MAX,-100},{66,-100,INT32_MAX,-98},{67,-98,INT32_MAX,-97},{68,-97,INT32_MAX,-96},{69,-96,INT32_MAX,-94},{70,-94,INT32_MAX,-93},{71,-93,INT32_MAX,-92},{72,-92,INT32_MAX,-91},{73,-91,INT32_MAX,-89},{74,-89,INT32_MAX,-88},{75,-88,INT32_MAX,-87},{76,-87,INT32_MAX,-85},{77,-85,INT32_MAX,-84},{78,-84,INT32_MAX,-83},{79,-83,INT32_MAX,-82},{80,-82,INT32_MAX,-80},{81,-80,INT32_MAX,-79},{82,-79,INT32_MAX,-78},{83,-78,INT32_MAX,-76},{84,-76,INT32_MAX,-75},{85,-75,INT32_MAX,-74},{86,-74,INT32_MAX,-73},{87,-73,INT32_MAX,-71},{88,-71,INT32_MAX,-70},{89,-70,INT32_MAX,-69},{90,-69,INT32_MAX,-67},{91,-67,INT32_MAX,-66},{92,-66,INT32_MAX,-65},{93,-65,INT32_MAX,-64},{94,-64,INT32_MAX,-62},{95,-62,INT32_MAX,-61},{96,-61,INT32_MAX,-60},{97,-60,INT32_MAX,-58},{98,-58,INT32_MAX,-57},{99,-57,INT32_MAX,-56},{100,-56,INT32_MAX,-55},{101,-55,INT32_MAX,-53},{102,-53,INT32_MAX,-52},{103,-52,INT32_MAX,-51},{104,-51,INT32_MAX,-49},{105,-49,INT32_MAX,-48},{106,-48,INT32_MAX,-47},{107,-47,INT32_MAX,-46},{108,-46,INT32_MAX,-44},{109,-44,INT32_MAX,-43},{110,-43,INT32_MAX,-42},{111,-42,INT32_MAX,-40},{112,-40,INT32_MAX,-39},{113,-39,INT32_MAX,-38},{114,-38,INT32_MAX,-37},{115,-37,INT32_MAX,-35},{116,-35,INT32_MAX,-34},{117,-34,INT32_MAX,-33},{118,-33,INT32_MAX,-31},{119,-31,INT32_MAX,-30},{120,-30,INT32_MAX,-29},{121,-29,INT32_MAX,-28},{122,-28,INT32_MAX,-27},{123,-27,INT32_MAX,-25},{124,-25,INT32_MAX,-24},{125,-24,INT32_MAX,-23},{126,-23,INT32_MAX,-22},{127,-22,INT32_MAX,-20},{128,-20,INT32_MAX,INT32_MAX},
//Third angle
{INT32_MIN,INT32_MIN,INT32_MAX,11},
{0,0,0,0},
{INT32_MIN,11,0,16},
{0,0,0,0},
{0,11,INT32_MAX,16},{INT32_MIN,16,INT32_MAX,INT32_MAX},
//Fourth angle
{3,INT32_MIN,INT32_MAX,-47},{2,-47,INT32_MAX,-46},{1,-46,INT32_MAX,-44},{0,-44,INT32_MAX,-43},{-1,-43,INT32_MAX,-42},{-2,-42,INT32_MAX,-40},{-3,-40,INT32_MAX,-39},{-4,-39,INT32_MAX,-38},{-5,-38,INT32_MAX,-36},{-6,-36,INT32_MAX,-35},{-7,-35,INT32_MAX,-34},{-8,-34,INT32_MAX,-32},{-9,-32,INT32_MAX,-31},{-10,-31,INT32_MAX,-30},{-11,-30,INT32_MAX,-28},{-12,-28,INT32_MAX,-27},{-13,-27,INT32_MAX,-26},{-14,-26,INT32_MAX,-24},{-15,-24,INT32_MAX,-23},{-16,-23,INT32_MAX,-22},{-17,-22,INT32_MAX,-20},{-18,-20,INT32_MAX,-19},{-19,-19,INT32_MAX,-18},{-20,-18,INT32_MAX,-16},{-21,-16,INT32_MAX,-15},{-22,-15,INT32_MAX,-14},{-23,-14,INT32_MAX,-12},{-24,-12,INT32_MAX,-11},{-25,-11,INT32_MAX,-10},{-26,-10,INT32_MAX,-8},{-27,-8,INT32_MAX,-7},{-28,-7,INT32_MAX,-6},{-29,-6,INT32_MAX,-4},{-30,-4,INT32_MAX,-3},{-31,-3,INT32_MAX,-2},{-32,-2,INT32_MAX,0},{-33,0,INT32_MAX,1},{-34,1,INT32_MAX,2},{-35,2,INT32_MAX,4},{-36,4,INT32_MAX,5},{-37,5,INT32_MAX,6},{-38,6,INT32_MAX,8},{-39,8,INT32_MAX,9},{-40,9,INT32_MAX,10},{-41,10,INT32_MAX,12},{-42,12,INT32_MAX,13},{-43,13,INT32_MAX,14},{-44,14,INT32_MAX,16},{-45,16,INT32_MAX,18},{-46,18,INT32_MAX,19},{-47,19,INT32_MAX,20},{-62,20,INT32_MAX,INT32_MAX},
{-62,INT32_MIN,3,-47},{-62,-47,2,-46},{-62,-46,1,-44},{-62,-44,0,-43},{-62,-43,-1,-42},{-62,-42,-2,-40},{-62,-40,-3,-39},{-62,-39,-4,-38},{-62,-38,-5,-36},{-62,-36,-6,-35},{-62,-35,-7,-34},{-62,-34,-8,-32},{-62,-32,-9,-31},{-62,-31,-10,-30},{-62,-30,-11,-28},{-62,-28,-12,-27},{-62,-27,-13,-26},{-62,-26,-14,-24},{-62,-24,-15,-23},{-62,-23,-16,-22},{-62,-22,-17,-20},{-62,-20,-18,-19},{-62,-19,-19,-18},{-62,-18,-20,-16},{-62,-16,-21,-15},{-62,-15,-22,-14},{-62,-14,-23,-12},{-62,-12,-24,-11},{-62,-11,-25,-10},{-62,-10,-26,-8},{-62,-8,-27,-7},{-62,-7,-28,-6},{-62,-6,-29,-4},{-62,-4,-30,-3},{-62,-3,-31,-2},{-62,-2,-32,0},{-62,0,-33,1},{-62,1,-34,2},{-62,2,-35,4},{-62,4,-36,5},{-62,5,-37,6},{-62,6,-38,8},{-62,8,-39,9},{-62,9,-40,10},{-62,10,-41,12},{-62,12,-42,13},{-62,13,-43,14},{-62,14,-44,16},{-62,16,-45,18},{-62,18,-46,19},{-62,19,-47,20},
{-63,-98,-62,-95},{-64,-95,-62,-92},{-65,-92,-62,-88},{-66,-88,-62,-85},{-67,-85,-62,-82},{-68,-82,-62,-78},{-69,-78,-62,-75},{-70,-75,-62,-72},{-71,-72,-62,-68},{-72,-68,-62,-65},{-73,-65,-62,-62},{-74,-62,-62,-58},{-75,-58,-62,-55},{-76,-55,-62,-52},{-77,-52,-62,-48},{-78,-48,-62,-45},{-79,-45,-62,-42},{-80,-42,-62,-38},{-81,-38,-62,-35},{-82,-35,-62,-32},{-83,-32,-62,-28},{-84,-28,-62,-25},{-85,-25,-62,-22},{-86,-22,-62,-18},{-87,-18,-62,-15},{-88,-15,-62,-12},{-89,-12,-62,-8},{-90,-8,-62,-5},{-91,-5,-62,-2},{-92,-2,-62,2},{-93,2,-62,5},{-94,5,-62,INT32_MAX},
{-126,INT32_MIN,-62,-98},{-126,-98,-63,-95},{-126,-95,-64,-92},{-126,-92,-65,-88},{-126,-88,-66,-85},{-126,-85,-67,-82},{-126,-82,-68,-78},{-126,-78,-69,-75},{-126,-75,-70,-72},{-126,-72,-71,-69},{-125,-69,-71,-68},{-125,-68,-72,-67},{-124,-67,-72,-65},{-123,-65,-73,-64},{-122,-64,-73,-62},{-121,-62,-74,-61},{-120,-61,-74,-59},{-119,-59,-74,-58},{-119,-58,-75,-57},{-118,-57,-75,-56},{-117,-56,-75,-55},{-117,-55,-76,-54},{-116,-54,-76,-53},{-115,-53,-76,-52},{-115,-52,-77,-51},{-114,-51,-77,-50},{-113,-50,-77,-48},{-112,-48,-78,-46},{-111,-46,-78,-45},{-111,-45,-79,-44},{-110,-44,-79,-43},{-109,-43,-79,-42},{-109,-42,-80,-41},{-108,-41,-80,-40},{-107,-40,-80,-38},{-106,-38,-81,-37},{-105,-37,-81,-35},{-104,-35,-82,-33},{-103,-33,-82,-32},{-102,-32,-83,-30},{-101,-30,-83,-29},{-100,-29,-83,-28},{-100,-28,-84,-27},{-99,-27,-84,-26},{-98,-26,-84,-25},{-98,-25,-85,-24},{-97,-24,-85,-22},{-96,-22,-86,-21},{-95,-21,-86,-19},{-94,-19,-86,-18},{-93,-18,-87,-16},{-92,-16,-87,-15},{-91,-15,-88,-13},{-90,-13,-88,-12},{-90,-12,-89,-11},
{INT32_MIN,INT32_MIN,-126,-69},{INT32_MIN,-69,-125,-67},{INT32_MIN,-67,-124,-65},{INT32_MIN,-65,-123,-64},{INT32_MIN,-64,-122,-62},{INT32_MIN,-62,-121,-61},{INT32_MIN,-61,-120,-59},{INT32_MIN,-59,-119,-57},{INT32_MIN,-57,-118,-56},{INT32_MIN,-56,-117,-54},{INT32_MIN,-54,-116,-53},{INT32_MIN,-53,-115,-51},{INT32_MIN,-51,-114,-50},{INT32_MIN,-50,-113,-48},{INT32_MIN,-48,-112,-46},{INT32_MIN,-46,-111,-44},{INT32_MIN,-44,-110,-43},{INT32_MIN,-43,-109,-41},{INT32_MIN,-41,-108,-40},{INT32_MIN,-40,-107,-38},{INT32_MIN,-38,-106,-37},{INT32_MIN,-37,-105,-35},{INT32_MIN,-35,-104,-33},{INT32_MIN,-33,-103,-32},{INT32_MIN,-32,-102,-30},{INT32_MIN,-30,-101,-29},{INT32_MIN,-29,-100,-27},{INT32_MIN,-27,-99,-26},{INT32_MIN,-26,-98,-24},{INT32_MIN,-24,-97,-22},{INT32_MIN,-22,-96,-21},{INT32_MIN,-21,-95,-19},{INT32_MIN,-19,-94,-18},{INT32_MIN,-18,-93,-16},{INT32_MIN,-16,-92,-15},{INT32_MIN,-15,-91,-13},{INT32_MIN,-13,-90,-11},{INT32_MIN,-11,-89,-8},{INT32_MIN,-8,-90,-5},{INT32_MIN,-5,-91,-2},{INT32_MIN,-2,-92,2},{INT32_MIN,2,-93,5},{INT32_MIN,5,-94,INT32_MAX}
};
mask_t medium_turn_left_gentle_up_masks[]={
{0,2,0,0,medium_turn_left_gentle_up_rects},{0,1,-32,32,medium_turn_left_gentle_up_rects+2},{0,1,0,0,medium_turn_left_gentle_up_rects+3},{0,1,-32,80,medium_turn_left_gentle_up_rects+4},{0,2,0,112,medium_turn_left_gentle_up_rects+5},
{0,1,0,0,medium_turn_left_gentle_up_rects+7},{0,1,-32,0,medium_turn_left_gentle_up_rects+8},{0,32,-64,24,medium_turn_left_gentle_up_rects+9},{0,76,-96,16,medium_turn_left_gentle_up_rects+41},{0,65,-128,48,medium_turn_left_gentle_up_rects+117},
{0,1,0,0,medium_turn_left_gentle_up_rects+182},{0,1,0,0,medium_turn_left_gentle_up_rects+183},{0,1,0,-8,medium_turn_left_gentle_up_rects+184},{0,1,0,0,medium_turn_left_gentle_up_rects+185},{0,2,0,-16,medium_turn_left_gentle_up_rects+186},
{0,52,0,0,medium_turn_left_gentle_up_rects+188},{0,51,32,32,medium_turn_left_gentle_up_rects+240},{0,32,64,24,medium_turn_left_gentle_up_rects+291},{0,55,96,48,medium_turn_left_gentle_up_rects+323},{0,43,128,48,medium_turn_left_gentle_up_rects+379}
};
track_section_t medium_turn_left_gentle_up={0,medium_turn_left_gentle_up_curve,NORM(1.25*M_PI*TILE_SIZE,6.0),{{0,5,medium_turn_left_gentle_up_masks},{0,5,medium_turn_left_gentle_up_masks+5},{0,5,medium_turn_left_gentle_up_masks+10},{0,5,medium_turn_left_gentle_up_masks+15}}};
rect_t medium_turn_right_gentle_up_rects[]={
//First angle
{INT32_MIN,INT32_MIN,-3,-46},{INT32_MIN,-46,-2,-44},{INT32_MIN,-44,-1,-43},{INT32_MIN,-43,0,-41},{INT32_MIN,-41,1,-40},{INT32_MIN,-40,2,-38},{INT32_MIN,-38,3,-37},{INT32_MIN,-37,4,-35},{INT32_MIN,-35,5,-34},{INT32_MIN,-34,6,-32},{INT32_MIN,-32,7,-31},{INT32_MIN,-31,8,-29},{INT32_MIN,-29,9,-28},{INT32_MIN,-28,10,-26},{INT32_MIN,-26,11,-25},{INT32_MIN,-25,12,-23},{INT32_MIN,-23,13,-22},{INT32_MIN,-22,14,-20},{INT32_MIN,-20,15,-19},{INT32_MIN,-19,16,-17},{INT32_MIN,-17,17,-16},{INT32_MIN,-16,18,-14},{INT32_MIN,-14,19,-12},{INT32_MIN,-12,20,-11},{INT32_MIN,-11,21,-9},{INT32_MIN,-9,22,-8},{INT32_MIN,-8,23,-6},{INT32_MIN,-6,24,-5},{INT32_MIN,-5,25,-3},{INT32_MIN,-3,26,-2},{INT32_MIN,-2,27,0},{INT32_MIN,0,28,2},{INT32_MIN,2,29,3},{INT32_MIN,3,30,5},{INT32_MIN,5,31,6},{INT32_MIN,6,32,8},{INT32_MIN,8,33,9},{INT32_MIN,9,34,11},{INT32_MIN,11,35,12},{INT32_MIN,12,36,14},{INT32_MIN,14,37,15},{INT32_MIN,15,38,17},{INT32_MIN,17,39,18},{INT32_MIN,18,40,20},{INT32_MIN,20,61,INT32_MAX},
{-3,INT32_MIN,61,-46},{-2,-46,61,-44},{-1,-44,61,-43},{0,-43,61,-41},{1,-41,61,-40},{2,-40,61,-38},{3,-38,61,-37},{4,-37,61,-35},{5,-35,61,-34},{6,-34,61,-32},{7,-32,61,-31},{8,-31,61,-29},{9,-29,61,-28},{10,-28,61,-26},{11,-26,61,-25},{12,-25,61,-23},{13,-23,61,-22},{14,-22,61,-20},{15,-20,61,-19},{16,-19,61,-17},{17,-17,61,-16},{18,-16,61,-14},{19,-14,61,-12},{20,-12,61,-11},{21,-11,61,-9},{22,-9,61,-8},{23,-8,61,-6},{24,-6,61,-5},{25,-5,61,-3},{26,-3,61,-2},{27,-2,61,0},{28,0,61,2},{29,2,61,3},{30,3,61,5},{31,5,61,6},{32,6,61,8},{33,8,61,9},{34,9,61,11},{35,11,61,12},{36,12,61,14},{37,14,61,15},{38,15,61,17},{39,17,61,18},{40,18,61,20},
{61,INT32_MIN,81,INT32_MAX},
{81,INT32_MIN,113,-63},{81,-63,112,-60},{81,-60,111,-57},{81,-57,110,-54},{81,-54,109,-51},{81,-51,108,-48},{81,-48,107,-46},{81,-46,106,-43},{81,-43,105,-40},{81,-40,104,-37},{81,-37,103,-34},{81,-34,102,-32},{81,-32,101,-29},{81,-29,100,-26},{81,-26,99,-23},{81,-23,98,-20},{81,-20,97,-17},{81,-17,96,-14},
{113,INT32_MIN,INT32_MAX,-63},{112,-63,INT32_MAX,-60},{111,-60,INT32_MAX,-57},{110,-57,INT32_MAX,-54},{109,-54,INT32_MAX,-51},{108,-51,INT32_MAX,-48},{107,-48,INT32_MAX,-46},{106,-46,INT32_MAX,-43},{105,-43,INT32_MAX,-40},{104,-40,INT32_MAX,-37},{103,-37,INT32_MAX,-34},{102,-34,INT32_MAX,-32},{101,-32,INT32_MAX,-29},{100,-29,INT32_MAX,-26},{99,-26,INT32_MAX,-23},{98,-23,INT32_MAX,-20},{97,-20,INT32_MAX,-17},{96,-17,INT32_MAX,-14},{81,-14,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,INT32_MAX,11},
{0,0,0,0},
{0,11,INT32_MAX,15},
{0,0,0,0},
{INT32_MIN,11,0,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,INT32_MAX},
{-64,INT32_MIN,-32,INT32_MAX},
{-79,-54,-64,-51},{-80,-51,-64,-48},{-81,-48,-64,-45},{-82,-45,-64,-42},{-83,-42,-64,-39},{-84,-39,-64,-36},{-85,-36,-64,-33},{-86,-33,-64,-30},{-87,-30,-64,-28},{-88,-28,-64,-25},{-89,-25,-64,-22},{-90,-22,-64,-19},{-91,-19,-64,-16},{-92,-16,-64,-13},{-93,-13,-64,-10},{-94,-10,-64,-7},{-95,-7,-64,-4},{-96,-4,-64,-1},{-97,-1,-64,2},{-98,2,-64,5},{-99,5,-64,INT32_MAX},
{-97,INT32_MIN,-64,-72},{-98,-72,-64,-70},{-99,-70,-64,-69},{-100,-69,-64,-67},{-101,-67,-64,-65},{-102,-65,-64,-64},{-103,-64,-64,-62},{-104,-62,-64,-60},{-105,-60,-64,-58},{-106,-58,-64,-57},{-107,-57,-64,-55},{-108,-55,-64,-54},{-108,-54,-79,-53},{-109,-53,-79,-52},{-110,-52,-79,-51},{-110,-51,-80,-50},{-111,-50,-80,-48},{-112,-48,-81,-47},{-113,-47,-81,-45},{-114,-45,-82,-43},{-115,-43,-82,-42},{-115,-42,-83,-41},{-116,-41,-83,-40},{-117,-40,-83,-39},{-117,-39,-84,-38},{-118,-38,-84,-36},{-119,-36,-85,-35},{-120,-35,-85,-33},{-121,-33,-86,-31},{-122,-31,-86,-30},{-123,-30,-87,-28},{-124,-28,-88,-26},{-125,-26,-88,-25},{-126,-25,-89,-23},{-127,-23,-89,-22},{-127,-22,-90,-21},{-128,-21,-90,-20},{-129,-20,-90,-19},{-130,-19,-91,-17},{-131,-17,-91,-16},{-131,-16,-92,-15},{-132,-15,-92,-14},{-133,-14,-92,-13},{-133,-13,-93,-12},{-134,-12,-93,-11},{-135,-11,-93,-10},{-135,-10,-94,-9},{-136,-9,-94,-7},{-136,-7,-95,-4},{-136,-4,-96,-1},{-136,-1,-97,2},{-136,2,-98,5},{-136,5,-99,INT32_MAX},
{INT32_MIN,INT32_MIN,-97,-72},{INT32_MIN,-72,-98,-70},{INT32_MIN,-70,-99,-69},{INT32_MIN,-69,-100,-67},{INT32_MIN,-67,-101,-65},{INT32_MIN,-65,-102,-64},{INT32_MIN,-64,-103,-62},{INT32_MIN,-62,-104,-60},{INT32_MIN,-60,-105,-58},{INT32_MIN,-58,-106,-57},{INT32_MIN,-57,-107,-55},{INT32_MIN,-55,-108,-53},{INT32_MIN,-53,-109,-52},{INT32_MIN,-52,-110,-50},{INT32_MIN,-50,-111,-48},{INT32_MIN,-48,-112,-47},{INT32_MIN,-47,-113,-45},{INT32_MIN,-45,-114,-43},{INT32_MIN,-43,-115,-41},{INT32_MIN,-41,-116,-40},{INT32_MIN,-40,-117,-38},{INT32_MIN,-38,-118,-36},{INT32_MIN,-36,-119,-35},{INT32_MIN,-35,-120,-33},{INT32_MIN,-33,-121,-31},{INT32_MIN,-31,-122,-30},{INT32_MIN,-30,-123,-28},{INT32_MIN,-28,-124,-26},{INT32_MIN,-26,-125,-25},{INT32_MIN,-25,-126,-23},{INT32_MIN,-23,-127,-21},{INT32_MIN,-21,-128,-20},{INT32_MIN,-20,-129,-19},{INT32_MIN,-19,-130,-17},{INT32_MIN,-17,-131,-15},{INT32_MIN,-15,-132,-14},{INT32_MIN,-14,-133,-12},{INT32_MIN,-12,-134,-11},{INT32_MIN,-11,-135,-9},{INT32_MIN,-9,-136,INT32_MAX},
//Fourth angle
{0,-46,INT32_MAX,-16},{INT32_MIN,-16,INT32_MAX,INT32_MAX},
{INT32_MIN,-46,0,-16},
{0,0,0,0},
{INT32_MIN,-90,0,-46},
{INT32_MIN,INT32_MIN,INT32_MAX,-90},{0,-90,INT32_MAX,-46}
};
mask_t medium_turn_right_gentle_up_masks[]={
{0,45,0,0,medium_turn_right_gentle_up_rects},{0,44,-32,32,medium_turn_right_gentle_up_rects+45},{0,1,-64,24,medium_turn_right_gentle_up_rects+89},{0,18,-96,48,medium_turn_right_gentle_up_rects+90},{0,19,-128,48,medium_turn_right_gentle_up_rects+108},
{0,1,0,0,medium_turn_right_gentle_up_rects+127},{0,1,-32,0,medium_turn_right_gentle_up_rects+128},{0,1,0,-8,medium_turn_right_gentle_up_rects+129},{0,1,-32,-8,medium_turn_right_gentle_up_rects+130},{0,2,0,-16,medium_turn_right_gentle_up_rects+131},
{0,1,0,0,medium_turn_right_gentle_up_rects+133},{0,1,32,0,medium_turn_right_gentle_up_rects+134},{0,21,64,24,medium_turn_right_gentle_up_rects+135},{0,53,96,16,medium_turn_right_gentle_up_rects+156},{0,40,128,48,medium_turn_right_gentle_up_rects+209},
{0,2,0,0,medium_turn_right_gentle_up_rects+249},{0,1,32,32,medium_turn_right_gentle_up_rects+251},{0,1,0,56,medium_turn_right_gentle_up_rects+252},{0,1,32,80,medium_turn_right_gentle_up_rects+253},{0,2,0,112,medium_turn_right_gentle_up_rects+254}
};
track_section_t medium_turn_right_gentle_up={0,medium_turn_right_gentle_up_curve,NORM(1.25*M_PI*TILE_SIZE,6.0),{{0,5,medium_turn_right_gentle_up_masks},{0,5,medium_turn_right_gentle_up_masks+5},{0,5,medium_turn_right_gentle_up_masks+10},{0,5,medium_turn_right_gentle_up_masks+15}}};
rect_t none={0,0,0,0};
rect_t very_small_turn_left_steep_up_rects[]={
{INT32_MIN,INT32_MIN,INT32_MAX,-34},{13,INT32_MIN,INT32_MAX,INT32_MAX},
{INT32_MIN,-34,13,INT32_MAX}
};
mask_t very_small_turn_left_steep_up_masks[]={
{0,1,0,0,&all},{0,1,0,0,&none},
{TRACK_MASK_UNION,2,0,0,very_small_turn_left_steep_up_rects},{TRACK_MASK_DIFFERENCE,1,0,0,very_small_turn_left_steep_up_rects+2},
{0,1,0,0,&none},{0,1,0,0,&all},
{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all},
};
track_section_t very_small_turn_left_steep_up={0,very_small_turn_left_steep_up_curve,NORM(0.25*M_PI*TILE_SIZE,6.0),{{0,2,very_small_turn_left_steep_up_masks},{VIEW_NEEDS_TRACK_MASK,2,very_small_turn_left_steep_up_masks+2},{0,2,very_small_turn_left_steep_up_masks+4},{VIEW_NEEDS_TRACK_MASK,2,very_small_turn_left_steep_up_masks+6}}};
rect_t very_small_turn_right_steep_up_rects[]={
{INT32_MIN,INT32_MIN,0,INT32_MAX},{0,INT32_MIN,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,-36},{INT32_MIN,INT32_MIN,-15,INT32_MAX},
{-15,-36,INT32_MAX,INT32_MAX}
};
mask_t very_small_turn_right_steep_up_masks[]={
{TRACK_MASK_UNION,1,0,0,very_small_turn_right_steep_up_rects},{TRACK_MASK_DIFFERENCE,1,0,0,very_small_turn_right_steep_up_rects+1},
{0,1,0,0,&none},{0,1,0,0,&all},
{TRACK_MASK_UNION,2,0,0,very_small_turn_right_steep_up_rects+2},{TRACK_MASK_DIFFERENCE,1,0,0,very_small_turn_right_steep_up_rects+4},
{0,1,0,0,&all},{0,1,0,0,&none},
};
track_section_t very_small_turn_right_steep_up={0,very_small_turn_right_steep_up_curve,NORM(0.25*M_PI*TILE_SIZE,6.0),{{VIEW_NEEDS_TRACK_MASK,2,very_small_turn_right_steep_up_masks},{0,2,very_small_turn_right_steep_up_masks+2},{VIEW_NEEDS_TRACK_MASK,2,very_small_turn_right_steep_up_masks+4},{0,2,very_small_turn_right_steep_up_masks+6}}};
rect_t vertical_twist_left_up_rects[]={
{INT32_MIN,INT32_MIN,INT32_MAX,-44},{INT32_MIN,-44,INT32_MAX,INT32_MAX},
{INT32_MIN,-43,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,-43},
};
mask_t vertical_twist_left_up_masks[]={
{0,1,0,0,vertical_twist_left_up_rects},{0,1,0,0,vertical_twist_left_up_rects+1},
{0,1,0,0,vertical_twist_left_up_rects+2},{0,1,0,0,vertical_twist_left_up_rects+3}
};
track_section_t vertical_twist_left_up={TRACK_VERTICAL,vertical_twist_left_up_curve,9.0,{{0,1,NULL},{0,2,vertical_twist_left_up_masks},{0,1,NULL},{0,2,vertical_twist_left_up_masks+2}}};
rect_t vertical_twist_right_up_rects[]={
{INT32_MIN,-44,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,-44},
{INT32_MIN,INT32_MIN,INT32_MAX,-44},{INT32_MIN,-44,INT32_MAX,INT32_MAX},
};
mask_t vertical_twist_right_up_masks[]={
{0,1,0,0,vertical_twist_right_up_rects},{0,1,0,0,vertical_twist_right_up_rects+1},
{0,1,0,0,vertical_twist_right_up_rects+2},{0,1,0,0,vertical_twist_right_up_rects+3}
};
track_section_t vertical_twist_right_up={TRACK_VERTICAL,vertical_twist_right_up_curve,9.0,{{0,2,vertical_twist_right_up_masks},{0,1,NULL},{0,2,vertical_twist_right_up_masks+2},{0,1,NULL}}};

//Banked sloped turns
mask_t gentle_up_to_gentle_up_left_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t gentle_up_to_gentle_up_left_bank={0,gentle_up_to_gentle_up_left_bank_curve,TILE_SIZE,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_left_bank_masks},{0,1,NULL},{0,1,NULL}}};
mask_t gentle_up_to_gentle_up_right_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t gentle_up_to_gentle_up_right_bank={0,gentle_up_to_gentle_up_right_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_right_bank_masks},{0,1,NULL}}};
mask_t gentle_up_left_bank_to_gentle_up_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t gentle_up_left_bank_to_gentle_up={0,gentle_up_left_bank_to_gentle_up_curve,TILE_SIZE,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_left_bank_to_gentle_up_masks},{0,1,NULL},{0,1,NULL}}};
mask_t gentle_up_right_bank_to_gentle_up_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t gentle_up_right_bank_to_gentle_up={0,gentle_up_right_bank_to_gentle_up_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_right_bank_to_gentle_up_masks},{0,1,NULL}}};

track_section_t left_bank_to_gentle_up_left_bank={0,left_bank_to_gentle_up_left_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t right_bank_to_gentle_up_right_bank={0,right_bank_to_gentle_up_right_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t gentle_up_left_bank_to_left_bank={0,gentle_up_left_bank_to_left_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t gentle_up_right_bank_to_right_bank={0,gentle_up_right_bank_to_right_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};

track_section_t gentle_up_left_bank={0,gentle_up_left_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
track_section_t gentle_up_right_bank={0,gentle_up_right_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};

mask_t flat_to_gentle_up_left_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t flat_to_gentle_up_left_bank={0,flat_to_gentle_up_left_bank_curve,TILE_SIZE,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_left_bank_masks},{0,1,NULL},{0,1,NULL}}};
mask_t flat_to_gentle_up_right_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t flat_to_gentle_up_right_bank={0,flat_to_gentle_up_right_bank_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_right_bank_masks},{0,1,NULL}}};
mask_t gentle_up_left_bank_to_flat_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t gentle_up_left_bank_to_flat={0,gentle_up_left_bank_to_flat_curve,TILE_SIZE,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_left_bank_masks},{0,1,NULL},{0,1,NULL}}};
mask_t gentle_up_right_bank_to_flat_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
track_section_t gentle_up_right_bank_to_flat={0,gentle_up_right_bank_to_flat_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_right_bank_masks},{0,1,NULL}}};
track_section_t small_turn_left_bank_gentle_up={0,small_turn_left_bank_gentle_up_curve,NORM(0.75*M_PI*TILE_SIZE,3.0),{{0,2,small_turn_left_gentle_up_masks},{0,2,small_turn_left_gentle_up_masks+2},{0,2,small_turn_left_gentle_up_masks+4},{0,2,small_turn_left_gentle_up_masks+6}}};
track_section_t small_turn_right_bank_gentle_up={0,small_turn_right_bank_gentle_up_curve,NORM(0.75*M_PI*TILE_SIZE,3.0),{{0,2,small_turn_right_gentle_up_masks},{0,2,small_turn_right_gentle_up_masks+2},{0,2,small_turn_right_gentle_up_masks+4},{0,2,small_turn_right_gentle_up_masks+6}}};
//TODO use correct splitting
track_section_t medium_turn_left_bank_gentle_up={0,medium_turn_left_bank_gentle_up_curve,NORM(1.25*M_PI*TILE_SIZE,6.0),{{0,5,medium_turn_left_gentle_up_masks},{0,5,medium_turn_left_gentle_up_masks+5},{0,5,medium_turn_left_gentle_up_masks+10},{0,5,medium_turn_left_gentle_up_masks+15}}};
track_section_t medium_turn_right_bank_gentle_up={0,medium_turn_right_bank_gentle_up_curve,NORM(1.25*M_PI*TILE_SIZE,6.0),{{0,5,medium_turn_right_gentle_up_masks},{0,5,medium_turn_right_gentle_up_masks+5},{0,5,medium_turn_right_gentle_up_masks+10},{0,5,medium_turn_right_gentle_up_masks+15}}};



rect_t s_bend_left_rects[]={
//First angle
{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{62,-32,INT32_MAX,-31},{60,-31,INT32_MAX,-30},{58,-30,INT32_MAX,-29},{56,-29,INT32_MAX,-28},{54,-28,INT32_MAX,-27},{52,-27,INT32_MAX,-26},{50,-26,INT32_MAX,-25},{48,-25,INT32_MAX,-24},{46,-24,INT32_MAX,-23},{44,-23,INT32_MAX,-22},{42,-22,INT32_MAX,-21},{40,-21,INT32_MAX,-20},{38,-20,INT32_MAX,-19},{36,-19,INT32_MAX,-18},{34,-18,INT32_MAX,-17},{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{2,0,INT32_MAX,1},{4,1,INT32_MAX,2},{6,2,INT32_MAX,3},{8,3,INT32_MAX,4},{10,4,INT32_MAX,5},{12,5,INT32_MAX,6},{14,6,INT32_MAX,7},{16,7,INT32_MAX,8},{18,8,INT32_MAX,9},{20,9,INT32_MAX,10},{22,10,INT32_MAX,11},{24,11,INT32_MAX,12},{26,12,INT32_MAX,13},{28,13,INT32_MAX,14},{30,14,INT32_MAX,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,62,-31},{INT32_MIN,-31,-4,-30},{4,-31,60,-30},{INT32_MIN,-30,-6,-29},{6,-30,58,-29},{INT32_MIN,-29,-8,-28},{8,-29,56,-28},{INT32_MIN,-28,-10,-27},{10,-28,54,-27},{INT32_MIN,-27,-12,-26},{12,-27,52,-26},{INT32_MIN,-26,-14,-25},{14,-26,50,-25},{INT32_MIN,-25,-16,-24},{16,-25,48,-24},{INT32_MIN,-24,-18,-23},{18,-24,46,-23},{INT32_MIN,-23,-20,-22},{20,-23,44,-22},{INT32_MIN,-22,-22,-21},{22,-22,42,-21},{INT32_MIN,-21,-24,-20},{24,-21,40,-20},{INT32_MIN,-20,-26,-19},{26,-20,38,-19},{INT32_MIN,-19,-28,-18},{28,-19,36,-18},{INT32_MIN,-18,-30,-17},{30,-18,34,-17},
//Second angle


{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,INT32_MAX},
{30,16,34,17},{28,17,36,18},{26,18,38,19},{24,19,40,20},{22,20,42,21},{20,21,44,22},{18,22,46,23},{16,23,48,24},{14,24,50,25},{12,25,52,26},{10,26,54,27},{8,27,56,28},{6,28,58,29},{4,29,60,30},{2,30,62,31},{0,31,64,INT32_MAX},
{32,INT32_MIN,96,16},{34,16,94,17},{36,17,92,18},{38,18,90,19},{40,19,88,20},{42,20,86,21},{44,21,84,22},{46,22,82,23},{48,23,80,24},{50,24,78,25},{52,25,76,26},{54,26,74,27},{56,27,72,28},{58,28,70,29},{60,29,68,30},{62,30,66,31},
{96,INT32_MIN,INT32_MAX,16},{94,16,INT32_MAX,17},{92,17,INT32_MAX,18},{90,18,INT32_MAX,19},{88,19,INT32_MAX,20},{86,20,INT32_MAX,21},{84,21,INT32_MAX,22},{82,22,INT32_MAX,23},{80,23,INT32_MAX,24},{78,24,INT32_MAX,25},{76,25,INT32_MAX,26},{74,26,INT32_MAX,27},{72,27,INT32_MAX,28},{70,28,INT32_MAX,29},{68,29,INT32_MAX,30},{66,30,INT32_MAX,31},{64,31,INT32_MAX,INT32_MAX}
};
mask_t s_bend_left_masks[]={
{0,33,0,0,s_bend_left_rects},{0,47,-32,16,s_bend_left_rects+33},{0,31,0,32,s_bend_left_rects+80},{0,31,-32,48,s_bend_left_rects+111},
{0,17,0,0,s_bend_left_rects+142},{0,16,-32,-16,s_bend_left_rects+159},{0,16,-64,0,s_bend_left_rects+175},{0,17,-96,-16,s_bend_left_rects+191}
};
track_section_t s_bend_left={0,s_bend_left_curve,NORM(3.0*TILE_SIZE,1.0*TILE_SIZE),{{0,4,s_bend_left_masks},{0,4,s_bend_left_masks+4},{0,0,NULL},{0,0,NULL}}};
//TODO use correct splitting
rect_t s_bend_right_rects[]={
//First angle
{INT32_MIN,INT32_MIN,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,16},{96,15,INT32_MAX,16},{INT32_MIN,16,34,17},{94,16,INT32_MAX,17},{INT32_MIN,17,36,18},{92,17,INT32_MAX,18},{INT32_MIN,18,38,19},{90,18,INT32_MAX,19},{INT32_MIN,19,40,20},{88,19,INT32_MAX,20},{INT32_MIN,20,42,21},{86,20,INT32_MAX,21},{INT32_MIN,21,44,22},{84,21,INT32_MAX,22},{INT32_MIN,22,46,23},{82,22,INT32_MAX,23},{INT32_MIN,23,48,24},{80,23,INT32_MAX,24},{INT32_MIN,24,50,25},{78,24,INT32_MAX,25},{INT32_MIN,25,52,26},{76,25,INT32_MAX,26},{INT32_MIN,26,54,27},{74,26,INT32_MAX,27},{INT32_MIN,27,56,28},{72,27,INT32_MAX,28},{INT32_MIN,28,58,29},{70,28,INT32_MAX,29},{INT32_MIN,29,60,30},{68,29,INT32_MAX,30},{INT32_MIN,30,62,31},{66,30,INT32_MAX,31},{INT32_MIN,31,INT32_MAX,INT32_MAX},
{0,INT32_MIN,64,0},{2,0,62,1},{4,1,60,2},{6,2,58,3},{8,3,56,4},{10,4,54,5},{12,5,52,6},{14,6,50,7},{16,7,48,8},{18,8,46,9},{20,9,44,10},{22,10,42,11},{24,11,40,12},{26,12,38,13},{28,13,36,14},{30,14,34,15},
{62,0,66,1},{60,1,68,2},{58,2,70,3},{56,3,72,4},{54,4,74,5},{52,5,76,6},{50,6,78,7},{48,7,80,8},{46,8,82,9},{44,9,84,10},{42,10,86,11},{40,11,88,12},{38,12,90,13},{36,13,92,14},{34,14,94,15},{32,15,96,16},{34,16,94,17},{36,17,92,18},{38,18,90,19},{40,19,88,20},{42,20,86,21},{44,21,84,22},{46,22,82,23},{48,23,80,24},{50,24,78,25},{52,25,76,26},{54,26,74,27},{56,27,72,28},{58,28,70,29},{60,29,68,30},{62,30,66,31},
{64,INT32_MIN,INT32_MAX,0},{66,0,INT32_MAX,1},{68,1,INT32_MAX,2},{70,2,INT32_MAX,3},{72,3,INT32_MAX,4},{74,4,INT32_MAX,5},{76,5,INT32_MAX,6},{78,6,INT32_MAX,7},{80,7,INT32_MAX,8},{82,8,INT32_MAX,9},{84,9,INT32_MAX,10},{86,10,INT32_MAX,11},{88,11,INT32_MAX,12},{90,12,INT32_MAX,13},{92,13,INT32_MAX,14},{94,14,INT32_MAX,15},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},
{32,INT32_MIN,INT32_MAX,16},{30,16,INT32_MAX,17},{28,17,INT32_MAX,18},{26,18,INT32_MAX,19},{24,19,INT32_MAX,20},{22,20,INT32_MAX,21},{20,21,INT32_MAX,22},{18,22,INT32_MAX,23},{16,23,INT32_MAX,24},{14,24,INT32_MAX,25},{12,25,INT32_MAX,26},{10,26,INT32_MAX,27},{8,27,INT32_MAX,28},{6,28,INT32_MAX,29},{4,29,INT32_MAX,30},{2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{2,32,62,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},
{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,48},{INT32_MIN,48,30,49},{INT32_MIN,49,28,50},{INT32_MIN,50,26,51},{INT32_MIN,51,24,52},{INT32_MIN,52,22,53},{INT32_MIN,53,20,54},{INT32_MIN,54,18,55},{INT32_MIN,55,16,56},{INT32_MIN,56,14,57},{INT32_MIN,57,12,58},{INT32_MIN,58,10,59},{INT32_MIN,59,8,60},{INT32_MIN,60,6,61},{INT32_MIN,61,4,62},{INT32_MIN,62,2,63},{INT32_MIN,63,0,64},{INT32_MIN,64,-2,INT32_MAX},
{62,32,INT32_MAX,33},{60,33,INT32_MAX,34},{58,34,INT32_MAX,35},{56,35,INT32_MAX,36},{54,36,INT32_MAX,37},{52,37,INT32_MAX,38},{50,38,INT32_MAX,39},{48,39,INT32_MAX,40},{46,40,INT32_MAX,41},{44,41,INT32_MAX,42},{42,42,INT32_MAX,43},{40,43,INT32_MAX,44},{38,44,INT32_MAX,45},{36,45,INT32_MAX,46},{34,46,INT32_MAX,47},{32,47,INT32_MAX,48},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{-2,64,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},
{-34,16,-30,17},{-36,17,-28,18},{-38,18,-26,19},{-40,19,-24,20},{-42,20,-22,21},{-44,21,-20,22},{-46,22,-18,23},{-48,23,-16,24},{-50,24,-14,25},{-52,25,-12,26},{-54,26,-10,27},{-56,27,-8,28},{-58,28,-6,29},{-60,29,-4,30},{-62,30,-2,31},{-64,31,INT32_MAX,32},{-62,32,INT32_MAX,33},{-60,33,INT32_MAX,34},{-58,34,INT32_MAX,35},{-56,35,INT32_MAX,36},{-54,36,INT32_MAX,37},{-52,37,INT32_MAX,38},{-50,38,INT32_MAX,39},{-48,39,INT32_MAX,40},{-46,40,INT32_MAX,41},{-44,41,INT32_MAX,42},{-42,42,INT32_MAX,43},{-40,43,INT32_MAX,44},{-38,44,INT32_MAX,45},{-36,45,INT32_MAX,46},{-34,46,INT32_MAX,47},{-32,47,INT32_MAX,INT32_MAX},
{-96,INT32_MIN,-32,16},{-94,16,-34,17},{-92,17,-36,18},{-90,18,-38,19},{-88,19,-40,20},{-86,20,-42,21},{-84,21,-44,22},{-82,22,-46,23},{-80,23,-48,24},{-78,24,-50,25},{-76,25,-52,26},{-74,26,-54,27},{-72,27,-56,28},{-70,28,-58,29},{-68,29,-60,30},{-66,30,-62,31},{-66,32,-62,33},{-68,33,-60,34},{-70,34,-58,35},{-72,35,-56,36},{-74,36,-54,37},{-76,37,-52,38},{-78,38,-50,39},{-80,39,-48,40},{-82,40,-46,41},{-84,41,-44,42},{-86,42,-42,43},{-88,43,-40,44},{-90,44,-38,45},{-92,45,-36,46},{-94,46,-34,47},{-96,47,-32,INT32_MAX},
{INT32_MIN,INT32_MIN,-96,16},{INT32_MIN,16,-94,17},{INT32_MIN,17,-92,18},{INT32_MIN,18,-90,19},{INT32_MIN,19,-88,20},{INT32_MIN,20,-86,21},{INT32_MIN,21,-84,22},{INT32_MIN,22,-82,23},{INT32_MIN,23,-80,24},{INT32_MIN,24,-78,25},{INT32_MIN,25,-76,26},{INT32_MIN,26,-74,27},{INT32_MIN,27,-72,28},{INT32_MIN,28,-70,29},{INT32_MIN,29,-68,30},{INT32_MIN,30,-66,31},{INT32_MIN,31,-64,32},{INT32_MIN,32,-66,33},{INT32_MIN,33,-68,34},{INT32_MIN,34,-70,35},{INT32_MIN,35,-72,36},{INT32_MIN,36,-74,37},{INT32_MIN,37,-76,38},{INT32_MIN,38,-78,39},{INT32_MIN,39,-80,40},{INT32_MIN,40,-82,41},{INT32_MIN,41,-84,42},{INT32_MIN,42,-86,43},{INT32_MIN,43,-88,44},{INT32_MIN,44,-90,45},{INT32_MIN,45,-92,46},{INT32_MIN,46,-94,47},{INT32_MIN,47,-96,INT32_MAX},
//Fourth angle
{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{INT32_MIN,-26,-50,-25},{INT32_MIN,-25,-48,-24},{INT32_MIN,-24,-46,-23},{INT32_MIN,-23,-44,-22},{INT32_MIN,-22,-42,-21},{INT32_MIN,-21,-40,-20},{INT32_MIN,-20,-38,-19},{INT32_MIN,-19,-36,-18},{INT32_MIN,-18,-34,-17},{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,-2,1},{INT32_MIN,1,-4,2},{INT32_MIN,2,-6,3},{INT32_MIN,3,-8,4},{INT32_MIN,4,-10,5},{INT32_MIN,5,-12,6},{INT32_MIN,6,-14,7},{INT32_MIN,7,-16,8},{INT32_MIN,8,-18,9},{INT32_MIN,9,-20,10},{INT32_MIN,10,-22,11},{INT32_MIN,11,-24,12},{INT32_MIN,12,-26,13},{INT32_MIN,13,-28,14},{INT32_MIN,14,-30,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,INT32_MAX,-31},{INT32_MIN,-31,-4,-30},{4,-31,INT32_MAX,-30},{INT32_MIN,-30,-6,-29},{6,-30,INT32_MAX,-29},{INT32_MIN,-29,-8,-28},{8,-29,INT32_MAX,-28},{INT32_MIN,-28,-10,-27},{10,-28,INT32_MAX,-27},{INT32_MIN,-27,-12,-26},{12,-27,INT32_MAX,-26},{-50,-26,-14,-25},{14,-26,INT32_MAX,-25},{-48,-25,-16,-24},{16,-25,INT32_MAX,-24},{-46,-24,-18,-23},{18,-24,INT32_MAX,-23},{-44,-23,-20,-22},{20,-23,INT32_MAX,-22},{-42,-22,-22,-21},{22,-22,INT32_MAX,-21},{-40,-21,-24,-20},{24,-21,INT32_MAX,-20},{-38,-20,-26,-19},{26,-20,INT32_MAX,-19},{-36,-19,-28,-18},{28,-19,INT32_MAX,-18},{-34,-18,-30,-17},{30,-18,INT32_MAX,-17}
};
mask_t s_bend_right_masks[]={
{0,49,0,0,s_bend_right_rects},{0,16,-32,16,s_bend_right_rects+49},{0,31,-64,0,s_bend_right_rects+65},{0,16,-96,16,s_bend_right_rects+96},
{0,16,0,0,s_bend_right_rects+112},{0,32,-32,-16,s_bend_right_rects+128},{0,34,0,-32,s_bend_right_rects+160},{0,33,-32,-48,s_bend_right_rects+194},
};
track_section_t s_bend_right={0,s_bend_right_curve,NORM(3.0*TILE_SIZE,1.0*TILE_SIZE),{{0,4,s_bend_right_masks},{0,4,s_bend_right_masks+4},{0,0,NULL},{0,0,NULL}}};
mask_t small_helix_left_up_masks[]={
	{TRACK_MASK_INTERSECT,13,0,0,small_turn_rects},{TRACK_MASK_DIFFERENCE,13,0,0,small_turn_rects},{0,13,-32,16,small_turn_rects+13},{0,26,0,32,small_turn_rects+26},
	{0,17,0,0,small_turn_rects+52},{0,16,-32,-16,small_turn_rects+69},{0,17,-64,0,small_turn_rects+85},
	{0,26,0,0,small_turn_rects+102},{0,13,32,-16,small_turn_rects+128},{TRACK_MASK_INTERSECT,13,0,-32,small_turn_rects+141},{TRACK_MASK_DIFFERENCE,13,0,-32,small_turn_rects+141},
	{0,4,0,0,small_turn_rects+154},{0,3,32,16,small_turn_rects+158},{0,4,64,0,small_turn_rects+161}

};
track_section_t small_helix_left_up={0,small_helix_left_up_curve,NORM(0.75,0.75*TILE_SIZE*3.1415926),{{VIEW_NEEDS_TRACK_MASK,4,small_helix_left_up_masks},{0,3,small_helix_left_up_masks+4},{VIEW_NEEDS_TRACK_MASK,4,small_helix_left_up_masks+7},{0,3,small_helix_left_up_masks+11}}};
rect_t small_turn_right_rects[]={
//First angle
{INT32_MIN,INT32_MIN,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,INT32_MAX},
{26,INT32_MIN,38,13},{28,13,36,14},{30,14,34,15},
{38,INT32_MIN,INT32_MAX,13},{36,13,INT32_MAX,14},{34,14,INT32_MAX,15},{32,15,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,INT32_MAX,11},{INT32_MIN,11,18,12},{INT32_MIN,12,16,13},{INT32_MIN,13,14,14},{INT32_MIN,14,12,15},{INT32_MIN,15,10,16},{INT32_MIN,16,8,17},{INT32_MIN,17,6,18},{INT32_MIN,18,4,19},{INT32_MIN,19,2,20},{INT32_MIN,20,0,23},{-16,23,0,24},{-14,24,0,25},{-12,25,0,26},{-10,26,0,27},{-8,27,0,28},{-6,28,0,29},{-4,29,0,30},{-2,30,0,31},
{18,11,INT32_MAX,12},{16,12,INT32_MAX,13},{14,13,INT32_MAX,14},{12,14,INT32_MAX,15},{10,15,INT32_MAX,16},{8,16,INT32_MAX,17},{6,17,INT32_MAX,18},{4,18,INT32_MAX,19},{2,19,INT32_MAX,20},{0,20,INT32_MAX,31},
{INT32_MIN,23,-16,24},{INT32_MIN,24,-14,25},{INT32_MIN,25,-12,26},{INT32_MIN,26,-10,27},{INT32_MIN,27,-8,28},{INT32_MIN,28,-6,29},{INT32_MIN,29,-4,30},{INT32_MIN,30,-2,31},{INT32_MIN,31,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,INT32_MAX},
{-34,16,-30,17},{-36,17,-28,18},{-38,18,-26,19},{-40,19,-24,20},{-42,20,-22,21},{-44,21,-20,22},{-46,22,-18,23},{-48,23,-16,INT32_MAX},
{INT32_MIN,INT32_MIN,-32,16},{INT32_MIN,16,-34,17},{INT32_MIN,17,-36,18},{INT32_MIN,18,-38,19},{INT32_MIN,19,-40,20},{INT32_MIN,20,-42,21},{INT32_MIN,21,-44,22},{INT32_MIN,22,-46,23},{INT32_MIN,23,-48,INT32_MAX},
//Fourth angle
{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{INT32_MIN,-1,INT32_MAX,INT32_MAX},
{INT32_MIN,-22,-18,-21},{INT32_MIN,-21,-16,-20},{INT32_MIN,-20,-14,-19},{INT32_MIN,-19,-12,-18},{INT32_MIN,-18,-10,-17},{INT32_MIN,-17,-8,-16},{INT32_MIN,-16,-6,-15},{INT32_MIN,-15,-4,-14},{INT32_MIN,-14,-2,-13},{INT32_MIN,-13,0,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-22},{-18,-22,INT32_MAX,-21},{-16,-21,INT32_MAX,-20},{-14,-20,INT32_MAX,-19},{-12,-19,INT32_MAX,-18},{-10,-18,INT32_MAX,-17},{-8,-17,INT32_MAX,-16},{-6,-16,INT32_MAX,-15},{-4,-15,INT32_MAX,-14},{-2,-14,INT32_MAX,-13},{0,-13,INT32_MAX,-10},{0,-10,18,-9},{0,-9,16,-8},{0,-8,14,-7},{0,-7,12,-6},{0,-6,10,-5},{0,-5,8,-4},{0,-4,6,-3},{0,-3,4,-2},{0,-2,2,-1}
};
mask_t small_helix_right_up_masks[]={
	{0,4,0,0,small_turn_right_rects},{0,3,-32,16,small_turn_right_rects+4},{0,4,-64,0,small_turn_right_rects+7},
	{0,19,0,0,small_turn_right_rects+11},{0,10,-32,-16,small_turn_right_rects+30},{TRACK_MASK_INTERSECT,9,0,-32,small_turn_right_rects+40},{TRACK_MASK_DIFFERENCE,9,0,-32,small_turn_right_rects+40},
	{0,9,0,0,small_turn_right_rects+49},{0,8,32,-16,small_turn_right_rects+58},{0,9,64,0,small_turn_right_rects+66},
	{TRACK_MASK_INTERSECT,10,0,0,small_turn_right_rects+75},{TRACK_MASK_DIFFERENCE,10,0,0,small_turn_right_rects+75},{0,10,32,16,small_turn_right_rects+85},{0,10,0,32,small_turn_right_rects+95}
};
track_section_t small_helix_right_up={0,small_helix_right_up_curve,NORM(0.75,0.75*TILE_SIZE*3.1415926),{{0,3,small_helix_right_up_masks},{VIEW_NEEDS_TRACK_MASK,4,small_helix_right_up_masks+3},{0,3,small_helix_right_up_masks+7},{VIEW_NEEDS_TRACK_MASK,4,small_helix_right_up_masks+10}}};
mask_t medium_helix_left_up_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_rects},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_rects},{0,32,-32,16,medium_turn_rects+33},{0,31,0,32,medium_turn_rects+65},{0,32,-32,48,medium_turn_rects+96},{0,32,0,64,medium_turn_rects+128},
{0,33,0,0,medium_turn_rects+160},{0,31,-32,-16,medium_turn_rects+193},{0,32,-64,0,medium_turn_rects+224},{0,31,-96,-16,medium_turn_rects+256},{0,33,-128,0,medium_turn_rects+287},
{0,33,0,0,medium_turn_rects+320},{0,32,32,-16,medium_turn_rects+353},{0,31,0,-32,medium_turn_rects+384},{0,32,32,-48,medium_turn_rects+416},{TRACK_MASK_INTERSECT,32,0,-64,medium_turn_rects+448},{TRACK_MASK_DIFFERENCE,32,0,-64,medium_turn_rects+448},

{0,33,0,0,medium_turn_rects+480},{0,31,32,16,medium_turn_rects+513},{0,32,64,0,medium_turn_rects+544},{0,31,96,16,medium_turn_rects+576},{0,33,128,0,medium_turn_rects+607}
};
track_section_t medium_helix_left_up={0,medium_helix_left_up_curve,NORM(0.75,1.25*TILE_SIZE*3.1415926),{{VIEW_NEEDS_TRACK_MASK,6,medium_helix_left_up_masks},{0,5,medium_helix_left_up_masks+6},{VIEW_NEEDS_TRACK_MASK,6,medium_helix_left_up_masks+11},{0,5,medium_helix_left_up_masks+17}}};
rect_t medium_turn_right_rects[]={
//First angle
{INT32_MIN,INT32_MIN,32,-16},{INT32_MIN,-16,30,-15},{INT32_MIN,-15,28,-14},{INT32_MIN,-14,26,-13},{INT32_MIN,-13,24,-12},{INT32_MIN,-12,22,-11},{INT32_MIN,-11,20,-10},{INT32_MIN,-10,18,-9},{INT32_MIN,-9,16,-8},{INT32_MIN,-8,14,-7},{INT32_MIN,-7,12,-6},{INT32_MIN,-6,10,-5},{INT32_MIN,-5,8,-4},{INT32_MIN,-4,6,-3},{INT32_MIN,-3,4,-2},{INT32_MIN,-2,2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,INT32_MAX},
{30,-16,34,-15},{28,-15,36,-14},{26,-14,38,-13},{24,-13,40,-12},{22,-12,42,-11},{20,-11,44,-10},{18,-10,46,-9},{16,-9,48,-8},{14,-8,50,-7},{12,-7,52,-6},{10,-6,54,-5},{8,-5,56,-4},{6,-4,58,-3},{4,-3,60,-2},{2,-2,62,-1},{0,-1,64,0},{2,0,62,1},{4,1,60,2},{6,2,58,3},{8,3,56,4},{10,4,54,5},{12,5,52,6},{14,6,50,7},{16,7,48,8},{18,8,46,9},{20,9,44,10},{22,10,42,11},{24,11,40,12},{26,12,38,13},{28,13,36,14},{30,14,34,15},
{32,INT32_MIN,96,-16},{34,-16,94,-15},{36,-15,92,-14},{38,-14,90,-13},{40,-13,88,-12},{42,-12,86,-11},{44,-11,84,-10},{46,-10,82,-9},{48,-9,80,-8},{50,-8,78,-7},{52,-7,76,-6},{54,-6,74,-5},{56,-5,72,-4},{58,-4,70,-3},{60,-3,68,-2},{62,-2,66,-1},{62,0,66,1},{60,1,68,2},{58,2,70,3},{56,3,72,4},{54,4,74,5},{52,5,76,6},{50,6,78,7},{48,7,80,8},{46,8,82,9},{44,9,84,10},{42,10,86,11},{40,11,88,12},{38,12,90,13},{36,13,92,14},{34,14,94,15},{32,15,96,INT32_MAX},
{94,-16,98,-15},{92,-15,100,-14},{90,-14,102,-13},{88,-13,104,-12},{86,-12,106,-11},{84,-11,108,-10},{82,-10,110,-9},{80,-9,112,-8},{78,-8,114,-7},{76,-7,116,-6},{74,-6,118,-5},{72,-5,120,-4},{70,-4,122,-3},{68,-3,124,-2},{66,-2,126,-1},{64,-1,128,0},{66,0,126,1},{68,1,124,2},{70,2,122,3},{72,3,120,4},{74,4,118,5},{76,5,116,6},{78,6,114,7},{80,7,112,8},{82,8,110,9},{84,9,108,10},{86,10,106,11},{88,11,104,12},{90,12,102,13},{92,13,100,14},{94,14,98,15},
{96,INT32_MIN,INT32_MAX,-16},{98,-16,INT32_MAX,-15},{100,-15,INT32_MAX,-14},{102,-14,INT32_MAX,-13},{104,-13,INT32_MAX,-12},{106,-12,INT32_MAX,-11},{108,-11,INT32_MAX,-10},{110,-10,INT32_MAX,-9},{112,-9,INT32_MAX,-8},{114,-8,INT32_MAX,-7},{116,-7,INT32_MAX,-6},{118,-6,INT32_MAX,-5},{120,-5,INT32_MAX,-4},{122,-4,INT32_MAX,-3},{124,-3,INT32_MAX,-2},{126,-2,INT32_MAX,-1},{128,-1,INT32_MAX,0},{126,0,INT32_MAX,1},{124,1,INT32_MAX,2},{122,2,INT32_MAX,3},{120,3,INT32_MAX,4},{118,4,INT32_MAX,5},{116,5,INT32_MAX,6},{114,6,INT32_MAX,7},{112,7,INT32_MAX,8},{110,8,INT32_MAX,9},{108,9,INT32_MAX,10},{106,10,INT32_MAX,11},{104,11,INT32_MAX,12},{102,12,INT32_MAX,13},{100,13,INT32_MAX,14},{98,14,INT32_MAX,15},{96,15,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,INT32_MAX,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,32},{INT32_MIN,32,-2,33},{INT32_MIN,33,-4,34},{INT32_MIN,34,-6,35},{INT32_MIN,35,-8,36},{INT32_MIN,36,-10,37},{INT32_MIN,37,-12,38},{INT32_MIN,38,-14,39},{INT32_MIN,39,-16,40},{INT32_MIN,40,-18,41},{INT32_MIN,41,-20,42},{INT32_MIN,42,-22,43},{INT32_MIN,43,-24,44},{INT32_MIN,44,-26,45},{INT32_MIN,45,-28,46},{INT32_MIN,46,-30,47},
{30,16,INT32_MAX,17},{28,17,INT32_MAX,18},{26,18,INT32_MAX,19},{24,19,INT32_MAX,20},{22,20,INT32_MAX,21},{20,21,INT32_MAX,22},{18,22,INT32_MAX,23},{16,23,INT32_MAX,24},{14,24,INT32_MAX,25},{12,25,INT32_MAX,26},{10,26,INT32_MAX,27},{8,27,INT32_MAX,28},{6,28,INT32_MAX,29},{4,29,INT32_MAX,30},{2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{2,32,INT32_MAX,33},{4,33,INT32_MAX,34},{6,34,INT32_MAX,35},{8,35,INT32_MAX,36},{10,36,INT32_MAX,37},{12,37,INT32_MAX,38},{14,38,INT32_MAX,39},{16,39,INT32_MAX,40},{18,40,INT32_MAX,41},{20,41,INT32_MAX,42},{22,42,INT32_MAX,43},{24,43,INT32_MAX,44},{26,44,INT32_MAX,45},{28,45,INT32_MAX,46},{30,46,INT32_MAX,47},
{-2,32,2,33},{-4,33,4,34},{-6,34,6,35},{-8,35,8,36},{-10,36,10,37},{-12,37,12,38},{-14,38,14,39},{-16,39,16,40},{-18,40,18,41},{-20,41,20,42},{-22,42,22,43},{-24,43,24,44},{-26,44,26,45},{-28,45,28,46},{-30,46,30,47},{-32,47,32,48},{-30,48,30,49},{-28,49,28,50},{-26,50,26,51},{-24,51,24,52},{-22,52,22,53},{-20,53,20,54},{-18,54,18,55},{-16,55,16,56},{-14,56,14,57},{-12,57,12,58},{-10,58,10,59},{-8,59,8,60},{-6,60,6,61},{-4,61,4,62},{-2,62,2,63},
{32,47,INT32_MAX,48},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{2,64,INT32_MAX,65},{4,65,INT32_MAX,66},{6,66,INT32_MAX,67},{8,67,INT32_MAX,68},{10,68,INT32_MAX,69},{12,69,INT32_MAX,70},{14,70,INT32_MAX,71},{16,71,INT32_MAX,72},{18,72,INT32_MAX,73},{20,73,INT32_MAX,74},{22,74,INT32_MAX,75},{24,75,INT32_MAX,76},{26,76,INT32_MAX,77},{28,77,INT32_MAX,78},{30,78,INT32_MAX,79},
{INT32_MIN,47,-32,48},{INT32_MIN,48,-30,49},{INT32_MIN,49,-28,50},{INT32_MIN,50,-26,51},{INT32_MIN,51,-24,52},{INT32_MIN,52,-22,53},{INT32_MIN,53,-20,54},{INT32_MIN,54,-18,55},{INT32_MIN,55,-16,56},{INT32_MIN,56,-14,57},{INT32_MIN,57,-12,58},{INT32_MIN,58,-10,59},{INT32_MIN,59,-8,60},{INT32_MIN,60,-6,61},{INT32_MIN,61,-4,62},{INT32_MIN,62,-2,63},{INT32_MIN,63,0,64},{INT32_MIN,64,2,65},{INT32_MIN,65,4,66},{INT32_MIN,66,6,67},{INT32_MIN,67,8,68},{INT32_MIN,68,10,69},{INT32_MIN,69,12,70},{INT32_MIN,70,14,71},{INT32_MIN,71,16,72},{INT32_MIN,72,18,73},{INT32_MIN,73,20,74},{INT32_MIN,74,22,75},{INT32_MIN,75,24,76},{INT32_MIN,76,26,77},{INT32_MIN,77,28,78},{INT32_MIN,78,30,79},{INT32_MIN,79,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{-2,32,INT32_MAX,33},{-4,33,INT32_MAX,34},{-6,34,INT32_MAX,35},{-8,35,INT32_MAX,36},{-10,36,INT32_MAX,37},{-12,37,INT32_MAX,38},{-14,38,INT32_MAX,39},{-16,39,INT32_MAX,40},{-18,40,INT32_MAX,41},{-20,41,INT32_MAX,42},{-22,42,INT32_MAX,43},{-24,43,INT32_MAX,44},{-26,44,INT32_MAX,45},{-28,45,INT32_MAX,46},{-30,46,INT32_MAX,47},{-32,47,INT32_MAX,INT32_MAX},
{-34,16,-30,17},{-36,17,-28,18},{-38,18,-26,19},{-40,19,-24,20},{-42,20,-22,21},{-44,21,-20,22},{-46,22,-18,23},{-48,23,-16,24},{-50,24,-14,25},{-52,25,-12,26},{-54,26,-10,27},{-56,27,-8,28},{-58,28,-6,29},{-60,29,-4,30},{-62,30,-2,31},{-64,31,0,32},{-62,32,-2,33},{-60,33,-4,34},{-58,34,-6,35},{-56,35,-8,36},{-54,36,-10,37},{-52,37,-12,38},{-50,38,-14,39},{-48,39,-16,40},{-46,40,-18,41},{-44,41,-20,42},{-42,42,-22,43},{-40,43,-24,44},{-38,44,-26,45},{-36,45,-28,46},{-34,46,-30,47},
{-96,INT32_MIN,-32,16},{-94,16,-34,17},{-92,17,-36,18},{-90,18,-38,19},{-88,19,-40,20},{-86,20,-42,21},{-84,21,-44,22},{-82,22,-46,23},{-80,23,-48,24},{-78,24,-50,25},{-76,25,-52,26},{-74,26,-54,27},{-72,27,-56,28},{-70,28,-58,29},{-68,29,-60,30},{-66,30,-62,31},{-66,32,-62,33},{-68,33,-60,34},{-70,34,-58,35},{-72,35,-56,36},{-74,36,-54,37},{-76,37,-52,38},{-78,38,-50,39},{-80,39,-48,40},{-82,40,-46,41},{-84,41,-44,42},{-86,42,-42,43},{-88,43,-40,44},{-90,44,-38,45},{-92,45,-36,46},{-94,46,-34,47},{-96,47,-32,INT32_MAX},
{-98,16,-94,17},{-100,17,-92,18},{-102,18,-90,19},{-104,19,-88,20},{-106,20,-86,21},{-108,21,-84,22},{-110,22,-82,23},{-112,23,-80,24},{-114,24,-78,25},{-116,25,-76,26},{-118,26,-74,27},{-120,27,-72,28},{-122,28,-70,29},{-124,29,-68,30},{-126,30,-66,31},{-128,31,-64,32},{-126,32,-66,33},{-124,33,-68,34},{-122,34,-70,35},{-120,35,-72,36},{-118,36,-74,37},{-116,37,-76,38},{-114,38,-78,39},{-112,39,-80,40},{-110,40,-82,41},{-108,41,-84,42},{-106,42,-86,43},{-104,43,-88,44},{-102,44,-90,45},{-100,45,-92,46},{-98,46,-94,47},
{INT32_MIN,INT32_MIN,-96,16},{INT32_MIN,16,-98,17},{INT32_MIN,17,-100,18},{INT32_MIN,18,-102,19},{INT32_MIN,19,-104,20},{INT32_MIN,20,-106,21},{INT32_MIN,21,-108,22},{INT32_MIN,22,-110,23},{INT32_MIN,23,-112,24},{INT32_MIN,24,-114,25},{INT32_MIN,25,-116,26},{INT32_MIN,26,-118,27},{INT32_MIN,27,-120,28},{INT32_MIN,28,-122,29},{INT32_MIN,29,-124,30},{INT32_MIN,30,-126,31},{INT32_MIN,31,-128,32},{INT32_MIN,32,-126,33},{INT32_MIN,33,-124,34},{INT32_MIN,34,-122,35},{INT32_MIN,35,-120,36},{INT32_MIN,36,-118,37},{INT32_MIN,37,-116,38},{INT32_MIN,38,-114,39},{INT32_MIN,39,-112,40},{INT32_MIN,40,-110,41},{INT32_MIN,41,-108,42},{INT32_MIN,42,-106,43},{INT32_MIN,43,-104,44},{INT32_MIN,44,-102,45},{INT32_MIN,45,-100,46},{INT32_MIN,46,-98,47},{INT32_MIN,47,-96,INT32_MAX},
//Fourth angle
{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,-2,1},{INT32_MIN,1,-4,2},{INT32_MIN,2,-6,3},{INT32_MIN,3,-8,4},{INT32_MIN,4,-10,5},{INT32_MIN,5,-12,6},{INT32_MIN,6,-14,7},{INT32_MIN,7,-16,8},{INT32_MIN,8,-18,9},{INT32_MIN,9,-20,10},{INT32_MIN,10,-22,11},{INT32_MIN,11,-24,12},{INT32_MIN,12,-26,13},{INT32_MIN,13,-28,14},{INT32_MIN,14,-30,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,-48,-30,-47},{INT32_MIN,-47,-28,-46},{INT32_MIN,-46,-26,-45},{INT32_MIN,-45,-24,-44},{INT32_MIN,-44,-22,-43},{INT32_MIN,-43,-20,-42},{INT32_MIN,-42,-18,-41},{INT32_MIN,-41,-16,-40},{INT32_MIN,-40,-14,-39},{INT32_MIN,-39,-12,-38},{INT32_MIN,-38,-10,-37},{INT32_MIN,-37,-8,-36},{INT32_MIN,-36,-6,-35},{INT32_MIN,-35,-4,-34},{INT32_MIN,-34,-2,-33},{INT32_MIN,-33,0,-32},{INT32_MIN,-32,-2,-31},{INT32_MIN,-31,-4,-30},{INT32_MIN,-30,-6,-29},{INT32_MIN,-29,-8,-28},{INT32_MIN,-28,-10,-27},{INT32_MIN,-27,-12,-26},{INT32_MIN,-26,-14,-25},{INT32_MIN,-25,-16,-24},{INT32_MIN,-24,-18,-23},{INT32_MIN,-23,-20,-22},{INT32_MIN,-22,-22,-21},{INT32_MIN,-21,-24,-20},{INT32_MIN,-20,-26,-19},{INT32_MIN,-19,-28,-18},{INT32_MIN,-18,-30,-17},{INT32_MIN,-17,-32,-16},
{INT32_MIN,INT32_MIN,INT32_MAX,-48},{-30,-48,INT32_MAX,-47},{-28,-47,INT32_MAX,-46},{-26,-46,INT32_MAX,-45},{-24,-45,INT32_MAX,-44},{-22,-44,INT32_MAX,-43},{-20,-43,INT32_MAX,-42},{-18,-42,INT32_MAX,-41},{-16,-41,INT32_MAX,-40},{-14,-40,INT32_MAX,-39},{-12,-39,INT32_MAX,-38},{-10,-38,INT32_MAX,-37},{-8,-37,INT32_MAX,-36},{-6,-36,INT32_MAX,-35},{-4,-35,INT32_MAX,-34},{-2,-34,INT32_MAX,-33},{0,-33,INT32_MAX,-32},{2,-32,INT32_MAX,-31},{4,-31,INT32_MAX,-30},{6,-30,INT32_MAX,-29},{8,-29,INT32_MAX,-28},{10,-28,INT32_MAX,-27},{12,-27,INT32_MAX,-26},{14,-26,INT32_MAX,-25},{16,-25,INT32_MAX,-24},{18,-24,INT32_MAX,-23},{20,-23,INT32_MAX,-22},{22,-22,INT32_MAX,-21},{24,-21,INT32_MAX,-20},{26,-20,INT32_MAX,-19},{28,-19,INT32_MAX,-18},{30,-18,INT32_MAX,-17},{32,-17,INT32_MAX,-16}
};
mask_t medium_helix_right_up_masks[]={
{0,33,0,0,medium_turn_right_rects},{0,31,-32,16,medium_turn_right_rects+33},{0,32,-64,0,medium_turn_right_rects+64},{0,31,-96,16,medium_turn_right_rects+96},{0,33,-128,0,medium_turn_right_rects+127},
{0,32,0,0,medium_turn_right_rects+160},{0,31,-32,-16,medium_turn_right_rects+192},{0,31,0,-32,medium_turn_right_rects+223},{0,32,-32,-48,medium_turn_right_rects+254},{TRACK_MASK_INTERSECT,33,0,-64,medium_turn_right_rects+286},{TRACK_MASK_DIFFERENCE,33,0,-64,medium_turn_right_rects+286},
{0,33,0,0,medium_turn_right_rects+319},{0,31,32,-16,medium_turn_right_rects+352},{0,32,64,0,medium_turn_right_rects+383},{0,31,96,-16,medium_turn_right_rects+415},{0,33,128,0,medium_turn_right_rects+446},
{TRACK_MASK_INTERSECT,32,0,0,medium_turn_right_rects+479},{TRACK_MASK_DIFFERENCE,32,0,0,medium_turn_right_rects+479},{0,31,32,16,medium_turn_right_rects+511},{0,31,0,32,medium_turn_right_rects+542},{0,32,32,48,medium_turn_right_rects+573},{0,33,0,64,medium_turn_right_rects+605}
};
track_section_t medium_helix_right_up={0,medium_helix_right_up_curve,NORM(0.75,1.25*TILE_SIZE*3.1415926),{{0,5,medium_helix_right_up_masks},{VIEW_NEEDS_TRACK_MASK,6,medium_helix_right_up_masks+5},{0,5,medium_helix_right_up_masks+11},{VIEW_NEEDS_TRACK_MASK,6,medium_helix_right_up_masks+16}}};




rect_t barrel_roll_left_rects[]={
//First angle
{INT32_MIN,INT32_MIN,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,16},{INT32_MIN,16,34,17},{INT32_MIN,17,36,18},{INT32_MIN,18,38,19},{INT32_MIN,19,40,20},{INT32_MIN,20,42,21},{INT32_MIN,21,44,22},{INT32_MIN,22,46,23},{INT32_MIN,23,INT32_MAX,INT32_MAX},
{-14,-28,61,-7},{-12,-7,61,-6},{-10,-6,61,-5},{-8,-5,61,-4},{-6,-4,61,-3},{-4,-3,61,-2},{-2,-2,61,-1},{0,-1,61,0},{2,0,61,1},{4,1,61,2},{6,2,61,3},{8,3,61,4},{10,4,61,5},{12,5,61,6},{14,6,61,7},{16,7,61,8},{18,8,61,9},{20,9,61,10},{22,10,61,11},{24,11,61,12},{26,12,61,13},{28,13,61,14},{30,14,61,15},{32,15,61,16},{34,16,61,17},{36,17,61,18},{38,18,61,19},{40,19,61,20},{42,20,61,21},{44,21,61,22},{46,22,61,23},
{-14,INT32_MIN,INT32_MAX,-28},{61,-28,INT32_MAX,23},
//Second angle
{INT32_MIN,INT32_MIN,15,-4},{INT32_MIN,-4,14,3},{INT32_MIN,3,13,10},{INT32_MIN,10,12,17},{INT32_MIN,17,11,24},{INT32_MIN,24,10,31},{INT32_MIN,31,9,38},{INT32_MIN,38,8,INT32_MAX},
{15,INT32_MIN,32,-4},{14,-4,32,3},{13,3,32,10},{12,10,32,17},{11,17,32,24},{10,24,32,31},{9,31,32,38},{8,38,32,INT32_MAX},
{32,INT32_MIN,INT32_MAX,INT32_MAX},
//Third angle
{-34,INT32_MIN,INT32_MAX,-15},{-33,-15,INT32_MAX,-11},{-32,-11,INT32_MAX,-8},{-31,-8,INT32_MAX,-4},{-30,-4,INT32_MAX,-1},{-29,-1,INT32_MAX,3},{-28,3,INT32_MAX,6},{-27,6,INT32_MAX,10},{-26,10,INT32_MAX,13},{-25,13,INT32_MAX,17},{-24,17,INT32_MAX,20},{-23,20,INT32_MAX,24},{-22,24,INT32_MAX,27},{-21,27,INT32_MAX,31},{-20,31,INT32_MAX,34},{-19,34,INT32_MAX,38},{-18,38,INT32_MAX,41},{-17,41,INT32_MAX,INT32_MAX},
{-65,INT32_MIN,-34,-15},{-65,-15,-33,-14},{-64,-14,-33,-11},{-64,-11,-32,-9},{-63,-9,-32,-8},{-63,-8,-31,-5},{-62,-5,-31,-4},{-62,-4,-30,-1},{-62,-1,-29,0},{-61,0,-29,3},{-61,3,-28,4},{-60,4,-28,6},{-60,6,-27,9},{-59,9,-27,10},{-59,10,-26,13},{-58,13,-25,17},{-57,17,-24,20},{-57,20,-23,22},{-56,22,-23,24},{-56,24,-22,26},{-55,26,-22,27},{-55,27,-21,31},{-54,31,-20,34},{-54,34,-19,35},{-53,35,-19,38},{-53,38,-18,40},{-52,40,-18,41},{-52,41,-17,INT32_MAX},
{INT32_MIN,INT32_MIN,-65,-14},{INT32_MIN,-14,-64,-9},{INT32_MIN,-9,-63,-5},{INT32_MIN,-5,-62,0},{INT32_MIN,0,-61,4},{INT32_MIN,4,-60,9},{INT32_MIN,9,-59,13},{INT32_MIN,13,-58,17},{INT32_MIN,17,-57,22},{INT32_MIN,22,-56,26},{INT32_MIN,26,-55,31},{INT32_MIN,31,-54,35},{INT32_MIN,35,-53,40},{INT32_MIN,40,-52,INT32_MAX},
//Fourth angle
{7,-30,INT32_MAX,-28},{6,-28,INT32_MAX,-27},{5,-27,INT32_MAX,-26},{4,-26,INT32_MAX,-25},{3,-25,INT32_MAX,-24},{2,-24,INT32_MAX,-22},{1,-22,INT32_MAX,-21},{0,-21,INT32_MAX,-20},{-1,-20,INT32_MAX,-19},{-2,-19,INT32_MAX,-18},{-3,-18,INT32_MAX,-17},{-4,-17,INT32_MAX,-15},{-5,-15,INT32_MAX,-14},{-6,-14,INT32_MAX,-13},{-7,-13,INT32_MAX,-12},{-8,-12,INT32_MAX,-11},{-9,-11,INT32_MAX,-9},{-10,-9,INT32_MAX,-8},{-11,-8,INT32_MAX,-7},{-12,-7,INT32_MAX,-6},{-13,-6,INT32_MAX,-5},{-14,-5,INT32_MAX,-4},{-15,-4,INT32_MAX,-2},{-16,-2,INT32_MAX,-1},{-17,-1,INT32_MAX,0},{-18,0,INT32_MAX,1},{-19,1,INT32_MAX,2},{-20,2,INT32_MAX,4},{-21,4,INT32_MAX,5},{-22,5,INT32_MAX,6},{-23,6,INT32_MAX,7},{-24,7,INT32_MAX,8},{-25,8,INT32_MAX,9},{-26,9,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,INT32_MAX,-57},{-33,-57,INT32_MAX,-55},{-34,-55,INT32_MAX,-53},{-35,-53,INT32_MAX,-51},{-36,-51,INT32_MAX,-49},{-37,-49,INT32_MAX,-47},{-38,-47,INT32_MAX,-45},{-39,-45,INT32_MAX,-43},{-40,-43,INT32_MAX,-41},{-41,-41,INT32_MAX,-39},{-42,-39,INT32_MAX,-37},{-43,-37,INT32_MAX,-34},{-44,-34,INT32_MAX,-32},{-45,-32,INT32_MAX,-30},{-46,-30,7,-28},{-47,-28,6,-27},{-47,-27,5,-26},{-48,-26,4,-25},{-48,-25,3,-24},{-49,-24,2,-22},{-50,-22,1,-21},{-50,-21,0,-20},{-51,-20,-1,-19},{-51,-19,-2,-18},{INT32_MIN,-18,-3,-17},{INT32_MIN,-17,-4,-15},{INT32_MIN,-15,-5,-14},{INT32_MIN,-14,-6,-13},{INT32_MIN,-13,-7,-12},{INT32_MIN,-12,-8,-11},{INT32_MIN,-11,-9,-9},{INT32_MIN,-9,-10,-8},{INT32_MIN,-8,-11,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-13,-5},{INT32_MIN,-5,-14,-4},{INT32_MIN,-4,-15,-2},{INT32_MIN,-2,-16,-1},{INT32_MIN,-1,-17,0},{INT32_MIN,0,-18,1},{INT32_MIN,1,-19,2},{INT32_MIN,2,-20,4},{INT32_MIN,4,-21,5},{INT32_MIN,5,-22,6},{INT32_MIN,6,-23,7},{INT32_MIN,7,-24,8},{INT32_MIN,8,-25,9},{INT32_MIN,9,-26,INT32_MAX},
{INT32_MIN,INT32_MIN,-32,-57},{INT32_MIN,-57,-33,-55},{INT32_MIN,-55,-34,-53},{INT32_MIN,-53,-35,-51},{INT32_MIN,-51,-36,-49},{INT32_MIN,-49,-37,-47},{INT32_MIN,-47,-38,-45},{INT32_MIN,-45,-39,-43},{INT32_MIN,-43,-40,-41},{INT32_MIN,-41,-41,-39},{INT32_MIN,-39,-42,-37},{INT32_MIN,-37,-43,-34},{INT32_MIN,-34,-44,-32},{INT32_MIN,-32,-45,-30},{INT32_MIN,-30,-46,-28},{INT32_MIN,-28,-47,-26},{INT32_MIN,-26,-48,-24},{INT32_MIN,-24,-49,-22},{INT32_MIN,-22,-50,-20},{INT32_MIN,-20,-51,-18},
};
mask_t barrel_roll_left_masks[]={
	{TRACK_MASK_INTERSECT,32,0,0,barrel_roll_left_rects},{TRACK_MASK_DIFFERENCE,32,0,0,barrel_roll_left_rects},{TRACK_MASK_INTERSECT,31,-32,16,barrel_roll_left_rects+32},{TRACK_MASK_DIFFERENCE,31,-32,16,barrel_roll_left_rects+32},{TRACK_MASK_INTERSECT,2,-64,32,barrel_roll_left_rects+63},{TRACK_MASK_DIFFERENCE,2,-64,32,barrel_roll_left_rects+63},
	{TRACK_MASK_INTERSECT,8,0,0,barrel_roll_left_rects+65},{TRACK_MASK_DIFFERENCE,8,0,0,barrel_roll_left_rects+65},{TRACK_MASK_INTERSECT,8,-32,-16,barrel_roll_left_rects+73},{TRACK_MASK_DIFFERENCE,8,-32,-16,barrel_roll_left_rects+73},{TRACK_MASK_INTERSECT,1,-64,-32,barrel_roll_left_rects+81},{TRACK_MASK_DIFFERENCE,1,-64,-32,barrel_roll_left_rects+81},
	{TRACK_MASK_INTERSECT,18,0,0,barrel_roll_left_rects+82},{TRACK_MASK_DIFFERENCE,18,0,0,barrel_roll_left_rects+82},{TRACK_MASK_INTERSECT,28,32,-16,barrel_roll_left_rects+100},{TRACK_MASK_DIFFERENCE,28,32,-16,barrel_roll_left_rects+100},{TRACK_MASK_INTERSECT,14,64,-32,barrel_roll_left_rects+128},{TRACK_MASK_DIFFERENCE,14,64,-32,barrel_roll_left_rects+128},
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_left_rects+142},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_left_rects+142},{TRACK_MASK_INTERSECT,48,32,16,barrel_roll_left_rects+176},{TRACK_MASK_DIFFERENCE,48,32,16,barrel_roll_left_rects+176},{TRACK_MASK_INTERSECT,20,64,32,barrel_roll_left_rects+224},{TRACK_MASK_DIFFERENCE,20,64,32,barrel_roll_left_rects+224}
};
track_section_t barrel_roll_left={0,barrel_roll_left_curve,3.0*TILE_SIZE,{{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_left_masks},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_left_masks+6},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_left_masks+12},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_left_masks+18}}};


rect_t barrel_roll_right_rects[]={
//First angle
{INT32_MIN,-30,-7,-28},{INT32_MIN,-28,-6,-27},{INT32_MIN,-27,-5,-26},{INT32_MIN,-26,-4,-25},{INT32_MIN,-25,-3,-24},{INT32_MIN,-24,-2,-22},{INT32_MIN,-22,-1,-21},{INT32_MIN,-21,0,-20},{INT32_MIN,-20,1,-19},{INT32_MIN,-19,2,-18},{INT32_MIN,-18,3,-17},{INT32_MIN,-17,4,-15},{INT32_MIN,-15,5,-14},{INT32_MIN,-14,6,-13},{INT32_MIN,-13,7,-12},{INT32_MIN,-12,8,-11},{INT32_MIN,-11,9,-9},{INT32_MIN,-9,10,-8},{INT32_MIN,-8,11,-7},{INT32_MIN,-7,12,-6},{INT32_MIN,-6,13,-5},{INT32_MIN,-5,14,-4},{INT32_MIN,-4,15,-2},{INT32_MIN,-2,16,-1},{INT32_MIN,-1,17,0},{INT32_MIN,0,18,1},{INT32_MIN,1,19,2},{INT32_MIN,2,20,4},{INT32_MIN,4,21,5},{INT32_MIN,5,22,6},{INT32_MIN,6,23,7},{INT32_MIN,7,24,8},{INT32_MIN,8,25,9},{INT32_MIN,9,26,INT32_MAX},
{INT32_MIN,INT32_MIN,32,-57},{INT32_MIN,-57,33,-55},{INT32_MIN,-55,34,-53},{INT32_MIN,-53,35,-51},{INT32_MIN,-51,36,-49},{INT32_MIN,-49,37,-47},{INT32_MIN,-47,38,-45},{INT32_MIN,-45,39,-43},{INT32_MIN,-43,40,-41},{INT32_MIN,-41,41,-39},{INT32_MIN,-39,42,-37},{INT32_MIN,-37,43,-34},{INT32_MIN,-34,44,-32},{INT32_MIN,-32,45,-30},{-7,-30,46,-28},{-6,-28,47,-27},{-5,-27,47,-26},{-4,-26,48,-25},{-3,-25,48,-24},{-2,-24,49,-22},{-1,-22,50,-21},{0,-21,50,-20},{1,-20,51,-19},{2,-19,51,-18},{3,-18,INT32_MAX,-17},{4,-17,INT32_MAX,-15},{5,-15,INT32_MAX,-14},{6,-14,INT32_MAX,-13},{7,-13,INT32_MAX,-12},{8,-12,INT32_MAX,-11},{9,-11,INT32_MAX,-9},{10,-9,INT32_MAX,-8},{11,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{13,-6,INT32_MAX,-5},{14,-5,INT32_MAX,-4},{15,-4,INT32_MAX,-2},{16,-2,INT32_MAX,-1},{17,-1,INT32_MAX,0},{18,0,INT32_MAX,1},{19,1,INT32_MAX,2},{20,2,INT32_MAX,4},{21,4,INT32_MAX,5},{22,5,INT32_MAX,6},{23,6,INT32_MAX,7},{24,7,INT32_MAX,8},{25,8,INT32_MAX,9},{26,9,INT32_MAX,INT32_MAX},
{32,INT32_MIN,INT32_MAX,-57},{33,-57,INT32_MAX,-55},{34,-55,INT32_MAX,-53},{35,-53,INT32_MAX,-51},{36,-51,INT32_MAX,-49},{37,-49,INT32_MAX,-47},{38,-47,INT32_MAX,-45},{39,-45,INT32_MAX,-43},{40,-43,INT32_MAX,-41},{41,-41,INT32_MAX,-39},{42,-39,INT32_MAX,-37},{43,-37,INT32_MAX,-34},{44,-34,INT32_MAX,-32},{45,-32,INT32_MAX,-30},{46,-30,INT32_MAX,-28},{47,-28,INT32_MAX,-26},{48,-26,INT32_MAX,-24},{49,-24,INT32_MAX,-22},{50,-22,INT32_MAX,-20},{51,-20,INT32_MAX,-18},
//Second angle
{INT32_MIN,INT32_MIN,34,-15},{INT32_MIN,-15,33,-11},{INT32_MIN,-11,32,-8},{INT32_MIN,-8,31,-4},{INT32_MIN,-4,30,-1},{INT32_MIN,-1,29,3},{INT32_MIN,3,28,6},{INT32_MIN,6,27,10},{INT32_MIN,10,26,13},{INT32_MIN,13,25,17},{INT32_MIN,17,24,20},{INT32_MIN,20,23,24},{INT32_MIN,24,22,27},{INT32_MIN,27,21,31},{INT32_MIN,31,20,34},{INT32_MIN,34,19,38},{INT32_MIN,38,18,41},{INT32_MIN,41,17,INT32_MAX},
{34,INT32_MIN,65,-15},{33,-15,65,-14},{33,-14,64,-11},{32,-11,64,-9},{32,-9,63,-8},{31,-8,63,-5},{31,-5,62,-4},{30,-4,62,-1},{29,-1,62,0},{29,0,61,3},{28,3,61,4},{28,4,60,6},{27,6,60,9},{27,9,59,10},{26,10,59,13},{25,13,58,17},{24,17,57,20},{23,20,57,22},{23,22,56,24},{22,24,56,26},{22,26,55,27},{21,27,55,31},{20,31,54,34},{19,34,54,35},{19,35,53,38},{18,38,53,40},{18,40,52,41},{17,41,52,INT32_MAX},
{65,INT32_MIN,INT32_MAX,-14},{64,-14,INT32_MAX,-9},{63,-9,INT32_MAX,-5},{62,-5,INT32_MAX,0},{61,0,INT32_MAX,4},{60,4,INT32_MAX,9},{59,9,INT32_MAX,13},{58,13,INT32_MAX,17},{57,17,INT32_MAX,22},{56,22,INT32_MAX,26},{55,26,INT32_MAX,31},{54,31,INT32_MAX,35},{53,35,INT32_MAX,40},{52,40,INT32_MAX,INT32_MAX},
//Third angle
{-15,INT32_MIN,INT32_MAX,-4},{-14,-4,INT32_MAX,3},{-13,3,INT32_MAX,10},{-12,10,INT32_MAX,17},{-11,17,INT32_MAX,24},{-10,24,INT32_MAX,31},{-9,31,INT32_MAX,38},{-8,38,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,-15,-4},{-32,-4,-14,3},{-32,3,-13,10},{-32,10,-12,17},{-32,17,-11,24},{-32,24,-10,31},{-32,31,-9,38},{-32,38,-8,INT32_MAX},
{INT32_MIN,INT32_MIN,-32,INT32_MAX},
//Fourth angle
{14,INT32_MIN,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{-32,15,INT32_MAX,16},{-34,16,INT32_MAX,17},{-36,17,INT32_MAX,18},{-38,18,INT32_MAX,19},{-40,19,INT32_MAX,20},{-42,20,INT32_MAX,21},{-44,21,INT32_MAX,22},{-46,22,INT32_MAX,23},{INT32_MIN,23,INT32_MAX,INT32_MAX},
{-61,-28,14,-7},{-61,-7,12,-6},{-61,-6,10,-5},{-61,-5,8,-4},{-61,-4,6,-3},{-61,-3,4,-2},{-61,-2,2,-1},{-61,-1,0,0},{-61,0,-2,1},{-61,1,-4,2},{-61,2,-6,3},{-61,3,-8,4},{-61,4,-10,5},{-61,5,-12,6},{-61,6,-14,7},{-61,7,-16,8},{-61,8,-18,9},{-61,9,-20,10},{-61,10,-22,11},{-61,11,-24,12},{-61,12,-26,13},{-61,13,-28,14},{-61,14,-30,15},{-61,15,-32,16},{-61,16,-34,17},{-61,17,-36,18},{-61,18,-38,19},{-61,19,-40,20},{-61,20,-42,21},{-61,21,-44,22},{-61,22,-46,23},
{INT32_MIN,INT32_MIN,14,-28},{INT32_MIN,-28,-61,23},
};
mask_t barrel_roll_right_masks[]={
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_right_rects},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_right_rects},{TRACK_MASK_INTERSECT,48,-32,16,barrel_roll_right_rects+34},{TRACK_MASK_DIFFERENCE,48,-32,16,barrel_roll_right_rects+34},{TRACK_MASK_INTERSECT,20,-64,32,barrel_roll_right_rects+82},{TRACK_MASK_DIFFERENCE,20,-64,32,barrel_roll_right_rects+82},
	{TRACK_MASK_INTERSECT,18,0,0,barrel_roll_right_rects+102},{TRACK_MASK_DIFFERENCE,18,0,0,barrel_roll_right_rects+102},{TRACK_MASK_INTERSECT,28,-32,-16,barrel_roll_right_rects+120},{TRACK_MASK_DIFFERENCE,28,-32,-16,barrel_roll_right_rects+120},{TRACK_MASK_INTERSECT,14,-64,-32,barrel_roll_right_rects+148},{TRACK_MASK_DIFFERENCE,14,-64,-32,barrel_roll_right_rects+148},
	{TRACK_MASK_INTERSECT,8,0,0,barrel_roll_right_rects+162},{TRACK_MASK_DIFFERENCE,8,0,0,barrel_roll_right_rects+162},{TRACK_MASK_INTERSECT,8,32,-16,barrel_roll_right_rects+170},{TRACK_MASK_DIFFERENCE,8,32,-16,barrel_roll_right_rects+170},{TRACK_MASK_INTERSECT,1,64,-32,barrel_roll_right_rects+178},{TRACK_MASK_DIFFERENCE,1,64,-32,barrel_roll_right_rects+178},
	{TRACK_MASK_INTERSECT,32,0,0,barrel_roll_right_rects+179},{TRACK_MASK_DIFFERENCE,32,0,0,barrel_roll_right_rects+179},{TRACK_MASK_INTERSECT,31,32,16,barrel_roll_right_rects+211},{TRACK_MASK_DIFFERENCE,31,32,16,barrel_roll_right_rects+211},{TRACK_MASK_INTERSECT,2,64,32,barrel_roll_right_rects+242},{TRACK_MASK_DIFFERENCE,2,64,32,barrel_roll_right_rects+242}
};
track_section_t barrel_roll_right={0,barrel_roll_right_curve,3.0*TILE_SIZE,{{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_right_masks},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_right_masks+6},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_right_masks+12},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_right_masks+18}}};

rect_t half_loop_rects[]={
//First angle
{INT32_MIN,-29,-1,-28},{INT32_MIN,-28,0,-27},{INT32_MIN,-27,1,-26},{INT32_MIN,-26,2,-25},{INT32_MIN,-25,3,-24},{INT32_MIN,-24,4,-23},{INT32_MIN,-23,5,-22},{INT32_MIN,-22,6,-21},{INT32_MIN,-21,7,-20},{INT32_MIN,-20,8,-19},{INT32_MIN,-19,9,-18},{INT32_MIN,-18,10,-17},{INT32_MIN,-17,11,-16},{INT32_MIN,-16,12,-15},{INT32_MIN,-15,13,-14},{INT32_MIN,-14,14,-13},{INT32_MIN,-13,15,-12},{INT32_MIN,-12,16,-11},{INT32_MIN,-11,17,-10},{INT32_MIN,-10,18,-9},{INT32_MIN,-9,19,-8},{INT32_MIN,-8,20,-7},{INT32_MIN,-7,21,-6},{INT32_MIN,-6,22,INT32_MAX},
{INT32_MIN,-77,28,-76},{INT32_MIN,-76,30,-75},{INT32_MIN,-75,33,-74},{INT32_MIN,-74,35,-73},{INT32_MIN,-73,37,-72},{INT32_MIN,-72,40,-71},{INT32_MIN,-71,42,-70},{INT32_MIN,-70,45,-69},{INT32_MIN,-69,47,-68},{INT32_MIN,-68,50,-67},{INT32_MIN,-67,52,-66},{INT32_MIN,-66,55,-65},{INT32_MIN,-65,57,-64},{INT32_MIN,-64,INT32_MAX,-29},{-1,-29,INT32_MAX,-28},{0,-28,INT32_MAX,-27},{1,-27,INT32_MAX,-26},{2,-26,INT32_MAX,-25},{3,-25,INT32_MAX,-24},{4,-24,INT32_MAX,-23},{5,-23,INT32_MAX,-22},{6,-22,INT32_MAX,-21},{7,-21,INT32_MAX,-20},{8,-20,INT32_MAX,-19},{9,-19,INT32_MAX,-18},{10,-18,INT32_MAX,-17},{11,-17,INT32_MAX,-16},{12,-16,INT32_MAX,-15},{13,-15,INT32_MAX,-14},{14,-14,INT32_MAX,-13},{15,-13,INT32_MAX,-12},{16,-12,INT32_MAX,-11},{17,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{19,-9,INT32_MAX,-8},{20,-8,INT32_MAX,-7},{21,-7,INT32_MAX,-6},{22,-6,INT32_MAX,INT32_MAX},
{48,INT32_MIN,INT32_MAX,-138},{28,-138,INT32_MAX,-76},{30,-76,INT32_MAX,-75},{33,-75,INT32_MAX,-74},{35,-74,INT32_MAX,-73},{37,-73,INT32_MAX,-72},{40,-72,INT32_MAX,-71},{42,-71,INT32_MAX,-70},{45,-70,INT32_MAX,-69},{47,-69,INT32_MAX,-68},{50,-68,INT32_MAX,-67},{52,-67,INT32_MAX,-66},{55,-66,INT32_MAX,-65},{57,-65,INT32_MAX,-64},
{INT32_MIN,INT32_MIN,48,-138},{INT32_MIN,-138,28,-77},
//Second angle
{INT32_MIN,-48,19,INT32_MAX},
{19,-48,42,-24},{19,-24,INT32_MAX,INT32_MAX},
{38,-108,INT32_MAX,-48},{42,-48,INT32_MAX,-24},
{INT32_MIN,INT32_MIN,INT32_MAX,-108},{INT32_MIN,-108,38,-48},
//Third angle
{-19,-48,INT32_MAX,INT32_MAX},
{-42,-48,-19,-24},{INT32_MIN,-24,-19,INT32_MAX},
{INT32_MIN,-108,-38,-48},{INT32_MIN,-48,-42,-24},
{INT32_MIN,INT32_MIN,INT32_MAX,-108},{-38,-108,INT32_MAX,-48},
//Fourth angle
{2,-30,INT32_MAX,-29},{1,-29,INT32_MAX,-28},{0,-28,INT32_MAX,-27},{-1,-27,INT32_MAX,-26},{-2,-26,INT32_MAX,-25},{-3,-25,INT32_MAX,-24},{-4,-24,INT32_MAX,-23},{-5,-23,INT32_MAX,-22},{-6,-22,INT32_MAX,-21},{-7,-21,INT32_MAX,-20},{-8,-20,INT32_MAX,-19},{-9,-19,INT32_MAX,-18},{-10,-18,INT32_MAX,-17},{-11,-17,INT32_MAX,-16},{-12,-16,INT32_MAX,-15},{-13,-15,INT32_MAX,-14},{-14,-14,INT32_MAX,-13},{-15,-13,INT32_MAX,-12},{-16,-12,INT32_MAX,-11},{-17,-11,INT32_MAX,-10},{-18,-10,INT32_MAX,-9},{-19,-9,INT32_MAX,-8},{-20,-8,INT32_MAX,-7},{-21,-7,INT32_MAX,INT32_MAX},
{-27,-78,INT32_MAX,-77},{-29,-77,INT32_MAX,-76},{-32,-76,INT32_MAX,-75},{-34,-75,INT32_MAX,-74},{-36,-74,INT32_MAX,-73},{-39,-73,INT32_MAX,-72},{-41,-72,INT32_MAX,-71},{-44,-71,INT32_MAX,-70},{-46,-70,INT32_MAX,-69},{-49,-69,INT32_MAX,-68},{-51,-68,INT32_MAX,-67},{-54,-67,INT32_MAX,-66},{-56,-66,INT32_MAX,-65},{INT32_MIN,-65,INT32_MAX,-30},{INT32_MIN,-30,2,-29},{INT32_MIN,-29,1,-28},{INT32_MIN,-28,0,-27},{INT32_MIN,-27,-1,-26},{INT32_MIN,-26,-2,-25},{INT32_MIN,-25,-3,-24},{INT32_MIN,-24,-4,-23},{INT32_MIN,-23,-5,-22},{INT32_MIN,-22,-6,-21},{INT32_MIN,-21,-7,-20},{INT32_MIN,-20,-8,-19},{INT32_MIN,-19,-9,-18},{INT32_MIN,-18,-10,-17},{INT32_MIN,-17,-11,-16},{INT32_MIN,-16,-12,-15},{INT32_MIN,-15,-13,-14},{INT32_MIN,-14,-14,-13},{INT32_MIN,-13,-15,-12},{INT32_MIN,-12,-16,-11},{INT32_MIN,-11,-17,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-19,-8},{INT32_MIN,-8,-20,-7},{INT32_MIN,-7,-21,INT32_MAX},
{INT32_MIN,INT32_MIN,-47,-139},{INT32_MIN,-139,-27,-77},{INT32_MIN,-77,-29,-76},{INT32_MIN,-76,-32,-75},{INT32_MIN,-75,-34,-74},{INT32_MIN,-74,-36,-73},{INT32_MIN,-73,-39,-72},{INT32_MIN,-72,-41,-71},{INT32_MIN,-71,-44,-70},{INT32_MIN,-70,-46,-69},{INT32_MIN,-69,-49,-68},{INT32_MIN,-68,-51,-67},{INT32_MIN,-67,-54,-66},{INT32_MIN,-66,-56,-65},
{-47,INT32_MIN,INT32_MAX,-139},{-27,-139,INT32_MAX,-78}
};
mask_t half_loop_masks[]={
	{TRACK_MASK_NONE,24,-6,-3,half_loop_rects},{TRACK_MASK_NONE,38,-32,32,half_loop_rects+24},{TRACK_MASK_UNION,14,-48,56,half_loop_rects+62},{TRACK_MASK_DIFFERENCE,2,-32,96+72,half_loop_rects+76},
	{TRACK_MASK_UNION,1,6,-3,half_loop_rects+78},{TRACK_MASK_DIFFERENCE,2,-18,-7,half_loop_rects+79},{TRACK_MASK_NONE,2,-76,-6,half_loop_rects+81},{TRACK_MASK_NONE,2,-32,136,half_loop_rects+83},
	{TRACK_MASK_UNION,1,-6,-3,half_loop_rects+85},{TRACK_MASK_DIFFERENCE,2,26,-3,half_loop_rects+86},{TRACK_MASK_NONE,2,58,-13,half_loop_rects+88},{TRACK_MASK_NONE,2,16,96+32,half_loop_rects+90},
	{TRACK_MASK_NONE,24,6,-3,half_loop_rects+92},{TRACK_MASK_NONE,38,38,29,half_loop_rects+116},{TRACK_MASK_UNION,14,64,48,half_loop_rects+154},{TRACK_MASK_DIFFERENCE,2,48,160,half_loop_rects+168}
};
track_section_t half_loop={0,half_loop_curve,HALF_LOOP_LENGTH,{{VIEW_NEEDS_TRACK_MASK|VIEW_ENFORCE_NON_OVERLAPPING,4,half_loop_masks},{VIEW_NEEDS_TRACK_MASK,4,half_loop_masks+4},{VIEW_NEEDS_TRACK_MASK,4,half_loop_masks+8},{VIEW_NEEDS_TRACK_MASK|VIEW_ENFORCE_NON_OVERLAPPING,4,half_loop_masks+12}}};



