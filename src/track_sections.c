#include <stdint.h>
#include <math.h>
#include "track.h"


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

track_point_t cubic_curve_horizontal(float xa,float xb,float xc,float xd,float ya,float yb,float yc,float yd,float distance)
{
return plane_curve_horizontal(vector3(cubic(ya,yb,yc,yd,distance),0.0,cubic(xa,xb,xc,xd,distance)),vector3_normalize(vector3(cubic_derivative(ya,yb,yc,distance),0.0,cubic_derivative(xa,xb,xc,distance))));
}




track_point_t flat_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,0.0,distance),vector3(0.0,0.0,1.0));
}


track_point_t flat_to_gentle_up_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,(1.0/18.0)*distance*distance,distance),vector3_normalize(vector3(0.0,(1.0/9.0)*distance,1.0)));
}

track_point_t flat_to_gentle_down_curve(float distance) //Unconfirmed
{
return plane_curve_vertical(vector3(0.0,-(1.0/18.0)*distance*distance,distance),vector3_normalize(vector3(0.0,-(1.0/9.0)*distance,1.0)));
}

track_point_t gentle_up_to_flat_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,-(1.0/18.0)*(TILE_SIZE-distance)*(TILE_SIZE-distance),distance),vector3_normalize(vector3(0.0,(1.0/9.0)*(TILE_SIZE-distance),1.0)));
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
return plane_curve_vertical(vector3(0.0,distance,distance*TILE_SIZE/6.0),vector3_normalize(vector3(0.0,4.5/TILE_SIZE,1.0)));
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

track_point_t vertical_twist_left_curve(float distance)
{
track_point_t point;
point.position=vector3(0.0,distance-0.75,0.0);
point.tangent=vector3(0.0,1.0,0.0);
point.normal=vector3(-sin(distance*3.141562654/18.0),0.0,-cos(distance*3.141562654/18.0));
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}

track_point_t vertical_twist_right_curve(float distance)
{
track_point_t point;
point.position=vector3(0.0,distance-0.75,0.0);
point.tangent=vector3(0.0,1.0,0.0);
point.normal=vector3(sin(distance*3.141562654/18.0),0.0,-cos(distance*3.141562654/18.0));
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}

track_point_t small_turn_left_curve(float distance)
{
float angle=distance/(1.5*TILE_SIZE);
track_point_t point;
point.position=vector3(1.5*TILE_SIZE*(1.0-cos(angle)),0,1.5*TILE_SIZE*sin(angle));
point.tangent=vector3(sin(angle),0.0,cos(angle));
point.normal=vector3(0.0,1.0,0.0);
point.binormal=vector3_cross(point.tangent,point.normal);
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
point.binormal=vector3_cross(point.tangent,point.normal);
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






track_point_t heartline_roll_left_curve(float x)
{
track_point_t point;
float length=4.5*sqrt(6)/3.1415926589;
point.position=vector3(-sin(x/length),1-cos(x/length),x);
point.tangent=vector3_normalize(vector3(-cos(x/length)/length,-sin(x/length)/length,1.0));
point.normal=vector3(sin(x/length),cos(x/length),0.0);
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}


//Slopes
track_section_t flat={0,flat_curve,TILE_SIZE,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t flat_to_gentle_up={0,flat_to_gentle_up_curve,TILE_SIZE,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t flat_to_gentle_down={0,flat_to_gentle_down_curve,TILE_SIZE,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t gentle_up_to_flat={0,gentle_up_to_flat_curve,TILE_SIZE,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t gentle={0,gentle_curve,TILE_SIZE,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t gentle_to_steep_up={0,gentle_to_steep_up_curve,3.0,{{1,NULL},{2,NULL},{2,NULL},{1,NULL}}};//Needs manual split
track_section_t steep_to_gentle_up={0,steep_to_gentle_up_curve,3.0,{{1,NULL},{2,NULL},{2,NULL},{1,NULL}}};//Needs manual split
track_section_t steep={0,steep_curve,6.0,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t steep_to_vertical_up={0,steep_to_vertical_up_curve,5.25,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t steep_to_vertical_down={0,steep_to_vertical_down_curve,5.25,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t vertical={0,vertical_curve,3.0,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};

//Turns
rect_t small_turn_rects[165]={
	{INT32_MIN,-1,INT32_MAX,INT32_MAX},{INT32_MIN,-2,-2,INT32_MAX},{INT32_MIN,-3,-4,INT32_MAX},{INT32_MIN,-4,-6,INT32_MAX},{INT32_MIN,-5,-8,INT32_MAX},{INT32_MIN,-6,-10,INT32_MAX},{INT32_MIN,-7,-12,INT32_MAX},{INT32_MIN,-8,-14,INT32_MAX},{INT32_MIN,-9,-16,INT32_MAX},{INT32_MIN,-10,-18,INT32_MAX},{INT32_MIN,-11,-20,INT32_MAX},{INT32_MIN,-12,-22,INT32_MAX},{INT32_MIN,-13,-24,INT32_MAX},{0,-12,INT32_MAX,1},{2,-13,INT32_MAX,1},{4,-14,INT32_MAX,1},{6,-15,INT32_MAX,1},{8,-16,INT32_MAX,1},{10,-17,INT32_MAX,1},{12,-18,INT32_MAX,1},{14,-19,INT32_MAX,1},{16,-20,INT32_MAX,1},{18,-21,INT32_MAX,1},{20,-22,INT32_MAX,1},{22,-23,INT32_MAX,1},{24,-24,INT32_MAX,1},{INT32_MIN,INT32_MIN,-24,-13},{-24,INT32_MIN,-22,-12},{-22,INT32_MIN,-20,-11},{-20,INT32_MIN,-18,-10},{-18,INT32_MIN,-16,-9},{-16,INT32_MIN,-14,-8},{-14,INT32_MIN,-12,-7},{-12,INT32_MIN,-10,-6},{-10,INT32_MIN,-8,-5},{-8,INT32_MIN,-6,-4},{-6,INT32_MIN,-4,-3},{-4,INT32_MIN,-2,-2},{-2,INT32_MIN,0,-1},{0,INT32_MIN,2,-12},{2,INT32_MIN,4,-13},{4,INT32_MIN,6,-14},{6,INT32_MIN,8,-15},{8,INT32_MIN,10,-16},{10,INT32_MIN,12,-17},{12,INT32_MIN,14,-18},{14,INT32_MIN,16,-19},{16,INT32_MIN,18,-20},{18,INT32_MIN,20,-21},{20,INT32_MIN,22,-22},{22,INT32_MIN,24,-23},{24,INT32_MIN,INT32_MAX,-24},{INT32_MIN,INT32_MIN,0,INT32_MAX},{0,INT32_MIN,2,31},{2,INT32_MIN,4,30},{4,INT32_MIN,6,29},{6,INT32_MIN,8,28},{8,INT32_MIN,10,27},{10,INT32_MIN,12,26},{12,INT32_MIN,14,25},{14,INT32_MIN,16,24},{16,INT32_MIN,18,23},{18,INT32_MIN,20,22},{20,INT32_MIN,22,21},{22,INT32_MIN,24,20},{24,INT32_MIN,26,19},{26,INT32_MIN,28,18},{28,INT32_MIN,30,17},{30,INT32_MIN,32,16},{30,16,34,INT32_MAX},{28,17,36,INT32_MAX},{26,18,38,INT32_MAX},{24,19,40,INT32_MAX},{22,20,42,INT32_MAX},{20,21,44,INT32_MAX},{18,22,46,INT32_MAX},{16,23,48,INT32_MAX},{14,24,50,INT32_MAX},{12,25,52,INT32_MAX},{10,26,54,INT32_MAX},{8,27,56,INT32_MAX},{6,28,58,INT32_MAX},{4,29,60,INT32_MAX},{2,30,62,INT32_MAX},{0,31,64,INT32_MAX},{32,INT32_MIN,34,16},{34,INT32_MIN,36,17},{36,INT32_MIN,38,18},{38,INT32_MIN,40,19},{40,INT32_MIN,42,20},{42,INT32_MIN,44,21},{44,INT32_MIN,46,22},{46,INT32_MIN,48,23},{48,INT32_MIN,50,24},{50,INT32_MIN,52,25},{52,INT32_MIN,54,26},{54,INT32_MIN,56,27},{56,INT32_MIN,58,28},{58,INT32_MIN,60,29},{60,INT32_MIN,62,30},{62,INT32_MIN,64,31},{64,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,7},{-24,INT32_MIN,-22,8},{-22,INT32_MIN,-20,9},{-20,INT32_MIN,-18,10},{-18,INT32_MIN,-16,11},{-16,INT32_MIN,-14,12},{-14,INT32_MIN,-12,13},{-12,INT32_MIN,-10,14},{-10,INT32_MIN,-8,15},{-8,INT32_MIN,-6,16},{-6,INT32_MIN,-4,17},{-4,INT32_MIN,-2,18},{-2,INT32_MIN,0,19},{0,INT32_MIN,2,31},{2,INT32_MIN,4,30},{4,INT32_MIN,6,29},{6,INT32_MIN,8,28},{8,INT32_MIN,10,27},{10,INT32_MIN,12,26},{12,INT32_MIN,14,25},{14,INT32_MIN,16,24},{16,INT32_MIN,18,23},{18,INT32_MIN,20,22},{20,INT32_MIN,22,21},{22,INT32_MIN,24,20},{24,INT32_MIN,INT32_MAX,19},{INT32_MIN,19,0,31},{INT32_MIN,18,-2,31},{INT32_MIN,17,-4,31},{INT32_MIN,16,-6,31},{INT32_MIN,15,-8,31},{INT32_MIN,14,-10,31},{INT32_MIN,13,-12,31},{INT32_MIN,12,-14,31},{INT32_MIN,11,-16,31},{INT32_MIN,10,-18,31},{INT32_MIN,9,-20,31},{INT32_MIN,8,-22,31},{INT32_MIN,7,-24,31},{INT32_MIN,31,INT32_MAX,INT32_MAX},{2,30,INT32_MAX,INT32_MAX},{4,29,INT32_MAX,INT32_MAX},{6,28,INT32_MAX,INT32_MAX},{8,27,INT32_MAX,INT32_MAX},{10,26,INT32_MAX,INT32_MAX},{12,25,INT32_MAX,INT32_MAX},{14,24,INT32_MAX,INT32_MAX},{16,23,INT32_MAX,INT32_MAX},{18,22,INT32_MAX,INT32_MAX},{20,21,INT32_MAX,INT32_MAX},{22,20,INT32_MAX,INT32_MAX},{24,19,INT32_MAX,INT32_MAX},{-26,INT32_MIN,INT32_MAX,INT32_MAX},{-28,13,-26,INT32_MAX},{-30,14,-28,INT32_MAX},{-32,15,-30,INT32_MAX},{-38,INT32_MIN,-26,13},{-39,INT32_MIN,-25,14},{-40,INT32_MIN,-24,15},{INT32_MIN,INT32_MIN,26-64,INT32_MAX},{26-64,13,28-64,INT32_MAX},{28-64,14,30-64,INT32_MAX},{30-64,15,32-64,INT32_MAX}
};
mask_t small_turn_masks[12]={
	{13,0,0,small_turn_rects},{13,-32,16,small_turn_rects+13},{26,0,32,small_turn_rects+26},
	{17,0,0,small_turn_rects+52},{16,-32,-16,small_turn_rects+69},{17,-64,0,small_turn_rects+85},
	{26,0,0,small_turn_rects+102},{13,32,-16,small_turn_rects+128},{13,0,-32,small_turn_rects+141},
	{4,0,0,small_turn_rects+154},{3,32,16,small_turn_rects+158},{4,64,0,small_turn_rects+161}
};
track_section_t small_turn_left={0,small_turn_left_curve,0.75*TILE_SIZE*3.1415926,{{3,small_turn_masks},{3,small_turn_masks+3},{3,small_turn_masks+6},{3,small_turn_masks+9}}};
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
{33,0,0,medium_turn_rects},{32,-32,16,medium_turn_rects+33},{31,0,32,medium_turn_rects+65},{32,-32,48,medium_turn_rects+96},{32,0,64,medium_turn_rects+128},
{33,0,0,medium_turn_rects+160},{31,-32,-16,medium_turn_rects+193},{32,-64,0,medium_turn_rects+224},{31,-96,-16,medium_turn_rects+256},{33,-128,0,medium_turn_rects+287},
{33,0,0,medium_turn_rects+320},{32,32,-16,medium_turn_rects+353},{31,0,-32,medium_turn_rects+384},{32,32,-48,medium_turn_rects+416},{32,0,-64,medium_turn_rects+448},
{33,0,0,medium_turn_rects+480},{31,32,16,medium_turn_rects+513},{32,64,0,medium_turn_rects+544},{31,96,16,medium_turn_rects+576},{33,128,0,medium_turn_rects+607}
};
track_section_t medium_turn_left={0,medium_turn_left_curve,1.25*TILE_SIZE*3.1415926,{{5,medium_turn_masks},{5,medium_turn_masks+5},{5,medium_turn_masks+10},{5,medium_turn_masks+15}}};
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
{33,0,0,large_turn_left_to_diag_rects},{47,-32,16,large_turn_left_to_diag_rects+33},{31,0,32,large_turn_left_to_diag_rects+80},{31,-32,48,large_turn_left_to_diag_rects+111},
{33,0,0,large_turn_left_to_diag_rects+142},{31,-32,-16,large_turn_left_to_diag_rects+175},{32,-64,0,large_turn_left_to_diag_rects+206},{33,-96,-16,large_turn_left_to_diag_rects+238},
{16,0,0,large_turn_left_to_diag_rects+271},{32,32,-16,large_turn_left_to_diag_rects+287},{33,0,-32,large_turn_left_to_diag_rects+319},{33,32,-48,large_turn_left_to_diag_rects+352},
{17,0,0,large_turn_left_to_diag_rects+385},{16,32,16,large_turn_left_to_diag_rects+402},{16,64,0,large_turn_left_to_diag_rects+418},{17,96,16,large_turn_left_to_diag_rects+434},
};
track_section_t large_turn_left_to_diag={0,large_turn_left_to_diag_curve,0.875*TILE_SIZE*3.1415926,{{4,large_turn_left_to_diag_masks},{4,large_turn_left_to_diag_masks+4},{4,large_turn_left_to_diag_masks+8},{4,large_turn_left_to_diag_masks+12}}};
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
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,-26,-50,-25},{INT32_MIN,-25,-48,-24},{INT32_MIN,-24,-46,-23},{INT32_MIN,-23,-44,-22},{INT32_MIN,-22,-42,-21},{INT32_MIN,-21,-40,-20},{INT32_MIN,-20,-38,-19},{INT32_MIN,-19,-36,-18},{INT32_MIN,-18,-34,-17},{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,-2,1},{INT32_MIN,1,-4,2},{INT32_MIN,2,-6,3},{INT32_MIN,3,-8,4},{INT32_MIN,4,-10,5},{INT32_MIN,5,-12,6},{INT32_MIN,6,-14,7},{INT32_MIN,7,-16,8},{INT32_MIN,8,-18,9},{INT32_MIN,9,-20,10},{INT32_MIN,10,-22,11},{INT32_MIN,11,-24,12},{INT32_MIN,12,-26,13},{INT32_MIN,13,-28,14},{INT32_MIN,14,-30,15},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,INT32_MAX,-31},{INT32_MIN,-31,-4,-30},{4,-31,INT32_MAX,-30},{INT32_MIN,-30,-6,-29},{6,-30,INT32_MAX,-29},{INT32_MIN,-29,-8,-28},{8,-29,INT32_MAX,-28},{INT32_MIN,-28,-10,-27},{10,-28,INT32_MAX,-27},{INT32_MIN,-27,-12,-26},{12,-27,INT32_MAX,-26},{-50,-26,-14,-25},{14,-26,INT32_MAX,-25},{-48,-25,-16,-24},{16,-25,INT32_MAX,-24},{-46,-24,-18,-23},{18,-24,INT32_MAX,-23},{-44,-23,-20,-22},{20,-23,INT32_MAX,-22},{-42,-22,-22,-21},{22,-22,INT32_MAX,-21},{-40,-21,-24,-20},{24,-21,INT32_MAX,-20},{-38,-20,-26,-19},{26,-20,INT32_MAX,-19},{-36,-19,-28,-18},{28,-19,INT32_MAX,-18},{-34,-18,-30,-17},{30,-18,INT32_MAX,-17}
};
mask_t large_turn_right_to_diag_masks[]={
{49,0,0,large_turn_right_to_diag_rects},{16,-32,16,large_turn_right_to_diag_rects+49},{31,-64,0,large_turn_right_to_diag_rects+65},{16,-96,16,large_turn_right_to_diag_rects+96},
{16,0,0,large_turn_right_to_diag_rects+112},{32,-32,-16,large_turn_right_to_diag_rects+128},{34,0,-32,large_turn_right_to_diag_rects+160},{33,-32,-48,large_turn_right_to_diag_rects+194},
{16,0,0,large_turn_right_to_diag_rects+227},{32,32,-16,large_turn_right_to_diag_rects+243},{32,64,0,large_turn_right_to_diag_rects+275},{33,96,-16,large_turn_right_to_diag_rects+307},
{33,0,0,large_turn_right_to_diag_rects+340},{31,32,16,large_turn_right_to_diag_rects+373},{41,0,32,large_turn_right_to_diag_rects+404},{31,32,48,large_turn_right_to_diag_rects+445},
};
track_section_t large_turn_right_to_diag={0,large_turn_right_to_diag_curve,0.875*TILE_SIZE*3.1415926,{{4,large_turn_right_to_diag_masks},{4,large_turn_right_to_diag_masks+4},{4,large_turn_right_to_diag_masks+8},{4,large_turn_right_to_diag_masks+12}}};

//Diagonals
track_section_t flat_diag={TRACK_DIAGONAL,flat_diag_curve,sqrt(2)*TILE_SIZE,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};



// 0

//+64


track_section_t vertical_twist_left={0,vertical_twist_left_curve,9.0,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};
track_section_t vertical_twist_right={0,vertical_twist_right_curve,9.0,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};












track_section_t heartline_roll_left={0,heartline_roll_left_curve,3.0*TILE_SIZE,{{1,NULL},{1,NULL},{1,NULL},{1,NULL}}};





