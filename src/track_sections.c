#include <stdint.h>
#include <math.h>
#include "track.h"
#define NORM(x,y) (sqrt((x)*(x)+(y)*(y)))

#define FLAT_LENGTH TILE_SIZE
#define FLAT_TO_GENTLE_LENGTH (1.027122*TILE_SIZE)
#define GENTLE_LENGTH (1.080123*TILE_SIZE)
#define GENTLE_TO_STEEP_LENGTH (1.314179*TILE_SIZE)
#define STEEP_LENGTH (1.914854*TILE_SIZE)
#define STEEP_TO_VERTICAL_LENGTH (1.531568*TILE_SIZE)
#define VERTICAL_LENGTH (0.816497*TILE_SIZE)

#define FLAT_DIAG_LENGTH (1.414213*TILE_SIZE)
#define FLAT_TO_GENTLE_DIAG_LENGTH (1.433617*TILE_SIZE)
#define GENTLE_DIAG_LENGTH (1.471960*TILE_SIZE)
#define GENTLE_TO_STEEP_DIAG_LENGTH (1.656243*TILE_SIZE)
#define STEEP_DIAG_LENGTH (2.160247*TILE_SIZE)

#define PI 3.1415926
#define VERY_SMALL_TURN_LENGTH (0.25*M_PI*TILE_SIZE)
#define SMALL_TURN_LENGTH (0.75*PI*TILE_SIZE)
#define MEDIUM_TURN_LENGTH (1.25*PI*TILE_SIZE)
#define LARGE_TURN_LENGTH (2.757100*TILE_SIZE)
#define SMALL_TURN_GENTLE_LENGTH (2.493656*TILE_SIZE)
#define MEDIUM_TURN_GENTLE_LENGTH (4.252990*TILE_SIZE)
#define VERY_SMALL_TURN_STEEP_LENGTH (1.812048*TILE_SIZE)
#define VERTICAL_TWIST_LENGTH (2.449490*TILE_SIZE)

#define S_BEND_LENGTH (3.240750*TILE_SIZE)
#define SMALL_HELIX_LENGTH (2.365020*TILE_SIZE)
#define MEDIUM_HELIX_LENGTH (3.932292*TILE_SIZE)
#define TURN_BANK_TRANSITION_LENGTH (2.442290*TILE_SIZE) 

#define BARREL_ROLL_LENGTH (3.091882*TILE_SIZE)
#define HALF_LOOP_SEGMENT1_LENGTH (0.540062*TILE_SIZE)
#define HALF_LOOP_SEGMENT2_LENGTH (HALF_LOOP_SEGMENT1_LENGTH+2.685141*TILE_SIZE)
#define HALF_LOOP_LENGTH (HALF_LOOP_SEGMENT2_LENGTH+1.956695*TILE_SIZE)
#define QUARTER_LOOP_LENGTH (4.253756*TILE_SIZE)

#define CORKSCREW_SEGMENT1_LENGTH (1.682311*TILE_SIZE)
#define CORKSCREW_SEGMENT2_LENGTH (1.744083*TILE_SIZE)
#define CORKSCREW_LENGTH (CORKSCREW_SEGMENT1_LENGTH+CORKSCREW_SEGMENT2_LENGTH)

#define LARGE_CORKSCREW_SEGMENT1_LENGTH (2.824277*TILE_SIZE)
#define LARGE_CORKSCREW_SEGMENT2_LENGTH (2.933438*TILE_SIZE)
#define LARGE_CORKSCREW_LENGTH (LARGE_CORKSCREW_SEGMENT1_LENGTH+LARGE_CORKSCREW_SEGMENT2_LENGTH)


#define LARGE_HALF_LOOP_FACTOR 1.0050562625650226
#define LARGE_HALF_LOOP_SEGMENT1_LENGTH (1.5*GENTLE_LENGTH)
#define LARGE_HALF_LOOP_SEGMENT2_LENGTH (LARGE_HALF_LOOP_SEGMENT1_LENGTH+4.766127*TILE_SIZE)
#define LARGE_HALF_LOOP_LENGTH ((LARGE_HALF_LOOP_SEGMENT2_LENGTH+3.545350*TILE_SIZE)*LARGE_HALF_LOOP_FACTOR)

#define FLAT_TO_STEEP_LENGTH (4.792426*TILE_SIZE)
#define SMALL_FLAT_TO_STEEP_LENGTH (1.221327*TILE_SIZE)
#define SMALL_FLAT_TO_STEEP_DIAG_LENGTH (1.584693*TILE_SIZE)




float cubic(float a,float b, float c,float d,float x)
{
return x*(x*(x*a+b)+c)+d;
}
float reparameterize_old(float a,float b,float c,float d,float e,float f,float g,float x)
{
x=3.674234614174767*x/TILE_SIZE;//TODO remove this correction
return x*(x*(x*(x*(x*(x*(x*a+b)+c)+d)+e)+f)+g);
}
float reparameterize(float a,float b,float c,float d,float e,float f,float g,float x)
{
return x*(x*(x*(x*(x*(x*(x*a+b)+c)+d)+e)+f)+g);
}

float cubic_derivative(float a,float b,float c,float x)
{
return (x*(3.0*x*a+2.0*b)+c);
}

float cubic_second_derivative(float a,float b,float x)
{
return 6.0*x*a+2.0*b;
}
track_point_t plane_curve_vertical(vector3_t position,vector3_t tangent)
{
track_point_t point;
point.position=position;
point.tangent=tangent;
point.normal=vector3(0.0,tangent.z,-tangent.y);
point.binormal=vector3(-1.0,0.0,0.0);
return point;
}
track_point_t plane_curve_vertical_diagonal(vector3_t position,vector3_t tangent)
{
track_point_t point;
point.position=position;
point.tangent=tangent;
point.normal=vector3(tangent.y/sqrt(2),tangent.z*sqrt(2),-tangent.y/sqrt(2));
point.binormal=vector3(-sqrt(0.5),0.0,-sqrt(0.5)); 
return point;
}
track_point_t plane_curve_horizontal(vector3_t position,vector3_t tangent)
{
track_point_t point;
point.position=position;
point.tangent=tangent;
point.normal=vector3(0.0,1.0,0.0);
point.binormal=vector3(-tangent.z,0.0,tangent.x);
return point;
}

track_point_t cubic_curve_vertical(float xa,float xb,float xc,float xd,float ya,float yb,float yc,float yd,float pa,float pb,float pc,float pd,float pe,float pf,float pg,float distance)
{
float u=reparameterize_old(pa,pb,pc,pd,pe,pf,pg,distance);
return plane_curve_vertical(vector3(0.0,cubic(ya,yb,yc,yd,u),cubic(xa,xb,xc,xd,u)),vector3_normalize(vector3(0.0,cubic_derivative(ya,yb,yc,u),cubic_derivative(xa,xb,xc,u))));
}
track_point_t cubic_curve_vertical_diagonal(float xa,float xb,float xc,float xd,float ya,float yb,float yc,float yd,float pa,float pb,float pc,float pd,float pe,float pf,float pg,float distance)
{
float u=reparameterize_old(pa,pb,pc,pd,pe,pf,pg,distance);
float x=cubic(xa,xb,xc,xd,u);
float y=cubic(ya,yb,yc,yd,u);
float dx=cubic_derivative(xa,xb,xc,u);
float dy=cubic_derivative(ya,yb,yc,u);
return plane_curve_vertical_diagonal(vector3(-x/sqrt(2),y,x/sqrt(2)),vector3_normalize(vector3(-dx/sqrt(2),dy,dx/sqrt(2))));
}
track_point_t cubic_curve_horizontal(float xa,float xb,float xc,float xd,float ya,float yb,float yc,float yd,float pa,float pb,float pc,float pd,float pe,float pf,float pg,float distance)
{
float u=reparameterize_old(pa,pb,pc,pd,pe,pf,pg,distance);
return plane_curve_horizontal(vector3(cubic(ya,yb,yc,yd,u),0.0,cubic(xa,xb,xc,xd,u)),vector3_normalize(vector3(cubic_derivative(ya,yb,yc,u),0.0,cubic_derivative(xa,xb,xc,u))));
}

track_point_t banked_curve(track_point_t unbanked_curve,float angle)
{
track_point_t point;
point.position=unbanked_curve.position;
point.tangent=unbanked_curve.tangent;
point.normal=vector3_add(vector3_mult(unbanked_curve.normal,cos(angle)),vector3_mult(unbanked_curve.binormal,-sin(angle)));
point.binormal=vector3_add(vector3_mult(unbanked_curve.normal,sin(angle)),vector3_mult(unbanked_curve.binormal,cos(angle)));
return point;
}

track_point_t flat_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,0.0,distance),vector3(0.0,0.0,1.0));
}
track_point_t flat_to_gentle_up_curve(float distance)
{
return cubic_curve_vertical(0,0,TILE_SIZE,0,0,CLEARANCE_HEIGHT,0,0,1.53990713e-09,-3.71353195e-07,5.71497773e-06,-2.15973089e-06,-5.57959067e-04,-9.43549275e-07,2.72165680e-01,distance);
}
//track_point_t flat_to_gentle_down_curve(float distance) //Unconfirmed
//{
//return plane_curve_vertical(vector3(0.0,-(1.0/18.0)*distance*distance,distance),vector3_normalize(vector3(0.0,-(1.0/9.0)*distance,1.0)));
//}
track_point_t gentle_up_to_flat_curve(float distance)
{
return cubic_curve_vertical(0,0,TILE_SIZE,0,0,-CLEARANCE_HEIGHT,2*CLEARANCE_HEIGHT,0,1.53990713e-09,-3.71353195e-07,5.71497773e-06,-2.15973089e-06,-5.57959067e-04,-9.43549275e-07,2.72165680e-01,distance);
}
track_point_t gentle_curve(float distance)
{
float u=distance/GENTLE_LENGTH;
return plane_curve_vertical(vector3(0.0,2*CLEARANCE_HEIGHT*u,u*TILE_SIZE),vector3_normalize(vector3(0.0,2*CLEARANCE_HEIGHT/TILE_SIZE,1.0)));
}

track_point_t gentle_to_steep_up_curve(float distance)
{
return cubic_curve_vertical(-0.5*TILE_SIZE,TILE_SIZE,0.5*TILE_SIZE,0,CLEARANCE_HEIGHT,2*CLEARANCE_HEIGHT,CLEARANCE_HEIGHT,0,1.03809332e-04,-2.00628254e-03,1.59305808e-02,-6.75327840e-02,1.68115244e-01,-2.67819389e-01,4.74022260e-01,distance);
}
track_point_t steep_to_gentle_up_curve(float distance)
{
return cubic_curve_vertical(-0.5*TILE_SIZE,0.5*TILE_SIZE,TILE_SIZE,0,CLEARANCE_HEIGHT,-5*CLEARANCE_HEIGHT,8*CLEARANCE_HEIGHT,0,1.03809332e-04,-1.50249574e-03,8.63282156e-03,-2.44630312e-02,3.57654761e-02,-1.76496309e-02,1.47759227e-01,distance);
}
track_point_t steep_curve(float distance)
{
float u=distance/STEEP_LENGTH;
return plane_curve_vertical(vector3(0.0,8*CLEARANCE_HEIGHT*u,TILE_SIZE*u),vector3_normalize(vector3(0.0,8*CLEARANCE_HEIGHT/TILE_SIZE,1.0)));
}
track_point_t vertical_curve(float distance)
{
return plane_curve_vertical(vector3(0.0,distance,0.0),vector3(0.0,1.0,0.0));
}
track_point_t steep_to_vertical_up_curve(float distance)
{
return cubic_curve_vertical(-TILE_SIZE/6,-TILE_SIZE/6,5*TILE_SIZE/6,-TILE_SIZE/2,2*CLEARANCE_HEIGHT/3,-CLEARANCE_HEIGHT/3,20*CLEARANCE_HEIGHT/3,0,-1.27409679e-07,1.66746015e-06,-6.60784097e-06,2.87367949e-06,5.24290249e-04,-1.54572818e-03,1.70550125e-01,distance);
}
track_point_t vertical_to_steep_up_curve(float distance)
{
return cubic_curve_vertical(-TILE_SIZE/6,2*TILE_SIZE/3,0,0,-2*CLEARANCE_HEIGHT/3,CLEARANCE_HEIGHT,20*CLEARANCE_HEIGHT/3,0,-1.27409680e-07,3.35138224e-06,-3.50358430e-05,1.85655626e-04,-3.24817476e-05,-6.05934285e-03,2.00014155e-01,distance);
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
point.binormal=vector3_cross(point.normal,point.tangent);
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
return cubic_curve_horizontal(68*CLEARANCE_HEIGHT/3-5*TILE_SIZE,7.5*TILE_SIZE-112*CLEARANCE_HEIGHT/3,44*CLEARANCE_HEIGHT/3,0,8*CLEARANCE_HEIGHT-2*TILE_SIZE,3*TILE_SIZE-8*CLEARANCE_HEIGHT,0,0,1.94965989e-09,-3.53421586e-08,3.08229652e-07,3.09682840e-06,-2.92456470e-06,3.25760288e-04,9.0924568e-02,distance);
}
track_point_t large_turn_right_to_diag_curve(float distance)
{
return cubic_curve_horizontal(68*CLEARANCE_HEIGHT/3-5*TILE_SIZE,7.5*TILE_SIZE-112*CLEARANCE_HEIGHT/3,44*CLEARANCE_HEIGHT/3,0,2*TILE_SIZE-8*CLEARANCE_HEIGHT,8*CLEARANCE_HEIGHT-3*TILE_SIZE,0,0,1.94965989e-09,-3.53421586e-08,3.08229652e-07,3.09682840e-06,-2.92456470e-06,3.25760288e-04,9.0924568e-02,distance);
}

track_point_t flat_diag_curve(float distance)
{
return plane_curve_horizontal(vector3(-distance/sqrt(2),0.0,distance/sqrt(2)),vector3(-sqrt(0.5),0.0,sqrt(0.5)));
}
track_point_t flat_to_gentle_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(0,0,FLAT_DIAG_LENGTH,0,0,CLEARANCE_HEIGHT,0,0,-1.92303122e-10,-3.87922510e-09,2.17273953e-07,-4.77178399e-08,-9.89319820e-05,-4.25613783e-08,1.92450100e-01,distance);
}
//track_point_t flat_to_gentle_down_diag_curve(float distance)
//{
//return cubic_curve_vertical_diagonal(0,0,FLAT_DIAG_LENGTH,0,0,-1.0/18.0,0,0.75,distance/sqrt(2));
//}
track_point_t gentle_to_flat_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(0,0,FLAT_DIAG_LENGTH,0,0,-CLEARANCE_HEIGHT,2*CLEARANCE_HEIGHT,0.0,-1.92302736e-10,1.09698407e-08,-1.73760158e-08,-3.07650071e-06,-5.61731001e-05,1.31496758e-03,1.84900055e-01,distance);
}
track_point_t gentle_diag_curve(float distance)
{
float u=distance/GENTLE_DIAG_LENGTH;
return plane_curve_vertical_diagonal(vector3(-TILE_SIZE*u,2*CLEARANCE_HEIGHT*u,TILE_SIZE*u),vector3_normalize(vector3(-1.0/sqrt(2),2*CLEARANCE_HEIGHT/(sqrt(2)*TILE_SIZE),1.0/sqrt(2))));
}
track_point_t gentle_to_steep_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(-0.5*FLAT_DIAG_LENGTH,FLAT_DIAG_LENGTH,0.5*FLAT_DIAG_LENGTH,0,CLEARANCE_HEIGHT,2*CLEARANCE_HEIGHT,CLEARANCE_HEIGHT,0,1.78635273e-05,-4.35933078e-04,4.37240480e-03,-2.34451422e-02,7.41421344e-02,-1.50070476e-01,3.50024806e-01,distance);
}
track_point_t steep_to_gentle_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(-0.5*FLAT_DIAG_LENGTH,0.5*FLAT_DIAG_LENGTH,FLAT_DIAG_LENGTH,0,CLEARANCE_HEIGHT,-5*CLEARANCE_HEIGHT,8*CLEARANCE_HEIGHT,0,1.78635273e-05,-3.25016888e-04,2.34748867e-03,-8.33887962e-03,1.52652332e-02,-1.07964578e-02,1.29831889e-01,distance);
}
track_point_t steep_diag_curve(float distance)
{
float u=distance/STEEP_DIAG_LENGTH;
return plane_curve_vertical_diagonal(vector3(-TILE_SIZE*u,8*CLEARANCE_HEIGHT*u,TILE_SIZE*u),vector3_normalize(vector3(-1.0,8*CLEARANCE_HEIGHT/TILE_SIZE,1.0)));
}

track_point_t flat_to_left_bank_curve(float distance)
{
return banked_curve(flat_curve(distance),0.25*M_PI*distance/FLAT_LENGTH);
}
track_point_t flat_to_right_bank_curve(float distance)
{
return banked_curve(flat_curve(distance),-0.25*M_PI*distance/FLAT_LENGTH);
}
track_point_t left_bank_to_gentle_up_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),0.25*M_PI*(1.0-distance/FLAT_TO_GENTLE_LENGTH));
}
track_point_t right_bank_to_gentle_up_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),-0.25*M_PI*(1.0-distance/FLAT_TO_GENTLE_LENGTH));
}
track_point_t gentle_up_to_left_bank_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),0.25*M_PI*distance/FLAT_TO_GENTLE_LENGTH);
}
track_point_t gentle_up_to_right_bank_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),-0.25*M_PI*distance/FLAT_TO_GENTLE_LENGTH);
}
track_point_t left_bank_curve(float distance)
{
return banked_curve(flat_curve(distance),0.25*M_PI);
}
track_point_t flat_to_left_bank_diag_curve(float distance)
{
return banked_curve(flat_diag_curve(distance),0.25*M_PI*distance/FLAT_DIAG_LENGTH);
}
track_point_t flat_to_right_bank_diag_curve(float distance)
{
return banked_curve(flat_diag_curve(distance),-0.25*M_PI*distance/FLAT_DIAG_LENGTH);
}
track_point_t left_bank_to_gentle_up_diag_curve(float distance)
{
return banked_curve(flat_to_gentle_up_diag_curve(distance),0.25*M_PI*(1.0-distance/FLAT_TO_GENTLE_DIAG_LENGTH));
}
track_point_t right_bank_to_gentle_up_diag_curve(float distance)
{
return banked_curve(flat_to_gentle_up_diag_curve(distance),-0.25*M_PI*(1.0-distance/FLAT_TO_GENTLE_DIAG_LENGTH));
}
track_point_t gentle_up_to_left_bank_diag_curve(float distance)
{
return banked_curve(gentle_to_flat_up_diag_curve(distance),0.25*M_PI*distance/FLAT_TO_GENTLE_DIAG_LENGTH);
}
track_point_t gentle_up_to_right_bank_diag_curve(float distance)
{
return banked_curve(gentle_to_flat_up_diag_curve(distance),-0.25*M_PI*distance/FLAT_TO_GENTLE_DIAG_LENGTH);
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
point.binormal=vector3_cross(point.tangent,point.normal);
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
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}
track_point_t small_turn_left_gentle_up_curve(float distance)
{
return sloped_turn_left_curve(1.5*TILE_SIZE,4*CLEARANCE_HEIGHT/SMALL_TURN_LENGTH,distance);
}
track_point_t small_turn_right_gentle_up_curve(float distance)
{
return sloped_turn_right_curve(1.5*TILE_SIZE,4*CLEARANCE_HEIGHT/SMALL_TURN_LENGTH,distance);
}
track_point_t medium_turn_left_gentle_up_curve(float distance)
{
return sloped_turn_left_curve(2.5*TILE_SIZE,8*CLEARANCE_HEIGHT/MEDIUM_TURN_LENGTH,distance);
}
track_point_t medium_turn_right_gentle_up_curve(float distance)
{
return sloped_turn_right_curve(2.5*TILE_SIZE,8*CLEARANCE_HEIGHT/MEDIUM_TURN_LENGTH,distance);
}
track_point_t very_small_turn_left_steep_up_curve(float distance)
{
return sloped_turn_left_curve(0.5*TILE_SIZE,8*CLEARANCE_HEIGHT/VERY_SMALL_TURN_LENGTH,distance);
}
track_point_t very_small_turn_right_steep_up_curve(float distance)
{
return sloped_turn_right_curve(0.5*TILE_SIZE,8*CLEARANCE_HEIGHT/VERY_SMALL_TURN_LENGTH,distance);
}

track_point_t vertical_twist_left_up_curve(float distance)
{
track_point_t point;
point.position=vector3(0.0,distance,0.0);
point.tangent=vector3(0.0,1.0,0.0);
point.normal=vector3(-sin(0.5*PI*distance/VERTICAL_TWIST_LENGTH),0.0,-cos(0.5*PI*distance/VERTICAL_TWIST_LENGTH));
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}
track_point_t vertical_twist_right_up_curve(float distance)
{
track_point_t point;
point.position=vector3(0.0,distance,0.0);
point.tangent=vector3(0.0,1.0,0.0);
point.normal=vector3(sin(0.5*PI*distance/VERTICAL_TWIST_LENGTH),0.0,-cos(0.5*PI*distance/VERTICAL_TWIST_LENGTH));
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}

track_point_t gentle_up_to_gentle_up_left_bank_curve(float distance)
{
return banked_curve(gentle_curve(distance),0.25*M_PI*distance/GENTLE_LENGTH);
}
track_point_t gentle_up_to_gentle_up_right_bank_curve(float distance)
{
return banked_curve(gentle_curve(distance),-0.25*M_PI*distance/GENTLE_LENGTH);
}
track_point_t gentle_up_left_bank_to_gentle_up_curve(float distance)
{
return banked_curve(gentle_curve(distance),0.25*M_PI*(1.0-distance/GENTLE_LENGTH));
}
track_point_t gentle_up_right_bank_to_gentle_up_curve(float distance)
{
return banked_curve(gentle_curve(distance),-0.25*M_PI*(1.0-distance/GENTLE_LENGTH));
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
return banked_curve(flat_to_gentle_up_curve(distance),0.25*M_PI*distance/FLAT_TO_GENTLE_LENGTH);
}
track_point_t flat_to_gentle_up_right_bank_curve(float distance)
{
return banked_curve(flat_to_gentle_up_curve(distance),-0.25*M_PI*distance/FLAT_TO_GENTLE_LENGTH);
}
track_point_t gentle_up_left_bank_to_flat_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),0.25*M_PI*(1.0-distance/FLAT_TO_GENTLE_LENGTH));
}
track_point_t gentle_up_right_bank_to_flat_curve(float distance)
{
return banked_curve(gentle_up_to_flat_curve(distance),-0.25*M_PI*(1.0-distance/FLAT_TO_GENTLE_LENGTH));
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



track_point_t s_bend_left_curve(float distance)
{
return cubic_curve_horizontal(152*CLEARANCE_HEIGHT/3-6*TILE_SIZE,9*TILE_SIZE-76*CLEARANCE_HEIGHT,76*CLEARANCE_HEIGHT/3,0,-TILE_SIZE*2,3*TILE_SIZE,0,0,-1.80937408e-07,7.54065079e-06,-1.12351547e-04,6.71657797e-04,-1.39559106e-03,5.48962382e-03,5.1374757e-02,distance);
}
track_point_t s_bend_right_curve(float distance)
{
return cubic_curve_horizontal(152*CLEARANCE_HEIGHT/3-6*TILE_SIZE,9*TILE_SIZE-76*CLEARANCE_HEIGHT,76*CLEARANCE_HEIGHT/3,0,TILE_SIZE*2,-3*TILE_SIZE,0,0,-1.80937408e-07,7.54065079e-06,-1.12351547e-04,6.71657797e-04,-1.39559106e-03,5.48962382e-03,5.1374757e-02,distance);
}
track_point_t small_helix_left_up_curve(float distance)
{
return banked_curve(sloped_turn_left_curve(1.5*TILE_SIZE,CLEARANCE_HEIGHT/(0.75*M_PI*TILE_SIZE),distance),0.25*M_PI);
}
track_point_t small_helix_right_up_curve(float distance)
{
return banked_curve(sloped_turn_right_curve(1.5*TILE_SIZE,CLEARANCE_HEIGHT/(0.75*M_PI*TILE_SIZE),distance),-0.25*M_PI);
}
track_point_t medium_helix_left_up_curve(float distance)
{
return banked_curve(sloped_turn_left_curve(2.5*TILE_SIZE,CLEARANCE_HEIGHT/(1.25*M_PI*TILE_SIZE),distance),0.25*M_PI);
}
track_point_t medium_helix_right_up_curve(float distance)
{
return banked_curve(sloped_turn_right_curve(2.5*TILE_SIZE,CLEARANCE_HEIGHT/(1.25*M_PI*TILE_SIZE),distance),-0.25*M_PI);
}


track_point_t barrel_roll_left_curve(float x)
{
track_point_t point;
float u=x/BARREL_ROLL_LENGTH;
float radius=7*CLEARANCE_HEIGHT/6;
point.position=vector3(-radius*sin(PI*u),radius*(1-cos(PI*u)),3*TILE_SIZE*u);
point.tangent=vector3_normalize(vector3(-radius*PI*cos(PI*u)/BARREL_ROLL_LENGTH,radius*PI*sin(PI*u)/BARREL_ROLL_LENGTH,1.0));
	if(x<1e-4||x>BARREL_ROLL_LENGTH-1e-4)point.tangent=vector3(0,0,1);
point.normal=vector3(sin(PI*u),cos(PI*u),0.0);
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}
track_point_t barrel_roll_right_curve(float x)
{
track_point_t left_point=barrel_roll_left_curve(x);
track_point_t point;
point.position=vector3(-left_point.position.x,left_point.position.y,left_point.position.z);
point.tangent=vector3(-left_point.tangent.x,left_point.tangent.y,left_point.tangent.z);
	if(x<1e-4||x>BARREL_ROLL_LENGTH-1e-4)point.tangent=vector3(0,0,1);
point.normal=vector3(-left_point.normal.x,left_point.normal.y,left_point.normal.z);
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}

track_point_t inline_twist_left_curve(float x)
{
track_point_t point;
float u=x/BARREL_ROLL_LENGTH;
float radius=CLEARANCE_HEIGHT/6;
point.position=vector3(-radius*sin(PI*u),radius*(1-cos(PI*u)),3*TILE_SIZE*u);
point.tangent=vector3_normalize(vector3(-radius*PI*cos(PI*u)/BARREL_ROLL_LENGTH,radius*PI*sin(PI*u)/BARREL_ROLL_LENGTH,1.0));
	if(x<1e-4||x>BARREL_ROLL_LENGTH-1e-4)point.tangent=vector3(0,0,1);
point.normal=vector3(sin(PI*u),cos(PI*u),0.0);
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}
track_point_t inline_twist_right_curve(float x)
{
track_point_t left_point=inline_twist_left_curve(x);
track_point_t point;
point.position=vector3(-left_point.position.x,left_point.position.y,left_point.position.z);
point.tangent=vector3(-left_point.tangent.x,left_point.tangent.y,left_point.tangent.z);
	if(x<1e-4||x>BARREL_ROLL_LENGTH-1e-4)point.tangent=vector3(0,0,1);
point.normal=vector3(-left_point.normal.x,left_point.normal.y,left_point.normal.z);
point.binormal=vector3_cross(point.tangent,point.normal);
return point;
}

track_point_t half_loop_curve(float distance)
{
	if(distance<HALF_LOOP_SEGMENT1_LENGTH)return plane_curve_vertical(vector3(0.0,CLEARANCE_HEIGHT*(distance/HALF_LOOP_SEGMENT1_LENGTH),0.5*TILE_SIZE*(distance/HALF_LOOP_SEGMENT1_LENGTH)),vector3_normalize(vector3(0.0,2*CLEARANCE_HEIGHT/TILE_SIZE,1.0)));
	//else if(distance<HALF_LOOP_SEGMENT2_LENGTH)return cubic_curve_vertical(4*TILE_SIZE-8,12-8.5*TILE_SIZE,5*TILE_SIZE,0.5*TILE_SIZE,-7,6.75,7.5,0.75,distance-HALF_LOOP_SEGMENT1_LENGTH);
	else if(distance<HALF_LOOP_SEGMENT2_LENGTH)return cubic_curve_vertical(3*TILE_SIZE-32*CLEARANCE_HEIGHT/3,16*CLEARANCE_HEIGHT-6.5*TILE_SIZE,4*TILE_SIZE,0.5*TILE_SIZE,-14*CLEARANCE_HEIGHT/3,19*CLEARANCE_HEIGHT/3,8*CLEARANCE_HEIGHT,CLEARANCE_HEIGHT,2.60735963e-07,-7.42305927e-06,8.47657813e-05,-4.81686166e-04,1.50741670e-03,3.64826748e-04,6.3993545e-02,distance-HALF_LOOP_SEGMENT1_LENGTH);
	else return cubic_curve_vertical(0,-16*CLEARANCE_HEIGHT/3,0,16*CLEARANCE_HEIGHT/3+TILE_SIZE,-8*CLEARANCE_HEIGHT/3,-4*CLEARANCE_HEIGHT/3,32*CLEARANCE_HEIGHT/3,32*CLEARANCE_HEIGHT/3,4.98545231e-07,-8.90813710e-06,4.65444884e-05,-1.03595298e-04,3.16464106e-04,1.97218182e-03,1.24963548e-01,distance-HALF_LOOP_SEGMENT2_LENGTH);
}

track_point_t flat_to_steep_up_curve(float distance)
{
//return cubic_curve_vertical(-0.5*TILE_SIZE,-0.5*TILE_SIZE,5*TILE_SIZE,0,-2*CLEARANCE_HEIGHT,13*CLEARANCE_HEIGHT,0,0,9.15921042e-12,-7.64176033e-10,1.91775506e-08,-6.20557903e-08,-1.07682192e-05,2.95991517e-04,5.44333126e-02,distance);
float o=0;//-1.25*TILE_SIZE/32;
float v=0;//-0.75*TILE_SIZE/32;
return cubic_curve_vertical(-0.5*TILE_SIZE-2*o,-0.5*TILE_SIZE+3*o,5*TILE_SIZE,0,-2*CLEARANCE_HEIGHT-2*v,13*CLEARANCE_HEIGHT+3*v,0,0,9.15921042e-12,-7.64176033e-10,1.91775506e-08,-6.20557903e-08,-1.07682192e-05,2.95991517e-04,5.44333126e-02,distance);
}

track_point_t heartline_curve(track_point_t unbanked_curve,float angle)
{
track_point_t point;
point.position=vector3_add(unbanked_curve.position,vector3_add(vector3_mult(unbanked_curve.normal,0.125*(1-cos(angle))),vector3_mult(unbanked_curve.binormal,-0.125*sin(angle))));
point.tangent=unbanked_curve.tangent;
point.normal=vector3_add(vector3_mult(unbanked_curve.normal,cos(angle)),vector3_mult(unbanked_curve.binormal,sin(angle)));
point.binormal=vector3_add(vector3_mult(unbanked_curve.normal,-sin(angle)),vector3_mult(unbanked_curve.binormal,cos(angle)));
return point;
}


track_point_t steep_to_flat_up_curve(float distance)
{
float o=0;//-1*TILE_SIZE/32;
float v=0;//-1.25*TILE_SIZE/32;
return cubic_curve_vertical(-0.5*TILE_SIZE+2*o,2*TILE_SIZE-3*o,2.5*TILE_SIZE,0+o,-2*CLEARANCE_HEIGHT+2*v,-7*CLEARANCE_HEIGHT-3*v,20*CLEARANCE_HEIGHT,0+v,9.15921034e-12,-3.64783573e-10,-1.92055403e-09,1.77492141e-07,-8.30160814e-06,1.17635683e-04,5.68534428e-02,distance);
}

track_point_t quarter_loop_up_curve(float distance)
{
return cubic_curve_vertical(5*TILE_SIZE-64*CLEARANCE_HEIGHT/3,-7.5*TILE_SIZE+64*CLEARANCE_HEIGHT/3,0,0,-22*CLEARANCE_HEIGHT/3,CLEARANCE_HEIGHT/3.0,64*CLEARANCE_HEIGHT/3.0,0,7.18882561e-10,-3.95603532e-08,6.77429101e-07,-4.86060220e-06,3.29621469e-05,-1.32508457e-04,6.25520141e-02,distance);
}

track_point_t bezier3d(float xa,float xb,float xc,float xd,float ya,float yb,float yc,float yd,float za,float zb,float zc,float zd,float ra,float rb,float rc,float rd,float pa,float pb,float pc,float pd,float pe,float pf,float pg,float distance)
{
float u=reparameterize(pa,pb,pc,pd,pe,pf,pg,distance);
vector3_t point=vector3(cubic(xa,xb,xc,xd,u),cubic(ya,yb,yc,yd,u),cubic(za,zb,zc,zd,u));
vector3_t tangent=vector3_normalize(vector3(cubic_derivative(xa,xb,xc,u),cubic_derivative(ya,yb,yc,u),cubic_derivative(za,zb,zc,u)));
vector3_t second_derivative=vector3(cubic_second_derivative(xa,xb,u),cubic_second_derivative(ya,yb,u),cubic_second_derivative(za,zb,u));
vector3_t normal=vector3_normalize(vector3_sub(second_derivative,vector3_mult(tangent,vector3_dot(tangent,second_derivative))));
vector3_t binormal=vector3_cross(normal,tangent);

track_point_t track_point;
float angle=cubic(ra,rb,rc,rd,u);
track_point.position=point;
track_point.tangent=tangent;
track_point.normal=vector3_add(vector3_mult(normal,cos(angle)),vector3_mult(binormal,sin(angle)));
track_point.binormal=vector3_add(vector3_mult(normal,sin(angle)),vector3_mult(binormal,-cos(angle)));
return track_point;
}

track_point_t corkscrew_left_curve(float distance)
{

	if(distance<CORKSCREW_SEGMENT1_LENGTH)return bezier3d(1.030829,-0.535829,0.000000,0.000000,-1.571756,4.378463,0.000000,0.000000,0.535829,-3.505829,7.425000,0.000000,0.104345,-0.906517,0.500000,0.121773,8.82243634e-08,2.23254996e-06,-4.18114465e-05,6.87173643e-05,1.08563628e-04,8.30764584e-03,1.44263227e-01,distance);
	else return bezier3d(0.535829,1.898342,2.020829,0.495000,-1.571756,0.336805,4.041658,2.806707,1.030829,-2.556658,2.020829,4.455000,0.729345,-0.031517,-1.000000,0.180399,-8.84322683e-07,1.94659293e-05,-1.48815075e-04,4.72284342e-04,-1.52382973e-03,4.02066359e-03,1.83543852e-01,distance-CORKSCREW_SEGMENT1_LENGTH); 
}

track_point_t corkscrew_right_curve(float distance)
{
track_point_t point=corkscrew_left_curve(distance);
point.position.x*=-1;
point.normal.x*=-1;
point.tangent.x*=-1;
point.binormal.y*=-1;
point.binormal.z*=-1;
return point;
}

track_point_t large_corkscrew_left_curve(float distance)
{
	if(distance<LARGE_CORKSCREW_SEGMENT1_LENGTH)return bezier3d(2.040694,-1.216024,0.000000,0.000000,-1.701532,6.242332,0.000000,0.000000,0.349774,-4.431544,11.503800,0.000000,-0.390790,0.873685,-0.825000,0.192393,-2.02638642e-09,1.19359406e-07,-1.71851966e-06,-5.19115702e-06,3.84473255e-05,2.91093716e-03,8.69288160e-02,distance);
	else return bezier3d(1.164874,2.570422,3.690034,0.824670,-2.848476,0.582681,7.380068,4.540800,2.034094,-4.896158,3.690034,7.422030,1.443125,-1.039687,-0.750000,0.196234,-1.75744125e-08,7.57741959e-07,-1.14090904e-05,7.19901377e-05,-3.22829928e-04,8.18802544e-04,1.10559344e-01,distance-LARGE_CORKSCREW_SEGMENT1_LENGTH); 
}

track_point_t large_corkscrew_right_curve(float distance)
{
track_point_t point=large_corkscrew_left_curve(distance);
point.position.x*=-1;
point.normal.x*=-1;
point.tangent.x*=-1;
point.binormal.y*=-1;
point.binormal.z*=-1;
return point;
}


track_point_t large_half_loop_left_curve(float distance)
{
track_point_t point;

float proj_distance=distance/LARGE_HALF_LOOP_FACTOR;

	if(proj_distance<LARGE_HALF_LOOP_SEGMENT1_LENGTH)
	{
	float u=proj_distance/GENTLE_LENGTH;
	point.position=vector3(TILE_SIZE*proj_distance/LARGE_HALF_LOOP_LENGTH,2*CLEARANCE_HEIGHT*u,u*TILE_SIZE);
	point.tangent=vector3_normalize(vector3(0.0,2*CLEARANCE_HEIGHT,TILE_SIZE));
	}
	else if(distance<LARGE_HALF_LOOP_SEGMENT2_LENGTH)
	{
	float u=reparameterize(4.92552773e-08,-3.05251408e-06,7.72071092e-05,-1.03134802e-03,7.91192883e-03,-3.64962014e-02,1.60807957e-01,proj_distance-LARGE_HALF_LOOP_SEGMENT1_LENGTH);
	point.position=vector3(TILE_SIZE*proj_distance/LARGE_HALF_LOOP_LENGTH,cubic(-0.403193,10.731874,2.020829,2.020829,u),cubic(-11.880000,15.345000,4.950000,4.950000,u));
	point.tangent=vector3_normalize(vector3(0,cubic_derivative(-0.403193,10.731874,2.020829,u),cubic_derivative(-11.880000,15.345000,4.950000,u)));
	}
	else
	{
	float u=reparameterize(1.99640047e-07,-6.93285191e-06,9.60806400e-05,-6.7055459e-04,2.46312103e-03,-1.96991475e-03,5.27553949e-02,proj_distance-LARGE_HALF_LOOP_SEGMENT2_LENGTH);
	point.position=vector3(TILE_SIZE*proj_distance/LARGE_HALF_LOOP_LENGTH,cubic(3.633368,-15.350052,19.800000,14.370340,u),cubic(8.580000,-15.345000,0.000000,13.365000,u));
	point.tangent=vector3_normalize(vector3(0,cubic_derivative(3.633368,-15.350052,19.800000,u),cubic_derivative(8.580000,-15.345000,0.000000,u)));
	}

point.tangent.x+=0.1006880872852946;
point.tangent=vector3_normalize(point.tangent);

point.normal=vector3_normalize(vector3(0.0,point.tangent.z,-point.tangent.y));
point.binormal=vector3_cross(point.tangent,point.normal);

return point;
}

track_point_t large_half_loop_right_curve(float distance)
{
track_point_t point=large_half_loop_left_curve(distance);
point.position.x*=-1;
point.normal.x*=-1;
point.tangent.x*=-1;
point.binormal.y*=-1;
point.binormal.z*=-1;
return point;
}


track_point_t small_turn_left_bank_to_gentle_up_curve(float distance)
{
float radius=1.5*TILE_SIZE;
float u=reparameterize(1.20514043e-11,-1.08738105e-09,2.56295980e-08,3.90911309e-07,-2.87550893e-05,-2.67048353e-04,1.27817285e-01,distance);

float a=1.1534817918544915;
float b=0.8673472459416303;

track_point_t point;
point.position=vector3(radius*(1.0-cos(0.5*M_PI*u)),u*(a*u+b),radius*sin(0.5*M_PI*u));
point.tangent=vector3_normalize(vector3(0.5*M_PI*radius*sin(0.5*M_PI*u),(2*a*u+b),0.5*M_PI*radius*cos(0.5*M_PI*u)));
point.binormal=vector3_normalize(vector3_cross(point.tangent,vector3(0,1,0)));
point.normal=vector3_cross(point.binormal,point.tangent);
return banked_curve(point,0.25*(1-u)*M_PI);
}

track_point_t small_turn_right_bank_to_gentle_up_curve(float distance)
{
track_point_t point=small_turn_left_bank_to_gentle_up_curve(distance);
point.position.x*=-1;
point.normal.x*=-1;
point.tangent.x*=-1;
point.binormal.y*=-1;
point.binormal.z*=-1;
return point;
}



track_point_t small_flat_to_steep_up_curve(float distance)
{
return cubic_curve_vertical(2*CLEARANCE_HEIGHT-TILE_SIZE,2*TILE_SIZE-4*CLEARANCE_HEIGHT,2*CLEARANCE_HEIGHT,0,2*CLEARANCE_HEIGHT,1*CLEARANCE_HEIGHT,0,0,3.05114828e-04,-5.41080425e-03,3.92230576e-02,-1.50356228e-01,3.31677795e-01,-4.47145240e-01,5.86667713e-01,distance);
}
track_point_t small_steep_to_flat_up_curve(float distance)
{
return cubic_curve_vertical(2*CLEARANCE_HEIGHT-TILE_SIZE,TILE_SIZE-2*CLEARANCE_HEIGHT,TILE_SIZE,0,2*CLEARANCE_HEIGHT,-7*CLEARANCE_HEIGHT,8*CLEARANCE_HEIGHT,0,3.05114828e-04,-4.16244486e-03,2.24366063e-02,-5.97476791e-02,8.15009919e-02,-4.17427549e-02,1.53388477e-01,distance);
}

#define SQRT_2 1.41421356237 
track_point_t small_flat_to_steep_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(SQRT_2*(2*CLEARANCE_HEIGHT-TILE_SIZE),SQRT_2*(2*TILE_SIZE-4*CLEARANCE_HEIGHT),SQRT_2*(2*CLEARANCE_HEIGHT),0,2*CLEARANCE_HEIGHT,1*CLEARANCE_HEIGHT,0,0,4.34857856e-05,-1.00180279e-03,9.43455215e-03,-4.70409694e-02,1.35543609e-01,-2.38941648e-01,4.19828724e-01,distance);
}
track_point_t small_steep_to_flat_up_diag_curve(float distance)
{
return cubic_curve_vertical_diagonal(SQRT_2*(2*CLEARANCE_HEIGHT-TILE_SIZE),SQRT_2*(TILE_SIZE-2*CLEARANCE_HEIGHT),SQRT_2*TILE_SIZE,0,2*CLEARANCE_HEIGHT,-7*CLEARANCE_HEIGHT,8*CLEARANCE_HEIGHT,0,4.34857856e-05,-7.68536624e-04,5.36464795e-03,-1.84338486e-02,3.22283971e-02,-2.27275623e-02,1.33593778e-01,distance);
}



rect_t large_half_loop_left_rects[]={
//First angle
{INT32_MIN,INT32_MIN,0,-41},{INT32_MIN,-41,1,-40},{INT32_MIN,-40,2,-39},{INT32_MIN,-39,3,-38},{INT32_MIN,-38,4,-37},{INT32_MIN,-37,5,-36},{INT32_MIN,-36,6,-35},{INT32_MIN,-35,7,-34},{INT32_MIN,-34,8,-33},{INT32_MIN,-33,9,-32},{INT32_MIN,-32,10,-31},{INT32_MIN,-31,11,-30},{INT32_MIN,-30,12,-29},{INT32_MIN,-29,13,-28},{INT32_MIN,-28,14,-27},{INT32_MIN,-27,15,-26},{INT32_MIN,-26,16,-25},{INT32_MIN,-25,17,-24},{INT32_MIN,-24,18,-23},{INT32_MIN,-23,19,-22},{INT32_MIN,-22,20,-21},{INT32_MIN,-21,21,-20},{INT32_MIN,-20,22,-19},{INT32_MIN,-19,23,-18},{INT32_MIN,-18,24,-17},{INT32_MIN,-17,25,-16},{INT32_MIN,-16,26,-15},{INT32_MIN,-15,27,-14},{INT32_MIN,-14,28,-13},{INT32_MIN,-13,29,-12},{INT32_MIN,-12,30,-11},{INT32_MIN,-11,31,-10},{INT32_MIN,-10,32,INT32_MAX},
{0,-204,32,-83},{0,-83,33,-82},{0,-82,34,-81},{0,-81,36,-80},{0,-80,37,-79},{0,-79,38,-78},{0,-78,40,-77},{0,-77,41,-76},{0,-76,42,-75},{0,-75,43,-74},{0,-74,45,-73},{0,-73,46,-72},{0,-72,47,-71},{0,-71,49,-70},{0,-70,50,-69},{0,-69,51,-68},{0,-68,52,-67},{0,-67,54,-66},{0,-66,55,-65},{0,-65,56,-64},{0,-64,58,-63},{0,-63,59,-62},{0,-62,60,-61},{0,-61,61,-59},{0,-59,63,-58},{0,-58,64,-41},{1,-41,64,-40},{2,-40,64,-39},{3,-39,64,-38},{4,-38,64,-37},{5,-37,64,-36},{6,-36,64,-35},{7,-35,64,-34},{8,-34,64,-33},{9,-33,64,-32},{10,-32,64,-31},{11,-31,64,-30},{12,-30,64,-29},{13,-29,64,-28},{14,-28,64,-27},{15,-27,64,-26},{16,-26,64,-25},{17,-25,64,-24},{18,-24,64,-23},{19,-23,64,-22},{20,-22,64,-21},{21,-21,64,-20},{22,-20,64,-19},{23,-19,64,-18},{24,-18,64,-17},{25,-17,64,-16},{26,-16,64,-15},{27,-15,64,-14},{28,-14,64,-13},{29,-13,64,-12},{30,-12,64,-11},{31,-11,64,-10},{32,-10,64,INT32_MAX},
{32,-204,64,-137},{32,-137,65,-136},{32,-136,67,-135},{32,-135,69,-134},{32,-134,71,-133},{32,-133,73,-132},{32,-132,75,-131},{32,-131,77,-130},{32,-130,79,-129},{32,-129,81,-128},{32,-128,83,-127},{32,-127,85,-126},{32,-126,87,-125},{32,-125,89,-124},{32,-124,91,-123},{32,-123,93,-122},{32,-122,95,-121},{32,-121,96,-83},{33,-83,96,-82},{34,-82,96,-81},{36,-81,96,-80},{37,-80,96,-79},{38,-79,96,-78},{40,-78,96,-77},{41,-77,96,-76},{42,-76,96,-75},{43,-75,96,-74},{45,-74,96,-73},{46,-73,96,-72},{47,-72,96,-71},{49,-71,96,-70},{50,-70,96,-69},{51,-69,96,-68},{52,-68,96,-67},{54,-67,96,-66},{55,-66,96,-65},{56,-65,96,-64},{58,-64,96,-63},{59,-63,96,-62},{60,-62,96,-61},{61,-61,96,-59},{63,-59,96,-58},{64,-58,96,INT32_MAX},
{64,-204,67,-203},{64,-203,70,-202},{64,-202,72,-201},{64,-201,75,-200},{64,-200,78,-199},{64,-199,80,-198},{64,-198,83,-197},{64,-197,85,-196},{64,-196,88,-195},{64,-195,91,-194},{64,-194,94,-193},{64,-193,96,-192},{64,-192,99,-191},{64,-191,102,-190},{64,-190,104,-189},{64,-189,107,-188},{64,-188,110,-187},{64,-187,112,-186},{64,-186,115,-185},{64,-185,118,-184},{64,-184,120,-183},{64,-183,123,-182},{64,-182,126,-181},{64,-181,128,-180},{64,-180,INT32_MAX,-137},{65,-137,INT32_MAX,-136},{67,-136,INT32_MAX,-135},{69,-135,INT32_MAX,-134},{71,-134,INT32_MAX,-133},{73,-133,INT32_MAX,-132},{75,-132,INT32_MAX,-131},{77,-131,INT32_MAX,-130},{79,-130,INT32_MAX,-129},{81,-129,INT32_MAX,-128},{83,-128,INT32_MAX,-127},{85,-127,INT32_MAX,-126},{87,-126,INT32_MAX,-125},{89,-125,INT32_MAX,-124},{91,-124,INT32_MAX,-123},{93,-123,INT32_MAX,-122},{95,-122,INT32_MAX,-121},{96,-121,INT32_MAX,INT32_MAX},
{96,INT32_MIN,INT32_MAX,-269},{92,-269,INT32_MAX,-268},{85,-268,INT32_MAX,-267},{78,-267,INT32_MAX,-266},{71,-266,INT32_MAX,-265},{64,-265,INT32_MAX,-204},{67,-204,INT32_MAX,-203},{70,-203,INT32_MAX,-202},{72,-202,INT32_MAX,-201},{75,-201,INT32_MAX,-200},{78,-200,INT32_MAX,-199},{80,-199,INT32_MAX,-198},{83,-198,INT32_MAX,-197},{85,-197,INT32_MAX,-196},{88,-196,INT32_MAX,-195},{91,-195,INT32_MAX,-194},{94,-194,INT32_MAX,-193},{96,-193,INT32_MAX,-192},{99,-192,INT32_MAX,-191},{102,-191,INT32_MAX,-190},{104,-190,INT32_MAX,-189},{107,-189,INT32_MAX,-188},{110,-188,INT32_MAX,-187},{112,-187,INT32_MAX,-186},{115,-186,INT32_MAX,-185},{118,-185,INT32_MAX,-184},{120,-184,INT32_MAX,-183},{123,-183,INT32_MAX,-182},{126,-182,INT32_MAX,-181},{128,-181,INT32_MAX,-180},
{95,-359,96,-358},{94,-358,96,-356},{93,-356,96,-355},{92,-355,96,-353},{91,-353,96,-352},{90,-352,96,-350},{89,-350,96,-349},{88,-349,96,-347},{87,-347,96,-346},{86,-346,96,-344},{85,-344,96,-343},{84,-343,96,-341},{83,-341,96,-340},{82,-340,96,-338},{81,-338,96,-337},{80,-337,96,-335},{79,-335,96,-334},{78,-334,96,-332},{77,-332,96,-331},{76,-331,96,-329},{75,-329,96,-328},{74,-328,96,-326},{73,-326,96,-325},{72,-325,96,-323},{71,-323,96,-322},{70,-322,96,-320},{69,-320,96,-319},{68,-319,96,-317},{67,-317,96,-316},{66,-316,96,-314},{65,-314,96,-313},{64,-313,96,-311},{63,-311,96,-309},{62,-309,96,-308},{61,-308,96,-306},{60,-306,96,-305},{59,-305,96,-303},{58,-303,96,-302},{57,-302,96,-300},{56,-300,96,-299},{55,-299,96,-297},{54,-297,96,-296},{53,-296,96,-294},{52,-294,96,-293},{51,-293,96,-291},{50,-291,96,-290},{49,-290,96,-288},{48,-288,96,-287},{47,-287,96,-285},{46,-285,96,-284},{45,-284,96,-282},{44,-282,96,-281},{43,-281,96,-279},{42,-279,96,-278},{41,-278,96,-276},{40,-276,96,-275},{39,-275,96,-273},{38,-273,96,-272},{37,-272,96,-270},{36,-270,96,-269},{35,-269,92,-268},{35,-268,85,-267},{34,-267,78,-266},{33,-266,71,-265},{33,-265,64,-264},{32,-264,64,-204},
{0,INT32_MIN,96,-359},{0,-359,95,-358},{0,-358,94,-356},{0,-356,93,-355},{0,-355,92,-353},{0,-353,91,-352},{0,-352,90,-350},{0,-350,89,-349},{0,-349,88,-347},{0,-347,87,-346},{0,-346,86,-344},{0,-344,85,-343},{0,-343,84,-341},{0,-341,83,-340},{0,-340,82,-338},{0,-338,81,-337},{0,-337,80,-335},{0,-335,79,-334},{0,-334,78,-332},{0,-332,77,-331},{0,-331,76,-329},{0,-329,75,-328},{0,-328,74,-326},{0,-326,73,-325},{0,-325,72,-323},{0,-323,71,-322},{0,-322,70,-320},{0,-320,69,-319},{0,-319,68,-317},{0,-317,67,-316},{0,-316,66,-314},{0,-314,65,-313},{0,-313,64,-311},{0,-311,63,-309},{0,-309,62,-308},{0,-308,61,-306},{0,-306,60,-305},{0,-305,59,-303},{0,-303,58,-302},{0,-302,57,-300},{0,-300,56,-299},{0,-299,55,-297},{0,-297,54,-296},{0,-296,53,-294},{0,-294,52,-293},{0,-293,51,-291},{0,-291,50,-290},{0,-290,49,-288},{0,-288,48,-287},{0,-287,47,-285},{0,-285,46,-284},{0,-284,45,-282},{0,-282,44,-281},{0,-281,43,-279},{0,-279,42,-278},{0,-278,41,-276},{0,-276,40,-275},{0,-275,39,-273},{0,-273,38,-272},{0,-272,37,-270},{0,-270,36,-269},{0,-269,35,-267},{0,-267,34,-266},{0,-266,33,-264},{0,-264,32,-204},
//Second angle
{INT32_MIN,INT32_MIN,26,INT32_MAX},
{26,-65,27,-63},{26,-63,28,-61},{26,-61,29,-59},{26,-59,30,-57},{26,-57,31,-55},{26,-55,32,-53},{26,-53,33,-51},{26,-51,34,-49},{26,-49,35,-47},{26,-47,36,-45},{26,-45,37,-43},{26,-43,38,-41},{26,-41,39,-39},{26,-39,40,-37},{26,-37,41,-35},{26,-35,42,-33},{26,-33,43,-31},{26,-31,44,-29},{26,-29,45,-27},{26,-27,46,-25},{26,-25,47,-23},{26,-23,48,-21},{26,-21,49,-19},{26,-19,50,-17},{26,-17,51,-15},{26,-15,52,-13},{26,-13,53,-11},{26,-11,54,-9},{26,-9,55,-7},{26,-7,56,-5},{26,-5,57,-3},{26,-3,58,-1},{26,-1,59,1},{26,1,60,3},{26,3,61,5},{26,5,62,7},{26,7,63,9},{26,9,64,11},{26,11,65,13},{26,13,66,15},{26,15,67,17},{26,17,68,19},{26,19,69,21},{26,21,70,23},{26,23,71,25},{26,25,72,27},{26,27,73,29},{26,29,74,31},{26,31,75,33},{26,33,76,35},{26,35,77,37},{26,37,78,39},{26,39,79,41},{26,41,80,43},{26,43,81,45},{26,45,82,47},{26,47,83,49},{26,49,84,51},{26,51,85,53},{26,53,86,55},{26,55,87,57},{26,57,88,59},{26,59,89,61},{26,61,90,63},{26,63,91,65},{26,65,92,67},{26,67,93,INT32_MAX},
{26,INT32_MIN,64,-65},{27,-65,64,-63},{28,-63,64,-61},{29,-61,64,-59},{30,-59,64,-57},{31,-57,64,-55},{32,-55,64,-53},{33,-53,64,-51},{34,-51,64,-49},{35,-49,65,-48},{35,-48,66,-47},{36,-47,67,-46},{36,-46,68,-45},{37,-45,69,-44},{37,-44,70,-43},{38,-43,71,-42},{38,-42,72,-41},{39,-41,73,-40},{39,-40,74,-39},{40,-39,75,-38},{40,-38,76,-37},{41,-37,77,-36},{41,-36,78,-35},{42,-35,79,-34},{42,-34,80,-33},{43,-33,81,-32},{43,-32,82,-31},{44,-31,83,-30},{44,-30,84,-29},{45,-29,85,-28},{45,-28,86,-27},{46,-27,87,-26},{46,-26,88,-25},{47,-25,89,-24},{47,-24,90,-23},{48,-23,91,-22},{48,-22,92,-21},{49,-21,93,-19},{50,-19,93,-17},{51,-17,93,-15},{52,-15,93,-13},{53,-13,93,-11},{54,-11,93,-9},{55,-9,93,-7},{56,-7,93,-5},{57,-5,93,-3},{58,-3,93,-1},{59,-1,93,1},{60,1,93,3},{61,3,93,5},{62,5,93,7},{63,7,93,9},{64,9,93,11},{65,11,93,13},{66,13,93,15},{67,15,93,17},{68,17,93,19},{69,19,93,21},{70,21,93,23},{71,23,93,25},{72,25,93,27},{73,27,93,29},{74,29,93,31},{75,31,93,33},{76,33,93,35},{77,35,93,37},{78,37,93,39},{79,39,93,41},{80,41,93,43},{81,43,93,45},{82,45,93,47},{83,47,93,49},{84,49,93,51},{85,51,93,53},{86,53,93,55},{87,55,93,57},{88,57,93,59},{89,59,93,61},{90,61,93,63},{91,63,93,65},{92,65,93,67},
{64,-133,128,-49},{65,-49,128,-48},{66,-48,128,-47},{67,-47,128,-46},{68,-46,128,-45},{69,-45,128,-44},{70,-44,128,-43},{71,-43,128,-42},{72,-42,128,-41},{73,-41,128,-40},{74,-40,128,-39},{75,-39,128,-38},{76,-38,128,-37},{77,-37,128,-36},{78,-36,128,-35},{79,-35,128,-34},{80,-34,128,-33},{81,-33,128,-32},{82,-32,128,-31},{83,-31,128,-30},{84,-30,128,-29},{85,-29,128,-28},{86,-28,128,-27},{87,-27,128,-26},{88,-26,128,-25},{89,-25,128,-24},{90,-24,128,-23},{91,-23,128,-22},{92,-22,128,-21},{93,-21,128,INT32_MAX},
{128,INT32_MIN,INT32_MAX,INT32_MAX},
{127,-243,128,-242},{126,-242,128,-241},{125,-241,128,-240},{124,-240,128,-239},{123,-239,128,-238},{122,-238,128,-237},{121,-237,128,-236},{120,-236,128,-235},{119,-235,128,-234},{118,-234,128,-233},{117,-233,128,-232},{116,-232,128,-231},{115,-231,128,-230},{114,-230,128,-229},{113,-229,128,-228},{112,-228,128,-227},{111,-227,128,-226},{110,-226,128,-225},{109,-225,128,-224},{108,-224,128,-223},{107,-223,128,-222},{106,-222,128,-221},{105,-221,128,-220},{104,-220,128,-219},{103,-219,128,-218},{102,-218,128,-217},{101,-217,128,-216},{100,-216,128,-215},{99,-215,128,-214},{98,-214,128,-213},{97,-213,128,-212},{96,-212,128,-211},{95,-211,128,-210},{94,-210,128,-209},{93,-209,128,-208},{92,-208,128,-207},{91,-207,128,-206},{90,-206,128,-205},{89,-205,128,-204},{88,-204,128,-203},{87,-203,128,-202},{86,-202,128,-201},{85,-201,128,-200},{84,-200,128,-199},{83,-199,128,-198},{82,-198,128,-197},{81,-197,128,-196},{80,-196,128,-195},{79,-195,128,-194},{78,-194,128,-193},{77,-193,128,-192},{76,-192,128,-191},{75,-191,128,-190},{74,-190,128,-189},{73,-189,128,-188},{72,-188,128,-187},{71,-187,128,-186},{70,-186,128,-185},{69,-185,128,-184},{68,-184,128,-183},{67,-183,128,-182},{66,-182,128,-181},{65,-181,128,-180},{64,-180,128,-133},
{64,INT32_MIN,128,-243},{64,-243,127,-242},{64,-242,126,-241},{64,-241,125,-240},{64,-240,124,-239},{64,-239,123,-238},{64,-238,122,-237},{64,-237,121,-236},{64,-236,120,-235},{64,-235,119,-234},{64,-234,118,-233},{64,-233,117,-232},{64,-232,116,-231},{64,-231,115,-230},{64,-230,114,-229},{64,-229,113,-228},{64,-228,112,-227},{64,-227,111,-226},{64,-226,110,-225},{64,-225,109,-224},{64,-224,108,-223},{64,-223,107,-222},{64,-222,106,-221},{64,-221,105,-220},{64,-220,104,-219},{64,-219,103,-218},{64,-218,102,-217},{64,-217,101,-216},{64,-216,100,-215},{64,-215,99,-214},{64,-214,98,-213},{64,-213,97,-212},{64,-212,96,-211},{64,-211,95,-210},{64,-210,94,-209},{64,-209,93,-208},{64,-208,92,-207},{64,-207,91,-206},{64,-206,90,-205},{64,-205,89,-204},{64,-204,88,-203},{64,-203,87,-202},{64,-202,86,-201},{64,-201,85,-200},{64,-200,84,-199},{64,-199,83,-198},{64,-198,82,-197},{64,-197,81,-196},{64,-196,80,-195},{64,-195,79,-194},{64,-194,78,-193},{64,-193,77,-192},{64,-192,76,-191},{64,-191,75,-190},{64,-190,74,-189},{64,-189,73,-188},{64,-188,72,-187},{64,-187,71,-186},{64,-186,70,-185},{64,-185,69,-184},{64,-184,68,-183},{64,-183,67,-182},{64,-182,66,-181},{64,-181,65,-180},
//Third angle
{-32,-113,INT32_MAX,-23},{-31,-23,INT32_MAX,-20},{-30,-20,INT32_MAX,-17},{-29,-17,INT32_MAX,-14},{-28,-14,INT32_MAX,-11},{-27,-11,INT32_MAX,-8},{-26,-8,INT32_MAX,-5},{-25,-5,INT32_MAX,-2},{-24,-2,INT32_MAX,1},{-23,1,INT32_MAX,4},{-22,4,INT32_MAX,7},{-21,7,INT32_MAX,10},{-20,10,INT32_MAX,13},{-19,13,INT32_MAX,16},{-18,16,INT32_MAX,19},{-17,19,INT32_MAX,22},{-16,22,INT32_MAX,25},{-15,25,INT32_MAX,28},{-14,28,INT32_MAX,31},{-13,31,INT32_MAX,34},{-12,34,INT32_MAX,37},{-11,37,INT32_MAX,40},{-10,40,INT32_MAX,43},{-9,43,INT32_MAX,46},{-8,46,INT32_MAX,49},{-7,49,INT32_MAX,52},{-6,52,INT32_MAX,55},{-5,55,INT32_MAX,58},{-4,58,INT32_MAX,61},{-3,61,INT32_MAX,64},{-2,64,INT32_MAX,67},{-1,67,INT32_MAX,70},{0,70,INT32_MAX,INT32_MAX},
{-33,-60,-32,-58},{-34,-58,-32,-56},{-35,-56,-32,-54},{-36,-54,-32,-52},{-37,-52,-32,-50},{-38,-50,-32,-48},{-39,-48,-32,-46},{-40,-46,-32,-44},{-41,-44,-32,-42},{-42,-42,-32,-40},{-43,-40,-32,-38},{-44,-38,-32,-36},{-45,-36,-32,-34},{-46,-34,-32,-32},{-47,-32,-32,-30},{-48,-30,-32,-28},{-49,-28,-32,-26},{-50,-26,-32,-24},{-51,-24,-32,-23},{-51,-23,-31,-22},{-52,-22,-31,-20},{-53,-20,-30,-18},{-54,-18,-30,-17},{-54,-17,-29,-16},{-55,-16,-29,-14},{-56,-14,-28,-12},{-57,-12,-28,-11},{-57,-11,-27,-10},{-58,-10,-27,-8},{-59,-8,-26,-6},{-60,-6,-26,-5},{-60,-5,-25,-4},{-61,-4,-25,-2},{-62,-2,-24,0},{-63,0,-24,1},{-63,1,-23,2},{-64,2,-23,4},{-64,4,-22,7},{-64,7,-21,10},{-64,10,-20,13},{-64,13,-19,16},{-64,16,-18,19},{-64,19,-17,22},{-64,22,-16,25},{-64,25,-15,28},{-64,28,-14,31},{-64,31,-13,34},{-64,34,-12,37},{-64,37,-11,40},{-64,40,-10,43},{-64,43,-9,46},{-64,46,-8,49},{-64,49,-7,52},{-64,52,-6,55},{-64,55,-5,58},{-64,58,-4,61},{-64,61,-3,64},{-64,64,-2,67},{-64,67,-1,70},{-64,70,0,INT32_MAX},
{-64,-113,-32,-60},{-64,-60,-33,-58},{-64,-58,-34,-56},{-64,-56,-35,-54},{-64,-54,-36,-52},{-64,-52,-37,-50},{-64,-50,-38,-48},{-64,-48,-39,-47},{-66,-47,-39,-46},{-67,-46,-40,-45},{-69,-45,-40,-44},{-70,-44,-41,-43},{-72,-43,-41,-42},{-73,-42,-42,-41},{-75,-41,-42,-40},{-76,-40,-43,-39},{-78,-39,-43,-38},{-79,-38,-44,-37},{-81,-37,-44,-36},{-82,-36,-45,-35},{-84,-35,-45,-34},{-85,-34,-46,-33},{-87,-33,-46,-32},{-88,-32,-47,-31},{-90,-31,-47,-30},{-91,-30,-48,-29},{-93,-29,-48,-28},{-94,-28,-49,-27},{-96,-27,-49,-26},{-96,-26,-50,-24},{-96,-24,-51,-22},{-96,-22,-52,-20},{-96,-20,-53,-18},{-96,-18,-54,-16},{-96,-16,-55,-14},{-96,-14,-56,-12},{-96,-12,-57,-10},{-96,-10,-58,-8},{-96,-8,-59,-6},{-96,-6,-60,-4},{-96,-4,-61,-2},{-96,-2,-62,0},{-96,0,-63,2},{-96,2,-64,INT32_MAX},
{-69,-80,-64,-79},{-74,-79,-64,-78},{-79,-78,-64,-77},{-84,-77,-64,-76},{-89,-76,-64,-75},{-94,-75,-64,-74},{-99,-74,-64,-73},{-104,-73,-64,-72},{-109,-72,-64,-71},{-114,-71,-64,-70},{-119,-70,-64,-69},{-124,-69,-64,-68},{INT32_MIN,-68,-64,-47},{INT32_MIN,-47,-66,-46},{INT32_MIN,-46,-67,-45},{INT32_MIN,-45,-69,-44},{INT32_MIN,-44,-70,-43},{INT32_MIN,-43,-72,-42},{INT32_MIN,-42,-73,-41},{INT32_MIN,-41,-75,-40},{INT32_MIN,-40,-76,-39},{INT32_MIN,-39,-78,-38},{INT32_MIN,-38,-79,-37},{INT32_MIN,-37,-81,-36},{INT32_MIN,-36,-82,-35},{INT32_MIN,-35,-84,-34},{INT32_MIN,-34,-85,-33},{INT32_MIN,-33,-87,-32},{INT32_MIN,-32,-88,-31},{INT32_MIN,-31,-90,-30},{INT32_MIN,-30,-91,-29},{INT32_MIN,-29,-93,-28},{INT32_MIN,-28,-94,-27},{INT32_MIN,-27,-96,INT32_MAX},
{INT32_MIN,INT32_MIN,-96,-161},{INT32_MIN,-161,-95,-160},{INT32_MIN,-160,-93,-159},{INT32_MIN,-159,-91,-158},{INT32_MIN,-158,-89,-157},{INT32_MIN,-157,-87,-156},{INT32_MIN,-156,-85,-155},{INT32_MIN,-155,-83,-154},{INT32_MIN,-154,-81,-153},{INT32_MIN,-153,-79,-152},{INT32_MIN,-152,-77,-151},{INT32_MIN,-151,-75,-150},{INT32_MIN,-150,-73,-149},{INT32_MIN,-149,-71,-148},{INT32_MIN,-148,-69,-147},{INT32_MIN,-147,-67,-146},{INT32_MIN,-146,-65,-145},{INT32_MIN,-145,-64,-80},{INT32_MIN,-80,-69,-79},{INT32_MIN,-79,-74,-78},{INT32_MIN,-78,-79,-77},{INT32_MIN,-77,-84,-76},{INT32_MIN,-76,-89,-75},{INT32_MIN,-75,-94,-74},{INT32_MIN,-74,-99,-73},{INT32_MIN,-73,-104,-72},{INT32_MIN,-72,-109,-71},{INT32_MIN,-71,-114,-70},{INT32_MIN,-70,-119,-69},{INT32_MIN,-69,-124,-68},
{-96,INT32_MIN,-64,-199},{-96,-199,-63,-198},{-96,-198,-61,-197},{-96,-197,-60,-196},{-96,-196,-58,-195},{-96,-195,-57,-194},{-96,-194,-55,-193},{-96,-193,-54,-192},{-96,-192,-52,-191},{-96,-191,-51,-190},{-96,-190,-49,-189},{-96,-189,-48,-188},{-96,-188,-46,-187},{-96,-187,-45,-186},{-96,-186,-43,-185},{-96,-185,-42,-184},{-96,-184,-40,-183},{-96,-183,-39,-182},{-96,-182,-37,-181},{-96,-181,-36,-180},{-96,-180,-34,-179},{-96,-179,-33,-178},{-96,-178,-32,-161},{-95,-161,-32,-160},{-93,-160,-32,-159},{-91,-159,-32,-158},{-89,-158,-32,-157},{-87,-157,-32,-156},{-85,-156,-32,-155},{-83,-155,-32,-154},{-81,-154,-32,-153},{-79,-153,-32,-152},{-77,-152,-32,-151},{-75,-151,-32,-150},{-73,-150,-32,-149},{-71,-149,-32,-148},{-69,-148,-32,-147},{-67,-147,-32,-146},{-65,-146,-32,-145},{-64,-145,-32,-113},
{-64,INT32_MIN,INT32_MAX,-199},{-63,-199,INT32_MAX,-198},{-61,-198,INT32_MAX,-197},{-60,-197,INT32_MAX,-196},{-58,-196,INT32_MAX,-195},{-57,-195,INT32_MAX,-194},{-55,-194,INT32_MAX,-193},{-54,-193,INT32_MAX,-192},{-52,-192,INT32_MAX,-191},{-51,-191,INT32_MAX,-190},{-49,-190,INT32_MAX,-189},{-48,-189,INT32_MAX,-188},{-46,-188,INT32_MAX,-187},{-45,-187,INT32_MAX,-186},{-43,-186,INT32_MAX,-185},{-42,-185,INT32_MAX,-184},{-40,-184,INT32_MAX,-183},{-39,-183,INT32_MAX,-182},{-37,-182,INT32_MAX,-181},{-36,-181,INT32_MAX,-180},{-34,-180,INT32_MAX,-179},{-33,-179,INT32_MAX,-178},{-32,-178,INT32_MAX,-113},
//Fourth angle
{0,-195,INT32_MAX,-30},{-1,-30,INT32_MAX,-29},{-2,-29,INT32_MAX,-28},{-3,-28,INT32_MAX,-27},{-4,-27,INT32_MAX,-26},{-5,-26,INT32_MAX,-25},{-6,-25,INT32_MAX,-24},{-7,-24,INT32_MAX,-23},{-8,-23,INT32_MAX,-22},{-9,-22,INT32_MAX,-21},{-10,-21,INT32_MAX,-20},{-11,-20,INT32_MAX,-19},{-12,-19,INT32_MAX,-18},{-13,-18,INT32_MAX,-17},{-14,-17,INT32_MAX,-16},{-15,-16,INT32_MAX,-15},{-16,-15,INT32_MAX,-14},{-17,-14,INT32_MAX,-13},{-18,-13,INT32_MAX,-12},{-19,-12,INT32_MAX,-11},{-20,-11,INT32_MAX,-10},{-21,-10,INT32_MAX,-9},{-22,-9,INT32_MAX,-8},{-23,-8,INT32_MAX,-7},{-24,-7,INT32_MAX,-6},{-25,-6,INT32_MAX,-5},{-26,-5,INT32_MAX,-4},{-27,-4,INT32_MAX,-3},{-28,-3,INT32_MAX,-2},{-29,-2,INT32_MAX,-1},{-30,-1,INT32_MAX,0},{-31,0,INT32_MAX,1},{-32,1,INT32_MAX,INT32_MAX},
{-32,-195,0,-71},{-33,-71,0,-70},{-34,-70,0,-69},{-35,-69,0,-68},{-36,-68,0,-67},{-37,-67,0,-66},{-38,-66,0,-65},{-39,-65,0,-64},{-40,-64,0,-63},{-41,-63,0,-62},{-42,-62,0,-61},{-43,-61,0,-60},{-44,-60,0,-59},{-45,-59,0,-58},{-46,-58,0,-57},{-47,-57,0,-56},{-48,-56,0,-55},{-49,-55,0,-54},{-50,-54,0,-53},{-51,-53,0,-52},{-52,-52,0,-51},{-53,-51,0,-50},{-54,-50,0,-49},{-55,-49,0,-48},{-56,-48,0,-47},{-57,-47,0,-46},{-58,-46,0,-45},{-59,-45,0,-44},{-60,-44,0,-43},{-61,-43,0,-42},{-62,-42,0,-41},{-63,-41,0,-40},{-64,-40,0,-30},{-64,-30,-1,-29},{-64,-29,-2,-28},{-64,-28,-3,-27},{-64,-27,-4,-26},{-64,-26,-5,-25},{-64,-25,-6,-24},{-64,-24,-7,-23},{-64,-23,-8,-22},{-64,-22,-9,-21},{-64,-21,-10,-20},{-64,-20,-11,-19},{-64,-19,-12,-18},{-64,-18,-13,-17},{-64,-17,-14,-16},{-64,-16,-15,-15},{-64,-15,-16,-14},{-64,-14,-17,-13},{-64,-13,-18,-12},{-64,-12,-19,-11},{-64,-11,-20,-10},{-64,-10,-21,-9},{-64,-9,-22,-8},{-64,-8,-23,-7},{-64,-7,-24,-6},{-64,-6,-25,-5},{-64,-5,-26,-4},{-64,-4,-27,-3},{-64,-3,-28,-2},{-64,-2,-29,-1},{-64,-1,-30,0},{-64,0,-31,1},{-64,1,-32,INT32_MAX},
{-96,-195,-32,-71},{-96,-71,-33,-70},{-96,-70,-34,-69},{-96,-69,-35,-68},{-96,-68,-36,-67},{-96,-67,-37,-66},{-96,-66,-38,-65},{-96,-65,-39,-64},{-96,-64,-40,-63},{-96,-63,-41,-62},{-96,-62,-42,-61},{-96,-61,-43,-60},{-96,-60,-44,-59},{-96,-59,-45,-58},{-96,-58,-46,-57},{-96,-57,-47,-56},{-96,-56,-48,-55},{-96,-55,-49,-54},{-96,-54,-50,-53},{-96,-53,-51,-52},{-96,-52,-52,-51},{-96,-51,-53,-50},{-96,-50,-54,-49},{-96,-49,-55,-48},{-96,-48,-56,-47},{-96,-47,-57,-46},{-96,-46,-58,-45},{-96,-45,-59,-44},{-96,-44,-60,-43},{-96,-43,-61,-42},{-96,-42,-62,-41},{-96,-41,-63,-40},{-96,-40,-64,INT32_MAX},
{-128,-195,-96,INT32_MAX},
{INT32_MIN,-229,-128,INT32_MAX},
{INT32_MIN,INT32_MIN,-128,-288},{INT32_MIN,-288,-127,-287},{INT32_MIN,-287,-126,-286},{INT32_MIN,-286,-125,-285},{INT32_MIN,-285,-124,-284},{INT32_MIN,-284,-123,-283},{INT32_MIN,-283,-122,-282},{INT32_MIN,-282,-121,-281},{INT32_MIN,-281,-120,-280},{INT32_MIN,-280,-119,-279},{INT32_MIN,-279,-118,-278},{INT32_MIN,-278,-117,-277},{INT32_MIN,-277,-116,-276},{INT32_MIN,-276,-115,-275},{INT32_MIN,-275,-114,-274},{INT32_MIN,-274,-113,-273},{INT32_MIN,-273,-112,-272},{INT32_MIN,-272,-111,-271},{INT32_MIN,-271,-110,-270},{INT32_MIN,-270,-109,-269},{INT32_MIN,-269,-108,-268},{INT32_MIN,-268,-107,-267},{INT32_MIN,-267,-106,-266},{INT32_MIN,-266,-105,-265},{INT32_MIN,-265,-104,-264},{INT32_MIN,-264,-103,-263},{INT32_MIN,-263,-102,-262},{INT32_MIN,-262,-101,-261},{INT32_MIN,-261,-100,-260},{INT32_MIN,-260,-99,-259},{INT32_MIN,-259,-98,-258},{INT32_MIN,-258,-97,-257},{INT32_MIN,-257,-96,-229},{-128,-229,-96,-195},
{-128,INT32_MIN,INT32_MAX,-288},{-127,-288,INT32_MAX,-287},{-126,-287,INT32_MAX,-286},{-125,-286,INT32_MAX,-285},{-124,-285,INT32_MAX,-284},{-123,-284,INT32_MAX,-283},{-122,-283,INT32_MAX,-282},{-121,-282,INT32_MAX,-281},{-120,-281,INT32_MAX,-280},{-119,-280,INT32_MAX,-279},{-118,-279,INT32_MAX,-278},{-117,-278,INT32_MAX,-277},{-116,-277,INT32_MAX,-276},{-115,-276,INT32_MAX,-275},{-114,-275,INT32_MAX,-274},{-113,-274,INT32_MAX,-273},{-112,-273,INT32_MAX,-272},{-111,-272,INT32_MAX,-271},{-110,-271,INT32_MAX,-270},{-109,-270,INT32_MAX,-269},{-108,-269,INT32_MAX,-268},{-107,-268,INT32_MAX,-267},{-106,-267,INT32_MAX,-266},{-105,-266,INT32_MAX,-265},{-104,-265,INT32_MAX,-264},{-103,-264,INT32_MAX,-263},{-102,-263,INT32_MAX,-262},{-101,-262,INT32_MAX,-261},{-100,-261,INT32_MAX,-260},{-99,-260,INT32_MAX,-259},{-98,-259,INT32_MAX,-258},{-97,-258,INT32_MAX,-257},{-96,-257,INT32_MAX,-195},
};
mask_t large_half_loop_left_masks[]={
{0,33,0,0,large_half_loop_left_rects},{0,58,-32,32,large_half_loop_left_rects+33},{0,43,-64,64,large_half_loop_left_rects+91},{0,42,-96,112,large_half_loop_left_rects+134},{0,30,-96,200,large_half_loop_left_rects+176},{0,66,-64,128,large_half_loop_left_rects+206},{0,65,-32,296,large_half_loop_left_rects+272},
{0,1,0,0,large_half_loop_left_rects+337},{0,67,-32,0,large_half_loop_left_rects+338},{0,81,-64,0,large_half_loop_left_rects+405},{0,30,-96,16,large_half_loop_left_rects+486},{0,1,-160,72,large_half_loop_left_rects+516},{0,64,-128,32,large_half_loop_left_rects+517},{0,64,-96,232,large_half_loop_left_rects+581},
{0,33,0,0,large_half_loop_left_rects+645},{0,60,32,0,large_half_loop_left_rects+678},{0,44,64,0,large_half_loop_left_rects+738},{0,34,96,16,large_half_loop_left_rects+782},{0,30,96,40,large_half_loop_left_rects+816},{0,40,64,0,large_half_loop_left_rects+846},{0,23,32,200,large_half_loop_left_rects+886},
{0,33,0,0,large_half_loop_left_rects+909},{0,65,32,32,large_half_loop_left_rects+942},{0,33,64,64,large_half_loop_left_rects+1007},{0,1,96,112,large_half_loop_left_rects+1040},{0,1,160,168,large_half_loop_left_rects+1041},{0,34,128,96,large_half_loop_left_rects+1042},{0,33,96,264,large_half_loop_left_rects+1076},
};

const track_section_t large_half_loop_left={TRACK_OFFSET_SPRITE_MASK,large_half_loop_left_curve,LARGE_HALF_LOOP_LENGTH,{{0,7,large_half_loop_left_masks},{0,7,large_half_loop_left_masks+7},{0,7,large_half_loop_left_masks+14},{0,7,large_half_loop_left_masks+21}}};



rect_t large_half_loop_right_rects[]={
//First angle
{INT32_MIN,-195,0,-30},{INT32_MIN,-30,1,-29},{INT32_MIN,-29,2,-28},{INT32_MIN,-28,3,-27},{INT32_MIN,-27,4,-26},{INT32_MIN,-26,5,-25},{INT32_MIN,-25,6,-24},{INT32_MIN,-24,7,-23},{INT32_MIN,-23,8,-22},{INT32_MIN,-22,9,-21},{INT32_MIN,-21,10,-20},{INT32_MIN,-20,11,-19},{INT32_MIN,-19,12,-18},{INT32_MIN,-18,13,-17},{INT32_MIN,-17,14,-16},{INT32_MIN,-16,15,-15},{INT32_MIN,-15,16,-14},{INT32_MIN,-14,17,-13},{INT32_MIN,-13,18,-12},{INT32_MIN,-12,19,-11},{INT32_MIN,-11,20,-10},{INT32_MIN,-10,21,-9},{INT32_MIN,-9,22,-8},{INT32_MIN,-8,23,-7},{INT32_MIN,-7,24,-6},{INT32_MIN,-6,25,-5},{INT32_MIN,-5,26,-4},{INT32_MIN,-4,27,-3},{INT32_MIN,-3,28,-2},{INT32_MIN,-2,29,-1},{INT32_MIN,-1,30,0},{INT32_MIN,0,31,1},{INT32_MIN,1,32,INT32_MAX},
{0,-195,32,-71},{0,-71,33,-70},{0,-70,34,-69},{0,-69,35,-68},{0,-68,36,-67},{0,-67,37,-66},{0,-66,38,-65},{0,-65,39,-64},{0,-64,40,-63},{0,-63,41,-62},{0,-62,42,-61},{0,-61,43,-60},{0,-60,44,-59},{0,-59,45,-58},{0,-58,46,-57},{0,-57,47,-56},{0,-56,48,-55},{0,-55,49,-54},{0,-54,50,-53},{0,-53,51,-52},{0,-52,52,-51},{0,-51,53,-50},{0,-50,54,-49},{0,-49,55,-48},{0,-48,56,-47},{0,-47,57,-46},{0,-46,58,-45},{0,-45,59,-44},{0,-44,60,-43},{0,-43,61,-42},{0,-42,62,-41},{0,-41,63,-40},{0,-40,64,-30},{1,-30,64,-29},{2,-29,64,-28},{3,-28,64,-27},{4,-27,64,-26},{5,-26,64,-25},{6,-25,64,-24},{7,-24,64,-23},{8,-23,64,-22},{9,-22,64,-21},{10,-21,64,-20},{11,-20,64,-19},{12,-19,64,-18},{13,-18,64,-17},{14,-17,64,-16},{15,-16,64,-15},{16,-15,64,-14},{17,-14,64,-13},{18,-13,64,-12},{19,-12,64,-11},{20,-11,64,-10},{21,-10,64,-9},{22,-9,64,-8},{23,-8,64,-7},{24,-7,64,-6},{25,-6,64,-5},{26,-5,64,-4},{27,-4,64,-3},{28,-3,64,-2},{29,-2,64,-1},{30,-1,64,0},{31,0,64,1},{32,1,64,INT32_MAX},
{32,-195,96,-71},{33,-71,96,-70},{34,-70,96,-69},{35,-69,96,-68},{36,-68,96,-67},{37,-67,96,-66},{38,-66,96,-65},{39,-65,96,-64},{40,-64,96,-63},{41,-63,96,-62},{42,-62,96,-61},{43,-61,96,-60},{44,-60,96,-59},{45,-59,96,-58},{46,-58,96,-57},{47,-57,96,-56},{48,-56,96,-55},{49,-55,96,-54},{50,-54,96,-53},{51,-53,96,-52},{52,-52,96,-51},{53,-51,96,-50},{54,-50,96,-49},{55,-49,96,-48},{56,-48,96,-47},{57,-47,96,-46},{58,-46,96,-45},{59,-45,96,-44},{60,-44,96,-43},{61,-43,96,-42},{62,-42,96,-41},{63,-41,96,-40},{64,-40,96,INT32_MAX},
{96,-195,128,INT32_MAX},
{128,-229,INT32_MAX,INT32_MAX},
{128,INT32_MIN,INT32_MAX,-288},{127,-288,INT32_MAX,-287},{126,-287,INT32_MAX,-286},{125,-286,INT32_MAX,-285},{124,-285,INT32_MAX,-284},{123,-284,INT32_MAX,-283},{122,-283,INT32_MAX,-282},{121,-282,INT32_MAX,-281},{120,-281,INT32_MAX,-280},{119,-280,INT32_MAX,-279},{118,-279,INT32_MAX,-278},{117,-278,INT32_MAX,-277},{116,-277,INT32_MAX,-276},{115,-276,INT32_MAX,-275},{114,-275,INT32_MAX,-274},{113,-274,INT32_MAX,-273},{112,-273,INT32_MAX,-272},{111,-272,INT32_MAX,-271},{110,-271,INT32_MAX,-270},{109,-270,INT32_MAX,-269},{108,-269,INT32_MAX,-268},{107,-268,INT32_MAX,-267},{106,-267,INT32_MAX,-266},{105,-266,INT32_MAX,-265},{104,-265,INT32_MAX,-264},{103,-264,INT32_MAX,-263},{102,-263,INT32_MAX,-262},{101,-262,INT32_MAX,-261},{100,-261,INT32_MAX,-260},{99,-260,INT32_MAX,-259},{98,-259,INT32_MAX,-258},{97,-258,INT32_MAX,-257},{96,-257,INT32_MAX,-229},{96,-229,128,-195},
{INT32_MIN,INT32_MIN,128,-288},{INT32_MIN,-288,127,-287},{INT32_MIN,-287,126,-286},{INT32_MIN,-286,125,-285},{INT32_MIN,-285,124,-284},{INT32_MIN,-284,123,-283},{INT32_MIN,-283,122,-282},{INT32_MIN,-282,121,-281},{INT32_MIN,-281,120,-280},{INT32_MIN,-280,119,-279},{INT32_MIN,-279,118,-278},{INT32_MIN,-278,117,-277},{INT32_MIN,-277,116,-276},{INT32_MIN,-276,115,-275},{INT32_MIN,-275,114,-274},{INT32_MIN,-274,113,-273},{INT32_MIN,-273,112,-272},{INT32_MIN,-272,111,-271},{INT32_MIN,-271,110,-270},{INT32_MIN,-270,109,-269},{INT32_MIN,-269,108,-268},{INT32_MIN,-268,107,-267},{INT32_MIN,-267,106,-266},{INT32_MIN,-266,105,-265},{INT32_MIN,-265,104,-264},{INT32_MIN,-264,103,-263},{INT32_MIN,-263,102,-262},{INT32_MIN,-262,101,-261},{INT32_MIN,-261,100,-260},{INT32_MIN,-260,99,-259},{INT32_MIN,-259,98,-258},{INT32_MIN,-258,97,-257},{INT32_MIN,-257,96,-195},
//Second angle
{INT32_MIN,-113,32,-23},{INT32_MIN,-23,31,-20},{INT32_MIN,-20,30,-17},{INT32_MIN,-17,29,-14},{INT32_MIN,-14,28,-11},{INT32_MIN,-11,27,-8},{INT32_MIN,-8,26,-5},{INT32_MIN,-5,25,-2},{INT32_MIN,-2,24,1},{INT32_MIN,1,23,4},{INT32_MIN,4,22,7},{INT32_MIN,7,21,10},{INT32_MIN,10,20,13},{INT32_MIN,13,19,16},{INT32_MIN,16,18,19},{INT32_MIN,19,17,22},{INT32_MIN,22,16,25},{INT32_MIN,25,15,28},{INT32_MIN,28,14,31},{INT32_MIN,31,13,34},{INT32_MIN,34,12,37},{INT32_MIN,37,11,40},{INT32_MIN,40,10,43},{INT32_MIN,43,9,46},{INT32_MIN,46,8,49},{INT32_MIN,49,7,52},{INT32_MIN,52,6,55},{INT32_MIN,55,5,58},{INT32_MIN,58,4,61},{INT32_MIN,61,3,64},{INT32_MIN,64,2,67},{INT32_MIN,67,1,70},{INT32_MIN,70,0,INT32_MAX},
{32,-60,33,-58},{32,-58,34,-56},{32,-56,35,-54},{32,-54,36,-52},{32,-52,37,-50},{32,-50,38,-48},{32,-48,39,-46},{32,-46,40,-44},{32,-44,41,-42},{32,-42,42,-40},{32,-40,43,-38},{32,-38,44,-36},{32,-36,45,-34},{32,-34,46,-32},{32,-32,47,-30},{32,-30,48,-28},{32,-28,49,-26},{32,-26,50,-24},{32,-24,51,-23},{31,-23,51,-22},{31,-22,52,-20},{30,-20,53,-18},{30,-18,54,-17},{29,-17,54,-16},{29,-16,55,-14},{28,-14,56,-12},{28,-12,57,-11},{27,-11,57,-10},{27,-10,58,-8},{26,-8,59,-6},{26,-6,60,-5},{25,-5,60,-4},{25,-4,61,-2},{24,-2,62,0},{24,0,63,1},{23,1,63,2},{23,2,64,4},{22,4,64,7},{21,7,64,10},{20,10,64,13},{19,13,64,16},{18,16,64,19},{17,19,64,22},{16,22,64,25},{15,25,64,28},{14,28,64,31},{13,31,64,34},{12,34,64,37},{11,37,64,40},{10,40,64,43},{9,43,64,46},{8,46,64,49},{7,49,64,52},{6,52,64,55},{5,55,64,58},{4,58,64,61},{3,61,64,64},{2,64,64,67},{1,67,64,70},{0,70,64,INT32_MAX},
{32,-113,64,-60},{33,-60,64,-58},{34,-58,64,-56},{35,-56,64,-54},{36,-54,64,-52},{37,-52,64,-50},{38,-50,64,-48},{39,-48,64,-47},{39,-47,66,-46},{40,-46,67,-45},{40,-45,69,-44},{41,-44,70,-43},{41,-43,72,-42},{42,-42,73,-41},{42,-41,75,-40},{43,-40,76,-39},{43,-39,78,-38},{44,-38,79,-37},{44,-37,81,-36},{45,-36,82,-35},{45,-35,84,-34},{46,-34,85,-33},{46,-33,87,-32},{47,-32,88,-31},{47,-31,90,-30},{48,-30,91,-29},{48,-29,93,-28},{49,-28,94,-27},{49,-27,96,-26},{50,-26,96,-24},{51,-24,96,-22},{52,-22,96,-20},{53,-20,96,-18},{54,-18,96,-16},{55,-16,96,-14},{56,-14,96,-12},{57,-12,96,-10},{58,-10,96,-8},{59,-8,96,-6},{60,-6,96,-4},{61,-4,96,-2},{62,-2,96,0},{63,0,96,2},{64,2,96,INT32_MAX},
{64,-80,69,-79},{64,-79,74,-78},{64,-78,79,-77},{64,-77,84,-76},{64,-76,89,-75},{64,-75,94,-74},{64,-74,99,-73},{64,-73,104,-72},{64,-72,109,-71},{64,-71,114,-70},{64,-70,119,-69},{64,-69,124,-68},{64,-68,INT32_MAX,-47},{66,-47,INT32_MAX,-46},{67,-46,INT32_MAX,-45},{69,-45,INT32_MAX,-44},{70,-44,INT32_MAX,-43},{72,-43,INT32_MAX,-42},{73,-42,INT32_MAX,-41},{75,-41,INT32_MAX,-40},{76,-40,INT32_MAX,-39},{78,-39,INT32_MAX,-38},{79,-38,INT32_MAX,-37},{81,-37,INT32_MAX,-36},{82,-36,INT32_MAX,-35},{84,-35,INT32_MAX,-34},{85,-34,INT32_MAX,-33},{87,-33,INT32_MAX,-32},{88,-32,INT32_MAX,-31},{90,-31,INT32_MAX,-30},{91,-30,INT32_MAX,-29},{93,-29,INT32_MAX,-28},{94,-28,INT32_MAX,-27},{96,-27,INT32_MAX,INT32_MAX},
{96,INT32_MIN,INT32_MAX,-161},{95,-161,INT32_MAX,-160},{93,-160,INT32_MAX,-159},{91,-159,INT32_MAX,-158},{89,-158,INT32_MAX,-157},{87,-157,INT32_MAX,-156},{85,-156,INT32_MAX,-155},{83,-155,INT32_MAX,-154},{81,-154,INT32_MAX,-153},{79,-153,INT32_MAX,-152},{77,-152,INT32_MAX,-151},{75,-151,INT32_MAX,-150},{73,-150,INT32_MAX,-149},{71,-149,INT32_MAX,-148},{69,-148,INT32_MAX,-147},{67,-147,INT32_MAX,-146},{65,-146,INT32_MAX,-145},{64,-145,INT32_MAX,-80},{69,-80,INT32_MAX,-79},{74,-79,INT32_MAX,-78},{79,-78,INT32_MAX,-77},{84,-77,INT32_MAX,-76},{89,-76,INT32_MAX,-75},{94,-75,INT32_MAX,-74},{99,-74,INT32_MAX,-73},{104,-73,INT32_MAX,-72},{109,-72,INT32_MAX,-71},{114,-71,INT32_MAX,-70},{119,-70,INT32_MAX,-69},{124,-69,INT32_MAX,-68},
{64,INT32_MIN,96,-199},{63,-199,96,-198},{61,-198,96,-197},{60,-197,96,-196},{58,-196,96,-195},{57,-195,96,-194},{55,-194,96,-193},{54,-193,96,-192},{52,-192,96,-191},{51,-191,96,-190},{49,-190,96,-189},{48,-189,96,-188},{46,-188,96,-187},{45,-187,96,-186},{43,-186,96,-185},{42,-185,96,-184},{40,-184,96,-183},{39,-183,96,-182},{37,-182,96,-181},{36,-181,96,-180},{34,-180,96,-179},{33,-179,96,-178},{32,-178,96,-161},{32,-161,95,-160},{32,-160,93,-159},{32,-159,91,-158},{32,-158,89,-157},{32,-157,87,-156},{32,-156,85,-155},{32,-155,83,-154},{32,-154,81,-153},{32,-153,79,-152},{32,-152,77,-151},{32,-151,75,-150},{32,-150,73,-149},{32,-149,71,-148},{32,-148,69,-147},{32,-147,67,-146},{32,-146,65,-145},{32,-145,64,-113},
{INT32_MIN,INT32_MIN,64,-199},{INT32_MIN,-199,63,-198},{INT32_MIN,-198,61,-197},{INT32_MIN,-197,60,-196},{INT32_MIN,-196,58,-195},{INT32_MIN,-195,57,-194},{INT32_MIN,-194,55,-193},{INT32_MIN,-193,54,-192},{INT32_MIN,-192,52,-191},{INT32_MIN,-191,51,-190},{INT32_MIN,-190,49,-189},{INT32_MIN,-189,48,-188},{INT32_MIN,-188,46,-187},{INT32_MIN,-187,45,-186},{INT32_MIN,-186,43,-185},{INT32_MIN,-185,42,-184},{INT32_MIN,-184,40,-183},{INT32_MIN,-183,39,-182},{INT32_MIN,-182,37,-181},{INT32_MIN,-181,36,-180},{INT32_MIN,-180,34,-179},{INT32_MIN,-179,33,-178},{INT32_MIN,-178,32,-113},
//Third angle
{-26,INT32_MIN,INT32_MAX,INT32_MAX},
{-27,-65,-26,-63},{-28,-63,-26,-61},{-29,-61,-26,-59},{-30,-59,-26,-57},{-31,-57,-26,-55},{-32,-55,-26,-53},{-33,-53,-26,-51},{-34,-51,-26,-49},{-35,-49,-26,-47},{-36,-47,-26,-45},{-37,-45,-26,-43},{-38,-43,-26,-41},{-39,-41,-26,-39},{-40,-39,-26,-37},{-41,-37,-26,-35},{-42,-35,-26,-33},{-43,-33,-26,-31},{-44,-31,-26,-29},{-45,-29,-26,-27},{-46,-27,-26,-25},{-47,-25,-26,-23},{-48,-23,-26,-21},{-49,-21,-26,-19},{-50,-19,-26,-17},{-51,-17,-26,-15},{-52,-15,-26,-13},{-53,-13,-26,-11},{-54,-11,-26,-9},{-55,-9,-26,-7},{-56,-7,-26,-5},{-57,-5,-26,-3},{-58,-3,-26,-1},{-59,-1,-26,1},{-60,1,-26,3},{-61,3,-26,5},{-62,5,-26,7},{-63,7,-26,9},{-64,9,-26,11},{-65,11,-26,13},{-66,13,-26,15},{-67,15,-26,17},{-68,17,-26,19},{-69,19,-26,21},{-70,21,-26,23},{-71,23,-26,25},{-72,25,-26,27},{-73,27,-26,29},{-74,29,-26,31},{-75,31,-26,33},{-76,33,-26,35},{-77,35,-26,37},{-78,37,-26,39},{-79,39,-26,41},{-80,41,-26,43},{-81,43,-26,45},{-82,45,-26,47},{-83,47,-26,49},{-84,49,-26,51},{-85,51,-26,53},{-86,53,-26,55},{-87,55,-26,57},{-88,57,-26,59},{-89,59,-26,61},{-90,61,-26,63},{-91,63,-26,65},{-92,65,-26,67},{-93,67,-26,INT32_MAX},
{-64,INT32_MIN,-26,-65},{-64,-65,-27,-63},{-64,-63,-28,-61},{-64,-61,-29,-59},{-64,-59,-30,-57},{-64,-57,-31,-55},{-64,-55,-32,-53},{-64,-53,-33,-51},{-64,-51,-34,-49},{-65,-49,-35,-48},{-66,-48,-35,-47},{-67,-47,-36,-46},{-68,-46,-36,-45},{-69,-45,-37,-44},{-70,-44,-37,-43},{-71,-43,-38,-42},{-72,-42,-38,-41},{-73,-41,-39,-40},{-74,-40,-39,-39},{-75,-39,-40,-38},{-76,-38,-40,-37},{-77,-37,-41,-36},{-78,-36,-41,-35},{-79,-35,-42,-34},{-80,-34,-42,-33},{-81,-33,-43,-32},{-82,-32,-43,-31},{-83,-31,-44,-30},{-84,-30,-44,-29},{-85,-29,-45,-28},{-86,-28,-45,-27},{-87,-27,-46,-26},{-88,-26,-46,-25},{-89,-25,-47,-24},{-90,-24,-47,-23},{-91,-23,-48,-22},{-92,-22,-48,-21},{-93,-21,-49,-19},{-93,-19,-50,-17},{-93,-17,-51,-15},{-93,-15,-52,-13},{-93,-13,-53,-11},{-93,-11,-54,-9},{-93,-9,-55,-7},{-93,-7,-56,-5},{-93,-5,-57,-3},{-93,-3,-58,-1},{-93,-1,-59,1},{-93,1,-60,3},{-93,3,-61,5},{-93,5,-62,7},{-93,7,-63,9},{-93,9,-64,11},{-93,11,-65,13},{-93,13,-66,15},{-93,15,-67,17},{-93,17,-68,19},{-93,19,-69,21},{-93,21,-70,23},{-93,23,-71,25},{-93,25,-72,27},{-93,27,-73,29},{-93,29,-74,31},{-93,31,-75,33},{-93,33,-76,35},{-93,35,-77,37},{-93,37,-78,39},{-93,39,-79,41},{-93,41,-80,43},{-93,43,-81,45},{-93,45,-82,47},{-93,47,-83,49},{-93,49,-84,51},{-93,51,-85,53},{-93,53,-86,55},{-93,55,-87,57},{-93,57,-88,59},{-93,59,-89,61},{-93,61,-90,63},{-93,63,-91,65},{-93,65,-92,67},
{-128,-133,-64,-49},{-128,-49,-65,-48},{-128,-48,-66,-47},{-128,-47,-67,-46},{-128,-46,-68,-45},{-128,-45,-69,-44},{-128,-44,-70,-43},{-128,-43,-71,-42},{-128,-42,-72,-41},{-128,-41,-73,-40},{-128,-40,-74,-39},{-128,-39,-75,-38},{-128,-38,-76,-37},{-128,-37,-77,-36},{-128,-36,-78,-35},{-128,-35,-79,-34},{-128,-34,-80,-33},{-128,-33,-81,-32},{-128,-32,-82,-31},{-128,-31,-83,-30},{-128,-30,-84,-29},{-128,-29,-85,-28},{-128,-28,-86,-27},{-128,-27,-87,-26},{-128,-26,-88,-25},{-128,-25,-89,-24},{-128,-24,-90,-23},{-128,-23,-91,-22},{-128,-22,-92,-21},{-128,-21,-93,INT32_MAX},
{INT32_MIN,INT32_MIN,-128,INT32_MAX},
{-128,-243,-127,-242},{-128,-242,-126,-241},{-128,-241,-125,-240},{-128,-240,-124,-239},{-128,-239,-123,-238},{-128,-238,-122,-237},{-128,-237,-121,-236},{-128,-236,-120,-235},{-128,-235,-119,-234},{-128,-234,-118,-233},{-128,-233,-117,-232},{-128,-232,-116,-231},{-128,-231,-115,-230},{-128,-230,-114,-229},{-128,-229,-113,-228},{-128,-228,-112,-227},{-128,-227,-111,-226},{-128,-226,-110,-225},{-128,-225,-109,-224},{-128,-224,-108,-223},{-128,-223,-107,-222},{-128,-222,-106,-221},{-128,-221,-105,-220},{-128,-220,-104,-219},{-128,-219,-103,-218},{-128,-218,-102,-217},{-128,-217,-101,-216},{-128,-216,-100,-215},{-128,-215,-99,-214},{-128,-214,-98,-213},{-128,-213,-97,-212},{-128,-212,-96,-211},{-128,-211,-95,-210},{-128,-210,-94,-209},{-128,-209,-93,-208},{-128,-208,-92,-207},{-128,-207,-91,-206},{-128,-206,-90,-205},{-128,-205,-89,-204},{-128,-204,-88,-203},{-128,-203,-87,-202},{-128,-202,-86,-201},{-128,-201,-85,-200},{-128,-200,-84,-199},{-128,-199,-83,-198},{-128,-198,-82,-197},{-128,-197,-81,-196},{-128,-196,-80,-195},{-128,-195,-79,-194},{-128,-194,-78,-193},{-128,-193,-77,-192},{-128,-192,-76,-191},{-128,-191,-75,-190},{-128,-190,-74,-189},{-128,-189,-73,-188},{-128,-188,-72,-187},{-128,-187,-71,-186},{-128,-186,-70,-185},{-128,-185,-69,-184},{-128,-184,-68,-183},{-128,-183,-67,-182},{-128,-182,-66,-181},{-128,-181,-65,-180},{-128,-180,-64,-133},
{-128,INT32_MIN,-64,-243},{-127,-243,-64,-242},{-126,-242,-64,-241},{-125,-241,-64,-240},{-124,-240,-64,-239},{-123,-239,-64,-238},{-122,-238,-64,-237},{-121,-237,-64,-236},{-120,-236,-64,-235},{-119,-235,-64,-234},{-118,-234,-64,-233},{-117,-233,-64,-232},{-116,-232,-64,-231},{-115,-231,-64,-230},{-114,-230,-64,-229},{-113,-229,-64,-228},{-112,-228,-64,-227},{-111,-227,-64,-226},{-110,-226,-64,-225},{-109,-225,-64,-224},{-108,-224,-64,-223},{-107,-223,-64,-222},{-106,-222,-64,-221},{-105,-221,-64,-220},{-104,-220,-64,-219},{-103,-219,-64,-218},{-102,-218,-64,-217},{-101,-217,-64,-216},{-100,-216,-64,-215},{-99,-215,-64,-214},{-98,-214,-64,-213},{-97,-213,-64,-212},{-96,-212,-64,-211},{-95,-211,-64,-210},{-94,-210,-64,-209},{-93,-209,-64,-208},{-92,-208,-64,-207},{-91,-207,-64,-206},{-90,-206,-64,-205},{-89,-205,-64,-204},{-88,-204,-64,-203},{-87,-203,-64,-202},{-86,-202,-64,-201},{-85,-201,-64,-200},{-84,-200,-64,-199},{-83,-199,-64,-198},{-82,-198,-64,-197},{-81,-197,-64,-196},{-80,-196,-64,-195},{-79,-195,-64,-194},{-78,-194,-64,-193},{-77,-193,-64,-192},{-76,-192,-64,-191},{-75,-191,-64,-190},{-74,-190,-64,-189},{-73,-189,-64,-188},{-72,-188,-64,-187},{-71,-187,-64,-186},{-70,-186,-64,-185},{-69,-185,-64,-184},{-68,-184,-64,-183},{-67,-183,-64,-182},{-66,-182,-64,-181},{-65,-181,-64,-180},
//Fourth angle
{0,INT32_MIN,INT32_MAX,-41},{-1,-41,INT32_MAX,-40},{-2,-40,INT32_MAX,-39},{-3,-39,INT32_MAX,-38},{-4,-38,INT32_MAX,-37},{-5,-37,INT32_MAX,-36},{-6,-36,INT32_MAX,-35},{-7,-35,INT32_MAX,-34},{-8,-34,INT32_MAX,-33},{-9,-33,INT32_MAX,-32},{-10,-32,INT32_MAX,-31},{-11,-31,INT32_MAX,-30},{-12,-30,INT32_MAX,-29},{-13,-29,INT32_MAX,-28},{-14,-28,INT32_MAX,-27},{-15,-27,INT32_MAX,-26},{-16,-26,INT32_MAX,-25},{-17,-25,INT32_MAX,-24},{-18,-24,INT32_MAX,-23},{-19,-23,INT32_MAX,-22},{-20,-22,INT32_MAX,-21},{-21,-21,INT32_MAX,-20},{-22,-20,INT32_MAX,-19},{-23,-19,INT32_MAX,-18},{-24,-18,INT32_MAX,-17},{-25,-17,INT32_MAX,-16},{-26,-16,INT32_MAX,-15},{-27,-15,INT32_MAX,-14},{-28,-14,INT32_MAX,-13},{-29,-13,INT32_MAX,-12},{-30,-12,INT32_MAX,-11},{-31,-11,INT32_MAX,-10},{-32,-10,INT32_MAX,INT32_MAX},
{-32,-204,0,-83},{-33,-83,0,-82},{-34,-82,0,-81},{-36,-81,0,-80},{-37,-80,0,-79},{-38,-79,0,-78},{-40,-78,0,-77},{-41,-77,0,-76},{-42,-76,0,-75},{-43,-75,0,-74},{-45,-74,0,-73},{-46,-73,0,-72},{-47,-72,0,-71},{-49,-71,0,-70},{-50,-70,0,-69},{-51,-69,0,-68},{-52,-68,0,-67},{-54,-67,0,-66},{-55,-66,0,-65},{-56,-65,0,-64},{-58,-64,0,-63},{-59,-63,0,-62},{-60,-62,0,-61},{-61,-61,0,-59},{-63,-59,0,-58},{-64,-58,0,-41},{-64,-41,-1,-40},{-64,-40,-2,-39},{-64,-39,-3,-38},{-64,-38,-4,-37},{-64,-37,-5,-36},{-64,-36,-6,-35},{-64,-35,-7,-34},{-64,-34,-8,-33},{-64,-33,-9,-32},{-64,-32,-10,-31},{-64,-31,-11,-30},{-64,-30,-12,-29},{-64,-29,-13,-28},{-64,-28,-14,-27},{-64,-27,-15,-26},{-64,-26,-16,-25},{-64,-25,-17,-24},{-64,-24,-18,-23},{-64,-23,-19,-22},{-64,-22,-20,-21},{-64,-21,-21,-20},{-64,-20,-22,-19},{-64,-19,-23,-18},{-64,-18,-24,-17},{-64,-17,-25,-16},{-64,-16,-26,-15},{-64,-15,-27,-14},{-64,-14,-28,-13},{-64,-13,-29,-12},{-64,-12,-30,-11},{-64,-11,-31,-10},{-64,-10,-32,INT32_MAX},
{-64,-204,-32,-137},{-65,-137,-32,-136},{-67,-136,-32,-135},{-69,-135,-32,-134},{-71,-134,-32,-133},{-73,-133,-32,-132},{-75,-132,-32,-131},{-77,-131,-32,-130},{-79,-130,-32,-129},{-81,-129,-32,-128},{-83,-128,-32,-127},{-85,-127,-32,-126},{-87,-126,-32,-125},{-89,-125,-32,-124},{-91,-124,-32,-123},{-93,-123,-32,-122},{-95,-122,-32,-121},{-96,-121,-32,-83},{-96,-83,-33,-82},{-96,-82,-34,-81},{-96,-81,-36,-80},{-96,-80,-37,-79},{-96,-79,-38,-78},{-96,-78,-40,-77},{-96,-77,-41,-76},{-96,-76,-42,-75},{-96,-75,-43,-74},{-96,-74,-45,-73},{-96,-73,-46,-72},{-96,-72,-47,-71},{-96,-71,-49,-70},{-96,-70,-50,-69},{-96,-69,-51,-68},{-96,-68,-52,-67},{-96,-67,-54,-66},{-96,-66,-55,-65},{-96,-65,-56,-64},{-96,-64,-58,-63},{-96,-63,-59,-62},{-96,-62,-60,-61},{-96,-61,-61,-59},{-96,-59,-63,-58},{-96,-58,-64,INT32_MAX},
{-67,-204,-64,-203},{-70,-203,-64,-202},{-72,-202,-64,-201},{-75,-201,-64,-200},{-78,-200,-64,-199},{-80,-199,-64,-198},{-83,-198,-64,-197},{-85,-197,-64,-196},{-88,-196,-64,-195},{-91,-195,-64,-194},{-94,-194,-64,-193},{-96,-193,-64,-192},{-99,-192,-64,-191},{-102,-191,-64,-190},{-104,-190,-64,-189},{-107,-189,-64,-188},{-110,-188,-64,-187},{-112,-187,-64,-186},{-115,-186,-64,-185},{-118,-185,-64,-184},{-120,-184,-64,-183},{-123,-183,-64,-182},{-126,-182,-64,-181},{-128,-181,-64,-180},{INT32_MIN,-180,-64,-137},{INT32_MIN,-137,-65,-136},{INT32_MIN,-136,-67,-135},{INT32_MIN,-135,-69,-134},{INT32_MIN,-134,-71,-133},{INT32_MIN,-133,-73,-132},{INT32_MIN,-132,-75,-131},{INT32_MIN,-131,-77,-130},{INT32_MIN,-130,-79,-129},{INT32_MIN,-129,-81,-128},{INT32_MIN,-128,-83,-127},{INT32_MIN,-127,-85,-126},{INT32_MIN,-126,-87,-125},{INT32_MIN,-125,-89,-124},{INT32_MIN,-124,-91,-123},{INT32_MIN,-123,-93,-122},{INT32_MIN,-122,-95,-121},{INT32_MIN,-121,-96,INT32_MAX},
{INT32_MIN,INT32_MIN,-96,-269},{INT32_MIN,-269,-92,-268},{INT32_MIN,-268,-85,-267},{INT32_MIN,-267,-78,-266},{INT32_MIN,-266,-71,-265},{INT32_MIN,-265,-64,-204},{INT32_MIN,-204,-67,-203},{INT32_MIN,-203,-70,-202},{INT32_MIN,-202,-72,-201},{INT32_MIN,-201,-75,-200},{INT32_MIN,-200,-78,-199},{INT32_MIN,-199,-80,-198},{INT32_MIN,-198,-83,-197},{INT32_MIN,-197,-85,-196},{INT32_MIN,-196,-88,-195},{INT32_MIN,-195,-91,-194},{INT32_MIN,-194,-94,-193},{INT32_MIN,-193,-96,-192},{INT32_MIN,-192,-99,-191},{INT32_MIN,-191,-102,-190},{INT32_MIN,-190,-104,-189},{INT32_MIN,-189,-107,-188},{INT32_MIN,-188,-110,-187},{INT32_MIN,-187,-112,-186},{INT32_MIN,-186,-115,-185},{INT32_MIN,-185,-118,-184},{INT32_MIN,-184,-120,-183},{INT32_MIN,-183,-123,-182},{INT32_MIN,-182,-126,-181},{INT32_MIN,-181,-128,-180},
{-96,-359,-95,-358},{-96,-358,-94,-356},{-96,-356,-93,-355},{-96,-355,-92,-353},{-96,-353,-91,-352},{-96,-352,-90,-350},{-96,-350,-89,-349},{-96,-349,-88,-347},{-96,-347,-87,-346},{-96,-346,-86,-344},{-96,-344,-85,-343},{-96,-343,-84,-341},{-96,-341,-83,-340},{-96,-340,-82,-338},{-96,-338,-81,-337},{-96,-337,-80,-335},{-96,-335,-79,-334},{-96,-334,-78,-332},{-96,-332,-77,-331},{-96,-331,-76,-329},{-96,-329,-75,-328},{-96,-328,-74,-326},{-96,-326,-73,-325},{-96,-325,-72,-323},{-96,-323,-71,-322},{-96,-322,-70,-320},{-96,-320,-69,-319},{-96,-319,-68,-317},{-96,-317,-67,-316},{-96,-316,-66,-314},{-96,-314,-65,-313},{-96,-313,-64,-311},{-96,-311,-63,-309},{-96,-309,-62,-308},{-96,-308,-61,-306},{-96,-306,-60,-305},{-96,-305,-59,-303},{-96,-303,-58,-302},{-96,-302,-57,-300},{-96,-300,-56,-299},{-96,-299,-55,-297},{-96,-297,-54,-296},{-96,-296,-53,-294},{-96,-294,-52,-293},{-96,-293,-51,-291},{-96,-291,-50,-290},{-96,-290,-49,-288},{-96,-288,-48,-287},{-96,-287,-47,-285},{-96,-285,-46,-284},{-96,-284,-45,-282},{-96,-282,-44,-281},{-96,-281,-43,-279},{-96,-279,-42,-278},{-96,-278,-41,-276},{-96,-276,-40,-275},{-96,-275,-39,-273},{-96,-273,-38,-272},{-96,-272,-37,-270},{-96,-270,-36,-269},{-92,-269,-35,-268},{-85,-268,-35,-267},{-78,-267,-34,-266},{-71,-266,-33,-265},{-64,-265,-33,-264},{-64,-264,-32,-204},
{-96,INT32_MIN,0,-359},{-95,-359,0,-358},{-94,-358,0,-356},{-93,-356,0,-355},{-92,-355,0,-353},{-91,-353,0,-352},{-90,-352,0,-350},{-89,-350,0,-349},{-88,-349,0,-347},{-87,-347,0,-346},{-86,-346,0,-344},{-85,-344,0,-343},{-84,-343,0,-341},{-83,-341,0,-340},{-82,-340,0,-338},{-81,-338,0,-337},{-80,-337,0,-335},{-79,-335,0,-334},{-78,-334,0,-332},{-77,-332,0,-331},{-76,-331,0,-329},{-75,-329,0,-328},{-74,-328,0,-326},{-73,-326,0,-325},{-72,-325,0,-323},{-71,-323,0,-322},{-70,-322,0,-320},{-69,-320,0,-319},{-68,-319,0,-317},{-67,-317,0,-316},{-66,-316,0,-314},{-65,-314,0,-313},{-64,-313,0,-311},{-63,-311,0,-309},{-62,-309,0,-308},{-61,-308,0,-306},{-60,-306,0,-305},{-59,-305,0,-303},{-58,-303,0,-302},{-57,-302,0,-300},{-56,-300,0,-299},{-55,-299,0,-297},{-54,-297,0,-296},{-53,-296,0,-294},{-52,-294,0,-293},{-51,-293,0,-291},{-50,-291,0,-290},{-49,-290,0,-288},{-48,-288,0,-287},{-47,-287,0,-285},{-46,-285,0,-284},{-45,-284,0,-282},{-44,-282,0,-281},{-43,-281,0,-279},{-42,-279,0,-278},{-41,-278,0,-276},{-40,-276,0,-275},{-39,-275,0,-273},{-38,-273,0,-272},{-37,-272,0,-270},{-36,-270,0,-269},{-35,-269,0,-267},{-34,-267,0,-266},{-33,-266,0,-264},{-32,-264,0,-204},
};
mask_t large_half_loop_right_masks[]={
{0,33,0,0,large_half_loop_right_rects},{0,65,-32,32,large_half_loop_right_rects+33},{0,33,-64,64,large_half_loop_right_rects+98},{0,1,-96,112,large_half_loop_right_rects+131},{0,1,-160,168,large_half_loop_right_rects+132},{0,34,-128,96,large_half_loop_right_rects+133},{0,33,-96,264,large_half_loop_right_rects+167},
{0,33,0,0,large_half_loop_right_rects+200},{0,60,-32,0,large_half_loop_right_rects+233},{0,44,-64,0,large_half_loop_right_rects+293},{0,34,-96,16,large_half_loop_right_rects+337},{0,30,-96,40,large_half_loop_right_rects+371},{0,40,-64,0,large_half_loop_right_rects+401},{0,23,-32,200,large_half_loop_right_rects+441},
{0,1,0,0,large_half_loop_right_rects+464},{0,67,32,0,large_half_loop_right_rects+465},{0,81,64,0,large_half_loop_right_rects+532},{0,30,96,16,large_half_loop_right_rects+613},{0,1,160,72,large_half_loop_right_rects+643},{0,64,128,32,large_half_loop_right_rects+644},{0,64,96,232,large_half_loop_right_rects+708},
{0,33,0,0,large_half_loop_right_rects+772},{0,58,32,32,large_half_loop_right_rects+805},{0,43,64,64,large_half_loop_right_rects+863},{0,42,96,112,large_half_loop_right_rects+906},{0,30,96,200,large_half_loop_right_rects+948},{0,66,64,128,large_half_loop_right_rects+978},{0,65,32,296,large_half_loop_right_rects+1044},
};

const track_section_t large_half_loop_right={TRACK_OFFSET_SPRITE_MASK,large_half_loop_right_curve,LARGE_HALF_LOOP_LENGTH,{{0,7,large_half_loop_right_masks},{0,7,large_half_loop_right_masks+7},{0,7,large_half_loop_right_masks+14},{0,7,large_half_loop_right_masks+21}}};





rect_t small_turn_left_bank_to_gentle_up_rects[]=
{
{INT32_MIN,-16,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,-16},
{INT32_MIN,INT32_MIN,32,INT32_MAX},{32,INT32_MIN,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,16},{INT32_MIN,16,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-32,INT32_MAX},
};

mask_t small_turn_left_bank_to_gentle_up_masks[]={
{0,1,-6,-3,small_turn_left_bank_to_gentle_up_rects},{0,1,6,45,small_turn_left_bank_to_gentle_up_rects+1},
{TRACK_MASK_INTERSECT,1,6,-3,small_turn_left_bank_to_gentle_up_rects+2},{TRACK_MASK_DIFFERENCE,1,6,-3,small_turn_left_bank_to_gentle_up_rects+2},{0,1,-64-6,16-3,small_turn_left_bank_to_gentle_up_rects+3},
{0,1,-6,-3,small_turn_left_bank_to_gentle_up_rects+4},{0,1,6,-16-3,small_turn_left_bank_to_gentle_up_rects+5},
{0,1,6,-3,small_turn_left_bank_to_gentle_up_rects+6},{0,1,64-6,16-3,small_turn_left_bank_to_gentle_up_rects+7},
};

const track_section_t small_turn_left_bank_to_gentle_up={TRACK_OFFSET_SPRITE_MASK|TRACK_ENTRY_BANK_LEFT|TRACK_SUPPORT_BASE,small_turn_left_bank_to_gentle_up_curve,TURN_BANK_TRANSITION_LENGTH,{{0,2,small_turn_left_bank_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,3,small_turn_left_bank_to_gentle_up_masks+2},{0,2,small_turn_left_bank_to_gentle_up_masks+5},{0,2,small_turn_left_bank_to_gentle_up_masks+7}}};


rect_t small_turn_right_bank_to_gentle_up_rects[]=
{
{INT32_MIN,INT32_MIN,32,INT32_MAX},{32,INT32_MIN,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,16},{INT32_MIN,16,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-32,INT32_MAX},
{INT32_MIN,-16,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,-16},
};

mask_t small_turn_right_bank_to_gentle_up_masks[]={
{0,1,-6,-3,small_turn_right_bank_to_gentle_up_rects},{0,1,-58,13,small_turn_right_bank_to_gentle_up_rects+1},
{0,1,6,-3,small_turn_right_bank_to_gentle_up_rects+2},{0,1,-6,-19,small_turn_right_bank_to_gentle_up_rects+3},
{TRACK_MASK_INTERSECT,1,-6,-3,small_turn_right_bank_to_gentle_up_rects+4},{TRACK_MASK_DIFFERENCE,1,-6,-3,small_turn_right_bank_to_gentle_up_rects+4},{0,1,70,13,small_turn_right_bank_to_gentle_up_rects+5},
{0,1,6,-3,small_turn_right_bank_to_gentle_up_rects+6},{0,1,-6,45,small_turn_right_bank_to_gentle_up_rects+7},
};

const track_section_t small_turn_right_bank_to_gentle_up={TRACK_OFFSET_SPRITE_MASK|TRACK_ENTRY_BANK_RIGHT|TRACK_SUPPORT_BASE,small_turn_right_bank_to_gentle_up_curve,TURN_BANK_TRANSITION_LENGTH,{{0,2,small_turn_right_bank_to_gentle_up_masks},{0,2,small_turn_right_bank_to_gentle_up_masks+2},{VIEW_NEEDS_TRACK_MASK,3,small_turn_right_bank_to_gentle_up_masks+4},{0,2,small_turn_right_bank_to_gentle_up_masks+7}}};





rect_t corkscrew_left_rects[]=
{
//First angle
{INT32_MIN,-40,2,-26},{INT32_MIN,-26,4,-25},{INT32_MIN,-25,5,-24},{INT32_MIN,-24,7,-23},{INT32_MIN,-23,8,-22},{INT32_MIN,-22,10,-21},{INT32_MIN,-21,11,-20},{INT32_MIN,-20,13,-19},{INT32_MIN,-19,14,-18},{INT32_MIN,-18,16,-17},{INT32_MIN,-17,17,-16},{INT32_MIN,-16,19,-15},{INT32_MIN,-15,20,-14},{INT32_MIN,-14,22,-13},{INT32_MIN,-13,23,-12},{INT32_MIN,-12,25,-11},{INT32_MIN,-11,27,-10},{INT32_MIN,-10,28,-9},{INT32_MIN,-9,30,-8},{INT32_MIN,-8,31,-7},{INT32_MIN,-7,33,-6},{INT32_MIN,-6,34,-5},{INT32_MIN,-5,36,-4},{INT32_MIN,-4,37,-3},{INT32_MIN,-3,39,-2},{INT32_MIN,-2,40,-1},{INT32_MIN,-1,41,0},{INT32_MIN,0,INT32_MAX,INT32_MAX},
{41,-83,INT32_MAX,-82},{39,-82,INT32_MAX,-81},{37,-81,INT32_MAX,-80},{35,-80,INT32_MAX,-79},{33,-79,INT32_MAX,-78},{31,-78,INT32_MAX,-77},{29,-77,INT32_MAX,-76},{27,-76,INT32_MAX,-75},{25,-75,INT32_MAX,-74},{23,-74,INT32_MAX,-73},{21,-73,INT32_MAX,-72},{19,-72,INT32_MAX,-71},{17,-71,INT32_MAX,-70},{15,-70,INT32_MAX,-69},{13,-69,INT32_MAX,-68},{11,-68,INT32_MAX,-67},{9,-67,INT32_MAX,-66},{7,-66,INT32_MAX,-65},{5,-65,INT32_MAX,-64},{3,-64,INT32_MAX,-63},{2,-63,INT32_MAX,-26},{4,-26,INT32_MAX,-25},{5,-25,INT32_MAX,-24},{7,-24,INT32_MAX,-23},{8,-23,INT32_MAX,-22},{10,-22,INT32_MAX,-21},{11,-21,INT32_MAX,-20},{13,-20,INT32_MAX,-19},{14,-19,INT32_MAX,-18},{16,-18,INT32_MAX,-17},{17,-17,INT32_MAX,-16},{19,-16,INT32_MAX,-15},{20,-15,INT32_MAX,-14},{22,-14,INT32_MAX,-13},{23,-13,INT32_MAX,-12},{25,-12,INT32_MAX,-11},{27,-11,INT32_MAX,-10},{28,-10,INT32_MAX,-9},{30,-9,INT32_MAX,-8},{31,-8,INT32_MAX,-7},{33,-7,INT32_MAX,-6},{34,-6,INT32_MAX,-5},{36,-5,INT32_MAX,-4},{37,-4,INT32_MAX,-3},{39,-3,INT32_MAX,-2},{40,-2,INT32_MAX,-1},{41,-1,INT32_MAX,0},
{INT32_MIN,INT32_MIN,INT32_MAX,-83},{INT32_MIN,-83,41,-82},{INT32_MIN,-82,39,-81},{INT32_MIN,-81,37,-80},{INT32_MIN,-80,35,-79},{INT32_MIN,-79,33,-78},{INT32_MIN,-78,31,-77},{INT32_MIN,-77,29,-76},{INT32_MIN,-76,27,-75},{INT32_MIN,-75,25,-74},{INT32_MIN,-74,23,-73},{INT32_MIN,-73,21,-72},{INT32_MIN,-72,19,-71},{INT32_MIN,-71,17,-70},{INT32_MIN,-70,15,-69},{INT32_MIN,-69,13,-68},{INT32_MIN,-68,11,-67},{INT32_MIN,-67,9,-66},{INT32_MIN,-66,7,-65},{INT32_MIN,-65,5,-64},{INT32_MIN,-64,3,-63},{INT32_MIN,-63,2,-40},
//Second angle
{INT32_MIN,INT32_MIN,0,0},{INT32_MIN,0,1,1},{INT32_MIN,1,0,INT32_MAX},
{0,INT32_MIN,33,-52},{0,-52,34,-51},{0,-51,35,-50},{0,-50,36,-49},{0,-49,37,-48},{0,-48,38,-47},{0,-47,39,-46},{0,-46,40,-45},{0,-45,41,-44},{0,-44,42,-43},{0,-43,43,-42},{0,-42,44,-41},{0,-41,45,-40},{0,-40,46,-39},{0,-39,47,-38},{0,-38,48,-37},{0,-37,49,-36},{0,-36,50,-35},{0,-35,51,-34},{0,-34,52,-33},{0,-33,53,-32},{0,-32,54,-31},{0,-31,55,-30},{0,-30,56,-29},{0,-29,57,-28},{0,-28,58,-27},{0,-27,59,-26},{0,-26,60,-25},{0,-25,61,-24},{0,-24,62,-23},{0,-23,63,-22},{0,-22,64,-21},{0,-21,65,0},{1,0,65,1},{0,1,65,INT32_MAX},
{33,INT32_MIN,INT32_MAX,-52},{34,-52,INT32_MAX,-51},{35,-51,INT32_MAX,-50},{36,-50,INT32_MAX,-49},{37,-49,INT32_MAX,-48},{38,-48,INT32_MAX,-47},{39,-47,INT32_MAX,-46},{40,-46,INT32_MAX,-45},{41,-45,INT32_MAX,-44},{42,-44,INT32_MAX,-43},{43,-43,INT32_MAX,-42},{44,-42,INT32_MAX,-41},{45,-41,INT32_MAX,-40},{46,-40,INT32_MAX,-39},{47,-39,INT32_MAX,-38},{48,-38,INT32_MAX,-37},{49,-37,INT32_MAX,-36},{50,-36,INT32_MAX,-35},{51,-35,INT32_MAX,-34},{52,-34,INT32_MAX,-33},{53,-33,INT32_MAX,-32},{54,-32,INT32_MAX,-31},{55,-31,INT32_MAX,-30},{56,-30,INT32_MAX,-29},{57,-29,INT32_MAX,-28},{58,-28,INT32_MAX,-27},{59,-27,INT32_MAX,-26},{60,-26,INT32_MAX,-25},{61,-25,INT32_MAX,-24},{62,-24,INT32_MAX,-23},{63,-23,INT32_MAX,-22},{64,-22,INT32_MAX,-21},{65,-21,INT32_MAX,INT32_MAX},
//Third angle
{-20,-9,INT32_MAX,-8},{-21,-8,INT32_MAX,-7},{-22,-7,INT32_MAX,-6},{-23,-6,INT32_MAX,-5},{-24,-5,INT32_MAX,-4},{-25,-4,INT32_MAX,-3},{-26,-3,INT32_MAX,-2},{-27,-2,INT32_MAX,-1},{-28,-1,INT32_MAX,0},{-29,0,INT32_MAX,1},{-30,1,INT32_MAX,2},{-31,2,INT32_MAX,3},{-32,3,INT32_MAX,4},{-33,4,INT32_MAX,5},{-34,5,INT32_MAX,6},{-35,6,INT32_MAX,7},{-36,7,INT32_MAX,8},{-37,8,INT32_MAX,9},{-38,9,INT32_MAX,10},{-39,10,INT32_MAX,11},{-40,11,INT32_MAX,12},{-41,12,INT32_MAX,13},{-42,13,INT32_MAX,14},{-43,14,INT32_MAX,15},{-44,15,INT32_MAX,16},{-45,16,INT32_MAX,17},{-46,17,INT32_MAX,18},{-47,18,INT32_MAX,19},{-48,19,INT32_MAX,20},{-49,20,INT32_MAX,21},{-50,21,INT32_MAX,22},{-51,22,INT32_MAX,23},{INT32_MIN,23,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,-19,-9},{INT32_MIN,-9,-20,-8},{INT32_MIN,-8,-21,-7},{INT32_MIN,-7,-22,-6},{INT32_MIN,-6,-23,-5},{INT32_MIN,-5,-24,-4},{INT32_MIN,-4,-25,-3},{INT32_MIN,-3,-26,-2},{INT32_MIN,-2,-27,-1},{INT32_MIN,-1,-28,0},{INT32_MIN,0,-29,1},{INT32_MIN,1,-30,2},{INT32_MIN,2,-31,3},{INT32_MIN,3,-32,4},{INT32_MIN,4,-33,5},{INT32_MIN,5,-34,6},{INT32_MIN,6,-35,7},{INT32_MIN,7,-36,8},{INT32_MIN,8,-37,9},{INT32_MIN,9,-38,10},{INT32_MIN,10,-39,11},{INT32_MIN,11,-40,12},{INT32_MIN,12,-41,13},{INT32_MIN,13,-42,14},{INT32_MIN,14,-43,15},{INT32_MIN,15,-44,16},{INT32_MIN,16,-45,17},{INT32_MIN,17,-46,18},{INT32_MIN,18,-47,19},{INT32_MIN,19,-48,20},{INT32_MIN,20,-49,21},{INT32_MIN,21,-50,22},{INT32_MIN,22,-51,23},
{-19,INT32_MIN,INT32_MAX,-9},
//Fourth angle
{-12,INT32_MIN,INT32_MAX,-54},{-13,-54,INT32_MAX,-52},{-14,-52,INT32_MAX,-51},{-15,-51,INT32_MAX,-50},{-16,-50,INT32_MAX,-48},{-17,-48,INT32_MAX,-47},{-18,-47,INT32_MAX,-45},{-19,-45,INT32_MAX,-44},{-20,-44,INT32_MAX,-42},{-21,-42,INT32_MAX,-41},{-22,-41,INT32_MAX,-39},{-23,-39,INT32_MAX,-38},{-24,-38,INT32_MAX,-37},{-25,-37,INT32_MAX,-35},{-26,-35,INT32_MAX,-34},{-27,-34,INT32_MAX,-32},{-28,-32,INT32_MAX,-31},{-29,-31,INT32_MAX,-29},{-30,-29,INT32_MAX,-28},{-31,-28,INT32_MAX,-26},{-32,-26,INT32_MAX,INT32_MAX},
{-44,INT32_MIN,-12,-54},{-44,-54,-13,-52},{-44,-52,-14,-51},{-44,-51,-15,-50},{-44,-50,-16,-48},{-44,-48,-17,-47},{-44,-47,-18,-45},{-44,-45,-19,-44},{-44,-44,-20,-42},{-44,-42,-21,-41},{-44,-41,-22,-39},{-44,-39,-23,-38},{-44,-38,-24,-37},{-44,-37,-25,-35},{-44,-35,-26,-34},{-44,-34,-27,-32},{-44,-32,-28,-31},{-44,-31,-29,-29},{-44,-29,-30,-28},{-44,-28,-31,-26},{-44,-26,-32,INT32_MAX},
{INT32_MIN,INT32_MIN,-44,INT32_MAX},
};
mask_t corkscrew_left_masks[]={

{TRACK_MASK_TRANSFER_NEXT,28,0,0,corkscrew_left_rects},{TRACK_MASK_DIFFERENCE,47,-32,40,corkscrew_left_rects+28},{0,22,0,80,corkscrew_left_rects+75},
{TRACK_MASK_TRANSFER_NEXT,3,0,0,corkscrew_left_rects+97},{TRACK_MASK_DIFFERENCE,35,-32,8,corkscrew_left_rects+100},{0,33,-64,48,corkscrew_left_rects+135},
{0,33,0,0,corkscrew_left_rects+168},{0,33,32,8,corkscrew_left_rects+201},{0,1,0,16,corkscrew_left_rects+234},
{0,21,0,0,corkscrew_left_rects+235},{TRACK_MASK_TRANSFER_NEXT,21,32,40,corkscrew_left_rects+256},{TRACK_MASK_DIFFERENCE,1,64,48,corkscrew_left_rects+277},
};

const track_section_t corkscrew_left={TRACK_NO_SUPPORTS|TRACK_OFFSET_SPRITE_MASK|TRACK_SPECIAL_CORKSCREW_LEFT,corkscrew_left_curve,CORKSCREW_LENGTH,{{VIEW_NEEDS_TRACK_MASK,3,corkscrew_left_masks},{VIEW_NEEDS_TRACK_MASK,3,corkscrew_left_masks+3},{0,3,corkscrew_left_masks+6},{VIEW_NEEDS_TRACK_MASK,3,corkscrew_left_masks+9}}};

rect_t corkscrew_right_rects[]=
{
//First angle
{INT32_MIN,INT32_MIN,12,-54},{INT32_MIN,-54,13,-52},{INT32_MIN,-52,14,-51},{INT32_MIN,-51,15,-50},{INT32_MIN,-50,16,-48},{INT32_MIN,-48,17,-47},{INT32_MIN,-47,18,-45},{INT32_MIN,-45,19,-44},{INT32_MIN,-44,20,-42},{INT32_MIN,-42,21,-41},{INT32_MIN,-41,22,-39},{INT32_MIN,-39,23,-38},{INT32_MIN,-38,24,-37},{INT32_MIN,-37,25,-35},{INT32_MIN,-35,26,-34},{INT32_MIN,-34,27,-32},{INT32_MIN,-32,28,-31},{INT32_MIN,-31,29,-29},{INT32_MIN,-29,30,-28},{INT32_MIN,-28,31,-26},{INT32_MIN,-26,32,INT32_MAX},
{12,INT32_MIN,44,-54},{13,-54,44,-52},{14,-52,44,-51},{15,-51,44,-50},{16,-50,44,-48},{17,-48,44,-47},{18,-47,44,-45},{19,-45,44,-44},{20,-44,44,-42},{21,-42,44,-41},{22,-41,44,-39},{23,-39,44,-38},{24,-38,44,-37},{25,-37,44,-35},{26,-35,44,-34},{27,-34,44,-32},{28,-32,44,-31},{29,-31,44,-29},{30,-29,44,-28},{31,-28,44,-26},{32,-26,44,INT32_MAX},
{44,INT32_MIN,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,-9,20,-8},{INT32_MIN,-8,21,-7},{INT32_MIN,-7,22,-6},{INT32_MIN,-6,23,-5},{INT32_MIN,-5,24,-4},{INT32_MIN,-4,25,-3},{INT32_MIN,-3,26,-2},{INT32_MIN,-2,27,-1},{INT32_MIN,-1,28,0},{INT32_MIN,0,29,1},{INT32_MIN,1,30,2},{INT32_MIN,2,31,3},{INT32_MIN,3,32,4},{INT32_MIN,4,33,5},{INT32_MIN,5,34,6},{INT32_MIN,6,35,7},{INT32_MIN,7,36,8},{INT32_MIN,8,37,9},{INT32_MIN,9,38,10},{INT32_MIN,10,39,11},{INT32_MIN,11,40,12},{INT32_MIN,12,41,13},{INT32_MIN,13,42,14},{INT32_MIN,14,43,15},{INT32_MIN,15,44,16},{INT32_MIN,16,45,17},{INT32_MIN,17,46,18},{INT32_MIN,18,47,19},{INT32_MIN,19,48,20},{INT32_MIN,20,49,21},{INT32_MIN,21,50,22},{INT32_MIN,22,51,23},{INT32_MIN,23,INT32_MAX,INT32_MAX},
{19,INT32_MIN,INT32_MAX,-9},{20,-9,INT32_MAX,-8},{21,-8,INT32_MAX,-7},{22,-7,INT32_MAX,-6},{23,-6,INT32_MAX,-5},{24,-5,INT32_MAX,-4},{25,-4,INT32_MAX,-3},{26,-3,INT32_MAX,-2},{27,-2,INT32_MAX,-1},{28,-1,INT32_MAX,0},{29,0,INT32_MAX,1},{30,1,INT32_MAX,2},{31,2,INT32_MAX,3},{32,3,INT32_MAX,4},{33,4,INT32_MAX,5},{34,5,INT32_MAX,6},{35,6,INT32_MAX,7},{36,7,INT32_MAX,8},{37,8,INT32_MAX,9},{38,9,INT32_MAX,10},{39,10,INT32_MAX,11},{40,11,INT32_MAX,12},{41,12,INT32_MAX,13},{42,13,INT32_MAX,14},{43,14,INT32_MAX,15},{44,15,INT32_MAX,16},{45,16,INT32_MAX,17},{46,17,INT32_MAX,18},{47,18,INT32_MAX,19},{48,19,INT32_MAX,20},{49,20,INT32_MAX,21},{50,21,INT32_MAX,22},{51,22,INT32_MAX,23},
{INT32_MIN,INT32_MIN,19,-9},
//Third angle
{0,INT32_MIN,INT32_MAX,INT32_MAX},
{-33,INT32_MIN,0,-52},{-34,-52,0,-51},{-35,-51,0,-50},{-36,-50,0,-49},{-37,-49,0,-48},{-38,-48,0,-47},{-39,-47,0,-46},{-40,-46,0,-45},{-41,-45,0,-44},{-42,-44,0,-43},{-43,-43,0,-42},{-44,-42,0,-41},{-45,-41,0,-40},{-46,-40,0,-39},{-47,-39,0,-38},{-48,-38,0,-37},{-49,-37,0,-36},{-50,-36,0,-35},{-51,-35,0,-34},{-52,-34,0,-33},{-53,-33,0,-32},{-54,-32,0,-31},{-55,-31,0,-30},{-56,-30,0,-29},{-57,-29,0,-28},{-58,-28,0,-27},{-59,-27,0,-26},{-60,-26,0,-25},{-61,-25,0,-24},{-62,-24,0,-23},{-63,-23,0,-22},{-64,-22,0,-21},{-65,-21,0,INT32_MAX},
{INT32_MIN,INT32_MIN,-33,-52},{INT32_MIN,-52,-34,-51},{INT32_MIN,-51,-35,-50},{INT32_MIN,-50,-36,-49},{INT32_MIN,-49,-37,-48},{INT32_MIN,-48,-38,-47},{INT32_MIN,-47,-39,-46},{INT32_MIN,-46,-40,-45},{INT32_MIN,-45,-41,-44},{INT32_MIN,-44,-42,-43},{INT32_MIN,-43,-43,-42},{INT32_MIN,-42,-44,-41},{INT32_MIN,-41,-45,-40},{INT32_MIN,-40,-46,-39},{INT32_MIN,-39,-47,-38},{INT32_MIN,-38,-48,-37},{INT32_MIN,-37,-49,-36},{INT32_MIN,-36,-50,-35},{INT32_MIN,-35,-51,-34},{INT32_MIN,-34,-52,-33},{INT32_MIN,-33,-53,-32},{INT32_MIN,-32,-54,-31},{INT32_MIN,-31,-55,-30},{INT32_MIN,-30,-56,-29},{INT32_MIN,-29,-57,-28},{INT32_MIN,-28,-58,-27},{INT32_MIN,-27,-59,-26},{INT32_MIN,-26,-60,-25},{INT32_MIN,-25,-61,-24},{INT32_MIN,-24,-62,-23},{INT32_MIN,-23,-63,-22},{INT32_MIN,-22,-64,-21},{INT32_MIN,-21,-65,INT32_MAX},
//Fourth angle
{-2,-40,INT32_MAX,-26},{-4,-26,INT32_MAX,-25},{-5,-25,INT32_MAX,-24},{-7,-24,INT32_MAX,-23},{-8,-23,INT32_MAX,-22},{-10,-22,INT32_MAX,-21},{-11,-21,INT32_MAX,-20},{-13,-20,INT32_MAX,-19},{-14,-19,INT32_MAX,-18},{-16,-18,INT32_MAX,-17},{-17,-17,INT32_MAX,-16},{-19,-16,INT32_MAX,-15},{-20,-15,INT32_MAX,-14},{-22,-14,INT32_MAX,-13},{-23,-13,INT32_MAX,-12},{-25,-12,INT32_MAX,-11},{-27,-11,INT32_MAX,-10},{-28,-10,INT32_MAX,-9},{-30,-9,INT32_MAX,-8},{-31,-8,INT32_MAX,-7},{-33,-7,INT32_MAX,-6},{-34,-6,INT32_MAX,-5},{-36,-5,INT32_MAX,-4},{-37,-4,INT32_MAX,-3},{-39,-3,INT32_MAX,-2},{-40,-2,INT32_MAX,-1},{-41,-1,INT32_MAX,0},{INT32_MIN,0,INT32_MAX,INT32_MAX},
{INT32_MIN,-83,-41,-82},{INT32_MIN,-82,-39,-81},{INT32_MIN,-81,-37,-80},{INT32_MIN,-80,-35,-79},{INT32_MIN,-79,-33,-78},{INT32_MIN,-78,-31,-77},{INT32_MIN,-77,-29,-76},{INT32_MIN,-76,-27,-75},{INT32_MIN,-75,-25,-74},{INT32_MIN,-74,-23,-73},{INT32_MIN,-73,-21,-72},{INT32_MIN,-72,-19,-71},{INT32_MIN,-71,-17,-70},{INT32_MIN,-70,-15,-69},{INT32_MIN,-69,-13,-68},{INT32_MIN,-68,-11,-67},{INT32_MIN,-67,-9,-66},{INT32_MIN,-66,-7,-65},{INT32_MIN,-65,-5,-64},{INT32_MIN,-64,-3,-63},{INT32_MIN,-63,-2,-26},{INT32_MIN,-26,-4,-25},{INT32_MIN,-25,-5,-24},{INT32_MIN,-24,-7,-23},{INT32_MIN,-23,-8,-22},{INT32_MIN,-22,-10,-21},{INT32_MIN,-21,-11,-20},{INT32_MIN,-20,-13,-19},{INT32_MIN,-19,-14,-18},{INT32_MIN,-18,-16,-17},{INT32_MIN,-17,-17,-16},{INT32_MIN,-16,-19,-15},{INT32_MIN,-15,-20,-14},{INT32_MIN,-14,-22,-13},{INT32_MIN,-13,-23,-12},{INT32_MIN,-12,-25,-11},{INT32_MIN,-11,-27,-10},{INT32_MIN,-10,-28,-9},{INT32_MIN,-9,-30,-8},{INT32_MIN,-8,-31,-7},{INT32_MIN,-7,-33,-6},{INT32_MIN,-6,-34,-5},{INT32_MIN,-5,-36,-4},{INT32_MIN,-4,-37,-3},{INT32_MIN,-3,-39,-2},{INT32_MIN,-2,-40,-1},{INT32_MIN,-1,-41,0},
{INT32_MIN,INT32_MIN,INT32_MAX,-83},{-41,-83,INT32_MAX,-82},{-39,-82,INT32_MAX,-81},{-37,-81,INT32_MAX,-80},{-35,-80,INT32_MAX,-79},{-33,-79,INT32_MAX,-78},{-31,-78,INT32_MAX,-77},{-29,-77,INT32_MAX,-76},{-27,-76,INT32_MAX,-75},{-25,-75,INT32_MAX,-74},{-23,-74,INT32_MAX,-73},{-21,-73,INT32_MAX,-72},{-19,-72,INT32_MAX,-71},{-17,-71,INT32_MAX,-70},{-15,-70,INT32_MAX,-69},{-13,-69,INT32_MAX,-68},{-11,-68,INT32_MAX,-67},{-9,-67,INT32_MAX,-66},{-7,-66,INT32_MAX,-65},{-5,-65,INT32_MAX,-64},{-3,-64,INT32_MAX,-63},{-2,-63,INT32_MAX,-40},
};

mask_t corkscrew_right_masks[]={
{0,21,0,0,corkscrew_right_rects},{TRACK_MASK_TRANSFER_NEXT,21,-32,40,corkscrew_right_rects+21},{TRACK_MASK_DIFFERENCE,1,-64,48,corkscrew_right_rects+42},
{0,33,0,0,corkscrew_right_rects+43},{0,33,-32,8,corkscrew_right_rects+76},{0,1,0,16,corkscrew_right_rects+109},
{TRACK_MASK_TRANSFER_NEXT,1,0,0,corkscrew_right_rects+110},{TRACK_MASK_DIFFERENCE,33,32,8,corkscrew_right_rects+111},{0,33,64,48,corkscrew_right_rects+144},
{TRACK_MASK_TRANSFER_NEXT,28,0,0,corkscrew_right_rects+177},{TRACK_MASK_DIFFERENCE,47,32,40,corkscrew_right_rects+205},{0,22,0,80,corkscrew_right_rects+252},
};

const track_section_t corkscrew_right={TRACK_NO_SUPPORTS|TRACK_OFFSET_SPRITE_MASK|TRACK_SPECIAL_CORKSCREW_RIGHT,corkscrew_right_curve,CORKSCREW_LENGTH,{{VIEW_NEEDS_TRACK_MASK,3,corkscrew_right_masks},{0,3,corkscrew_right_masks+3},{VIEW_NEEDS_TRACK_MASK,3,corkscrew_right_masks+6},{VIEW_NEEDS_TRACK_MASK,3,corkscrew_right_masks+9}}};


rect_t large_corkscrew_left_rects[]=
{
//First angle
{INT32_MIN,-64,0,-16},{INT32_MIN,-16,2,-15},{INT32_MIN,-15,4,-14},{INT32_MIN,-14,6,-13},{INT32_MIN,-13,8,-12},{INT32_MIN,-12,10,-11},{INT32_MIN,-11,12,-10},{INT32_MIN,-10,14,-9},{INT32_MIN,-9,16,-8},{INT32_MIN,-8,18,-7},{INT32_MIN,-7,20,-6},{INT32_MIN,-6,22,-5},{INT32_MIN,-5,24,-4},{INT32_MIN,-4,26,-3},{INT32_MIN,-3,28,-2},{INT32_MIN,-2,30,-1},{INT32_MIN,-1,32,INT32_MAX},
{0,-48,2,-47},{0,-47,4,-46},{0,-46,6,-45},{0,-45,8,-44},{0,-44,10,-43},{0,-43,12,-42},{0,-42,14,-41},{0,-41,16,-40},{0,-40,18,-39},{0,-39,20,-38},{0,-38,22,-37},{0,-37,24,-36},{0,-36,26,-35},{0,-35,28,-34},{0,-34,30,-33},{0,-33,32,-32},{0,-32,34,-31},{0,-31,36,-30},{0,-30,38,-29},{0,-29,40,-28},{0,-28,42,-27},{0,-27,44,-26},{0,-26,46,-25},{0,-25,48,-24},{0,-24,50,-23},{0,-23,52,-22},{0,-22,54,-21},{0,-21,56,-20},{0,-20,58,-19},{0,-19,60,-18},{0,-18,62,-17},{0,-17,64,-16},{2,-16,64,-15},{4,-15,64,-14},{6,-14,64,-13},{8,-13,64,-12},{10,-12,64,-11},{12,-11,64,-10},{14,-10,64,-9},{16,-9,64,-8},{18,-8,64,-7},{20,-7,64,-6},{22,-6,64,-5},{24,-5,64,-4},{26,-4,64,-3},{28,-3,64,-2},{30,-2,64,-1},{32,-1,64,INT32_MAX},
{64,INT32_MIN,INT32_MAX,-110},{62,-110,INT32_MAX,-109},{60,-109,INT32_MAX,-108},{58,-108,INT32_MAX,-107},{56,-107,INT32_MAX,-106},{54,-106,INT32_MAX,-105},{52,-105,INT32_MAX,-104},{50,-104,INT32_MAX,-103},{48,-103,INT32_MAX,-102},{46,-102,INT32_MAX,-101},{44,-101,INT32_MAX,-100},{42,-100,INT32_MAX,-99},{40,-99,INT32_MAX,-98},{38,-98,INT32_MAX,-97},{36,-97,INT32_MAX,-96},{34,-96,INT32_MAX,-95},{32,-95,INT32_MAX,-94},{30,-94,INT32_MAX,-93},{28,-93,INT32_MAX,-92},{26,-92,INT32_MAX,-91},{24,-91,INT32_MAX,-90},{22,-90,INT32_MAX,-89},{20,-89,INT32_MAX,-88},{18,-88,INT32_MAX,-87},{16,-87,INT32_MAX,-86},{14,-86,INT32_MAX,-85},{12,-85,INT32_MAX,-84},{10,-84,INT32_MAX,-83},{8,-83,INT32_MAX,-82},{6,-82,INT32_MAX,-81},{4,-81,INT32_MAX,-80},{2,-80,INT32_MAX,-79},{0,-79,INT32_MAX,-48},{2,-48,INT32_MAX,-47},{4,-47,INT32_MAX,-46},{6,-46,INT32_MAX,-45},{8,-45,INT32_MAX,-44},{10,-44,INT32_MAX,-43},{12,-43,INT32_MAX,-42},{14,-42,INT32_MAX,-41},{16,-41,INT32_MAX,-40},{18,-40,INT32_MAX,-39},{20,-39,INT32_MAX,-38},{22,-38,INT32_MAX,-37},{24,-37,INT32_MAX,-36},{26,-36,INT32_MAX,-35},{28,-35,INT32_MAX,-34},{30,-34,INT32_MAX,-33},{32,-33,INT32_MAX,-32},{34,-32,INT32_MAX,-31},{36,-31,INT32_MAX,-30},{38,-30,INT32_MAX,-29},{40,-29,INT32_MAX,-28},{42,-28,INT32_MAX,-27},{44,-27,INT32_MAX,-26},{46,-26,INT32_MAX,-25},{48,-25,INT32_MAX,-24},{50,-24,INT32_MAX,-23},{52,-23,INT32_MAX,-22},{54,-22,INT32_MAX,-21},{56,-21,INT32_MAX,-20},{58,-20,INT32_MAX,-19},{60,-19,INT32_MAX,-18},{62,-18,INT32_MAX,-17},{64,-17,INT32_MAX,INT32_MAX},
{32,INT32_MIN,64,-126},{30,-126,64,-125},{28,-125,64,-124},{26,-124,64,-123},{24,-123,64,-122},{22,-122,64,-121},{20,-121,64,-120},{18,-120,64,-119},{16,-119,64,-118},{14,-118,64,-117},{12,-117,64,-116},{10,-116,64,-115},{8,-115,64,-114},{6,-114,64,-113},{4,-113,64,-112},{2,-112,64,-111},{0,-111,64,-110},{0,-110,62,-109},{0,-109,60,-108},{0,-108,58,-107},{0,-107,56,-106},{0,-106,54,-105},{0,-105,52,-104},{0,-104,50,-103},{0,-103,48,-102},{0,-102,46,-101},{0,-101,44,-100},{0,-100,42,-99},{0,-99,40,-98},{0,-98,38,-97},{0,-97,36,-96},{0,-96,34,-95},{0,-95,32,-94},{0,-94,30,-93},{0,-93,28,-92},{0,-92,26,-91},{0,-91,24,-90},{0,-90,22,-89},{0,-89,20,-88},{0,-88,18,-87},{0,-87,16,-86},{0,-86,14,-85},{0,-85,12,-84},{0,-84,10,-83},{0,-83,8,-82},{0,-82,6,-81},{0,-81,4,-80},{0,-80,2,-79},
{INT32_MIN,INT32_MIN,32,-126},{INT32_MIN,-126,30,-125},{INT32_MIN,-125,28,-124},{INT32_MIN,-124,26,-123},{INT32_MIN,-123,24,-122},{INT32_MIN,-122,22,-121},{INT32_MIN,-121,20,-120},{INT32_MIN,-120,18,-119},{INT32_MIN,-119,16,-118},{INT32_MIN,-118,14,-117},{INT32_MIN,-117,12,-116},{INT32_MIN,-116,10,-115},{INT32_MIN,-115,8,-114},{INT32_MIN,-114,6,-113},{INT32_MIN,-113,4,-112},{INT32_MIN,-112,2,-111},{INT32_MIN,-111,0,-64},
//Second angle
{INT32_MIN,INT32_MIN,0,INT32_MAX},
{0,INT32_MIN,32,INT32_MAX},
{32,INT32_MIN,64,-63},{32,-63,65,-62},{32,-62,66,-61},{32,-61,67,-60},{32,-60,68,-59},{32,-59,69,-58},{32,-58,70,-57},{32,-57,71,-56},{32,-56,72,-55},{32,-55,73,-54},{32,-54,74,-53},{32,-53,75,-52},{32,-52,76,-51},{32,-51,77,-50},{32,-50,78,-49},{32,-49,79,-48},{32,-48,80,-47},{32,-47,81,-46},{32,-46,82,-45},{32,-45,83,-44},{32,-44,84,-43},{32,-43,85,-42},{32,-42,86,-41},{32,-41,87,-40},{32,-40,88,-39},{32,-39,89,-38},{32,-38,90,-37},{32,-37,91,-36},{32,-36,92,-35},{32,-35,93,-34},{32,-34,94,-33},{32,-33,95,-32},{32,-32,96,INT32_MAX},
{64,INT32_MIN,96,-105},{64,-105,97,-104},{64,-104,98,-103},{64,-103,99,-102},{64,-102,100,-101},{64,-101,101,-100},{64,-100,102,-99},{64,-99,103,-98},{64,-98,104,-97},{64,-97,105,-96},{64,-96,106,-95},{64,-95,107,-94},{64,-94,108,-93},{64,-93,109,-92},{64,-92,110,-91},{64,-91,111,-90},{64,-90,112,-89},{64,-89,113,-88},{64,-88,114,-87},{64,-87,115,-86},{64,-86,116,-85},{64,-85,117,-84},{64,-84,118,-83},{64,-83,119,-82},{64,-82,120,-81},{64,-81,121,-80},{64,-80,122,-79},{64,-79,123,-78},{64,-78,124,-77},{64,-77,125,-76},{64,-76,126,-75},{64,-75,127,-74},{64,-74,128,-63},{65,-63,128,-62},{66,-62,128,-61},{67,-61,128,-60},{68,-60,128,-59},{69,-59,128,-58},{70,-58,128,-57},{71,-57,128,-56},{72,-56,128,-55},{73,-55,128,-54},{74,-54,128,-53},{75,-53,128,-52},{76,-52,128,-51},{77,-51,128,-50},{78,-50,128,-49},{79,-49,128,-48},{80,-48,128,-47},{81,-47,128,-46},{82,-46,128,-45},{83,-45,128,-44},{84,-44,128,-43},{85,-43,128,-42},{86,-42,128,-41},{87,-41,128,-40},{88,-40,128,-39},{89,-39,128,-38},{90,-38,128,-37},{91,-37,128,-36},{92,-36,128,-35},{93,-35,128,-34},{94,-34,128,-33},{95,-33,128,-32},{96,-32,128,INT32_MAX},
{96,INT32_MIN,INT32_MAX,-105},{97,-105,INT32_MAX,-104},{98,-104,INT32_MAX,-103},{99,-103,INT32_MAX,-102},{100,-102,INT32_MAX,-101},{101,-101,INT32_MAX,-100},{102,-100,INT32_MAX,-99},{103,-99,INT32_MAX,-98},{104,-98,INT32_MAX,-97},{105,-97,INT32_MAX,-96},{106,-96,INT32_MAX,-95},{107,-95,INT32_MAX,-94},{108,-94,INT32_MAX,-93},{109,-93,INT32_MAX,-92},{110,-92,INT32_MAX,-91},{111,-91,INT32_MAX,-90},{112,-90,INT32_MAX,-89},{113,-89,INT32_MAX,-88},{114,-88,INT32_MAX,-87},{115,-87,INT32_MAX,-86},{116,-86,INT32_MAX,-85},{117,-85,INT32_MAX,-84},{118,-84,INT32_MAX,-83},{119,-83,INT32_MAX,-82},{120,-82,INT32_MAX,-81},{121,-81,INT32_MAX,-80},{122,-80,INT32_MAX,-79},{123,-79,INT32_MAX,-78},{124,-78,INT32_MAX,-77},{125,-77,INT32_MAX,-76},{126,-76,INT32_MAX,-75},{127,-75,INT32_MAX,-74},{128,-74,INT32_MAX,INT32_MAX},
//Third angle
{-4,-16,INT32_MAX,INT32_MAX},
{-33,-16,-4,-15},{-34,-15,-4,-14},{-35,-14,-4,-13},{-36,-13,-4,-12},{-37,-12,-4,-11},{-38,-11,-4,-10},{-39,-10,-4,-9},{-40,-9,-4,-8},{-41,-8,-4,-7},{-42,-7,-4,-6},{-43,-6,-4,-5},{-44,-5,-4,-4},{-45,-4,-4,-3},{-46,-3,-4,-2},{-47,-2,-4,-1},{-48,-1,-4,0},{-49,0,-4,1},{-50,1,-4,2},{-51,2,-4,3},{-52,3,-4,4},{-53,4,-4,5},{-54,5,-4,6},{-55,6,-4,7},{-56,7,-4,8},{-57,8,-4,9},{-58,9,-4,10},{-59,10,-4,11},{-60,11,-4,12},{-61,12,-4,13},{-62,13,-4,14},{-63,14,-4,INT32_MAX},
{INT32_MIN,INT32_MIN,-32,-16},{INT32_MIN,-16,-33,-15},{INT32_MIN,-15,-34,-14},{INT32_MIN,-14,-35,-13},{INT32_MIN,-13,-36,-12},{INT32_MIN,-12,-37,-11},{INT32_MIN,-11,-38,-10},{INT32_MIN,-10,-39,-9},{INT32_MIN,-9,-40,-8},{INT32_MIN,-8,-41,-7},{INT32_MIN,-7,-42,-6},{INT32_MIN,-6,-43,-5},{INT32_MIN,-5,-44,-4},{INT32_MIN,-4,-45,-3},{INT32_MIN,-3,-46,-2},{INT32_MIN,-2,-47,-1},{INT32_MIN,-1,-48,0},{INT32_MIN,0,-49,1},{INT32_MIN,1,-50,2},{INT32_MIN,2,-51,3},{INT32_MIN,3,-52,4},{INT32_MIN,4,-53,5},{INT32_MIN,5,-54,6},{INT32_MIN,6,-55,7},{INT32_MIN,7,-56,8},{INT32_MIN,8,-57,9},{INT32_MIN,9,-58,10},{INT32_MIN,10,-59,11},{INT32_MIN,11,-60,12},{INT32_MIN,12,-61,13},{INT32_MIN,13,-62,14},{INT32_MIN,14,-63,INT32_MAX},
{-32,INT32_MIN,-4,-16},
{-4,INT32_MIN,INT32_MAX,-16},
//Fourth angle
{0,INT32_MIN,INT32_MAX,-15},{-2,-15,INT32_MAX,-14},{-4,-14,INT32_MAX,-13},{-6,-13,INT32_MAX,-12},{-8,-12,INT32_MAX,-11},{-10,-11,INT32_MAX,-10},{-12,-10,INT32_MAX,-9},{-14,-9,INT32_MAX,-8},{-16,-8,INT32_MAX,-7},{-18,-7,INT32_MAX,-6},{-20,-6,INT32_MAX,-5},{-22,-5,INT32_MAX,-4},{-24,-4,INT32_MAX,-3},{-26,-3,INT32_MAX,-2},{-28,-2,INT32_MAX,-1},{-30,-1,INT32_MAX,0},{-32,0,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,0,-62},{-33,-62,0,-61},{-34,-61,0,-60},{-35,-60,0,-59},{-36,-59,0,-58},{-37,-58,0,-57},{-38,-57,0,-56},{-39,-56,0,-55},{-40,-55,0,-54},{-41,-54,0,-53},{-42,-53,0,-52},{-43,-52,0,-51},{-44,-51,0,-50},{-45,-50,0,-49},{-46,-49,0,-48},{-47,-48,0,-47},{-48,-47,0,-46},{-49,-46,0,-45},{-50,-45,0,-44},{-51,-44,0,-43},{-52,-43,0,-42},{-53,-42,0,-41},{-54,-41,0,-40},{-55,-40,0,-39},{-56,-39,0,-38},{-57,-38,0,-37},{-58,-37,0,-36},{-59,-36,0,-35},{-60,-35,0,-34},{-61,-34,0,-33},{-62,-33,0,-32},{-63,-32,0,-31},{-64,-31,0,-15},{-64,-15,-2,-14},{-64,-14,-4,-13},{-64,-13,-6,-12},{-64,-12,-8,-11},{-64,-11,-10,-10},{-64,-10,-12,-9},{-64,-9,-14,-8},{-64,-8,-16,-7},{-64,-7,-18,-6},{-64,-6,-20,-5},{-64,-5,-22,-4},{-64,-4,-24,-3},{-64,-3,-26,-2},{-64,-2,-28,-1},{-64,-1,-30,0},{-64,0,-32,INT32_MAX},
{-64,INT32_MIN,-32,-106},{-65,-106,-32,-105},{-66,-105,-32,-104},{-67,-104,-32,-103},{-68,-103,-32,-102},{-69,-102,-32,-101},{-70,-101,-32,-100},{-71,-100,-32,-99},{-72,-99,-32,-98},{-73,-98,-32,-97},{-74,-97,-32,-96},{-75,-96,-32,-95},{-76,-95,-32,-94},{-77,-94,-32,-93},{-78,-93,-32,-92},{-79,-92,-32,-91},{-80,-91,-32,-90},{-81,-90,-32,-89},{-82,-89,-32,-88},{-83,-88,-32,-87},{-84,-87,-32,-86},{-85,-86,-32,-85},{-86,-85,-32,-84},{-87,-84,-32,-83},{-88,-83,-32,-82},{-89,-82,-32,-81},{-90,-81,-32,-80},{-91,-80,-32,-79},{-92,-79,-32,-78},{-93,-78,-32,-77},{-94,-77,-32,-76},{-95,-76,-32,-75},{-96,-75,-32,-62},{-96,-62,-33,-61},{-96,-61,-34,-60},{-96,-60,-35,-59},{-96,-59,-36,-58},{-96,-58,-37,-57},{-96,-57,-38,-56},{-96,-56,-39,-55},{-96,-55,-40,-54},{-96,-54,-41,-53},{-96,-53,-42,-52},{-96,-52,-43,-51},{-96,-51,-44,-50},{-96,-50,-45,-49},{-96,-49,-46,-48},{-96,-48,-47,-47},{-96,-47,-48,-46},{-96,-46,-49,-45},{-96,-45,-50,-44},{-96,-44,-51,-43},{-96,-43,-52,-42},{-96,-42,-53,-41},{-96,-41,-54,-40},{-96,-40,-55,-39},{-96,-39,-56,-38},{-96,-38,-57,-37},{-96,-37,-58,-36},{-96,-36,-59,-35},{-96,-35,-60,-34},{-96,-34,-61,-33},{-96,-33,-62,-32},{-96,-32,-63,-31},{-96,-31,-64,INT32_MAX},
{-112,INT32_MIN,-64,-106},{-112,-106,-65,-105},{-112,-105,-66,-104},{-112,-104,-67,-103},{-112,-103,-68,-102},{-112,-102,-69,-101},{-112,-101,-70,-100},{-112,-100,-71,-99},{-112,-99,-72,-98},{-112,-98,-73,-97},{-112,-97,-74,-96},{-112,-96,-75,-95},{-112,-95,-76,-94},{-112,-94,-77,-93},{-112,-93,-78,-92},{-112,-92,-79,-91},{-112,-91,-80,-90},{-112,-90,-81,-89},{-112,-89,-82,-88},{-112,-88,-83,-87},{-112,-87,-84,-86},{-112,-86,-85,-85},{-112,-85,-86,-84},{-112,-84,-87,-83},{-112,-83,-88,-82},{-112,-82,-89,-81},{-112,-81,-90,-80},{-112,-80,-91,-79},{-112,-79,-92,-78},{-112,-78,-93,-77},{-112,-77,-94,-76},{-112,-76,-95,-75},{-112,-75,-96,INT32_MAX},
{INT32_MIN,INT32_MIN,-112,INT32_MAX},
};
mask_t large_corkscrew_left_masks[]={
{0,17,0,0,large_corkscrew_left_rects},{TRACK_MASK_TRANSFER_NEXT,48,-32,24,large_corkscrew_left_rects+17},{TRACK_MASK_DIFFERENCE,65,-64,64,large_corkscrew_left_rects+65},{0,48,-32,96,large_corkscrew_left_rects+130},{0,17,0,152,large_corkscrew_left_rects+178},
{TRACK_MASK_TRANSFER_NEXT,1,0,0,large_corkscrew_left_rects+195},{TRACK_MASK_DIFFERENCE,1,-32,-8,large_corkscrew_left_rects+196},{0,33,-64,0,large_corkscrew_left_rects+197},{0,65,-96,32,large_corkscrew_left_rects+230},{0,33,-128,88,large_corkscrew_left_rects+295},
{0,1,0,0,large_corkscrew_left_rects+328},{TRACK_MASK_TRANSFER_NEXT,31,32,-8,large_corkscrew_left_rects+329},{TRACK_MASK_DIFFERENCE,32,64,0,large_corkscrew_left_rects+360},{0,1,32,0,large_corkscrew_left_rects+392},{0,1,0,24,large_corkscrew_left_rects+393},
{0,17,0,0,large_corkscrew_left_rects+394},{0,49,32,24,large_corkscrew_left_rects+411},{0,65,64,64,large_corkscrew_left_rects+460},{TRACK_MASK_TRANSFER_NEXT,33,96,64,large_corkscrew_left_rects+525},{TRACK_MASK_DIFFERENCE,1,128,88,large_corkscrew_left_rects+558}
};


const track_section_t large_corkscrew_left={TRACK_NO_SUPPORTS|TRACK_OFFSET_SPRITE_MASK|TRACK_SPECIAL_CORKSCREW_LEFT,large_corkscrew_left_curve,LARGE_CORKSCREW_LENGTH,{{VIEW_NEEDS_TRACK_MASK,5,large_corkscrew_left_masks},{VIEW_NEEDS_TRACK_MASK,5,large_corkscrew_left_masks+5},{VIEW_NEEDS_TRACK_MASK,5,large_corkscrew_left_masks+10},{VIEW_NEEDS_TRACK_MASK,5,large_corkscrew_left_masks+15}}};

rect_t large_corkscrew_right_rects[]=
{
//First angle
{INT32_MIN,INT32_MIN,0,-15},{INT32_MIN,-15,2,-14},{INT32_MIN,-14,4,-13},{INT32_MIN,-13,6,-12},{INT32_MIN,-12,8,-11},{INT32_MIN,-11,10,-10},{INT32_MIN,-10,12,-9},{INT32_MIN,-9,14,-8},{INT32_MIN,-8,16,-7},{INT32_MIN,-7,18,-6},{INT32_MIN,-6,20,-5},{INT32_MIN,-5,22,-4},{INT32_MIN,-4,24,-3},{INT32_MIN,-3,26,-2},{INT32_MIN,-2,28,-1},{INT32_MIN,-1,30,0},{INT32_MIN,0,32,INT32_MAX},
{0,INT32_MIN,32,-62},{0,-62,33,-61},{0,-61,34,-60},{0,-60,35,-59},{0,-59,36,-58},{0,-58,37,-57},{0,-57,38,-56},{0,-56,39,-55},{0,-55,40,-54},{0,-54,41,-53},{0,-53,42,-52},{0,-52,43,-51},{0,-51,44,-50},{0,-50,45,-49},{0,-49,46,-48},{0,-48,47,-47},{0,-47,48,-46},{0,-46,49,-45},{0,-45,50,-44},{0,-44,51,-43},{0,-43,52,-42},{0,-42,53,-41},{0,-41,54,-40},{0,-40,55,-39},{0,-39,56,-38},{0,-38,57,-37},{0,-37,58,-36},{0,-36,59,-35},{0,-35,60,-34},{0,-34,61,-33},{0,-33,62,-32},{0,-32,63,-31},{0,-31,64,-15},{2,-15,64,-14},{4,-14,64,-13},{6,-13,64,-12},{8,-12,64,-11},{10,-11,64,-10},{12,-10,64,-9},{14,-9,64,-8},{16,-8,64,-7},{18,-7,64,-6},{20,-6,64,-5},{22,-5,64,-4},{24,-4,64,-3},{26,-3,64,-2},{28,-2,64,-1},{30,-1,64,0},{32,0,64,INT32_MAX},
{32,INT32_MIN,64,-106},{32,-106,65,-105},{32,-105,66,-104},{32,-104,67,-103},{32,-103,68,-102},{32,-102,69,-101},{32,-101,70,-100},{32,-100,71,-99},{32,-99,72,-98},{32,-98,73,-97},{32,-97,74,-96},{32,-96,75,-95},{32,-95,76,-94},{32,-94,77,-93},{32,-93,78,-92},{32,-92,79,-91},{32,-91,80,-90},{32,-90,81,-89},{32,-89,82,-88},{32,-88,83,-87},{32,-87,84,-86},{32,-86,85,-85},{32,-85,86,-84},{32,-84,87,-83},{32,-83,88,-82},{32,-82,89,-81},{32,-81,90,-80},{32,-80,91,-79},{32,-79,92,-78},{32,-78,93,-77},{32,-77,94,-76},{32,-76,95,-75},{32,-75,96,-62},{33,-62,96,-61},{34,-61,96,-60},{35,-60,96,-59},{36,-59,96,-58},{37,-58,96,-57},{38,-57,96,-56},{39,-56,96,-55},{40,-55,96,-54},{41,-54,96,-53},{42,-53,96,-52},{43,-52,96,-51},{44,-51,96,-50},{45,-50,96,-49},{46,-49,96,-48},{47,-48,96,-47},{48,-47,96,-46},{49,-46,96,-45},{50,-45,96,-44},{51,-44,96,-43},{52,-43,96,-42},{53,-42,96,-41},{54,-41,96,-40},{55,-40,96,-39},{56,-39,96,-38},{57,-38,96,-37},{58,-37,96,-36},{59,-36,96,-35},{60,-35,96,-34},{61,-34,96,-33},{62,-33,96,-32},{63,-32,96,-31},{64,-31,96,INT32_MAX},
{64,INT32_MIN,112,-106},{65,-106,112,-105},{66,-105,112,-104},{67,-104,112,-103},{68,-103,112,-102},{69,-102,112,-101},{70,-101,112,-100},{71,-100,112,-99},{72,-99,112,-98},{73,-98,112,-97},{74,-97,112,-96},{75,-96,112,-95},{76,-95,112,-94},{77,-94,112,-93},{78,-93,112,-92},{79,-92,112,-91},{80,-91,112,-90},{81,-90,112,-89},{82,-89,112,-88},{83,-88,112,-87},{84,-87,112,-86},{85,-86,112,-85},{86,-85,112,-84},{87,-84,112,-83},{88,-83,112,-82},{89,-82,112,-81},{90,-81,112,-80},{91,-80,112,-79},{92,-79,112,-78},{93,-78,112,-77},{94,-77,112,-76},{95,-76,112,-75},{96,-75,112,INT32_MAX},
{112,INT32_MIN,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,-16,4,INT32_MAX},
{4,-16,33,-15},{4,-15,34,-14},{4,-14,35,-13},{4,-13,36,-12},{4,-12,37,-11},{4,-11,38,-10},{4,-10,39,-9},{4,-9,40,-8},{4,-8,41,-7},{4,-7,42,-6},{4,-6,43,-5},{4,-5,44,-4},{4,-4,45,-3},{4,-3,46,-2},{4,-2,47,-1},{4,-1,48,0},{4,0,49,1},{4,1,50,2},{4,2,51,3},{4,3,52,4},{4,4,53,5},{4,5,54,6},{4,6,55,7},{4,7,56,8},{4,8,57,9},{4,9,58,10},{4,10,59,11},{4,11,60,12},{4,12,61,13},{4,13,62,14},{4,14,63,INT32_MAX},
{32,INT32_MIN,INT32_MAX,-16},{33,-16,INT32_MAX,-15},{34,-15,INT32_MAX,-14},{35,-14,INT32_MAX,-13},{36,-13,INT32_MAX,-12},{37,-12,INT32_MAX,-11},{38,-11,INT32_MAX,-10},{39,-10,INT32_MAX,-9},{40,-9,INT32_MAX,-8},{41,-8,INT32_MAX,-7},{42,-7,INT32_MAX,-6},{43,-6,INT32_MAX,-5},{44,-5,INT32_MAX,-4},{45,-4,INT32_MAX,-3},{46,-3,INT32_MAX,-2},{47,-2,INT32_MAX,-1},{48,-1,INT32_MAX,0},{49,0,INT32_MAX,1},{50,1,INT32_MAX,2},{51,2,INT32_MAX,3},{52,3,INT32_MAX,4},{53,4,INT32_MAX,5},{54,5,INT32_MAX,6},{55,6,INT32_MAX,7},{56,7,INT32_MAX,8},{57,8,INT32_MAX,9},{58,9,INT32_MAX,10},{59,10,INT32_MAX,11},{60,11,INT32_MAX,12},{61,12,INT32_MAX,13},{62,13,INT32_MAX,14},{63,14,INT32_MAX,INT32_MAX},
{4,INT32_MIN,32,-16},
{INT32_MIN,INT32_MIN,4,-16},
//Third angle
{0,INT32_MIN,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,0,INT32_MAX},
{-64,INT32_MIN,-32,-63},{-65,-63,-32,-62},{-66,-62,-32,-61},{-67,-61,-32,-60},{-68,-60,-32,-59},{-69,-59,-32,-58},{-70,-58,-32,-57},{-71,-57,-32,-56},{-72,-56,-32,-55},{-73,-55,-32,-54},{-74,-54,-32,-53},{-75,-53,-32,-52},{-76,-52,-32,-51},{-77,-51,-32,-50},{-78,-50,-32,-49},{-79,-49,-32,-48},{-80,-48,-32,-47},{-81,-47,-32,-46},{-82,-46,-32,-45},{-83,-45,-32,-44},{-84,-44,-32,-43},{-85,-43,-32,-42},{-86,-42,-32,-41},{-87,-41,-32,-40},{-88,-40,-32,-39},{-89,-39,-32,-38},{-90,-38,-32,-37},{-91,-37,-32,-36},{-92,-36,-32,-35},{-93,-35,-32,-34},{-94,-34,-32,-33},{-95,-33,-32,-32},{-96,-32,-32,INT32_MAX},
{-96,INT32_MIN,-64,-105},{-97,-105,-64,-104},{-98,-104,-64,-103},{-99,-103,-64,-102},{-100,-102,-64,-101},{-101,-101,-64,-100},{-102,-100,-64,-99},{-103,-99,-64,-98},{-104,-98,-64,-97},{-105,-97,-64,-96},{-106,-96,-64,-95},{-107,-95,-64,-94},{-108,-94,-64,-93},{-109,-93,-64,-92},{-110,-92,-64,-91},{-111,-91,-64,-90},{-112,-90,-64,-89},{-113,-89,-64,-88},{-114,-88,-64,-87},{-115,-87,-64,-86},{-116,-86,-64,-85},{-117,-85,-64,-84},{-118,-84,-64,-83},{-119,-83,-64,-82},{-120,-82,-64,-81},{-121,-81,-64,-80},{-122,-80,-64,-79},{-123,-79,-64,-78},{-124,-78,-64,-77},{-125,-77,-64,-76},{-126,-76,-64,-75},{-127,-75,-64,-74},{-128,-74,-64,-63},{-128,-63,-65,-62},{-128,-62,-66,-61},{-128,-61,-67,-60},{-128,-60,-68,-59},{-128,-59,-69,-58},{-128,-58,-70,-57},{-128,-57,-71,-56},{-128,-56,-72,-55},{-128,-55,-73,-54},{-128,-54,-74,-53},{-128,-53,-75,-52},{-128,-52,-76,-51},{-128,-51,-77,-50},{-128,-50,-78,-49},{-128,-49,-79,-48},{-128,-48,-80,-47},{-128,-47,-81,-46},{-128,-46,-82,-45},{-128,-45,-83,-44},{-128,-44,-84,-43},{-128,-43,-85,-42},{-128,-42,-86,-41},{-128,-41,-87,-40},{-128,-40,-88,-39},{-128,-39,-89,-38},{-128,-38,-90,-37},{-128,-37,-91,-36},{-128,-36,-92,-35},{-128,-35,-93,-34},{-128,-34,-94,-33},{-128,-33,-95,-32},{-128,-32,-96,INT32_MAX},
{INT32_MIN,INT32_MIN,-96,-105},{INT32_MIN,-105,-97,-104},{INT32_MIN,-104,-98,-103},{INT32_MIN,-103,-99,-102},{INT32_MIN,-102,-100,-101},{INT32_MIN,-101,-101,-100},{INT32_MIN,-100,-102,-99},{INT32_MIN,-99,-103,-98},{INT32_MIN,-98,-104,-97},{INT32_MIN,-97,-105,-96},{INT32_MIN,-96,-106,-95},{INT32_MIN,-95,-107,-94},{INT32_MIN,-94,-108,-93},{INT32_MIN,-93,-109,-92},{INT32_MIN,-92,-110,-91},{INT32_MIN,-91,-111,-90},{INT32_MIN,-90,-112,-89},{INT32_MIN,-89,-113,-88},{INT32_MIN,-88,-114,-87},{INT32_MIN,-87,-115,-86},{INT32_MIN,-86,-116,-85},{INT32_MIN,-85,-117,-84},{INT32_MIN,-84,-118,-83},{INT32_MIN,-83,-119,-82},{INT32_MIN,-82,-120,-81},{INT32_MIN,-81,-121,-80},{INT32_MIN,-80,-122,-79},{INT32_MIN,-79,-123,-78},{INT32_MIN,-78,-124,-77},{INT32_MIN,-77,-125,-76},{INT32_MIN,-76,-126,-75},{INT32_MIN,-75,-127,-74},{INT32_MIN,-74,-128,INT32_MAX},
//Fourth angle
{0,-64,INT32_MAX,-16},{-2,-16,INT32_MAX,-15},{-4,-15,INT32_MAX,-14},{-6,-14,INT32_MAX,-13},{-8,-13,INT32_MAX,-12},{-10,-12,INT32_MAX,-11},{-12,-11,INT32_MAX,-10},{-14,-10,INT32_MAX,-9},{-16,-9,INT32_MAX,-8},{-18,-8,INT32_MAX,-7},{-20,-7,INT32_MAX,-6},{-22,-6,INT32_MAX,-5},{-24,-5,INT32_MAX,-4},{-26,-4,INT32_MAX,-3},{-28,-3,INT32_MAX,-2},{-30,-2,INT32_MAX,-1},{-32,-1,INT32_MAX,INT32_MAX},
{-2,-48,0,-47},{-4,-47,0,-46},{-6,-46,0,-45},{-8,-45,0,-44},{-10,-44,0,-43},{-12,-43,0,-42},{-14,-42,0,-41},{-16,-41,0,-40},{-18,-40,0,-39},{-20,-39,0,-38},{-22,-38,0,-37},{-24,-37,0,-36},{-26,-36,0,-35},{-28,-35,0,-34},{-30,-34,0,-33},{-32,-33,0,-32},{-34,-32,0,-31},{-36,-31,0,-30},{-38,-30,0,-29},{-40,-29,0,-28},{-42,-28,0,-27},{-44,-27,0,-26},{-46,-26,0,-25},{-48,-25,0,-24},{-50,-24,0,-23},{-52,-23,0,-22},{-54,-22,0,-21},{-56,-21,0,-20},{-58,-20,0,-19},{-60,-19,0,-18},{-62,-18,0,-17},{-64,-17,0,-16},{-64,-16,-2,-15},{-64,-15,-4,-14},{-64,-14,-6,-13},{-64,-13,-8,-12},{-64,-12,-10,-11},{-64,-11,-12,-10},{-64,-10,-14,-9},{-64,-9,-16,-8},{-64,-8,-18,-7},{-64,-7,-20,-6},{-64,-6,-22,-5},{-64,-5,-24,-4},{-64,-4,-26,-3},{-64,-3,-28,-2},{-64,-2,-30,-1},{-64,-1,-32,INT32_MAX},
{INT32_MIN,INT32_MIN,-64,-110},{INT32_MIN,-110,-62,-109},{INT32_MIN,-109,-60,-108},{INT32_MIN,-108,-58,-107},{INT32_MIN,-107,-56,-106},{INT32_MIN,-106,-54,-105},{INT32_MIN,-105,-52,-104},{INT32_MIN,-104,-50,-103},{INT32_MIN,-103,-48,-102},{INT32_MIN,-102,-46,-101},{INT32_MIN,-101,-44,-100},{INT32_MIN,-100,-42,-99},{INT32_MIN,-99,-40,-98},{INT32_MIN,-98,-38,-97},{INT32_MIN,-97,-36,-96},{INT32_MIN,-96,-34,-95},{INT32_MIN,-95,-32,-94},{INT32_MIN,-94,-30,-93},{INT32_MIN,-93,-28,-92},{INT32_MIN,-92,-26,-91},{INT32_MIN,-91,-24,-90},{INT32_MIN,-90,-22,-89},{INT32_MIN,-89,-20,-88},{INT32_MIN,-88,-18,-87},{INT32_MIN,-87,-16,-86},{INT32_MIN,-86,-14,-85},{INT32_MIN,-85,-12,-84},{INT32_MIN,-84,-10,-83},{INT32_MIN,-83,-8,-82},{INT32_MIN,-82,-6,-81},{INT32_MIN,-81,-4,-80},{INT32_MIN,-80,-2,-79},{INT32_MIN,-79,0,-48},{INT32_MIN,-48,-2,-47},{INT32_MIN,-47,-4,-46},{INT32_MIN,-46,-6,-45},{INT32_MIN,-45,-8,-44},{INT32_MIN,-44,-10,-43},{INT32_MIN,-43,-12,-42},{INT32_MIN,-42,-14,-41},{INT32_MIN,-41,-16,-40},{INT32_MIN,-40,-18,-39},{INT32_MIN,-39,-20,-38},{INT32_MIN,-38,-22,-37},{INT32_MIN,-37,-24,-36},{INT32_MIN,-36,-26,-35},{INT32_MIN,-35,-28,-34},{INT32_MIN,-34,-30,-33},{INT32_MIN,-33,-32,-32},{INT32_MIN,-32,-34,-31},{INT32_MIN,-31,-36,-30},{INT32_MIN,-30,-38,-29},{INT32_MIN,-29,-40,-28},{INT32_MIN,-28,-42,-27},{INT32_MIN,-27,-44,-26},{INT32_MIN,-26,-46,-25},{INT32_MIN,-25,-48,-24},{INT32_MIN,-24,-50,-23},{INT32_MIN,-23,-52,-22},{INT32_MIN,-22,-54,-21},{INT32_MIN,-21,-56,-20},{INT32_MIN,-20,-58,-19},{INT32_MIN,-19,-60,-18},{INT32_MIN,-18,-62,-17},{INT32_MIN,-17,-64,INT32_MAX},
{-64,INT32_MIN,-32,-126},{-64,-126,-30,-125},{-64,-125,-28,-124},{-64,-124,-26,-123},{-64,-123,-24,-122},{-64,-122,-22,-121},{-64,-121,-20,-120},{-64,-120,-18,-119},{-64,-119,-16,-118},{-64,-118,-14,-117},{-64,-117,-12,-116},{-64,-116,-10,-115},{-64,-115,-8,-114},{-64,-114,-6,-113},{-64,-113,-4,-112},{-64,-112,-2,-111},{-64,-111,0,-110},{-62,-110,0,-109},{-60,-109,0,-108},{-58,-108,0,-107},{-56,-107,0,-106},{-54,-106,0,-105},{-52,-105,0,-104},{-50,-104,0,-103},{-48,-103,0,-102},{-46,-102,0,-101},{-44,-101,0,-100},{-42,-100,0,-99},{-40,-99,0,-98},{-38,-98,0,-97},{-36,-97,0,-96},{-34,-96,0,-95},{-32,-95,0,-94},{-30,-94,0,-93},{-28,-93,0,-92},{-26,-92,0,-91},{-24,-91,0,-90},{-22,-90,0,-89},{-20,-89,0,-88},{-18,-88,0,-87},{-16,-87,0,-86},{-14,-86,0,-85},{-12,-85,0,-84},{-10,-84,0,-83},{-8,-83,0,-82},{-6,-82,0,-81},{-4,-81,0,-80},{-2,-80,0,-79},
{-32,INT32_MIN,INT32_MAX,-126},{-30,-126,INT32_MAX,-125},{-28,-125,INT32_MAX,-124},{-26,-124,INT32_MAX,-123},{-24,-123,INT32_MAX,-122},{-22,-122,INT32_MAX,-121},{-20,-121,INT32_MAX,-120},{-18,-120,INT32_MAX,-119},{-16,-119,INT32_MAX,-118},{-14,-118,INT32_MAX,-117},{-12,-117,INT32_MAX,-116},{-10,-116,INT32_MAX,-115},{-8,-115,INT32_MAX,-114},{-6,-114,INT32_MAX,-113},{-4,-113,INT32_MAX,-112},{-2,-112,INT32_MAX,-111},{0,-111,INT32_MAX,-64}
};
mask_t large_corkscrew_right_masks[]={
{0,17,0,0,large_corkscrew_right_rects},{0,49,-32,24,large_corkscrew_right_rects+17},{0,65,-64,64,large_corkscrew_right_rects+66},{TRACK_MASK_TRANSFER_NEXT,33,-96,64,large_corkscrew_right_rects+131},{TRACK_MASK_DIFFERENCE,1,-128,88,large_corkscrew_right_rects+164},
{0,1,0,0,large_corkscrew_right_rects+165},{TRACK_MASK_TRANSFER_NEXT,31,-32,-8,large_corkscrew_right_rects+166},{TRACK_MASK_DIFFERENCE,32,-64,0,large_corkscrew_right_rects+197},{0,1,-32,0,large_corkscrew_right_rects+229},{0,1,0,24,large_corkscrew_right_rects+230},
{TRACK_MASK_TRANSFER_NEXT,1,0,0,large_corkscrew_right_rects+231},{TRACK_MASK_DIFFERENCE,1,32,-8,large_corkscrew_right_rects+232},{0,33,64,0,large_corkscrew_right_rects+233},{0,65,96,32,large_corkscrew_right_rects+266},{0,33,128,88,large_corkscrew_right_rects+331},
{0,17,0,0,large_corkscrew_right_rects+364},{TRACK_MASK_TRANSFER_NEXT,48,32,24,large_corkscrew_right_rects+381},{TRACK_MASK_DIFFERENCE,65,64,64,large_corkscrew_right_rects+429},{0,48,32,96,large_corkscrew_right_rects+494},{0,17,0,152,large_corkscrew_right_rects+542},
};

const track_section_t large_corkscrew_right={TRACK_NO_SUPPORTS|TRACK_OFFSET_SPRITE_MASK|TRACK_SPECIAL_CORKSCREW_LEFT,large_corkscrew_right_curve,LARGE_CORKSCREW_LENGTH,{{VIEW_NEEDS_TRACK_MASK,5,large_corkscrew_right_masks},{VIEW_NEEDS_TRACK_MASK,5,large_corkscrew_right_masks+5},{VIEW_NEEDS_TRACK_MASK,5,large_corkscrew_right_masks+10},{VIEW_NEEDS_TRACK_MASK,5,large_corkscrew_right_masks+15}}};




//Slopes
const track_section_t flat={0,flat_curve,FLAT_LENGTH,{{0,1,NULL},{0,1,NULL},{0,0,NULL},{0,0,NULL}}};
const track_section_t flat_asymmetric={0,flat_curve,FLAT_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t brake={TRACK_SPECIAL_BRAKE,flat_curve,FLAT_LENGTH,{{0,1,NULL},{0,1,NULL},{0,0,NULL},{0,0,NULL}}};
const track_section_t block_brake={TRACK_SPECIAL_BLOCK_BRAKE,flat_curve,FLAT_LENGTH,{{0,1,NULL},{0,1,NULL},{0,0,NULL},{0,0,NULL}}};
const track_section_t booster={TRACK_SPECIAL_BOOSTER,flat_curve,FLAT_LENGTH,{{0,1,NULL},{0,1,NULL},{0,0,NULL},{0,0,NULL}}};
const track_section_t launched_lift={TRACK_SPECIAL_LAUNCHED_LIFT,gentle_curve,GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t flat_to_gentle_up={0,flat_to_gentle_up_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
//const track_section_t flat_to_gentle_down={0,flat_to_gentle_down_curve,TILE_SIZE,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t gentle_up_to_flat={0,gentle_up_to_flat_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t gentle={0,gentle_curve,GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
rect_t all={INT32_MIN,INT32_MIN,INT32_MAX,INT32_MAX};
mask_t gentle_to_steep_up_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all},{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t gentle_to_steep_up={0,gentle_to_steep_up_curve,GENTLE_TO_STEEP_LENGTH,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_to_steep_up_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_to_steep_up_masks+2},{0,1,NULL}}};
rect_t steep_to_gentle_up_rects[]={{9,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,9,INT32_MAX},{INT32_MIN,INT32_MIN,-12,INT32_MAX},{-12,INT32_MIN,INT32_MAX,INT32_MAX}};
mask_t steep_to_gentle_up_masks[]={{TRACK_MASK_UNION,1,0,0,steep_to_gentle_up_rects},{TRACK_MASK_DIFFERENCE,1,0,0,steep_to_gentle_up_rects+1},{TRACK_MASK_UNION,1,0,0,steep_to_gentle_up_rects+2},{TRACK_MASK_DIFFERENCE,1,0,0,steep_to_gentle_up_rects+3}};
const track_section_t steep_to_gentle_up={0,steep_to_gentle_up_curve,GENTLE_TO_STEEP_LENGTH,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,steep_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,2,steep_to_gentle_up_masks+2},{0,1,NULL}}};
const track_section_t steep={0,steep_curve,STEEP_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t steep_to_vertical_up={TRACK_VERTICAL|TRACK_NO_SUPPORTS|TRACK_SPECIAL_STEEP_TO_VERTICAL,steep_to_vertical_up_curve,STEEP_TO_VERTICAL_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t vertical_to_steep_up={TRACK_VERTICAL|TRACK_NO_SUPPORTS|TRACK_SPECIAL_VERTICAL_TO_STEEP,vertical_to_steep_up_curve,STEEP_TO_VERTICAL_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t vertical={TRACK_VERTICAL|TRACK_NO_SUPPORTS|TRACK_SPECIAL_VERTICAL,vertical_curve,VERTICAL_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};

//Turns
rect_t small_turn_left_rects[165]={
	{INT32_MIN,-1,INT32_MAX,INT32_MAX},{INT32_MIN,-2,-2,INT32_MAX},{INT32_MIN,-3,-4,INT32_MAX},{INT32_MIN,-4,-6,INT32_MAX},{INT32_MIN,-5,-8,INT32_MAX},{INT32_MIN,-6,-10,INT32_MAX},{INT32_MIN,-7,-12,INT32_MAX},{INT32_MIN,-8,-14,INT32_MAX},{INT32_MIN,-9,-16,INT32_MAX},{INT32_MIN,-10,-18,INT32_MAX},{INT32_MIN,-11,-20,INT32_MAX},{INT32_MIN,-12,-22,INT32_MAX},{INT32_MIN,-13,-24,INT32_MAX},{0,-12,INT32_MAX,1},{2,-13,INT32_MAX,1},{4,-14,INT32_MAX,1},{6,-15,INT32_MAX,1},{8,-16,INT32_MAX,1},{10,-17,INT32_MAX,1},{12,-18,INT32_MAX,1},{14,-19,INT32_MAX,1},{16,-20,INT32_MAX,1},{18,-21,INT32_MAX,1},{20,-22,INT32_MAX,1},{22,-23,INT32_MAX,1},{24,-24,INT32_MAX,1},{INT32_MIN,INT32_MIN,-24,-13},{-24,INT32_MIN,-22,-12},{-22,INT32_MIN,-20,-11},{-20,INT32_MIN,-18,-10},{-18,INT32_MIN,-16,-9},{-16,INT32_MIN,-14,-8},{-14,INT32_MIN,-12,-7},{-12,INT32_MIN,-10,-6},{-10,INT32_MIN,-8,-5},{-8,INT32_MIN,-6,-4},{-6,INT32_MIN,-4,-3},{-4,INT32_MIN,-2,-2},{-2,INT32_MIN,0,-1},{0,INT32_MIN,2,-12},{2,INT32_MIN,4,-13},{4,INT32_MIN,6,-14},{6,INT32_MIN,8,-15},{8,INT32_MIN,10,-16},{10,INT32_MIN,12,-17},{12,INT32_MIN,14,-18},{14,INT32_MIN,16,-19},{16,INT32_MIN,18,-20},{18,INT32_MIN,20,-21},{20,INT32_MIN,22,-22},{22,INT32_MIN,24,-23},{24,INT32_MIN,INT32_MAX,-24},{INT32_MIN,INT32_MIN,0,INT32_MAX},{0,INT32_MIN,2,31},{2,INT32_MIN,4,30},{4,INT32_MIN,6,29},{6,INT32_MIN,8,28},{8,INT32_MIN,10,27},{10,INT32_MIN,12,26},{12,INT32_MIN,14,25},{14,INT32_MIN,16,24},{16,INT32_MIN,18,23},{18,INT32_MIN,20,22},{20,INT32_MIN,22,21},{22,INT32_MIN,24,20},{24,INT32_MIN,26,19},{26,INT32_MIN,28,18},{28,INT32_MIN,30,17},{30,INT32_MIN,32,16},{30,16,34,INT32_MAX},{28,17,36,INT32_MAX},{26,18,38,INT32_MAX},{24,19,40,INT32_MAX},{22,20,42,INT32_MAX},{20,21,44,INT32_MAX},{18,22,46,INT32_MAX},{16,23,48,INT32_MAX},{14,24,50,INT32_MAX},{12,25,52,INT32_MAX},{10,26,54,INT32_MAX},{8,27,56,INT32_MAX},{6,28,58,INT32_MAX},{4,29,60,INT32_MAX},{2,30,62,INT32_MAX},{0,31,64,INT32_MAX},{32,INT32_MIN,34,16},{34,INT32_MIN,36,17},{36,INT32_MIN,38,18},{38,INT32_MIN,40,19},{40,INT32_MIN,42,20},{42,INT32_MIN,44,21},{44,INT32_MIN,46,22},{46,INT32_MIN,48,23},{48,INT32_MIN,50,24},{50,INT32_MIN,52,25},{52,INT32_MIN,54,26},{54,INT32_MIN,56,27},{56,INT32_MIN,58,28},{58,INT32_MIN,60,29},{60,INT32_MIN,62,30},{62,INT32_MIN,64,31},{64,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,7},{-24,INT32_MIN,-22,8},{-22,INT32_MIN,-20,9},{-20,INT32_MIN,-18,10},{-18,INT32_MIN,-16,11},{-16,INT32_MIN,-14,12},{-14,INT32_MIN,-12,13},{-12,INT32_MIN,-10,14},{-10,INT32_MIN,-8,15},{-8,INT32_MIN,-6,16},{-6,INT32_MIN,-4,17},{-4,INT32_MIN,-2,18},{-2,INT32_MIN,0,19},{0,INT32_MIN,2,31},{2,INT32_MIN,4,30},{4,INT32_MIN,6,29},{6,INT32_MIN,8,28},{8,INT32_MIN,10,27},{10,INT32_MIN,12,26},{12,INT32_MIN,14,25},{14,INT32_MIN,16,24},{16,INT32_MIN,18,23},{18,INT32_MIN,20,22},{20,INT32_MIN,22,21},{22,INT32_MIN,24,20},{24,INT32_MIN,INT32_MAX,19},{INT32_MIN,19,0,31},{INT32_MIN,18,-2,31},{INT32_MIN,17,-4,31},{INT32_MIN,16,-6,31},{INT32_MIN,15,-8,31},{INT32_MIN,14,-10,31},{INT32_MIN,13,-12,31},{INT32_MIN,12,-14,31},{INT32_MIN,11,-16,31},{INT32_MIN,10,-18,31},{INT32_MIN,9,-20,31},{INT32_MIN,8,-22,31},{INT32_MIN,7,-24,31},{INT32_MIN,31,INT32_MAX,INT32_MAX},{2,30,INT32_MAX,INT32_MAX},{4,29,INT32_MAX,INT32_MAX},{6,28,INT32_MAX,INT32_MAX},{8,27,INT32_MAX,INT32_MAX},{10,26,INT32_MAX,INT32_MAX},{12,25,INT32_MAX,INT32_MAX},{14,24,INT32_MAX,INT32_MAX},{16,23,INT32_MAX,INT32_MAX},{18,22,INT32_MAX,INT32_MAX},{20,21,INT32_MAX,INT32_MAX},{22,20,INT32_MAX,INT32_MAX},{24,19,INT32_MAX,INT32_MAX},{-26,INT32_MIN,INT32_MAX,INT32_MAX},{-28,13,-26,INT32_MAX},{-30,14,-28,INT32_MAX},{-32,15,-30,INT32_MAX},{-38,INT32_MIN,-26,13},{-39,INT32_MIN,-25,14},{-40,INT32_MIN,-24,15},{INT32_MIN,INT32_MIN,26-64,INT32_MAX},{26-64,13,28-64,INT32_MAX},{28-64,14,30-64,INT32_MAX},{30-64,15,32-64,INT32_MAX}
};
mask_t small_turn_masks[12]={
	{0,13,0,0,small_turn_left_rects},{0,13,-32,16,small_turn_left_rects+13},{0,26,0,32,small_turn_left_rects+26},
	{0,17,0,0,small_turn_left_rects+52},{0,16,-32,-16,small_turn_left_rects+69},{0,17,-64,0,small_turn_left_rects+85},
	{0,26,0,0,small_turn_left_rects+102},{0,13,32,-16,small_turn_left_rects+128},{0,13,0,-32,small_turn_left_rects+141},
	{0,4,0,0,small_turn_left_rects+154},{0,3,32,16,small_turn_left_rects+158},{0,4,64,0,small_turn_left_rects+161}
};
const track_section_t small_turn_left={0,small_turn_left_curve,SMALL_TURN_LENGTH,{{0,3,small_turn_masks},{0,3,small_turn_masks+3},{0,3,small_turn_masks+6},{0,3,small_turn_masks+9}}};
rect_t medium_turn_left_rects[]={
//First angle
{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,INT32_MAX},
{64,-33,INT32_MAX,-32},{62,-32,INT32_MAX,-31},{60,-31,INT32_MAX,-30},{58,-30,INT32_MAX,-29},{56,-29,INT32_MAX,-28},{54,-28,INT32_MAX,-27},{52,-27,INT32_MAX,-26},{50,-26,INT32_MAX,-25},{48,-25,INT32_MAX,-24},{46,-24,INT32_MAX,-23},{44,-23,INT32_MAX,-22},{42,-22,INT32_MAX,-21},{40,-21,INT32_MAX,-20},{38,-20,INT32_MAX,-19},{36,-19,INT32_MAX,-18},{34,-18,INT32_MAX,-17},{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{2,0,INT32_MAX,1},{4,1,INT32_MAX,2},{6,2,INT32_MAX,3},{8,3,INT32_MAX,4},{10,4,INT32_MAX,5},{12,5,INT32_MAX,6},{14,6,INT32_MAX,7},{16,7,INT32_MAX,8},{18,8,INT32_MAX,9},{20,9,INT32_MAX,10},{22,10,INT32_MAX,11},{24,11,INT32_MAX,12},{26,12,INT32_MAX,13},{28,13,INT32_MAX,14},{30,14,INT32_MAX,15},{32,15,INT32_MAX,INT32_MAX},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{32,INT32_MIN,INT32_MAX,-48},{30,-48,INT32_MAX,-47},{28,-47,INT32_MAX,-46},{26,-46,INT32_MAX,-45},{24,-45,INT32_MAX,-44},{22,-44,INT32_MAX,-43},{20,-43,INT32_MAX,-42},{18,-42,INT32_MAX,-41},{16,-41,INT32_MAX,-40},{14,-40,INT32_MAX,-39},{12,-39,INT32_MAX,-38},{10,-38,INT32_MAX,-37},{8,-37,INT32_MAX,-36},{6,-36,INT32_MAX,-35},{4,-35,INT32_MAX,-34},{2,-34,INT32_MAX,-33},{0,-33,64,-32},{2,-32,62,-31},{4,-31,60,-30},{6,-30,58,-29},{8,-29,56,-28},{10,-28,54,-27},{12,-27,52,-26},{14,-26,50,-25},{16,-25,48,-24},{18,-24,46,-23},{20,-23,44,-22},{22,-22,42,-21},{24,-21,40,-20},{26,-20,38,-19},{28,-19,36,-18},{30,-18,34,-17},
{INT32_MIN,INT32_MIN,32,-48},{INT32_MIN,-48,30,-47},{INT32_MIN,-47,28,-46},{INT32_MIN,-46,26,-45},{INT32_MIN,-45,24,-44},{INT32_MIN,-44,22,-43},{INT32_MIN,-43,20,-42},{INT32_MIN,-42,18,-41},{INT32_MIN,-41,16,-40},{INT32_MIN,-40,14,-39},{INT32_MIN,-39,12,-38},{INT32_MIN,-38,10,-37},{INT32_MIN,-37,8,-36},{INT32_MIN,-36,6,-35},{INT32_MIN,-35,4,-34},{INT32_MIN,-34,2,-33},{INT32_MIN,-33,0,-32},{INT32_MIN,-32,-2,-31},{INT32_MIN,-31,-4,-30},{INT32_MIN,-30,-6,-29},{INT32_MIN,-29,-8,-28},{INT32_MIN,-28,-10,-27},{INT32_MIN,-27,-12,-26},{INT32_MIN,-26,-14,-25},{INT32_MIN,-25,-16,-24},{INT32_MIN,-24,-18,-23},{INT32_MIN,-23,-20,-22},{INT32_MIN,-22,-22,-21},{INT32_MIN,-21,-24,-20},{INT32_MIN,-20,-26,-19},{INT32_MIN,-19,-28,-18},{INT32_MIN,-18,-30,-17},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,INT32_MAX},
{30,16,34,17},{28,17,36,18},{26,18,38,19},{24,19,40,20},{22,20,42,21},{20,21,44,22},{18,22,46,23},{16,23,48,24},{14,24,50,25},{12,25,52,26},{10,26,54,27},{8,27,56,28},{6,28,58,29},{4,29,60,30},{2,30,62,31},{0,31,64,INT32_MAX},
{32,INT32_MIN,96,16},{34,16,94,17},{36,17,92,18},{38,18,90,19},{40,19,88,20},{42,20,86,21},{44,21,84,22},{46,22,82,23},{48,23,80,24},{50,24,78,25},{52,25,76,26},{54,26,74,27},{56,27,72,28},{58,28,70,29},{60,29,68,30},{62,30,66,31},
{94,16,98,17},{92,17,100,18},{90,18,102,19},{88,19,104,20},{86,20,106,21},{84,21,108,22},{82,22,110,23},{80,23,112,24},{78,24,114,25},{76,25,116,26},{74,26,118,27},{72,27,120,28},{70,28,122,29},{68,29,124,30},{66,30,126,31},{64,31,128,INT32_MAX},
{96,INT32_MIN,INT32_MAX,16},{98,16,INT32_MAX,17},{100,17,INT32_MAX,18},{102,18,INT32_MAX,19},{104,19,INT32_MAX,20},{106,20,INT32_MAX,21},{108,21,INT32_MAX,22},{110,22,INT32_MAX,23},{112,23,INT32_MAX,24},{114,24,INT32_MAX,25},{116,25,INT32_MAX,26},{118,26,INT32_MAX,27},{120,27,INT32_MAX,28},{122,28,INT32_MAX,29},{124,29,INT32_MAX,30},{126,30,INT32_MAX,31},{128,31,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{2,32,INT32_MAX,33},{4,33,INT32_MAX,34},{6,34,INT32_MAX,35},{8,35,INT32_MAX,36},{10,36,INT32_MAX,37},{12,37,INT32_MAX,38},{14,38,INT32_MAX,39},{16,39,INT32_MAX,40},{18,40,INT32_MAX,41},{20,41,INT32_MAX,42},{22,42,INT32_MAX,43},{24,43,INT32_MAX,44},{26,44,INT32_MAX,45},{28,45,INT32_MAX,46},{30,46,INT32_MAX,47},{32,47,INT32_MAX,48},
{INT32_MIN,INT32_MIN,-32,16},{INT32_MIN,16,-30,17},{INT32_MIN,17,-28,18},{INT32_MIN,18,-26,19},{INT32_MIN,19,-24,20},{INT32_MIN,20,-22,21},{INT32_MIN,21,-20,22},{INT32_MIN,22,-18,23},{INT32_MIN,23,-16,24},{INT32_MIN,24,-14,25},{INT32_MIN,25,-12,26},{INT32_MIN,26,-10,27},{INT32_MIN,27,-8,28},{INT32_MIN,28,-6,29},{INT32_MIN,29,-4,30},{INT32_MIN,30,-2,31},{-64,31,0,32},{-62,32,-2,33},{-60,33,-4,34},{-58,34,-6,35},{-56,35,-8,36},{-54,36,-10,37},{-52,37,-12,38},{-50,38,-14,39},{-48,39,-16,40},{-46,40,-18,41},{-44,41,-20,42},{-42,42,-22,43},{-40,43,-24,44},{-38,44,-26,45},{-36,45,-28,46},{-34,46,-30,47},
{-2,32,2,33},{-4,33,4,34},{-6,34,6,35},{-8,35,8,36},{-10,36,10,37},{-12,37,12,38},{-14,38,14,39},{-16,39,16,40},{-18,40,18,41},{-20,41,20,42},{-22,42,22,43},{-24,43,24,44},{-26,44,26,45},{-28,45,28,46},{-30,46,30,47},{-32,47,32,48},{-30,48,30,49},{-28,49,28,50},{-26,50,26,51},{-24,51,24,52},{-22,52,22,53},{-20,53,20,54},{-18,54,18,55},{-16,55,16,56},{-14,56,14,57},{-12,57,12,58},{-10,58,10,59},{-8,59,8,60},{-6,60,6,61},{-4,61,4,62},{-2,62,2,63},
{INT32_MIN,31,-64,32},{INT32_MIN,32,-62,33},{INT32_MIN,33,-60,34},{INT32_MIN,34,-58,35},{INT32_MIN,35,-56,36},{INT32_MIN,36,-54,37},{INT32_MIN,37,-52,38},{INT32_MIN,38,-50,39},{INT32_MIN,39,-48,40},{INT32_MIN,40,-46,41},{INT32_MIN,41,-44,42},{INT32_MIN,42,-42,43},{INT32_MIN,43,-40,44},{INT32_MIN,44,-38,45},{INT32_MIN,45,-36,46},{INT32_MIN,46,-34,47},{INT32_MIN,47,-32,48},{INT32_MIN,48,-30,49},{INT32_MIN,49,-28,50},{INT32_MIN,50,-26,51},{INT32_MIN,51,-24,52},{INT32_MIN,52,-22,53},{INT32_MIN,53,-20,54},{INT32_MIN,54,-18,55},{INT32_MIN,55,-16,56},{INT32_MIN,56,-14,57},{INT32_MIN,57,-12,58},{INT32_MIN,58,-10,59},{INT32_MIN,59,-8,60},{INT32_MIN,60,-6,61},{INT32_MIN,61,-4,62},{INT32_MIN,62,-2,63},{INT32_MIN,63,0,64},{INT32_MIN,64,-2,65},{INT32_MIN,65,-4,66},{INT32_MIN,66,-6,67},{INT32_MIN,67,-8,68},{INT32_MIN,68,-10,69},{INT32_MIN,69,-12,70},{INT32_MIN,70,-14,71},{INT32_MIN,71,-16,72},{INT32_MIN,72,-18,73},{INT32_MIN,73,-20,74},{INT32_MIN,74,-22,75},{INT32_MIN,75,-24,76},{INT32_MIN,76,-26,77},{INT32_MIN,77,-28,78},{INT32_MIN,78,-30,79},{INT32_MIN,79,-32,INT32_MAX},
{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{-2,64,INT32_MAX,65},{-4,65,INT32_MAX,66},{-6,66,INT32_MAX,67},{-8,67,INT32_MAX,68},{-10,68,INT32_MAX,69},{-12,69,INT32_MAX,70},{-14,70,INT32_MAX,71},{-16,71,INT32_MAX,72},{-18,72,INT32_MAX,73},{-20,73,INT32_MAX,74},{-22,74,INT32_MAX,75},{-24,75,INT32_MAX,76},{-26,76,INT32_MAX,77},{-28,77,INT32_MAX,78},{-30,78,INT32_MAX,79},{-32,79,INT32_MAX,INT32_MAX},
//Fourth angle
{-32,15,INT32_MAX,INT32_MAX},{-30,14,INT32_MAX,15},{-28,13,INT32_MAX,14},{-26,12,INT32_MAX,13},{-24,11,INT32_MAX,12},{-22,10,INT32_MAX,11},{-20,9,INT32_MAX,10},{-18,8,INT32_MAX,9},{-16,7,INT32_MAX,8},{-14,6,INT32_MAX,7},{-12,5,INT32_MAX,6},{-10,4,INT32_MAX,5},{-8,3,INT32_MAX,4},{-6,2,INT32_MAX,3},{-4,1,INT32_MAX,2},{-2,0,INT32_MAX,1},{0,-1,INT32_MAX,0},{-2,-2,INT32_MAX,-1},{-4,-3,INT32_MAX,-2},{-6,-4,INT32_MAX,-3},{-8,-5,INT32_MAX,-4},{-10,-6,INT32_MAX,-5},{-12,-7,INT32_MAX,-6},{-14,-8,INT32_MAX,-7},{-16,-9,INT32_MAX,-8},{-18,-10,INT32_MAX,-9},{-20,-11,INT32_MAX,-10},{-22,-12,INT32_MAX,-11},{-24,-13,INT32_MAX,-12},{-26,-14,INT32_MAX,-13},{-28,-15,INT32_MAX,-14},{-30,-16,INT32_MAX,-15},{-32,INT32_MIN,INT32_MAX,-16},{-34,14,-30,15},{-36,13,-28,14},{-38,12,-26,13},{-40,11,-24,12},{-42,10,-22,11},{-44,9,-20,10},{-46,8,-18,9},{-48,7,-16,8},{-50,6,-14,7},{-52,5,-12,6},{-54,4,-10,5},{-56,3,-8,4},{-58,2,-6,3},{-60,1,-4,2},{-62,0,-2,1},{-64,-1,0,0},{-62,-2,-2,-1},{-60,-3,-4,-2},{-58,-4,-6,-3},{-56,-5,-8,-4},{-54,-6,-10,-5},{-52,-7,-12,-6},{-50,-8,-14,-7},{-48,-9,-16,-8},{-46,-10,-18,-9},{-44,-11,-20,-10},{-42,-12,-22,-11},{-40,-13,-24,-12},{-38,-14,-26,-13},{-36,-15,-28,-14},{-34,-16,-30,-15},{-96,15,-32,INT32_MAX},{-94,14,-34,15},{-92,13,-36,14},{-90,12,-38,13},{-88,11,-40,12},{-86,10,-42,11},{-84,9,-44,10},{-82,8,-46,9},{-80,7,-48,8},{-78,6,-50,7},{-76,5,-52,6},{-74,4,-54,5},{-72,3,-56,4},{-70,2,-58,3},{-68,1,-60,2},{-66,0,-62,1},{-66,-2,-62,-1},{-68,-3,-60,-2},{-70,-4,-58,-3},{-72,-5,-56,-4},{-74,-6,-54,-5},{-76,-7,-52,-6},{-78,-8,-50,-7},{-80,-9,-48,-8},{-82,-10,-46,-9},{-84,-11,-44,-10},{-86,-12,-42,-11},{-88,-13,-40,-12},{-90,-14,-38,-13},{-92,-15,-36,-14},{-94,-16,-34,-15},{-96,INT32_MIN,-32,-16},{-98,14,-94,15},{-100,13,-92,14},{-102,12,-90,13},{-104,11,-88,12},{-106,10,-86,11},{-108,9,-84,10},{-110,8,-82,9},{-112,7,-80,8},{-114,6,-78,7},{-116,5,-76,6},{-118,4,-74,5},{-120,3,-72,4},{-122,2,-70,3},{-124,1,-68,2},{-126,0,-66,1},{-128,-1,-64,0},{-126,-2,-66,-1},{-124,-3,-68,-2},{-122,-4,-70,-3},{-120,-5,-72,-4},{-118,-6,-74,-5},{-116,-7,-76,-6},{-114,-8,-78,-7},{-112,-9,-80,-8},{-110,-10,-82,-9},{-108,-11,-84,-10},{-106,-12,-86,-11},{-104,-13,-88,-12},{-102,-14,-90,-13},{-100,-15,-92,-14},{-98,-16,-94,-15},{INT32_MIN,15,-96,INT32_MAX},{INT32_MIN,14,-98,15},{INT32_MIN,13,-100,14},{INT32_MIN,12,-102,13},{INT32_MIN,11,-104,12},{INT32_MIN,10,-106,11},{INT32_MIN,9,-108,10},{INT32_MIN,8,-110,9},{INT32_MIN,7,-112,8},{INT32_MIN,6,-114,7},{INT32_MIN,5,-116,6},{INT32_MIN,4,-118,5},{INT32_MIN,3,-120,4},{INT32_MIN,2,-122,3},{INT32_MIN,1,-124,2},{INT32_MIN,0,-126,1},{INT32_MIN,-1,-128,0},{INT32_MIN,-2,-126,-1},{INT32_MIN,-3,-124,-2},{INT32_MIN,-4,-122,-3},{INT32_MIN,-5,-120,-4},{INT32_MIN,-6,-118,-5},{INT32_MIN,-7,-116,-6},{INT32_MIN,-8,-114,-7},{INT32_MIN,-9,-112,-8},{INT32_MIN,-10,-110,-9},{INT32_MIN,-11,-108,-10},{INT32_MIN,-12,-106,-11},{INT32_MIN,-13,-104,-12},{INT32_MIN,-14,-102,-13},{INT32_MIN,-15,-100,-14},{INT32_MIN,-16,-98,-15},{INT32_MIN,INT32_MIN,-96,-16}
};
mask_t medium_turn_left_masks[]={
{0,33,0,0,medium_turn_left_rects},{0,49,-32,16,medium_turn_left_rects+33},{0,31,0,32,medium_turn_left_rects+82},{0,32,-32,48,medium_turn_left_rects+113},{0,32,0,64,medium_turn_left_rects+145},
{0,17,0,0,medium_turn_left_rects+177},{0,16,-32,-16,medium_turn_left_rects+194},{0,16,-64,0,medium_turn_left_rects+210},{0,16,-96,-16,medium_turn_left_rects+226},{0,17,-128,0,medium_turn_left_rects+242},
{0,33,0,0,medium_turn_left_rects+259},{0,32,32,-16,medium_turn_left_rects+292},{0,31,0,-32,medium_turn_left_rects+324},{0,49,32,-48,medium_turn_left_rects+355},{0,32,0,-64,medium_turn_left_rects+404},
{0,33,0,0,medium_turn_left_rects+436},{0,31,32,16,medium_turn_left_rects+469},{0,32,64,0,medium_turn_left_rects+500},{0,31,96,16,medium_turn_left_rects+532},{0,33,128,0,medium_turn_left_rects+563}
};
const track_section_t medium_turn_left={0,medium_turn_left_curve,MEDIUM_TURN_LENGTH,{{0,5,medium_turn_left_masks},{0,5,medium_turn_left_masks+5},{0,5,medium_turn_left_masks+10},{0,5,medium_turn_left_masks+15}}};
rect_t large_turn_left_to_diag_rects[]={
//First angle
{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{62,-32,INT32_MAX,-31},{60,-31,INT32_MAX,-30},{58,-30,INT32_MAX,-29},{56,-29,INT32_MAX,-28},{54,-28,INT32_MAX,-27},{52,-27,INT32_MAX,-26},{50,-26,INT32_MAX,-25},{48,-25,INT32_MAX,-24},{46,-24,INT32_MAX,-23},{44,-23,INT32_MAX,-22},{42,-22,INT32_MAX,-21},{40,-21,INT32_MAX,-20},{38,-20,INT32_MAX,-19},{36,-19,INT32_MAX,-18},{34,-18,INT32_MAX,-17},{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{2,0,INT32_MAX,1},{4,1,INT32_MAX,2},{6,2,INT32_MAX,3},{8,3,INT32_MAX,4},{10,4,INT32_MAX,5},{12,5,INT32_MAX,6},{14,6,INT32_MAX,7},{16,7,INT32_MAX,8},{18,8,INT32_MAX,9},{20,9,INT32_MAX,10},{22,10,INT32_MAX,11},{24,11,INT32_MAX,12},{26,12,INT32_MAX,13},{28,13,INT32_MAX,14},{30,14,INT32_MAX,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,62,-31},{INT32_MIN,-31,-4,-30},{4,-31,60,-30},{INT32_MIN,-30,-6,-29},{6,-30,58,-29},{INT32_MIN,-29,-8,-28},{8,-29,56,-28},{INT32_MIN,-28,-10,-27},{10,-28,54,-27},{INT32_MIN,-27,-12,-26},{12,-27,52,-26},{INT32_MIN,-26,-14,-25},{14,-26,50,-25},{INT32_MIN,-25,-16,-24},{16,-25,48,-24},{INT32_MIN,-24,-18,-23},{18,-24,46,-23},{INT32_MIN,-23,-20,-22},{20,-23,44,-22},{INT32_MIN,-22,-22,-21},{22,-22,42,-21},{INT32_MIN,-21,-24,-20},{24,-21,40,-20},{INT32_MIN,-20,-26,-19},{26,-20,38,-19},{INT32_MIN,-19,-28,-18},{28,-19,36,-18},{INT32_MIN,-18,-30,-17},{30,-18,34,-17},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,INT32_MAX},
{30,16,34,17},{28,17,36,18},{26,18,38,19},{24,19,40,20},{22,20,42,21},{20,21,44,22},{18,22,46,23},{16,23,48,24},{14,24,50,25},{12,25,52,26},{10,26,54,27},{8,27,56,28},{6,28,58,29},{4,29,60,30},{2,30,62,31},{0,31,64,INT32_MAX},
{32,INT32_MIN,96,16},{34,16,94,17},{36,17,92,18},{38,18,90,19},{40,19,88,20},{42,20,86,21},{44,21,84,22},{46,22,82,23},{48,23,80,24},{50,24,78,25},{52,25,76,26},{54,26,74,27},{56,27,72,28},{58,28,70,29},{60,29,68,30},{62,30,66,31},
{96,INT32_MIN,INT32_MAX,16},{94,16,INT32_MAX,17},{92,17,INT32_MAX,18},{90,18,INT32_MAX,19},{88,19,INT32_MAX,20},{86,20,INT32_MAX,21},{84,21,INT32_MAX,22},{82,22,INT32_MAX,23},{80,23,INT32_MAX,24},{78,24,INT32_MAX,25},{76,25,INT32_MAX,26},{74,26,INT32_MAX,27},{72,27,INT32_MAX,28},{70,28,INT32_MAX,29},{68,29,INT32_MAX,30},{66,30,INT32_MAX,31},{64,31,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},
{INT32_MIN,INT32_MIN,-32,16},{INT32_MIN,16,-30,17},{INT32_MIN,17,-28,18},{INT32_MIN,18,-26,19},{INT32_MIN,19,-24,20},{INT32_MIN,20,-22,21},{INT32_MIN,21,-20,22},{INT32_MIN,22,-18,23},{INT32_MIN,23,-16,24},{INT32_MIN,24,-14,25},{INT32_MIN,25,-12,26},{INT32_MIN,26,-10,27},{INT32_MIN,27,-8,28},{INT32_MIN,28,-6,29},{INT32_MIN,29,-4,30},{INT32_MIN,30,-2,31},{-64,31,0,32},{-62,32,-2,33},{-60,33,-4,34},{-58,34,-6,35},{-56,35,-8,36},{-54,36,-10,37},{-52,37,-12,38},{-50,38,-14,39},{-48,39,-16,40},{-46,40,-18,41},{-44,41,-20,42},{-42,42,-22,43},{-40,43,-24,44},{-38,44,-26,45},{-36,45,-28,46},{-34,46,-30,47},
{0,31,INT32_MAX,32},{-2,32,INT32_MAX,33},{-4,33,INT32_MAX,34},{-6,34,INT32_MAX,35},{-8,35,INT32_MAX,36},{-10,36,INT32_MAX,37},{-12,37,INT32_MAX,38},{-14,38,INT32_MAX,39},{-16,39,INT32_MAX,40},{-18,40,INT32_MAX,41},{-20,41,INT32_MAX,42},{-22,42,INT32_MAX,43},{-24,43,INT32_MAX,44},{-26,44,INT32_MAX,45},{-28,45,INT32_MAX,46},{-30,46,INT32_MAX,47},{-32,47,INT32_MAX,48},{-30,48,INT32_MAX,49},{-28,49,INT32_MAX,50},{-26,50,INT32_MAX,51},{-24,51,INT32_MAX,52},{-22,52,INT32_MAX,53},{-20,53,INT32_MAX,54},{-18,54,INT32_MAX,55},{-16,55,INT32_MAX,56},{-14,56,INT32_MAX,57},{-12,57,INT32_MAX,58},{-10,58,INT32_MAX,59},{-8,59,INT32_MAX,60},{-6,60,INT32_MAX,61},{-4,61,INT32_MAX,62},{-2,62,INT32_MAX,63},{0,63,INT32_MAX,INT32_MAX},
{INT32_MIN,31,-64,32},{INT32_MIN,32,-62,33},{INT32_MIN,33,-60,34},{INT32_MIN,34,-58,35},{INT32_MIN,35,-56,36},{INT32_MIN,36,-54,37},{INT32_MIN,37,-52,38},{INT32_MIN,38,-50,39},{INT32_MIN,39,-48,40},{INT32_MIN,40,-46,41},{INT32_MIN,41,-44,42},{INT32_MIN,42,-42,43},{INT32_MIN,43,-40,44},{INT32_MIN,44,-38,45},{INT32_MIN,45,-36,46},{INT32_MIN,46,-34,47},{INT32_MIN,47,-32,48},{INT32_MIN,48,-30,49},{INT32_MIN,49,-28,50},{INT32_MIN,50,-26,51},{INT32_MIN,51,-24,52},{INT32_MIN,52,-22,53},{INT32_MIN,53,-20,54},{INT32_MIN,54,-18,55},{INT32_MIN,55,-16,56},{INT32_MIN,56,-14,57},{INT32_MIN,57,-12,58},{INT32_MIN,58,-10,59},{INT32_MIN,59,-8,60},{INT32_MIN,60,-6,61},{INT32_MIN,61,-4,62},{INT32_MIN,62,-2,63},{INT32_MIN,63,0,INT32_MAX},
//Fourth angle
{0,INT32_MIN,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{-32,15,INT32_MAX,INT32_MAX},
{-64,INT32_MIN,0,0},{-62,0,-2,1},{-60,1,-4,2},{-58,2,-6,3},{-56,3,-8,4},{-54,4,-10,5},{-52,5,-12,6},{-50,6,-14,7},{-48,7,-16,8},{-46,8,-18,9},{-44,9,-20,10},{-42,10,-22,11},{-40,11,-24,12},{-38,12,-26,13},{-36,13,-28,14},{-34,14,-30,15},
{-66,0,-62,1},{-68,1,-60,2},{-70,2,-58,3},{-72,3,-56,4},{-74,4,-54,5},{-76,5,-52,6},{-78,6,-50,7},{-80,7,-48,8},{-82,8,-46,9},{-84,9,-44,10},{-86,10,-42,11},{-88,11,-40,12},{-90,12,-38,13},{-92,13,-36,14},{-94,14,-34,15},{-96,15,-32,INT32_MAX},
{INT32_MIN,INT32_MIN,-64,0},{INT32_MIN,0,-66,1},{INT32_MIN,1,-68,2},{INT32_MIN,2,-70,3},{INT32_MIN,3,-72,4},{INT32_MIN,4,-74,5},{INT32_MIN,5,-76,6},{INT32_MIN,6,-78,7},{INT32_MIN,7,-80,8},{INT32_MIN,8,-82,9},{INT32_MIN,9,-84,10},{INT32_MIN,10,-86,11},{INT32_MIN,11,-88,12},{INT32_MIN,12,-90,13},{INT32_MIN,13,-92,14},{INT32_MIN,14,-94,15},{INT32_MIN,15,-96,INT32_MAX}
};
mask_t large_turn_left_to_diag_masks[]={
{0,33,0,0,large_turn_left_to_diag_rects},{0,47,-32,16,large_turn_left_to_diag_rects+33},{0,31,0,32,large_turn_left_to_diag_rects+80},{0,31,-32,48,large_turn_left_to_diag_rects+111},
{0,17,0,0,large_turn_left_to_diag_rects+142},{0,16,-32,-16,large_turn_left_to_diag_rects+159},{0,16,-64,0,large_turn_left_to_diag_rects+175},{0,17,-96,-16,large_turn_left_to_diag_rects+191},
{0,16,0,0,large_turn_left_to_diag_rects+208},{0,32,32,-16,large_turn_left_to_diag_rects+224},{0,33,0,-32,large_turn_left_to_diag_rects+256},{0,33,32,-48,large_turn_left_to_diag_rects+289},
{0,17,0,0,large_turn_left_to_diag_rects+322},{0,16,32,16,large_turn_left_to_diag_rects+339},{0,16,64,0,large_turn_left_to_diag_rects+355},{0,17,96,16,large_turn_left_to_diag_rects+371},
};
const track_section_t large_turn_left_to_diag={0,large_turn_left_to_diag_curve,LARGE_TURN_LENGTH,{{0,4,large_turn_left_to_diag_masks},{0,4,large_turn_left_to_diag_masks+4},{0,4,large_turn_left_to_diag_masks+8},{0,4,large_turn_left_to_diag_masks+12}}};
rect_t large_turn_right_to_diag_rects[]={
//First angle
{INT32_MIN,INT32_MIN,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,INT32_MAX},
{0,INT32_MIN,64,0},{2,0,62,1},{4,1,60,2},{6,2,58,3},{8,3,56,4},{10,4,54,5},{12,5,52,6},{14,6,50,7},{16,7,48,8},{18,8,46,9},{20,9,44,10},{22,10,42,11},{24,11,40,12},{26,12,38,13},{28,13,36,14},{30,14,34,15},
{62,0,66,1},{60,1,68,2},{58,2,70,3},{56,3,72,4},{54,4,74,5},{52,5,76,6},{50,6,78,7},{48,7,80,8},{46,8,82,9},{44,9,84,10},{42,10,86,11},{40,11,88,12},{38,12,90,13},{36,13,92,14},{34,14,94,15},{32,15,96,INT32_MAX},
{64,INT32_MIN,INT32_MAX,0},{66,0,INT32_MAX,1},{68,1,INT32_MAX,2},{70,2,INT32_MAX,3},{72,3,INT32_MAX,4},{74,4,INT32_MAX,5},{76,5,INT32_MAX,6},{78,6,INT32_MAX,7},{80,7,INT32_MAX,8},{82,8,INT32_MAX,9},{84,9,INT32_MAX,10},{86,10,INT32_MAX,11},{88,11,INT32_MAX,12},{90,12,INT32_MAX,13},{92,13,INT32_MAX,14},{94,14,INT32_MAX,15},{96,15,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},
{32,INT32_MIN,INT32_MAX,16},{30,16,INT32_MAX,17},{28,17,INT32_MAX,18},{26,18,INT32_MAX,19},{24,19,INT32_MAX,20},{22,20,INT32_MAX,21},{20,21,INT32_MAX,22},{18,22,INT32_MAX,23},{16,23,INT32_MAX,24},{14,24,INT32_MAX,25},{12,25,INT32_MAX,26},{10,26,INT32_MAX,27},{8,27,INT32_MAX,28},{6,28,INT32_MAX,29},{4,29,INT32_MAX,30},{2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{2,32,62,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},
{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,48},{INT32_MIN,48,30,49},{INT32_MIN,49,28,50},{INT32_MIN,50,26,51},{INT32_MIN,51,24,52},{INT32_MIN,52,22,53},{INT32_MIN,53,20,54},{INT32_MIN,54,18,55},{INT32_MIN,55,16,56},{INT32_MIN,56,14,57},{INT32_MIN,57,12,58},{INT32_MIN,58,10,59},{INT32_MIN,59,8,60},{INT32_MIN,60,6,61},{INT32_MIN,61,4,62},{INT32_MIN,62,2,63},{INT32_MIN,63,0,64},{INT32_MIN,64,-2,INT32_MAX},
{62,32,INT32_MAX,33},{60,33,INT32_MAX,34},{58,34,INT32_MAX,35},{56,35,INT32_MAX,36},{54,36,INT32_MAX,37},{52,37,INT32_MAX,38},{50,38,INT32_MAX,39},{48,39,INT32_MAX,40},{46,40,INT32_MAX,41},{44,41,INT32_MAX,42},{42,42,INT32_MAX,43},{40,43,INT32_MAX,44},{38,44,INT32_MAX,45},{36,45,INT32_MAX,46},{34,46,INT32_MAX,47},{32,47,INT32_MAX,48},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{-2,64,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},{0,31,INT32_MAX,INT32_MAX},
{-34,16,-30,17},{-36,17,-28,18},{-38,18,-26,19},{-40,19,-24,20},{-42,20,-22,21},{-44,21,-20,22},{-46,22,-18,23},{-48,23,-16,24},{-50,24,-14,25},{-52,25,-12,26},{-54,26,-10,27},{-56,27,-8,28},{-58,28,-6,29},{-60,29,-4,30},{-62,30,-2,31},{-64,31,0,INT32_MAX},
{-96,INT32_MIN,-32,16},{-94,16,-34,17},{-92,17,-36,18},{-90,18,-38,19},{-88,19,-40,20},{-86,20,-42,21},{-84,21,-44,22},{-82,22,-46,23},{-80,23,-48,24},{-78,24,-50,25},{-76,25,-52,26},{-74,26,-54,27},{-72,27,-56,28},{-70,28,-58,29},{-68,29,-60,30},{-66,30,-62,31},
{INT32_MIN,INT32_MIN,-96,16},{INT32_MIN,16,-94,17},{INT32_MIN,17,-92,18},{INT32_MIN,18,-90,19},{INT32_MIN,19,-88,20},{INT32_MIN,20,-86,21},{INT32_MIN,21,-84,22},{INT32_MIN,22,-82,23},{INT32_MIN,23,-80,24},{INT32_MIN,24,-78,25},{INT32_MIN,25,-76,26},{INT32_MIN,26,-74,27},{INT32_MIN,27,-72,28},{INT32_MIN,28,-70,29},{INT32_MIN,29,-68,30},{INT32_MIN,30,-66,31},{INT32_MIN,31,-64,INT32_MAX},
//Fourth angle
{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{INT32_MIN,-26,-50,-25},{INT32_MIN,-25,-48,-24},{INT32_MIN,-24,-46,-23},{INT32_MIN,-23,-44,-22},{INT32_MIN,-22,-42,-21},{INT32_MIN,-21,-40,-20},{INT32_MIN,-20,-38,-19},{INT32_MIN,-19,-36,-18},{INT32_MIN,-18,-34,-17},{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,-2,1},{INT32_MIN,1,-4,2},{INT32_MIN,2,-6,3},{INT32_MIN,3,-8,4},{INT32_MIN,4,-10,5},{INT32_MIN,5,-12,6},{INT32_MIN,6,-14,7},{INT32_MIN,7,-16,8},{INT32_MIN,8,-18,9},{INT32_MIN,9,-20,10},{INT32_MIN,10,-22,11},{INT32_MIN,11,-24,12},{INT32_MIN,12,-26,13},{INT32_MIN,13,-28,14},{INT32_MIN,14,-30,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,INT32_MAX,-31},{INT32_MIN,-31,-4,-30},{4,-31,INT32_MAX,-30},{INT32_MIN,-30,-6,-29},{6,-30,INT32_MAX,-29},{INT32_MIN,-29,-8,-28},{8,-29,INT32_MAX,-28},{INT32_MIN,-28,-10,-27},{10,-28,INT32_MAX,-27},{INT32_MIN,-27,-12,-26},{12,-27,INT32_MAX,-26},{-50,-26,-14,-25},{14,-26,INT32_MAX,-25},{-48,-25,-16,-24},{16,-25,INT32_MAX,-24},{-46,-24,-18,-23},{18,-24,INT32_MAX,-23},{-44,-23,-20,-22},{20,-23,INT32_MAX,-22},{-42,-22,-22,-21},{22,-22,INT32_MAX,-21},{-40,-21,-24,-20},{24,-21,INT32_MAX,-20},{-38,-20,-26,-19},{26,-20,INT32_MAX,-19},{-36,-19,-28,-18},{28,-19,INT32_MAX,-18},{-34,-18,-30,-17},{30,-18,INT32_MAX,-17}
};
mask_t large_turn_right_to_diag_masks[]={
{0,17,0,0,large_turn_right_to_diag_rects},{0,16,-32,16,large_turn_right_to_diag_rects+17},{0,16,-64,0,large_turn_right_to_diag_rects+33},{0,17,-96,16,large_turn_right_to_diag_rects+49},
{0,16,0,0,large_turn_right_to_diag_rects+66},{0,32,-32,-16,large_turn_right_to_diag_rects+82},{0,34,0,-32,large_turn_right_to_diag_rects+114},{0,33,-32,-48,large_turn_right_to_diag_rects+148},
{0,17,0,0,large_turn_right_to_diag_rects+181},{0,16,32,-16,large_turn_right_to_diag_rects+198},{0,16,64,0,large_turn_right_to_diag_rects+214},{0,17,96,-16,large_turn_right_to_diag_rects+230},
{0,33,0,0,large_turn_right_to_diag_rects+247},{0,41,32,16,large_turn_right_to_diag_rects+280},{0,31,0,32,large_turn_right_to_diag_rects+321},{0,31,32,48,large_turn_right_to_diag_rects+352},
};
const track_section_t large_turn_right_to_diag={0,large_turn_right_to_diag_curve,LARGE_TURN_LENGTH,{{0,4,large_turn_right_to_diag_masks},{0,4,large_turn_right_to_diag_masks+4},{0,4,large_turn_right_to_diag_masks+8},{0,4,large_turn_right_to_diag_masks+12}}};

//Diagonals
rect_t diag_slope_rect={-32,INT32_MIN,32,INT32_MAX};
mask_t diag_slope_mask={0,1,0,0,&diag_slope_rect};
const track_section_t flat_diag={TRACK_DIAGONAL,flat_diag_curve,FLAT_DIAG_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t flat_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_SUPPORT_BASE,flat_to_gentle_up_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t gentle_to_flat_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,gentle_to_flat_up_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
const track_section_t gentle_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,gentle_diag_curve,GENTLE_DIAG_LENGTH,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
const track_section_t gentle_to_steep_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,gentle_to_steep_up_diag_curve,GENTLE_TO_STEEP_DIAG_LENGTH,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
const track_section_t steep_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,steep_to_gentle_up_diag_curve,GENTLE_TO_STEEP_DIAG_LENGTH,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
const track_section_t steep_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,steep_diag_curve,STEEP_DIAG_LENGTH,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
const track_section_t small_flat_to_steep_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,small_flat_to_steep_up_diag_curve,SMALL_FLAT_TO_STEEP_DIAG_LENGTH,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
const track_section_t small_steep_to_flat_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,small_steep_to_flat_up_diag_curve,SMALL_FLAT_TO_STEEP_DIAG_LENGTH,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};

//Banked turns
rect_t flat_to_left_bank_rects[]={{INT32_MIN,INT32_MIN,-1,INT32_MAX},{-1,INT32_MIN,INT32_MAX,INT32_MAX}};
mask_t flat_to_left_bank_masks[]={{TRACK_MASK_UNION,1,0,0,flat_to_left_bank_rects},{TRACK_MASK_DIFFERENCE,1,0,0,flat_to_left_bank_rects+1},{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t flat_to_left_bank={TRACK_EXIT_BANK_LEFT,flat_to_left_bank_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,flat_to_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,flat_to_left_bank_masks+2},{0,1,NULL},{0,1,NULL}}};
rect_t flat_to_right_bank_rects[]={{-1,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-1,INT32_MAX}};
mask_t flat_to_right_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all},{TRACK_MASK_UNION,1,0,0,flat_to_right_bank_rects},{TRACK_MASK_DIFFERENCE,1,0,0,flat_to_right_bank_rects+1}};
const track_section_t flat_to_right_bank={TRACK_EXIT_BANK_RIGHT,flat_to_right_bank_curve,FLAT_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,flat_to_right_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,flat_to_right_bank_masks+2}}};
rect_t left_bank_to_gentle_up_rects[]={{6,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,6,INT32_MAX}};
mask_t left_bank_to_gentle_up_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all},{TRACK_MASK_UNION,1,0,0,left_bank_to_gentle_up_rects},{TRACK_MASK_DIFFERENCE,1,0,0,left_bank_to_gentle_up_rects+1}};
const track_section_t left_bank_to_gentle_up={TRACK_ENTRY_BANK_LEFT,left_bank_to_gentle_up_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,left_bank_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,2,left_bank_to_gentle_up_masks+2},{0,1,NULL},{0,1,NULL}}};
rect_t right_bank_to_gentle_up_rects[]={{INT32_MIN,INT32_MIN,-5,INT32_MAX},{-5,INT32_MIN,INT32_MAX,INT32_MAX}};
mask_t right_bank_to_gentle_up_masks[]={{TRACK_MASK_UNION,1,0,0,right_bank_to_gentle_up_rects},{TRACK_MASK_DIFFERENCE,1,0,0,right_bank_to_gentle_up_rects+1},{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t right_bank_to_gentle_up={TRACK_ENTRY_BANK_RIGHT,right_bank_to_gentle_up_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,right_bank_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,2,right_bank_to_gentle_up_masks+2}}};
rect_t gentle_up_to_left_bank_rects[]={{INT32_MIN,INT32_MIN,-2,INT32_MAX},{-2,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-16,INT32_MAX},{-16,INT32_MIN,INT32_MAX,INT32_MAX}};
mask_t gentle_up_to_left_bank_masks[]={{TRACK_MASK_UNION,1,0,0,gentle_up_to_left_bank_rects},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_to_left_bank_rects+1},{TRACK_MASK_UNION,1,0,0,gentle_up_to_left_bank_rects+2},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_to_left_bank_rects+3}};
const track_section_t gentle_up_to_left_bank={TRACK_EXIT_BANK_LEFT,gentle_up_to_left_bank_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_left_bank_masks+2},{0,1,NULL},{0,1,NULL}}};
rect_t gentle_up_to_right_bank_rects[]={{13,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,13,INT32_MAX},{-2,INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,-2,INT32_MAX}};
mask_t gentle_up_to_right_bank_masks[]={{TRACK_MASK_UNION,1,0,0,gentle_up_to_right_bank_rects},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_to_right_bank_rects+1},{TRACK_MASK_UNION,1,0,0,gentle_up_to_right_bank_rects+2},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_to_right_bank_rects+3}};
const track_section_t gentle_up_to_right_bank={TRACK_EXIT_BANK_RIGHT,gentle_up_to_right_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_right_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_right_bank_masks+2}}};
const track_section_t left_bank={TRACK_BANK_LEFT,left_bank_curve,FLAT_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
//Note this is not the splitting used for the existing sprites, but it works better
mask_t diagonal_transition_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&diag_slope_rect},{TRACK_MASK_DIFFERENCE,1,0,0,&diag_slope_rect}};
const track_section_t flat_to_left_bank_diag={TRACK_DIAGONAL|TRACK_EXIT_BANK_LEFT,flat_to_left_bank_diag_curve,FLAT_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t flat_to_right_bank_diag={TRACK_DIAGONAL|TRACK_EXIT_BANK_RIGHT,flat_to_right_bank_diag_curve,FLAT_DIAG_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,NULL}}};
const track_section_t left_bank_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_ENTRY_BANK_LEFT|TRACK_SUPPORT_BASE,left_bank_to_gentle_up_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t right_bank_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_ENTRY_BANK_RIGHT|TRACK_SUPPORT_BASE,right_bank_to_gentle_up_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,NULL}}};
const track_section_t gentle_up_to_left_bank_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_EXIT_BANK_LEFT|TRACK_SUPPORT_BASE,gentle_up_to_left_bank_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{0,1,&diag_slope_mask}}};
const track_section_t gentle_up_to_right_bank_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_EXIT_BANK_RIGHT|TRACK_SUPPORT_BASE,gentle_up_to_right_bank_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{0,1,&diag_slope_mask},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{0,1,&diag_slope_mask}}};
const track_section_t left_bank_diag={TRACK_DIAGONAL|TRACK_BANK_LEFT,left_bank_diag_curve,FLAT_DIAG_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
mask_t small_turn_left_bank_masks[]={
	{TRACK_MASK_INTERSECT,13,0,0,small_turn_left_rects},{TRACK_MASK_DIFFERENCE,13,0,0,small_turn_left_rects},{0,13,-32,16,small_turn_left_rects+13},{0,26,0,32,small_turn_left_rects+26},
	{0,17,0,0,small_turn_left_rects+52},{0,16,-32,-16,small_turn_left_rects+69},{0,17,-64,0,small_turn_left_rects+85},
	{0,26,0,0,small_turn_left_rects+102},{0,13,32,-16,small_turn_left_rects+128},{TRACK_MASK_INTERSECT,13,0,-32,small_turn_left_rects+141},{TRACK_MASK_DIFFERENCE,13,0,-32,small_turn_left_rects+141},
	{0,4,0,0,small_turn_left_rects+154},{0,3,32,16,small_turn_left_rects+158},{0,4,64,0,small_turn_left_rects+161}

};
const track_section_t small_turn_left_bank={TRACK_BANK_LEFT,small_turn_left_bank_curve,0.75*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,4,small_turn_left_bank_masks},{0,3,small_turn_left_bank_masks+4},{VIEW_NEEDS_TRACK_MASK,4,small_turn_left_bank_masks+7},{0,3,small_turn_left_bank_masks+11}}};

mask_t medium_turn_left_bank_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects},{0,49,-32,16,medium_turn_left_rects+33},{0,31,0,32,medium_turn_left_rects+82},{0,32,-32,48,medium_turn_left_rects+113},{0,32,0,64,medium_turn_left_rects+145},//TODO proper splitting
{0,17,0,0,medium_turn_left_rects+177},{0,16,-32,-16,medium_turn_left_rects+194},{0,16,-64,0,medium_turn_left_rects+210},{0,16,-96,-16,medium_turn_left_rects+226},{0,17,-128,0,medium_turn_left_rects+242},
{0,33,0,0,medium_turn_left_rects+259},{0,32,32,-16,medium_turn_left_rects+292},{0,31,0,-32,medium_turn_left_rects+324},{0,49,32,-48,medium_turn_left_rects+355},{TRACK_MASK_INTERSECT,32,0,-64,medium_turn_left_rects+404},{TRACK_MASK_DIFFERENCE,32,0,-64,medium_turn_left_rects+404},//TODO proper splitting
{0,33,0,0,medium_turn_left_rects+436},{0,31,32,16,medium_turn_left_rects+469},{0,32,64,0,medium_turn_left_rects+500},{0,31,96,16,medium_turn_left_rects+532},{0,33,128,0,medium_turn_left_rects+563}
};

const track_section_t medium_turn_left_bank={TRACK_BANK_LEFT,medium_turn_left_bank_curve,1.25*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,6,medium_turn_left_bank_masks},{0,5,medium_turn_left_bank_masks+6},{VIEW_NEEDS_TRACK_MASK,6,medium_turn_left_bank_masks+11},{0,5,medium_turn_left_bank_masks+17}}};
rect_t large_turn_left_to_diag_bank_rects[]={
//First view
{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},
{62,-32,INT32_MAX,-31},{60,-31,INT32_MAX,-30},{58,-30,INT32_MAX,-29},{56,-29,INT32_MAX,-28},{54,-28,INT32_MAX,-27},{52,-27,INT32_MAX,-26},{50,-26,INT32_MAX,-25},{48,-25,INT32_MAX,-24},{46,-24,INT32_MAX,-23},{44,-23,INT32_MAX,-22},{42,-22,INT32_MAX,-21},{40,-21,INT32_MAX,-20},{38,-20,INT32_MAX,-19},{36,-19,INT32_MAX,-18},{34,-18,INT32_MAX,-17},{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{2,0,INT32_MAX,1},{4,1,INT32_MAX,2},{6,2,INT32_MAX,3},{8,3,INT32_MAX,4},{10,4,INT32_MAX,5},{12,5,INT32_MAX,6},{14,6,INT32_MAX,7},{16,7,INT32_MAX,8},{18,8,INT32_MAX,9},{20,9,INT32_MAX,10},{22,10,INT32_MAX,11},{24,11,INT32_MAX,12},{26,12,INT32_MAX,13},{28,13,INT32_MAX,14},{30,14,INT32_MAX,15},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,62,-31},{INT32_MIN,-31,-4,-30},{4,-31,60,-30},{INT32_MIN,-30,-6,-29},{6,-30,58,-29},{INT32_MIN,-29,-8,-28},{8,-29,56,-28},{INT32_MIN,-28,-10,-27},{10,-28,54,-27},{INT32_MIN,-27,-12,-26},{12,-27,52,-26},{INT32_MIN,-26,-14,-25},{14,-26,50,-25},{INT32_MIN,-25,-16,-24},{16,-25,48,-24},{INT32_MIN,-24,-18,-23},{18,-24,46,-23},{INT32_MIN,-23,-20,-22},{20,-23,44,-22},{INT32_MIN,-22,-22,-21},{22,-22,42,-21},{INT32_MIN,-21,-24,-20},{24,-21,40,-20},{INT32_MIN,-20,-26,-19},{26,-20,38,-19},{INT32_MIN,-19,-28,-18},{28,-19,36,-18},{INT32_MIN,-18,-30,-17},{30,-18,34,-17},
//Second view
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,INT32_MAX},
{0,INT32_MIN,64,32},{2,32,64,33},{4,33,64,34},{6,34,64,35},{8,35,64,36},{10,36,64,37},{12,37,64,38},{14,38,64,39},{16,39,64,40},{18,40,64,41},{20,41,64,42},{22,42,64,43},{24,43,64,44},{26,44,64,45},{28,45,64,46},{30,46,64,47},{32,47,64,INT32_MAX},
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
{0,33,0,0,large_turn_left_to_diag_bank_rects+142},{0,17,-32,-16,large_turn_left_to_diag_bank_rects+175},{0,1,-64,0,large_turn_left_to_diag_bank_rects+192},{0,1,-96,-16,large_turn_left_to_diag_bank_rects+193},
{0,16,0,0,large_turn_left_to_diag_bank_rects+194},{0,32,32,-16,large_turn_left_to_diag_bank_rects+210},{0,33,0,-32,large_turn_left_to_diag_bank_rects+242},{0,33,32,-48,large_turn_left_to_diag_bank_rects+275},
{0,17,0,0,large_turn_left_to_diag_bank_rects+308},{0,16,32,16,large_turn_left_to_diag_bank_rects+325},{0,16,64,0,large_turn_left_to_diag_bank_rects+341},{0,17,96,16,large_turn_left_to_diag_bank_rects+357},
};
const track_section_t large_turn_left_to_diag_bank={TRACK_BANK_LEFT,large_turn_left_to_diag_bank_curve,0.875*TILE_SIZE*3.1415926,{{0,4,large_turn_left_to_diag_bank_masks},{0,4,large_turn_left_to_diag_bank_masks+4},{0,4,large_turn_left_to_diag_bank_masks+8},{0,4,large_turn_left_to_diag_bank_masks+12}}};
rect_t large_turn_right_to_diag_bank_rects[]={
//First angle
{INT32_MIN,INT32_MIN,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,INT32_MAX},
{0,INT32_MIN,64,0},{2,0,62,1},{4,1,60,2},{6,2,58,3},{8,3,56,4},{10,4,54,5},{12,5,52,6},{14,6,50,7},{16,7,48,8},{18,8,46,9},{20,9,44,10},{22,10,42,11},{24,11,40,12},{26,12,38,13},{28,13,36,14},{30,14,34,15},
{62,0,66,1},{60,1,68,2},{58,2,70,3},{56,3,72,4},{54,4,74,5},{52,5,76,6},{50,6,78,7},{48,7,80,8},{46,8,82,9},{44,9,84,10},{42,10,86,11},{40,11,88,12},{38,12,90,13},{36,13,92,14},{34,14,94,15},{32,15,96,INT32_MAX},
{64,INT32_MIN,INT32_MAX,0},{66,0,INT32_MAX,1},{68,1,INT32_MAX,2},{70,2,INT32_MAX,3},{72,3,INT32_MAX,4},{74,4,INT32_MAX,5},{76,5,INT32_MAX,6},{78,6,INT32_MAX,7},{80,7,INT32_MAX,8},{82,8,INT32_MAX,9},{84,9,INT32_MAX,10},{86,10,INT32_MAX,11},{88,11,INT32_MAX,12},{90,12,INT32_MAX,13},{92,13,INT32_MAX,14},{94,14,INT32_MAX,15},{96,15,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},
{32,INT32_MIN,INT32_MAX,16},{30,16,INT32_MAX,17},{28,17,INT32_MAX,18},{26,18,INT32_MAX,19},{24,19,INT32_MAX,20},{22,20,INT32_MAX,21},{20,21,INT32_MAX,22},{18,22,INT32_MAX,23},{16,23,INT32_MAX,24},{14,24,INT32_MAX,25},{12,25,INT32_MAX,26},{10,26,INT32_MAX,27},{8,27,INT32_MAX,28},{6,28,INT32_MAX,29},{4,29,INT32_MAX,30},{2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{2,32,62,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},
{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,48},{INT32_MIN,48,30,49},{INT32_MIN,49,28,50},{INT32_MIN,50,26,51},{INT32_MIN,51,24,52},{INT32_MIN,52,22,53},{INT32_MIN,53,20,54},{INT32_MIN,54,18,55},{INT32_MIN,55,16,56},{INT32_MIN,56,14,57},{INT32_MIN,57,12,58},{INT32_MIN,58,10,59},{INT32_MIN,59,8,60},{INT32_MIN,60,6,61},{INT32_MIN,61,4,62},{INT32_MIN,62,2,63},{INT32_MIN,63,0,64},{INT32_MIN,64,-2,INT32_MAX},
{62,32,INT32_MAX,33},{60,33,INT32_MAX,34},{58,34,INT32_MAX,35},{56,35,INT32_MAX,36},{54,36,INT32_MAX,37},{52,37,INT32_MAX,38},{50,38,INT32_MAX,39},{48,39,INT32_MAX,40},{46,40,INT32_MAX,41},{44,41,INT32_MAX,42},{42,42,INT32_MAX,43},{40,43,INT32_MAX,44},{38,44,INT32_MAX,45},{36,45,INT32_MAX,46},{34,46,INT32_MAX,47},{32,47,INT32_MAX,48},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{-2,64,INT32_MAX,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},{0,31,INT32_MAX,INT32_MAX},
{-64,INT32_MIN,0,INT32_MAX},
{-64,INT32_MIN,-64,INT32_MAX},
{INT32_MIN,INT32_MIN,-64,INT32_MAX},
//Fourth angle
{32,-17,INT32_MAX,-16},{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{-32,-10,-18,-9},{18,-10,INT32_MAX,-9},{-32,-9,-16,-8},{16,-9,INT32_MAX,-8},{-32,-8,-14,-7},{14,-8,INT32_MAX,-7},{-32,-7,-12,-6},{12,-7,INT32_MAX,-6},{-32,-6,-10,-5},{10,-6,INT32_MAX,-5},{-32,-5,-8,-4},{8,-5,INT32_MAX,-4},{-32,-4,-6,-3},{6,-4,INT32_MAX,-3},{-32,-3,-4,-2},{4,-3,INT32_MAX,-2},{-32,-2,-2,-1},{2,-2,INT32_MAX,-1},{-32,-1,INT32_MAX,15},{INT32_MIN,15,INT32_MAX,INT32_MAX},

{INT32_MIN,-26,-50,-25},{INT32_MIN,-25,-48,-24},{INT32_MIN,-24,-46,-23},{INT32_MIN,-23,-44,-22},{INT32_MIN,-22,-42,-21},{INT32_MIN,-21,-40,-20},{INT32_MIN,-20,-38,-19},{INT32_MIN,-19,-36,-18},{INT32_MIN,-18,-34,-17},{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,-2,1},{INT32_MIN,1,-4,2},{INT32_MIN,2,-6,3},{INT32_MIN,3,-8,4},{INT32_MIN,4,-10,5},{INT32_MIN,5,-12,6},{INT32_MIN,6,-14,7},{INT32_MIN,7,-16,8},{INT32_MIN,8,-18,9},{INT32_MIN,9,-20,10},{INT32_MIN,10,-22,11},{INT32_MIN,11,-24,12},{INT32_MIN,12,-26,13},{INT32_MIN,13,-28,14},{INT32_MIN,14,-30,15},

{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},

{INT32_MIN,INT32_MIN,INT32_MAX,-32},{INT32_MIN,-32,-2,-31},{2,-32,INT32_MAX,-31},{INT32_MIN,-31,-4,-30},{4,-31,INT32_MAX,-30},{INT32_MIN,-30,-6,-29},{6,-30,INT32_MAX,-29},{INT32_MIN,-29,-8,-28},{8,-29,INT32_MAX,-28},{INT32_MIN,-28,-10,-27},{10,-28,INT32_MAX,-27},{INT32_MIN,-27,-12,-26},{12,-27,INT32_MAX,-26},{-50,-26,-14,-25},{14,-26,INT32_MAX,-25},{-48,-25,-16,-24},{16,-25,INT32_MAX,-24},{-46,-24,-18,-23},{18,-24,INT32_MAX,-23},{-44,-23,-20,-22},{20,-23,INT32_MAX,-22},{-42,-22,-22,-21},{22,-22,INT32_MAX,-21},{-40,-21,-24,-20},{24,-21,INT32_MAX,-20},{-38,-20,-26,-19},{26,-20,INT32_MAX,-19},{-36,-19,-28,-18},{28,-19,INT32_MAX,-18},{-34,-18,-30,-17},{30,-18,INT32_MAX,-17},

};//TODO consider the need for track masks on this and the previous
mask_t large_turn_right_to_diag_bank_masks[]={
{0,17,0,0,large_turn_right_to_diag_bank_rects},{0,16,-32,16,large_turn_right_to_diag_bank_rects+17},{0,16,-64,0,large_turn_right_to_diag_bank_rects+33},{0,17,-96,16,large_turn_right_to_diag_bank_rects+49},
{0,16,0,0,large_turn_right_to_diag_bank_rects+66},{0,32,-32,-16,large_turn_right_to_diag_bank_rects+82},{0,34,0,-32,large_turn_right_to_diag_bank_rects+114},{0,33,-32,-48,large_turn_right_to_diag_bank_rects+148},
{0,17,0,0,large_turn_right_to_diag_bank_rects+181},{0,1,32,-16,large_turn_right_to_diag_bank_rects+198},{0,1,64,0,large_turn_right_to_diag_bank_rects+199},{0,1,96,-16,large_turn_right_to_diag_bank_rects+200},
{0,27,0,0,large_turn_right_to_diag_bank_rects+201},{0,41,32,16,large_turn_right_to_diag_bank_rects+228},{0,31,0,32,large_turn_right_to_diag_bank_rects+269},{0,31,32,48,large_turn_right_to_diag_bank_rects+300},
};
const track_section_t large_turn_right_to_diag_bank={TRACK_BANK_RIGHT,large_turn_right_to_diag_bank_curve,0.875*TILE_SIZE*3.1415926,{{0,4,large_turn_right_to_diag_bank_masks},{0,4,large_turn_right_to_diag_bank_masks+4},{0,4,large_turn_right_to_diag_bank_masks+8},{0,4,large_turn_right_to_diag_bank_masks+12}}};

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
const track_section_t small_turn_left_gentle_up={TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,small_turn_left_gentle_up_curve,SMALL_TURN_GENTLE_LENGTH,{{0,2,small_turn_left_gentle_up_masks},{0,2,small_turn_left_gentle_up_masks+2},{0,2,small_turn_left_gentle_up_masks+4},{0,2,small_turn_left_gentle_up_masks+6}}};
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
const track_section_t small_turn_right_gentle_up={TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,small_turn_right_gentle_up_curve,SMALL_TURN_GENTLE_LENGTH,{{0,2,small_turn_right_gentle_up_masks},{0,2,small_turn_right_gentle_up_masks+2},{0,2,small_turn_right_gentle_up_masks+4},{0,2,small_turn_right_gentle_up_masks+6}}};
rect_t medium_turn_left_gentle_up_rects[]={
//First angle
{INT32_MIN,-46,0,-18},{INT32_MIN,-18,32,INT32_MAX},
{0,-46,INT32_MAX,-18},{32,-18,INT32_MAX,INT32_MAX},
{0,0,0,0},
{32,INT32_MIN,INT32_MAX,-91},{0,-91,INT32_MAX,-46},
{INT32_MIN,INT32_MIN,32,-91},{INT32_MIN,-91,0,-46},
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
{0,INT32_MIN,INT32_MAX,-43},{-1,-43,INT32_MAX,-42},{-2,-42,INT32_MAX,-40},{-3,-40,INT32_MAX,-39},{-4,-39,INT32_MAX,-38},{-5,-38,INT32_MAX,-36},{-6,-36,INT32_MAX,-35},{-7,-35,INT32_MAX,-34},{-8,-34,INT32_MAX,-32},{-9,-32,INT32_MAX,-31},{-10,-31,INT32_MAX,-30},{-11,-30,INT32_MAX,-28},{-12,-28,INT32_MAX,-27},{-13,-27,INT32_MAX,-26},{-14,-26,INT32_MAX,-24},{-15,-24,INT32_MAX,-23},{-16,-23,INT32_MAX,-22},{-17,-22,INT32_MAX,-20},{-18,-20,INT32_MAX,-19},{-19,-19,INT32_MAX,-18},{-20,-18,INT32_MAX,-16},{-21,-16,INT32_MAX,-15},{-22,-15,INT32_MAX,-14},{-23,-14,INT32_MAX,-12},{-24,-12,INT32_MAX,-11},{-25,-11,INT32_MAX,-10},{-26,-10,INT32_MAX,-8},{-27,-8,INT32_MAX,-7},{-28,-7,INT32_MAX,-6},{-29,-6,INT32_MAX,-4},{-30,-4,INT32_MAX,-3},{-31,-3,INT32_MAX,-2},{-32,-2,INT32_MAX,INT32_MAX},
{-64,INT32_MIN,0,-43},{-64,-43,-1,-42},{-64,-42,-2,-40},{-64,-40,-3,-39},{-64,-39,-4,-38},{-64,-38,-5,-36},{-64,-36,-6,-35},{-64,-35,-7,-34},{-64,-34,-8,-32},{-64,-32,-9,-31},{-64,-31,-10,-30},{-64,-30,-11,-28},{-64,-28,-12,-27},{-64,-27,-13,-26},{-64,-26,-14,-24},{-64,-24,-15,-23},{-64,-23,-16,-22},{-64,-22,-17,-20},{-64,-20,-18,-19},{-62,-19,-19,-18},{-60,-18,-20,-17},{-58,-17,-20,-16},{-56,-16,-21,-15},{-54,-15,-22,-14},{-52,-14,-23,-13},{-50,-13,-23,-12},{-48,-12,-24,-11},{-46,-11,-25,-10},{-44,-10,-26,-9},{-42,-9,-26,-8},{-40,-8,-27,-7},{-38,-7,-28,-6},{-36,-6,-29,-5},{-34,-5,-29,-4},{-32,-4,-30,-3},{-32,-3,-31,-2},
{-65,-92,-64,-88},{-66,-88,-64,-85},{-67,-85,-64,-82},{-68,-82,-64,-78},{-69,-78,-64,-75},{-70,-75,-64,-72},{-71,-72,-64,-68},{-72,-68,-64,-65},{-73,-65,-64,-62},{-74,-62,-64,-58},{-75,-58,-64,-55},{-76,-55,-64,-52},{-77,-52,-64,-48},{-78,-48,-64,-45},{-79,-45,-64,-42},{-80,-42,-64,-38},{-81,-38,-64,-35},{-82,-35,-64,-32},{-83,-32,-64,-28},{-84,-28,-64,-25},{-85,-25,-64,-22},{-86,-22,-64,-19},{-86,-19,-62,-18},{-87,-18,-60,-17},{-87,-17,-58,-16},{-87,-16,-56,-15},{-88,-15,-54,-14},{-88,-14,-52,-13},{-88,-13,-50,-12},{-89,-12,-48,-11},{-89,-11,-46,-10},{-89,-10,-44,-9},{-89,-9,-42,-8},{-90,-8,-40,-7},{-90,-7,-38,-6},{-90,-6,-36,-5},{-91,-5,-34,-4},{-91,-4,-32,-2},{-92,-2,-32,2},{-93,2,-32,5},{-94,5,-32,9},{-95,9,-32,12},{-96,12,-32,INT32_MAX},
{-126,INT32_MIN,-64,-92},{-126,-92,-65,-88},{-126,-88,-66,-85},{-126,-85,-67,-82},{-126,-82,-68,-78},{-126,-78,-69,-75},{-126,-75,-70,-72},{-126,-72,-71,-69},{-125,-69,-71,-68},{-125,-68,-72,-67},{-124,-67,-72,-65},{-123,-65,-73,-64},{-122,-64,-73,-62},{-121,-62,-74,-61},{-120,-61,-74,-59},{-119,-59,-74,-58},{-119,-58,-75,-57},{-118,-57,-75,-56},{-117,-56,-75,-55},{-117,-55,-76,-54},{-116,-54,-76,-53},{-115,-53,-76,-52},{-115,-52,-77,-51},{-114,-51,-77,-50},{-113,-50,-77,-48},{-112,-48,-78,-46},{-111,-46,-78,-45},{-111,-45,-79,-44},{-110,-44,-79,-43},{-109,-43,-79,-42},{-109,-42,-80,-41},{-108,-41,-80,-40},{-107,-40,-80,-38},{-106,-38,-81,-37},{-105,-37,-81,-35},{-104,-35,-82,-33},{-103,-33,-82,-32},{-102,-32,-83,-30},{-101,-30,-83,-29},{-100,-29,-83,-28},{-100,-28,-84,-27},{-99,-27,-84,-26},{-98,-26,-84,-25},{-98,-25,-85,-24},{-97,-24,-85,-22},{-96,-22,-86,-21},{-95,-21,-86,-19},{-94,-19,-86,-18},{-93,-18,-87,-16},{-92,-16,-87,-15},{-91,-15,-88,-13},{-90,-13,-88,-12},{-90,-12,-89,-11},
{INT32_MIN,INT32_MIN,-126,-69},{INT32_MIN,-69,-125,-67},{INT32_MIN,-67,-124,-65},{INT32_MIN,-65,-123,-64},{INT32_MIN,-64,-122,-62},{INT32_MIN,-62,-121,-61},{INT32_MIN,-61,-120,-59},{INT32_MIN,-59,-119,-57},{INT32_MIN,-57,-118,-56},{INT32_MIN,-56,-117,-54},{INT32_MIN,-54,-116,-53},{INT32_MIN,-53,-115,-51},{INT32_MIN,-51,-114,-50},{INT32_MIN,-50,-113,-48},{INT32_MIN,-48,-112,-46},{INT32_MIN,-46,-111,-44},{INT32_MIN,-44,-110,-43},{INT32_MIN,-43,-109,-41},{INT32_MIN,-41,-108,-40},{INT32_MIN,-40,-107,-38},{INT32_MIN,-38,-106,-37},{INT32_MIN,-37,-105,-35},{INT32_MIN,-35,-104,-33},{INT32_MIN,-33,-103,-32},{INT32_MIN,-32,-102,-30},{INT32_MIN,-30,-101,-29},{INT32_MIN,-29,-100,-27},{INT32_MIN,-27,-99,-26},{INT32_MIN,-26,-98,-24},{INT32_MIN,-24,-97,-22},{INT32_MIN,-22,-96,-21},{INT32_MIN,-21,-95,-19},{INT32_MIN,-19,-94,-18},{INT32_MIN,-18,-93,-16},{INT32_MIN,-16,-92,-15},{INT32_MIN,-15,-91,-13},{INT32_MIN,-13,-90,-11},{INT32_MIN,-11,-89,-8},{INT32_MIN,-8,-90,-5},{INT32_MIN,-5,-91,-2},{INT32_MIN,-2,-92,2},{INT32_MIN,2,-93,5},{INT32_MIN,5,-94,9},{INT32_MIN,9,-95,12},{INT32_MIN,12,-96,INT32_MAX},};
mask_t medium_turn_left_gentle_up_masks[]={
{0,2,0,0,medium_turn_left_gentle_up_rects},{0,2,-32,32,medium_turn_left_gentle_up_rects+2},{0,1,0,0,medium_turn_left_gentle_up_rects+4},{0,2,-32,80,medium_turn_left_gentle_up_rects+5},{0,2,0,112,medium_turn_left_gentle_up_rects+7},
{0,1,0,0,medium_turn_left_gentle_up_rects+9},{0,1,-32,0,medium_turn_left_gentle_up_rects+10},{0,32,-64,24,medium_turn_left_gentle_up_rects+11},{0,76,-96,16,medium_turn_left_gentle_up_rects+43},{0,65,-128,48,medium_turn_left_gentle_up_rects+119},
{0,1,0,0,medium_turn_left_gentle_up_rects+184},{0,1,0,0,medium_turn_left_gentle_up_rects+185},{0,1,0,-8,medium_turn_left_gentle_up_rects+186},{0,1,0,0,medium_turn_left_gentle_up_rects+187},{0,2,0,-16,medium_turn_left_gentle_up_rects+188},
{0,33,0,0,medium_turn_left_gentle_up_rects+190},{0,36,32,32,medium_turn_left_gentle_up_rects+223},{0,43,64,24,medium_turn_left_gentle_up_rects+259},{0,53,96,48,medium_turn_left_gentle_up_rects+302},{0,45,128,48,medium_turn_left_gentle_up_rects+355}
};
const track_section_t medium_turn_left_gentle_up={TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,medium_turn_left_gentle_up_curve,MEDIUM_TURN_GENTLE_LENGTH,{{0,5,medium_turn_left_gentle_up_masks},{0,5,medium_turn_left_gentle_up_masks+5},{0,5,medium_turn_left_gentle_up_masks+10},{0,5,medium_turn_left_gentle_up_masks+15}}};
rect_t medium_turn_right_gentle_up_rects[]={
//First angle
{INT32_MIN,INT32_MIN,0,-43},{INT32_MIN,-43,1,-42},{INT32_MIN,-42,2,-40},{INT32_MIN,-40,3,-39},{INT32_MIN,-39,4,-38},{INT32_MIN,-38,5,-36},{INT32_MIN,-36,6,-35},{INT32_MIN,-35,7,-34},{INT32_MIN,-34,8,-32},{INT32_MIN,-32,9,-31},{INT32_MIN,-31,10,-30},{INT32_MIN,-30,11,-28},{INT32_MIN,-28,12,-27},{INT32_MIN,-27,13,-26},{INT32_MIN,-26,14,-24},{INT32_MIN,-24,15,-23},{INT32_MIN,-23,16,-22},{INT32_MIN,-22,17,-20},{INT32_MIN,-20,18,-19},{INT32_MIN,-19,19,-18},{INT32_MIN,-18,20,-16},{INT32_MIN,-16,21,-15},{INT32_MIN,-15,22,-14},{INT32_MIN,-14,23,-12},{INT32_MIN,-12,24,-11},{INT32_MIN,-11,25,-10},{INT32_MIN,-10,26,-8},{INT32_MIN,-8,27,-7},{INT32_MIN,-7,28,-6},{INT32_MIN,-6,29,-4},{INT32_MIN,-4,30,-3},{INT32_MIN,-3,31,-2},{INT32_MIN,-2,32,INT32_MAX},
{0,INT32_MIN,64,-43},{1,-43,64,-42},{2,-42,64,-40},{3,-40,64,-39},{4,-39,64,-38},{5,-38,64,-36},{6,-36,64,-35},{7,-35,64,-34},{8,-34,64,-32},{9,-32,64,-31},{10,-31,64,-30},{11,-30,64,-28},{12,-28,64,-27},{13,-27,64,-26},{14,-26,64,-24},{15,-24,64,-23},{16,-23,64,-22},{17,-22,64,-20},{18,-20,64,-19},{19,-19,62,-18},{20,-18,60,-17},{20,-17,58,-16},{21,-16,56,-15},{22,-15,54,-14},{23,-14,52,-13},{23,-13,50,-12},{24,-12,48,-11},{25,-11,46,-10},{26,-10,44,-9},{26,-9,42,-8},{27,-8,40,-7},{28,-7,38,-6},{29,-6,36,-5},{29,-5,34,-4},{30,-4,32,-3},{31,-3,32,-2},
{64,-92,65,-88},{64,-88,66,-85},{64,-85,67,-82},{64,-82,68,-78},{64,-78,69,-75},{64,-75,70,-72},{64,-72,71,-68},{64,-68,72,-65},{64,-65,73,-62},{64,-62,74,-58},{64,-58,75,-55},{64,-55,76,-52},{64,-52,77,-48},{64,-48,78,-45},{64,-45,79,-42},{64,-42,80,-38},{64,-38,81,-35},{64,-35,82,-32},{64,-32,83,-28},{64,-28,84,-25},{64,-25,85,-22},{64,-22,86,-19},{62,-19,86,-18},{60,-18,87,-17},{58,-17,87,-16},{56,-16,87,-15},{54,-15,88,-14},{52,-14,88,-13},{50,-13,88,-12},{48,-12,89,-11},{46,-11,89,-10},{44,-10,89,-9},{42,-9,89,-8},{40,-8,90,-7},{38,-7,90,-6},{36,-6,90,-5},{34,-5,91,-4},{32,-4,91,-2},{32,-2,92,2},{32,2,93,5},{32,5,94,9},{32,9,95,12},{32,12,96,INT32_MAX},
{64,INT32_MIN,126,-92},{65,-92,126,-88},{66,-88,126,-85},{67,-85,126,-82},{68,-82,126,-78},{69,-78,126,-75},{70,-75,126,-72},{71,-72,126,-69},{71,-69,125,-68},{72,-68,125,-67},{72,-67,124,-65},{73,-65,123,-64},{73,-64,122,-62},{74,-62,121,-61},{74,-61,120,-59},{74,-59,119,-58},{75,-58,119,-57},{75,-57,118,-56},{75,-56,117,-55},{76,-55,117,-54},{76,-54,116,-53},{76,-53,115,-52},{77,-52,115,-51},{77,-51,114,-50},{77,-50,113,-48},{78,-48,112,-46},{78,-46,111,-45},{79,-45,111,-44},{79,-44,110,-43},{79,-43,109,-42},{80,-42,109,-41},{80,-41,108,-40},{80,-40,107,-38},{81,-38,106,-37},{81,-37,105,-35},{82,-35,104,-33},{82,-33,103,-32},{83,-32,102,-30},{83,-30,101,-29},{83,-29,100,-28},{84,-28,100,-27},{84,-27,99,-26},{84,-26,98,-25},{85,-25,98,-24},{85,-24,97,-22},{86,-22,96,-21},{86,-21,95,-19},{86,-19,94,-18},{87,-18,93,-16},{87,-16,92,-15},{88,-15,91,-13},{88,-13,90,-12},{89,-12,90,-11},
{126,INT32_MIN,INT32_MAX,-69},{125,-69,INT32_MAX,-67},{124,-67,INT32_MAX,-65},{123,-65,INT32_MAX,-64},{122,-64,INT32_MAX,-62},{121,-62,INT32_MAX,-61},{120,-61,INT32_MAX,-59},{119,-59,INT32_MAX,-57},{118,-57,INT32_MAX,-56},{117,-56,INT32_MAX,-54},{116,-54,INT32_MAX,-53},{115,-53,INT32_MAX,-51},{114,-51,INT32_MAX,-50},{113,-50,INT32_MAX,-48},{112,-48,INT32_MAX,-46},{111,-46,INT32_MAX,-44},{110,-44,INT32_MAX,-43},{109,-43,INT32_MAX,-41},{108,-41,INT32_MAX,-40},{107,-40,INT32_MAX,-38},{106,-38,INT32_MAX,-37},{105,-37,INT32_MAX,-35},{104,-35,INT32_MAX,-33},{103,-33,INT32_MAX,-32},{102,-32,INT32_MAX,-30},{101,-30,INT32_MAX,-29},{100,-29,INT32_MAX,-27},{99,-27,INT32_MAX,-26},{98,-26,INT32_MAX,-24},{97,-24,INT32_MAX,-22},{96,-22,INT32_MAX,-21},{95,-21,INT32_MAX,-19},{94,-19,INT32_MAX,-18},{93,-18,INT32_MAX,-16},{92,-16,INT32_MAX,-15},{91,-15,INT32_MAX,-13},{90,-13,INT32_MAX,-11},{89,-11,INT32_MAX,-8},{90,-8,INT32_MAX,-5},{91,-5,INT32_MAX,-2},{92,-2,INT32_MAX,2},{93,2,INT32_MAX,5},{94,5,INT32_MAX,9},{95,9,INT32_MAX,12},{96,12,INT32_MAX,INT32_MAX},
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
{0,-46,INT32_MAX,-16},{-32,-16,INT32_MAX,INT32_MAX},
{INT32_MIN,-46,0,-16},{INT32_MIN,-16,-32,INT32_MAX},
{0,0,0,0},
{INT32_MIN,INT32_MIN,-32,-90},{INT32_MIN,-90,0,-46},
{-32,INT32_MIN,INT32_MAX,-90},{0,-90,INT32_MAX,-46},
};
mask_t medium_turn_right_gentle_up_masks[]={
{0,33,0,0,medium_turn_right_gentle_up_rects},{0,36,-32,32,medium_turn_right_gentle_up_rects+33},{0,43,-64,24,medium_turn_right_gentle_up_rects+69},{0,53,-96,48,medium_turn_right_gentle_up_rects+112},{0,45,-128,48,medium_turn_right_gentle_up_rects+165},
{0,1,0,0,medium_turn_right_gentle_up_rects+210},{0,1,-32,0,medium_turn_right_gentle_up_rects+211},{0,1,0,-8,medium_turn_right_gentle_up_rects+212},{0,1,-32,-8,medium_turn_right_gentle_up_rects+213},{0,2,0,-16,medium_turn_right_gentle_up_rects+214},
{0,1,0,0,medium_turn_right_gentle_up_rects+216},{0,1,32,0,medium_turn_right_gentle_up_rects+217},{0,21,64,24,medium_turn_right_gentle_up_rects+218},{0,53,96,16,medium_turn_right_gentle_up_rects+239},{0,40,128,48,medium_turn_right_gentle_up_rects+292},
{0,2,0,0,medium_turn_right_gentle_up_rects+332},{0,2,32,32,medium_turn_right_gentle_up_rects+334},{0,1,0,56,medium_turn_right_gentle_up_rects+336},{0,2,32,80,medium_turn_right_gentle_up_rects+337},{0,2,0,112,medium_turn_right_gentle_up_rects+339}
};
const track_section_t medium_turn_right_gentle_up={TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,medium_turn_right_gentle_up_curve,MEDIUM_TURN_GENTLE_LENGTH,{{0,5,medium_turn_right_gentle_up_masks},{0,5,medium_turn_right_gentle_up_masks+5},{0,5,medium_turn_right_gentle_up_masks+10},{0,5,medium_turn_right_gentle_up_masks+15}}};
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
const track_section_t very_small_turn_left_steep_up={TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,very_small_turn_left_steep_up_curve,VERY_SMALL_TURN_STEEP_LENGTH,{{0,2,very_small_turn_left_steep_up_masks},{VIEW_NEEDS_TRACK_MASK,2,very_small_turn_left_steep_up_masks+2},{0,2,very_small_turn_left_steep_up_masks+4},{VIEW_NEEDS_TRACK_MASK,2,very_small_turn_left_steep_up_masks+6}}};
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
const track_section_t very_small_turn_right_steep_up={TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,very_small_turn_right_steep_up_curve,VERY_SMALL_TURN_STEEP_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,very_small_turn_right_steep_up_masks},{0,2,very_small_turn_right_steep_up_masks+2},{VIEW_NEEDS_TRACK_MASK,2,very_small_turn_right_steep_up_masks+4},{0,2,very_small_turn_right_steep_up_masks+6}}};
rect_t vertical_twist_left_up_rects[]={
{INT32_MIN,INT32_MIN,INT32_MAX,-44},{INT32_MIN,-44,INT32_MAX,INT32_MAX},
{INT32_MIN,-43,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,-43},
};
mask_t vertical_twist_left_up_masks[]={
{0,1,0,0,vertical_twist_left_up_rects},{0,1,0,0,vertical_twist_left_up_rects+1},
{0,1,0,0,vertical_twist_left_up_rects+2},{0,1,0,0,vertical_twist_left_up_rects+3}
};
const track_section_t vertical_twist_left_up={TRACK_VERTICAL|TRACK_OFFSET_SPRITE_MASK|TRACK_NO_SUPPORTS|TRACK_SPECIAL_VERTICAL_TWIST_LEFT,vertical_twist_left_up_curve,VERTICAL_TWIST_LENGTH,{{0,1,NULL},{0,2,vertical_twist_left_up_masks},{0,1,NULL},{0,2,vertical_twist_left_up_masks+2}}};
rect_t vertical_twist_right_up_rects[]={
{INT32_MIN,-44,INT32_MAX,INT32_MAX},{INT32_MIN,INT32_MIN,INT32_MAX,-44},
{INT32_MIN,INT32_MIN,INT32_MAX,-44},{INT32_MIN,-44,INT32_MAX,INT32_MAX},
};
mask_t vertical_twist_right_up_masks[]={
{0,1,0,0,vertical_twist_right_up_rects},{0,1,0,0,vertical_twist_right_up_rects+1},
{0,1,0,0,vertical_twist_right_up_rects+2},{0,1,0,0,vertical_twist_right_up_rects+3}
};
const track_section_t vertical_twist_right_up={TRACK_VERTICAL|TRACK_OFFSET_SPRITE_MASK|TRACK_NO_SUPPORTS|TRACK_SPECIAL_VERTICAL_TWIST_RIGHT,vertical_twist_right_up_curve,VERTICAL_TWIST_LENGTH,{{0,2,vertical_twist_right_up_masks},{0,1,NULL},{0,2,vertical_twist_right_up_masks+2},{0,1,NULL}}};

//Banked sloped turns
mask_t gentle_up_to_gentle_up_left_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t gentle_up_to_gentle_up_left_bank={TRACK_EXIT_BANK_LEFT,gentle_up_to_gentle_up_left_bank_curve,GENTLE_LENGTH,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_left_bank_masks},{0,1,NULL},{0,1,NULL}}};
mask_t gentle_up_to_gentle_up_right_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t gentle_up_to_gentle_up_right_bank={TRACK_EXIT_BANK_RIGHT,gentle_up_to_gentle_up_right_bank_curve,GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_right_bank_masks},{0,1,NULL}}};


rect_t gentle_up_left_bank_to_gentle_up_rects[]={
{16,INT32_MIN,INT32_MAX,INT32_MAX},{0,0,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,0,INT32_MAX},{0,INT32_MIN,16,0}
};
mask_t gentle_up_left_bank_to_gentle_up_masks[]={{TRACK_MASK_UNION,2,0,0,gentle_up_left_bank_to_gentle_up_rects+0},{TRACK_MASK_DIFFERENCE,2,0,0,gentle_up_left_bank_to_gentle_up_rects+2}};
const track_section_t gentle_up_left_bank_to_gentle_up={TRACK_ENTRY_BANK_LEFT|TRACK_OFFSET_SPRITE_MASK,gentle_up_left_bank_to_gentle_up_curve,GENTLE_LENGTH,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_left_bank_to_gentle_up_masks},{0,1,NULL},{0,1,NULL}}};
rect_t gentle_up_right_bank_to_gentle_up_rects[]={
{INT32_MIN,INT32_MIN,-16,INT32_MAX},{INT32_MIN,0,0,INT32_MAX},
{0,INT32_MIN,INT32_MAX,INT32_MAX},{-16,INT32_MIN,INT32_MAX,0}
};
mask_t gentle_up_right_bank_to_gentle_up_masks[]={{TRACK_MASK_UNION,2,0,0,gentle_up_right_bank_to_gentle_up_rects+0},{TRACK_MASK_DIFFERENCE,2,0,0,gentle_up_right_bank_to_gentle_up_rects+2}};
const track_section_t gentle_up_right_bank_to_gentle_up={TRACK_ENTRY_BANK_RIGHT|TRACK_OFFSET_SPRITE_MASK,gentle_up_right_bank_to_gentle_up_curve,GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_right_bank_to_gentle_up_masks},{0,1,NULL}}};

const track_section_t left_bank_to_gentle_up_left_bank={TRACK_BANK_LEFT,left_bank_to_gentle_up_left_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t right_bank_to_gentle_up_right_bank={TRACK_BANK_RIGHT,right_bank_to_gentle_up_right_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t gentle_up_left_bank_to_left_bank={TRACK_BANK_LEFT,gentle_up_left_bank_to_left_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t gentle_up_right_bank_to_right_bank={TRACK_BANK_RIGHT,gentle_up_right_bank_to_right_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};

const track_section_t gentle_up_left_bank={TRACK_BANK_LEFT,gentle_up_left_bank_curve,GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};
const track_section_t gentle_up_right_bank={TRACK_BANK_RIGHT,gentle_up_right_bank_curve,GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};

mask_t flat_to_gentle_up_left_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t flat_to_gentle_up_left_bank={TRACK_EXIT_BANK_LEFT,flat_to_gentle_up_left_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_left_bank_masks},{0,1,NULL},{0,1,NULL}}};
mask_t flat_to_gentle_up_right_bank_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t flat_to_gentle_up_right_bank={TRACK_EXIT_BANK_RIGHT,flat_to_gentle_up_right_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_gentle_up_right_bank_masks},{0,1,NULL}}};


rect_t gentle_up_left_bank_to_flat_rects[]={
{16,INT32_MIN,INT32_MAX,INT32_MAX},{0,8,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,0,INT32_MAX},{INT32_MIN,INT32_MIN,16,8}
};
mask_t gentle_up_left_bank_to_flat_masks[]={
{TRACK_MASK_UNION,2,0,0,gentle_up_left_bank_to_flat_rects+0},{TRACK_MASK_DIFFERENCE,2,0,0,gentle_up_left_bank_to_flat_rects+2}
};
const track_section_t gentle_up_left_bank_to_flat={TRACK_ENTRY_BANK_LEFT|TRACK_OFFSET_SPRITE_MASK,gentle_up_left_bank_to_flat_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_left_bank_to_flat_masks},{0,1,NULL},{0,1,NULL}}};
rect_t gentle_up_right_bank_to_flat_rects[]={
{INT32_MIN,INT32_MIN,-16,INT32_MAX},{INT32_MIN,8,0,INT32_MAX},
{0,INT32_MIN,INT32_MAX,INT32_MAX},{-16,INT32_MIN,INT32_MAX,8}
};
mask_t gentle_up_right_bank_to_flat_masks[]={{TRACK_MASK_UNION,2,0,0,gentle_up_right_bank_to_flat_rects+0},{TRACK_MASK_DIFFERENCE,2,0,0,gentle_up_right_bank_to_flat_rects+2}};
const track_section_t gentle_up_right_bank_to_flat={TRACK_ENTRY_BANK_RIGHT|TRACK_OFFSET_SPRITE_MASK,gentle_up_right_bank_to_flat_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_right_bank_to_flat_masks},{0,1,NULL}}};





const track_section_t small_turn_left_bank_gentle_up={TRACK_BANK_LEFT|TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,small_turn_left_bank_gentle_up_curve,SMALL_TURN_GENTLE_LENGTH,{{0,2,small_turn_left_gentle_up_masks},{0,2,small_turn_left_gentle_up_masks+2},{0,2,small_turn_left_gentle_up_masks+4},{0,2,small_turn_left_gentle_up_masks+6}}};
const track_section_t small_turn_right_bank_gentle_up={TRACK_BANK_RIGHT|TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,small_turn_right_bank_gentle_up_curve,SMALL_TURN_GENTLE_LENGTH,{{0,2,small_turn_right_gentle_up_masks},{0,2,small_turn_right_gentle_up_masks+2},{0,2,small_turn_right_gentle_up_masks+4},{0,2,small_turn_right_gentle_up_masks+6}}};
//TODO use correct splitting
const track_section_t medium_turn_left_bank_gentle_up={TRACK_BANK_LEFT|TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,medium_turn_left_bank_gentle_up_curve,MEDIUM_TURN_GENTLE_LENGTH,{{0,5,medium_turn_left_gentle_up_masks},{0,5,medium_turn_left_gentle_up_masks+5},{0,5,medium_turn_left_gentle_up_masks+10},{0,5,medium_turn_left_gentle_up_masks+15}}};
const track_section_t medium_turn_right_bank_gentle_up={TRACK_BANK_RIGHT|TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,medium_turn_right_bank_gentle_up_curve,MEDIUM_TURN_GENTLE_LENGTH,{{0,5,medium_turn_right_gentle_up_masks},{0,5,medium_turn_right_gentle_up_masks+5},{0,5,medium_turn_right_gentle_up_masks+10},{0,5,medium_turn_right_gentle_up_masks+15}}};



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
const track_section_t s_bend_left={0,s_bend_left_curve,S_BEND_LENGTH,{{0,4,s_bend_left_masks},{0,4,s_bend_left_masks+4},{0,0,NULL},{0,0,NULL}}};
//TODO use correct splitting
rect_t s_bend_right_rects[]={
//First angle
{INT32_MIN,INT32_MIN,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,INT32_MAX},
{0,INT32_MIN,64,0},{2,0,62,1},{4,1,60,2},{6,2,58,3},{8,3,56,4},{10,4,54,5},{12,5,52,6},{14,6,50,7},{16,7,48,8},{18,8,46,9},{20,9,44,10},{22,10,42,11},{24,11,40,12},{26,12,38,13},{28,13,36,14},{30,14,34,15},
{62,0,66,1},{60,1,68,2},{58,2,70,3},{56,3,72,4},{54,4,74,5},{52,5,76,6},{50,6,78,7},{48,7,80,8},{46,8,82,9},{44,9,84,10},{42,10,86,11},{40,11,88,12},{38,12,90,13},{36,13,92,14},{34,14,94,15},{32,15,96,INT32_MAX},
{64,INT32_MIN,INT32_MAX,0},{66,0,INT32_MAX,1},{68,1,INT32_MAX,2},{70,2,INT32_MAX,3},{72,3,INT32_MAX,4},{74,4,INT32_MAX,5},{76,5,INT32_MAX,6},{78,6,INT32_MAX,7},{80,7,INT32_MAX,8},{82,8,INT32_MAX,9},{84,9,INT32_MAX,10},{86,10,INT32_MAX,11},{88,11,INT32_MAX,12},{90,12,INT32_MAX,13},{92,13,INT32_MAX,14},{94,14,INT32_MAX,15},{96,15,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{-30,16,30,17},{-28,17,28,18},{-26,18,26,19},{-24,19,24,20},{-22,20,22,21},{-20,21,20,22},{-18,22,18,23},{-16,23,16,24},{-14,24,14,25},{-12,25,12,26},{-10,26,10,27},{-8,27,8,28},{-6,28,6,29},{-4,29,4,30},{-2,30,2,31},
{32,INT32_MIN,INT32_MAX,16},{30,16,INT32_MAX,17},{28,17,INT32_MAX,18},{26,18,INT32_MAX,19},{24,19,INT32_MAX,20},{22,20,INT32_MAX,21},{20,21,INT32_MAX,22},{18,22,INT32_MAX,23},{16,23,INT32_MAX,24},{14,24,INT32_MAX,25},{12,25,INT32_MAX,26},{10,26,INT32_MAX,27},{8,27,INT32_MAX,28},{6,28,INT32_MAX,29},{4,29,INT32_MAX,30},{2,30,INT32_MAX,31},{0,31,INT32_MAX,32},{2,32,INT32_MAX,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},
{INT32_MIN,16,-30,17},{INT32_MIN,17,-28,18},{INT32_MIN,18,-26,19},{INT32_MIN,19,-24,20},{INT32_MIN,20,-22,21},{INT32_MIN,21,-20,22},{INT32_MIN,22,-18,23},{INT32_MIN,23,-16,24},{INT32_MIN,24,-14,25},{INT32_MIN,25,-12,26},{INT32_MIN,26,-10,27},{INT32_MIN,27,-8,28},{INT32_MIN,28,-6,29},{INT32_MIN,29,-4,30},{INT32_MIN,30,-2,31},{INT32_MIN,31,0,32},{INT32_MIN,32,2,33},{INT32_MIN,33,4,34},{INT32_MIN,34,6,35},{INT32_MIN,35,8,36},{INT32_MIN,36,10,37},{INT32_MIN,37,12,38},{INT32_MIN,38,14,39},{INT32_MIN,39,16,40},{INT32_MIN,40,18,41},{INT32_MIN,41,20,42},{INT32_MIN,42,22,43},{INT32_MIN,43,24,44},{INT32_MIN,44,26,45},{INT32_MIN,45,28,46},{INT32_MIN,46,30,47},{INT32_MIN,47,32,48},{INT32_MIN,48,30,49},{INT32_MIN,49,28,50},{INT32_MIN,50,26,51},{INT32_MIN,51,24,52},{INT32_MIN,52,22,53},{INT32_MIN,53,20,54},{INT32_MIN,54,18,55},{INT32_MIN,55,16,56},{INT32_MIN,56,14,57},{INT32_MIN,57,12,58},{INT32_MIN,58,10,59},{INT32_MIN,59,8,60},{INT32_MIN,60,6,61},{INT32_MIN,61,4,62},{INT32_MIN,62,2,63},{INT32_MIN,63,0,INT32_MAX},
{60,33,INT32_MAX,34},{58,34,INT32_MAX,35},{56,35,INT32_MAX,36},{54,36,INT32_MAX,37},{52,37,INT32_MAX,38},{50,38,INT32_MAX,39},{48,39,INT32_MAX,40},{46,40,INT32_MAX,41},{44,41,INT32_MAX,42},{42,42,INT32_MAX,43},{40,43,INT32_MAX,44},{38,44,INT32_MAX,45},{36,45,INT32_MAX,46},{34,46,INT32_MAX,47},{32,47,INT32_MAX,48},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,INT32_MAX},
};
mask_t s_bend_right_masks[]={
{0,17,0,0,s_bend_right_rects},{0,16,-32,16,s_bend_right_rects+17},{0,16,-64,0,s_bend_right_rects+33},{0,17,-96,16,s_bend_right_rects+49},
{0,16,0,0,s_bend_right_rects+66},{0,32,-32,-16,s_bend_right_rects+82},{0,48,0,-32,s_bend_right_rects+114},{0,31,-32,-48,s_bend_right_rects+162},
};
const track_section_t s_bend_right={0,s_bend_right_curve,S_BEND_LENGTH,{{0,4,s_bend_right_masks},{0,4,s_bend_right_masks+4},{0,0,NULL},{0,0,NULL}}};
mask_t small_helix_left_up_masks[]={
	{TRACK_MASK_INTERSECT,13,0,0,small_turn_left_rects},{TRACK_MASK_DIFFERENCE,13,0,0,small_turn_left_rects},{0,13,-32,16,small_turn_left_rects+13},{0,26,0,32,small_turn_left_rects+26},
	{0,17,0,0,small_turn_left_rects+52},{0,16,-32,-16,small_turn_left_rects+69},{0,17,-64,0,small_turn_left_rects+85},
	{0,26,0,0,small_turn_left_rects+102},{0,13,32,-16,small_turn_left_rects+128},{TRACK_MASK_INTERSECT,13,0,-32,small_turn_left_rects+141},{TRACK_MASK_DIFFERENCE,13,0,-32,small_turn_left_rects+141},
	{0,4,0,0,small_turn_left_rects+154},{0,3,32,16,small_turn_left_rects+158},{0,4,64,0,small_turn_left_rects+161}

};
const track_section_t small_helix_left_up={TRACK_BANK_LEFT|TRACK_SUPPORT_BASE,small_helix_left_up_curve,SMALL_HELIX_LENGTH,{{VIEW_NEEDS_TRACK_MASK,4,small_helix_left_up_masks},{0,3,small_helix_left_up_masks+4},{VIEW_NEEDS_TRACK_MASK,4,small_helix_left_up_masks+7},{0,3,small_helix_left_up_masks+11}}};
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
	{TRACK_MASK_INTERSECT,10,0,0,small_turn_right_rects+75},{TRACK_MASK_DIFFERENCE,10,0,0,small_turn_right_rects+75},{0,10,32,16,small_turn_right_rects+85},{0,20,0,32,small_turn_right_rects+95}
};
const track_section_t small_helix_right_up={TRACK_BANK_RIGHT|TRACK_SUPPORT_BASE,small_helix_right_up_curve,SMALL_HELIX_LENGTH,{{0,3,small_helix_right_up_masks},{VIEW_NEEDS_TRACK_MASK,4,small_helix_right_up_masks+3},{0,3,small_helix_right_up_masks+7},{VIEW_NEEDS_TRACK_MASK,4,small_helix_right_up_masks+10}}};


mask_t medium_helix_left_up_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects},{0,49,-32,16,medium_turn_left_rects+33},{0,31,0,32,medium_turn_left_rects+82},{0,32,-32,48,medium_turn_left_rects+113},{0,32,0,64,medium_turn_left_rects+145},
{0,17,0,0,medium_turn_left_rects+177},{0,16,-32,-16,medium_turn_left_rects+194},{0,16,-64,0,medium_turn_left_rects+210},{0,16,-96,-16,medium_turn_left_rects+226},{0,17,-128,0,medium_turn_left_rects+242},
{0,33,0,0,medium_turn_left_rects+259},{0,32,32,-16,medium_turn_left_rects+292},{0,31,0,-32,medium_turn_left_rects+324},{0,49,32,-48,medium_turn_left_rects+355},{TRACK_MASK_INTERSECT,32,0,-64,medium_turn_left_rects+404},{TRACK_MASK_DIFFERENCE,32,0,-64,medium_turn_left_rects+404},
{0,33,0,0,medium_turn_left_rects+436},{0,31,32,16,medium_turn_left_rects+469},{0,32,64,0,medium_turn_left_rects+500},{0,31,96,16,medium_turn_left_rects+532},{0,33,128,0,medium_turn_left_rects+563}
};
const track_section_t medium_helix_left_up={TRACK_BANK_LEFT|TRACK_SUPPORT_BASE,medium_helix_left_up_curve,MEDIUM_HELIX_LENGTH,{{VIEW_NEEDS_TRACK_MASK,6,medium_helix_left_up_masks},{0,5,medium_helix_left_up_masks+6},{VIEW_NEEDS_TRACK_MASK,6,medium_helix_left_up_masks+11},{0,5,medium_helix_left_up_masks+17}}};
rect_t medium_turn_right_rects[]={
//First angle
{INT32_MIN,INT32_MIN,32,-16},{INT32_MIN,-16,30,-15},{INT32_MIN,-15,28,-14},{INT32_MIN,-14,26,-13},{INT32_MIN,-13,24,-12},{INT32_MIN,-12,22,-11},{INT32_MIN,-11,20,-10},{INT32_MIN,-10,18,-9},{INT32_MIN,-9,16,-8},{INT32_MIN,-8,14,-7},{INT32_MIN,-7,12,-6},{INT32_MIN,-6,10,-5},{INT32_MIN,-5,8,-4},{INT32_MIN,-4,6,-3},{INT32_MIN,-3,4,-2},{INT32_MIN,-2,2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,INT32_MAX},
{30,-16,34,-15},{28,-15,36,-14},{26,-14,38,-13},{24,-13,40,-12},{22,-12,42,-11},{20,-11,44,-10},{18,-10,46,-9},{16,-9,48,-8},{14,-8,50,-7},{12,-7,52,-6},{10,-6,54,-5},{8,-5,56,-4},{6,-4,58,-3},{4,-3,60,-2},{2,-2,62,-1},{0,-1,64,0},{2,0,62,1},{4,1,60,2},{6,2,58,3},{8,3,56,4},{10,4,54,5},{12,5,52,6},{14,6,50,7},{16,7,48,8},{18,8,46,9},{20,9,44,10},{22,10,42,11},{24,11,40,12},{26,12,38,13},{28,13,36,14},{30,14,34,15},
{32,INT32_MIN,96,-16},{34,-16,94,-15},{36,-15,92,-14},{38,-14,90,-13},{40,-13,88,-12},{42,-12,86,-11},{44,-11,84,-10},{46,-10,82,-9},{48,-9,80,-8},{50,-8,78,-7},{52,-7,76,-6},{54,-6,74,-5},{56,-5,72,-4},{58,-4,70,-3},{60,-3,68,-2},{62,-2,66,-1},{62,0,66,1},{60,1,68,2},{58,2,70,3},{56,3,72,4},{54,4,74,5},{52,5,76,6},{50,6,78,7},{48,7,80,8},{46,8,82,9},{44,9,84,10},{42,10,86,11},{40,11,88,12},{38,12,90,13},{36,13,92,14},{34,14,94,15},{32,15,96,INT32_MAX},
{94,-16,98,-15},{92,-15,100,-14},{90,-14,102,-13},{88,-13,104,-12},{86,-12,106,-11},{84,-11,108,-10},{82,-10,110,-9},{80,-9,112,-8},{78,-8,114,-7},{76,-7,116,-6},{74,-6,118,-5},{72,-5,120,-4},{70,-4,122,-3},{68,-3,124,-2},{66,-2,126,-1},{64,-1,128,0},{66,0,126,1},{68,1,124,2},{70,2,122,3},{72,3,120,4},{74,4,118,5},{76,5,116,6},{78,6,114,7},{80,7,112,8},{82,8,110,9},{84,9,108,10},{86,10,106,11},{88,11,104,12},{90,12,102,13},{92,13,100,14},{94,14,98,15},
{96,INT32_MIN,INT32_MAX,-16},{98,-16,INT32_MAX,-15},{100,-15,INT32_MAX,-14},{102,-14,INT32_MAX,-13},{104,-13,INT32_MAX,-12},{106,-12,INT32_MAX,-11},{108,-11,INT32_MAX,-10},{110,-10,INT32_MAX,-9},{112,-9,INT32_MAX,-8},{114,-8,INT32_MAX,-7},{116,-7,INT32_MAX,-6},{118,-6,INT32_MAX,-5},{120,-5,INT32_MAX,-4},{122,-4,INT32_MAX,-3},{124,-3,INT32_MAX,-2},{126,-2,INT32_MAX,-1},{128,-1,INT32_MAX,0},{126,0,INT32_MAX,1},{124,1,INT32_MAX,2},{122,2,INT32_MAX,3},{120,3,INT32_MAX,4},{118,4,INT32_MAX,5},{116,5,INT32_MAX,6},{114,6,INT32_MAX,7},{112,7,INT32_MAX,8},{110,8,INT32_MAX,9},{108,9,INT32_MAX,10},{106,10,INT32_MAX,11},{104,11,INT32_MAX,12},{102,12,INT32_MAX,13},{100,13,INT32_MAX,14},{98,14,INT32_MAX,15},{96,15,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,32,16},{INT32_MIN,16,30,17},{INT32_MIN,17,28,18},{INT32_MIN,18,26,19},{INT32_MIN,19,24,20},{INT32_MIN,20,22,21},{INT32_MIN,21,20,22},{INT32_MIN,22,18,23},{INT32_MIN,23,16,24},{INT32_MIN,24,14,25},{INT32_MIN,25,12,26},{INT32_MIN,26,10,27},{INT32_MIN,27,8,28},{INT32_MIN,28,6,29},{INT32_MIN,29,4,30},{INT32_MIN,30,2,31},{INT32_MIN,31,0,32},{INT32_MIN,32,-2,33},{INT32_MIN,33,-4,34},{INT32_MIN,34,-6,35},{INT32_MIN,35,-8,36},{INT32_MIN,36,-10,37},{INT32_MIN,37,-12,38},{INT32_MIN,38,-14,39},{INT32_MIN,39,-16,40},{INT32_MIN,40,-18,41},{INT32_MIN,41,-20,42},{INT32_MIN,42,-22,43},{INT32_MIN,43,-24,44},{INT32_MIN,44,-26,45},{INT32_MIN,45,-28,46},{INT32_MIN,46,-30,47},
{32,INT32_MIN,INT32_MAX,16},{30,16,INT32_MAX,17},{28,17,INT32_MAX,18},{26,18,INT32_MAX,19},{24,19,INT32_MAX,20},{22,20,INT32_MAX,21},{20,21,INT32_MAX,22},{18,22,INT32_MAX,23},{16,23,INT32_MAX,24},{14,24,INT32_MAX,25},{12,25,INT32_MAX,26},{10,26,INT32_MAX,27},{8,27,INT32_MAX,28},{6,28,INT32_MAX,29},{4,29,INT32_MAX,30},{2,30,INT32_MAX,31},{0,31,64,32},{2,32,62,33},{4,33,60,34},{6,34,58,35},{8,35,56,36},{10,36,54,37},{12,37,52,38},{14,38,50,39},{16,39,48,40},{18,40,46,41},{20,41,44,42},{22,42,42,43},{24,43,40,44},{26,44,38,45},{28,45,36,46},{30,46,34,47},
{-2,32,2,33},{-4,33,4,34},{-6,34,6,35},{-8,35,8,36},{-10,36,10,37},{-12,37,12,38},{-14,38,14,39},{-16,39,16,40},{-18,40,18,41},{-20,41,20,42},{-22,42,22,43},{-24,43,24,44},{-26,44,26,45},{-28,45,28,46},{-30,46,30,47},{-32,47,32,48},{-30,48,30,49},{-28,49,28,50},{-26,50,26,51},{-24,51,24,52},{-22,52,22,53},{-20,53,20,54},{-18,54,18,55},{-16,55,16,56},{-14,56,14,57},{-12,57,12,58},{-10,58,10,59},{-8,59,8,60},{-6,60,6,61},{-4,61,4,62},{-2,62,2,63},
{64,31,INT32_MAX,32},{62,32,INT32_MAX,33},{60,33,INT32_MAX,34},{58,34,INT32_MAX,35},{56,35,INT32_MAX,36},{54,36,INT32_MAX,37},{52,37,INT32_MAX,38},{50,38,INT32_MAX,39},{48,39,INT32_MAX,40},{46,40,INT32_MAX,41},{44,41,INT32_MAX,42},{42,42,INT32_MAX,43},{40,43,INT32_MAX,44},{38,44,INT32_MAX,45},{36,45,INT32_MAX,46},{34,46,INT32_MAX,47},{32,47,INT32_MAX,48},{30,48,INT32_MAX,49},{28,49,INT32_MAX,50},{26,50,INT32_MAX,51},{24,51,INT32_MAX,52},{22,52,INT32_MAX,53},{20,53,INT32_MAX,54},{18,54,INT32_MAX,55},{16,55,INT32_MAX,56},{14,56,INT32_MAX,57},{12,57,INT32_MAX,58},{10,58,INT32_MAX,59},{8,59,INT32_MAX,60},{6,60,INT32_MAX,61},{4,61,INT32_MAX,62},{2,62,INT32_MAX,63},{0,63,INT32_MAX,64},{2,64,INT32_MAX,65},{4,65,INT32_MAX,66},{6,66,INT32_MAX,67},{8,67,INT32_MAX,68},{10,68,INT32_MAX,69},{12,69,INT32_MAX,70},{14,70,INT32_MAX,71},{16,71,INT32_MAX,72},{18,72,INT32_MAX,73},{20,73,INT32_MAX,74},{22,74,INT32_MAX,75},{24,75,INT32_MAX,76},{26,76,INT32_MAX,77},{28,77,INT32_MAX,78},{30,78,INT32_MAX,79},{32,79,INT32_MAX,INT32_MAX},
{INT32_MIN,47,-32,48},{INT32_MIN,48,-30,49},{INT32_MIN,49,-28,50},{INT32_MIN,50,-26,51},{INT32_MIN,51,-24,52},{INT32_MIN,52,-22,53},{INT32_MIN,53,-20,54},{INT32_MIN,54,-18,55},{INT32_MIN,55,-16,56},{INT32_MIN,56,-14,57},{INT32_MIN,57,-12,58},{INT32_MIN,58,-10,59},{INT32_MIN,59,-8,60},{INT32_MIN,60,-6,61},{INT32_MIN,61,-4,62},{INT32_MIN,62,-2,63},{INT32_MIN,63,0,64},{INT32_MIN,64,2,65},{INT32_MIN,65,4,66},{INT32_MIN,66,6,67},{INT32_MIN,67,8,68},{INT32_MIN,68,10,69},{INT32_MIN,69,12,70},{INT32_MIN,70,14,71},{INT32_MIN,71,16,72},{INT32_MIN,72,18,73},{INT32_MIN,73,20,74},{INT32_MIN,74,22,75},{INT32_MIN,75,24,76},{INT32_MIN,76,26,77},{INT32_MIN,77,28,78},{INT32_MIN,78,30,79},{INT32_MIN,79,32,INT32_MAX},
//Third angle
{-32,INT32_MIN,INT32_MAX,16},{-30,16,INT32_MAX,17},{-28,17,INT32_MAX,18},{-26,18,INT32_MAX,19},{-24,19,INT32_MAX,20},{-22,20,INT32_MAX,21},{-20,21,INT32_MAX,22},{-18,22,INT32_MAX,23},{-16,23,INT32_MAX,24},{-14,24,INT32_MAX,25},{-12,25,INT32_MAX,26},{-10,26,INT32_MAX,27},{-8,27,INT32_MAX,28},{-6,28,INT32_MAX,29},{-4,29,INT32_MAX,30},{-2,30,INT32_MAX,31},{0,31,INT32_MAX,INT32_MAX},
{-34,16,-30,17},{-36,17,-28,18},{-38,18,-26,19},{-40,19,-24,20},{-42,20,-22,21},{-44,21,-20,22},{-46,22,-18,23},{-48,23,-16,24},{-50,24,-14,25},{-52,25,-12,26},{-54,26,-10,27},{-56,27,-8,28},{-58,28,-6,29},{-60,29,-4,30},{-62,30,-2,31},{-64,31,0,INT32_MAX},
{-96,INT32_MIN,-32,16},{-94,16,-34,17},{-92,17,-36,18},{-90,18,-38,19},{-88,19,-40,20},{-86,20,-42,21},{-84,21,-44,22},{-82,22,-46,23},{-80,23,-48,24},{-78,24,-50,25},{-76,25,-52,26},{-74,26,-54,27},{-72,27,-56,28},{-70,28,-58,29},{-68,29,-60,30},{-66,30,-62,31},
{-98,16,-94,17},{-100,17,-92,18},{-102,18,-90,19},{-104,19,-88,20},{-106,20,-86,21},{-108,21,-84,22},{-110,22,-82,23},{-112,23,-80,24},{-114,24,-78,25},{-116,25,-76,26},{-118,26,-74,27},{-120,27,-72,28},{-122,28,-70,29},{-124,29,-68,30},{-126,30,-66,31},{-128,31,-64,INT32_MAX},
{INT32_MIN,INT32_MIN,-96,16},{INT32_MIN,16,-98,17},{INT32_MIN,17,-100,18},{INT32_MIN,18,-102,19},{INT32_MIN,19,-104,20},{INT32_MIN,20,-106,21},{INT32_MIN,21,-108,22},{INT32_MIN,22,-110,23},{INT32_MIN,23,-112,24},{INT32_MIN,24,-114,25},{INT32_MIN,25,-116,26},{INT32_MIN,26,-118,27},{INT32_MIN,27,-120,28},{INT32_MIN,28,-122,29},{INT32_MIN,29,-124,30},{INT32_MIN,30,-126,31},{INT32_MIN,31,-128,INT32_MAX},
//Fourth angle
{30,-16,INT32_MAX,-15},{28,-15,INT32_MAX,-14},{26,-14,INT32_MAX,-13},{24,-13,INT32_MAX,-12},{22,-12,INT32_MAX,-11},{20,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{16,-9,INT32_MAX,-8},{14,-8,INT32_MAX,-7},{12,-7,INT32_MAX,-6},{10,-6,INT32_MAX,-5},{8,-5,INT32_MAX,-4},{6,-4,INT32_MAX,-3},{4,-3,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{-32,15,INT32_MAX,INT32_MAX},
{INT32_MIN,-33,-64,-32},{INT32_MIN,-32,-62,-31},{INT32_MIN,-31,-60,-30},{INT32_MIN,-30,-58,-29},{INT32_MIN,-29,-56,-28},{INT32_MIN,-28,-54,-27},{INT32_MIN,-27,-52,-26},{INT32_MIN,-26,-50,-25},{INT32_MIN,-25,-48,-24},{INT32_MIN,-24,-46,-23},{INT32_MIN,-23,-44,-22},{INT32_MIN,-22,-42,-21},{INT32_MIN,-21,-40,-20},{INT32_MIN,-20,-38,-19},{INT32_MIN,-19,-36,-18},{INT32_MIN,-18,-34,-17},{INT32_MIN,-17,-32,-16},{INT32_MIN,-16,-30,-15},{INT32_MIN,-15,-28,-14},{INT32_MIN,-14,-26,-13},{INT32_MIN,-13,-24,-12},{INT32_MIN,-12,-22,-11},{INT32_MIN,-11,-20,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-16,-8},{INT32_MIN,-8,-14,-7},{INT32_MIN,-7,-12,-6},{INT32_MIN,-6,-10,-5},{INT32_MIN,-5,-8,-4},{INT32_MIN,-4,-6,-3},{INT32_MIN,-3,-4,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,-2,1},{INT32_MIN,1,-4,2},{INT32_MIN,2,-6,3},{INT32_MIN,3,-8,4},{INT32_MIN,4,-10,5},{INT32_MIN,5,-12,6},{INT32_MIN,6,-14,7},{INT32_MIN,7,-16,8},{INT32_MIN,8,-18,9},{INT32_MIN,9,-20,10},{INT32_MIN,10,-22,11},{INT32_MIN,11,-24,12},{INT32_MIN,12,-26,13},{INT32_MIN,13,-28,14},{INT32_MIN,14,-30,15},{INT32_MIN,15,-32,INT32_MAX},
{-2,-32,2,-31},{-4,-31,4,-30},{-6,-30,6,-29},{-8,-29,8,-28},{-10,-28,10,-27},{-12,-27,12,-26},{-14,-26,14,-25},{-16,-25,16,-24},{-18,-24,18,-23},{-20,-23,20,-22},{-22,-22,22,-21},{-24,-21,24,-20},{-26,-20,26,-19},{-28,-19,28,-18},{-30,-18,30,-17},{-32,-17,32,-16},{-30,-16,30,-15},{-28,-15,28,-14},{-26,-14,26,-13},{-24,-13,24,-12},{-22,-12,22,-11},{-20,-11,20,-10},{-18,-10,18,-9},{-16,-9,16,-8},{-14,-8,14,-7},{-12,-7,12,-6},{-10,-6,10,-5},{-8,-5,8,-4},{-6,-4,6,-3},{-4,-3,4,-2},{-2,-2,2,-1},
{INT32_MIN,INT32_MIN,-32,-48},{INT32_MIN,-48,-30,-47},{INT32_MIN,-47,-28,-46},{INT32_MIN,-46,-26,-45},{INT32_MIN,-45,-24,-44},{INT32_MIN,-44,-22,-43},{INT32_MIN,-43,-20,-42},{INT32_MIN,-42,-18,-41},{INT32_MIN,-41,-16,-40},{INT32_MIN,-40,-14,-39},{INT32_MIN,-39,-12,-38},{INT32_MIN,-38,-10,-37},{INT32_MIN,-37,-8,-36},{INT32_MIN,-36,-6,-35},{INT32_MIN,-35,-4,-34},{INT32_MIN,-34,-2,-33},{-64,-33,0,-32},{-62,-32,-2,-31},{-60,-31,-4,-30},{-58,-30,-6,-29},{-56,-29,-8,-28},{-54,-28,-10,-27},{-52,-27,-12,-26},{-50,-26,-14,-25},{-48,-25,-16,-24},{-46,-24,-18,-23},{-44,-23,-20,-22},{-42,-22,-22,-21},{-40,-21,-24,-20},{-38,-20,-26,-19},{-36,-19,-28,-18},{-34,-18,-30,-17},
{-32,INT32_MIN,INT32_MAX,-48},{-30,-48,INT32_MAX,-47},{-28,-47,INT32_MAX,-46},{-26,-46,INT32_MAX,-45},{-24,-45,INT32_MAX,-44},{-22,-44,INT32_MAX,-43},{-20,-43,INT32_MAX,-42},{-18,-42,INT32_MAX,-41},{-16,-41,INT32_MAX,-40},{-14,-40,INT32_MAX,-39},{-12,-39,INT32_MAX,-38},{-10,-38,INT32_MAX,-37},{-8,-37,INT32_MAX,-36},{-6,-36,INT32_MAX,-35},{-4,-35,INT32_MAX,-34},{-2,-34,INT32_MAX,-33},{0,-33,INT32_MAX,-32},{2,-32,INT32_MAX,-31},{4,-31,INT32_MAX,-30},{6,-30,INT32_MAX,-29},{8,-29,INT32_MAX,-28},{10,-28,INT32_MAX,-27},{12,-27,INT32_MAX,-26},{14,-26,INT32_MAX,-25},{16,-25,INT32_MAX,-24},{18,-24,INT32_MAX,-23},{20,-23,INT32_MAX,-22},{22,-22,INT32_MAX,-21},{24,-21,INT32_MAX,-20},{26,-20,INT32_MAX,-19},{28,-19,INT32_MAX,-18},{30,-18,INT32_MAX,-17},{32,-17,INT32_MAX,-16},
};
mask_t medium_helix_right_up_masks[]={
{0,33,0,0,medium_turn_right_rects},{0,31,-32,16,medium_turn_right_rects+33},{0,32,-64,0,medium_turn_right_rects+64},{0,31,-96,16,medium_turn_right_rects+96},{0,33,-128,0,medium_turn_right_rects+127},
{0,32,0,0,medium_turn_right_rects+160},{0,32,-32,-16,medium_turn_right_rects+192},{0,31,0,-32,medium_turn_right_rects+224},{0,49,-32,-48,medium_turn_right_rects+255},{TRACK_MASK_INTERSECT,33,0,-64,medium_turn_right_rects+304},{TRACK_MASK_DIFFERENCE,33,0,-64,medium_turn_right_rects+304},
{0,17,0,0,medium_turn_right_rects+337},{0,16,32,-16,medium_turn_right_rects+354},{0,16,64,0,medium_turn_right_rects+370},{0,16,96,-16,medium_turn_right_rects+386},{0,17,128,0,medium_turn_right_rects+402},
{TRACK_MASK_INTERSECT,32,0,0,medium_turn_right_rects+419},{TRACK_MASK_DIFFERENCE,32,0,0,medium_turn_right_rects+419},{0,49,32,16,medium_turn_right_rects+451},{0,31,0,32,medium_turn_right_rects+500},{0,32,32,48,medium_turn_right_rects+531},{0,33,0,64,medium_turn_right_rects+563}
};
const track_section_t medium_helix_right_up={TRACK_BANK_RIGHT|TRACK_SUPPORT_BASE,medium_helix_right_up_curve,MEDIUM_HELIX_LENGTH,{{0,5,medium_helix_right_up_masks},{VIEW_NEEDS_TRACK_MASK,6,medium_helix_right_up_masks+5},{0,5,medium_helix_right_up_masks+11},{VIEW_NEEDS_TRACK_MASK,6,medium_helix_right_up_masks+16}}};

rect_t barrel_roll_left_rects[]={
//First angle
{INT32_MIN,INT32_MIN,-3,-2},{INT32_MIN,-2,-2,-1},{INT32_MIN,-1,0,0},{INT32_MIN,0,2,1},{INT32_MIN,1,4,2},{INT32_MIN,2,6,3},{INT32_MIN,3,8,4},{INT32_MIN,4,10,5},{INT32_MIN,5,12,6},{INT32_MIN,6,14,7},{INT32_MIN,7,16,8},{INT32_MIN,8,18,9},{INT32_MIN,9,20,10},{INT32_MIN,10,22,11},{INT32_MIN,11,24,12},{INT32_MIN,12,26,13},{INT32_MIN,13,28,14},{INT32_MIN,14,30,15},{INT32_MIN,15,32,16},{INT32_MIN,16,34,17},{INT32_MIN,17,36,18},{INT32_MIN,18,38,19},{INT32_MIN,19,40,20},{INT32_MIN,20,42,21},{INT32_MIN,21,44,22},{INT32_MIN,22,46,23},{INT32_MIN,23,48,24},{INT32_MIN,24,50,25},{INT32_MIN,25,52,26},{INT32_MIN,26,54,27},{INT32_MIN,27,56,28},{INT32_MIN,28,58,29},{INT32_MIN,29,60,30},{INT32_MIN,30,61,INT32_MAX},
{-3,-38,52,-36},{-3,-36,53,-34},{-3,-34,54,-32},{-3,-32,55,-30},{-3,-30,56,-28},{-3,-28,57,-26},{-3,-26,58,-24},{-3,-24,59,-22},{-3,-22,60,-20},{-3,-20,61,-2},{-2,-2,61,-1},{0,-1,61,0},{2,0,61,1},{4,1,61,2},{6,2,61,3},{8,3,61,4},{10,4,61,5},{12,5,61,6},{14,6,61,7},{16,7,61,8},{18,8,61,9},{20,9,61,10},{22,10,61,11},{24,11,61,12},{26,12,61,13},{28,13,61,14},{30,14,61,15},{32,15,61,16},{34,16,61,17},{36,17,61,18},{38,18,61,19},{40,19,61,20},{42,20,61,21},{44,21,61,22},{46,22,61,23},{48,23,61,24},{50,24,61,25},{52,25,61,26},{54,26,61,27},{56,27,61,28},{58,28,61,29},{60,29,61,30},
{-3,INT32_MIN,INT32_MAX,-38},{52,-38,INT32_MAX,-36},{53,-36,INT32_MAX,-34},{54,-34,INT32_MAX,-32},{55,-32,INT32_MAX,-30},{56,-30,INT32_MAX,-28},{57,-28,INT32_MAX,-26},{58,-26,INT32_MAX,-24},{59,-24,INT32_MAX,-22},{60,-22,INT32_MAX,-20},{61,-20,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,4,INT32_MAX},
{4,INT32_MIN,32,INT32_MAX},
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
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_left_rects},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_left_rects},{TRACK_MASK_INTERSECT,42,-32,16,barrel_roll_left_rects+34},{TRACK_MASK_DIFFERENCE,42,-32,16,barrel_roll_left_rects+34},{TRACK_MASK_INTERSECT,11,-64,32,barrel_roll_left_rects+76},{TRACK_MASK_DIFFERENCE,11,-64,32,barrel_roll_left_rects+76},
	{TRACK_MASK_INTERSECT,1,0,0,barrel_roll_left_rects+87},{TRACK_MASK_DIFFERENCE,1,0,0,barrel_roll_left_rects+87},{TRACK_MASK_INTERSECT,1,-32,-16,barrel_roll_left_rects+88},{TRACK_MASK_DIFFERENCE,1,-32,-16,barrel_roll_left_rects+88},{TRACK_MASK_INTERSECT,1,-64,-32,barrel_roll_left_rects+89},{TRACK_MASK_DIFFERENCE,1,-64,-32,barrel_roll_left_rects+89},
	{TRACK_MASK_INTERSECT,18,0,0,barrel_roll_left_rects+90},{TRACK_MASK_DIFFERENCE,18,0,0,barrel_roll_left_rects+90},{TRACK_MASK_INTERSECT,28,32,-16,barrel_roll_left_rects+108},{TRACK_MASK_DIFFERENCE,28,32,-16,barrel_roll_left_rects+108},{TRACK_MASK_INTERSECT,14,64,-32,barrel_roll_left_rects+136},{TRACK_MASK_DIFFERENCE,14,64,-32,barrel_roll_left_rects+136},
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_left_rects+150},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_left_rects+150},{TRACK_MASK_INTERSECT,48,32,16,barrel_roll_left_rects+184},{TRACK_MASK_DIFFERENCE,48,32,16,barrel_roll_left_rects+184},{TRACK_MASK_INTERSECT,20,64,32,barrel_roll_left_rects+232},{TRACK_MASK_DIFFERENCE,20,64,32,barrel_roll_left_rects+232}
};
const track_section_t barrel_roll_left={TRACK_NO_SUPPORTS|TRACK_SPECIAL_BARREL_ROLL_LEFT|TRACK_OFFSET_SPRITE_MASK,barrel_roll_left_curve,BARREL_ROLL_LENGTH,{{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_left_masks},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_left_masks+6},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_left_masks+12},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_left_masks+18}}};


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
{-4,INT32_MIN,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,-4,INT32_MAX},
{INT32_MIN,INT32_MIN,-32,INT32_MAX},
//Fourth angle
{3,INT32_MIN,INT32_MAX,-2},{2,-2,INT32_MAX,-1},{0,-1,INT32_MAX,0},{-2,0,INT32_MAX,1},{-4,1,INT32_MAX,2},{-6,2,INT32_MAX,3},{-8,3,INT32_MAX,4},{-10,4,INT32_MAX,5},{-12,5,INT32_MAX,6},{-14,6,INT32_MAX,7},{-16,7,INT32_MAX,8},{-18,8,INT32_MAX,9},{-20,9,INT32_MAX,10},{-22,10,INT32_MAX,11},{-24,11,INT32_MAX,12},{-26,12,INT32_MAX,13},{-28,13,INT32_MAX,14},{-30,14,INT32_MAX,15},{-32,15,INT32_MAX,16},{-34,16,INT32_MAX,17},{-36,17,INT32_MAX,18},{-38,18,INT32_MAX,19},{-40,19,INT32_MAX,20},{-42,20,INT32_MAX,21},{-44,21,INT32_MAX,22},{-46,22,INT32_MAX,23},{-48,23,INT32_MAX,24},{-50,24,INT32_MAX,25},{-52,25,INT32_MAX,26},{-54,26,INT32_MAX,27},{-56,27,INT32_MAX,28},{-58,28,INT32_MAX,29},{-60,29,INT32_MAX,30},{-61,30,INT32_MAX,INT32_MAX},
{-52,-38,3,-36},{-53,-36,3,-34},{-54,-34,3,-32},{-55,-32,3,-30},{-56,-30,3,-28},{-57,-28,3,-26},{-58,-26,3,-24},{-59,-24,3,-22},{-60,-22,3,-20},{-61,-20,3,-2},{-61,-2,2,-1},{-61,-1,0,0},{-61,0,-2,1},{-61,1,-4,2},{-61,2,-6,3},{-61,3,-8,4},{-61,4,-10,5},{-61,5,-12,6},{-61,6,-14,7},{-61,7,-16,8},{-61,8,-18,9},{-61,9,-20,10},{-61,10,-22,11},{-61,11,-24,12},{-61,12,-26,13},{-61,13,-28,14},{-61,14,-30,15},{-61,15,-32,16},{-61,16,-34,17},{-61,17,-36,18},{-61,18,-38,19},{-61,19,-40,20},{-61,20,-42,21},{-61,21,-44,22},{-61,22,-46,23},{-61,23,-48,24},{-61,24,-50,25},{-61,25,-52,26},{-61,26,-54,27},{-61,27,-56,28},{-61,28,-58,29},{-61,29,-60,30},
{INT32_MIN,INT32_MIN,3,-38},{INT32_MIN,-38,-52,-36},{INT32_MIN,-36,-53,-34},{INT32_MIN,-34,-54,-32},{INT32_MIN,-32,-55,-30},{INT32_MIN,-30,-56,-28},{INT32_MIN,-28,-57,-26},{INT32_MIN,-26,-58,-24},{INT32_MIN,-24,-59,-22},{INT32_MIN,-22,-60,-20},{INT32_MIN,-20,-61,INT32_MAX},
};
mask_t barrel_roll_right_masks[]={
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_right_rects},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_right_rects},{TRACK_MASK_INTERSECT,48,-32,16,barrel_roll_right_rects+34},{TRACK_MASK_DIFFERENCE,48,-32,16,barrel_roll_right_rects+34},{TRACK_MASK_INTERSECT,20,-64,32,barrel_roll_right_rects+82},{TRACK_MASK_DIFFERENCE,20,-64,32,barrel_roll_right_rects+82},
	{TRACK_MASK_INTERSECT,18,0,0,barrel_roll_right_rects+102},{TRACK_MASK_DIFFERENCE,18,0,0,barrel_roll_right_rects+102},{TRACK_MASK_INTERSECT,28,-32,-16,barrel_roll_right_rects+120},{TRACK_MASK_DIFFERENCE,28,-32,-16,barrel_roll_right_rects+120},{TRACK_MASK_INTERSECT,14,-64,-32,barrel_roll_right_rects+148},{TRACK_MASK_DIFFERENCE,14,-64,-32,barrel_roll_right_rects+148},
	{TRACK_MASK_INTERSECT,1,0,0,barrel_roll_right_rects+162},{TRACK_MASK_DIFFERENCE,1,0,0,barrel_roll_right_rects+162},{TRACK_MASK_INTERSECT,1,32,-16,barrel_roll_right_rects+163},{TRACK_MASK_DIFFERENCE,1,32,-16,barrel_roll_right_rects+163},{TRACK_MASK_INTERSECT,1,64,-32,barrel_roll_right_rects+164},{TRACK_MASK_DIFFERENCE,1,64,-32,barrel_roll_right_rects+164},
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_right_rects+165},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_right_rects+165},{TRACK_MASK_INTERSECT,42,32,16,barrel_roll_right_rects+199},{TRACK_MASK_DIFFERENCE,42,32,16,barrel_roll_right_rects+199},{TRACK_MASK_INTERSECT,11,64,32,barrel_roll_right_rects+241},{TRACK_MASK_DIFFERENCE,11,64,32,barrel_roll_right_rects+241}
};
const track_section_t barrel_roll_right={TRACK_NO_SUPPORTS|TRACK_SPECIAL_BARREL_ROLL_RIGHT|TRACK_OFFSET_SPRITE_MASK,barrel_roll_right_curve,BARREL_ROLL_LENGTH,{{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_right_masks},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_right_masks+6},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_right_masks+12},{VIEW_NEEDS_TRACK_MASK,6,barrel_roll_right_masks+18}}};



//TODO needs new rects
mask_t inline_twist_left_masks[]={
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_left_rects},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_left_rects},{TRACK_MASK_INTERSECT,42,-32,16,barrel_roll_left_rects+34},{TRACK_MASK_DIFFERENCE,42,-32,16,barrel_roll_left_rects+34},{TRACK_MASK_INTERSECT,11,-64,48,barrel_roll_left_rects+76},{TRACK_MASK_DIFFERENCE,11,-64,48,barrel_roll_left_rects+76},
	{TRACK_MASK_INTERSECT,1,0,0,barrel_roll_left_rects+87},{TRACK_MASK_DIFFERENCE,1,0,0,barrel_roll_left_rects+87},{TRACK_MASK_INTERSECT,1,-32,-16,barrel_roll_left_rects+88},{TRACK_MASK_DIFFERENCE,1,-32,-16,barrel_roll_left_rects+88},{TRACK_MASK_INTERSECT,1,-64,-16,barrel_roll_left_rects+89},{TRACK_MASK_DIFFERENCE,1,-64,-16,barrel_roll_left_rects+89},
	{TRACK_MASK_INTERSECT,18,0,0,barrel_roll_left_rects+90},{TRACK_MASK_DIFFERENCE,18,0,0,barrel_roll_left_rects+90},{TRACK_MASK_INTERSECT,28,32,-16,barrel_roll_left_rects+108},{TRACK_MASK_DIFFERENCE,28,32,-16,barrel_roll_left_rects+108},{TRACK_MASK_INTERSECT,14,64,-16,barrel_roll_left_rects+136},{TRACK_MASK_DIFFERENCE,14,64,-16,barrel_roll_left_rects+136},
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_left_rects+150},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_left_rects+150},{TRACK_MASK_INTERSECT,48,32,16,barrel_roll_left_rects+184},{TRACK_MASK_DIFFERENCE,48,32,16,barrel_roll_left_rects+184},{TRACK_MASK_INTERSECT,20,64,48,barrel_roll_left_rects+232},{TRACK_MASK_DIFFERENCE,20,64,48,barrel_roll_left_rects+232}
};
const track_section_t inline_twist_left={TRACK_NO_SUPPORTS|TRACK_OFFSET_SPRITE_MASK,inline_twist_left_curve,BARREL_ROLL_LENGTH,{{VIEW_NEEDS_TRACK_MASK,6,inline_twist_left_masks},{VIEW_NEEDS_TRACK_MASK,6,inline_twist_left_masks+6},{VIEW_NEEDS_TRACK_MASK,6,inline_twist_left_masks+12},{VIEW_NEEDS_TRACK_MASK,6,inline_twist_left_masks+18}}};
mask_t inline_twist_right_masks[]={
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_right_rects},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_right_rects},{TRACK_MASK_INTERSECT,48,-32,16,barrel_roll_right_rects+34},{TRACK_MASK_DIFFERENCE,48,-32,16,barrel_roll_right_rects+34},{TRACK_MASK_INTERSECT,20,-64,48,barrel_roll_right_rects+82},{TRACK_MASK_DIFFERENCE,20,-64,48,barrel_roll_right_rects+82},
	{TRACK_MASK_INTERSECT,18,0,0,barrel_roll_right_rects+102},{TRACK_MASK_DIFFERENCE,18,0,0,barrel_roll_right_rects+102},{TRACK_MASK_INTERSECT,28,-32,-16,barrel_roll_right_rects+120},{TRACK_MASK_DIFFERENCE,28,-32,-16,barrel_roll_right_rects+120},{TRACK_MASK_INTERSECT,14,-64,-16,barrel_roll_right_rects+148},{TRACK_MASK_DIFFERENCE,14,-64,-16,barrel_roll_right_rects+148},
	{TRACK_MASK_INTERSECT,1,0,0,barrel_roll_right_rects+162},{TRACK_MASK_DIFFERENCE,1,0,0,barrel_roll_right_rects+162},{TRACK_MASK_INTERSECT,1,32,-16,barrel_roll_right_rects+163},{TRACK_MASK_DIFFERENCE,1,32,-16,barrel_roll_right_rects+163},{TRACK_MASK_INTERSECT,1,64,-16,barrel_roll_right_rects+164},{TRACK_MASK_DIFFERENCE,1,64,-16,barrel_roll_right_rects+164},
	{TRACK_MASK_INTERSECT,34,0,0,barrel_roll_right_rects+165},{TRACK_MASK_DIFFERENCE,34,0,0,barrel_roll_right_rects+165},{TRACK_MASK_INTERSECT,42,32,16,barrel_roll_right_rects+199},{TRACK_MASK_DIFFERENCE,42,32,16,barrel_roll_right_rects+199},{TRACK_MASK_INTERSECT,11,64,48,barrel_roll_right_rects+241},{TRACK_MASK_DIFFERENCE,11,64,48,barrel_roll_right_rects+241}
};
const track_section_t inline_twist_right={TRACK_NO_SUPPORTS|TRACK_OFFSET_SPRITE_MASK,inline_twist_right_curve,BARREL_ROLL_LENGTH,{{VIEW_NEEDS_TRACK_MASK,6,inline_twist_right_masks},{VIEW_NEEDS_TRACK_MASK,6,inline_twist_right_masks+6},{VIEW_NEEDS_TRACK_MASK,6,inline_twist_right_masks+12},{VIEW_NEEDS_TRACK_MASK,6,inline_twist_right_masks+18}}};





rect_t half_loop_rects[]={
//First angle
{INT32_MIN,-29,-1,-28},{INT32_MIN,-28,0,-27},{INT32_MIN,-27,1,-26},{INT32_MIN,-26,2,-25},{INT32_MIN,-25,3,-24},{INT32_MIN,-24,4,-23},{INT32_MIN,-23,5,-22},{INT32_MIN,-22,6,-21},{INT32_MIN,-21,7,-20},{INT32_MIN,-20,8,-19},{INT32_MIN,-19,9,-18},{INT32_MIN,-18,10,-17},{INT32_MIN,-17,11,-16},{INT32_MIN,-16,12,-15},{INT32_MIN,-15,13,-14},{INT32_MIN,-14,14,-13},{INT32_MIN,-13,15,-12},{INT32_MIN,-12,16,-11},{INT32_MIN,-11,17,-10},{INT32_MIN,-10,18,-9},{INT32_MIN,-9,19,-8},{INT32_MIN,-8,20,-7},{INT32_MIN,-7,21,-6},{INT32_MIN,-6,22,INT32_MAX},
{INT32_MIN,-75,33,-74},{INT32_MIN,-74,35,-73},{INT32_MIN,-73,37,-72},{INT32_MIN,-72,40,-71},{INT32_MIN,-71,42,-70},{INT32_MIN,-70,45,-69},{INT32_MIN,-69,47,-68},{INT32_MIN,-68,50,-67},{INT32_MIN,-67,52,-66},{INT32_MIN,-66,55,-65},{INT32_MIN,-65,57,-64},{INT32_MIN,-64,INT32_MAX,-29},{-1,-29,INT32_MAX,-28},{0,-28,INT32_MAX,-27},{1,-27,INT32_MAX,-26},{2,-26,INT32_MAX,-25},{3,-25,INT32_MAX,-24},{4,-24,INT32_MAX,-23},{5,-23,INT32_MAX,-22},{6,-22,INT32_MAX,-21},{7,-21,INT32_MAX,-20},{8,-20,INT32_MAX,-19},{9,-19,INT32_MAX,-18},{10,-18,INT32_MAX,-17},{11,-17,INT32_MAX,-16},{12,-16,INT32_MAX,-15},{13,-15,INT32_MAX,-14},{14,-14,INT32_MAX,-13},{15,-13,INT32_MAX,-12},{16,-12,INT32_MAX,-11},{17,-11,INT32_MAX,-10},{18,-10,INT32_MAX,-9},{19,-9,INT32_MAX,-8},{20,-8,INT32_MAX,-7},{21,-7,INT32_MAX,-6},{22,-6,INT32_MAX,INT32_MAX},
{48,INT32_MIN,INT32_MAX,-138},{33,-138,INT32_MAX,-74},{35,-74,INT32_MAX,-73},{37,-73,INT32_MAX,-72},{40,-72,INT32_MAX,-71},{42,-71,INT32_MAX,-70},{45,-70,INT32_MAX,-69},{47,-69,INT32_MAX,-68},{50,-68,INT32_MAX,-67},{52,-67,INT32_MAX,-66},{55,-66,INT32_MAX,-65},{57,-65,INT32_MAX,-64},
{INT32_MIN,INT32_MIN,48,-138},{INT32_MIN,-138,33,-75},
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
{-32,-76,INT32_MAX,-75},{-34,-75,INT32_MAX,-74},{-36,-74,INT32_MAX,-73},{-39,-73,INT32_MAX,-72},{-41,-72,INT32_MAX,-71},{-44,-71,INT32_MAX,-70},{-46,-70,INT32_MAX,-69},{-49,-69,INT32_MAX,-68},{-51,-68,INT32_MAX,-67},{-54,-67,INT32_MAX,-66},{-56,-66,INT32_MAX,-65},{INT32_MIN,-65,INT32_MAX,-30},{INT32_MIN,-30,2,-29},{INT32_MIN,-29,1,-28},{INT32_MIN,-28,0,-27},{INT32_MIN,-27,-1,-26},{INT32_MIN,-26,-2,-25},{INT32_MIN,-25,-3,-24},{INT32_MIN,-24,-4,-23},{INT32_MIN,-23,-5,-22},{INT32_MIN,-22,-6,-21},{INT32_MIN,-21,-7,-20},{INT32_MIN,-20,-8,-19},{INT32_MIN,-19,-9,-18},{INT32_MIN,-18,-10,-17},{INT32_MIN,-17,-11,-16},{INT32_MIN,-16,-12,-15},{INT32_MIN,-15,-13,-14},{INT32_MIN,-14,-14,-13},{INT32_MIN,-13,-15,-12},{INT32_MIN,-12,-16,-11},{INT32_MIN,-11,-17,-10},{INT32_MIN,-10,-18,-9},{INT32_MIN,-9,-19,-8},{INT32_MIN,-8,-20,-7},{INT32_MIN,-7,-21,INT32_MAX},
{INT32_MIN,INT32_MIN,-47,-139},{INT32_MIN,-139,-32,-75},{INT32_MIN,-75,-34,-74},{INT32_MIN,-74,-36,-73},{INT32_MIN,-73,-39,-72},{INT32_MIN,-72,-41,-71},{INT32_MIN,-71,-44,-70},{INT32_MIN,-70,-46,-69},{INT32_MIN,-69,-49,-68},{INT32_MIN,-68,-51,-67},{INT32_MIN,-67,-54,-66},{INT32_MIN,-66,-56,-65},
{-47,INT32_MIN,INT32_MAX,-139},{-32,-139,INT32_MAX,-76}
};
mask_t half_loop_masks[]={
	{TRACK_MASK_NONE,24,-6,-3,half_loop_rects},{TRACK_MASK_NONE,36,-32,32,half_loop_rects+24},{TRACK_MASK_UNION,12,-48,56,half_loop_rects+60},{TRACK_MASK_DIFFERENCE,2,-32,96+72,half_loop_rects+72},
	{TRACK_MASK_UNION,1,6,-3,half_loop_rects+74},{TRACK_MASK_DIFFERENCE,2,-18,-7,half_loop_rects+75},{TRACK_MASK_NONE,2,-76,-6,half_loop_rects+77},{TRACK_MASK_NONE,2,-32,136,half_loop_rects+79},
	{TRACK_MASK_UNION,1,-6,-3,half_loop_rects+81},{TRACK_MASK_DIFFERENCE,2,26,-3,half_loop_rects+82},{TRACK_MASK_NONE,2,58,-13,half_loop_rects+84},{TRACK_MASK_NONE,2,16,96+32,half_loop_rects+86},
	{TRACK_MASK_NONE,24,6,-3,half_loop_rects+88},{TRACK_MASK_NONE,36,38,29,half_loop_rects+112},{TRACK_MASK_UNION,12,64,48,half_loop_rects+148},{TRACK_MASK_DIFFERENCE,2,48,160,half_loop_rects+160}
};
const track_section_t half_loop={TRACK_NO_SUPPORTS|TRACK_SPECIAL_HALF_LOOP,half_loop_curve,HALF_LOOP_LENGTH,{{VIEW_NEEDS_TRACK_MASK|VIEW_ENFORCE_NON_OVERLAPPING,4,half_loop_masks},{VIEW_NEEDS_TRACK_MASK,4,half_loop_masks+4},{VIEW_NEEDS_TRACK_MASK,4,half_loop_masks+8},{VIEW_NEEDS_TRACK_MASK|VIEW_ENFORCE_NON_OVERLAPPING,4,half_loop_masks+12}}};


rect_t flat_to_steep_up_rects[]={
//First angle
{INT32_MIN,INT32_MIN,-10,-27},{INT32_MIN,-27,-9,-26},{INT32_MIN,-26,-8,-25},{INT32_MIN,-25,-7,-24},{INT32_MIN,-24,-6,-23},{INT32_MIN,-23,-5,-22},{INT32_MIN,-22,-4,-21},{INT32_MIN,-21,-3,-20},{INT32_MIN,-20,-2,-19},{INT32_MIN,-19,-1,-18},{INT32_MIN,-18,0,-17},{INT32_MIN,-17,2,-16},{INT32_MIN,-16,3,-15},{INT32_MIN,-15,4,-14},{INT32_MIN,-14,5,-13},{INT32_MIN,-13,6,-12},{INT32_MIN,-12,7,-11},{INT32_MIN,-11,8,-10},{INT32_MIN,-10,9,-9},{INT32_MIN,-9,10,-8},{INT32_MIN,-8,11,-7},{INT32_MIN,-7,12,-6},{INT32_MIN,-6,13,-5},{INT32_MIN,-5,14,-4},{INT32_MIN,-4,15,-3},{INT32_MIN,-3,16,-2},{INT32_MIN,-2,18,-1},{INT32_MIN,-1,19,0},{INT32_MIN,0,20,1},{INT32_MIN,1,21,2},{INT32_MIN,2,22,3},{INT32_MIN,3,23,4},{INT32_MIN,4,24,5},{INT32_MIN,5,25,6},{INT32_MIN,6,26,7},{INT32_MIN,7,27,8},{INT32_MIN,8,28,9},{INT32_MIN,9,29,10},{INT32_MIN,10,30,11},{INT32_MIN,11,32,12},{INT32_MIN,12,33,13},{INT32_MIN,13,34,14},{INT32_MIN,14,35,15},{INT32_MIN,15,36,16},{INT32_MIN,16,37,17},{INT32_MIN,17,38,18},{INT32_MIN,18,39,19},{INT32_MIN,19,40,20},{INT32_MIN,20,41,21},{INT32_MIN,21,42,22},{INT32_MIN,22,125,INT32_MAX},
{-10,-84,-9,-83},{-10,-83,-8,-82},{-10,-82,-7,-81},{-10,-81,-5,-80},{-10,-80,-4,-79},{-10,-79,-3,-78},{-10,-78,-2,-77},{-10,-77,-1,-76},{-10,-76,1,-75},{-10,-75,2,-74},{-10,-74,3,-73},{-10,-73,4,-72},{-10,-72,5,-71},{-10,-71,7,-70},{-10,-70,8,-69},{-10,-69,9,-68},{-10,-68,10,-67},{-10,-67,11,-66},{-10,-66,12,-65},{-10,-65,14,-64},{-10,-64,15,-63},{-10,-63,16,-62},{-10,-62,17,-61},{-10,-61,18,-60},{-10,-60,20,-59},{-10,-59,21,-58},{-10,-58,22,-57},{-10,-57,23,-56},{-10,-56,24,-55},{-10,-55,26,-54},{-10,-54,27,-53},{-10,-53,28,-52},{-10,-52,29,-51},{-10,-51,30,-50},{-10,-50,32,-49},{-10,-49,33,-48},{-10,-48,34,-47},{-10,-47,35,-46},{-10,-46,36,-45},{-10,-45,37,-44},{-10,-44,39,-43},{-10,-43,40,-42},{-10,-42,41,-41},{-10,-41,42,-40},{-10,-40,43,-39},{-10,-39,45,-38},{-10,-38,46,-37},{-10,-37,47,-36},{-10,-36,48,-35},{-10,-35,49,-34},{-10,-34,51,-33},{-10,-33,52,-32},{-10,-32,53,-31},{-10,-31,55,-30},{-10,-30,56,-29},{-10,-29,57,-28},{-10,-28,59,-27},{-9,-27,60,-26},{-8,-26,61,-25},{-7,-25,61,-24},{-6,-24,63,-23},{-5,-23,64,-22},{-4,-22,65,-21},{-3,-21,66,-20},{-2,-20,68,-19},{-1,-19,69,-18},{0,-18,70,-17},{2,-17,71,-16},{3,-16,73,-15},{4,-15,74,-14},{5,-14,75,-13},{6,-13,77,-12},{7,-12,78,-11},{8,-11,79,-10},{9,-10,80,-9},{10,-9,81,-8},{11,-8,82,-7},{12,-7,84,-6},{13,-6,85,-5},{14,-5,86,-4},{15,-4,87,-3},{16,-3,89,-2},{18,-2,90,-1},{19,-1,91,0},{20,0,92,1},{21,1,93,2},{22,2,95,3},{23,3,96,4},{24,4,97,5},{25,5,98,6},{26,6,100,7},{27,7,101,8},{28,8,102,9},{29,9,103,10},{30,10,105,11},{32,11,106,12},{33,12,107,13},{34,13,108,14},{35,14,109,15},{36,15,111,16},{37,16,112,17},{38,17,113,18},{39,18,114,19},{40,19,116,20},{41,20,117,21},{42,21,118,22},
{-10,-101,31,-100},{-10,-100,33,-99},{-10,-99,35,-98},{-10,-98,37,-97},{-10,-97,39,-96},{-10,-96,41,-95},{-10,-95,43,-94},{-10,-94,45,-93},{-10,-93,47,-92},{-10,-92,49,-91},{-10,-91,51,-90},{-10,-90,53,-89},{-10,-89,55,-88},{-10,-88,57,-87},{-10,-87,59,-86},{-10,-86,61,-85},{-10,-85,63,-84},{-9,-84,65,-83},{-8,-83,67,-82},{-7,-82,69,-81},{-5,-81,71,-80},{-4,-80,73,-79},{-3,-79,75,-78},{-2,-78,77,-77},{-1,-77,78,-76},{1,-76,79,-75},{2,-75,80,-74},{3,-74,82,-73},{4,-73,84,-72},{5,-72,86,-71},{7,-71,88,-70},{8,-70,90,-69},{9,-69,92,-68},{10,-68,94,-67},{11,-67,96,-66},{12,-66,98,-65},{14,-65,100,-64},{15,-64,102,-63},{16,-63,104,-62},{17,-62,106,-61},{18,-61,108,-60},{20,-60,109,-59},{21,-59,111,-58},{22,-58,113,-57},{23,-57,115,-56},{24,-56,117,-55},{26,-55,119,-54},{27,-54,121,-53},{28,-53,123,-52},{29,-52,125,-51},{30,-51,125,-50},{32,-50,125,-49},{33,-49,125,-48},{34,-48,125,-47},{35,-47,125,-46},{36,-46,125,-45},{37,-45,125,-44},{39,-44,125,-43},{40,-43,125,-42},{41,-42,125,-41},{42,-41,125,-40},{43,-40,125,-39},{45,-39,125,-38},{46,-38,125,-37},{47,-37,125,-36},{48,-36,125,-35},{49,-35,125,-34},{51,-34,125,-33},{52,-33,125,-32},{53,-32,125,-31},{55,-31,125,-30},{56,-30,125,-29},{57,-29,125,-28},{59,-28,125,-27},{60,-27,125,-26},{61,-26,125,-24},{63,-24,125,-23},{64,-23,125,-22},{65,-22,125,-21},{66,-21,125,-20},{68,-20,125,-19},{69,-19,125,-18},{70,-18,125,-17},{71,-17,125,-16},{73,-16,125,-15},{74,-15,125,-14},{75,-14,125,-13},{77,-13,125,-12},{78,-12,125,-11},{79,-11,125,-10},{80,-10,125,-9},{81,-9,125,-8},{82,-8,125,-7},{84,-7,125,-6},{85,-6,125,-5},{86,-5,125,-4},{87,-4,125,-3},{89,-3,125,-2},{90,-2,125,-1},{91,-1,125,0},{92,0,125,1},{93,1,125,2},{95,2,125,3},{96,3,125,4},{97,4,125,5},{98,5,125,6},{100,6,125,7},{101,7,125,8},{102,8,125,9},{103,9,125,10},{105,10,125,11},{106,11,125,12},{107,12,125,13},{108,13,125,14},{109,14,125,15},{111,15,125,16},{112,16,125,17},{113,17,125,18},{114,18,125,19},{116,19,125,20},{117,20,125,21},{118,21,125,22},
{-10,INT32_MIN,INT32_MAX,-101},{31,-101,INT32_MAX,-100},{33,-100,INT32_MAX,-99},{35,-99,INT32_MAX,-98},{37,-98,INT32_MAX,-97},{39,-97,INT32_MAX,-96},{41,-96,INT32_MAX,-95},{43,-95,INT32_MAX,-94},{45,-94,INT32_MAX,-93},{47,-93,INT32_MAX,-92},{49,-92,INT32_MAX,-91},{51,-91,INT32_MAX,-90},{53,-90,INT32_MAX,-89},{55,-89,INT32_MAX,-88},{57,-88,INT32_MAX,-87},{59,-87,INT32_MAX,-86},{61,-86,INT32_MAX,-85},{63,-85,INT32_MAX,-84},{65,-84,INT32_MAX,-83},{67,-83,INT32_MAX,-82},{69,-82,INT32_MAX,-81},{71,-81,INT32_MAX,-80},{73,-80,INT32_MAX,-79},{75,-79,INT32_MAX,-78},{77,-78,INT32_MAX,-77},{78,-77,INT32_MAX,-76},{79,-76,INT32_MAX,-75},{80,-75,INT32_MAX,-74},{82,-74,INT32_MAX,-73},{84,-73,INT32_MAX,-72},{86,-72,INT32_MAX,-71},{88,-71,INT32_MAX,-70},{90,-70,INT32_MAX,-69},{92,-69,INT32_MAX,-68},{94,-68,INT32_MAX,-67},{96,-67,INT32_MAX,-66},{98,-66,INT32_MAX,-65},{100,-65,INT32_MAX,-64},{102,-64,INT32_MAX,-63},{104,-63,INT32_MAX,-62},{106,-62,INT32_MAX,-61},{108,-61,INT32_MAX,-60},{109,-60,INT32_MAX,-59},{111,-59,INT32_MAX,-58},{113,-58,INT32_MAX,-57},{115,-57,INT32_MAX,-56},{117,-56,INT32_MAX,-55},{119,-55,INT32_MAX,-54},{121,-54,INT32_MAX,-53},{123,-53,INT32_MAX,-52},{125,-52,INT32_MAX,INT32_MAX},
//Second angle
{INT32_MIN,INT32_MIN,76,-53},{INT32_MIN,-53,75,-52},{INT32_MIN,-52,74,-51},{INT32_MIN,-51,73,-50},{INT32_MIN,-50,72,-49},{INT32_MIN,-49,71,-48},{INT32_MIN,-48,70,-47},{INT32_MIN,-47,69,-46},{INT32_MIN,-46,68,-45},{INT32_MIN,-45,67,-44},{INT32_MIN,-44,66,-43},{INT32_MIN,-43,65,-42},{INT32_MIN,-42,64,-41},{INT32_MIN,-41,63,-40},{INT32_MIN,-40,62,-39},{INT32_MIN,-39,61,-38},{INT32_MIN,-38,60,-37},{INT32_MIN,-37,59,-36},{INT32_MIN,-36,58,-35},{INT32_MIN,-35,57,-34},{INT32_MIN,-34,56,-33},{INT32_MIN,-33,55,-32},{INT32_MIN,-32,54,-31},{INT32_MIN,-31,53,-30},{INT32_MIN,-30,52,-29},{INT32_MIN,-29,51,-28},{INT32_MIN,-28,50,-27},{INT32_MIN,-27,49,-26},{INT32_MIN,-26,48,-25},{INT32_MIN,-25,47,-24},{INT32_MIN,-24,46,-23},{INT32_MIN,-23,45,-22},{INT32_MIN,-22,44,-21},{INT32_MIN,-21,43,-20},{INT32_MIN,-20,42,-19},{INT32_MIN,-19,41,-18},{INT32_MIN,-18,40,-17},{INT32_MIN,-17,39,-16},{INT32_MIN,-16,38,-15},{INT32_MIN,-15,37,-14},{INT32_MIN,-14,36,-13},{INT32_MIN,-13,35,-12},{INT32_MIN,-12,34,-11},{INT32_MIN,-11,33,-10},{INT32_MIN,-10,32,-9},{INT32_MIN,-9,31,-8},{INT32_MIN,-8,30,-7},{INT32_MIN,-7,29,-6},{INT32_MIN,-6,28,-5},{INT32_MIN,-5,27,-4},{INT32_MIN,-4,26,-3},{INT32_MIN,-3,25,-2},{INT32_MIN,-2,24,-1},{INT32_MIN,-1,23,0},{INT32_MIN,0,22,1},{INT32_MIN,1,21,2},{INT32_MIN,2,20,3},{INT32_MIN,3,19,4},{INT32_MIN,4,18,5},{INT32_MIN,5,17,6},{INT32_MIN,6,16,7},{INT32_MIN,7,15,8},{INT32_MIN,8,14,9},{INT32_MIN,9,13,10},{INT32_MIN,10,12,11},{INT32_MIN,11,11,12},{INT32_MIN,12,10,13},{INT32_MIN,13,9,14},{INT32_MIN,14,8,15},{INT32_MIN,15,7,16},{INT32_MIN,16,6,17},{INT32_MIN,17,5,18},{INT32_MIN,18,4,19},{INT32_MIN,19,3,INT32_MAX},
{75,-53,76,-52},{74,-52,76,-51},{73,-51,76,-50},{72,-50,76,-49},{71,-49,76,-48},{70,-48,76,-47},{69,-47,76,-46},{68,-46,76,-45},{67,-45,76,-44},{66,-44,76,-43},{65,-43,76,-42},{64,-42,76,-41},{63,-41,76,-40},{62,-40,76,-39},{61,-39,76,-38},{60,-38,76,-37},{59,-37,76,-36},{58,-36,76,-35},{57,-35,76,-34},{56,-34,76,-33},{55,-33,76,-32},{54,-32,76,-31},{53,-31,76,-30},{52,-30,76,-29},{51,-29,76,-28},{50,-28,76,-27},{49,-27,76,-26},{48,-26,76,-25},{47,-25,76,-24},{46,-24,76,-23},{45,-23,76,-22},{44,-22,76,-21},{43,-21,76,-20},{42,-20,76,-19},{41,-19,76,-18},{40,-18,75,-17},{39,-17,74,-16},{38,-16,73,-15},{37,-15,72,-14},{36,-14,71,-13},{35,-13,70,-12},{34,-12,69,-11},{33,-11,68,-10},{32,-10,67,-9},{31,-9,66,-8},{30,-8,65,-7},{29,-7,64,-6},{28,-6,63,-5},{27,-5,62,-4},{26,-4,61,-3},{25,-3,60,-2},{24,-2,59,-1},{23,-1,58,0},{22,0,57,1},{21,1,56,2},{20,2,55,3},{19,3,54,4},{18,4,53,5},{17,5,52,6},{16,6,51,7},{15,7,50,8},{14,8,49,9},{13,9,48,10},{12,10,47,11},{11,11,46,12},{10,12,45,13},{9,13,44,14},{8,14,43,15},{7,15,42,16},{6,16,41,17},{5,17,40,18},{4,18,39,19},{3,19,39,INT32_MAX},
{75,-18,76,-17},{74,-17,76,-16},{73,-16,76,-15},{72,-15,76,-14},{71,-14,76,-13},{70,-13,76,-12},{69,-12,76,-11},{68,-11,76,-10},{67,-10,76,-9},{66,-9,76,-8},{65,-8,76,-7},{64,-7,76,-6},{63,-6,76,-5},{62,-5,76,-4},{61,-4,76,-3},{60,-3,76,-2},{59,-2,76,-1},{58,-1,76,0},{57,0,76,1},{56,1,76,2},{55,2,76,3},{54,3,76,4},{53,4,76,5},{52,5,76,6},{51,6,76,7},{50,7,76,8},{49,8,76,9},{48,9,76,10},{47,10,76,11},{46,11,76,12},{45,12,76,13},{44,13,76,14},{43,14,76,15},{42,15,76,16},{41,16,76,17},{40,17,76,18},{39,18,76,INT32_MAX},
{76,INT32_MIN,INT32_MAX,INT32_MAX},
//Third angle
{-87,INT32_MIN,INT32_MAX,-55},{-86,-55,INT32_MAX,-54},{-85,-54,INT32_MAX,-53},{-84,-53,INT32_MAX,-52},{-83,-52,INT32_MAX,-51},{-82,-51,INT32_MAX,-50},{-81,-50,INT32_MAX,-49},{-80,-49,INT32_MAX,-48},{-79,-48,INT32_MAX,-47},{-78,-47,INT32_MAX,-46},{-77,-46,INT32_MAX,-45},{-76,-45,INT32_MAX,-44},{-75,-44,INT32_MAX,-43},{-74,-43,INT32_MAX,-42},{-73,-42,INT32_MAX,-41},{-72,-41,INT32_MAX,-40},{-71,-40,INT32_MAX,-39},{-70,-39,INT32_MAX,-38},{-69,-38,INT32_MAX,-37},{-68,-37,INT32_MAX,-36},{-67,-36,INT32_MAX,-35},{-66,-35,INT32_MAX,-34},{-65,-34,INT32_MAX,-33},{-64,-33,INT32_MAX,-32},{-63,-32,INT32_MAX,-31},{-62,-31,INT32_MAX,-30},{-61,-30,INT32_MAX,-29},{-60,-29,INT32_MAX,-28},{-59,-28,INT32_MAX,-27},{-58,-27,INT32_MAX,-26},{-57,-26,INT32_MAX,-25},{-56,-25,INT32_MAX,-24},{-55,-24,INT32_MAX,-23},{-54,-23,INT32_MAX,-22},{-53,-22,INT32_MAX,-21},{-52,-21,INT32_MAX,-20},{-51,-20,INT32_MAX,-19},{-50,-19,INT32_MAX,-18},{-49,-18,INT32_MAX,-17},{-48,-17,INT32_MAX,-16},{-47,-16,INT32_MAX,-15},{-46,-15,INT32_MAX,-14},{-45,-14,INT32_MAX,-13},{-44,-13,INT32_MAX,-12},{-43,-12,INT32_MAX,-11},{-42,-11,INT32_MAX,-10},{-41,-10,INT32_MAX,-9},{-40,-9,INT32_MAX,-8},{-39,-8,INT32_MAX,-7},{-38,-7,INT32_MAX,-6},{-37,-6,INT32_MAX,-5},{-36,-5,INT32_MAX,-4},{-35,-4,INT32_MAX,-3},{-34,-3,INT32_MAX,-2},{-33,-2,INT32_MAX,-1},{-32,-1,INT32_MAX,0},{-31,0,INT32_MAX,1},{-30,1,INT32_MAX,2},{-29,2,INT32_MAX,3},{-28,3,INT32_MAX,4},{-27,4,INT32_MAX,5},{-26,5,INT32_MAX,6},{-25,6,INT32_MAX,7},{-24,7,INT32_MAX,8},{-23,8,INT32_MAX,9},{-22,9,INT32_MAX,10},{-21,10,INT32_MAX,11},{-20,11,INT32_MAX,12},{-19,12,INT32_MAX,13},{-18,13,INT32_MAX,14},{-17,14,INT32_MAX,15},{-16,15,INT32_MAX,16},{-15,16,INT32_MAX,17},{-14,17,INT32_MAX,INT32_MAX},
{-87,-55,-86,-54},{-87,-54,-85,-53},{-87,-53,-84,-52},{-87,-52,-83,-51},{-87,-51,-82,-50},{-87,-50,-81,-49},{-87,-49,-80,-48},{-87,-48,-79,-47},{-87,-47,-78,-46},{-87,-46,-77,-45},{-87,-45,-76,-44},{-87,-44,-75,-43},{-87,-43,-74,-42},{-87,-42,-73,-41},{-87,-41,-72,-40},{-87,-40,-71,-39},{-87,-39,-70,-38},{-87,-38,-69,-37},{-87,-37,-68,-36},{-87,-36,-67,-35},{-87,-35,-66,-34},{-87,-34,-65,-33},{-87,-33,-64,-32},{-87,-32,-63,-31},{-87,-31,-62,-30},{-87,-30,-61,-29},{-87,-29,-60,-28},{-87,-28,-59,-27},{-87,-27,-58,-26},{-87,-26,-57,-25},{-87,-25,-56,-24},{-87,-24,-55,-23},{-87,-23,-54,-22},{-87,-22,-53,-21},{-87,-21,-52,-20},{-86,-20,-51,-19},{-85,-19,-50,-18},{-84,-18,-49,-17},{-83,-17,-48,-16},{-82,-16,-47,-15},{-81,-15,-46,-14},{-80,-14,-45,-13},{-79,-13,-44,-12},{-78,-12,-43,-11},{-77,-11,-42,-10},{-76,-10,-41,-9},{-75,-9,-40,-8},{-74,-8,-39,-7},{-73,-7,-38,-6},{-72,-6,-37,-5},{-71,-5,-36,-4},{-70,-4,-35,-3},{-69,-3,-34,-2},{-68,-2,-33,-1},{-67,-1,-32,0},{-66,0,-31,1},{-65,1,-30,2},{-64,2,-29,3},{-63,3,-28,4},{-62,4,-27,5},{-61,5,-26,6},{-60,6,-25,7},{-59,7,-24,8},{-58,8,-23,9},{-57,9,-22,10},{-56,10,-21,11},{-55,11,-20,12},{-54,12,-19,13},{-53,13,-18,14},{-52,14,-17,15},{-51,15,-16,16},{-50,16,-15,17},{-50,17,-14,INT32_MAX},
{-87,-20,-86,-19},{-87,-19,-85,-18},{-87,-18,-84,-17},{-87,-17,-83,-16},{-87,-16,-82,-15},{-87,-15,-81,-14},{-87,-14,-80,-13},{-87,-13,-79,-12},{-87,-12,-78,-11},{-87,-11,-77,-10},{-87,-10,-76,-9},{-87,-9,-75,-8},{-87,-8,-74,-7},{-87,-7,-73,-6},{-87,-6,-72,-5},{-87,-5,-71,-4},{-87,-4,-70,-3},{-87,-3,-69,-2},{-87,-2,-68,-1},{-87,-1,-67,0},{-87,0,-66,1},{-87,1,-65,2},{-87,2,-64,3},{-87,3,-63,4},{-87,4,-62,5},{-87,5,-61,6},{-87,6,-60,7},{-87,7,-59,8},{-87,8,-58,9},{-87,9,-57,10},{-87,10,-56,11},{-87,11,-55,12},{-87,12,-54,13},{-87,13,-53,14},{-87,14,-52,15},{-87,15,-51,16},{-87,16,-50,INT32_MAX},
{INT32_MIN,INT32_MIN,-87,INT32_MAX},
//Fourth angle
{10,INT32_MIN,INT32_MAX,-27},{9,-27,INT32_MAX,-26},{8,-26,INT32_MAX,-25},{7,-25,INT32_MAX,-24},{6,-24,INT32_MAX,-23},{5,-23,INT32_MAX,-22},{4,-22,INT32_MAX,-21},{3,-21,INT32_MAX,-20},{2,-20,INT32_MAX,-19},{1,-19,INT32_MAX,-18},{0,-18,INT32_MAX,-17},{-2,-17,INT32_MAX,-16},{-3,-16,INT32_MAX,-15},{-4,-15,INT32_MAX,-14},{-5,-14,INT32_MAX,-13},{-6,-13,INT32_MAX,-12},{-7,-12,INT32_MAX,-11},{-8,-11,INT32_MAX,-10},{-9,-10,INT32_MAX,-9},{-10,-9,INT32_MAX,-8},{-11,-8,INT32_MAX,-7},{-12,-7,INT32_MAX,-6},{-13,-6,INT32_MAX,-5},{-14,-5,INT32_MAX,-4},{-15,-4,INT32_MAX,-3},{-16,-3,INT32_MAX,-2},{-18,-2,INT32_MAX,-1},{-19,-1,INT32_MAX,0},{-20,0,INT32_MAX,1},{-21,1,INT32_MAX,2},{-22,2,INT32_MAX,3},{-23,3,INT32_MAX,4},{-24,4,INT32_MAX,5},{-25,5,INT32_MAX,6},{-26,6,INT32_MAX,7},{-27,7,INT32_MAX,8},{-28,8,INT32_MAX,9},{-29,9,INT32_MAX,10},{-30,10,INT32_MAX,11},{-32,11,INT32_MAX,12},{-33,12,INT32_MAX,13},{-34,13,INT32_MAX,14},{-35,14,INT32_MAX,15},{-36,15,INT32_MAX,16},{-37,16,INT32_MAX,17},{-38,17,INT32_MAX,18},{-39,18,INT32_MAX,19},{-40,19,INT32_MAX,20},{-41,20,INT32_MAX,21},{-42,21,INT32_MAX,22},{-125,22,INT32_MAX,INT32_MAX},
{9,-84,10,-83},{8,-83,10,-82},{7,-82,10,-81},{5,-81,10,-80},{4,-80,10,-79},{3,-79,10,-78},{2,-78,10,-77},{1,-77,10,-76},{-1,-76,10,-75},{-2,-75,10,-74},{-3,-74,10,-73},{-4,-73,10,-72},{-5,-72,10,-71},{-7,-71,10,-70},{-8,-70,10,-69},{-9,-69,10,-68},{-10,-68,10,-67},{-11,-67,10,-66},{-12,-66,10,-65},{-14,-65,10,-64},{-15,-64,10,-63},{-16,-63,10,-62},{-17,-62,10,-61},{-18,-61,10,-60},{-20,-60,10,-59},{-21,-59,10,-58},{-22,-58,10,-57},{-23,-57,10,-56},{-24,-56,10,-55},{-26,-55,10,-54},{-27,-54,10,-53},{-28,-53,10,-52},{-29,-52,10,-51},{-30,-51,10,-50},{-32,-50,10,-49},{-33,-49,10,-48},{-34,-48,10,-47},{-35,-47,10,-46},{-36,-46,10,-45},{-37,-45,10,-44},{-39,-44,10,-43},{-40,-43,10,-42},{-41,-42,10,-41},{-42,-41,10,-40},{-43,-40,10,-39},{-45,-39,10,-38},{-46,-38,10,-37},{-47,-37,10,-36},{-48,-36,10,-35},{-49,-35,10,-34},{-51,-34,10,-33},{-52,-33,10,-32},{-53,-32,10,-31},{-55,-31,10,-30},{-56,-30,10,-29},{-57,-29,10,-28},{-59,-28,10,-27},{-60,-27,9,-26},{-61,-26,8,-25},{-61,-25,7,-24},{-63,-24,6,-23},{-64,-23,5,-22},{-65,-22,4,-21},{-66,-21,3,-20},{-68,-20,2,-19},{-69,-19,1,-18},{-70,-18,0,-17},{-71,-17,-2,-16},{-73,-16,-3,-15},{-74,-15,-4,-14},{-75,-14,-5,-13},{-77,-13,-6,-12},{-78,-12,-7,-11},{-79,-11,-8,-10},{-80,-10,-9,-9},{-81,-9,-10,-8},{-82,-8,-11,-7},{-84,-7,-12,-6},{-85,-6,-13,-5},{-86,-5,-14,-4},{-87,-4,-15,-3},{-89,-3,-16,-2},{-90,-2,-18,-1},{-91,-1,-19,0},{-92,0,-20,1},{-93,1,-21,2},{-95,2,-22,3},{-96,3,-23,4},{-97,4,-24,5},{-98,5,-25,6},{-100,6,-26,7},{-101,7,-27,8},{-102,8,-28,9},{-103,9,-29,10},{-105,10,-30,11},{-106,11,-32,12},{-107,12,-33,13},{-108,13,-34,14},{-109,14,-35,15},{-111,15,-36,16},{-112,16,-37,17},{-113,17,-38,18},{-114,18,-39,19},{-116,19,-40,20},{-117,20,-41,21},{-118,21,-42,22},
{-31,-101,10,-100},{-33,-100,10,-99},{-35,-99,10,-98},{-37,-98,10,-97},{-39,-97,10,-96},{-41,-96,10,-95},{-43,-95,10,-94},{-45,-94,10,-93},{-47,-93,10,-92},{-49,-92,10,-91},{-51,-91,10,-90},{-53,-90,10,-89},{-55,-89,10,-88},{-57,-88,10,-87},{-59,-87,10,-86},{-61,-86,10,-85},{-63,-85,10,-84},{-65,-84,9,-83},{-67,-83,8,-82},{-69,-82,7,-81},{-71,-81,5,-80},{-73,-80,4,-79},{-75,-79,3,-78},{-77,-78,2,-77},{-78,-77,1,-76},{-79,-76,-1,-75},{-80,-75,-2,-74},{-82,-74,-3,-73},{-84,-73,-4,-72},{-86,-72,-5,-71},{-88,-71,-7,-70},{-90,-70,-8,-69},{-92,-69,-9,-68},{-94,-68,-10,-67},{-96,-67,-11,-66},{-98,-66,-12,-65},{-100,-65,-14,-64},{-102,-64,-15,-63},{-104,-63,-16,-62},{-106,-62,-17,-61},{-108,-61,-18,-60},{-109,-60,-20,-59},{-111,-59,-21,-58},{-113,-58,-22,-57},{-115,-57,-23,-56},{-117,-56,-24,-55},{-119,-55,-26,-54},{-121,-54,-27,-53},{-123,-53,-28,-52},{-125,-52,-29,-51},{-125,-51,-30,-50},{-125,-50,-32,-49},{-125,-49,-33,-48},{-125,-48,-34,-47},{-125,-47,-35,-46},{-125,-46,-36,-45},{-125,-45,-37,-44},{-125,-44,-39,-43},{-125,-43,-40,-42},{-125,-42,-41,-41},{-125,-41,-42,-40},{-125,-40,-43,-39},{-125,-39,-45,-38},{-125,-38,-46,-37},{-125,-37,-47,-36},{-125,-36,-48,-35},{-125,-35,-49,-34},{-125,-34,-51,-33},{-125,-33,-52,-32},{-125,-32,-53,-31},{-125,-31,-55,-30},{-125,-30,-56,-29},{-125,-29,-57,-28},{-125,-28,-59,-27},{-125,-27,-60,-26},{-125,-26,-61,-24},{-125,-24,-63,-23},{-125,-23,-64,-22},{-125,-22,-65,-21},{-125,-21,-66,-20},{-125,-20,-68,-19},{-125,-19,-69,-18},{-125,-18,-70,-17},{-125,-17,-71,-16},{-125,-16,-73,-15},{-125,-15,-74,-14},{-125,-14,-75,-13},{-125,-13,-77,-12},{-125,-12,-78,-11},{-125,-11,-79,-10},{-125,-10,-80,-9},{-125,-9,-81,-8},{-125,-8,-82,-7},{-125,-7,-84,-6},{-125,-6,-85,-5},{-125,-5,-86,-4},{-125,-4,-87,-3},{-125,-3,-89,-2},{-125,-2,-90,-1},{-125,-1,-91,0},{-125,0,-92,1},{-125,1,-93,2},{-125,2,-95,3},{-125,3,-96,4},{-125,4,-97,5},{-125,5,-98,6},{-125,6,-100,7},{-125,7,-101,8},{-125,8,-102,9},{-125,9,-103,10},{-125,10,-105,11},{-125,11,-106,12},{-125,12,-107,13},{-125,13,-108,14},{-125,14,-109,15},{-125,15,-111,16},{-125,16,-112,17},{-125,17,-113,18},{-125,18,-114,19},{-125,19,-116,20},{-125,20,-117,21},{-125,21,-118,22},
{INT32_MIN,INT32_MIN,10,-101},{INT32_MIN,-101,-31,-100},{INT32_MIN,-100,-33,-99},{INT32_MIN,-99,-35,-98},{INT32_MIN,-98,-37,-97},{INT32_MIN,-97,-39,-96},{INT32_MIN,-96,-41,-95},{INT32_MIN,-95,-43,-94},{INT32_MIN,-94,-45,-93},{INT32_MIN,-93,-47,-92},{INT32_MIN,-92,-49,-91},{INT32_MIN,-91,-51,-90},{INT32_MIN,-90,-53,-89},{INT32_MIN,-89,-55,-88},{INT32_MIN,-88,-57,-87},{INT32_MIN,-87,-59,-86},{INT32_MIN,-86,-61,-85},{INT32_MIN,-85,-63,-84},{INT32_MIN,-84,-65,-83},{INT32_MIN,-83,-67,-82},{INT32_MIN,-82,-69,-81},{INT32_MIN,-81,-71,-80},{INT32_MIN,-80,-73,-79},{INT32_MIN,-79,-75,-78},{INT32_MIN,-78,-77,-77},{INT32_MIN,-77,-78,-76},{INT32_MIN,-76,-79,-75},{INT32_MIN,-75,-80,-74},{INT32_MIN,-74,-82,-73},{INT32_MIN,-73,-84,-72},{INT32_MIN,-72,-86,-71},{INT32_MIN,-71,-88,-70},{INT32_MIN,-70,-90,-69},{INT32_MIN,-69,-92,-68},{INT32_MIN,-68,-94,-67},{INT32_MIN,-67,-96,-66},{INT32_MIN,-66,-98,-65},{INT32_MIN,-65,-100,-64},{INT32_MIN,-64,-102,-63},{INT32_MIN,-63,-104,-62},{INT32_MIN,-62,-106,-61},{INT32_MIN,-61,-108,-60},{INT32_MIN,-60,-109,-59},{INT32_MIN,-59,-111,-58},{INT32_MIN,-58,-113,-57},{INT32_MIN,-57,-115,-56},{INT32_MIN,-56,-117,-55},{INT32_MIN,-55,-119,-54},{INT32_MIN,-54,-121,-53},{INT32_MIN,-53,-123,-52},{INT32_MIN,-52,-125,INT32_MAX}
};
mask_t flat_to_steep_up_masks[]={
	{TRACK_MASK_NONE,51,0,0,flat_to_steep_up_rects},{TRACK_MASK_NONE,106,-32,16,flat_to_steep_up_rects+51},{TRACK_MASK_NONE,122,-64,48,flat_to_steep_up_rects+157},{TRACK_MASK_NONE,51,-96,88,flat_to_steep_up_rects+279},
	{TRACK_MASK_NONE,74,0,0,flat_to_steep_up_rects+330},{TRACK_MASK_NONE,73,-32,-16,flat_to_steep_up_rects+404},{TRACK_MASK_NONE,37,-64,-16,flat_to_steep_up_rects+477},{TRACK_MASK_NONE,1,-96,-8,flat_to_steep_up_rects+514},
	{TRACK_MASK_NONE,74,0,0,flat_to_steep_up_rects+515},{TRACK_MASK_NONE,73,32,-16,flat_to_steep_up_rects+589},{TRACK_MASK_NONE,37,64,-16,flat_to_steep_up_rects+662},{TRACK_MASK_NONE,1,96,-8,flat_to_steep_up_rects+699},
	{TRACK_MASK_NONE,51,0,0,flat_to_steep_up_rects+700},{TRACK_MASK_NONE,106,32,16,flat_to_steep_up_rects+751},{TRACK_MASK_NONE,122,64,48,flat_to_steep_up_rects+857},{TRACK_MASK_NONE,51,96,88,flat_to_steep_up_rects+979}
};

const track_section_t flat_to_steep_up={TRACK_OFFSET_SPRITE_MASK,flat_to_steep_up_curve,FLAT_TO_STEEP_LENGTH,{{0,4,flat_to_steep_up_masks},{0,4,flat_to_steep_up_masks+4},{0,4,flat_to_steep_up_masks+8},{0,4,flat_to_steep_up_masks+12}}};

rect_t steep_to_flat_up_rects[]={
//First angle
{INT32_MIN,INT32_MIN,-16,-58},{INT32_MIN,-58,-14,-57},{INT32_MIN,-57,-12,-56},{INT32_MIN,-56,-10,-55},{INT32_MIN,-55,-8,-54},{INT32_MIN,-54,-6,-53},{INT32_MIN,-53,-4,-52},{INT32_MIN,-52,-2,-51},{INT32_MIN,-51,0,-50},{INT32_MIN,-50,2,-49},{INT32_MIN,-49,4,-48},{INT32_MIN,-48,6,-47},{INT32_MIN,-47,8,-46},{INT32_MIN,-46,10,-45},{INT32_MIN,-45,12,-44},{INT32_MIN,-44,14,-43},{INT32_MIN,-43,16,-42},{INT32_MIN,-42,18,-41},{INT32_MIN,-41,20,-40},{INT32_MIN,-40,22,-39},{INT32_MIN,-39,24,-38},{INT32_MIN,-38,26,-37},{INT32_MIN,-37,28,-36},{INT32_MIN,-36,30,-35},{INT32_MIN,-35,32,-34},{INT32_MIN,-34,34,-33},{INT32_MIN,-33,36,-32},{INT32_MIN,-32,38,-31},{INT32_MIN,-31,40,-30},{INT32_MIN,-30,42,-29},{INT32_MIN,-29,44,-28},{INT32_MIN,-28,46,-27},{INT32_MIN,-27,INT32_MAX,INT32_MAX},
{-16,-144,-15,-143},{-16,-143,-14,-142},{-16,-142,-13,-141},{-16,-141,-12,-140},{-16,-140,-11,-139},{-16,-139,-10,-138},{-16,-138,-9,-137},{-16,-137,-8,-136},{-16,-136,-7,-135},{-16,-135,-6,-134},{-16,-134,-5,-133},{-16,-133,-4,-132},{-16,-132,-3,-131},{-16,-131,-2,-130},{-16,-130,-1,-129},{-16,-129,0,-128},{-16,-128,1,-127},{-16,-127,2,-126},{-16,-126,3,-125},{-16,-125,4,-124},{-16,-124,5,-123},{-16,-123,6,-122},{-16,-122,7,-121},{-16,-121,8,-120},{-16,-120,9,-119},{-16,-119,10,-118},{-16,-118,11,-117},{-16,-117,12,-116},{-16,-116,13,-115},{-16,-115,14,-114},{-16,-114,15,-113},{-16,-113,16,-112},{-16,-112,17,-111},{-16,-111,18,-110},{-16,-110,19,-109},{-16,-109,20,-108},{-16,-108,21,-107},{-16,-107,22,-106},{-16,-106,23,-105},{-16,-105,24,-104},{-16,-104,25,-103},{-16,-103,26,-102},{-16,-102,27,-101},{-16,-101,28,-100},{-16,-100,29,-99},{-16,-99,30,-98},{-16,-98,31,-97},{-16,-97,32,-96},{-16,-96,33,-95},{-16,-95,34,-94},{-16,-94,35,-93},{-16,-93,36,-92},{-16,-92,37,-91},{-16,-91,38,-90},{-16,-90,39,-89},{-16,-89,40,-88},{-16,-88,41,-87},{-16,-87,42,-86},{-16,-86,43,-85},{-16,-85,44,-84},{-16,-84,45,-83},{-16,-83,47,-82},{-16,-82,49,-81},{-16,-81,51,-80},{-16,-80,53,-79},{-16,-79,55,-78},{-16,-78,57,-77},{-16,-77,59,-76},{-16,-76,61,-75},{-16,-75,63,-74},{-16,-74,65,-73},{-16,-73,67,-72},{-16,-72,69,-71},{-16,-71,71,-70},{-16,-70,73,-69},{-16,-69,75,-68},{-16,-68,77,-67},{-16,-67,79,-66},{-16,-66,81,-65},{-16,-65,83,-64},{-16,-64,85,-63},{-16,-63,87,-62},{-16,-62,89,-61},{-16,-61,91,-60},{-16,-60,93,-59},{-16,-59,95,-58},{-14,-58,97,-57},{-12,-57,99,-56},{-10,-56,101,-55},{-8,-55,103,-54},{-6,-54,105,-53},{-4,-53,107,-52},{-2,-52,109,-51},{0,-51,111,-50},{2,-50,113,-49},{4,-49,115,-48},{6,-48,117,-47},{8,-47,119,-46},{10,-46,121,-45},{12,-45,123,-44},{14,-44,125,-43},{16,-43,127,-42},{18,-42,129,-41},{20,-41,131,-40},{22,-40,133,-39},{24,-39,135,-38},{26,-38,137,-37},{28,-37,139,-36},{30,-36,141,-35},{32,-35,143,-34},{34,-34,145,-33},{36,-33,147,-32},{38,-32,149,-31},{40,-31,151,-30},{42,-30,153,-29},{44,-29,155,-28},{46,-28,157,-27},
{-16,-163,-14,-162},{-16,-162,-12,-161},{-16,-161,-10,-160},{-16,-160,-8,-159},{-16,-159,-6,-158},{-16,-158,-4,-157},{-16,-157,-2,-156},{-16,-156,0,-155},{-16,-155,2,-154},{-16,-154,4,-153},{-16,-153,6,-152},{-16,-152,8,-151},{-16,-151,10,-150},{-16,-150,12,-149},{-16,-149,14,-148},{-16,-148,16,-147},{-16,-147,18,-146},{-16,-146,20,-145},{-16,-145,22,-144},{-15,-144,24,-143},{-14,-143,26,-142},{-13,-142,28,-141},{-12,-141,30,-140},{-11,-140,32,-139},{-10,-139,34,-138},{-9,-138,36,-137},{-8,-137,38,-136},{-7,-136,40,-135},{-6,-135,42,-134},{-5,-134,44,-133},{-4,-133,46,-132},{-3,-132,48,-131},{-2,-131,50,-130},{-1,-130,52,-129},{0,-129,54,-128},{1,-128,56,-127},{2,-127,58,-126},{3,-126,60,-125},{4,-125,62,-124},{5,-124,64,-123},{6,-123,66,-122},{7,-122,68,-121},{8,-121,70,-120},{9,-120,72,-119},{10,-119,74,-118},{11,-118,76,-117},{12,-117,78,-116},{13,-116,80,-115},{14,-115,82,-114},{15,-114,83,-113},{16,-113,84,-112},{17,-112,85,-111},{18,-111,86,-110},{19,-110,87,-109},{20,-109,88,-108},{21,-108,89,-107},{22,-107,90,-106},{23,-106,91,-105},{24,-105,92,-104},{25,-104,93,-103},{26,-103,94,-102},{27,-102,95,-101},{28,-101,96,-100},{29,-100,97,-99},{30,-99,98,-98},{31,-98,99,-97},{32,-97,100,-96},{33,-96,101,-95},{34,-95,102,-94},{35,-94,103,-93},{36,-93,104,-92},{37,-92,105,-91},{38,-91,106,-90},{39,-90,107,-89},{40,-89,108,-88},{41,-88,109,-87},{42,-87,110,-86},{43,-86,111,-85},{44,-85,112,-84},{45,-84,113,-83},{47,-83,114,-82},{49,-82,115,-81},{51,-81,116,-80},{53,-80,117,-79},{55,-79,118,-78},{57,-78,119,-77},{59,-77,120,-76},{61,-76,121,-75},{63,-75,122,-74},{65,-74,123,-73},{67,-73,124,-72},{69,-72,125,-71},{71,-71,126,-70},{73,-70,127,-69},{75,-69,128,-68},{77,-68,129,-67},{79,-67,130,-66},{81,-66,131,-65},{83,-65,132,-64},{85,-64,133,-63},{87,-63,134,-62},{89,-62,135,-61},{91,-61,136,-60},{93,-60,137,-59},{95,-59,138,-58},{97,-58,139,-57},{99,-57,140,-56},{101,-56,141,-55},{103,-55,142,-54},{105,-54,143,-53},{107,-53,144,-52},{109,-52,145,-51},{111,-51,146,-50},{113,-50,147,-49},{115,-49,148,-48},{117,-48,149,-47},{119,-47,150,-46},{121,-46,151,-45},{123,-45,152,-44},{125,-44,153,-43},{127,-43,154,-42},{129,-42,155,-41},{131,-41,156,-40},{133,-40,157,-39},{135,-39,158,-38},{137,-38,159,-37},{139,-37,160,-36},{141,-36,161,-35},{143,-35,162,-34},{145,-34,163,-33},{147,-33,164,-32},{149,-32,165,-31},{151,-31,166,-30},{153,-30,167,-29},{155,-29,168,-28},{157,-28,169,-27},
{-16,INT32_MIN,INT32_MAX,-163},{-14,-163,INT32_MAX,-162},{-12,-162,INT32_MAX,-161},{-10,-161,INT32_MAX,-160},{-8,-160,INT32_MAX,-159},{-6,-159,INT32_MAX,-158},{-4,-158,INT32_MAX,-157},{-2,-157,INT32_MAX,-156},{0,-156,INT32_MAX,-155},{2,-155,INT32_MAX,-154},{4,-154,INT32_MAX,-153},{6,-153,INT32_MAX,-152},{8,-152,INT32_MAX,-151},{10,-151,INT32_MAX,-150},{12,-150,INT32_MAX,-149},{14,-149,INT32_MAX,-148},{16,-148,INT32_MAX,-147},{18,-147,INT32_MAX,-146},{20,-146,INT32_MAX,-145},{22,-145,INT32_MAX,-144},{24,-144,INT32_MAX,-143},{26,-143,INT32_MAX,-142},{28,-142,INT32_MAX,-141},{30,-141,INT32_MAX,-140},{32,-140,INT32_MAX,-139},{34,-139,INT32_MAX,-138},{36,-138,INT32_MAX,-137},{38,-137,INT32_MAX,-136},{40,-136,INT32_MAX,-135},{42,-135,INT32_MAX,-134},{44,-134,INT32_MAX,-133},{46,-133,INT32_MAX,-132},{48,-132,INT32_MAX,-131},{50,-131,INT32_MAX,-130},{52,-130,INT32_MAX,-129},{54,-129,INT32_MAX,-128},{56,-128,INT32_MAX,-127},{58,-127,INT32_MAX,-126},{60,-126,INT32_MAX,-125},{62,-125,INT32_MAX,-124},{64,-124,INT32_MAX,-123},{66,-123,INT32_MAX,-122},{68,-122,INT32_MAX,-121},{70,-121,INT32_MAX,-120},{72,-120,INT32_MAX,-119},{74,-119,INT32_MAX,-118},{76,-118,INT32_MAX,-117},{78,-117,INT32_MAX,-116},{80,-116,INT32_MAX,-115},{82,-115,INT32_MAX,-114},{83,-114,INT32_MAX,-113},{84,-113,INT32_MAX,-112},{85,-112,INT32_MAX,-111},{86,-111,INT32_MAX,-110},{87,-110,INT32_MAX,-109},{88,-109,INT32_MAX,-108},{89,-108,INT32_MAX,-107},{90,-107,INT32_MAX,-106},{91,-106,INT32_MAX,-105},{92,-105,INT32_MAX,-104},{93,-104,INT32_MAX,-103},{94,-103,INT32_MAX,-102},{95,-102,INT32_MAX,-101},{96,-101,INT32_MAX,-100},{97,-100,INT32_MAX,-99},{98,-99,INT32_MAX,-98},{99,-98,INT32_MAX,-97},{100,-97,INT32_MAX,-96},{101,-96,INT32_MAX,-95},{102,-95,INT32_MAX,-94},{103,-94,INT32_MAX,-93},{104,-93,INT32_MAX,-92},{105,-92,INT32_MAX,-91},{106,-91,INT32_MAX,-90},{107,-90,INT32_MAX,-89},{108,-89,INT32_MAX,-88},{109,-88,INT32_MAX,-87},{110,-87,INT32_MAX,-86},{111,-86,INT32_MAX,-85},{112,-85,INT32_MAX,-84},{113,-84,INT32_MAX,-83},{114,-83,INT32_MAX,-82},{115,-82,INT32_MAX,-81},{116,-81,INT32_MAX,-80},{117,-80,INT32_MAX,-79},{118,-79,INT32_MAX,-78},{119,-78,INT32_MAX,-77},{120,-77,INT32_MAX,-76},{121,-76,INT32_MAX,-75},{122,-75,INT32_MAX,-74},{123,-74,INT32_MAX,-73},{124,-73,INT32_MAX,-72},{125,-72,INT32_MAX,-71},{126,-71,INT32_MAX,-70},{127,-70,INT32_MAX,-69},{128,-69,INT32_MAX,-68},{129,-68,INT32_MAX,-67},{130,-67,INT32_MAX,-66},{131,-66,INT32_MAX,-65},{132,-65,INT32_MAX,-64},{133,-64,INT32_MAX,-63},{134,-63,INT32_MAX,-62},{135,-62,INT32_MAX,-61},{136,-61,INT32_MAX,-60},{137,-60,INT32_MAX,-59},{138,-59,INT32_MAX,-58},{139,-58,INT32_MAX,-57},{140,-57,INT32_MAX,-56},{141,-56,INT32_MAX,-55},{142,-55,INT32_MAX,-54},{143,-54,INT32_MAX,-53},{144,-53,INT32_MAX,-52},{145,-52,INT32_MAX,-51},{146,-51,INT32_MAX,-50},{147,-50,INT32_MAX,-49},{148,-49,INT32_MAX,-48},{149,-48,INT32_MAX,-47},{150,-47,INT32_MAX,-46},{151,-46,INT32_MAX,-45},{152,-45,INT32_MAX,-44},{153,-44,INT32_MAX,-43},{154,-43,INT32_MAX,-42},{155,-42,INT32_MAX,-41},{156,-41,INT32_MAX,-40},{157,-40,INT32_MAX,-39},{158,-39,INT32_MAX,-38},{159,-38,INT32_MAX,-37},{160,-37,INT32_MAX,-36},{161,-36,INT32_MAX,-35},{162,-35,INT32_MAX,-34},{163,-34,INT32_MAX,-33},{164,-33,INT32_MAX,-32},{165,-32,INT32_MAX,-31},{166,-31,INT32_MAX,-30},{167,-30,INT32_MAX,-29},{168,-29,INT32_MAX,-28},{169,-28,INT32_MAX,-27},
//Second angle
{INT32_MIN,INT32_MIN,0,-48},{INT32_MIN,-48,1,-47},{INT32_MIN,-47,2,-45},{INT32_MIN,-45,3,-44},{INT32_MIN,-44,4,-42},{INT32_MIN,-42,5,-41},{INT32_MIN,-41,6,-39},{INT32_MIN,-39,7,-38},{INT32_MIN,-38,8,-36},{INT32_MIN,-36,9,-35},{INT32_MIN,-35,10,-33},{INT32_MIN,-33,11,-32},{INT32_MIN,-32,12,-30},{INT32_MIN,-30,13,-29},{INT32_MIN,-29,14,-27},{INT32_MIN,-27,15,-26},{INT32_MIN,-26,16,-24},{INT32_MIN,-24,17,-23},{INT32_MIN,-23,18,-21},{INT32_MIN,-21,19,-20},{INT32_MIN,-20,20,-18},{INT32_MIN,-18,21,-17},{INT32_MIN,-17,22,-15},{INT32_MIN,-15,23,-14},{INT32_MIN,-14,24,-12},{INT32_MIN,-12,25,-11},{INT32_MIN,-11,26,-9},{INT32_MIN,-9,27,-8},{INT32_MIN,-8,28,-6},{INT32_MIN,-6,29,-5},{INT32_MIN,-5,30,-3},{INT32_MIN,-3,31,-2},{INT32_MIN,-2,32,0},{INT32_MIN,0,33,1},{INT32_MIN,1,34,3},{INT32_MIN,3,35,4},{INT32_MIN,4,36,6},{INT32_MIN,6,37,7},{INT32_MIN,7,38,9},{INT32_MIN,9,39,10},{INT32_MIN,10,40,INT32_MAX},
{0,INT32_MIN,68,-66},{0,-66,67,-64},{0,-64,66,-62},{0,-62,65,-60},{0,-60,64,-58},{0,-58,63,-56},{0,-56,62,-54},{0,-54,61,-52},{0,-52,60,-50},{0,-50,59,-48},{1,-48,58,-47},{2,-47,58,-46},{2,-46,57,-45},{3,-45,57,-44},{4,-44,56,-42},{5,-42,55,-41},{6,-41,55,-40},{6,-40,54,-39},{7,-39,54,-38},{8,-38,53,-36},{9,-36,52,-35},{10,-35,52,-34},{10,-34,51,-33},{11,-33,51,-32},{12,-32,50,-30},{13,-30,49,-29},{14,-29,49,-28},{14,-28,48,-27},{15,-27,48,-26},{16,-26,47,-24},{17,-24,46,-23},{18,-23,46,-22},{18,-22,45,-21},{19,-21,45,-20},{20,-20,44,-18},{21,-18,43,-17},{22,-17,43,-16},{22,-16,42,-15},{23,-15,42,-14},{24,-14,41,-12},{25,-12,40,-11},{26,-11,40,-10},{26,-10,39,-9},{27,-9,39,-8},{28,-8,38,-6},{29,-6,37,-5},{30,-5,37,-4},{30,-4,36,-3},{31,-3,36,-2},{32,-2,35,0},{33,0,34,1},
{68,INT32_MIN,112,-66},{67,-66,112,-64},{66,-64,112,-62},{65,-62,112,-60},{64,-60,111,-59},{64,-59,110,-58},{63,-58,109,-57},{63,-57,108,-56},{62,-56,107,-55},{62,-55,106,-54},{61,-54,105,-53},{61,-53,104,-52},{60,-52,103,-51},{60,-51,102,-50},{59,-50,101,-49},{59,-49,100,-48},{58,-48,99,-47},{58,-47,98,-46},{57,-46,97,-45},{57,-45,96,-44},{56,-44,95,-43},{56,-43,94,-42},{55,-42,93,-41},{55,-41,92,-40},{54,-40,91,-39},{54,-39,90,-38},{53,-38,89,-37},{53,-37,88,-36},{52,-36,87,-35},{52,-35,86,-34},{51,-34,85,-33},{51,-33,84,-32},{50,-32,83,-31},{50,-31,82,-30},{49,-30,81,-29},{49,-29,80,-28},{48,-28,79,-27},{48,-27,78,-26},{47,-26,77,-25},{47,-25,76,-24},{46,-24,75,-23},{46,-23,74,-22},{45,-22,73,-21},{45,-21,72,-20},{44,-20,71,-19},{44,-19,70,-18},{43,-18,69,-17},{43,-17,68,-16},{42,-16,67,-15},{42,-15,66,-14},{41,-14,65,-13},{41,-13,64,-12},{40,-12,63,-11},{40,-11,62,-10},{39,-10,61,-9},{39,-9,60,-8},{38,-8,59,-7},{38,-7,58,-6},{37,-6,57,-5},{37,-5,56,-4},{36,-4,55,-3},{36,-3,54,-2},{35,-2,53,-1},{35,-1,52,0},{34,0,51,1},{34,1,50,2},{34,2,49,3},{35,3,48,4},{36,4,47,5},{36,5,46,6},{37,6,45,7},{38,7,44,8},{38,8,43,9},{39,9,42,10},{40,10,41,11},
{112,INT32_MIN,INT32_MAX,-60},{111,-60,INT32_MAX,-59},{110,-59,INT32_MAX,-58},{109,-58,INT32_MAX,-57},{108,-57,INT32_MAX,-56},{107,-56,INT32_MAX,-55},{106,-55,INT32_MAX,-54},{105,-54,INT32_MAX,-53},{104,-53,INT32_MAX,-52},{103,-52,INT32_MAX,-51},{102,-51,INT32_MAX,-50},{101,-50,INT32_MAX,-49},{100,-49,INT32_MAX,-48},{99,-48,INT32_MAX,-47},{98,-47,INT32_MAX,-46},{97,-46,INT32_MAX,-45},{96,-45,INT32_MAX,-44},{95,-44,INT32_MAX,-43},{94,-43,INT32_MAX,-42},{93,-42,INT32_MAX,-41},{92,-41,INT32_MAX,-40},{91,-40,INT32_MAX,-39},{90,-39,INT32_MAX,-38},{89,-38,INT32_MAX,-37},{88,-37,INT32_MAX,-36},{87,-36,INT32_MAX,-35},{86,-35,INT32_MAX,-34},{85,-34,INT32_MAX,-33},{84,-33,INT32_MAX,-32},{83,-32,INT32_MAX,-31},{82,-31,INT32_MAX,-30},{81,-30,INT32_MAX,-29},{80,-29,INT32_MAX,-28},{79,-28,INT32_MAX,-27},{78,-27,INT32_MAX,-26},{77,-26,INT32_MAX,-25},{76,-25,INT32_MAX,-24},{75,-24,INT32_MAX,-23},{74,-23,INT32_MAX,-22},{73,-22,INT32_MAX,-21},{72,-21,INT32_MAX,-20},{71,-20,INT32_MAX,-19},{70,-19,INT32_MAX,-18},{69,-18,INT32_MAX,-17},{68,-17,INT32_MAX,-16},{67,-16,INT32_MAX,-15},{66,-15,INT32_MAX,-14},{65,-14,INT32_MAX,-13},{64,-13,INT32_MAX,-12},{63,-12,INT32_MAX,-11},{62,-11,INT32_MAX,-10},{61,-10,INT32_MAX,-9},{60,-9,INT32_MAX,-8},{59,-8,INT32_MAX,-7},{58,-7,INT32_MAX,-6},{57,-6,INT32_MAX,-5},{56,-5,INT32_MAX,-4},{55,-4,INT32_MAX,-3},{54,-3,INT32_MAX,-2},{53,-2,INT32_MAX,-1},{52,-1,INT32_MAX,0},{51,0,INT32_MAX,1},{50,1,INT32_MAX,2},{49,2,INT32_MAX,3},{48,3,INT32_MAX,4},{47,4,INT32_MAX,5},{46,5,INT32_MAX,6},{45,6,INT32_MAX,7},{44,7,INT32_MAX,8},{43,8,INT32_MAX,9},{42,9,INT32_MAX,10},{41,10,INT32_MAX,11},{40,11,INT32_MAX,INT32_MAX},
//Third angle
{0,INT32_MIN,INT32_MAX,-65},{-1,-65,INT32_MAX,-62},{-2,-62,INT32_MAX,-59},{-3,-59,INT32_MAX,-56},{-4,-56,INT32_MAX,-53},{-5,-53,INT32_MAX,-50},{-6,-50,INT32_MAX,-47},{-7,-47,INT32_MAX,-44},{-8,-44,INT32_MAX,-41},{-9,-41,INT32_MAX,-38},{-10,-38,INT32_MAX,-35},{-11,-35,INT32_MAX,-32},{-12,-32,INT32_MAX,-29},{-13,-29,INT32_MAX,-26},{-14,-26,INT32_MAX,-23},{-15,-23,INT32_MAX,-20},{-16,-20,INT32_MAX,-17},{-17,-17,INT32_MAX,-14},{-18,-14,INT32_MAX,-11},{-19,-11,INT32_MAX,-8},{-20,-8,INT32_MAX,-5},{-21,-5,INT32_MAX,-2},{-22,-2,INT32_MAX,1},{-23,1,INT32_MAX,4},{-24,4,INT32_MAX,7},{-25,7,INT32_MAX,10},{-26,10,INT32_MAX,13},{-27,13,INT32_MAX,16},{-28,16,INT32_MAX,19},{-29,19,INT32_MAX,22},{-30,22,INT32_MAX,25},{-31,25,INT32_MAX,28},{-32,28,INT32_MAX,31},{-33,31,INT32_MAX,34},{-34,34,INT32_MAX,37},{-35,37,INT32_MAX,40},{-36,40,INT32_MAX,43},{-37,43,INT32_MAX,46},{-38,46,INT32_MAX,INT32_MAX},
{-52,INT32_MIN,0,-78},{-51,-78,0,-72},{-50,-72,0,-66},{-49,-66,0,-65},{-49,-65,-1,-62},{-49,-62,-2,-60},{-48,-60,-2,-59},{-48,-59,-3,-56},{-48,-56,-4,-54},{-47,-54,-4,-53},{-47,-53,-5,-50},{-47,-50,-6,-48},{-46,-48,-6,-47},{-46,-47,-7,-44},{-46,-44,-8,-42},{-45,-42,-8,-41},{-45,-41,-9,-38},{-45,-38,-10,-37},{-44,-37,-10,-35},{-44,-35,-11,-32},{-44,-32,-12,-31},{-43,-31,-12,-29},{-43,-29,-13,-26},{-43,-26,-14,-24},{-42,-24,-14,-23},{-42,-23,-15,-20},{-42,-20,-16,-18},{-41,-18,-16,-17},{-41,-17,-17,-14},{-41,-14,-18,-12},{-40,-12,-18,-11},{-40,-11,-19,-8},{-40,-8,-20,-6},{-39,-6,-20,-5},{-39,-5,-21,-2},{-39,-2,-22,0},{-38,0,-22,1},{-38,1,-23,4},{-38,4,-24,6},{-37,6,-24,7},{-37,7,-25,10},{-37,10,-26,12},{-36,12,-26,13},{-36,13,-27,16},{-36,16,-28,18},{-35,18,-28,19},{-35,19,-29,22},{-35,22,-30,24},{-34,24,-30,25},{-34,25,-31,28},{-34,28,-32,30},{-33,30,-32,31},
{-100,INT32_MIN,-52,-78},{-100,-78,-51,-76},{-99,-76,-51,-74},{-98,-74,-51,-72},{-97,-72,-50,-70},{-96,-70,-50,-68},{-95,-68,-50,-66},{-94,-66,-49,-64},{-93,-64,-49,-62},{-92,-62,-49,-60},{-91,-60,-48,-58},{-90,-58,-48,-56},{-89,-56,-48,-54},{-88,-54,-47,-52},{-87,-52,-47,-50},{-86,-50,-47,-48},{-85,-48,-46,-46},{-84,-46,-46,-44},{-83,-44,-46,-42},{-82,-42,-45,-40},{-81,-40,-45,-38},{-80,-38,-45,-37},{-80,-37,-44,-36},{-79,-36,-44,-34},{-78,-34,-44,-32},{-77,-32,-44,-31},{-77,-31,-43,-30},{-76,-30,-43,-28},{-75,-28,-43,-26},{-74,-26,-43,-24},{-73,-24,-42,-22},{-72,-22,-42,-20},{-71,-20,-42,-18},{-70,-18,-41,-16},{-69,-16,-41,-14},{-68,-14,-41,-12},{-67,-12,-40,-10},{-66,-10,-40,-8},{-65,-8,-40,-6},{-64,-6,-39,-4},{-63,-4,-39,-2},{-62,-2,-39,0},{-61,0,-38,2},{-60,2,-38,4},{-59,4,-38,6},{-58,6,-37,8},{-57,8,-37,10},{-56,10,-37,12},{-55,12,-36,14},{-54,14,-36,16},{-53,16,-36,18},{-52,18,-35,20},{-51,20,-35,22},{-50,22,-35,24},{-49,24,-34,26},{-48,26,-34,28},{-47,28,-34,30},{-46,30,-33,32},{-45,32,-33,34},{-44,34,-34,36},{-43,36,-34,37},{-43,37,-35,38},{-42,38,-35,40},{-41,40,-36,42},{-40,42,-36,43},{-40,43,-37,44},{-39,44,-37,46},
{INT32_MIN,INT32_MIN,-100,-76},{INT32_MIN,-76,-99,-74},{INT32_MIN,-74,-98,-72},{INT32_MIN,-72,-97,-70},{INT32_MIN,-70,-96,-68},{INT32_MIN,-68,-95,-66},{INT32_MIN,-66,-94,-64},{INT32_MIN,-64,-93,-62},{INT32_MIN,-62,-92,-60},{INT32_MIN,-60,-91,-58},{INT32_MIN,-58,-90,-56},{INT32_MIN,-56,-89,-54},{INT32_MIN,-54,-88,-52},{INT32_MIN,-52,-87,-50},{INT32_MIN,-50,-86,-48},{INT32_MIN,-48,-85,-46},{INT32_MIN,-46,-84,-44},{INT32_MIN,-44,-83,-42},{INT32_MIN,-42,-82,-40},{INT32_MIN,-40,-81,-38},{INT32_MIN,-38,-80,-36},{INT32_MIN,-36,-79,-34},{INT32_MIN,-34,-78,-32},{INT32_MIN,-32,-77,-30},{INT32_MIN,-30,-76,-28},{INT32_MIN,-28,-75,-26},{INT32_MIN,-26,-74,-24},{INT32_MIN,-24,-73,-22},{INT32_MIN,-22,-72,-20},{INT32_MIN,-20,-71,-18},{INT32_MIN,-18,-70,-16},{INT32_MIN,-16,-69,-14},{INT32_MIN,-14,-68,-12},{INT32_MIN,-12,-67,-10},{INT32_MIN,-10,-66,-8},{INT32_MIN,-8,-65,-6},{INT32_MIN,-6,-64,-4},{INT32_MIN,-4,-63,-2},{INT32_MIN,-2,-62,0},{INT32_MIN,0,-61,2},{INT32_MIN,2,-60,4},{INT32_MIN,4,-59,6},{INT32_MIN,6,-58,8},{INT32_MIN,8,-57,10},{INT32_MIN,10,-56,12},{INT32_MIN,12,-55,14},{INT32_MIN,14,-54,16},{INT32_MIN,16,-53,18},{INT32_MIN,18,-52,20},{INT32_MIN,20,-51,22},{INT32_MIN,22,-50,24},{INT32_MIN,24,-49,26},{INT32_MIN,26,-48,28},{INT32_MIN,28,-47,30},{INT32_MIN,30,-46,32},{INT32_MIN,32,-45,34},{INT32_MIN,34,-44,36},{INT32_MIN,36,-43,38},{INT32_MIN,38,-42,40},{INT32_MIN,40,-41,42},{INT32_MIN,42,-40,44},{INT32_MIN,44,-39,46},{INT32_MIN,46,-38,INT32_MAX},
//Fourth angle
{17,INT32_MIN,INT32_MAX,-57},{15,-57,INT32_MAX,-56},{13,-56,INT32_MAX,-55},{11,-55,INT32_MAX,-54},{9,-54,INT32_MAX,-53},{7,-53,INT32_MAX,-52},{5,-52,INT32_MAX,-51},{3,-51,INT32_MAX,-50},{1,-50,INT32_MAX,-49},{-1,-49,INT32_MAX,-48},{-3,-48,INT32_MAX,-47},{-5,-47,INT32_MAX,-46},{-7,-46,INT32_MAX,-45},{-9,-45,INT32_MAX,-44},{-11,-44,INT32_MAX,-43},{-13,-43,INT32_MAX,-42},{-15,-42,INT32_MAX,-41},{-17,-41,INT32_MAX,-40},{-19,-40,INT32_MAX,-39},{-21,-39,INT32_MAX,-38},{-23,-38,INT32_MAX,-37},{-25,-37,INT32_MAX,-36},{-27,-36,INT32_MAX,-35},{-29,-35,INT32_MAX,-34},{-31,-34,INT32_MAX,-33},{-33,-33,INT32_MAX,-32},{-35,-32,INT32_MAX,-31},{-37,-31,INT32_MAX,-30},{-39,-30,INT32_MAX,-29},{-41,-29,INT32_MAX,-28},{-43,-28,INT32_MAX,-27},{-45,-27,INT32_MAX,-26},{INT32_MIN,-26,INT32_MAX,INT32_MAX},
{16,-146,17,-145},{15,-145,17,-144},{14,-144,17,-143},{13,-143,17,-142},{12,-142,17,-141},{11,-141,17,-140},{10,-140,17,-139},{9,-139,17,-138},{8,-138,17,-137},{7,-137,17,-136},{6,-136,17,-135},{5,-135,17,-134},{4,-134,17,-133},{3,-133,17,-132},{2,-132,17,-131},{1,-131,17,-130},{0,-130,17,-129},{-1,-129,17,-128},{-2,-128,17,-127},{-3,-127,17,-126},{-4,-126,17,-125},{-5,-125,17,-124},{-6,-124,17,-123},{-7,-123,17,-122},{-8,-122,17,-121},{-9,-121,17,-120},{-10,-120,17,-119},{-11,-119,17,-118},{-12,-118,17,-117},{-13,-117,17,-116},{-14,-116,17,-115},{-15,-115,17,-114},{-16,-114,17,-113},{-17,-113,17,-112},{-18,-112,17,-111},{-19,-111,17,-110},{-20,-110,17,-109},{-21,-109,17,-108},{-22,-108,17,-107},{-23,-107,17,-106},{-24,-106,17,-105},{-25,-105,17,-104},{-26,-104,17,-103},{-27,-103,17,-102},{-28,-102,17,-101},{-29,-101,17,-100},{-30,-100,17,-99},{-31,-99,17,-98},{-32,-98,17,-97},{-33,-97,17,-96},{-34,-96,17,-95},{-35,-95,17,-94},{-36,-94,17,-93},{-37,-93,17,-92},{-38,-92,17,-91},{-39,-91,17,-90},{-40,-90,17,-89},{-41,-89,17,-88},{-42,-88,17,-87},{-43,-87,17,-86},{-44,-86,17,-85},{-45,-85,17,-84},{-46,-84,17,-83},{-48,-83,17,-82},{-50,-82,17,-81},{-52,-81,17,-80},{-54,-80,17,-79},{-56,-79,17,-78},{-58,-78,17,-77},{-60,-77,17,-76},{-62,-76,17,-75},{-64,-75,17,-74},{-66,-74,17,-73},{-68,-73,17,-72},{-70,-72,17,-71},{-72,-71,17,-70},{-74,-70,17,-69},{-76,-69,17,-68},{-78,-68,17,-67},{-80,-67,17,-66},{-82,-66,17,-65},{-84,-65,17,-64},{-86,-64,17,-63},{-88,-63,17,-62},{-90,-62,17,-61},{-92,-61,17,-60},{-94,-60,17,-59},{-96,-59,17,-58},{-98,-58,17,-57},{-99,-57,15,-56},{-101,-56,13,-55},{-103,-55,11,-54},{-105,-54,9,-53},{-107,-53,7,-52},{-109,-52,5,-51},{-111,-51,3,-50},{-113,-50,1,-49},{-115,-49,-1,-48},{-117,-48,-3,-47},{-119,-47,-5,-46},{-121,-46,-7,-45},{-123,-45,-9,-44},{-125,-44,-11,-43},{-127,-43,-13,-42},{-129,-42,-15,-41},{-131,-41,-17,-40},{-133,-40,-19,-39},{-135,-39,-21,-38},{-137,-38,-23,-37},{-139,-37,-25,-36},{-141,-36,-27,-35},{-143,-35,-29,-34},{-145,-34,-31,-33},{-147,-33,-33,-32},{-149,-32,-35,-31},{-151,-31,-37,-30},{-153,-30,-39,-29},{-155,-29,-41,-28},{-157,-28,-43,-27},{-159,-27,-45,-26},
{16,-208,17,-207},{15,-207,17,-206},{14,-206,17,-205},{13,-205,17,-204},{12,-204,17,-203},{11,-203,17,-202},{10,-202,17,-201},{9,-201,17,-200},{8,-200,17,-199},{7,-199,17,-198},{6,-198,17,-197},{5,-197,17,-196},{4,-196,17,-195},{3,-195,17,-194},{2,-194,17,-193},{1,-193,17,-192},{0,-192,17,-191},{-1,-191,17,-190},{-2,-190,17,-189},{-3,-189,17,-188},{-4,-188,17,-187},{-5,-187,17,-186},{-6,-186,17,-185},{-7,-185,17,-184},{-8,-184,17,-183},{-9,-183,17,-182},{-10,-182,17,-181},{-11,-181,17,-180},{-12,-180,17,-179},{-13,-179,17,-178},{-14,-178,17,-177},{-15,-177,17,-176},{-16,-176,17,-175},{-17,-175,17,-174},{-18,-174,17,-173},{-19,-173,17,-172},{-20,-172,17,-171},{-21,-171,17,-170},{-22,-170,17,-169},{-23,-169,17,-168},{-24,-168,17,-167},{-25,-167,17,-166},{-26,-166,17,-165},{-27,-165,17,-164},{-28,-164,17,-163},{-29,-163,17,-162},{-30,-162,17,-161},{-31,-161,17,-160},{-32,-160,17,-159},{-33,-159,17,-158},{-34,-158,17,-157},{-35,-157,17,-156},{-36,-156,17,-155},{-37,-155,17,-154},{-38,-154,17,-153},{-39,-153,17,-152},{-40,-152,17,-151},{-41,-151,17,-150},{-42,-150,17,-149},{-43,-149,17,-148},{-44,-148,17,-147},{-45,-147,17,-146},{-46,-146,16,-145},{-47,-145,15,-144},{-48,-144,14,-143},{-49,-143,13,-142},{-50,-142,12,-141},{-51,-141,11,-140},{-52,-140,10,-139},{-53,-139,9,-138},{-54,-138,8,-137},{-55,-137,7,-136},{-56,-136,6,-135},{-57,-135,5,-134},{-58,-134,4,-133},{-59,-133,3,-132},{-60,-132,2,-131},{-61,-131,1,-130},{-62,-130,0,-129},{-63,-129,-1,-128},{-64,-128,-2,-127},{-65,-127,-3,-126},{-66,-126,-4,-125},{-67,-125,-5,-124},{-68,-124,-6,-123},{-69,-123,-7,-122},{-70,-122,-8,-121},{-71,-121,-9,-120},{-72,-120,-10,-119},{-73,-119,-11,-118},{-74,-118,-12,-117},{-75,-117,-13,-116},{-76,-116,-14,-115},{-77,-115,-15,-114},{-78,-114,-16,-113},{-79,-113,-17,-112},{-80,-112,-18,-111},{-81,-111,-19,-110},{-82,-110,-20,-109},{-83,-109,-21,-108},{-84,-108,-22,-107},{-85,-107,-23,-106},{-86,-106,-24,-105},{-87,-105,-25,-104},{-88,-104,-26,-103},{-89,-103,-27,-102},{-90,-102,-28,-101},{-91,-101,-29,-100},{-92,-100,-30,-99},{-93,-99,-31,-98},{-94,-98,-32,-97},{-95,-97,-33,-96},{-96,-96,-34,-95},{-97,-95,-35,-94},{-98,-94,-36,-93},{-99,-93,-37,-92},{-100,-92,-38,-91},{-101,-91,-39,-90},{-102,-90,-40,-89},{-103,-89,-41,-88},{-104,-88,-42,-87},{-105,-87,-43,-86},{-106,-86,-44,-85},{-107,-85,-45,-84},{-108,-84,-46,-83},{-109,-83,-48,-82},{-110,-82,-50,-81},{-111,-81,-52,-80},{-112,-80,-54,-79},{-113,-79,-56,-78},{-114,-78,-58,-77},{-115,-77,-60,-76},{-116,-76,-62,-75},{-117,-75,-64,-74},{-118,-74,-66,-73},{-119,-73,-68,-72},{-120,-72,-70,-71},{-121,-71,-72,-70},{-122,-70,-74,-69},{-123,-69,-76,-68},{-124,-68,-78,-67},{-125,-67,-80,-66},{-126,-66,-82,-65},{-127,-65,-84,-64},{-128,-64,-86,-63},{-129,-63,-88,-62},{-130,-62,-90,-61},{-131,-61,-92,-60},{-132,-60,-94,-59},{-133,-59,-96,-58},{-134,-58,-98,-57},{-135,-57,-99,-56},{-136,-56,-101,-55},{-137,-55,-103,-54},{-138,-54,-105,-53},{-139,-53,-107,-52},{-140,-52,-109,-51},{-141,-51,-111,-50},{-142,-50,-113,-49},{-143,-49,-115,-48},{-144,-48,-117,-47},{-145,-47,-119,-46},{-146,-46,-121,-45},{-147,-45,-123,-44},{-148,-44,-125,-43},{-149,-43,-127,-42},{-150,-42,-129,-41},{-151,-41,-131,-40},{-152,-40,-133,-39},{-153,-39,-135,-38},{-154,-38,-137,-37},{-155,-37,-139,-36},{-156,-36,-141,-35},{-157,-35,-143,-34},{-158,-34,-145,-33},{-159,-33,-147,-32},{-160,-32,-149,-31},{-161,-31,-151,-30},{-162,-30,-153,-29},{-163,-29,-155,-28},{-164,-28,-157,-27},{-165,-27,-159,-26},
{INT32_MIN,INT32_MIN,17,-208},{INT32_MIN,-208,16,-207},{INT32_MIN,-207,15,-206},{INT32_MIN,-206,14,-205},{INT32_MIN,-205,13,-204},{INT32_MIN,-204,12,-203},{INT32_MIN,-203,11,-202},{INT32_MIN,-202,10,-201},{INT32_MIN,-201,9,-200},{INT32_MIN,-200,8,-199},{INT32_MIN,-199,7,-198},{INT32_MIN,-198,6,-197},{INT32_MIN,-197,5,-196},{INT32_MIN,-196,4,-195},{INT32_MIN,-195,3,-194},{INT32_MIN,-194,2,-193},{INT32_MIN,-193,1,-192},{INT32_MIN,-192,0,-191},{INT32_MIN,-191,-1,-190},{INT32_MIN,-190,-2,-189},{INT32_MIN,-189,-3,-188},{INT32_MIN,-188,-4,-187},{INT32_MIN,-187,-5,-186},{INT32_MIN,-186,-6,-185},{INT32_MIN,-185,-7,-184},{INT32_MIN,-184,-8,-183},{INT32_MIN,-183,-9,-182},{INT32_MIN,-182,-10,-181},{INT32_MIN,-181,-11,-180},{INT32_MIN,-180,-12,-179},{INT32_MIN,-179,-13,-178},{INT32_MIN,-178,-14,-177},{INT32_MIN,-177,-15,-176},{INT32_MIN,-176,-16,-175},{INT32_MIN,-175,-17,-174},{INT32_MIN,-174,-18,-173},{INT32_MIN,-173,-19,-172},{INT32_MIN,-172,-20,-171},{INT32_MIN,-171,-21,-170},{INT32_MIN,-170,-22,-169},{INT32_MIN,-169,-23,-168},{INT32_MIN,-168,-24,-167},{INT32_MIN,-167,-25,-166},{INT32_MIN,-166,-26,-165},{INT32_MIN,-165,-27,-164},{INT32_MIN,-164,-28,-163},{INT32_MIN,-163,-29,-162},{INT32_MIN,-162,-30,-161},{INT32_MIN,-161,-31,-160},{INT32_MIN,-160,-32,-159},{INT32_MIN,-159,-33,-158},{INT32_MIN,-158,-34,-157},{INT32_MIN,-157,-35,-156},{INT32_MIN,-156,-36,-155},{INT32_MIN,-155,-37,-154},{INT32_MIN,-154,-38,-153},{INT32_MIN,-153,-39,-152},{INT32_MIN,-152,-40,-151},{INT32_MIN,-151,-41,-150},{INT32_MIN,-150,-42,-149},{INT32_MIN,-149,-43,-148},{INT32_MIN,-148,-44,-147},{INT32_MIN,-147,-45,-146},{INT32_MIN,-146,-46,-145},{INT32_MIN,-145,-47,-144},{INT32_MIN,-144,-48,-143},{INT32_MIN,-143,-49,-142},{INT32_MIN,-142,-50,-141},{INT32_MIN,-141,-51,-140},{INT32_MIN,-140,-52,-139},{INT32_MIN,-139,-53,-138},{INT32_MIN,-138,-54,-137},{INT32_MIN,-137,-55,-136},{INT32_MIN,-136,-56,-135},{INT32_MIN,-135,-57,-134},{INT32_MIN,-134,-58,-133},{INT32_MIN,-133,-59,-132},{INT32_MIN,-132,-60,-131},{INT32_MIN,-131,-61,-130},{INT32_MIN,-130,-62,-129},{INT32_MIN,-129,-63,-128},{INT32_MIN,-128,-64,-127},{INT32_MIN,-127,-65,-126},{INT32_MIN,-126,-66,-125},{INT32_MIN,-125,-67,-124},{INT32_MIN,-124,-68,-123},{INT32_MIN,-123,-69,-122},{INT32_MIN,-122,-70,-121},{INT32_MIN,-121,-71,-120},{INT32_MIN,-120,-72,-119},{INT32_MIN,-119,-73,-118},{INT32_MIN,-118,-74,-117},{INT32_MIN,-117,-75,-116},{INT32_MIN,-116,-76,-115},{INT32_MIN,-115,-77,-114},{INT32_MIN,-114,-78,-113},{INT32_MIN,-113,-79,-112},{INT32_MIN,-112,-80,-111},{INT32_MIN,-111,-81,-110},{INT32_MIN,-110,-82,-109},{INT32_MIN,-109,-83,-108},{INT32_MIN,-108,-84,-107},{INT32_MIN,-107,-85,-106},{INT32_MIN,-106,-86,-105},{INT32_MIN,-105,-87,-104},{INT32_MIN,-104,-88,-103},{INT32_MIN,-103,-89,-102},{INT32_MIN,-102,-90,-101},{INT32_MIN,-101,-91,-100},{INT32_MIN,-100,-92,-99},{INT32_MIN,-99,-93,-98},{INT32_MIN,-98,-94,-97},{INT32_MIN,-97,-95,-96},{INT32_MIN,-96,-96,-95},{INT32_MIN,-95,-97,-94},{INT32_MIN,-94,-98,-93},{INT32_MIN,-93,-99,-92},{INT32_MIN,-92,-100,-91},{INT32_MIN,-91,-101,-90},{INT32_MIN,-90,-102,-89},{INT32_MIN,-89,-103,-88},{INT32_MIN,-88,-104,-87},{INT32_MIN,-87,-105,-86},{INT32_MIN,-86,-106,-85},{INT32_MIN,-85,-107,-84},{INT32_MIN,-84,-108,-83},{INT32_MIN,-83,-109,-82},{INT32_MIN,-82,-110,-81},{INT32_MIN,-81,-111,-80},{INT32_MIN,-80,-112,-79},{INT32_MIN,-79,-113,-78},{INT32_MIN,-78,-114,-77},{INT32_MIN,-77,-115,-76},{INT32_MIN,-76,-116,-75},{INT32_MIN,-75,-117,-74},{INT32_MIN,-74,-118,-73},{INT32_MIN,-73,-119,-72},{INT32_MIN,-72,-120,-71},{INT32_MIN,-71,-121,-70},{INT32_MIN,-70,-122,-69},{INT32_MIN,-69,-123,-68},{INT32_MIN,-68,-124,-67},{INT32_MIN,-67,-125,-66},{INT32_MIN,-66,-126,-65},{INT32_MIN,-65,-127,-64},{INT32_MIN,-64,-128,-63},{INT32_MIN,-63,-129,-62},{INT32_MIN,-62,-130,-61},{INT32_MIN,-61,-131,-60},{INT32_MIN,-60,-132,-59},{INT32_MIN,-59,-133,-58},{INT32_MIN,-58,-134,-57},{INT32_MIN,-57,-135,-56},{INT32_MIN,-56,-136,-55},{INT32_MIN,-55,-137,-54},{INT32_MIN,-54,-138,-53},{INT32_MIN,-53,-139,-52},{INT32_MIN,-52,-140,-51},{INT32_MIN,-51,-141,-50},{INT32_MIN,-50,-142,-49},{INT32_MIN,-49,-143,-48},{INT32_MIN,-48,-144,-47},{INT32_MIN,-47,-145,-46},{INT32_MIN,-46,-146,-45},{INT32_MIN,-45,-147,-44},{INT32_MIN,-44,-148,-43},{INT32_MIN,-43,-149,-42},{INT32_MIN,-42,-150,-41},{INT32_MIN,-41,-151,-40},{INT32_MIN,-40,-152,-39},{INT32_MIN,-39,-153,-38},{INT32_MIN,-38,-154,-37},{INT32_MIN,-37,-155,-36},{INT32_MIN,-36,-156,-35},{INT32_MIN,-35,-157,-34},{INT32_MIN,-34,-158,-33},{INT32_MIN,-33,-159,-32},{INT32_MIN,-32,-160,-31},{INT32_MIN,-31,-161,-30},{INT32_MIN,-30,-162,-29},{INT32_MIN,-29,-163,-28},{INT32_MIN,-28,-164,-27},{INT32_MIN,-27,-165,-26},
};
mask_t steep_to_flat_up_masks[]={
	{TRACK_MASK_NONE,33,0,0,steep_to_flat_up_rects},{TRACK_MASK_NONE,117,-32,56,steep_to_flat_up_rects+33},{TRACK_MASK_NONE,136,-64,96,steep_to_flat_up_rects+150},{TRACK_MASK_NONE,137,-96,128,steep_to_flat_up_rects+286},
	{TRACK_MASK_NONE,41,0,0,steep_to_flat_up_rects+423},{TRACK_MASK_NONE,51,-32,24,steep_to_flat_up_rects+464},{TRACK_MASK_NONE,75,-64,32,steep_to_flat_up_rects+515},{TRACK_MASK_NONE,73,-96,32,steep_to_flat_up_rects+590},
	{TRACK_MASK_NONE,39,0,0,steep_to_flat_up_rects+663},{TRACK_MASK_NONE,52,32,24,steep_to_flat_up_rects+702},{TRACK_MASK_NONE,67,64,32,steep_to_flat_up_rects+754},{TRACK_MASK_NONE,63,96,32,steep_to_flat_up_rects+821},
	{TRACK_MASK_NONE,33,0,0,steep_to_flat_up_rects+884},{TRACK_MASK_NONE,120,32,56,steep_to_flat_up_rects+917},{TRACK_MASK_NONE,182,64,96,steep_to_flat_up_rects+1037},{TRACK_MASK_NONE,183,96,128,steep_to_flat_up_rects+1219}
};

const track_section_t steep_to_flat_up={TRACK_OFFSET_SPRITE_MASK,steep_to_flat_up_curve,FLAT_TO_STEEP_LENGTH,{{0,4,steep_to_flat_up_masks},{0,4,steep_to_flat_up_masks+4},{0,4,steep_to_flat_up_masks+8},{0,4,steep_to_flat_up_masks+12}}};


rect_t quarter_loop_up_rects[]={
//First angle
{0,INT32_MIN,INT32_MAX,-57},{-2,-57,INT32_MAX,-56},{-4,-56,INT32_MAX,-55},{-6,-55,INT32_MAX,-54},{-8,-54,INT32_MAX,-53},{-10,-53,INT32_MAX,-52},{-12,-52,INT32_MAX,-51},{-14,-51,INT32_MAX,-50},{-16,-50,INT32_MAX,-49},{-18,-49,INT32_MAX,-48},{-20,-48,INT32_MAX,-47},{-22,-47,INT32_MAX,-46},{-24,-46,INT32_MAX,-45},{-26,-45,INT32_MAX,-44},{-28,-44,INT32_MAX,-43},{-30,-43,INT32_MAX,-42},{-32,-42,INT32_MAX,INT32_MAX},
{-32,INT32_MIN,0,-57},{-32,-57,-2,-56},{-32,-56,-4,-55},{-32,-55,-6,-54},{-32,-54,-8,-53},{-32,-53,-10,-52},{-32,-52,-12,-51},{-32,-51,-14,-50},{-32,-50,-16,-49},{-32,-49,-18,-48},{-32,-48,-20,-47},{-32,-47,-22,-46},{-32,-46,-24,-45},{-32,-45,-26,-44},{-32,-44,-28,-43},{-32,-43,-30,-42},
{INT32_MIN,INT32_MIN,-32,INT32_MAX},
//Second angle
{16,-138,INT32_MAX,-92},{14,-92,INT32_MAX,-91},{12,-91,INT32_MAX,-90},{10,-90,INT32_MAX,-89},{8,-89,INT32_MAX,-88},{6,-88,INT32_MAX,-87},{4,-87,INT32_MAX,-86},{2,-86,INT32_MAX,-85},{0,-85,INT32_MAX,-84},{-2,-84,INT32_MAX,-83},{-4,-83,INT32_MAX,-82},{-6,-82,INT32_MAX,-81},{-8,-81,INT32_MAX,-80},{-10,-80,INT32_MAX,-79},{-12,-79,INT32_MAX,-78},{-14,-78,INT32_MAX,-77},{-16,-77,INT32_MAX,-76},{-18,-76,INT32_MAX,-75},{-20,-75,INT32_MAX,-74},{-22,-74,INT32_MAX,-73},{-24,-73,INT32_MAX,-72},{-26,-72,INT32_MAX,-71},{-28,-71,INT32_MAX,-70},{-30,-70,INT32_MAX,-69},{-32,-69,INT32_MAX,INT32_MAX},
{-64,-138,16,-92},{-64,-92,14,-91},{-64,-91,12,-90},{-64,-90,10,-89},{-64,-89,8,-88},{-64,-88,6,-87},{-64,-87,4,-86},{-64,-86,2,-85},{-64,-85,0,-84},{-64,-84,-2,-83},{-64,-83,-4,-82},{-64,-82,-6,-81},{-64,-81,-8,-80},{-64,-80,-10,-79},{-64,-79,-12,-78},{-64,-78,-14,-77},{-64,-77,-16,-76},{-64,-76,-18,-75},{-64,-75,-20,-74},{-64,-74,-22,-73},{-64,-73,-24,-72},{-64,-72,-26,-71},{-64,-71,-28,-70},{-64,-70,-30,-69},{-64,-69,-32,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,-138},{INT32_MIN,-138,-64,INT32_MAX},
//Third angle
{INT32_MIN,-140,-16,-91},{INT32_MIN,-91,-14,-90},{INT32_MIN,-90,-12,-89},{INT32_MIN,-89,-10,-88},{INT32_MIN,-88,-8,-87},{INT32_MIN,-87,-6,-86},{INT32_MIN,-86,-4,-85},{INT32_MIN,-85,-2,-84},{INT32_MIN,-84,0,-83},{INT32_MIN,-83,2,-82},{INT32_MIN,-82,4,-81},{INT32_MIN,-81,6,-80},{INT32_MIN,-80,8,-79},{INT32_MIN,-79,10,-78},{INT32_MIN,-78,12,-77},{INT32_MIN,-77,14,-76},{INT32_MIN,-76,16,-75},{INT32_MIN,-75,18,-74},{INT32_MIN,-74,20,-73},{INT32_MIN,-73,22,-72},{INT32_MIN,-72,24,-71},{INT32_MIN,-71,26,-70},{INT32_MIN,-70,28,-69},{INT32_MIN,-69,30,-68},{INT32_MIN,-68,32,INT32_MAX},
{-16,-140,64,-91},{-14,-91,64,-90},{-12,-90,64,-89},{-10,-89,64,-88},{-8,-88,64,-87},{-6,-87,64,-86},{-4,-86,64,-85},{-2,-85,64,-84},{0,-84,64,-83},{2,-83,64,-82},{4,-82,64,-81},{6,-81,64,-80},{8,-80,64,-79},{10,-79,64,-78},{12,-78,64,-77},{14,-77,64,-76},{16,-76,64,-75},{18,-75,64,-74},{20,-74,64,-73},{22,-73,64,-72},{24,-72,64,-71},{26,-71,64,-70},{28,-70,64,-69},{30,-69,64,-68},{32,-68,64,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,-140},{64,-140,INT32_MAX,INT32_MAX},
//Fourth angle
{INT32_MIN,INT32_MIN,0,-58},{INT32_MIN,-58,2,-57},{INT32_MIN,-57,4,-56},{INT32_MIN,-56,6,-55},{INT32_MIN,-55,8,-54},{INT32_MIN,-54,10,-53},{INT32_MIN,-53,12,-52},{INT32_MIN,-52,14,-51},{INT32_MIN,-51,16,-50},{INT32_MIN,-50,18,-49},{INT32_MIN,-49,20,-48},{INT32_MIN,-48,22,-47},{INT32_MIN,-47,24,-46},{INT32_MIN,-46,26,-45},{INT32_MIN,-45,28,-44},{INT32_MIN,-44,30,-43},{INT32_MIN,-43,32,INT32_MAX},
{0,INT32_MIN,32,-58},{2,-58,32,-57},{4,-57,32,-56},{6,-56,32,-55},{8,-55,32,-54},{10,-54,32,-53},{12,-53,32,-52},{14,-52,32,-51},{16,-51,32,-50},{18,-50,32,-49},{20,-49,32,-48},{22,-48,32,-47},{24,-47,32,-46},{26,-46,32,-45},{28,-45,32,-44},{30,-44,32,-43},
{32,INT32_MIN,INT32_MAX,INT32_MAX},
};
mask_t quarter_loop_up_masks[]={
	{TRACK_MASK_NONE,17,0,0,quarter_loop_up_rects},{TRACK_MASK_NONE,16,32,40,quarter_loop_up_rects+17},{TRACK_MASK_NONE,1,64,64,quarter_loop_up_rects+33},
	{TRACK_MASK_NONE,25,0,0,quarter_loop_up_rects+34},{TRACK_MASK_NONE,25,32,72,quarter_loop_up_rects+59},{TRACK_MASK_NONE,2,64,128,quarter_loop_up_rects+84},
	{TRACK_MASK_NONE,25,0,0,quarter_loop_up_rects+86},{TRACK_MASK_NONE,25,-32,72,quarter_loop_up_rects+111},{TRACK_MASK_NONE,2,-64,128,quarter_loop_up_rects+136},
	{TRACK_MASK_NONE,17,0,0,quarter_loop_up_rects+138},{TRACK_MASK_NONE,16,-32,40,quarter_loop_up_rects+155},{TRACK_MASK_NONE,1,-64,64,quarter_loop_up_rects+171}
};

const track_section_t quarter_loop_up={TRACK_VERTICAL|TRACK_NO_SUPPORTS|TRACK_SPECIAL_QUARTER_LOOP,quarter_loop_up_curve,QUARTER_LOOP_LENGTH,{{0,3,quarter_loop_up_masks},{0,3,quarter_loop_up_masks+3},{0,3,quarter_loop_up_masks+6},{0,3,quarter_loop_up_masks+9}}};



rect_t rect_all={INT32_MIN,INT32_MIN,INT32_MAX,INT32_MAX};

mask_t single_split_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&rect_all},{TRACK_MASK_DIFFERENCE,1,0,0,&rect_all}};
const track_section_t semi_split_left_bank={TRACK_BANK_LEFT,left_bank_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_split_masks},{0,1,NULL},{0,1,NULL}}};

const track_section_t semi_split_left_bank_diag={TRACK_DIAGONAL|TRACK_BANK_LEFT,left_bank_diag_curve,FLAT_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_split_masks},{0,1,NULL},{0,1,NULL},{0,1,NULL}}};



mask_t semi_split_small_turn_left_bank_masks[]={
	{TRACK_MASK_INTERSECT,13,0,0,small_turn_left_rects},{TRACK_MASK_DIFFERENCE,13,0,0,small_turn_left_rects},{0,13,-32,16,small_turn_left_rects+13},{0,26,0,32,small_turn_left_rects+26},
	{TRACK_MASK_INTERSECT,17,0,0,small_turn_left_rects+52},{TRACK_MASK_DIFFERENCE,17,0,0,small_turn_left_rects+52},{0,16,-32,-16,small_turn_left_rects+69},{TRACK_MASK_INTERSECT,17,-64,0,small_turn_left_rects+85},{TRACK_MASK_DIFFERENCE,17,-64,0,small_turn_left_rects+85},
	{0,26,0,0,small_turn_left_rects+102},{0,13,32,-16,small_turn_left_rects+128},{TRACK_MASK_INTERSECT,13,0,-32,small_turn_left_rects+141},{TRACK_MASK_DIFFERENCE,13,0,-32,small_turn_left_rects+141},
	{0,4,0,0,small_turn_left_rects+154},{0,3,32,16,small_turn_left_rects+158},{0,4,64,0,small_turn_left_rects+161}

};
const track_section_t semi_split_small_turn_left_bank={TRACK_BANK_LEFT,small_turn_left_bank_curve,0.75*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,4,semi_split_small_turn_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,5,semi_split_small_turn_left_bank_masks+4},{VIEW_NEEDS_TRACK_MASK,4,semi_split_small_turn_left_bank_masks+9},{0,3,semi_split_small_turn_left_bank_masks+13}}};


mask_t semi_split_medium_turn_left_bank_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects},{0,49,-32,16,medium_turn_left_rects+33},{0,31,0,32,medium_turn_left_rects+82},{0,32,-32,48,medium_turn_left_rects+113},{0,32,0,64,medium_turn_left_rects+145},//TODO proper splitting
{TRACK_MASK_INTERSECT,17,0,0,medium_turn_left_rects+177},{TRACK_MASK_DIFFERENCE,17,0,0,medium_turn_left_rects+177},{0,16,-32,-16,medium_turn_left_rects+194},{TRACK_MASK_INTERSECT,16,-64,0,medium_turn_left_rects+210},{TRACK_MASK_DIFFERENCE,16,-64,0,medium_turn_left_rects+210},{0,16,-96,-16,medium_turn_left_rects+226},{TRACK_MASK_INTERSECT,17,-128,0,medium_turn_left_rects+242},{TRACK_MASK_DIFFERENCE,17,-128,0,medium_turn_left_rects+242},
{0,33,0,0,medium_turn_left_rects+259},{0,32,32,-16,medium_turn_left_rects+292},{0,31,0,-32,medium_turn_left_rects+324},{0,49,32,-48,medium_turn_left_rects+355},{TRACK_MASK_INTERSECT,32,0,-64,medium_turn_left_rects+404},{TRACK_MASK_DIFFERENCE,32,0,-64,medium_turn_left_rects+404},//TODO proper splitting
{0,33,0,0,medium_turn_left_rects+436},{0,31,32,16,medium_turn_left_rects+469},{0,32,64,0,medium_turn_left_rects+500},{0,31,96,16,medium_turn_left_rects+532},{0,33,128,0,medium_turn_left_rects+563}
};
const track_section_t semi_split_medium_turn_left_bank={TRACK_BANK_LEFT,medium_turn_left_bank_curve,1.25*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,6,semi_split_medium_turn_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,8,semi_split_medium_turn_left_bank_masks+6},{VIEW_NEEDS_TRACK_MASK,6,semi_split_medium_turn_left_bank_masks+14},{0,5,semi_split_medium_turn_left_bank_masks+20}}};


rect_t semi_split_large_turn_left_to_diag_bank_alternate_rects[]={
{32,INT32_MIN,64,16},{30,16,64,17},{28,17,64,18},{26,18,64,19},{24,19,64,20},{22,20,64,21},{20,21,64,22},{18,22,64,23},{16,23,64,24},{14,24,64,25},{12,25,64,26},{10,26,64,27},{8,27,64,28},{6,28,64,29},{4,29,64,30},{2,30,64,31},{0,31,64,32},{2,32,64,33},{4,33,64,34},{6,34,64,35},{8,35,64,36},{10,36,64,37},{12,37,64,38},{14,38,64,39},{16,39,64,40},{18,40,64,41},{20,41,64,42},{22,42,64,43},{24,43,64,44},{26,44,64,45},{28,45,64,46},{30,46,64,47},{32,47,64,INT32_MAX}
};

mask_t semi_split_large_turn_left_to_diag_bank_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,large_turn_left_to_diag_bank_rects},{TRACK_MASK_DIFFERENCE,33,0,0,large_turn_left_to_diag_bank_rects},{0,47,-32,16,large_turn_left_to_diag_bank_rects+33},{0,31,0,32,large_turn_left_to_diag_bank_rects+80},{0,31*2,-32,48,large_turn_left_to_diag_bank_rects+111-31},
{TRACK_MASK_INTERSECT,33,0,0,large_turn_left_to_diag_bank_rects+142},{TRACK_MASK_DIFFERENCE,33,0,0,large_turn_left_to_diag_bank_rects+142},{TRACK_MASK_INTERSECT,33,-32,-16,semi_split_large_turn_left_to_diag_bank_alternate_rects},{TRACK_MASK_DIFFERENCE,17,-32,-16,large_turn_left_to_diag_bank_rects+175},{0,1,-64,0,large_turn_left_to_diag_bank_rects+192},{TRACK_MASK_INTERSECT,1,-96,-16,large_turn_left_to_diag_bank_rects+193},{TRACK_MASK_DIFFERENCE,1,-96,-16,large_turn_left_to_diag_bank_rects+193},
{0,16,0,0,large_turn_left_to_diag_bank_rects+194},{0,32,32,-16,large_turn_left_to_diag_bank_rects+210},{0,33,0,-32,large_turn_left_to_diag_bank_rects+242},{0,33,32,-48,large_turn_left_to_diag_bank_rects+275},
{0,17,0,0,large_turn_left_to_diag_bank_rects+308},{0,16,32,16,large_turn_left_to_diag_bank_rects+325},{0,16,64,0,large_turn_left_to_diag_bank_rects+341},{0,17,96,16,large_turn_left_to_diag_bank_rects+357},
};
const track_section_t semi_split_large_turn_left_to_diag_bank={TRACK_BANK_LEFT,large_turn_left_to_diag_bank_curve,0.875*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,5,semi_split_large_turn_left_to_diag_bank_masks},{VIEW_NEEDS_TRACK_MASK,7,semi_split_large_turn_left_to_diag_bank_masks+5},{0,4,semi_split_large_turn_left_to_diag_bank_masks+12},{0,4,semi_split_large_turn_left_to_diag_bank_masks+16}}};


rect_t semi_split_large_turn_right_to_diag_bank_alternate_rects[]={
{-64,INT32_MIN,-32,16},{-64,16,-30,17},{-64,17,-28,18},{-64,18,-26,19},{-64,19,-24,20},{-64,20,-22,21},{-64,21,-20,22},{-64,22,-18,23},{-64,23,-16,24},{-64,24,-14,25},{-64,25,-12,26},{-64,26,-10,27},{-64,27,-8,28},{-64,28,-6,29},{-64,29,-4,30},{-64,30,-2,31},{-64,31,0,INT32_MAX},
};

mask_t semi_split_large_turn_right_to_diag_bank_masks[]={
{0,17,0,0,large_turn_right_to_diag_bank_rects},{0,16,-32,16,large_turn_right_to_diag_bank_rects+17},{0,16,-64,0,large_turn_right_to_diag_bank_rects+33},{0,17,-96,16,large_turn_right_to_diag_bank_rects+49},
{0,16,0,0,large_turn_right_to_diag_bank_rects+66},{0,32,-32,-16,large_turn_right_to_diag_bank_rects+82},{0,34,0,-32,large_turn_right_to_diag_bank_rects+114},{0,33,-32,-48,large_turn_right_to_diag_bank_rects+148},
{TRACK_MASK_INTERSECT,17,0,0,large_turn_right_to_diag_bank_rects+181},{TRACK_MASK_DIFFERENCE,17,0,0,large_turn_right_to_diag_bank_rects+181},{TRACK_MASK_INTERSECT,17,32,-16,semi_split_large_turn_right_to_diag_bank_alternate_rects},{TRACK_MASK_DIFFERENCE,1,32,-16,large_turn_right_to_diag_bank_rects+198},{0,1,64,0,large_turn_right_to_diag_bank_rects+199},{TRACK_MASK_INTERSECT,1,96,-16,large_turn_right_to_diag_bank_rects+200},{TRACK_MASK_DIFFERENCE,1,96,-16,large_turn_right_to_diag_bank_rects+200},
{TRACK_MASK_INTERSECT,27,0,0,large_turn_right_to_diag_bank_rects+201},{TRACK_MASK_DIFFERENCE,27,0,0,large_turn_right_to_diag_bank_rects+201},{0,41,32,16,large_turn_right_to_diag_bank_rects+228},{0,31,0,32,large_turn_right_to_diag_bank_rects+269},{0,31,32,48,large_turn_right_to_diag_bank_rects+300},
};
const track_section_t semi_split_large_turn_right_to_diag_bank={TRACK_BANK_RIGHT,large_turn_right_to_diag_bank_curve,0.875*TILE_SIZE*3.1415926,{{0,4,semi_split_large_turn_right_to_diag_bank_masks},{0,4,semi_split_large_turn_right_to_diag_bank_masks+4},{VIEW_NEEDS_TRACK_MASK,7,semi_split_large_turn_right_to_diag_bank_masks+8},{VIEW_NEEDS_TRACK_MASK,5,semi_split_large_turn_right_to_diag_bank_masks+15}}};




const track_section_t semi_split_left_bank_to_gentle_up_left_bank={TRACK_BANK_LEFT,left_bank_to_gentle_up_left_bank_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_split_masks},{0,1,NULL},{0,1,NULL}}};
const track_section_t semi_split_right_bank_to_gentle_up_right_bank={TRACK_BANK_RIGHT,right_bank_to_gentle_up_right_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,single_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_split_masks}}};

rect_t gentle_up_left_bank_to_left_bank_rects[]={
{INT32_MIN,8,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,8},
{16,INT32_MIN,INT32_MAX,INT32_MAX},{0,8,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,0,INT32_MAX},{0,INT32_MIN,16,8}
};
mask_t gentle_up_left_bank_to_left_bank_masks[]={{TRACK_MASK_UNION,1,0,0,gentle_up_left_bank_to_left_bank_rects+0},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_left_bank_to_left_bank_rects+1},{TRACK_MASK_UNION,2,0,0,gentle_up_left_bank_to_left_bank_rects+2},{TRACK_MASK_DIFFERENCE,2,0,0,gentle_up_left_bank_to_left_bank_rects+4}};
const track_section_t semi_split_gentle_up_left_bank_to_left_bank={TRACK_BANK_LEFT,gentle_up_left_bank_to_left_bank_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,gentle_up_left_bank_to_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_left_bank_to_left_bank_masks+2},{0,1,NULL},{0,1,NULL}}};
rect_t gentle_up_right_bank_to_right_bank_rects[]={
{INT32_MIN,INT32_MIN,-16,INT32_MAX},{INT32_MIN,8,0,INT32_MAX},
{0,INT32_MIN,INT32_MAX,INT32_MAX},{-16,INT32_MIN,INT32_MAX,8},
{INT32_MIN,8,INT32_MAX,INT32_MAX},
{INT32_MIN,INT32_MIN,INT32_MAX,8}
};
mask_t gentle_up_right_bank_to_right_bank_masks[]={{TRACK_MASK_UNION,2,0,0,gentle_up_right_bank_to_right_bank_rects+0},{TRACK_MASK_DIFFERENCE,2,0,0,gentle_up_right_bank_to_right_bank_rects+2},{TRACK_MASK_UNION,1,0,0,gentle_up_right_bank_to_right_bank_rects+4},{TRACK_MASK_DIFFERENCE,1,0,0,gentle_up_right_bank_to_right_bank_rects+5}};
const track_section_t semi_split_gentle_up_right_bank_to_right_bank={TRACK_BANK_RIGHT,gentle_up_right_bank_to_right_bank_curve,FLAT_TO_GENTLE_LENGTH,{{0,1,NULL},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_right_bank_to_right_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_right_bank_to_right_bank_masks+2}}};

mask_t small_flat_to_steep_up_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t small_flat_to_steep_up={0,small_flat_to_steep_up_curve,SMALL_FLAT_TO_STEEP_LENGTH,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,small_flat_to_steep_up_masks},{VIEW_NEEDS_TRACK_MASK,2,small_flat_to_steep_up_masks},{0,1,NULL}}};
const track_section_t small_steep_to_flat_up={0,small_steep_to_flat_up_curve,SMALL_FLAT_TO_STEEP_LENGTH,{{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,small_flat_to_steep_up_masks},{VIEW_NEEDS_TRACK_MASK,2,small_flat_to_steep_up_masks},{0,1,NULL}}};

mask_t semi_split_small_turn_left_bank_gentle_up_masks[]={
{0,1,-6,-3,small_turn_left_gentle_up_rects},{0,1,6,45,small_turn_left_gentle_up_rects+1},
{TRACK_MASK_INTERSECT,1,6,-3,small_turn_left_gentle_up_rects+2},{TRACK_MASK_DIFFERENCE,1,6,-3,small_turn_left_gentle_up_rects+2},{TRACK_MASK_INTERSECT,1,-70,13,small_turn_left_gentle_up_rects+3},{TRACK_MASK_DIFFERENCE,1,-70,13,small_turn_left_gentle_up_rects+3},
{0,1,-6,-3,small_turn_left_gentle_up_rects+4},{TRACK_MASK_INTERSECT,1,6,-19,small_turn_left_gentle_up_rects+5},{TRACK_MASK_DIFFERENCE,1,6,-19,small_turn_left_gentle_up_rects+5},
{0,1,6,-3,small_turn_left_gentle_up_rects+6},{0,1,58,13,small_turn_left_gentle_up_rects+7}
};
const track_section_t semi_split_small_turn_left_bank_gentle_up={TRACK_BANK_LEFT|TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,small_turn_left_bank_gentle_up_curve,SMALL_TURN_GENTLE_LENGTH,{{0,2,semi_split_small_turn_left_bank_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,4,semi_split_small_turn_left_bank_gentle_up_masks+2},{VIEW_NEEDS_TRACK_MASK,3,semi_split_small_turn_left_bank_gentle_up_masks+6},{0,2,semi_split_small_turn_left_bank_gentle_up_masks+9}}};
mask_t semi_split_small_turn_right_bank_gentle_up_masks[]={
{0,1,-6,-3,small_turn_right_gentle_up_rects},{0,1,-58,13,small_turn_right_gentle_up_rects+1},
{0,1,6,-3,small_turn_right_gentle_up_rects+2},{TRACK_MASK_INTERSECT,1,-6,-19,small_turn_right_gentle_up_rects+3},{TRACK_MASK_DIFFERENCE,1,-6,-19,small_turn_right_gentle_up_rects+3},
{TRACK_MASK_INTERSECT,1,-6,-3,small_turn_right_gentle_up_rects+4},{TRACK_MASK_DIFFERENCE,1,-6,-3,small_turn_right_gentle_up_rects+4},{TRACK_MASK_INTERSECT,1,70,13,small_turn_right_gentle_up_rects+5},{TRACK_MASK_DIFFERENCE,1,70,13,small_turn_right_gentle_up_rects+5},
{0,1,6,-3,small_turn_right_gentle_up_rects+6},{0,1,-6,45,small_turn_right_gentle_up_rects+7}
};
const track_section_t semi_split_small_turn_right_bank_gentle_up={TRACK_BANK_RIGHT|TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,small_turn_right_bank_gentle_up_curve,SMALL_TURN_GENTLE_LENGTH,{{0,2,semi_split_small_turn_right_bank_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,3,semi_split_small_turn_right_bank_gentle_up_masks+2},{VIEW_NEEDS_TRACK_MASK,4,semi_split_small_turn_right_bank_gentle_up_masks+5},{0,2,semi_split_small_turn_right_bank_gentle_up_masks+9}}};
mask_t semi_split_medium_turn_left_bank_gentle_up_masks[]={
{0,2,0,0,medium_turn_left_gentle_up_rects},{0,2,-32,32,medium_turn_left_gentle_up_rects+2},{0,1,0,0,medium_turn_left_gentle_up_rects+4},{0,2,-32,80,medium_turn_left_gentle_up_rects+5},{0,2,0,112,medium_turn_left_gentle_up_rects+7},
{TRACK_MASK_INTERSECT,1,0,0,medium_turn_left_gentle_up_rects+9},{TRACK_MASK_DIFFERENCE,1,0,0,medium_turn_left_gentle_up_rects+9},{TRACK_MASK_INTERSECT,1,-32,0,medium_turn_left_gentle_up_rects+10},{TRACK_MASK_DIFFERENCE,1,-32,0,medium_turn_left_gentle_up_rects+10},{TRACK_MASK_INTERSECT,32,-64,24,medium_turn_left_gentle_up_rects+11},{TRACK_MASK_DIFFERENCE,32,-64,24,medium_turn_left_gentle_up_rects+11},{TRACK_MASK_INTERSECT,76,-96,16,medium_turn_left_gentle_up_rects+43},{TRACK_MASK_DIFFERENCE,76,-96,16,medium_turn_left_gentle_up_rects+43},{TRACK_MASK_INTERSECT,65,-128,48,medium_turn_left_gentle_up_rects+119},{TRACK_MASK_DIFFERENCE,65,-128,48,medium_turn_left_gentle_up_rects+119},
{0,1,0,0,medium_turn_left_gentle_up_rects+184},{0,1,0,0,medium_turn_left_gentle_up_rects+185},{0,1,0,-8,medium_turn_left_gentle_up_rects+186},{0,1,0,0,medium_turn_left_gentle_up_rects+187},{TRACK_MASK_INTERSECT,2,0,-16,medium_turn_left_gentle_up_rects+188},{TRACK_MASK_DIFFERENCE,2,0,-16,medium_turn_left_gentle_up_rects+188},
{0,33,0,0,medium_turn_left_gentle_up_rects+190},{0,36,32,32,medium_turn_left_gentle_up_rects+223},{0,43,64,24,medium_turn_left_gentle_up_rects+259},{0,53,96,48,medium_turn_left_gentle_up_rects+302},{0,45,128,48,medium_turn_left_gentle_up_rects+355}
};
const track_section_t semi_split_medium_turn_left_bank_gentle_up={TRACK_BANK_LEFT|TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,medium_turn_left_bank_gentle_up_curve,MEDIUM_TURN_GENTLE_LENGTH,{{0,5,semi_split_medium_turn_left_bank_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,10,semi_split_medium_turn_left_bank_gentle_up_masks+5},{VIEW_NEEDS_TRACK_MASK,6,semi_split_medium_turn_left_bank_gentle_up_masks+15},{0,5,semi_split_medium_turn_left_bank_gentle_up_masks+21}}};
mask_t semi_split_medium_turn_right_bank_gentle_up_masks[]={
{0,33,0,0,medium_turn_right_gentle_up_rects},{0,36,-32,32,medium_turn_right_gentle_up_rects+33},{0,43,-64,24,medium_turn_right_gentle_up_rects+69},{0,53,-96,48,medium_turn_right_gentle_up_rects+112},{0,45,-128,48,medium_turn_right_gentle_up_rects+165},
{0,1,0,0,medium_turn_right_gentle_up_rects+210},{0,1,-32,0,medium_turn_right_gentle_up_rects+211},{0,1,0,-8,medium_turn_right_gentle_up_rects+212},{0,1,-32,-8,medium_turn_right_gentle_up_rects+213},{TRACK_MASK_INTERSECT,2,0,-16,medium_turn_right_gentle_up_rects+214},{TRACK_MASK_DIFFERENCE,2,0,-16,medium_turn_right_gentle_up_rects+214},
{TRACK_MASK_INTERSECT,1,0,0,medium_turn_right_gentle_up_rects+216},{TRACK_MASK_DIFFERENCE,1,0,0,medium_turn_right_gentle_up_rects+216},{TRACK_MASK_INTERSECT,1,32,0,medium_turn_right_gentle_up_rects+217},{TRACK_MASK_DIFFERENCE,1,32,0,medium_turn_right_gentle_up_rects+217},{TRACK_MASK_INTERSECT,21,64,24,medium_turn_right_gentle_up_rects+218},{TRACK_MASK_DIFFERENCE,21,64,24,medium_turn_right_gentle_up_rects+218},{TRACK_MASK_INTERSECT,53,96,16,medium_turn_right_gentle_up_rects+239},{TRACK_MASK_DIFFERENCE,53,96,16,medium_turn_right_gentle_up_rects+239},{TRACK_MASK_INTERSECT,40,128,48,medium_turn_right_gentle_up_rects+292},{TRACK_MASK_DIFFERENCE,40,128,48,medium_turn_right_gentle_up_rects+292},
{0,2,0,0,medium_turn_right_gentle_up_rects+332},{0,2,32,32,medium_turn_right_gentle_up_rects+334},{0,1,0,56,medium_turn_right_gentle_up_rects+336},{0,2,32,80,medium_turn_right_gentle_up_rects+337},{0,2,0,112,medium_turn_right_gentle_up_rects+339}
};
const track_section_t semi_split_medium_turn_right_bank_gentle_up={TRACK_BANK_RIGHT|TRACK_OFFSET_SPRITE_MASK|TRACK_SUPPORT_BASE,medium_turn_right_bank_gentle_up_curve,MEDIUM_TURN_GENTLE_LENGTH,{{0,5,semi_split_medium_turn_right_bank_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,6,semi_split_medium_turn_right_bank_gentle_up_masks+5},{VIEW_NEEDS_TRACK_MASK,10,semi_split_medium_turn_right_bank_gentle_up_masks+11},{0,5,semi_split_medium_turn_right_bank_gentle_up_masks+21}}};
mask_t semi_split_small_helix_left_up_masks[]={
	{TRACK_MASK_INTERSECT,13,0,0,small_turn_left_rects},{TRACK_MASK_DIFFERENCE,13,0,0,small_turn_left_rects},{0,13,-32,16,small_turn_left_rects+13},{0,26,0,32,small_turn_left_rects+26},
	{TRACK_MASK_INTERSECT,17,0,0,small_turn_left_rects+52},{TRACK_MASK_DIFFERENCE,17,0,0,small_turn_left_rects+52},{0,16,-32,-16,small_turn_left_rects+69},{TRACK_MASK_INTERSECT,17,-64,0,small_turn_left_rects+85},{TRACK_MASK_DIFFERENCE,17,-64,0,small_turn_left_rects+85},
	{0,26,0,0,small_turn_left_rects+102},{0,13,32,-16,small_turn_left_rects+128},{TRACK_MASK_INTERSECT,13,0,-32,small_turn_left_rects+141},{TRACK_MASK_DIFFERENCE,13,0,-32,small_turn_left_rects+141},
	{0,4,0,0,small_turn_left_rects+154},{0,3,32,16,small_turn_left_rects+158},{0,4,64,0,small_turn_left_rects+161}

};
const track_section_t semi_split_small_helix_left_up={TRACK_BANK_LEFT|TRACK_SUPPORT_BASE,small_helix_left_up_curve,SMALL_HELIX_LENGTH,{{VIEW_NEEDS_TRACK_MASK,4,semi_split_small_helix_left_up_masks},{VIEW_NEEDS_TRACK_MASK,5,semi_split_small_helix_left_up_masks+4},{VIEW_NEEDS_TRACK_MASK,4,semi_split_small_helix_left_up_masks+9},{0,3,semi_split_small_helix_left_up_masks+13}}};
mask_t semi_split_small_helix_right_up_masks[]={
	{0,4,0,0,small_turn_right_rects},{0,3,-32,16,small_turn_right_rects+4},{0,4,-64,0,small_turn_right_rects+7},
	{0,19,0,0,small_turn_right_rects+11},{0,10,-32,-16,small_turn_right_rects+30},{TRACK_MASK_INTERSECT,9,0,-32,small_turn_right_rects+40},{TRACK_MASK_DIFFERENCE,9,0,-32,small_turn_right_rects+40},
	{TRACK_MASK_INTERSECT,9,0,0,small_turn_right_rects+49},{TRACK_MASK_DIFFERENCE,9,0,0,small_turn_right_rects+49},{0,8,32,-16,small_turn_right_rects+58},{TRACK_MASK_INTERSECT,9,64,0,small_turn_right_rects+66},{TRACK_MASK_DIFFERENCE,9,64,0,small_turn_right_rects+66},
	{TRACK_MASK_INTERSECT,10,0,0,small_turn_right_rects+75},{TRACK_MASK_DIFFERENCE,10,0,0,small_turn_right_rects+75},{0,10,32,16,small_turn_right_rects+85},{0,20,0,32,small_turn_right_rects+95}
};
const track_section_t semi_split_small_helix_right_up={TRACK_BANK_RIGHT|TRACK_SUPPORT_BASE,small_helix_right_up_curve,SMALL_HELIX_LENGTH,{{0,3,semi_split_small_helix_right_up_masks},{VIEW_NEEDS_TRACK_MASK,4,semi_split_small_helix_right_up_masks+3},{VIEW_NEEDS_TRACK_MASK,5,semi_split_small_helix_right_up_masks+7},{VIEW_NEEDS_TRACK_MASK,4,semi_split_small_helix_right_up_masks+12}}};
mask_t semi_split_medium_helix_left_up_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects},{0,49,-32,16,medium_turn_left_rects+33},{0,31,0,32,medium_turn_left_rects+82},{0,32,-32,48,medium_turn_left_rects+113},{0,32,0,64,medium_turn_left_rects+145},
{TRACK_MASK_INTERSECT,17,0,0,medium_turn_left_rects+177},{TRACK_MASK_DIFFERENCE,17,0,0,medium_turn_left_rects+177},{0,16,-32,-16,medium_turn_left_rects+194},{TRACK_MASK_INTERSECT,16,-64,0,medium_turn_left_rects+210},{TRACK_MASK_DIFFERENCE,16,-64,0,medium_turn_left_rects+210},{0,16,-96,-16,medium_turn_left_rects+226},{TRACK_MASK_INTERSECT,17,-128,0,medium_turn_left_rects+242},{TRACK_MASK_DIFFERENCE,17,-128,0,medium_turn_left_rects+242},
{0,33,0,0,medium_turn_left_rects+259},{0,32,32,-16,medium_turn_left_rects+292},{0,31,0,-32,medium_turn_left_rects+324},{0,49,32,-48,medium_turn_left_rects+355},{TRACK_MASK_INTERSECT,32,0,-64,medium_turn_left_rects+404},{TRACK_MASK_DIFFERENCE,32,0,-64,medium_turn_left_rects+404},
{0,33,0,0,medium_turn_left_rects+436},{0,31,32,16,medium_turn_left_rects+469},{0,32,64,0,medium_turn_left_rects+500},{0,31,96,16,medium_turn_left_rects+532},{0,33,128,0,medium_turn_left_rects+563}
};
const track_section_t semi_split_medium_helix_left_up={TRACK_BANK_LEFT|TRACK_SUPPORT_BASE,medium_helix_left_up_curve,MEDIUM_HELIX_LENGTH,{{VIEW_NEEDS_TRACK_MASK,6,semi_split_medium_helix_left_up_masks},{VIEW_NEEDS_TRACK_MASK,8,semi_split_medium_helix_left_up_masks+6},{VIEW_NEEDS_TRACK_MASK,6,semi_split_medium_helix_left_up_masks+14},{0,5,semi_split_medium_helix_left_up_masks+20}}};
mask_t semi_split_medium_helix_right_up_masks[]={
{0,33,0,0,medium_turn_right_rects},{0,31,-32,16,medium_turn_right_rects+33},{0,32,-64,0,medium_turn_right_rects+64},{0,31,-96,16,medium_turn_right_rects+96},{0,33,-128,0,medium_turn_right_rects+127},
{0,32,0,0,medium_turn_right_rects+160},{0,32,-32,-16,medium_turn_right_rects+192},{0,31,0,-32,medium_turn_right_rects+224},{0,49,-32,-48,medium_turn_right_rects+255},{TRACK_MASK_INTERSECT,33,0,-64,medium_turn_right_rects+304},{TRACK_MASK_DIFFERENCE,33,0,-64,medium_turn_right_rects+304},
{TRACK_MASK_INTERSECT,17,0,0,medium_turn_right_rects+337},{TRACK_MASK_DIFFERENCE,17,0,0,medium_turn_right_rects+337},{0,16,32,-16,medium_turn_right_rects+354},{TRACK_MASK_INTERSECT,16,64,0,medium_turn_right_rects+370},{TRACK_MASK_DIFFERENCE,16,64,0,medium_turn_right_rects+370},{0,16,96,-16,medium_turn_right_rects+386},{TRACK_MASK_INTERSECT,17,128,0,medium_turn_right_rects+402},{TRACK_MASK_DIFFERENCE,17,128,0,medium_turn_right_rects+402},
{TRACK_MASK_INTERSECT,32,0,0,medium_turn_right_rects+419},{TRACK_MASK_DIFFERENCE,32,0,0,medium_turn_right_rects+419},{0,49,32,16,medium_turn_right_rects+451},{0,31,0,32,medium_turn_right_rects+500},{0,32,32,48,medium_turn_right_rects+531},{0,33,0,64,medium_turn_right_rects+563}
};
const track_section_t semi_split_medium_helix_right_up={TRACK_BANK_RIGHT|TRACK_SUPPORT_BASE,medium_helix_right_up_curve,MEDIUM_HELIX_LENGTH,{{0,5,semi_split_medium_helix_right_up_masks},{VIEW_NEEDS_TRACK_MASK,6,semi_split_medium_helix_right_up_masks+5},{VIEW_NEEDS_TRACK_MASK,8,semi_split_medium_helix_right_up_masks+11},{VIEW_NEEDS_TRACK_MASK,6,semi_split_medium_helix_right_up_masks+19}}};
mask_t semi_split_quarter_loop_up_masks[]={
	{TRACK_MASK_INTERSECT,17,0,0,quarter_loop_up_rects},{TRACK_MASK_DIFFERENCE,17,0,0,quarter_loop_up_rects},{TRACK_MASK_INTERSECT,16,32,40,quarter_loop_up_rects+17},{TRACK_MASK_DIFFERENCE,16,32,40,quarter_loop_up_rects+17},{TRACK_MASK_INTERSECT,1,64,64,quarter_loop_up_rects+33},{TRACK_MASK_DIFFERENCE,1,64,64,quarter_loop_up_rects+33},
	{TRACK_MASK_INTERSECT,25,0,0,quarter_loop_up_rects+34},{TRACK_MASK_DIFFERENCE,25,0,0,quarter_loop_up_rects+34},{TRACK_MASK_INTERSECT,25,32,72,quarter_loop_up_rects+59},{TRACK_MASK_DIFFERENCE,25,32,72,quarter_loop_up_rects+59},{TRACK_MASK_INTERSECT,2,64,128,quarter_loop_up_rects+84},{TRACK_MASK_DIFFERENCE,2,64,128,quarter_loop_up_rects+84},
	{TRACK_MASK_INTERSECT,25,0,0,quarter_loop_up_rects+86},{TRACK_MASK_DIFFERENCE,25,0,0,quarter_loop_up_rects+86},{TRACK_MASK_INTERSECT,25,-32,72,quarter_loop_up_rects+111},{TRACK_MASK_DIFFERENCE,25,-32,72,quarter_loop_up_rects+111},{TRACK_MASK_INTERSECT,2,-64,128,quarter_loop_up_rects+136},{TRACK_MASK_DIFFERENCE,2,-64,128,quarter_loop_up_rects+136},
	{TRACK_MASK_INTERSECT,17,0,0,quarter_loop_up_rects+138},{TRACK_MASK_DIFFERENCE,17,0,0,quarter_loop_up_rects+138},{TRACK_MASK_INTERSECT,16,-32,40,quarter_loop_up_rects+155},{TRACK_MASK_DIFFERENCE,16,-32,40,quarter_loop_up_rects+155},{TRACK_MASK_INTERSECT,1,-64,64,quarter_loop_up_rects+171},{TRACK_MASK_DIFFERENCE,1,-64,64,quarter_loop_up_rects+171}
};
const track_section_t semi_split_quarter_loop_up={TRACK_VERTICAL|TRACK_NO_SUPPORTS|TRACK_SPECIAL_QUARTER_LOOP,quarter_loop_up_curve,QUARTER_LOOP_LENGTH,{{VIEW_NEEDS_TRACK_MASK,6,semi_split_quarter_loop_up_masks},{VIEW_NEEDS_TRACK_MASK,6,semi_split_quarter_loop_up_masks+6},{VIEW_NEEDS_TRACK_MASK,6,semi_split_quarter_loop_up_masks+12},{VIEW_NEEDS_TRACK_MASK,6,semi_split_quarter_loop_up_masks+18}}};


mask_t single_tile_split_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all}};
const track_section_t split_flat={0,flat_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{0,0,NULL},{0,0,NULL}}};
const track_section_t split_flat_asymmetric={0,flat_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_brake={TRACK_SPECIAL_BRAKE,flat_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{0,0,NULL},{0,0,NULL}}};
const track_section_t split_block_brake={TRACK_SPECIAL_BLOCK_BRAKE,flat_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{0,0,NULL},{0,0,NULL}}};
const track_section_t split_booster={TRACK_SPECIAL_BOOSTER,flat_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_launched_lift={TRACK_SPECIAL_LAUNCHED_LIFT,gentle_curve,GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_flat_to_gentle_up={0,flat_to_gentle_up_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_gentle_up_to_flat={0,gentle_up_to_flat_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_gentle={0,gentle_curve,GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_gentle_to_steep_up={0,gentle_to_steep_up_curve,GENTLE_TO_STEEP_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_steep_to_gentle_up={0,steep_to_gentle_up_curve,GENTLE_TO_STEEP_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_steep={0,steep_curve,STEEP_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};

mask_t split_small_turn_masks[]={
	{TRACK_MASK_INTERSECT,13,0,0,small_turn_left_rects},    {TRACK_MASK_DIFFERENCE,13,0,0,small_turn_left_rects},    {TRACK_MASK_INTERSECT,13,-32,16,small_turn_left_rects+13}, {TRACK_MASK_DIFFERENCE,13,-32,16,small_turn_left_rects+13}, {TRACK_MASK_INTERSECT,26,0,32,small_turn_left_rects+26},  {TRACK_MASK_DIFFERENCE,26,0,32,small_turn_left_rects+26},
	{TRACK_MASK_INTERSECT,17,0,0,small_turn_left_rects+52}, {TRACK_MASK_DIFFERENCE,17,0,0,small_turn_left_rects+52}, {TRACK_MASK_INTERSECT,16,-32,-16,small_turn_left_rects+69},{TRACK_MASK_DIFFERENCE,16,-32,-16,small_turn_left_rects+69},{TRACK_MASK_INTERSECT,17,-64,0,small_turn_left_rects+85}, {TRACK_MASK_DIFFERENCE,17,-64,0,small_turn_left_rects+85},
	{TRACK_MASK_INTERSECT,26,0,0,small_turn_left_rects+102},{TRACK_MASK_DIFFERENCE,26,0,0,small_turn_left_rects+102},{TRACK_MASK_INTERSECT,13,32,-16,small_turn_left_rects+128},{TRACK_MASK_DIFFERENCE,13,32,-16,small_turn_left_rects+128},{TRACK_MASK_INTERSECT,13,0,-32,small_turn_left_rects+141},{TRACK_MASK_DIFFERENCE,13,0,-32,small_turn_left_rects+141},
	{TRACK_MASK_INTERSECT,4,0,0,small_turn_left_rects+154}, {TRACK_MASK_DIFFERENCE,4,0,0,small_turn_left_rects+154}, {TRACK_MASK_INTERSECT,3,32,16,small_turn_left_rects+158},  {TRACK_MASK_DIFFERENCE,3,32,16,small_turn_left_rects+158},  {TRACK_MASK_INTERSECT,4,64,0,small_turn_left_rects+161}  ,{TRACK_MASK_DIFFERENCE,4,64,0,small_turn_left_rects+161}
};
const track_section_t split_small_turn_left={0,small_turn_left_curve,SMALL_TURN_LENGTH,{{VIEW_NEEDS_TRACK_MASK,6,split_small_turn_masks},{VIEW_NEEDS_TRACK_MASK,6,split_small_turn_masks+6},{VIEW_NEEDS_TRACK_MASK,6,split_small_turn_masks+12},{VIEW_NEEDS_TRACK_MASK,6,split_small_turn_masks+18}}};
mask_t split_medium_turn_left_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects},    {TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects},    {TRACK_MASK_INTERSECT,49,-32,16,medium_turn_left_rects+33}  ,{TRACK_MASK_DIFFERENCE,49,-32,16,medium_turn_left_rects+33}  ,{TRACK_MASK_INTERSECT,31,0,32,medium_turn_left_rects+82},  {TRACK_MASK_DIFFERENCE,31,0,32,medium_turn_left_rects+82},  {TRACK_MASK_INTERSECT,32,-32,48,medium_turn_left_rects+113}, {TRACK_MASK_DIFFERENCE,32,-32,48,medium_turn_left_rects+113}, {TRACK_MASK_INTERSECT,32,0,64,medium_turn_left_rects+145},  {TRACK_MASK_DIFFERENCE,32,0,64,medium_turn_left_rects+145},
{TRACK_MASK_INTERSECT,17,0,0,medium_turn_left_rects+177},{TRACK_MASK_DIFFERENCE,17,0,0,medium_turn_left_rects+177},{TRACK_MASK_INTERSECT,16,-32,-16,medium_turn_left_rects+194},{TRACK_MASK_DIFFERENCE,16,-32,-16,medium_turn_left_rects+194},{TRACK_MASK_INTERSECT,16,-64,0,medium_turn_left_rects+210},{TRACK_MASK_DIFFERENCE,16,-64,0,medium_turn_left_rects+210},{TRACK_MASK_INTERSECT,16,-96,-16,medium_turn_left_rects+226},{TRACK_MASK_DIFFERENCE,16,-96,-16,medium_turn_left_rects+226},{TRACK_MASK_INTERSECT,17,-128,0,medium_turn_left_rects+242},{TRACK_MASK_DIFFERENCE,17,-128,0,medium_turn_left_rects+242},
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects+259},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects+259},{TRACK_MASK_INTERSECT,32,32,-16,medium_turn_left_rects+292} ,{TRACK_MASK_DIFFERENCE,32,32,-16,medium_turn_left_rects+292} ,{TRACK_MASK_INTERSECT,31,0,-32,medium_turn_left_rects+324},{TRACK_MASK_DIFFERENCE,31,0,-32,medium_turn_left_rects+324},{TRACK_MASK_INTERSECT,49,32,-48,medium_turn_left_rects+355}, {TRACK_MASK_DIFFERENCE,49,32,-48,medium_turn_left_rects+355}, {TRACK_MASK_INTERSECT,32,0,-64,medium_turn_left_rects+404}, {TRACK_MASK_DIFFERENCE,32,0,-64,medium_turn_left_rects+404},
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects+436},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects+436},{TRACK_MASK_INTERSECT,31,32,16,medium_turn_left_rects+469}  ,{TRACK_MASK_DIFFERENCE,31,32,16,medium_turn_left_rects+469}  ,{TRACK_MASK_INTERSECT,32,64,0,medium_turn_left_rects+500}, {TRACK_MASK_DIFFERENCE,32,64,0,medium_turn_left_rects+500}, {TRACK_MASK_INTERSECT,31,96,16,medium_turn_left_rects+532},  {TRACK_MASK_DIFFERENCE,31,96,16,medium_turn_left_rects+532},  {TRACK_MASK_INTERSECT,33,128,0,medium_turn_left_rects+563}, {TRACK_MASK_DIFFERENCE,33,128,0,medium_turn_left_rects+563}
};
const track_section_t split_medium_turn_left={0,medium_turn_left_curve,MEDIUM_TURN_LENGTH,{{VIEW_NEEDS_TRACK_MASK,10,split_medium_turn_left_masks},{VIEW_NEEDS_TRACK_MASK,10,split_medium_turn_left_masks+10},{VIEW_NEEDS_TRACK_MASK,10,split_medium_turn_left_masks+20},{VIEW_NEEDS_TRACK_MASK,10,split_medium_turn_left_masks+30}}};
mask_t split_large_turn_left_to_diag_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,large_turn_left_to_diag_rects},    {TRACK_MASK_DIFFERENCE,33,0,0,large_turn_left_to_diag_rects},    {TRACK_MASK_INTERSECT,47,-32,16,large_turn_left_to_diag_rects+33},  {TRACK_MASK_DIFFERENCE,47,-32,16,large_turn_left_to_diag_rects+33},  {TRACK_MASK_INTERSECT,31,0,32,large_turn_left_to_diag_rects+80},  {TRACK_MASK_DIFFERENCE,31,0,32,large_turn_left_to_diag_rects+80},  {TRACK_MASK_INTERSECT,31,-32,48,large_turn_left_to_diag_rects+111}, {TRACK_MASK_DIFFERENCE,31,-32,48,large_turn_left_to_diag_rects+111},
{TRACK_MASK_INTERSECT,17,0,0,large_turn_left_to_diag_rects+142},{TRACK_MASK_DIFFERENCE,17,0,0,large_turn_left_to_diag_rects+142},{TRACK_MASK_INTERSECT,16,-32,-16,large_turn_left_to_diag_rects+159},{TRACK_MASK_DIFFERENCE,16,-32,-16,large_turn_left_to_diag_rects+159},{TRACK_MASK_INTERSECT,16,-64,0,large_turn_left_to_diag_rects+175},{TRACK_MASK_DIFFERENCE,16,-64,0,large_turn_left_to_diag_rects+175},{TRACK_MASK_INTERSECT,17,-96,-16,large_turn_left_to_diag_rects+191},{TRACK_MASK_DIFFERENCE,17,-96,-16,large_turn_left_to_diag_rects+191},
{TRACK_MASK_INTERSECT,16,0,0,large_turn_left_to_diag_rects+208},{TRACK_MASK_DIFFERENCE,16,0,0,large_turn_left_to_diag_rects+208},{TRACK_MASK_INTERSECT,32,32,-16,large_turn_left_to_diag_rects+224}, {TRACK_MASK_DIFFERENCE,32,32,-16,large_turn_left_to_diag_rects+224}, {TRACK_MASK_INTERSECT,33,0,-32,large_turn_left_to_diag_rects+256},{TRACK_MASK_DIFFERENCE,33,0,-32,large_turn_left_to_diag_rects+256},{TRACK_MASK_INTERSECT,33,32,-48,large_turn_left_to_diag_rects+289}, {TRACK_MASK_DIFFERENCE,33,32,-48,large_turn_left_to_diag_rects+289},
{TRACK_MASK_INTERSECT,17,0,0,large_turn_left_to_diag_rects+322},{TRACK_MASK_DIFFERENCE,17,0,0,large_turn_left_to_diag_rects+322},{TRACK_MASK_INTERSECT,16,32,16,large_turn_left_to_diag_rects+339},  {TRACK_MASK_DIFFERENCE,16,32,16,large_turn_left_to_diag_rects+339},  {TRACK_MASK_INTERSECT,16,64,0,large_turn_left_to_diag_rects+355}, {TRACK_MASK_DIFFERENCE,16,64,0,large_turn_left_to_diag_rects+355}, {TRACK_MASK_INTERSECT,17,96,16,large_turn_left_to_diag_rects+371},  {TRACK_MASK_DIFFERENCE,17,96,16,large_turn_left_to_diag_rects+371},
};
const track_section_t split_large_turn_left_to_diag={0,large_turn_left_to_diag_curve,LARGE_TURN_LENGTH,{{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_left_to_diag_masks},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_left_to_diag_masks+8},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_left_to_diag_masks+16},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_left_to_diag_masks+24}}};
mask_t split_large_turn_right_to_diag_masks[]={
{TRACK_MASK_INTERSECT,17,0,0,large_turn_right_to_diag_rects},    {TRACK_MASK_DIFFERENCE,17,0,0,large_turn_right_to_diag_rects},    {TRACK_MASK_INTERSECT,16,-32,16,large_turn_right_to_diag_rects+17}, {TRACK_MASK_DIFFERENCE,16,-32,16,large_turn_right_to_diag_rects+17}, {TRACK_MASK_INTERSECT,16,-64,0,large_turn_right_to_diag_rects+33}, {TRACK_MASK_DIFFERENCE,16,-64,0,large_turn_right_to_diag_rects+33}, {TRACK_MASK_INTERSECT,17,-96,16,large_turn_right_to_diag_rects+49},  {TRACK_MASK_DIFFERENCE,17,-96,16,large_turn_right_to_diag_rects+49},
{TRACK_MASK_INTERSECT,16,0,0,large_turn_right_to_diag_rects+66}, {TRACK_MASK_DIFFERENCE,16,0,0,large_turn_right_to_diag_rects+66}, {TRACK_MASK_INTERSECT,32,-32,-16,large_turn_right_to_diag_rects+82},{TRACK_MASK_DIFFERENCE,32,-32,-16,large_turn_right_to_diag_rects+82},{TRACK_MASK_INTERSECT,34,0,-32,large_turn_right_to_diag_rects+114},{TRACK_MASK_DIFFERENCE,34,0,-32,large_turn_right_to_diag_rects+114},{TRACK_MASK_INTERSECT,33,-32,-48,large_turn_right_to_diag_rects+148},{TRACK_MASK_DIFFERENCE,33,-32,-48,large_turn_right_to_diag_rects+148},
{TRACK_MASK_INTERSECT,17,0,0,large_turn_right_to_diag_rects+181},{TRACK_MASK_DIFFERENCE,17,0,0,large_turn_right_to_diag_rects+181},{TRACK_MASK_INTERSECT,16,32,-16,large_turn_right_to_diag_rects+198},{TRACK_MASK_DIFFERENCE,16,32,-16,large_turn_right_to_diag_rects+198},{TRACK_MASK_INTERSECT,16,64,0,large_turn_right_to_diag_rects+214}, {TRACK_MASK_DIFFERENCE,16,64,0,large_turn_right_to_diag_rects+214}, {TRACK_MASK_INTERSECT,17,96,-16,large_turn_right_to_diag_rects+230}, {TRACK_MASK_DIFFERENCE,17,96,-16,large_turn_right_to_diag_rects+230},
{TRACK_MASK_INTERSECT,33,0,0,large_turn_right_to_diag_rects+247},{TRACK_MASK_DIFFERENCE,33,0,0,large_turn_right_to_diag_rects+247},{TRACK_MASK_INTERSECT,41,32,16,large_turn_right_to_diag_rects+280}, {TRACK_MASK_DIFFERENCE,41,32,16,large_turn_right_to_diag_rects+280}, {TRACK_MASK_INTERSECT,31,0,32,large_turn_right_to_diag_rects+321}, {TRACK_MASK_DIFFERENCE,31,0,32,large_turn_right_to_diag_rects+321}, {TRACK_MASK_INTERSECT,31,32,48,large_turn_right_to_diag_rects+352},  {TRACK_MASK_DIFFERENCE,31,32,48,large_turn_right_to_diag_rects+352},
};
const track_section_t split_large_turn_right_to_diag={0,large_turn_right_to_diag_curve,LARGE_TURN_LENGTH,{{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_right_to_diag_masks},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_right_to_diag_masks+8},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_right_to_diag_masks+16},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_right_to_diag_masks+24}}};


mask_t split_diag_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&all},{TRACK_MASK_DIFFERENCE,1,0,0,&all},0};
mask_t split_diag_slope_masks[]={{TRACK_MASK_INTERSECT,1,0,0,&diag_slope_rect},{TRACK_MASK_DIFFERENCE,1,0,0,&diag_slope_rect},0};

const track_section_t split_flat_diag={TRACK_DIAGONAL,flat_diag_curve,FLAT_DIAG_LENGTH,                                                                               {{VIEW_NEEDS_TRACK_MASK,2,split_diag_masks},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,split_diag_masks},{0,1,NULL}}};
const track_section_t split_flat_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_SUPPORT_BASE,flat_to_gentle_up_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,                        {{VIEW_NEEDS_TRACK_MASK,2,split_diag_masks},{0,1,NULL},{VIEW_NEEDS_TRACK_MASK,2,split_diag_masks},{0,1,NULL}}};
const track_section_t split_gentle_to_flat_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,gentle_to_flat_up_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,   {{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask}}};
const track_section_t split_gentle_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,gentle_diag_curve,GENTLE_DIAG_LENGTH,                                 {{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask}}};
const track_section_t split_gentle_to_steep_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,gentle_to_steep_up_diag_curve,GENTLE_TO_STEEP_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask}}};
const track_section_t split_steep_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,steep_to_gentle_up_diag_curve,GENTLE_TO_STEEP_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask}}};
const track_section_t split_steep_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_SUPPORT_BASE,steep_diag_curve,STEEP_DIAG_LENGTH,                                    {{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask}}};
const track_section_t split_small_flat_to_steep_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,small_flat_to_steep_up_diag_curve,SMALL_FLAT_TO_STEEP_DIAG_LENGTH,       {{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask}}};
const track_section_t split_small_steep_to_flat_up_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND,small_steep_to_flat_up_diag_curve,SMALL_FLAT_TO_STEEP_DIAG_LENGTH,       {{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask},{VIEW_NEEDS_TRACK_MASK,2,split_diag_slope_masks},{0,1,&diag_slope_mask}}};


const track_section_t split_flat_to_left_bank={TRACK_EXIT_BANK_LEFT,flat_to_left_bank_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,flat_to_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,flat_to_left_bank_masks+2},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_flat_to_right_bank={TRACK_EXIT_BANK_RIGHT,flat_to_right_bank_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,flat_to_right_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,flat_to_right_bank_masks+2}}};
const track_section_t split_left_bank_to_gentle_up={TRACK_ENTRY_BANK_LEFT,left_bank_to_gentle_up_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,left_bank_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,2,left_bank_to_gentle_up_masks+2},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_right_bank_to_gentle_up={TRACK_ENTRY_BANK_RIGHT,right_bank_to_gentle_up_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,right_bank_to_gentle_up_masks},{VIEW_NEEDS_TRACK_MASK,2,right_bank_to_gentle_up_masks+2}}};
const track_section_t split_gentle_up_to_left_bank={TRACK_EXIT_BANK_LEFT,gentle_up_to_left_bank_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_left_bank_masks+2},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};
const track_section_t split_gentle_up_to_right_bank={TRACK_EXIT_BANK_RIGHT,gentle_up_to_right_bank_curve,FLAT_TO_GENTLE_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_right_bank_masks},{VIEW_NEEDS_TRACK_MASK,2,gentle_up_to_right_bank_masks+2}}};
const track_section_t split_left_bank={TRACK_BANK_LEFT,left_bank_curve,FLAT_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks},{VIEW_NEEDS_TRACK_MASK,2,single_tile_split_masks}}};


const track_section_t split_flat_to_left_bank_diag={TRACK_DIAGONAL|TRACK_EXIT_BANK_LEFT,flat_to_left_bank_diag_curve,FLAT_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks}}};
const track_section_t split_flat_to_right_bank_diag={TRACK_DIAGONAL|TRACK_EXIT_BANK_RIGHT,flat_to_right_bank_diag_curve,FLAT_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks}}};
const track_section_t split_left_bank_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_ENTRY_BANK_LEFT|TRACK_SUPPORT_BASE,left_bank_to_gentle_up_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks}}};
const track_section_t split_right_bank_to_gentle_up_diag={TRACK_DIAGONAL|TRACK_ENTRY_BANK_RIGHT|TRACK_SUPPORT_BASE,right_bank_to_gentle_up_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks}}};
const track_section_t split_gentle_up_to_left_bank_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_EXIT_BANK_LEFT|TRACK_SUPPORT_BASE,gentle_up_to_left_bank_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks}}};
const track_section_t split_gentle_up_to_right_bank_diag={TRACK_DIAGONAL|TRACK_EXTRUDE_BEHIND|TRACK_EXIT_BANK_RIGHT|TRACK_SUPPORT_BASE,gentle_up_to_right_bank_diag_curve,FLAT_TO_GENTLE_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks}}};
const track_section_t split_left_bank_diag={TRACK_DIAGONAL|TRACK_BANK_LEFT,left_bank_diag_curve,FLAT_DIAG_LENGTH,{{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks},{VIEW_NEEDS_TRACK_MASK,2,diagonal_transition_masks}}};




mask_t split_small_turn_left_bank_masks[]={
	{TRACK_MASK_INTERSECT,13,0,0,small_turn_left_rects}    ,{TRACK_MASK_DIFFERENCE,13,0,0,small_turn_left_rects},    {TRACK_MASK_INTERSECT,13,-32,16,small_turn_left_rects+13}, {TRACK_MASK_DIFFERENCE,13,-32,16,small_turn_left_rects+13}, {TRACK_MASK_INTERSECT,26,0,32,small_turn_left_rects+26}                     ,{TRACK_MASK_DIFFERENCE,26,0,32,small_turn_left_rects+26}, 
	{TRACK_MASK_INTERSECT,17,0,0,small_turn_left_rects+52} ,{TRACK_MASK_DIFFERENCE,17,0,0,small_turn_left_rects+52}, {TRACK_MASK_INTERSECT,16,-32,-16,small_turn_left_rects+69},{TRACK_MASK_DIFFERENCE,16,-32,-16,small_turn_left_rects+69},{TRACK_MASK_INTERSECT,17,-64,0,small_turn_left_rects+85}                    ,{TRACK_MASK_DIFFERENCE,17,-64,0,small_turn_left_rects+85},
	{TRACK_MASK_INTERSECT,26,0,0,small_turn_left_rects+102},{TRACK_MASK_DIFFERENCE,26,0,0,small_turn_left_rects+102},{TRACK_MASK_INTERSECT,13,32,-16,small_turn_left_rects+128},{TRACK_MASK_DIFFERENCE,13,32,-16,small_turn_left_rects+128},{TRACK_MASK_INTERSECT,13,0,-32,small_turn_left_rects+141},{TRACK_MASK_DIFFERENCE,13,0,-32,small_turn_left_rects+141},
	{TRACK_MASK_INTERSECT,4,0,0,small_turn_left_rects+154}, {TRACK_MASK_DIFFERENCE,4,0,0,small_turn_left_rects+154}, {TRACK_MASK_INTERSECT,3,32,16,small_turn_left_rects+158},  {TRACK_MASK_DIFFERENCE,3,32,16,small_turn_left_rects+158},  {TRACK_MASK_INTERSECT,4,64,0,small_turn_left_rects+161}                     ,{TRACK_MASK_DIFFERENCE,4,64,0,small_turn_left_rects+161}

};
const track_section_t split_small_turn_left_bank={TRACK_BANK_LEFT,small_turn_left_bank_curve,0.75*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,6,split_small_turn_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,6,split_small_turn_left_bank_masks+6},{VIEW_NEEDS_TRACK_MASK,6,split_small_turn_left_bank_masks+12},{VIEW_NEEDS_TRACK_MASK,6,split_small_turn_left_bank_masks+18}}};


mask_t split_medium_turn_left_bank_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects}    ,{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects}    ,{TRACK_MASK_INTERSECT,49,-32,16,medium_turn_left_rects+33}  ,{TRACK_MASK_DIFFERENCE,49,-32,16,medium_turn_left_rects+33}  ,{TRACK_MASK_INTERSECT,31,0,32,medium_turn_left_rects+82}  ,{TRACK_MASK_DIFFERENCE,31,0,32,medium_turn_left_rects+82}  ,{TRACK_MASK_INTERSECT,32,-32,48,medium_turn_left_rects+113} ,{TRACK_MASK_DIFFERENCE,32,-32,48,medium_turn_left_rects+113} ,{TRACK_MASK_INTERSECT,32,0,64,medium_turn_left_rects+145}  ,{TRACK_MASK_DIFFERENCE,32,0,64,medium_turn_left_rects+145},
{TRACK_MASK_INTERSECT,17,0,0,medium_turn_left_rects+177},{TRACK_MASK_DIFFERENCE,17,0,0,medium_turn_left_rects+177},{TRACK_MASK_INTERSECT,16,-32,-16,medium_turn_left_rects+194},{TRACK_MASK_DIFFERENCE,16,-32,-16,medium_turn_left_rects+194},{TRACK_MASK_INTERSECT,16,-64,0,medium_turn_left_rects+210},{TRACK_MASK_DIFFERENCE,16,-64,0,medium_turn_left_rects+210},{TRACK_MASK_INTERSECT,16,-96,-16,medium_turn_left_rects+226},{TRACK_MASK_DIFFERENCE,16,-96,-16,medium_turn_left_rects+226},{TRACK_MASK_INTERSECT,17,-128,0,medium_turn_left_rects+242},{TRACK_MASK_DIFFERENCE,17,-128,0,medium_turn_left_rects+242},
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects+259},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects+259},{TRACK_MASK_INTERSECT,32,32,-16,medium_turn_left_rects+292} ,{TRACK_MASK_DIFFERENCE,32,32,-16,medium_turn_left_rects+292} ,{TRACK_MASK_INTERSECT,31,0,-32,medium_turn_left_rects+324},{TRACK_MASK_DIFFERENCE,31,0,-32,medium_turn_left_rects+324},{TRACK_MASK_INTERSECT,49,32,-48,medium_turn_left_rects+355} ,{TRACK_MASK_DIFFERENCE,49,32,-48,medium_turn_left_rects+355} ,{TRACK_MASK_INTERSECT,32,0,-64,medium_turn_left_rects+404} ,{TRACK_MASK_DIFFERENCE,32,0,-64,medium_turn_left_rects+404},
{TRACK_MASK_INTERSECT,33,0,0,medium_turn_left_rects+436},{TRACK_MASK_DIFFERENCE,33,0,0,medium_turn_left_rects+436},{TRACK_MASK_INTERSECT,31,32,16,medium_turn_left_rects+469}  ,{TRACK_MASK_DIFFERENCE,31,32,16,medium_turn_left_rects+469}  ,{TRACK_MASK_INTERSECT,32,64,0,medium_turn_left_rects+500} ,{TRACK_MASK_DIFFERENCE,32,64,0,medium_turn_left_rects+500} ,{TRACK_MASK_INTERSECT,31,96,16,medium_turn_left_rects+532}  ,{TRACK_MASK_DIFFERENCE,31,96,16,medium_turn_left_rects+532}  ,{TRACK_MASK_INTERSECT,33,128,0,medium_turn_left_rects+563} ,{TRACK_MASK_DIFFERENCE,33,128,0,medium_turn_left_rects+563}

};
const track_section_t split_medium_turn_left_bank={TRACK_BANK_LEFT,medium_turn_left_bank_curve,1.25*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,10,split_medium_turn_left_bank_masks},{VIEW_NEEDS_TRACK_MASK,10,split_medium_turn_left_bank_masks+10},{VIEW_NEEDS_TRACK_MASK,10,split_medium_turn_left_bank_masks+20},{VIEW_NEEDS_TRACK_MASK,10,split_medium_turn_left_bank_masks+30}}};


mask_t split_large_turn_left_to_diag_bank_masks[]={
{TRACK_MASK_INTERSECT,33,0,0,large_turn_left_to_diag_bank_rects}    ,{TRACK_MASK_DIFFERENCE,33,0,0,large_turn_left_to_diag_bank_rects}    ,{TRACK_MASK_INTERSECT,47,-32,16,large_turn_left_to_diag_bank_rects+33}  ,{TRACK_MASK_DIFFERENCE,47,-32,16,large_turn_left_to_diag_bank_rects+33}  ,{TRACK_MASK_INTERSECT,31,0,32,large_turn_left_to_diag_bank_rects+80}  ,{TRACK_MASK_DIFFERENCE,31,0,32,large_turn_left_to_diag_bank_rects+80}  ,{TRACK_MASK_INTERSECT,62,-32,48,large_turn_left_to_diag_bank_rects+80}, {TRACK_MASK_DIFFERENCE,62,-32,48,large_turn_left_to_diag_bank_rects+80},
{TRACK_MASK_INTERSECT,33,0,0,large_turn_left_to_diag_bank_rects+142},{TRACK_MASK_DIFFERENCE,33,0,0,large_turn_left_to_diag_bank_rects+142},{TRACK_MASK_INTERSECT,17,-32,-16,large_turn_left_to_diag_bank_rects+175},{TRACK_MASK_DIFFERENCE,17,-32,-16,large_turn_left_to_diag_bank_rects+175},{TRACK_MASK_INTERSECT,1,-64,0,large_turn_left_to_diag_bank_rects+192} ,{TRACK_MASK_DIFFERENCE,1,-64,0,large_turn_left_to_diag_bank_rects+192} ,{TRACK_MASK_INTERSECT,1,-96,-16,large_turn_left_to_diag_bank_rects+193},{TRACK_MASK_DIFFERENCE,1,-96,-16,large_turn_left_to_diag_bank_rects+193},
{TRACK_MASK_INTERSECT,16,0,0,large_turn_left_to_diag_bank_rects+194},{TRACK_MASK_DIFFERENCE,16,0,0,large_turn_left_to_diag_bank_rects+194},{TRACK_MASK_INTERSECT,32,32,-16,large_turn_left_to_diag_bank_rects+210} ,{TRACK_MASK_DIFFERENCE,32,32,-16,large_turn_left_to_diag_bank_rects+210} ,{TRACK_MASK_INTERSECT,33,0,-32,large_turn_left_to_diag_bank_rects+242},{TRACK_MASK_DIFFERENCE,33,0,-32,large_turn_left_to_diag_bank_rects+242},{TRACK_MASK_INTERSECT,33,32,-48,large_turn_left_to_diag_bank_rects+275},{TRACK_MASK_DIFFERENCE,33,32,-48,large_turn_left_to_diag_bank_rects+275},
{TRACK_MASK_INTERSECT,17,0,0,large_turn_left_to_diag_bank_rects+308},{TRACK_MASK_DIFFERENCE,17,0,0,large_turn_left_to_diag_bank_rects+308},{TRACK_MASK_INTERSECT,16,32,16,large_turn_left_to_diag_bank_rects+325}  ,{TRACK_MASK_DIFFERENCE,16,32,16,large_turn_left_to_diag_bank_rects+325}  ,{TRACK_MASK_INTERSECT,16,64,0,large_turn_left_to_diag_bank_rects+341} ,{TRACK_MASK_DIFFERENCE,16,64,0,large_turn_left_to_diag_bank_rects+341} ,{TRACK_MASK_INTERSECT,17,96,16,large_turn_left_to_diag_bank_rects+357}, {TRACK_MASK_DIFFERENCE,17,96,16,large_turn_left_to_diag_bank_rects+357},
};
const track_section_t split_large_turn_left_to_diag_bank={TRACK_BANK_LEFT,large_turn_left_to_diag_bank_curve,0.875*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_left_to_diag_bank_masks},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_left_to_diag_bank_masks+8},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_left_to_diag_bank_masks+16},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_left_to_diag_bank_masks+24}}};


mask_t split_large_turn_right_to_diag_bank_masks[]={
{TRACK_MASK_INTERSECT,17,0,0,large_turn_right_to_diag_bank_rects}    ,{TRACK_MASK_DIFFERENCE,17,0,0,large_turn_right_to_diag_bank_rects}    ,{TRACK_MASK_INTERSECT,16,-32,16,large_turn_right_to_diag_bank_rects+17} ,{TRACK_MASK_DIFFERENCE,16,-32,16,large_turn_right_to_diag_bank_rects+17} ,{TRACK_MASK_INTERSECT,16,-64,0,large_turn_right_to_diag_bank_rects+33} ,{TRACK_MASK_DIFFERENCE,16,-64,0,large_turn_right_to_diag_bank_rects+33} ,{TRACK_MASK_INTERSECT,17,-96,16,large_turn_right_to_diag_bank_rects+49}  ,{TRACK_MASK_DIFFERENCE,17,-96,16,large_turn_right_to_diag_bank_rects+49},
{TRACK_MASK_INTERSECT,16,0,0,large_turn_right_to_diag_bank_rects+66} ,{TRACK_MASK_DIFFERENCE,16,0,0,large_turn_right_to_diag_bank_rects+66} ,{TRACK_MASK_INTERSECT,32,-32,-16,large_turn_right_to_diag_bank_rects+82},{TRACK_MASK_DIFFERENCE,32,-32,-16,large_turn_right_to_diag_bank_rects+82},{TRACK_MASK_INTERSECT,34,0,-32,large_turn_right_to_diag_bank_rects+114},{TRACK_MASK_DIFFERENCE,34,0,-32,large_turn_right_to_diag_bank_rects+114},{TRACK_MASK_INTERSECT,33,-32,-48,large_turn_right_to_diag_bank_rects+148},{TRACK_MASK_DIFFERENCE,33,-32,-48,large_turn_right_to_diag_bank_rects+148},
{TRACK_MASK_INTERSECT,17,0,0,large_turn_right_to_diag_bank_rects+181},{TRACK_MASK_DIFFERENCE,17,0,0,large_turn_right_to_diag_bank_rects+181},{TRACK_MASK_INTERSECT,1,32,-16,large_turn_right_to_diag_bank_rects+198} ,{TRACK_MASK_DIFFERENCE,1,32,-16,large_turn_right_to_diag_bank_rects+198} ,{TRACK_MASK_INTERSECT,1,64,0,large_turn_right_to_diag_bank_rects+199}  ,{TRACK_MASK_DIFFERENCE,1,64,0,large_turn_right_to_diag_bank_rects+199}  ,{TRACK_MASK_INTERSECT,1,96,-16,large_turn_right_to_diag_bank_rects+200}  ,{TRACK_MASK_DIFFERENCE,1,96,-16,large_turn_right_to_diag_bank_rects+200},
{TRACK_MASK_INTERSECT,27,0,0,large_turn_right_to_diag_bank_rects+201},{TRACK_MASK_DIFFERENCE,27,0,0,large_turn_right_to_diag_bank_rects+201},{TRACK_MASK_INTERSECT,41,32,16,large_turn_right_to_diag_bank_rects+228} ,{TRACK_MASK_DIFFERENCE,41,32,16,large_turn_right_to_diag_bank_rects+228} ,{TRACK_MASK_INTERSECT,31,0,32,large_turn_right_to_diag_bank_rects+269} ,{TRACK_MASK_DIFFERENCE,31,0,32,large_turn_right_to_diag_bank_rects+269} ,{TRACK_MASK_INTERSECT,31,32,48,large_turn_right_to_diag_bank_rects+300}  ,{TRACK_MASK_DIFFERENCE,31,32,48,large_turn_right_to_diag_bank_rects+300},
};
const track_section_t split_large_turn_right_to_diag_bank={TRACK_BANK_RIGHT,large_turn_right_to_diag_bank_curve,0.875*TILE_SIZE*3.1415926,{{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_right_to_diag_bank_masks},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_right_to_diag_bank_masks+8},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_right_to_diag_bank_masks+16},{VIEW_NEEDS_TRACK_MASK,8,split_large_turn_right_to_diag_bank_masks+24}}};

mask_t split_half_loop_masks[]={
	{TRACK_MASK_INTERSECT,24,-6,-3,half_loop_rects},{TRACK_MASK_DIFFERENCE,24,-6,-3,half_loop_rects},{TRACK_MASK_INTERSECT,36,-32,32,half_loop_rects+24},{TRACK_MASK_DIFFERENCE,36,-32,32,half_loop_rects+24},{TRACK_MASK_INTERSECT,12,-48,56,half_loop_rects+60},{TRACK_MASK_DIFFERENCE,12,-48,56,half_loop_rects+60},{TRACK_MASK_INTERSECT,2,-32,96+72,half_loop_rects+72},{TRACK_MASK_DIFFERENCE,2,-32,96+72,half_loop_rects+72},
	{TRACK_MASK_INTERSECT,1,6,-3,half_loop_rects+74}, {TRACK_MASK_DIFFERENCE,1,6,-3,half_loop_rects+74}, {TRACK_MASK_INTERSECT,2,-18,-7,half_loop_rects+75}, {TRACK_MASK_DIFFERENCE,2,-18,-7,half_loop_rects+75}, {TRACK_MASK_INTERSECT,2,-76,-6,half_loop_rects+77}, {TRACK_MASK_DIFFERENCE,2,-76,-6,half_loop_rects+77}, {TRACK_MASK_INTERSECT,2,-32,136,half_loop_rects+79}, {TRACK_MASK_DIFFERENCE,2,-32,136,half_loop_rects+79},
	{TRACK_MASK_INTERSECT,1,-6,-3,half_loop_rects+81},{TRACK_MASK_DIFFERENCE,1,-6,-3,half_loop_rects+81},{TRACK_MASK_INTERSECT,2,26,-3,half_loop_rects+82},  {TRACK_MASK_DIFFERENCE,2,26,-3,half_loop_rects+82},  {TRACK_MASK_INTERSECT,2,58,-13,half_loop_rects+84},{TRACK_MASK_DIFFERENCE,2,58,-13,half_loop_rects+84}, {TRACK_MASK_INTERSECT,2,16,96+32,half_loop_rects+86},{TRACK_MASK_DIFFERENCE,2,16,96+32,half_loop_rects+86},
	{TRACK_MASK_INTERSECT,24,6,-3,half_loop_rects+88},{TRACK_MASK_DIFFERENCE,24,6,-3,half_loop_rects+88},{TRACK_MASK_INTERSECT,36,38,29,half_loop_rects+112},{TRACK_MASK_DIFFERENCE,36,38,29,half_loop_rects+112},{TRACK_MASK_INTERSECT,12,64,48,half_loop_rects+148},{TRACK_MASK_DIFFERENCE,12,64,48,half_loop_rects+148},{TRACK_MASK_INTERSECT,2,48,160,half_loop_rects+160} ,{TRACK_MASK_DIFFERENCE,2,48,160,half_loop_rects+160}
};
const track_section_t split_half_loop={TRACK_NO_SUPPORTS|TRACK_SPECIAL_HALF_LOOP,half_loop_curve,HALF_LOOP_LENGTH,{{VIEW_NEEDS_TRACK_MASK,8,split_half_loop_masks},{VIEW_NEEDS_TRACK_MASK,8,split_half_loop_masks+8},{VIEW_NEEDS_TRACK_MASK,8,split_half_loop_masks+16},{VIEW_NEEDS_TRACK_MASK,8,split_half_loop_masks+24}}};



track_list_t track_list_default={
	flat,
	flat_asymmetric,
	brake,
	block_brake,
	booster,
	flat_to_gentle_up,
	gentle_up_to_flat,
	gentle,
	gentle_to_steep_up,
	steep_to_gentle_up,
	steep,
	steep_to_vertical_up,
	vertical_to_steep_up,
	vertical,
	small_turn_left,
	medium_turn_left,
	large_turn_left_to_diag,
	large_turn_right_to_diag,
	flat_diag,
	flat_to_gentle_up_diag,
	gentle_to_flat_up_diag,
	gentle_diag,
	gentle_to_steep_up_diag,
	steep_to_gentle_up_diag,
	steep_diag,
	flat_to_left_bank,
	flat_to_right_bank,
	left_bank_to_gentle_up,
	right_bank_to_gentle_up,
	gentle_up_to_left_bank,
	gentle_up_to_right_bank,
	left_bank,
	flat_to_left_bank_diag,
	flat_to_right_bank_diag,
	left_bank_to_gentle_up_diag,
	right_bank_to_gentle_up_diag,
	gentle_up_to_left_bank_diag,
	gentle_up_to_right_bank_diag,
	left_bank_diag,
	small_turn_left_bank,
	medium_turn_left_bank,
	large_turn_left_to_diag_bank,
	large_turn_right_to_diag_bank,
	small_turn_left_gentle_up,
	small_turn_right_gentle_up,
	medium_turn_left_gentle_up,
	medium_turn_right_gentle_up,
	very_small_turn_left_steep_up,
	very_small_turn_right_steep_up,
	vertical_twist_left_up,
	vertical_twist_right_up,
	gentle_up_to_gentle_up_left_bank,
	gentle_up_to_gentle_up_right_bank,
	gentle_up_left_bank_to_gentle_up,
	gentle_up_right_bank_to_gentle_up,
	left_bank_to_gentle_up_left_bank,
	gentle_up_left_bank_to_left_bank,
	right_bank_to_gentle_up_right_bank,
	gentle_up_right_bank_to_right_bank,
	gentle_up_left_bank,
	gentle_up_right_bank,
	flat_to_gentle_up_left_bank,
	flat_to_gentle_up_right_bank,
	gentle_up_left_bank_to_flat,
	gentle_up_right_bank_to_flat,
	small_turn_left_bank_gentle_up,
	small_turn_right_bank_gentle_up,
	medium_turn_left_bank_gentle_up,
	medium_turn_right_bank_gentle_up,
	s_bend_left,
	s_bend_right,
	small_helix_left_up,
	small_helix_right_up,
	medium_helix_left_up,
	medium_helix_right_up,
	barrel_roll_left,
	barrel_roll_right,
	inline_twist_left,
	inline_twist_right,
	half_loop,
	large_half_loop_left,
	large_half_loop_right,
	flat_to_steep_up,
	steep_to_flat_up,
	small_flat_to_steep_up,
	small_steep_to_flat_up,
	small_flat_to_steep_up_diag,
	small_steep_to_flat_up_diag,
	quarter_loop_up,
	corkscrew_left,
	corkscrew_right,
	large_corkscrew_left,
	large_corkscrew_right,
	small_turn_left_bank_to_gentle_up,
	small_turn_right_bank_to_gentle_up,
	launched_lift
	};


track_list_t track_list_semi_split={
	flat,
	flat_asymmetric,
	brake,
	block_brake,
	booster,
	flat_to_gentle_up,
	gentle_up_to_flat,
	gentle,
	gentle_to_steep_up,
	steep_to_gentle_up,
	steep,
	steep_to_vertical_up,
	vertical_to_steep_up,
	vertical,
	small_turn_left,
	medium_turn_left,
	large_turn_left_to_diag,
	large_turn_right_to_diag,
	flat_diag,
	flat_to_gentle_up_diag,
	gentle_to_flat_up_diag,
	gentle_diag,
	gentle_to_steep_up_diag,
	steep_to_gentle_up_diag,
	steep_diag,
	flat_to_left_bank,
	flat_to_right_bank,
	left_bank_to_gentle_up,
	right_bank_to_gentle_up,
	gentle_up_to_left_bank,
	gentle_up_to_right_bank,
	semi_split_left_bank,
	flat_to_left_bank_diag,
	flat_to_right_bank_diag,
	left_bank_to_gentle_up_diag,
	right_bank_to_gentle_up_diag,
	gentle_up_to_left_bank_diag,
	gentle_up_to_right_bank_diag,
	semi_split_left_bank_diag,
	semi_split_small_turn_left_bank,
	semi_split_medium_turn_left_bank,
	semi_split_large_turn_left_to_diag_bank,
	semi_split_large_turn_right_to_diag_bank,
	small_turn_left_gentle_up,
	small_turn_right_gentle_up,
	medium_turn_left_gentle_up,
	medium_turn_right_gentle_up,
	very_small_turn_left_steep_up,
	very_small_turn_right_steep_up,
	vertical_twist_left_up,
	vertical_twist_right_up,
	gentle_up_to_gentle_up_left_bank,
	gentle_up_to_gentle_up_right_bank,
	gentle_up_left_bank_to_gentle_up,
	gentle_up_right_bank_to_gentle_up,
	semi_split_left_bank_to_gentle_up_left_bank,
	semi_split_gentle_up_left_bank_to_left_bank,
	semi_split_right_bank_to_gentle_up_right_bank,
	semi_split_gentle_up_right_bank_to_right_bank,
	gentle_up_left_bank,
	gentle_up_right_bank,
	flat_to_gentle_up_left_bank,
	flat_to_gentle_up_right_bank,
	gentle_up_left_bank_to_flat,
	gentle_up_right_bank_to_flat,
	semi_split_small_turn_left_bank_gentle_up,
	semi_split_small_turn_right_bank_gentle_up,
	semi_split_medium_turn_left_bank_gentle_up,
	semi_split_medium_turn_right_bank_gentle_up,
	s_bend_left,
	s_bend_right,
	semi_split_small_helix_left_up,
	semi_split_small_helix_right_up,
	semi_split_medium_helix_left_up,
	semi_split_medium_helix_right_up,
	barrel_roll_left,
	barrel_roll_right,
	inline_twist_left,
	inline_twist_right,
	half_loop,
	large_half_loop_left,
	large_half_loop_right,
	flat_to_steep_up,
	steep_to_flat_up,
	small_flat_to_steep_up,
	small_steep_to_flat_up,
	small_flat_to_steep_up_diag,
	small_steep_to_flat_up_diag,
	semi_split_quarter_loop_up,
	corkscrew_left,//TODO may need semi split version
	corkscrew_right,
	large_corkscrew_left,
	large_corkscrew_right,
	small_turn_left_bank_to_gentle_up,
	small_turn_right_bank_to_gentle_up,
	launched_lift
	};


track_list_t track_list_split={
	split_flat,
	split_flat_asymmetric,
	split_brake,
	split_block_brake,
	split_booster,
	split_flat_to_gentle_up,
	split_gentle_up_to_flat,
	split_gentle,
	split_gentle_to_steep_up,
	split_steep_to_gentle_up,
	split_steep,
	steep_to_vertical_up,
	vertical_to_steep_up,
	vertical,
	split_small_turn_left,
	split_medium_turn_left,
	split_large_turn_left_to_diag,
	split_large_turn_right_to_diag,
	split_flat_diag,
	split_flat_to_gentle_up_diag,
	split_gentle_to_flat_up_diag,
	split_gentle_diag,
	split_gentle_to_steep_up_diag,
	split_steep_to_gentle_up_diag,
	split_steep_diag,
	split_flat_to_left_bank,
	split_flat_to_right_bank,
	split_left_bank_to_gentle_up,
	split_right_bank_to_gentle_up,
	split_gentle_up_to_left_bank,
	split_gentle_up_to_right_bank,
	split_left_bank,
	split_flat_to_left_bank_diag,
	split_flat_to_right_bank_diag,
	split_left_bank_to_gentle_up_diag,
	split_right_bank_to_gentle_up_diag,
	split_gentle_up_to_left_bank_diag,
	split_gentle_up_to_right_bank_diag,
	split_left_bank_diag,
	split_small_turn_left_bank,
	split_medium_turn_left_bank,
	split_large_turn_left_to_diag_bank,
	split_large_turn_right_to_diag_bank,
	small_turn_left_gentle_up,
	small_turn_right_gentle_up,
	medium_turn_left_gentle_up,
	medium_turn_right_gentle_up,
	very_small_turn_left_steep_up,
	very_small_turn_right_steep_up,
	vertical_twist_left_up,
	vertical_twist_right_up,
	gentle_up_to_gentle_up_left_bank,
	gentle_up_to_gentle_up_right_bank,
	gentle_up_left_bank_to_gentle_up,
	gentle_up_right_bank_to_gentle_up,
	left_bank_to_gentle_up_left_bank,
	gentle_up_left_bank_to_left_bank,
	right_bank_to_gentle_up_right_bank,
	gentle_up_right_bank_to_right_bank,
	gentle_up_left_bank,
	gentle_up_right_bank,
	flat_to_gentle_up_left_bank,
	flat_to_gentle_up_right_bank,
	gentle_up_left_bank_to_flat,
	gentle_up_right_bank_to_flat,
	small_turn_left_bank_gentle_up,
	small_turn_right_bank_gentle_up,
	medium_turn_left_bank_gentle_up,
	medium_turn_right_bank_gentle_up,
	s_bend_left,
	s_bend_right,
	small_helix_left_up,
	small_helix_right_up,
	medium_helix_left_up,
	medium_helix_right_up,
	barrel_roll_left,
	barrel_roll_right,
	inline_twist_left,
	inline_twist_right,
	split_half_loop,
	large_half_loop_left,
	large_half_loop_right,
	flat_to_steep_up,
	steep_to_flat_up,
	small_flat_to_steep_up,
	small_steep_to_flat_up,
	small_flat_to_steep_up_diag,
	small_steep_to_flat_up_diag,
	quarter_loop_up,
	corkscrew_left,
	corkscrew_right,
	large_corkscrew_left,
	large_corkscrew_right,
	small_turn_left_bank_to_gentle_up,
	small_turn_right_bank_to_gentle_up,
	launched_lift
	};



