#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include "track.h"

//       y
// z     .     x
//    .  .  .
//       .
//


enum
{
SPRITE_GROUP_BASE=1,
SPRITE_GROUP_INLINE_TWIST=2,
SPRITE_GROUP_CORKSCREW=4,
SPRITE_GROUP_ZERO_G_ROLLS=8,
};

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
#define M_PI_8 (M_PI/8.0)
#define M_PI_12 (M_PI/12.0)

#define CORKSCREW_ANGLE_1 2.0 * M_PI_12
#define CORKSCREW_ANGLE_2 4.0 * M_PI_12
#define CORKSCREW_ANGLE_3 M_PI_2
#define CORKSCREW_ANGLE_4 8.0 * M_PI_12
#define CORKSCREW_ANGLE_5 10.0 * M_PI_12
#define G atan(1/sqrt(6)) //22 degrees
#define S atan(4/sqrt(6)) //58.5 degrees
#define V (0.5*M_PI) //90 degrees
#define FG (0.5*G) //11 degrees
#define GS (0.5*(G+S)) //40 degrees
#define SV (0.5*(V+S)) //74 degrees
#define GD atan(1/sqrt(12)) //16 degrees
#define FGD (0.5*GD) //8 degrees
#define SD atan(4/sqrt(12)) //49 degrees

#define CRY(angle) (atan2(0.5*(1-cos(angle)),1-0.5*(1-cos(angle))))
#define CRP(angle) (-asin(-sin(angle)/sqrt(2.0)))
#define CRR(angle) (atan2(sin(angle)/sqrt(2.0),cos(angle)))

#define CLY(angle) (-CRY(angle))
#define CLP(angle) (-CRP(-angle))
#define CLR(angle) (-CRR(angle))


//SPST
sprite_rotation_t base_sprite_rotations[708]=
	{
	//Flat sprites
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
	//Gentle slope sprites
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
	//Steep slope sprites
	{ 0,3,0,Y( 0), GS,0.0},
	{ 4,3,0,Y( 4), GS,0.0},
	{ 8,3,0,Y( 8), GS,0.0},
	{12,3,0,Y(12), GS,0.0},
	{16,3,0,Y(16), GS,0.0},
	{20,3,0,Y(20), GS,0.0},
	{24,3,0,Y(24), GS,0.0},
	{28,3,0,Y(28), GS,0.0},
	{ 0,7,0,Y( 0),-GS,0.0},
	{ 4,7,0,Y( 4),-GS,0.0},
	{ 8,7,0,Y( 8),-GS,0.0},
	{12,7,0,Y(12),-GS,0.0},
	{16,7,0,Y(16),-GS,0.0},
	{20,7,0,Y(20),-GS,0.0},
	{24,7,0,Y(24),-GS,0.0},
	{28,7,0,Y(28),-GS,0.0},
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
	//Diagonal sprites
	{4 ,50,0, Y(4), FGD,0.0},
	{12,50,0,Y(12), FGD,0.0},
	{20,50,0,Y(20), FGD,0.0},
	{28,50,0,Y(28), FGD,0.0},
	{4 ,53,0, Y(4),-FGD,0.0},
	{12,53,0,Y(12),-FGD,0.0},
	{20,53,0,Y(20),-FGD,0.0},
	{28,53,0,Y(28),-FGD,0.0},
	{4 ,51,0, Y(4), GD,0.0},
	{12,51,0,Y(12), GD,0.0},
	{20,51,0,Y(20), GD,0.0},
	{28,51,0,Y(28), GD,0.0},
	{4 ,54,0, Y(4),-GD,0.0},
	{12,54,0,Y(12),-GD,0.0},
	{20,54,0,Y(20),-GD,0.0},
	{28,54,0,Y(28),-GD,0.0},
	{4 ,52,0, Y(4), SD,0.0},
	{12,52,0,Y(12), SD,0.0},
	{20,52,0,Y(20), SD,0.0},
	{28,52,0,Y(28), SD,0.0},
	{4 ,55,0, Y(4),-SD,0.0},
	{12,55,0,Y(12),-SD,0.0},
	{20,55,0,Y(20),-SD,0.0},
	{28,55,0,Y(28),-SD,0.0},
	//Banked turn sprites
	{ 0,0,1,Y( 0),0, M_PI_8},
	{ 4,0,1,Y( 4),0, M_PI_8},
	{ 8,0,1,Y( 8),0, M_PI_8},
	{12,0,1,Y(12),0, M_PI_8},
	{16,0,1,Y(16),0, M_PI_8},
	{20,0,1,Y(20),0, M_PI_8},
	{24,0,1,Y(24),0, M_PI_8},
	{28,0,1,Y(28),0, M_PI_8},
	{ 0,0,3,Y( 0),0,-M_PI_8},
	{ 4,0,3,Y( 4),0,-M_PI_8},
	{ 8,0,3,Y( 8),0,-M_PI_8},
	{12,0,3,Y(12),0,-M_PI_8},
	{16,0,3,Y(16),0,-M_PI_8},
	{20,0,3,Y(20),0,-M_PI_8},
	{24,0,3,Y(24),0,-M_PI_8},
	{28,0,3,Y(28),0,-M_PI_8},
	{ 0,0,2,Y( 0),0,M_PI_4},
	{ 1,0,2,Y( 1),0,M_PI_4},
	{ 2,0,2,Y( 2),0,M_PI_4},
	{ 3,0,2,Y( 3),0,M_PI_4},
	{ 4,0,2,Y( 4),0,M_PI_4},
	{ 5,0,2,Y( 5),0,M_PI_4},
	{ 6,0,2,Y( 6),0,M_PI_4},
	{ 7,0,2,Y( 7),0,M_PI_4},
	{ 8,0,2,Y( 8),0,M_PI_4},
	{ 9,0,2,Y( 9),0,M_PI_4},
	{10,0,2,Y(10),0,M_PI_4},
	{11,0,2,Y(11),0,M_PI_4},
	{12,0,2,Y(12),0,M_PI_4},
	{13,0,2,Y(13),0,M_PI_4},
	{14,0,2,Y(14),0,M_PI_4},
	{15,0,2,Y(15),0,M_PI_4},
	{16,0,2,Y(16),0,M_PI_4},
	{17,0,2,Y(17),0,M_PI_4},
	{18,0,2,Y(18),0,M_PI_4},
	{19,0,2,Y(19),0,M_PI_4},
	{20,0,2,Y(20),0,M_PI_4},
	{21,0,2,Y(21),0,M_PI_4},
	{22,0,2,Y(22),0,M_PI_4},
	{23,0,2,Y(23),0,M_PI_4},
	{24,0,2,Y(24),0,M_PI_4},
	{25,0,2,Y(25),0,M_PI_4},
	{26,0,2,Y(26),0,M_PI_4},
	{27,0,2,Y(27),0,M_PI_4},
	{28,0,2,Y(28),0,M_PI_4},
	{29,0,2,Y(29),0,M_PI_4},
	{30,0,2,Y(30),0,M_PI_4},
	{31,0,2,Y(31),0,M_PI_4},
	{ 0,0,4,Y( 0),0,-M_PI_4},
	{ 1,0,4,Y( 1),0,-M_PI_4},
	{ 2,0,4,Y( 2),0,-M_PI_4},
	{ 3,0,4,Y( 3),0,-M_PI_4},
	{ 4,0,4,Y( 4),0,-M_PI_4},
	{ 5,0,4,Y( 5),0,-M_PI_4},
	{ 6,0,4,Y( 6),0,-M_PI_4},
	{ 7,0,4,Y( 7),0,-M_PI_4},
	{ 8,0,4,Y( 8),0,-M_PI_4},
	{ 9,0,4,Y( 9),0,-M_PI_4},
	{10,0,4,Y(10),0,-M_PI_4},
	{11,0,4,Y(11),0,-M_PI_4},
	{12,0,4,Y(12),0,-M_PI_4},
	{13,0,4,Y(13),0,-M_PI_4},
	{14,0,4,Y(14),0,-M_PI_4},
	{15,0,4,Y(15),0,-M_PI_4},
	{16,0,4,Y(16),0,-M_PI_4},
	{17,0,4,Y(17),0,-M_PI_4},
	{18,0,4,Y(18),0,-M_PI_4},
	{19,0,4,Y(19),0,-M_PI_4},
	{20,0,4,Y(20),0,-M_PI_4},
	{21,0,4,Y(21),0,-M_PI_4},
	{22,0,4,Y(22),0,-M_PI_4},
	{23,0,4,Y(23),0,-M_PI_4},
	{24,0,4,Y(24),0,-M_PI_4},
	{25,0,4,Y(25),0,-M_PI_4},
	{26,0,4,Y(26),0,-M_PI_4},
	{27,0,4,Y(27),0,-M_PI_4},
	{28,0,4,Y(28),0,-M_PI_4},
	{29,0,4,Y(29),0,-M_PI_4},
	{30,0,4,Y(30),0,-M_PI_4},
	{31,0,4,Y(31),0,-M_PI_4},
	//Slope bank transition
	{ 0,1,1,Y( 0),FG,M_PI_8},
	{ 1,1,1,Y( 1),FG,M_PI_8},
	{ 2,1,1,Y( 2),FG,M_PI_8},
	{ 3,1,1,Y( 3),FG,M_PI_8},
	{ 4,1,1,Y( 4),FG,M_PI_8},
	{ 5,1,1,Y( 5),FG,M_PI_8},
	{ 6,1,1,Y( 6),FG,M_PI_8},
	{ 7,1,1,Y( 7),FG,M_PI_8},
	{ 8,1,1,Y( 8),FG,M_PI_8},
	{ 9,1,1,Y( 9),FG,M_PI_8},
	{10,1,1,Y(10),FG,M_PI_8},
	{11,1,1,Y(11),FG,M_PI_8},
	{12,1,1,Y(12),FG,M_PI_8},
	{13,1,1,Y(13),FG,M_PI_8},
	{14,1,1,Y(14),FG,M_PI_8},
	{15,1,1,Y(15),FG,M_PI_8},
	{16,1,1,Y(16),FG,M_PI_8},
	{17,1,1,Y(17),FG,M_PI_8},
	{18,1,1,Y(18),FG,M_PI_8},
	{19,1,1,Y(19),FG,M_PI_8},
	{20,1,1,Y(20),FG,M_PI_8},
	{21,1,1,Y(21),FG,M_PI_8},
	{22,1,1,Y(22),FG,M_PI_8},
	{23,1,1,Y(23),FG,M_PI_8},
	{24,1,1,Y(24),FG,M_PI_8},
	{25,1,1,Y(25),FG,M_PI_8},
	{26,1,1,Y(26),FG,M_PI_8},
	{27,1,1,Y(27),FG,M_PI_8},
	{28,1,1,Y(28),FG,M_PI_8},
	{29,1,1,Y(29),FG,M_PI_8},
	{30,1,1,Y(30),FG,M_PI_8},
	{31,1,1,Y(31),FG,M_PI_8},
	{ 0,1,3,Y( 0),FG,-M_PI_8},
	{ 1,1,3,Y( 1),FG,-M_PI_8},
	{ 2,1,3,Y( 2),FG,-M_PI_8},
	{ 3,1,3,Y( 3),FG,-M_PI_8},
	{ 4,1,3,Y( 4),FG,-M_PI_8},
	{ 5,1,3,Y( 5),FG,-M_PI_8},
	{ 6,1,3,Y( 6),FG,-M_PI_8},
	{ 7,1,3,Y( 7),FG,-M_PI_8},
	{ 8,1,3,Y( 8),FG,-M_PI_8},
	{ 9,1,3,Y( 9),FG,-M_PI_8},
	{10,1,3,Y(10),FG,-M_PI_8},
	{11,1,3,Y(11),FG,-M_PI_8},
	{12,1,3,Y(12),FG,-M_PI_8},
	{13,1,3,Y(13),FG,-M_PI_8},
	{14,1,3,Y(14),FG,-M_PI_8},
	{15,1,3,Y(15),FG,-M_PI_8},
	{16,1,3,Y(16),FG,-M_PI_8},
	{17,1,3,Y(17),FG,-M_PI_8},
	{18,1,3,Y(18),FG,-M_PI_8},
	{19,1,3,Y(19),FG,-M_PI_8},
	{20,1,3,Y(20),FG,-M_PI_8},
	{21,1,3,Y(21),FG,-M_PI_8},
	{22,1,3,Y(22),FG,-M_PI_8},
	{23,1,3,Y(23),FG,-M_PI_8},
	{24,1,3,Y(24),FG,-M_PI_8},
	{25,1,3,Y(25),FG,-M_PI_8},
	{26,1,3,Y(26),FG,-M_PI_8},
	{27,1,3,Y(27),FG,-M_PI_8},
	{28,1,3,Y(28),FG,-M_PI_8},
	{29,1,3,Y(29),FG,-M_PI_8},
	{30,1,3,Y(30),FG,-M_PI_8},
	{31,1,3,Y(31),FG,-M_PI_8},
	{ 0,5,1,Y( 0),-FG,M_PI_8},
	{ 1,5,1,Y( 1),-FG,M_PI_8},
	{ 2,5,1,Y( 2),-FG,M_PI_8},
	{ 3,5,1,Y( 3),-FG,M_PI_8},
	{ 4,5,1,Y( 4),-FG,M_PI_8},
	{ 5,5,1,Y( 5),-FG,M_PI_8},
	{ 6,5,1,Y( 6),-FG,M_PI_8},
	{ 7,5,1,Y( 7),-FG,M_PI_8},
	{ 8,5,1,Y( 8),-FG,M_PI_8},
	{ 9,5,1,Y( 9),-FG,M_PI_8},
	{10,5,1,Y(10),-FG,M_PI_8},
	{11,5,1,Y(11),-FG,M_PI_8},
	{12,5,1,Y(12),-FG,M_PI_8},
	{13,5,1,Y(13),-FG,M_PI_8},
	{14,5,1,Y(14),-FG,M_PI_8},
	{15,5,1,Y(15),-FG,M_PI_8},
	{16,5,1,Y(16),-FG,M_PI_8},
	{17,5,1,Y(17),-FG,M_PI_8},
	{18,5,1,Y(18),-FG,M_PI_8},
	{19,5,1,Y(19),-FG,M_PI_8},
	{20,5,1,Y(20),-FG,M_PI_8},
	{21,5,1,Y(21),-FG,M_PI_8},
	{22,5,1,Y(22),-FG,M_PI_8},
	{23,5,1,Y(23),-FG,M_PI_8},
	{24,5,1,Y(24),-FG,M_PI_8},
	{25,5,1,Y(25),-FG,M_PI_8},
	{26,5,1,Y(26),-FG,M_PI_8},
	{27,5,1,Y(27),-FG,M_PI_8},
	{28,5,1,Y(28),-FG,M_PI_8},
	{29,5,1,Y(29),-FG,M_PI_8},
	{30,5,1,Y(30),-FG,M_PI_8},
	{31,5,1,Y(31),-FG,M_PI_8},
	{ 0,5,3,Y( 0),-FG,-M_PI_8},
	{ 1,5,3,Y( 1),-FG,-M_PI_8},
	{ 2,5,3,Y( 2),-FG,-M_PI_8},
	{ 3,5,3,Y( 3),-FG,-M_PI_8},
	{ 4,5,3,Y( 4),-FG,-M_PI_8},
	{ 5,5,3,Y( 5),-FG,-M_PI_8},
	{ 6,5,3,Y( 6),-FG,-M_PI_8},
	{ 7,5,3,Y( 7),-FG,-M_PI_8},
	{ 8,5,3,Y( 8),-FG,-M_PI_8},
	{ 9,5,3,Y( 9),-FG,-M_PI_8},
	{10,5,3,Y(10),-FG,-M_PI_8},
	{11,5,3,Y(11),-FG,-M_PI_8},
	{12,5,3,Y(12),-FG,-M_PI_8},
	{13,5,3,Y(13),-FG,-M_PI_8},
	{14,5,3,Y(14),-FG,-M_PI_8},
	{15,5,3,Y(15),-FG,-M_PI_8},
	{16,5,3,Y(16),-FG,-M_PI_8},
	{17,5,3,Y(17),-FG,-M_PI_8},
	{18,5,3,Y(18),-FG,-M_PI_8},
	{19,5,3,Y(19),-FG,-M_PI_8},
	{20,5,3,Y(20),-FG,-M_PI_8},
	{21,5,3,Y(21),-FG,-M_PI_8},
	{22,5,3,Y(22),-FG,-M_PI_8},
	{23,5,3,Y(23),-FG,-M_PI_8},
	{24,5,3,Y(24),-FG,-M_PI_8},
	{25,5,3,Y(25),-FG,-M_PI_8},
	{26,5,3,Y(26),-FG,-M_PI_8},
	{27,5,3,Y(27),-FG,-M_PI_8},
	{28,5,3,Y(28),-FG,-M_PI_8},
	{29,5,3,Y(29),-FG,-M_PI_8},
	{30,5,3,Y(30),-FG,-M_PI_8},
	{31,5,3,Y(31),-FG,-M_PI_8},
	//Diagonal bank transition
	{4 ,50,0, Y(4), FGD,M_PI_8},
	{12,50,0,Y(12), FGD,M_PI_8},
	{20,50,0,Y(20), FGD,M_PI_8},
	{28,50,0,Y(28), FGD,M_PI_8},
	{4 ,50,0, Y(4), FGD,-M_PI_8},
	{12,50,0,Y(12), FGD,-M_PI_8},
	{20,50,0,Y(20), FGD,-M_PI_8},
	{28,50,0,Y(28), FGD,-M_PI_8},
	{4 ,53,0, Y(4),-FGD,M_PI_8},
	{12,53,0,Y(12),-FGD,M_PI_8},
	{20,53,0,Y(20),-FGD,M_PI_8},
	{28,53,0,Y(28),-FGD,M_PI_8},
	{4 ,53,0, Y(4),-FGD,-M_PI_8},
	{12,53,0,Y(12),-FGD,-M_PI_8},
	{20,53,0,Y(20),-FGD,-M_PI_8},
	{28,53,0,Y(28),-FGD,-M_PI_8},
	//Sloped bank transition
	{ 0,2,0,Y( 0), G,M_PI_8},
	{ 8,2,0,Y( 8), G,M_PI_8},
	{16,2,0,Y(16), G,M_PI_8},
	{24,2,0,Y(24), G,M_PI_8},
	{ 0,2,0,Y( 0), G,-M_PI_8},
	{ 8,2,0,Y( 8), G,-M_PI_8},
	{16,2,0,Y(16), G,-M_PI_8},
	{24,2,0,Y(24), G,-M_PI_8},
	{ 0,6,0,Y( 0),-G,M_PI_8},
	{ 8,6,0,Y( 8),-G,M_PI_8},
	{16,6,0,Y(16),-G,M_PI_8},
	{24,6,0,Y(24),-G,M_PI_8},
	{ 0,6,0,Y( 0),-G,-M_PI_8},
	{ 8,6,0,Y( 8),-G,-M_PI_8},
	{16,6,0,Y(16),-G,-M_PI_8},
	{24,6,0,Y(24),-G,-M_PI_8},
	//Sloped banked turn
	{ 0,2,2,Y( 0),G,M_PI_4},
	{ 1,2,2,Y( 1),G,M_PI_4},
	{ 2,2,2,Y( 2),G,M_PI_4},
	{ 3,2,2,Y( 3),G,M_PI_4},
	{ 4,2,2,Y( 4),G,M_PI_4},
	{ 5,2,2,Y( 5),G,M_PI_4},
	{ 6,2,2,Y( 6),G,M_PI_4},
	{ 7,2,2,Y( 7),G,M_PI_4},
	{ 8,2,2,Y( 8),G,M_PI_4},
	{ 9,2,2,Y( 9),G,M_PI_4},
	{10,2,2,Y(10),G,M_PI_4},
	{11,2,2,Y(11),G,M_PI_4},
	{12,2,2,Y(12),G,M_PI_4},
	{13,2,2,Y(13),G,M_PI_4},
	{14,2,2,Y(14),G,M_PI_4},
	{15,2,2,Y(15),G,M_PI_4},
	{16,2,2,Y(16),G,M_PI_4},
	{17,2,2,Y(17),G,M_PI_4},
	{18,2,2,Y(18),G,M_PI_4},
	{19,2,2,Y(19),G,M_PI_4},
	{20,2,2,Y(20),G,M_PI_4},
	{21,2,2,Y(21),G,M_PI_4},
	{22,2,2,Y(22),G,M_PI_4},
	{23,2,2,Y(23),G,M_PI_4},
	{24,2,2,Y(24),G,M_PI_4},
	{25,2,2,Y(25),G,M_PI_4},
	{26,2,2,Y(26),G,M_PI_4},
	{27,2,2,Y(27),G,M_PI_4},
	{28,2,2,Y(28),G,M_PI_4},
	{29,2,2,Y(29),G,M_PI_4},
	{30,2,2,Y(30),G,M_PI_4},
	{31,2,2,Y(31),G,M_PI_4},
	{ 0,2,4,Y( 0),G,-M_PI_4},
	{ 1,2,4,Y( 1),G,-M_PI_4},
	{ 2,2,4,Y( 2),G,-M_PI_4},
	{ 3,2,4,Y( 3),G,-M_PI_4},
	{ 4,2,4,Y( 4),G,-M_PI_4},
	{ 5,2,4,Y( 5),G,-M_PI_4},
	{ 6,2,4,Y( 6),G,-M_PI_4},
	{ 7,2,4,Y( 7),G,-M_PI_4},
	{ 8,2,4,Y( 8),G,-M_PI_4},
	{ 9,2,4,Y( 9),G,-M_PI_4},
	{10,2,4,Y(10),G,-M_PI_4},
	{11,2,4,Y(11),G,-M_PI_4},
	{12,2,4,Y(12),G,-M_PI_4},
	{13,2,4,Y(13),G,-M_PI_4},
	{14,2,4,Y(14),G,-M_PI_4},
	{15,2,4,Y(15),G,-M_PI_4},
	{16,2,4,Y(16),G,-M_PI_4},
	{17,2,4,Y(17),G,-M_PI_4},
	{18,2,4,Y(18),G,-M_PI_4},
	{19,2,4,Y(19),G,-M_PI_4},
	{20,2,4,Y(20),G,-M_PI_4},
	{21,2,4,Y(21),G,-M_PI_4},
	{22,2,4,Y(22),G,-M_PI_4},
	{23,2,4,Y(23),G,-M_PI_4},
	{24,2,4,Y(24),G,-M_PI_4},
	{25,2,4,Y(25),G,-M_PI_4},
	{26,2,4,Y(26),G,-M_PI_4},
	{27,2,4,Y(27),G,-M_PI_4},
	{28,2,4,Y(28),G,-M_PI_4},
	{29,2,4,Y(29),G,-M_PI_4},
	{30,2,4,Y(30),G,-M_PI_4},
	{31,2,4,Y(31),G,-M_PI_4},
	{ 0,6,2,Y( 0),-G,M_PI_4},
	{ 1,6,2,Y( 1),-G,M_PI_4},
	{ 2,6,2,Y( 2),-G,M_PI_4},
	{ 3,6,2,Y( 3),-G,M_PI_4},
	{ 4,6,2,Y( 4),-G,M_PI_4},
	{ 5,6,2,Y( 5),-G,M_PI_4},
	{ 6,6,2,Y( 6),-G,M_PI_4},
	{ 7,6,2,Y( 7),-G,M_PI_4},
	{ 8,6,2,Y( 8),-G,M_PI_4},
	{ 9,6,2,Y( 9),-G,M_PI_4},
	{10,6,2,Y(10),-G,M_PI_4},
	{11,6,2,Y(11),-G,M_PI_4},
	{12,6,2,Y(12),-G,M_PI_4},
	{13,6,2,Y(13),-G,M_PI_4},
	{14,6,2,Y(14),-G,M_PI_4},
	{15,6,2,Y(15),-G,M_PI_4},
	{16,6,2,Y(16),-G,M_PI_4},
	{17,6,2,Y(17),-G,M_PI_4},
	{18,6,2,Y(18),-G,M_PI_4},
	{19,6,2,Y(19),-G,M_PI_4},
	{20,6,2,Y(20),-G,M_PI_4},
	{21,6,2,Y(21),-G,M_PI_4},
	{22,6,2,Y(22),-G,M_PI_4},
	{23,6,2,Y(23),-G,M_PI_4},
	{24,6,2,Y(24),-G,M_PI_4},
	{25,6,2,Y(25),-G,M_PI_4},
	{26,6,2,Y(26),-G,M_PI_4},
	{27,6,2,Y(27),-G,M_PI_4},
	{28,6,2,Y(28),-G,M_PI_4},
	{29,6,2,Y(29),-G,M_PI_4},
	{30,6,2,Y(30),-G,M_PI_4},
	{31,6,2,Y(31),-G,M_PI_4},
	{ 0,6,4,Y( 0),-G,-M_PI_4},
	{ 1,6,4,Y( 1),-G,-M_PI_4},
	{ 2,6,4,Y( 2),-G,-M_PI_4},
	{ 3,6,4,Y( 3),-G,-M_PI_4},
	{ 4,6,4,Y( 4),-G,-M_PI_4},
	{ 5,6,4,Y( 5),-G,-M_PI_4},
	{ 6,6,4,Y( 6),-G,-M_PI_4},
	{ 7,6,4,Y( 7),-G,-M_PI_4},
	{ 8,6,4,Y( 8),-G,-M_PI_4},
	{ 9,6,4,Y( 9),-G,-M_PI_4},
	{10,6,4,Y(10),-G,-M_PI_4},
	{11,6,4,Y(11),-G,-M_PI_4},
	{12,6,4,Y(12),-G,-M_PI_4},
	{13,6,4,Y(13),-G,-M_PI_4},
	{14,6,4,Y(14),-G,-M_PI_4},
	{15,6,4,Y(15),-G,-M_PI_4},
	{16,6,4,Y(16),-G,-M_PI_4},
	{17,6,4,Y(17),-G,-M_PI_4},
	{18,6,4,Y(18),-G,-M_PI_4},
	{19,6,4,Y(19),-G,-M_PI_4},
	{20,6,4,Y(20),-G,-M_PI_4},
	{21,6,4,Y(21),-G,-M_PI_4},
	{22,6,4,Y(22),-G,-M_PI_4},
	{23,6,4,Y(23),-G,-M_PI_4},
	{24,6,4,Y(24),-G,-M_PI_4},
	{25,6,4,Y(25),-G,-M_PI_4},
	{26,6,4,Y(26),-G,-M_PI_4},
	{27,6,4,Y(27),-G,-M_PI_4},
	{28,6,4,Y(28),-G,-M_PI_4},
	{29,6,4,Y(29),-G,-M_PI_4},
	{30,6,4,Y(30),-G,-M_PI_4},
	{31,6,4,Y(31),-G,-M_PI_4},
	//Banked slope transition
	{ 0,1,2,Y( 0), FG, M_PI_4},
	{ 8,1,2,Y( 8), FG, M_PI_4},
	{16,1,2,Y(16), FG, M_PI_4},
	{24,1,2,Y(24), FG, M_PI_4},
	{ 0,1,4,Y( 0), FG,-M_PI_4},
	{ 8,1,4,Y( 8), FG,-M_PI_4},
	{16,1,4,Y(16), FG,-M_PI_4},
	{24,1,4,Y(24), FG,-M_PI_4},
	{ 0,5,2,Y( 0),-FG, M_PI_4},
	{ 8,5,2,Y( 8),-FG, M_PI_4},
	{16,5,2,Y(16),-FG, M_PI_4},
	{24,5,2,Y(24),-FG, M_PI_4},
	{ 0,5,4,Y( 0),-FG,-M_PI_4},
	{ 8,5,4,Y( 8),-FG,-M_PI_4},
	{16,5,4,Y(16),-FG,-M_PI_4},
	{24,5,4,Y(24),-FG,-M_PI_4}
};

sprite_rotation_t inline_twist_sprite_rotations[40]=
	{
	//Inline twist sprites
	{ 0,0,5,Y(0),0,3*M_PI_8},
	{ 8,0,5,Y(8),0,3*M_PI_8},
	{16,0,5,Y(16),0,3*M_PI_8},
	{24,0,5,Y(24),0,3*M_PI_8},
	{ 0,0,10,Y(0),0,-3*M_PI_8},
	{ 8,0,10,Y(8),0,-3*M_PI_8},
	{16,0,10,Y(16),0,-3*M_PI_8},
	{24,0,10,Y(24),0,-3*M_PI_8},
	{ 0,0,6,Y(0),0,4*M_PI_8},
	{ 8,0,6,Y(8),0,4*M_PI_8},
	{16,0,6,Y(16),0,4*M_PI_8},
	{24,0,6,Y(24),0,4*M_PI_8},
	{ 0,0,11,Y(0),0,-4*M_PI_8},
	{ 8,0,11,Y(8),0,-4*M_PI_8},
	{16,0,11,Y(16),0,-4*M_PI_8},
	{24,0,11,Y(24),0,-4*M_PI_8},
	{ 0,0,7,Y(0),0,5*M_PI_8},
	{ 8,0,7,Y(8),0,5*M_PI_8},
	{16,0,7,Y(16),0,5*M_PI_8},
	{24,0,7,Y(24),0,5*M_PI_8},
	{ 0,0,12,Y(0),0,-5*M_PI_8},
	{ 8,0,12,Y(8),0,-5*M_PI_8},
	{16,0,12,Y(16),0,-5*M_PI_8},
	{24,0,12,Y(24),0,-5*M_PI_8},
	{ 0,0,8,Y(0),0,6*M_PI_8},
	{ 8,0,8,Y(8),0,6*M_PI_8},
	{16,0,8,Y(16),0,6*M_PI_8},
	{24,0,8,Y(24),0,6*M_PI_8},
	{ 0,0,13,Y(0),0,-6*M_PI_8},
	{ 8,0,13,Y(8),0,-6*M_PI_8},
	{16,0,13,Y(16),0,-6*M_PI_8},
	{24,0,13,Y(24),0,-6*M_PI_8},
	{ 0,0,9,Y(0),0,7*M_PI_8},
	{ 8,0,9,Y(8),0,7*M_PI_8},
	{16,0,9,Y(16),0,7*M_PI_8},
	{24,0,9,Y(24),0,7*M_PI_8},
	{ 0,0,14,Y(0),0,-7*M_PI_8},
	{ 8,0,14,Y(8),0,-7*M_PI_8},
	{16,0,14,Y(16),0,-7*M_PI_8},
	{24,0,14,Y(24),0,-7*M_PI_8}
	};

sprite_rotation_t corkscrew_sprite_rotations[80]=
	{
	//Corkscrew sprites
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
	{24,33,0,Y(24)+CLY(-5*M_PI_6),CLP(-5*M_PI_6),CLR(-5*M_PI_6)}
};

//Zero G rolls

#define NUM_SPRITE_GROUPS 4
int sprite_group_counts[NUM_SPRITE_GROUPS]={708,40,80};
sprite_rotation_t* sprite_group_rotations[NUM_SPRITE_GROUPS]={base_sprite_rotations,inline_twist_sprite_rotations};




matrix_t track_point_get_rotation(track_point_t point)
{
return matrix(
point.tangent.z,point.normal.z,-point.binormal.z,
point.tangent.y,point.normal.y,-point.binormal.y,
point.tangent.x,point.normal.x,-point.binormal.x);
}

float get_rotation_distance(matrix_t a,matrix_t b)
{
matrix_t diff=matrix_mult(a,matrix_transpose(b));
return acos((diff.entries[0]+diff.entries[4]+diff.entries[8]-1.0)/2.0);
}

sprite_rotation_t get_closest_rotation(matrix_t rotation,int groups)
{
float min_dist=1000.0;
int min_group=0;
int min_index=0;
	for(int j=0;j<NUM_SPRITE_GROUPS;j++)
	{
		if(!(groups&(1<<j)))continue;
	sprite_rotation_t* sprite_rotations=sprite_group_rotations[j];
	for(int i=0;i<sprite_group_counts[j];i++)
	{
	matrix_t candidate_rotation=matrix_mult(matrix_mult(rotate_y(-sprite_rotations[i].yaw),rotate_z(sprite_rotations[i].pitch)),rotate_x(sprite_rotations[i].roll));
	float dist=get_rotation_distance(rotation,candidate_rotation);
		if(dist<min_dist)
		{
		min_dist=dist;
		min_group=j;
		min_index=i;
		}
	}
	}
return sprite_group_rotations[min_group][min_index];
}

//Reverse value is difference in height (in pixels) between final tile element base and track element end +1
//Inverted track is 3 pixels higher than upright track of same height

//Generate the subposition data for a track piece
void generate_view_subposition_data(track_section_t* track_section,char* name,int groups,int view,int reverse)
{
int length=(int)floor(0.5+32.0*(track_section->length/TILE_SIZE));


track_point_t end=track_section->curve(track_section->length);
float finish_angle=M_PI_2*roundf(2.0*atan2(-end.tangent.x,-end.tangent.z)/M_PI);
//printf("Angle %f\n",finish_angle);
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
	int z=(int)floor(0.5+(16*sqrt(6))*point.position.y/TILE_SIZE)+reverse-1;
	//matrix_t r=rotate_y(0.5*M_PI);
	//printf("%.2f\t%.2f %.2f\n",r.entries[0],r.entries[1],r.entries[2]);
	//printf("%.2f\t%.2f %.2f\n",r.entries[3],r.entries[4],r.entries[5]);
	//printf("%.2f\t%.2f %.2f\n\n",r.entries[6],r.entries[7],r.entries[8]);
	//r=track_point_get_rotation(point);
	//printf("%.2f\t%.2f %.2f\n",r.entries[0],r.entries[1],r.entries[2]);
	//printf("%.2f\t%.2f %.2f\n",r.entries[3],r.entries[4],r.entries[5]);
	//printf("%.2f\t%.2f %.2f\n\n",r.entries[6],r.entries[7],r.entries[8]);

	sprite_rotation_t rotation=get_closest_rotation(track_point_get_rotation(point),groups);

	
	
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

void generate_subposition_data(track_section_t* track_section,char* name,int groups,int reverse)
{
	for(int i=0;i<4;i++)
	{
	generate_view_subposition_data(track_section,name,groups,i,reverse);
	}
}



int main(int argc,char** argv)
{
/*
generate_subposition_data(&(track_list_default.zero_g_roll_left),"LeftZeroGRollUp",0);
generate_subposition_data(&(track_list_default.zero_g_roll_right),"RightZeroGRollUp",0);
generate_subposition_data(&(track_list_default.zero_g_roll_left),"LeftZeroGRollDown",20);
generate_subposition_data(&(track_list_default.zero_g_roll_right),"RightZeroGRollDown",20);
//generate_subposition_data(&(track_list_default.medium_half_loop_left),"LeftMediumHalfLoopUp",0);
//generate_subposition_data(&(track_list_default.medium_half_loop_right),"RightMediumHalfLoopUp",0);
//generate_subposition_data(&(track_list_default.medium_half_loop_right),"LeftMediumHalfLoopDown",36);
//generate_subposition_data(&(track_list_default.medium_half_loop_left),"RightMediumHalfLoopDown",36);
*/

generate_subposition_data(&(track_list_default.large_zero_g_roll_left),"LeftLargeZeroGRollUp",SPRITE_GROUP_BASE,0);
/*
generate_subposition_data(&(track_list_default.large_zero_g_roll_right),"RightLargeZeroGRollUp",0);
generate_subposition_data(&(track_list_default.large_zero_g_roll_left),"LeftLargeZeroGRollDown",20);
generate_subposition_data(&(track_list_default.large_zero_g_roll_right),"RightLargeZeroGRollDown",20);
*/

return 0;
}
