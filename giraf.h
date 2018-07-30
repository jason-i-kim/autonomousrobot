//=====================================================================
// Includes
//=====================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"


//=====================================================================
// Definitions
//=====================================================================

#define WHEEL_DIAMETER   0.07	/* m 0.067*/
#define WHEEL_SEPARATION 0.26	/* m 0.2662 */

#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT 24902

#define BLACK_CALIB 0
#define WHITE_CALIB 255

#define MIN_IR_DIST_CALIB 223
#define MAX_IR_DIST_CALIB 92

#define MAX_SPEED 0.5
#define MAX_ACCELERATION 0.5


//=====================================================================
// Variable definitions
//=====================================================================

double visionpar[10];
double laserpar[10];
float ref_angle;
float reg_k;
float angle_cal;
int cnt,count,toggle;

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;


 symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

typedef struct{ 
		// Flag definitions
		int object_left, object_right, object_front;
		double linearr[8];
		double irarr[5];
		} sentype;

typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
		} odotype;


typedef struct{//input
		double left_pos,right_pos;
		// parameters
		//output
		double motorspeed_l,motorspeed_r; 
		// internal variables
		double startpos;
		double startr;
		double startl;
	       }motiontype;
	       
    

typedef struct{
                int state,oldstate;
		int time;
	       }smtype;

// SMR input/output data
symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

sentype sen;
odotype odo;
motiontype mot;
smtype mission;

enum {mot_stop,mot_move,mot_turn,mot_follow};
enum {ms_init,ms_fwd,ms_turn,ms_follow,ms_end};


//=====================================================================
// Functions
//=====================================================================

void sm_update(smtype *p);
void update_motcon(motiontype *p);	
void reset_odo(odotype *p);
void update_odo(odotype *p);

void reset_sen(sentype *p);
void update_sen(sentype *p);

void line_transform();
float lowest_line();

int fwd(double dist, double speed,int time,motiontype *p);
int turn(double angle, double speed,int time,motiontype *p);
void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv,camsrv;


//=====================================================================
// Sensor functions
//=====================================================================

void reset_sen(sentype * p)
{
  for(cnt = 0; cnt < 8; cnt++) {
    p->linearr[cnt] = 0;
  }
  for(cnt = 0; cnt < 5; cnt++) {
    p->irarr[cnt] = 0;
  }
  
  p->object_left = 0;
  p->object_front = 0;
  p->object_right = 0;
}

void update_sen(sentype *p)
{
  for(cnt = 0; cnt < 8; cnt++) {
    p->linearr[cnt] = (linesensor->data[cnt]-BLACK_CALIB)*1.0/(WHITE_CALIB-BLACK_CALIB);
  }
  for(cnt = 0; cnt < 5; cnt++) {
    p->irarr[cnt] = 1-(irsensor->data[cnt]-MAX_IR_DIST_CALIB)*1.0/(MIN_IR_DIST_CALIB-MAX_IR_DIST_CALIB);
  }
  
  //Left IR sensor object detection
  if(p->irarr[0] != 1)
    p->object_left = 1;
  else
    p->object_left = 0;
  
  //Right IR sensor object detection
  if(p->irarr[4] != 1)
    p->object_right = 1;
  else
    p->object_right = 0;
  
  //Center IR sensor object detection
  if(p->irarr[1] != 1 || p->irarr[2] != 1 || p->irarr[3] != 1)
    p->object_front = 1;
  else
    p->object_front = 0;
}


//=====================================================================
// Odometry functions
//=====================================================================

void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
}

void update_odo(odotype *p)
{
  int delta;
  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  
  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
}


//=====================================================================
// Movement functions
//=====================================================================

/*void update_motcon(motiontype *p){ 

  if (p->cmd !=0){
     
     p->finished=0;
     switch (p->cmd){
     case mot_stop:
       p->curcmd=mot_stop;
       break;
       case mot_move:
	  p->startpos=(p->left_pos+p->right_pos)/2;
	  p->curcmd=mot_move;
       break;
       
       case mot_turn:
         if (p->angle > 0) 
	    p->startpos=p->right_pos;
	 else
	    p->startpos=p->left_pos;
         p->curcmd=mot_turn;
       break;
     }
     
     p->cmd=0;
  }
   
   switch (p->curcmd){
     case mot_stop:
       p->motorspeed_l=0;
       p->motorspeed_r=0;
     break;
     
     case mot_move:   
       printf("%f %f %f %f %f %d %d %d\n",sen.irarr[0],sen.irarr[1],sen.irarr[2],sen.irarr[3],sen.irarr[4],sen.object_left,sen.object_front,sen.object_right);
       //p->motorspeed_l =MAX_SPEED-reg_k*lowest_line();
       //p->motorspeed_r =MAX_SPEED+reg_k*lowest_line();
       if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist){
          p->finished=1;
	  p->motorspeed_l=0;
          p->motorspeed_r=0;
       }
     break;
     
     case mot_turn:
       angle_cal = (p->right_pos-p->left_pos)/p->w;
       p->motorspeed_l =-reg_k*(ref_angle-angle_cal);
       p->motorspeed_r =reg_k*(ref_angle-angle_cal);
       if (angle_cal >= ref_angle*0.982){
	 p->motorspeed_r = 0;
	 p->motorspeed_l = 0;
         p->finished=1;
       }
     break;
   }
}   */


int fwd(double dist, double speed,int time,motiontype *p){    
   if (time==0){ 
     p->startl = p->left_pos;
     p->startr = p->right_pos;
     p->motorspeed_l = 0;
     p->motorspeed_r = 0;
   }
     
   if (sqrt(2*MAX_ACCELERATION*(dist-(p->left_pos-p->startl))) > speed){ 
      if (((speed - MAX_ACCELERATION/100) < p->motorspeed_l) && (p->motorspeed_l < speed)) {
	  p->motorspeed_l = speed;
	  p->motorspeed_r = p->motorspeed_l;
      }
      else if (p->motorspeed_l < speed) {
	  p->motorspeed_l += MAX_ACCELERATION/100;
	  p->motorspeed_r = p->motorspeed_l;
      }
    }
    else{
      p->motorspeed_l = sqrt(2*MAX_ACCELERATION*(dist-(p->left_pos-p->startl)));
      p->motorspeed_r = p->motorspeed_l;
    }
    
    if (p->left_pos - p->startl >= dist)
      return 1;
    else
      return 0;
}

int turn(double angle, double speed,int time,motiontype *p){
   //double angle_rad = 
   if (time==0){ 
     p->startl = p->left_pos;
     p->startr = p->right_pos;
     p->motorspeed_l = 0;
     p->motorspeed_r = 0;
   }
   
	if (angle>0){
	  if (p->right_pos-p->startr + p->startl-p->left_pos < angle*WHEEL_SEPARATION) {
	    if (p->motorspeed_r < sqrt(2*MAX_ACCELERATION*(angle*WHEEL_SEPARATION - (p->right_pos-p->startr + p->startl-p->left_pos)))) {
		if (((speed - MAX_ACCELERATION/100) < p->motorspeed_r) && (p->motorspeed_r < speed))  {
		    p->motorspeed_r = speed;
		    p->motorspeed_l = -p->motorspeed_r;
		}
		else if (p->motorspeed_r < speed) {
		  p->motorspeed_r += MAX_ACCELERATION/100;
		  p->motorspeed_l = -p->motorspeed_r;
		}
	    }
	     else {
		p->motorspeed_r = sqrt(2*MAX_ACCELERATION*(angle*WHEEL_SEPARATION - (p->right_pos-p->startr + p->startl-p->left_pos)));
		p->motorspeed_l = -p->motorspeed_r;
	     }
	     return 0;
	  }
	  else {
	    printf("%f\n",angle*WHEEL_SEPARATION-p->right_pos-p->startr + p->startl-p->left_pos);
            p->motorspeed_r=0;
            p->motorspeed_l=0;
	    return 1;
	  }
	}
	else {
	  if (p->startr-p->right_pos + p->left_pos-p->startl < fabs(angle)*WHEEL_SEPARATION) {
	    if (p->motorspeed_l < sqrt(2*MAX_ACCELERATION*(fabs(angle)*WHEEL_SEPARATION - (p->startr-p->right_pos + p->left_pos-p->startl)))) {
		if (((speed - MAX_ACCELERATION/100) < p->motorspeed_l) && (p->motorspeed_l < speed))  {
		    p->motorspeed_l = speed;
		    p->motorspeed_r = -p->motorspeed_l;
		}
		else if (p->motorspeed_r < speed) {
		  p->motorspeed_l += MAX_ACCELERATION/100;
		  p->motorspeed_r = -p->motorspeed_l;
		}
	    }
	     else {
		p->motorspeed_l = sqrt(2*MAX_ACCELERATION*(fabs(angle)*WHEEL_SEPARATION - (p->startr-p->right_pos + p->left_pos-p->startl)));
		p->motorspeed_r = -p->motorspeed_l;
	     }
	     return 0;
	  }
	  else {
            p->motorspeed_r=0;
            p->motorspeed_l=0;
	    return 1;
	  }
	}
}

//=====================================================================
// User functions
//=====================================================================

float lowest_line() {
  
  float dist[8] = {-35, -15, -5, -1, 1, 5, 15, 35};
  float numerator=0;
  float denominator=0;
  
  for(cnt = 0; cnt < 8; cnt++) {
    numerator += (1-sen.linearr[cnt]) * dist[cnt];
    denominator += (1-sen.linearr[cnt]);
  }
  
  return atan(numerator/denominator/21);  
}


