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
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT 24902

#define MAX_ACCELERATION 0.5

//Odometry definitions
#define WHEEL_DIAMETER   0.067	/* m 0.067*/
#define WHEEL_SEPARATION 0.2662	/* m 0.2662 */

//Line following definitions
#define BLACK_CALIB 0
#define WHITE_CALIB 255
#define LINE_DETECTION_LIMIT 0.9
#define reg_k 2

//IR definitions
#define OBJECT_DETECTION_RANGE 60
#define OBJECT_DETECTION_NEAR_RANGE 20
#define K_a 1632.5
#define K_b 73.6269

//Command definitions
#define COMMAND_CHAR_BUFFER_SIZE 15
#define br 0
#define bl 1

//=====================================================================
// Variable definitions
//=====================================================================

double visionpar[10];
double laserpar[10];
double MAX_SPEED;
int cnt, ignoreobstacles;

//State variable
enum {state_read,state_execute}state;
enum {cmd_turn,cmd_fwd,cmd_br_irfront,cmd_br_irleft,cmd_br_cross,cmd_wm,cmd_drive_ir,cmd_drive_cross,cmd_eval}cmd;
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
		int object_left, object_right, object_front; //Object detection flags
		int object_left_near, object_right_near, object_front_near; //Object detection near flag
		int line_flag;//0 = no lines, 1 = 1 line, 2 = 2 lines, 3 = crossing line
		int line_cnt;//Line counting variable
		int line_centers[2]; //Detected line centers
		double irarr[5]; //IR sensor array -> 4=right, 0=left
		double linearr[8]; //Line detector array -> 0=right, 7=left
		float dist[8];//Line sensor distance
		float numerator;
		float denominator;
		float CM;//Center of mass
		} sentype;

typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos,right_pos_old,left_pos_old;
		double theta, distance, y;
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

// SMR input/output data
symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

sentype sen;
odotype odo;
motiontype mot;


//=====================================================================
// Functions
//=====================================================================

void update_motcon(motiontype *p);
void reset_odo(odotype *p);
void update_odo(odotype *p);

void reset_sen(sentype *p);
void update_sen(sentype *p);

int fwd(double dist, double speed,int time,motiontype *p,sentype *s);
int turn(double angle, double speed,int time,motiontype *p);
void followline(double speed, int operation,motiontype *m,sentype *p);
void drive(double speed,int time,motiontype *p, sentype *s);
void stop(motiontype *p);

int char2int(char cmdbuffer[1000]);

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
  p->object_left_near = 0;
  p->object_front_near = 0;
  p->object_right_near = 0;
  p->dist[0] = -7;
  p->dist[1] = -5;
  p->dist[2] = -3;
  p->dist[3] = -1;
  p->dist[4] = 1;
  p->dist[5] = 3;
  p->dist[6] = 5;
  p->dist[7] = 7;
  p->numerator=0;
  p->denominator=0;
  p->CM = 0;
  p->line_flag = 1;
  p->line_cnt = 0;
  p->line_centers[0] = 0;
  p->line_centers[1] = 0;
}

void update_sen(sentype *p)
{
  //=====================================================================
  // IR sensor update
  //=====================================================================
  for(cnt = 0; cnt < 5; cnt++) {
    p->irarr[cnt] = K_a/(irsensor->data[cnt]-K_b);
  }
  
  //Left IR sensor object detection
  if(p->irarr[0] <= OBJECT_DETECTION_RANGE)
    p->object_left = 1;
  else
    p->object_left = 0;
  
  //Right IR sensor object detection
  if(p->irarr[4] <= OBJECT_DETECTION_RANGE)
    p->object_right = 1;
  else
    p->object_right = 0;
  
  //Center IR sensor object detection
  if(p->irarr[1] <= OBJECT_DETECTION_RANGE || p->irarr[2] <= OBJECT_DETECTION_RANGE || p->irarr[3] <= OBJECT_DETECTION_RANGE)
    p->object_front = 1;
  else
    p->object_front = 0;
  
  //Left IR sensor object detection near
  if(p->irarr[0] <= OBJECT_DETECTION_NEAR_RANGE)
    p->object_left_near = 1;
  else
    p->object_left_near = 0;
  
  //Right IR sensor object detection near
  if(p->irarr[4] <= OBJECT_DETECTION_NEAR_RANGE)
    p->object_right_near = 1;
  else
    p->object_right_near = 0;
  
  //Center IR sensor object detection near
  if(p->irarr[1] <= OBJECT_DETECTION_NEAR_RANGE || p->irarr[2] <= OBJECT_DETECTION_NEAR_RANGE || p->irarr[3] <= OBJECT_DETECTION_NEAR_RANGE)
    p->object_front_near = 1;
  else
    p->object_front_near = 0;
  
  //=====================================================================
  // Line sensor update
  //=====================================================================
  p->line_cnt = 0;
  p->line_centers[0] = 0;
  p->line_centers[1] = 0;

  for(cnt = 0; cnt < 8; cnt++) {
    p->linearr[cnt] = (linesensor->data[cnt]-BLACK_CALIB)*1.0/(WHITE_CALIB-BLACK_CALIB);
  }
  if(p->linearr[0] > LINE_DETECTION_LIMIT && p->linearr[1] > LINE_DETECTION_LIMIT && p->linearr[2] > LINE_DETECTION_LIMIT && p->linearr[3] > LINE_DETECTION_LIMIT && p->linearr[4] > LINE_DETECTION_LIMIT && p->linearr[5] > LINE_DETECTION_LIMIT && p->linearr[6] > LINE_DETECTION_LIMIT && p->linearr[7] > LINE_DETECTION_LIMIT){
    p->line_flag = 0;
  }
  else if(p->linearr[0] <= LINE_DETECTION_LIMIT && p->linearr[1] <= LINE_DETECTION_LIMIT && p->linearr[2] <= LINE_DETECTION_LIMIT && p->linearr[3] <= LINE_DETECTION_LIMIT && p->linearr[4] <= LINE_DETECTION_LIMIT && p->linearr[5] <= LINE_DETECTION_LIMIT && p->linearr[6] <= LINE_DETECTION_LIMIT && p->linearr[7] <= LINE_DETECTION_LIMIT){
    p->line_flag = 3;
  }
  else{	
    if(p->linearr[0] <= LINE_DETECTION_LIMIT && p->linearr[0] < p->linearr[1]){
	p->line_centers[0] = 0;	
	p->line_cnt++;
    }
    for(cnt = 1; cnt < 7; cnt++) {
      	if(p->linearr[cnt] <= LINE_DETECTION_LIMIT && p->linearr[cnt] < p->linearr[cnt-1] && p->linearr[cnt] < p->linearr[cnt+1]){
	    p->line_centers[p->line_cnt] = cnt;	
	    p->line_cnt++;
        }
    }
    if(p->linearr[7] <= LINE_DETECTION_LIMIT && p->linearr[7] < p->linearr[6]){
	p->line_centers[p->line_cnt] = 7;	
	p->line_cnt++;
    }
    if(p->line_cnt == 1)
    	p->line_flag = 1;
    else
    	p->line_flag = 2;
  }
}


//=====================================================================
// Odometry functions
//=====================================================================

void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_pos_old = p->left_pos_old = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->theta = 0;
  p->distance = 0;
  p->y = 0;
}

void update_odo(odotype *p)
{
  p->left_enc=lenc->data[0];
  p->right_enc=renc->data[0];
  
  int delta;
  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos_old = p->right_pos;
  p->right_pos += delta * p->cr;
  
  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos_old = p->left_pos;
  p->left_pos += delta * p->cl;
  
  p->theta += ((p->right_pos - p->right_pos_old) - (p->left_pos - p->left_pos_old)) / WHEEL_SEPARATION;
  p->distance = ((p->right_pos - p->right_pos_old) + (p->left_pos - p->left_pos_old)) / 2;
  p->y += p->distance * sin(p->theta);
}

void update_motcon(motiontype *p){ 
  p->left_pos=odo.left_pos;
  p->right_pos=odo.right_pos;
}  


//=====================================================================
// Movement functions
//=====================================================================

int fwd(double dist, double speed,int time,motiontype *p,sentype *s){    
    if (time==0){ 
        p->startl = p->left_pos;
        p->startr = p->right_pos;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
    }
    if(dist > 0 && ((ignoreobstacles == 0 && s->object_front_near == 0) || ignoreobstacles == 1)){  
      if (sqrt(2*MAX_ACCELERATION*(dist-(p->left_pos-p->startl))) > p->motorspeed_l){ 
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
    else if (dist <= 0){  
      if (sqrt(2*MAX_ACCELERATION*(fabs(dist)-(p->startl-p->left_pos))) > fabs(p->motorspeed_l)){ 
          if (((speed - MAX_ACCELERATION/100) < fabs(p->motorspeed_l)) && (fabs(p->motorspeed_l) < fabs(speed))) {
	      p->motorspeed_l = -speed;
	      p->motorspeed_r = p->motorspeed_l;
          }
          else if (fabs(p->motorspeed_l) < fabs(speed)) {
	      p->motorspeed_l -= MAX_ACCELERATION/100;
	      p->motorspeed_r = p->motorspeed_l;
          }
      }
      else{
          p->motorspeed_l = -sqrt(2*MAX_ACCELERATION*(fabs(dist)-(p->startl-p->left_pos)));
          p->motorspeed_r = p->motorspeed_l;
      }
      if (p->startl - p->left_pos  >= fabs(dist))
	return 1;
      else
        return 0;
    }
    else{
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
      return 0;
   }
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
	  else 
	    return 1;
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
	  else 
	    return 1;
	}
}


void followline(double speed, int operation,motiontype *m,sentype *p){
    p->numerator=0;
    p->denominator=0;
    
    if((ignoreobstacles == 0 && p->object_front_near == 0) || ignoreobstacles == 1){ 
      switch(operation){
      case br:
	if(p->line_flag == 2){
	    for(cnt = 0; cnt < 8; cnt++) {
	        if(!(cnt == p->line_centers[1]-1 || cnt == p->line_centers[1] || cnt == p->line_centers[1]+1)){
	    	    p->numerator += (1-p->linearr[cnt]) * p->dist[cnt];
      	            p->denominator += (1-p->linearr[cnt]);
                }
    	    }
        }
        else{
            for(cnt = 0; cnt < 8; cnt++) {
	        p->numerator += (1-p->linearr[cnt]) * p->dist[cnt];
      	        p->denominator += (1-p->linearr[cnt]);
    	    }
        }
	break;
      case bl:
      	if(p->line_flag == 2){
	    for(cnt = 0; cnt < 8; cnt++) {
	        if(!(cnt == p->line_centers[0]-1 || cnt == p->line_centers[0] || cnt == p->line_centers[0]+1)){
	    	    p->numerator += (1-p->linearr[cnt]) * p->dist[cnt];
      	            p->denominator += (1-p->linearr[cnt]);
                }
    	    }
        }
        else{
            for(cnt = 0; cnt < 8; cnt++) {
	        p->numerator += (1-p->linearr[cnt]) * p->dist[cnt];
      	        p->denominator += (1-p->linearr[cnt]);
    	    }
        }
	break;
      }
      p->CM = atan(p->numerator/p->denominator/21);
      m->motorspeed_l = speed-reg_k*p->CM;
      m->motorspeed_r = speed+reg_k*p->CM;
    }
    else{
      m->motorspeed_l = 0;
      m->motorspeed_r = 0;
    }
}

void drive(double speed,int time,motiontype *p, sentype *s){    
    if (time==0){ 
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
    }
    if (((speed - MAX_ACCELERATION/100) < p->motorspeed_l) && (p->motorspeed_l < speed && ((ignoreobstacles == 0 && s->object_front_near == 0) || ignoreobstacles == 1))) {
      p->motorspeed_l = speed;
      p->motorspeed_r = p->motorspeed_l;
    }
    else if (p->motorspeed_l < speed && (p->motorspeed_l < speed && ((ignoreobstacles == 0 && s->object_front_near == 0) || ignoreobstacles == 1))) {
      p->motorspeed_l += MAX_ACCELERATION/100;
      p->motorspeed_r = p->motorspeed_l;
    }
    else {
      p->motorspeed_l += 0;
      p->motorspeed_r = p->motorspeed_l;
    }
}

void stop(motiontype *p){
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
}

int char2int(char cmdbuffer[1000]){
    int len = strlen(cmdbuffer);
    int dec = 0;
    for(cnt = 0; cnt < len; cnt++){
	dec = dec * 10 + (cmdbuffer[cnt] - '0');
    }
    return dec;
}
