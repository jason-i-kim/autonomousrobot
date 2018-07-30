/*
 * An example SMR program.
 *
 */
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

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];
double linearr[10];
float arrposl[5000];
float arrposr[5000];
float arrx[5000];
float arry[5000];
float arrtheta[5000];
int count;
float disp[5000];
FILE *f;
int toggle;
float ref_angle;
float reg_k;
float angle_cal;
int q;

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv,camsrv;

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
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.2662	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902
#define NUMBER_OF_TURNS	4
#define MAX_SPEED .6
#define BLACK_CALIB 0
#define WHITE_CALIB 255


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

void reset_odo(odotype *p);
void update_odo(odotype *p);


/********************************************
* Motion control
*/

typedef struct{//input
                int cmd;
		int curcmd;
		double speedcmd;
		double dist;
		double angle;
		double left_pos,right_pos;
		// parameters
		double w;
		//output
		double motorspeed_l,motorspeed_r; 
		int finished;
		// internal variables
		double startpos;
		double startr;
		double startl;
	       }motiontype;
	       
enum {mot_stop=1,mot_move,mot_turn};

void update_motcon(motiontype *p);	       


int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
void line_transform();
float lowest_line();



typedef struct{
                int state,oldstate;
		int time;
	       }smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init,ms_fwd,ms_turn,ms_end};

int main()
{
  int running,n=0,arg,time=0;
  double dist=0,angle=0;
  count = 1;
  arrposl[0] = 0;
  arrposr[0] = 0;
  arrtheta[0] = 0;
  arrx[0] = 0;
  arry[0] = 0;
  disp[0] = 0;
  reg_k = 0.35;

  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);
           
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);
     // **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}   
 
   
   
   
// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
     len=sprintf(buf,"scanpush cmd='zoneobst'\n");
     send(lmssrv.sockfd,buf,len,0);
   }

}   
   
 
  /* Read sensors and zero our position.
   */
  rhdSync();
  
  odo.w=WHEEL_SEPARATION;
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  mot.w=odo.w;
running=1; 
mission.state=ms_init;
mission.oldstate=-1;
while (running){ 
   if (lmssrv.config && lmssrv.status && lmssrv.connected){
           while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
      }
      
      if (camsrv.config && camsrv.status && camsrv.connected){
          while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }
       

  rhdSync();
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  update_odo(&odo);
  
/****************************************
/ mission statemachine   
*/
   sm_update(&mission);
   switch (mission.state) {
     case ms_init:
       n=NUMBER_OF_TURNS; dist=2;angle=90.0/180*M_PI;
       mission.state= ms_fwd;      
       toggle = 1;
     break;
  
     case ms_fwd:
       if (toggle) {
	 mot.startr = mot.right_pos;
	 mot.startl = mot.left_pos;
	 toggle = 0;
       }
       if (fwd(dist,MAX_SPEED,mission.time)) {
	 toggle = 1;
	 ref_angle = angle * (NUMBER_OF_TURNS-n+1);
	 mission.state=ms_end;
      }
     break;
  
     case ms_turn:
       if (toggle) {
	 mot.startr = mot.right_pos;
	 mot.startl = mot.left_pos;
	 toggle = 0;
       }
       if (turn(angle,MAX_SPEED,mission.time)){
	 toggle = 1;
         n=n-1;
	 if (n==0) 
	   mission.state=ms_end;
	 else 
	   mission.state=ms_fwd;
	}
     break;    
     
     case ms_end:
       mot.cmd=mot_stop;
       running=0;

     break;
   }  
/*  end of mission  */
 
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  if (time  % 100 ==0)
    //    printf(" laser %f \n",laserpar[3]);
  time++;
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;
    
}/* end of main control loop */
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  f = fopen("output.dat", "w");
  fprintf(f, "time, spdl, spdr, x, y, theta\n");
  fprintf(f, "%d %d %d %d %d %d\n", 0, 0, 0, 0, 0, 0);
  for(count = 1; count < 5000; count++) {
    fprintf(f, "%d %f %f %f %f %f\n", count, ((arrposl[count] - arrposl[count - 1]) / .01), ((arrposr[count] - arrposr[count - 1]) / .01), arrx[count], arry[count], arrtheta[count]);
  }
  fclose(f);
  exit(0);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


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
  
  if (count < 5000) {
    arrposl[count] = p->left_pos;
    arrposr[count] = p->right_pos;
    arrtheta[count] = arrtheta[count - 1] + ((p->right_pos - arrposr[count - 1]) - (p->left_pos - arrposl[count - 1])) / WHEEL_SEPARATION;
    disp[count] = ((p->right_pos - arrposr[count - 1]) + (p->left_pos - arrposl[count - 1])) / 2;
    arrx[count] = arrx[count - 1] + disp[count] * cos(arrtheta[count]);
    arry[count] = arry[count - 1] + disp[count] * sin(arrtheta[count]);
    count++;
  }
  
}


void update_motcon(motiontype *p){ 

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
       p->motorspeed_l =MAX_SPEED-reg_k*lowest_line();
       p->motorspeed_r =MAX_SPEED+reg_k*lowest_line();
       if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist){
          p->finished=1;
	  p->motorspeed_l=0;
          p->motorspeed_r=0;
       }
     break;
     
     case mot_turn:
       //printf("ref_angle %f\n", ref_angle);
       angle_cal = (p->right_pos-p->left_pos)/p->w;
       p->motorspeed_l =-reg_k*(ref_angle-angle_cal);
       p->motorspeed_r =reg_k*(ref_angle-angle_cal);
        //printf("motorspeed_l %f, motorspeed_r %f and angle %f\n",p->motorspeed_l,p->motorspeed_r,angle_cal);
       if (angle_cal >= ref_angle*0.982){
	 p->motorspeed_r = 0;
	 p->motorspeed_l = 0;
         p->finished=1;
       }
       
   }
}   


int fwd(double dist, double speed,int time){    
   if (time==0){ 
     mot.cmd=mot_move;
     mot.speedcmd=speed;
     mot.dist=dist;
     return 0;
   }
   else
     return mot.finished;
}

int turn(double angle, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_turn;     
     mot.speedcmd=speed;
     mot.angle=angle;
     return 0;
   }
   else
     return mot.finished;
}


void sm_update(smtype *p){
  if (p->state!=p->oldstate){
       p->time=0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
   }
}

void line_transform() {
  for(q = 0; q < 8; q++) {
    linearr[q] = (linesensor->data[q]-BLACK_CALIB)*1.0/(WHITE_CALIB-BLACK_CALIB);
  }
}

float lowest_line() {
  
  float dist[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
  float numerator=0;
  float denominator=0;
  
  line_transform();
  
  for(q = 0; q < 8; q++) {
    numerator += (1-linearr[q]) * dist[q];
    denominator += (1-linearr[q]);
  }
  
  return atan(numerator/denominator/21);  
}


