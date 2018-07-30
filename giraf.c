#include "giraf.h"

int main()
{
//=====================================================================
// Initialize variables
//=====================================================================
  
  int running,n=0,arg,time=0;
  double dist=0,angle=0;
  count = 1;
  reg_k = 0.35;
  

//=====================================================================
// Establish connection to robot sensors and actuators.
//=====================================================================
  
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

      
//=====================================================================
// Camera server code initialization
//=====================================================================

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


//=====================================================================
// LMS server code initialization
//=====================================================================

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
  running=1; 
  mission.state=ms_init;
  mission.oldstate=-1;
  
  reset_sen(&sen);
  
  
//=====================================================================
// Main program loop
//=====================================================================

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
  update_sen(&sen);
  
//=====================================================================
// Mission reading function !!!!!!MAKE!!!!!
//=====================================================================
  if(turn(-M_PI,0.5,time,&mot) == 1)
    running = 0;

//printf("%f %f %f %f %f %d %d %d\n",sen.irarr[0],sen.irarr[1],sen.irarr[2],sen.irarr[3],sen.irarr[4]);
 
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  //update_motcon(&mot);
  
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  if (time  % 100 ==0)
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
  exit(0);
}