#include "project.h"
int main()
{
//=====================================================================
// Initialize variables
//=====================================================================
  
  int running = 1, time = 0, arg,dec=0;
  double dist=0, angle=0;
  char cmdbuffer[1000];
  MAX_SPEED = 1;
  ignoreobstacles = 0;
  state = state_read;

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
  
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);  
  reset_sen(&sen);
  
//=====================================================================
// Opening mission file
//=====================================================================
  FILE *fptr;
 
  //Error reading file
  if ((fptr = fopen("program.txt", "r")) == NULL)
    {
        printf("Error! opening file");
        // Program exits if file pointer returns NULL.
        exit(1);         
    }
  
  
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
  update_odo(&odo);
  update_sen(&sen);
  update_motcon(&mot);
  
  
//=====================================================================
// Program reading function
//=====================================================================
  switch(state){
    //Read command
    case state_read:
      //Reset
      dec = 0;
      time = 0;
      
      //Read mission
      if((fscanf(fptr,"%s", cmdbuffer)) != EOF){
	//Mission decoding
	
	// ===== Ignore obstacless ===== 
	if (strcmp("ignoreobstacles",cmdbuffer)==0)
	{
	  ignoreobstacles = 1;
	}
	  
	// ===== Followline ===== 
	if (strcmp("followline",cmdbuffer)==0)
	{
	  state = state_execute;
	  if(fscanf(fptr,"%s", cmdbuffer) == 0)
          {
           printf("Error! reading command");
           exit(1);         
          }
	  dec = char2int(cmdbuffer);
	  if(dec==-8354){ //br
	    if(fscanf(fptr,"%s", cmdbuffer) == 0)
	    {
	      printf("Error! reading command");
	      exit(1);         
	    }
	    
	    // ===== Check speed change =====
	    if (strcmp("@v0.1",cmdbuffer)==0 || strcmp("@v0.2",cmdbuffer)==0 || strcmp("@v0.3",cmdbuffer)==0 || strcmp("@v0.4",cmdbuffer)==0 || strcmp("@v0.5",cmdbuffer)==0){
	      dec = char2int(cmdbuffer)-229980;
	      MAX_SPEED = (dec)*1.0/10;
	      if(fscanf(fptr,"%s", cmdbuffer) == 0)
	      {
		printf("Error! reading command");
		exit(1);         
	      }
	    }
	    
	    // ===== Normal function decoding =====
	    if (strcmp(":($irdistfrontmiddle",cmdbuffer)==0){
	      cmd = cmd_br_irfront;
	      if(fscanf(fptr,"%s", cmdbuffer) == 0)
	      {
		printf("Error! reading command");
		exit(1);         
	      } // always <
	      if(fscanf(fptr,"%s", cmdbuffer) == 0)
	      {
		printf("Error! reading command");
		exit(1);         
	      } // distance value
	      dec = char2int(cmdbuffer);
	      dist = (dec+20)*100.0/10;        
	    }
	   else if(strcmp(":($irdistleft",cmdbuffer)==0){
	      cmd = cmd_br_irleft;
	      if(fscanf(fptr,"%s", cmdbuffer) == 0)
	      {
		printf("Error! reading command");
		exit(1);         
	      } // always <
	      if(fscanf(fptr,"%s", cmdbuffer) == 0)
	      {
		printf("Error! reading command");
		exit(1);         
	      } // distance value
	      dec = char2int(cmdbuffer);
	      dist = (dec+20)*100.0/10; 
	   }
	   else 
	     cmd = cmd_br_cross;
          }
          else //wm
	    cmd = cmd_wm;  
	}
      
	// ===== Turn ===== 
	if (strcmp("turn",cmdbuffer)==0)
	{
	  cmd = cmd_turn;
	  printf("cmd %d\n",cmd);
	  if(fscanf(fptr,"%s", cmdbuffer) == 0)
          {
           printf("Error! reading command");
           exit(1);         
          }
	  dec = char2int(cmdbuffer);
	  if(dec > 0){
	    angle = dec*M_PI/180;
	    printf("%d ",dec);}
	  else{
	    if(dec > -1000)
	      dec = -300-dec;
	    else
	      dec = -3000-dec;
	    printf("%d ",dec);
	    angle = dec*M_PI/180;
	   }
	   printf("%f\n",angle);
	  state = state_execute;
	}
	
	// ===== Forward ===== 
	if (strcmp("fwd",cmdbuffer)==0)
	{
	  cmd = cmd_fwd;
	  printf("cmd %d\n",cmd);
	  if(fscanf(fptr,"%s", cmdbuffer) == 0)
          {
           printf("Error! reading command");
           exit(1);         
          }
	  dec = char2int(cmdbuffer);
	  if (dec > 80 && dec < 90 ) // 1.x
	    dist = 1+(dec-80)*1.0/10;
	  else if(dec > 180 && dec < 190 ) //2.x
	    dist = 2+(dec-180)*1.0/10; 
	  else if(dec > 280 && dec < 290 ) //3.x
	    dist = 3+(dec-280)*1.0/10;
	  else if(dec > 380 && dec < 390 ) //4.x
	    dist = 4+(dec-380)*1.0/10;
	  else if(dec > 480 && dec < 490 ) //5.x
	    dist = 5+(dec-480)*1.0/10;
	  else if(dec > -20 && dec < -10 ) //0.x
	    dist = (dec+20)*1.0/10; 
	  else if(dec > -320 && dec < -310 )
	    dist = -(dec+320)*1.0/10;
	  else if(dec > -2920 && dec < -2910 )
	    dist = -(1+(dec+2920)*1.0/10);
	  else if(dec > -2820 && dec < -2810 )
	    dist = -(2+(dec+2820)*1.0/10);
	  else if(dec > -2720 && dec < -2710 )
	    dist = -(3+(dec+2820)*1.0/10);
	  else if(dec > -2620 && dec < -2610 )
	    dist = -(4+(dec+2620)*1.0/10);
	  else if(dec > 0 && dec < 10 )
	    dist = dec*1.0;
	  else if(dec > -30 && dec < -20 )
	    dist = -(dec+30)*1.0;
	  else 
	    dist = 0.0;
	  state = state_execute;
	  }
	  
	// ===== Drive ===== 
	if (strcmp("drive",cmdbuffer) == 0)
	{
	  printf("cmd %d\n",cmd);
	  if(fscanf(fptr,"%s", cmdbuffer) == 0)
          {
           printf("Error! reading command");
           exit(1);         
          }
          
          // ===== Check speed change =====
	    if (strcmp("@v0.1",cmdbuffer)==0 || strcmp("@v0.2",cmdbuffer)==0 || strcmp("@v0.3",cmdbuffer)==0 || strcmp("@v0.4",cmdbuffer)==0 || strcmp("@v0.5",cmdbuffer)==0){
	      dec = char2int(cmdbuffer)-229980;
	      MAX_SPEED = (dec)*1.0/10;
	      if(fscanf(fptr,"%s", cmdbuffer) == 0)
	      {
		printf("Error! reading command");
		exit(1);         
	      }
	    }
	  
	  // ===== Normal function decoding =====
          if (strcmp(":($irdistleft",cmdbuffer)==0){
	    if(fscanf(fptr,"%s", cmdbuffer) == 0)
            {
              printf("Error! reading command");
              exit(1);         
            }
            if(fscanf(fptr,"%s", cmdbuffer) == 0)
            {
              printf("Error! reading command");
              exit(1);         
            }
            dec = char2int(cmdbuffer); 
	    dist = (dec+20)*100.0/10;
	    cmd = cmd_drive_ir;
	  }
	  else 
	    cmd = cmd_drive_cross;
	  state = state_execute;
	  }
	  
	// ===== Eval ===== 
	if (strcmp("eval",cmdbuffer) == 0)
	{ 
	  cmd = cmd_eval;
	  state = state_execute;
	}
      } 
	else
	  running = 0;//End of file = end of mission
	break;
	
    //Execute command
    case state_execute:
	switch(cmd){
	  case cmd_fwd:
	    if(fwd(dist,MAX_SPEED,time,&mot,&sen) == 1){
	      ignoreobstacles = 0;
	      state = state_read;
	    }
	    break;
	    
	  case cmd_turn:
	    if(turn(angle,MAX_SPEED,time,&mot) == 1){
	      ignoreobstacles = 0;
	      state = state_read;
	    }
	  break;
	  
	  case cmd_br_irfront:
	    if(sen.irarr[2] < dist){
	      stop(&mot);
	      ignoreobstacles = 0;
	      state = state_read;
	    }
	    else
	      followline(MAX_SPEED,br,&mot,&sen);
	    break;
	    
	  case cmd_br_irleft:
	    if(sen.irarr[0] < dist){
	      stop(&mot);
	      ignoreobstacles = 0;
	      state = state_read;
	    }
	    else
	      followline(MAX_SPEED,br,&mot,&sen);
	    break;
	    
	  case cmd_br_cross:
	    if(sen.line_flag == 3){
	      stop(&mot);
	      ignoreobstacles = 0;
	      state = state_read;
	    }
	    else
	      followline(MAX_SPEED,br,&mot,&sen);
	    break;
	    
	  case cmd_wm:
	    
	    state = state_read;
	    break;
	    
	  case cmd_drive_ir:
	    if(sen.irarr[0] > dist){
	      stop(&mot);
	      ignoreobstacles = 0;
	      state = state_read;
	    }
	    else
	      drive(MAX_SPEED,time,&mot,&sen);
	    break;
	    
	  case cmd_drive_cross:
	    if(sen.line_flag == 3){
	      stop(&mot);
	      ignoreobstacles = 0;
	      state = state_read;
	    }
	    else
	      drive(MAX_SPEED,time,&mot,&sen);
	    break;
	    
	  case cmd_eval:
	    printf("Distance to box %f\n",-odo.y+0.22+sen.irarr[2]/100-0.08);
	    ignoreobstacles = 0;
	    state = state_read;
	    break;
	}
	if (time == 0)
	  time++;
	break;
  }


//=====================================================================
// Update values
//=====================================================================

  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  
/* stop if keyboard is activated*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;
    
}
//=====================================================================
// End of program
//=====================================================================
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  fclose(fptr);
  exit(0);
}
