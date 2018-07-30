#include <stdio.h>
#include <stdlib.h> // For exit() function
#include <string.h>
#define M_PI 3.1415
int char2int(char cmdbuffer[1000]);
double dist;
int main()
{
    char cmdbuffer[1000];
    int dec = 0, i, j, len, cmd =0 ;
    double angel = 0;
    
    
    if ((fptr = fopen("program.txt", "r")) == NULL)
    {
        printf("Error! opening file");
        // Program exits if file pointer returns NULL.
        exit(1);         
    }
    while((fscanf(fptr,"%s", cmdbuffer)) != EOF) // Read one word
    //while((fgets(c, sizeof c, fptr)) != NULL) // Read one line
    {
      
      if (strcmp("followline",cmdbuffer)==0)
      {
	cmd = 1;
	printf("cmd %d\n",cmd);
      }
      
      if (strcmp("turn",cmdbuffer)==0)
      {
	cmd = 2;
	printf("cmd %d\n",cmd);
	fscanf(fptr,"%s", cmdbuffer);
	dec = char2int(cmdbuffer);
         if(dec > 0)
         angel = dec*M_PI/180; 
	 else{
	  dec = dec+120;
	  angel = dec*M_PI/180;
	  
	   }
	  dec=0;
       }
      if (strcmp("fwd",cmdbuffer)==0)
      {
	cmd = 3;
	printf("cmd %d\n",cmd);
	fscanf(fptr,"%s", cmdbuffer);
	dec = char2int(cmdbuffer);
         if (dec > 80 && dec < 90 ){
	   dist =1+(dec-80)*1.0/10;

	   dec = 0;
           }
         else if(dec > 180 && dec < 190 ){
	   dist =2+(dec-180)*1.0/10; 
	   dec = 0;
	    }
	else if(dec > 280 && dec < 290 ){
	   dist =3+(dec-280)*1.0/10;
	   dec = 0;
	    }
         else if(dec > 380 && dec < 390 ){
	   dist =4+(dec-380)*1.0/10;
	   dec = 0;
	    }
	else if(dec > 480 && dec < 490 ){
	   dist =5+(dec-480)*1.0/10;
	   dec = 0;
	}
	else if(dec > -20 && dec < -10 ){
	   dist =(dec+20)*1.0/10;
	   dec = 0;
	}
	else if(dec > -320 && dec < -310 ){
	   dist =-(dec+320)*1.0/10;
	   dec = 0;
	}
	else if(dec > -2920 && dec < -2910 ){
	   dist =-(1+(dec+2920)*1.0/10);
	   dec = 0;
	}
	
        else if(dec > -2820 && dec < -2810 ){
	   dist =-(2+(dec+2820)*1.0/10);
	   dec = 0;
	}
	else if(dec > -2720 && dec < -2710 ){
	   dist =-(3+(dec+2820)*1.0/10);
	   dec = 0;
	}
	else if(dec > -2620 && dec < -2610 ){
	   dist =-(4+(dec+2620)*1.0/10);
	   dec = 0;
	}
	
	printf("%f\n",dist); 
	dec = 0; 
	
      }
      if (strcmp("ignoreobstacles",cmdbuffer)==0)
      {
	cmd = 4;
	printf("cmd %d\n",cmd);
      }
      if (strcmp("stop",cmdbuffer)==0)
      {
	cmd = 5;
	printf("cmd %d\n",cmd);
      }
      if (strcmp(":($crossingblackline)",cmdbuffer)==0)
      {
	cmd = 6;
	printf("cmd %d\n",cmd);
      }
      dec = char2int(cmdbuffer);
      if(dec==-8354){ //br
	 cmd = 7;
	 printf("cmd %d\n",cmd);
      }
      if(dec==-6304 ){ //wm
	 cmd = 8;
	 printf("cmd %d\n",cmd);
      }
       //printf("%d \n", dec);
	dec = 0;
	
    }
    fclose(fptr);
    
    return 0;
}

int char2int(char cmdbuffer[1000]){
  int len = strlen(cmdbuffer);
  int dec = 0,i=0;
  for(i=0; i<len; i++){
     dec = dec * 10 + ( cmdbuffer[i] - '0' );
    }
    return dec;
  
}

