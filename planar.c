#include <math.h>
#include "serial.h"
#define PI		3.14159265358
#define DTR 	PI/180.0				   // Conversi degree to radian
#define RTD 	180.0/PI				   // Conversi degree to radian

float q1;
float q2;
float q3;
float q4;
float q5;
float q6;

float objx=0.3;
float objy=0.5;
float dt = 0.02;

void init_robot()
{
	q1 = 20.0 * DTR;
	q2 = 100.0 * DTR;
	q3 = 0.0 * DTR;
	q4 = 45.0 * DTR;
	q5 = -90.0 * DTR;
	q6 = 0.0 * DTR;
}

void Retrieve_serial(void) {
  int retval=1, i,j,k,l;

  unsigned char sdata[3]; 
  unsigned char baca;
  
  
	i=1;

  while (i>0) {
    fcntl(fd, F_SETFL, FNDELAY); 
    i=read(fd, &baca, 1);
    if ((i==1) && (baca == 0xF5)) {
    	printf("masuk\n");
    	sdata[0]=baca;
    	while (i<3) {
    		  if (read(fd, &baca, 1)>0) {sdata[i]=baca; i++;}
    	}
   	  printf("terbaca %x  %x  %x \n",sdata[0],sdata[1],sdata[2]);
   	  q1=(sdata[1])*180.0/255.0*DTR;
   	  q2=(sdata[2])*180.0/255.0*DTR;
    }
  } 

}
