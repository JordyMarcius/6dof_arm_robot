/////////////////////////////////////////////////////////////
/* Template OpengGL sengaja dibuat untuk kuliah robotik 
*  di Departemen Teknik Elektro
*  Bagi yang ingin memodifikasi untuk keperluan yang lain,
*  dipersilahkan dengan menuliskan acknowledgement pada
*    Dr. Abdul Muis, MEng.
*    Autonomous Control Electronics (ACONICS) Research Group
*    http://www.ee.ui.ac.id/aconics
*////////////////////////////////////////////////////////////

#include <stdio.h> 
#include <stdlib.h> 
#include <GL/glut.h> 		// Header File For The GLUT Library
#include <GL/gl.h> 			// Header File For The OpenGL32 Library
#include <GL/glu.h> 		// Header File For The GLu32 Library
#include <unistd.h> 		// Header file for sleeping.
#include <math.h> 
#include <fcntl.h>			/* File control definitions */
#include <errno.h>			/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */ 
#include <sys/time.h>
#include "planar.c"

/* ascii code for the escape key */
#define ESCkey	27

/* The number/handle of our GLUT window */
int window, wcam, window1, window2; 

/* To draw a quadric model */
GLUquadricObj *obj;

// ROBOT MODEL PARAMATER
#define x_base1 0
#define y_base1 0
#define z_base1 0.2

#define x_base2 0
#define y_base2 0
#define z_base2 0.025

#define x_link1 0
#define y_link1 0
#define z_link1 0.3

#define x_link2 0
#define y_link2 0
#define z_link2 0.3

#define x_link2 0
#define y_link2 0
#define z_link3 0.1

#define d_0 0.1
#define a_1 0.02
#define a_2 0.13
#define a_3 0.02
#define d_4 0.15
#define a_5 0.0
#define d_6 0.125

float *tetha1=&q1;	
float *tetha2=&q2;  
float *tetha3=&q3;	
float *tetha4=&q4;	
float *tetha5=&q5;	
float *tetha6=&q6;	

//float *x=&objx;
//float *y=&objy;

char debug=0;

float x = 0;
float y = 0;
float z = 0;

float x_init = 0;
float y_init = 0;
float z_init = 0;

float x_cmd = 0;
float y_cmd = 0;
float z_cmd = 0;

float x_d = 0;
float y_d = 0;
float z_d = 0;

float ddx = 0;
float ddy = 0;
float ddz = 0;

float ex = 0;
float ey = 0;
float ez = 0;

float ex_old = 0;
float ey_old = 0;
float ez_old = 0;

float Kp = 40.0;
float Ki = 15.0;
float Kd = 25.0;

float der_ex = 0;
float der_ey = 0;
float der_ez = 0;

float int_ex = 0;
float int_ey = 0;
float int_ez = 0;

float ddq1_ref = 0;
float ddq2_ref = 0;
float ddq3_ref = 0;
float ddq4_ref = 0;
float ddq5_ref = 0;
float ddq6_ref = 0;

float v1 = 0;
float v2 = 0;
float v3 = 0;
float v4 = 0;
float v5 = 0;
float v6 = 0;

float dq1 = 0;
float dq2 = 0;
float dq3 = 0;
float dq4 = 0;
float dq5 = 0;
float dq6 = 0;

float t = 0;
float waktutraj = 4;
float k = 10;
int linemode = 0;

float L0 = d_0;
float L1 = a_1;
float L2 = a_2;
float L3 = a_3;
float L4 = d_4;
float L5 = a_5;
float L6 = d_6;
float L7 = 0;

void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal

/* define color */  
GLfloat green1[4] 	={0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]  	={0.1, 0.1, 1.0, 1.0};
GLfloat blue2[4]  	={0.2, 0.2, 1.0, 1.0};
GLfloat blue3[4]  	={0.3, 0.3, 1.0, 1.0};
GLfloat yellow1[4]	={0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]	={0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4]  	={0.8, 0.55, 0.6, 1.0};
GLfloat yellow5[4]	={0.8, 0.8, 0.0, 1.0};
GLfloat abu2[4]	  	={0.5, 0.5, 0.5, 1.0};
GLfloat gray1[4]  	={0.1, 0.1, 0.1, 1.0};
GLfloat gray2[4]  	={0.2, 0.2, 0.2, 1.0};
GLfloat gray3[4]  	={0.3, 0.3, 0.3, 1.0};
GLfloat gray4[4]  	={0.4, 0.4, 0.4, 1.0};
GLfloat gray5[4]  	={0.5, 0.5, 0.5, 1.0};
GLfloat gray6[4]  	={0.6, 0.6, 0.6, 1.0};
GLfloat gray7[4]  	={0.7, 0.7, 0.7, 1.0};
GLfloat gray8[4]  	={0.8, 0.8, 0.7, 1.0};
GLfloat gray9[4]  	={0.9, 0.9, 0.7, 1.0};
GLfloat merah1[4] 	={1.0, 0, 0, 1.0};
GLfloat orange1[4]	={1.0, 0.69, 0.28, 1.0};
GLfloat orange2[4]	={0.82, 0.35, 0.12, 1.0};
GLfloat orange3[4]	={0.82, 0.45, 0.25, 1.0};
GLfloat cyan1[4]  	={0.28, 1, 1, 1.0};
GLfloat gray10[4] 	={0.32, 0.39, 0.4, 1.0};
GLfloat gray11[4] 	={0.97, 0.97, 0.97, 1.0};
GLfloat hijau1[4] 	={0.5, 0.968, 0.243, 1.0};


void  drawOneLine(double x1, double y1, double x2, double y2) 
   {glBegin(GL_LINES); glVertex3f((x1),(y1),0.0); glVertex3f((x2),(y2),0.0); glEnd();}
   
void  model_cylinder(GLUquadricObj * object, GLdouble lowerRadius,
  GLdouble upperRadius, GLdouble length, GLint res, GLfloat *color1, GLfloat *color2)
{
  glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glTranslatef(0,0,-length/2);
	  gluCylinder(object, lowerRadius, upperRadius, length, 20, res);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    gluDisk(object, 0.01, lowerRadius, 20, res); 
    glTranslatef(0, 0, length);
    gluDisk(object, 0.01, upperRadius, 20, res); 
  glPopMatrix();
}

void  model_box(GLfloat width, GLfloat depth, GLfloat height, GLfloat *color1, GLfloat *color2, GLfloat *color3, int color)
{
   width=width/2.0;depth=depth/2.0;height=height/2.0;
   glBegin(GL_QUADS);
// top
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth, height);
    glVertex3f( width,-depth, height);
    glVertex3f( width, depth, height);
    glVertex3f(-width, depth, height);
   glEnd();
   glBegin(GL_QUADS);
// bottom
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth,-height);
    glVertex3f( width,-depth,-height);
    glVertex3f( width, depth,-height);
    glVertex3f(-width, depth,-height);
   glEnd();
   glBegin(GL_QUAD_STRIP);
// sides
    if (color==1) 
	    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    glVertex3f(-width,-depth,height);
    glVertex3f(-width,-depth,-height);
    glVertex3f(width,-depth,height);
    glVertex3f(width,-depth,-height);
    glVertex3f(width,depth,height);
    glVertex3f(width,depth,-height);
    glVertex3f(-width,depth,height);
    glVertex3f(-width,depth,-height);
    glVertex3f(-width,-depth,height);
   glEnd();
}



void disp_floor(void)
{
  int i,j,flagc=1;

  glPushMatrix();
  
  GLfloat dx=4.5,dy=4.5;
  GLint amount=15;
  GLfloat x_min=-dx/2.0, x_max=dx/2.0, x_sp=(GLfloat) dx/amount, y_min=-dy/2.0, y_max=dy/2.0, y_sp=(GLfloat) dy/amount;

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
  for(i = 0; i<=48; i++){
     drawOneLine(-2.4+0.1*i, -2.4,       -2.4+0.1*i,  2.4);
     drawOneLine(-2.4,       -2.4+0.1*i,  2.4,       -2.4+0.1*i);
  }

  glPopMatrix();
}

void  lighting(void)
{

	GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
	GLfloat light_diffuse[] =  {0.4, 0.4, 0.4, 1.0};
	GLfloat light_specular[] = {0.3, 0.3, 0.3, 1.0};
	GLfloat light_position[] = {2, 0.1, 7,1.0};
	GLfloat spot_direction[] = {0.0, -0.1, -1.0, 1.0};

	glClearColor(0.0, 0.0, 0.0, 0.0);     
  
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 40.0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 4);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

void disp_robot(void)
{
	glPushMatrix();
		glRotatef(90, 0, 0, 1);
		
		//menuju joint 1
		glRotatef(*tetha1*RTD, 0, 0, 1);
		glPushMatrix();
			//gambar base (d0)
			glTranslatef(0, 0, d_0/2);
			model_cylinder(obj, 0.1, 0.1, d_0, 2, gray9, gray9);
		glPopMatrix();
		
		glTranslatef(0, 0, d_0);
		glRotatef(90, 0, 1, 0);
		glPushMatrix();
			//gambar link 1 (a1)
			glTranslatef(0, 0, a_1/2);
			model_cylinder(obj, 0.02, 0.02, a_1, 2, pink6, pink6);
		glPopMatrix();
		
		//menuju joint 2
		glTranslatef(0, 0, a_1);
		glRotatef(-*tetha2*RTD, 0, 1, 0);  
		glPushMatrix();
			//gambar link 2 (a2)
			glTranslatef(0, 0, a_2/2);
			model_cylinder(obj, 0.02, 0.02, a_2, 2, orange1, orange1);
		glPopMatrix();
		
		//menuju joint 3
		glTranslatef(0, 0, a_2);
		glRotatef(-*tetha3*RTD, 0, 1, 0);  
		glPushMatrix();
			//gambar link 3 (a3)
			glTranslatef(0, 0, a_3/2);
			model_cylinder(obj, 0.02, 0.02, a_3, 2, pink6, pink6);
		glPopMatrix();
		
		//menuju joint 4
		glTranslatef(0, 0, a_3);
		glRotatef(90, 0, 1, 0); 
		glRotatef(*tetha4*RTD, 0, 0, 1);  
		glPushMatrix();  
			//gambar link 4 (d3)
			glTranslatef(0, 0, d_4/2);
			model_cylinder(obj, 0.02, 0.02, d_4, 2, yellow5, yellow5); 
		glPopMatrix();
		
		//menuju joint 5
		glTranslatef(0, 0, d_4); 
		glRotatef(90, 0, 1, 0); 
		glRotatef(-*tetha5*RTD, 0, 1, 0);
		glRotatef(-90, 0, 1, 0); 
		
		//menuju joint 6
		glRotatef(-*tetha6*RTD, 0, 0, 1);
		glPushMatrix();    
			//gambar link 6 (d6)  
			glTranslatef(0, 0, d_6/2); 
			model_cylinder(obj, 0.02, 0.02, d_6, 2, hijau1, hijau1); 
		glPopMatrix();
		  
	glPopMatrix(); 
}

// Draw Object
void display(void)
{
//   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   //glLoadIdentity();  // Reset View
   disp_floor();
   
   disp_robot();

   /* since window is double buffered, 
      Swap the front and back buffers (used in double buffering). */
   glutSwapBuffers() ; 
}

void display_window1(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, 2, 0.2, 8);
	gluLookAt(0.0, 0.0, 1.4,  -0.1, 0.0, 0.4,  0.0, 0.0, 1.0); 
	glMatrixMode(GL_MODELVIEW);
	lighting();
	
	disp_floor();
	disp_robot();
	
	glutSwapBuffers();
}

void display_window2(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, 2, 0.2, 8);
	gluLookAt(0.6, 0.6, 1.0,  -0.1, 0.0, 0.4,  0.0, 0.0, 1.0); 
	glMatrixMode(GL_MODELVIEW);
	lighting();
	
	disp_floor();
	disp_robot();
	
	glutSwapBuffers();
}

void keyboard(unsigned char key, int i, int j)
{
	 switch(key){
      case ESCkey: exit(1); break;
      case '1': *tetha1+=5*DTR; break;
      case '!': *tetha1-=5*DTR; break;
      
      case '2': *tetha2+=5*DTR; break;
      case '@': *tetha2-=5*DTR; break;
	  
	  case '3': *tetha3+=5*DTR; break;
      case '#': *tetha3-=5*DTR; break;
      
      case '4': *tetha4+=5*DTR; break;
      case '$': *tetha4-=5*DTR; break;
      
      case '5': *tetha5+=5*DTR; break;
      case '%': *tetha5-=5*DTR; break;
      
      case '6': *tetha6+=5*DTR; break;
      case '^': *tetha6-=5*DTR; break;
      
      case 'x': glTranslatef(0,0,0.05); break;
      case 'c': glTranslatef(0,0,-0.05); break;
      case 'u': glTranslatef(0.1,0,0); break; 
      case 'U': glTranslatef(-0.1,0,0); break;
      case 'i': glTranslatef(0,0.1,0); break;
	  case 'I': glTranslatef(0,-0.1,0); break;
	  case 'o': glTranslatef(0,0,0.1); break;
	  case 'O': glTranslatef(0,0,-0.1); break;
	  case 'j': glRotatef(10,1,0,0); break;
	  case 'J': glRotatef(-10,1,0,0); break; 
	  case 'k': glRotatef(10,0,1,0); break;
	  case 'K': glRotatef(-10,0,1,0); break;
	  case 'l': glRotatef(10,0,0,1); break;
	  case 'L': glRotatef(-10,0,0,1); break;
   }
}


void main_window(void) 
{ 
   glutInitWindowSize(650,400);	
   glutInitWindowPosition (40, 100);
   
   window = glutCreateWindow ("Main Window - Jordy Marcius");
   
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glEnable(GL_DEPTH_TEST); // Enables Depth Testing
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(60.0, 2, 0.2, 8);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   //gluLookAt(0.3, 0.0, 1.5,  -0.1, 0.0, 0.4,  0.0, 0.0, 1.0); 
   gluLookAt(0.8, 0.0, 0.4,  -0.1, 0.0, 0.4,  0.0, 0.0, 1.0); 
   lighting();
   glShadeModel(GL_SMOOTH) ; 

   glutDisplayFunc (&display) ;
   glutKeyboardFunc(&keyboard);
}


void window_1(void)
{
	glutInitWindowSize(500,300);
	glutInitWindowPosition(400,100);
	window1 = glutCreateWindow("Window 1 - Jordy Marcius");
	printf("window1 id : %d\n", window1);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glutDisplayFunc(&display_window1);
	glutKeyboardFunc(&keyboard);
}

void window_2(void)
{
	glutInitWindowSize(650,400);
	glutInitWindowPosition(400,400);
	window2 = glutCreateWindow("Window 2 - Jordy Marcius");
	printf("window2 id : %d\n", window2);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glutDisplayFunc(&display_window2);
	glutKeyboardFunc(&keyboard);
}

void forward_kinematic(float *x, float *y, float *z, float q1, float q2, float q3, float q4, float q5, float q6)
{
	*x = L1*cos(q1) + L5*(cos(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L6*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L7*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + L4*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + L2*cos(q1)*cos(q2);
	*y = L4*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L1*sin(q1) - L5*(cos(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L6*(sin(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L7*(sin(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + L2*cos(q2)*sin(q1);
	*z = L0 + L3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L4*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + L5*(sin(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + cos(q4)*cos(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - L6*(cos(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - cos(q4)*sin(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - L7*(cos(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - cos(q4)*sin(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + L2*sin(q2);
}

void hitung_PIDController(float *ddx, float *ddy, float *ddz, float ex, float ey, float ez)
{
	der_ex = (ex - ex_old)/dt;
	der_ey = (ey - ey_old)/dt;
	der_ez = (ez - ez_old)/dt;
	
	int_ex += ex*dt;
	int_ey += ey*dt;
	int_ez += ez*dt;
	
	*ddx = Kp*ex + Ki*int_ex + Kd*der_ex;
	*ddy = Kp*ey + Ki*int_ey + Kd*der_ey;
	*ddz = Kp*ez + Ki*int_ez + Kd*der_ez;
}

void inverse_jacobian(float *ddq1_ref, float *ddq2_ref, float *ddq3_ref, float *ddq4_ref, float *ddq5_ref, float *ddq6_ref, float ddx, float ddy, float ddz, float q1, float q2, float q3, float q4, float q5, float q6)
{
	// rumus dari pseudo inverse dirumuskan dengan persamaan di bawah ini
	// pseudo_inverse_jacobian = transpose(jacobian) * inverse(jacobian * transpose(jacobian))
	
	
	// menghitung komponen matriks jacobian
	double J11 = L3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - L4*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - L1*sin(q1) + L5*(cos(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + L6*(sin(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) + L7*(sin(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L2*cos(q2)*sin(q1);
	double J12 = L6*(cos(q5)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - cos(q4)*sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - L5*(sin(q5)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + cos(q4)*cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L7*(cos(q5)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - cos(q4)*sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - L3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + L4*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L2*cos(q1)*sin(q2);
	double J13 = L6*(cos(q5)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - cos(q4)*sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - L5*(sin(q5)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + cos(q4)*cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L7*(cos(q5)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - cos(q4)*sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - L3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + L4*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3));
	double J14 = L5*cos(q5)*(cos(q4)*sin(q1) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + L6*sin(q5)*(cos(q4)*sin(q1) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + L7*sin(q5)*(cos(q4)*sin(q1) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)));
	double J15 = L6*(cos(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - L5*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L7*(cos(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
	double J16 = 0;
	
	double J21 = L1*cos(q1) + L5*(cos(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L6*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L7*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + L3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + L4*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + L2*cos(q1)*cos(q2);
	double J22 = L5*(sin(q5)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L4*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - L3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - L6*(cos(q5)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L7*(cos(q5)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L2*sin(q1)*sin(q2);
	double J23 = L5*(sin(q5)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L4*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - L3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - L6*(cos(q5)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L7*(cos(q5)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)));
	double J24 = - L5*cos(q5)*(cos(q1)*cos(q4) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - L6*sin(q5)*(cos(q1)*cos(q4) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - L7*sin(q5)*(cos(q1)*cos(q4) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)));
	double J25 = L5*(sin(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L6*(cos(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - L7*(cos(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)));
	double J26 = 0;
	
	double J31 = 0;
	double J32 = L3*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + L4*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L5*(sin(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*cos(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + L6*(cos(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + cos(q4)*sin(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + L7*(cos(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + cos(q4)*sin(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + L2*cos(q2);
	double J33 = L3*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + L4*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L5*(sin(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*cos(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + L6*(cos(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + cos(q4)*sin(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + L7*(cos(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + cos(q4)*sin(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
	double J34 = - L6*sin(q4)*sin(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L7*sin(q4)*sin(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - L5*cos(q5)*sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2));
	double J35 = L5*(cos(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - cos(q4)*sin(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + L6*(sin(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + cos(q4)*cos(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + L7*(sin(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + cos(q4)*cos(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)));
	double J36 = 0;
	
	
	// menghitung komponen matriks transpose jacobian
	double trans_J11 = J11;
	double trans_J12 = J21;
	double trans_J13 = J31;
	
	double trans_J21 = J12;
	double trans_J22 = J22;
	double trans_J23 = J32;
	
	double trans_J31 = J13;
	double trans_J32 = J23;
	double trans_J33 = J33;
	
	double trans_J41 = J14;
	double trans_J42 = J24;
	double trans_J43 = J34;
	
	double trans_J51 = J15;
	double trans_J52 = J25;
	double trans_J53 = J35;
	
	double trans_J61 = J16;
	double trans_J62 = J26;
	double trans_J63 = J36;
	
	
	// menghitung komponen perkalian --> jacobian * transpose(jacobian)
	double comp_11 = J11*trans_J11 + J12*trans_J21 + J13*trans_J31 + J14*trans_J41 + J15*trans_J51 + J16*trans_J61;
	double comp_12 = J11*trans_J12 + J12*trans_J22 + J13*trans_J32 + J14*trans_J42 + J15*trans_J52 + J16*trans_J62;
	double comp_13 = J11*trans_J13 + J12*trans_J23 + J13*trans_J33 + J14*trans_J43 + J15*trans_J53 + J16*trans_J63;
	
	double comp_21 = J21*trans_J11 + J22*trans_J21 + J23*trans_J31 + J24*trans_J41 + J25*trans_J51 + J26*trans_J61;
	double comp_22 = J21*trans_J12 + J22*trans_J22 + J23*trans_J32 + J24*trans_J42 + J25*trans_J52 + J26*trans_J62;
	double comp_23 = J21*trans_J13 + J22*trans_J23 + J23*trans_J33 + J24*trans_J43 + J25*trans_J53 + J26*trans_J63;
	
	double comp_31 = J31*trans_J11 + J32*trans_J21 + J33*trans_J31 + J34*trans_J41 + J35*trans_J51 + J36*trans_J61;
	double comp_32 = J31*trans_J12 + J32*trans_J22 + J33*trans_J32 + J34*trans_J42 + J35*trans_J52 + J36*trans_J62;
	double comp_33 = J31*trans_J13 + J32*trans_J23 + J33*trans_J33 + J34*trans_J43 + J35*trans_J53 + J36*trans_J63;
	
	
	// menghitung determinan matriks comp
	double determinan = comp_11*comp_22*comp_33 - comp_11*comp_23*comp_32 - comp_12*comp_21*comp_33 + comp_12*comp_23*comp_31 + comp_13*comp_21*comp_32 - comp_13*comp_22*comp_31;
	
	
	// menghitung inverse dari matriks comp
	double inv_comp_11 =  (comp_22*comp_33 - comp_23*comp_32) / determinan;
	double inv_comp_12 = -(comp_12*comp_33 - comp_13*comp_32) / determinan;
	double inv_comp_13 =  (comp_12*comp_23 - comp_13*comp_22) / determinan;
	
	double inv_comp_21 = -(comp_21*comp_33 - comp_23*comp_31) / determinan;
	double inv_comp_22 =  (comp_11*comp_33 - comp_13*comp_31) / determinan;
	double inv_comp_23 = -(comp_11*comp_23 - comp_13*comp_21) / determinan;
	
	double inv_comp_31 =  (comp_21*comp_32 - comp_22*comp_31) / determinan;
	double inv_comp_32 = -(comp_11*comp_32 - comp_12*comp_31) / determinan;
	double inv_comp_33 =  (comp_11*comp_22 - comp_12*comp_21) / determinan;
	
	
	// menghitung inverse matriks jacobian
	double inv_J11 = trans_J11*inv_comp_11 + trans_J12*inv_comp_21 + trans_J13*inv_comp_31;
	double inv_J12 = trans_J11*inv_comp_12 + trans_J12*inv_comp_22 + trans_J13*inv_comp_32;
	double inv_J13 = trans_J11*inv_comp_13 + trans_J12*inv_comp_23 + trans_J13*inv_comp_33;
	
	double inv_J21 = trans_J21*inv_comp_11 + trans_J22*inv_comp_21 + trans_J23*inv_comp_31;
	double inv_J22 = trans_J21*inv_comp_12 + trans_J22*inv_comp_22 + trans_J23*inv_comp_32;
	double inv_J23 = trans_J21*inv_comp_13 + trans_J22*inv_comp_23 + trans_J23*inv_comp_33;
	
	double inv_J31 = trans_J31*inv_comp_11 + trans_J32*inv_comp_21 + trans_J33*inv_comp_31;
	double inv_J32 = trans_J31*inv_comp_12 + trans_J32*inv_comp_22 + trans_J33*inv_comp_32;
	double inv_J33 = trans_J31*inv_comp_13 + trans_J32*inv_comp_23 + trans_J33*inv_comp_33;
	
	double inv_J41 = trans_J41*inv_comp_11 + trans_J42*inv_comp_21 + trans_J43*inv_comp_31;
	double inv_J42 = trans_J41*inv_comp_12 + trans_J42*inv_comp_22 + trans_J43*inv_comp_32;
	double inv_J43 = trans_J41*inv_comp_13 + trans_J42*inv_comp_23 + trans_J43*inv_comp_33;
	
	double inv_J51 = trans_J51*inv_comp_11 + trans_J52*inv_comp_21 + trans_J53*inv_comp_31;
	double inv_J52 = trans_J51*inv_comp_12 + trans_J52*inv_comp_22 + trans_J53*inv_comp_32;
	double inv_J53 = trans_J51*inv_comp_13 + trans_J52*inv_comp_23 + trans_J53*inv_comp_33;
	
	double inv_J61 = trans_J61*inv_comp_11 + trans_J62*inv_comp_21 + trans_J63*inv_comp_31;
	double inv_J62 = trans_J61*inv_comp_12 + trans_J62*inv_comp_22 + trans_J63*inv_comp_32;
	double inv_J63 = trans_J61*inv_comp_13 + trans_J62*inv_comp_23 + trans_J63*inv_comp_33;
	
	
	// menghitung percepatan sudut referensi
	*ddq1_ref = inv_J11*ddx*dt + inv_J12*ddy*dt + inv_J13*ddz*dt;
	*ddq2_ref = inv_J21*ddx*dt + inv_J22*ddy*dt + inv_J23*ddz*dt;
	*ddq3_ref = inv_J31*ddx*dt + inv_J32*ddy*dt + inv_J33*ddz*dt;
	*ddq4_ref = inv_J41*ddx*dt + inv_J42*ddy*dt + inv_J43*ddz*dt;
	*ddq5_ref = inv_J51*ddx*dt + inv_J52*ddy*dt + inv_J53*ddz*dt;
	*ddq6_ref = inv_J61*ddx*dt + inv_J62*ddy*dt + inv_J63*ddz*dt;
}

void trajectory_line(float t)
{
	if(t < 0.02)
	{	
		x_init = x;
		y_init = y;
		z_init = z;
		
		if(linemode == 1)
		{
			linemode = 0;
		}
		else
		{
			linemode = 1;
		}
		 
		if(linemode==0)
		{
			x_cmd = x + 0.1;
			y_cmd = y + 0.1; 
			z_cmd = z + 0.1;
		}
		else
		{
			x_cmd = x - 0.1;
			y_cmd = y - 0.1;
			z_cmd = z - 0.1;
		}
	}
	
	x_d = (x_cmd-x_init)*t/waktutraj + x_init;
	y_d = (y_cmd-y_init)*t/waktutraj + y_init;
	z_d = (z_cmd-z_init)*t/waktutraj + z_init;
}

void control_robot()
{
	static int i = 0;
	
	forward_kinematic(&x, &y, &z, q1, q2, q3, q4, q5, q6);
	trajectory_line(waktutraj*(t/waktutraj - trunc(t/waktutraj)));
	
	ex = x_d - x;
	ey = y_d - y;
	ez = z_d - z;
	
	hitung_PIDController(&ddx, &ddy, &ddz, ex, ey, ez);
	
	ex_old = ex;
	ey_old = ey;
	ez_old = ez;
	
	inverse_jacobian(&ddq1_ref, &ddq2_ref, &ddq3_ref, &ddq4_ref, &ddq5_ref, &ddq6_ref, ddx, ddy, ddz, q1, q2, q3, q4, q5, q6);
	
	v1 = k*ddq1_ref;
	v2 = k*ddq2_ref;
	v3 = k*ddq3_ref;
	v4 = k*ddq4_ref;
	v5 = k*ddq5_ref;
	v6 = k*ddq6_ref;
	
	dq1 = dq1 + (2.083*v1 - 2.71*dq1)*dt; //parameter motor
	dq2 = dq2 + (2.083*v2 - 2.71*dq2)*dt;
	dq3 = dq3 + (2.083*v3 - 2.71*dq3)*dt;
	dq4 = dq4 + (2.083*v4 - 2.71*dq4)*dt; 
	dq5 = dq5 + (2.083*v5 - 2.71*dq5)*dt;
	dq6 = dq6 + (2.083*v6 - 2.71*dq6)*dt;

	q1 = q1 + dq1*dt; 
	q2 = q2 + dq2*dt;
	q3 = q3 + dq3*dt;
	q4 = q4 + dq4*dt; 
	q5 = q5 + dq5*dt;
	q6 = q6 + dq6*dt;
	
	if(i%10==0)
	{
		printf("t:%.2f, x_cmd:%.2f, y_cmd:%.2f, z_cmd:%.2f, x:%.2f, y:%.2f, z:%.2f, q1:%.2f, q2:%.2f, q3:%.2f, q4:%.2f, q5:%.2f, q6:%.2f, linemode:%d\n", t, x_cmd, y_cmd, z_cmd, x, y, z, q1, q2, q3, q4, q5, q6, linemode);
		//printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d\n", t, x_cmd, y_cmd, z_cmd, x, y, z, q1, q2, q3, q4, q5, q6, linemode);
	}
	
	t = t + dt;
	//i++;
}

void Sim_main(void)
{
	unsigned long Xr=0,Yr=0, Xg=0,Yg=0, Xb=0,Yb=0; // titik untuk menghitung sum
	int Nr=0, Ng=0, Nb=0;
	static unsigned int Rx,Ry, Gx,Gy, Bx,By; // untuk menyimpan hasil titik berat
	unsigned int i,j,k;
	
  	glutSetWindow(window);
  	display();
  	glutSetWindow(window1);
  	display_window1();
  	glutSetWindow(window2);
  	display_window2();
  	Retrieve_serial();
  	
  	//usleep(10000);
  	control_robot();
}

// Main Program
int main(int argc, char** argv)
{
 // Initialize GLUT
   /* Initialize GLUT state - glut will take any command line arguments 
      see summary on OpenGL Summary */  
   glutInit (&argc, argv);
   
   // Berikut jika ingin menggunakan serial port
   //fd = open_port();
   //init_port(fd);

   /* Select type of Display mode:   
      Double buffer 
      RGBA color
      Alpha components supported 
      Depth buffer */  
   //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
   
   obj = gluNewQuadric();  

   /* Initialize our window. */
   init_robot();
   main_window();
   window_1();
   window_2();

   /* Register the function to do all our OpenGL drawing. */
   glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

   /* Start Event Processing Engine */ 
   glutMainLoop () ;
   return 0 ;
}           
