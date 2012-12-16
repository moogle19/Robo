// g++ -o rr rr.c -lGL -lGLU -lglut

#include <stdlib.h>     // General Header Files
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <GL/gl.h>      // Header File For The OpenGL32 Library
#include <GL/glu.h>     // Header File For The GLu32 Library
#include <GL/glut.h>    // HEader File For The glu toolkit
#include <GL/glx.h>     // Header File For The glx libraries.

#include <fstream>
#include <iostream>
#include "mini.h"

#include "ANN/ANN.h"

using std::cout;
using std::cerr;
using std::endl;
using std::ofstream;
using std::ios;
using std::ifstream;

#include <vector>
using std::vector;

#define EPSILON_FINISH 0.01
#define EPSILON_KDTREE 1.0
#define REDUCTION_FACTOR 10

const int RGB           = 3;                   // PPM files has 3 bytes of color
// levels per image pixel
const int RGBA          = 4;                   // The GL Buffer, however, has 4 bytes

GLfloat  cangle      = 75.0;                 // current camera opening angle
GLfloat  rotX = 0.0, rotY = -10.0, rotZ = 0.0; // current rotation of the scene
GLfloat  X    = -60;
GLfloat  Y    = -50.0,    Z = -250.0; // current scene translation

GLenum   polygonMode = GL_LINE;              // current ploygon mode

int window;

int frameNr;
char ppmfname[255];

vector<double*> pointlist;
vector<double*> pointlist2;

ANNkd_tree* tree;
bool doICP = false;

double lastError = HUGE_VAL;
int iterations = 0;

/****************************************
 * get euclidean distance
 ****************************************/
double peuklidD(point p1, point p2){
	return p1[0]*p2[0]+p1[1]*p2[1]+p1[2]*p2[2];
}

/****************************************
 * Get the corresponding 
 ****************************************/
vector<PtPair> getCorresponding(vector<double*> points2, vector<double*> points1, ANNkd_tree* tree){
	vector<PtPair> result;
	result.clear();
		
	for(int i = 0; i < points2.size(); i+=REDUCTION_FACTOR){
		int index = -1;
		double dist; 
		tree->annkSearch(points2[i], 1, &index, &dist, EPSILON_KDTREE);
		/*double minDist = HUGE_VAL;
		int minIndex = -1;
		for(int j = 0; j < points1.size(); j+=1){
			if(peuklidD(points1[j], points2[i]) < minDist){
				minIndex = j;
				minDist = peuklidD(points1[j], points2[i]);
			}
		}*/
		if(true || iterations < 20 || dist < 20*20){
			PtPair p = {points1[index][0],points1[index][1],points1[index][2],points2[i][0],points2[i][1],points2[i][2],
			   	 1};
			result.push_back(p);
		}
	}
	//printf("Correspondence done! \n");
	return result;
}


/*
   +++++++++-------------++++++++++++
   glDumpWindowPPM-function Purpose:
   writes the framebuffer content to a ppm file
   +++++++++-------------++++++++++++
   */
void glDumpWindowPPM(const char *filename)
{
	int win_height, win_width;
	int i,j,k,l;                  /* Counter variables */
	GLubyte *buffer;              /* The GL Frame Buffer */
	unsigned char *ibuffer;       /* The PPM Output Buffer */
	ofstream fp;                  /* The PPM File */

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	win_height = viewport[3];
	win_width  = viewport[2];

	/* Allocate memory for the the frame buffer and output buffer */
	buffer = new GLubyte[win_width * win_height * RGBA];
	ibuffer = new unsigned char[win_width * win_height * RGB];

	/* Read window contents from GL frame buffer with glReadPixels */
	glFinish();
	glReadPixels(0, 0, win_width, win_height,
			GL_RGBA, GL_UNSIGNED_BYTE, buffer);

	/* Open the output file */
	fp.open(filename, ios::out);

	/* Write a proper P6 PPM header */
	fp << "P6" << endl
		<< "# CREATOR: 3D_Viewer by Andreas Nuechter, FhG-AiS" << endl
		<< win_width  << " " << win_height << " " << UCHAR_MAX << endl;

	/* Loop through the frame buffer data, writing to the PPM file.  Be careful
	   to account for the frame buffer having 4 bytes per pixel while the
	   output file has 3 bytes per pixel */
	l = 0;
	for (i = 0; i < win_height; i++) {     /* For each row */
		for (j = 0; j < win_width; j++) {    /* For each column */
			for (k = 0; k < RGB; k++) {        /* For each RGB component */
				ibuffer[l++] = (unsigned char)
					*(buffer + (RGBA*((win_height-1-i)*win_width+j)+k));
			}                                  /* end RGB */
		}                                    /* end column */
	}

	/* Write output buffer to the file */
	fp.write((const char*)ibuffer, sizeof(unsigned char) * (RGB * win_width * win_height));
	fp.close();
	fp.clear();
	delete [] buffer;
	delete [] ibuffer;
}

/*********************
 * Apply matrix on pointlist
 *********************/
void applyMatrix(double* matrix, vector<double*> pointlist){
	for(int i = 0; i < pointlist.size(); i++){
		double point[3];
		point[0] = matrix[0]*pointlist[i][0] + matrix[4]*pointlist[i][1] + matrix[8]*pointlist[i][2];
		point[1] = matrix[1]*pointlist[i][0] + matrix[5]*pointlist[i][1] + matrix[9]*pointlist[i][2];
		point[2] = matrix[2]*pointlist[i][0] + matrix[6]*pointlist[i][1] + matrix[10]*pointlist[i][2];
		pointlist[i][0] = point[0]+matrix[12];
		pointlist[i][1] = point[1]+matrix[13];
		pointlist[i][2] = point[2]+matrix[14];
	}
}

static void init(int width, int height)
{
	/* select clearing color 	*/
	glClearColor (1.0, 1.0, 1.0, 1.0);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	/* angle, aspect, near clip, far clip: */
	gluPerspective(cangle, 1.0, 1.0, 32000.0);

	/* now use modelview-matrix as current matrix: */
	glMatrixMode(GL_MODELVIEW);
}

// *************************
// * Do a ICP iteration
// *************************
void doICPIteration(ANNkd_tree* tree){
	double alignfx[16];
	vector<PtPair> cor = getCorresponding(pointlist2, pointlist, tree);
	double error = Point_Point_Align(cor, alignfx);
	cor.clear();
	iterations++;
	//printf("result for itertation nr %ld:\n %lf, %lf, %lf, %lf \n %lf, %lf, %lf, %lf \n %lf, %lf, %lf, %lf \n  %lf, %lf, %lf, %lf \n",iterations, alignfx[0], alignfx[1], alignfx[2], alignfx[3], alignfx[4], alignfx[5], alignfx[6], alignfx[7], alignfx[8], alignfx[9], alignfx[10], alignfx[11], alignfx[12], alignfx[13], alignfx[14], alignfx[15]);

	applyMatrix(alignfx, pointlist2);
	if(fabs(error-lastError) < EPSILON_FINISH){
		doICP = false;
		printf("finished afer %d iterations!!!\n", iterations);
	}
	lastError = error;
}

static void display()
{
	// init modelview matrix
	glLoadIdentity ();

	glPolygonMode (GL_FRONT_AND_BACK, polygonMode);

	// -- Display first Scan -- 
	// do the model-rotation
	glRotatef (rotX, 1.0, 0.0, 0.0);  // rotate around X-Axis
	glRotatef (rotY, 0.0, 1.0, 0.0);  // rotate around Y-Axis
	glRotatef (rotZ, 0.0, 0.0, 1.0);  // rotate around Z-Axis

	// do the modell-transformation
	glTranslatef (X, 0.0 , 0.0);  // translate X
	glTranslatef (0.0, Y , 0.0);  // translate Y
	glTranslatef (0.0, 0.0 , Z);  // translate Z

	glPushMatrix ();  

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// to do
	glColor4f(1,1,1,1);
	glBegin(GL_POINTS);
	for(int i = 0; i < pointlist.size(); i++){
		glVertex4f(pointlist[i][0],pointlist[i][1],-1.0*pointlist[i][2],1.0);
		//cout << pointlist[i][0] << " " << pointlist[i][1] << " " << pointlist[i][2] << endl;
	}
	glEnd();
	glColor4f(1,0.2,0.2,1);
	glBegin(GL_POINTS);
	for(int i = 0; i < pointlist2.size(); i++){
		glVertex4f(pointlist2[i][0],pointlist2[i][1],-1.0*pointlist2[i][2],1.0);
		//cout << pointlist2[i][0] << " " << pointlist2[i][1] << " " << pointlist2[i][2] << endl;
	}
	glEnd();

	glFlush();
	glPopMatrix ();

	/* copy Back-Buffer to front */
	glFinish();
	glutSwapBuffers();

	return;
}

static void idle(){
	if(doICP){
		doICPIteration(tree);
		glutPostRedisplay();
	}

	usleep(500);
}

static void key(unsigned char key, int x, int y)
{
	// to do
	if(key=='u'){
		Y-=10;
	}
	else if(key=='j'){
		Y+=10;
	}
	else if(key=='h'){
		X+=10;
	}
	else if(key=='k'){
		X-=10;
	}
	else if(key=='t'){
		Z+=10;
	}
	else if(key=='g'){
		Z-=10;
	}
	else if(key=='r'){
		doICP = true;
	}
	else if(key=='R'){
		printf("stopped \n");
		doICP = false;
	}
	else if(key=='q' || (int)key == 27){
	  exit(1);
	}
	glutPostRedisplay();
	return;
}


// *************************
// main program starts here:
// *************************
int main(int argc, char **argv)
{
	// read pointlist
	ifstream infile;
	infile.open("points_1.pts");
	while (infile.good()) {
		double *point = new double[3];
		double dummy;
		infile >> point[0] >> point[1] >> point[2] >> dummy;
		//cout << point[0] << " " << point[1] << " " << point[2] << endl;
		pointlist.push_back(point);
	}

	// read pointlist
	ifstream infile2;
	infile2.open("points_2.pts");
	while (infile2.good()) {
		double *point2 = new double[3];
		double dummy;
		infile2 >> point2[0] >> point2[1] >> point2[2] >> dummy;
		//cout << point2[0] << " " << point2[1] << " " << point2[2] << endl;
		pointlist2.push_back(point2);
	}

	//Init KDtree
	double* pa[pointlist.size()];
	for(int i=0; i < pointlist.size(); i++){
		pa[i] = pointlist[i];
	}

	tree = new ANNkd_tree(pa, pointlist.size(), 3);
	
	frameNr = 0;

	/***************/
	/* init OpenGL */
	/***************/
	glutInit(&argc, argv);

	// define the windowposition and size
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(720, 576);

	// init and create display
	glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);
	window = glutCreateWindow("3D_Test");
	if (window == GL_FALSE) exit(1);

	// select callback functions
	glutDisplayFunc(display);
	glutKeyboardFunc(key);
	glutIdleFunc(idle);

	init(640, 480);

	/* pass the control over to the glutMainEventLoop */
	glutMainLoop();

	return 0;
}

