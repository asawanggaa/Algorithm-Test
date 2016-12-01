//
//  main.cpp
//  Algorithm Test
//
//  Created by t_wangju on 23/11/2016.
//  Copyright © 2016 t_wangju. All rights reserved.
//
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <GLUT/GLUT.h>
#include <OpenGL/OpenGL.h>

#include "PredictedPath.h"
#include "Eigen/Dense"

GLuint tex;

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace PredictedPath;

std::vector<Sample> CurveCut(std::vector<Sample> &curve, Sample endpoint){
	std::vector<Sample> answer;
	std::vector<Sample> surplus;
	auto iter=curve.begin();
	for(iter=curve.begin();iter!=curve.end();iter++){
		answer.push_back(*iter);
		if(iter->x==endpoint.x&&iter->y==endpoint.y){
			break;
		}
	}
	for(;iter!=curve.end();iter++){
		surplus.push_back(*iter);
	}
	curve=surplus;
	return answer;
}

static float Tension=0;
static float Bias=0;
static float Continuity=0;

Vector4f CubicCurveLinearRegression(std::vector<Sample> &curve){
	float width=(curve.end()-1)->x-(curve.begin())->x;
	float height=(curve.end()-1)->y-(curve.begin())->y;
	float h00=0,h01=0,h10=0,h11=0,dx0=0,dx1=0,dy0=0,dy1=0;//use for computing coefficients;
	
	int n=static_cast<int>(curve.size());
	float sx=width/n,sy=height/n,st=1.0f/n;
	float xi=0,yi=0,ti=0;
	for(auto iter=curve.begin();iter!=curve.end();iter++,ti+=st,xi+=sx,yi+=sy){
		float x=(iter->x-curve.begin()->x)/width;
		float y=(iter->y-curve.begin()->y)/height;
		h00+=(ti*ti-ti)*(ti*ti-ti);
		h01+=(ti*ti-ti)*(ti*ti*ti-ti);
		h10+=(ti*ti*ti-ti)*(ti*ti-ti);
		h11+=(ti*ti*ti-ti)*(ti*ti*ti-ti);
		dx0+=(ti*ti-ti)*(ti-x);
		dx1+=(ti*ti*ti-ti)*(ti-x);
		dy0+=(ti*ti-ti)*(ti-y);
		dy1+=(ti*ti*ti-ti)*(ti-y);
	}
	Matrix2f A;
	A<<h00,h01,h10,h11;
	Vector2f xb,yb;
	xb<<-dx0,-dx1;
	yb<<-dy0,-dy1;
	Vector2f x,y;
	x=A.colPivHouseholderQr().solve(xb);
	y=A.colPivHouseholderQr().solve(yb);
	// the theta is still from (0,0) to (1,1);
	// should transform it to (x0,y0)~(x1,y1);
	Vector4f theta;
	theta<<x(0),x(1),y(0),y(1);
	return theta;
}
std::vector<Sample> CubicCurveKochanekBartels(std::vector<Sample> &curve){
	std::vector<Sample> answer;
	if(curve.size()<4){
		return answer;
	}
	Matrix4f HMatrix;
	HMatrix<< 1,0,-3,2,0,1,-2,1,0,0,3,-2,0,0,-1,1;
	
	float t=Tension,b=Bias,c=Continuity;
	for(auto iter=curve.begin()+1;iter!=curve.end()-2;iter++){
		float dx0=(1-t)*(1+b)*(1+c)*(iter->x-(iter-1)->x)/2+(1-t)*(1-b)*(1-c)*((iter+1)->x-iter->x)/2;
		float dy0=(1-t)*(1+b)*(1+c)*(iter->y-(iter-1)->y)/2+(1-t)*(1-b)*(1-c)*((iter+1)->y-iter->y)/2;
		float dx1=(1-t)*(1+b)*(1-c)*((iter+1)->x-iter->x)/2+(1-t)*(1-b)*(1+c)*((iter+2)->x-(iter+1)->x)/2;
		float dy1=(1-t)*(1+b)*(1-c)*((iter+1)->y-iter->y)/2+(1-t)*(1-b)*(1+c)*((iter+2)->y-(iter+1)->y)/2;
		
		for(float t=0;t<=1;t+=0.01){
			Vector4f b,xv,yv;
			b<<1,t,t*t,t*t*t;
			xv<<iter->x,dx0,(iter+1)->x,dx1;
			yv<<iter->y,dy0,(iter+1)->y,dy1;
			float x=xv.transpose()*HMatrix*b;
			float y=yv.transpose()*HMatrix*b;
			Sample temp(x,y);
			answer.push_back(temp);
		}
	}
	return answer;
}
void DrawHandle(int x, int y, int Radius){
	double theta;
	
	glBegin(GL_LINE_LOOP);
	{
		for (int i=0; i<36; i++) {
			theta=i*2*M_PI/36;
			double dx=Radius*cos(theta)+x;
			double dy=Radius*sin(theta)+y;
			glVertex2d(dx, dy);
		}
	}
	glEnd();
}
std::vector<Sample> Sampler(std::vector<Sample> &curve, double segmentsize){
	std::vector<Sample> answer;
	answer.push_back(*curve.begin());
	double totaldis=0;
	for(auto iter=curve.begin();iter!=curve.end()-1;iter++){
		double dis=((iter+1)->y-iter->y)*((iter+1)->y-iter->y)+((iter+1)->x-iter->x)*((iter+1)->x-iter->x);
		dis=sqrt(dis);
		totaldis+=dis;
		if(totaldis>segmentsize){
			totaldis=0;
			answer.push_back(*(iter+1));
		}
	}
	answer.push_back(*(curve.end()-1));
	return answer;
}

std::vector<Sample> curve;
void testinit(int argc, char * argv[], int x, int y, int width, int height){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
	glutInitWindowPosition(x, y);
	glutInitWindowSize(width, height);
	glutCreateWindow("Algorithm Test");
	gluOrtho2D(x, x+width, y, y+height);
	
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glReadBuffer(GL_FRONT);
	glDrawBuffer(GL_BACK);
	int xmin=100;
	int xmax=width-100;
	int ymin=100;
	int ymax=height-100;
	
	glFlush();
	glutSwapBuffers();
}
void KeyboardFunc(unsigned char key, int x, int y){
	y=800-y;
	switch (key) {
		case 'q':
			Tension+=0.1;
			break;
		case 'w':
			Tension-=0.1;
			break;
		case 'a':
			Bias+=0.1;
			break;
		case 's':
			Bias-=0.1;
			break;
		case 'z':
			Continuity+=0.1;
			break;
		case 'x':
			Continuity-=0.1;
			break;
		case 'p':
		default:
			break;
	}
	std::vector<Sample> kcurve=PredictedPath::SolvePath(curve,10,4000);
	std::vector<Sample> kbcurve=CubicCurveKochanekBartels(curve);
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(0.0, 0.0, 0.0);
	glBegin(GL_LINE_STRIP);
	{
		for(auto iter=kcurve.begin();iter!=kcurve.end();iter++){
			glVertex2f(iter->x, iter->y+200);
		}
	}
	glEnd();
	glFlush();
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINE_STRIP);
	{
		for(auto iter=kbcurve.begin();iter!=kbcurve.end();iter++){
			glVertex2f(iter->x, iter->y+200);
		}
	}
	glEnd();
	for(auto iter=curve.begin();iter!=curve.end();iter++){
		DrawHandle(iter->x, iter->y+200, 5);
	}
	glFlush();
	glutSwapBuffers();
	std::cout<<"Tension:"<<Tension<<std::endl<<"Bias:"<<Bias<<std::endl<<"Continuity:"<<Continuity<<std::endl<<std::endl;
	return;
}

void texinit(const char * path){
	glShadeModel(GL_SMOOTH);
	glEnable(GL_COLOR_MATERIAL);
	glMatrixMode(GL_PROJECTION);
	
	cv::Mat img = cv::imread("/Users/t_wangju/Workplace/chessboard.jpg");
	cv::imshow("chessboard", img);
	
	cv::flip(img, img, 0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, (img.step&3)?1:4);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, img.step/img.elemSize());
	
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img.ptr());
	
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
	
}

void MouseClickFunc(int button, int state, int x, int y){
	if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN){
		glClear(GL_COLOR_BUFFER_BIT);
		curve.clear();
		Sample temp(x,800-y);
		curve.push_back(temp);
	}
	if(button==GLUT_LEFT_BUTTON&&state==GLUT_UP){
		glClear(GL_COLOR_BUFFER_BIT);
		Sample temp(x,800-y);
		curve.push_back(temp);
		curve=Sampler(curve, 100);
		std::vector<Sample> kcurve=PredictedPath::SolvePath(curve,10,400000);
		std::vector<Sample> kbcurve=CubicCurveKochanekBartels(curve);
		int count=0;
		
		for(auto iter=kcurve.begin();iter!=kcurve.end()-1;iter++){
			if(iter->x==(iter+1)->x){
				std::cout<<++count<<std::endl;
			}
		}
		
		glBegin(GL_LINES);
		{
			glVertex2i(0, 200);
			glVertex2i(1600, 200);
		}
		glEnd();
		
		glBegin(GL_LINE_STRIP);
		{
			for(auto iter=curve.begin();iter!=curve.end();iter++){
				glVertex2f(iter->x, iter->y);
			}
		}
		glEnd();
		
		for(auto iter=curve.begin();iter!=curve.end();iter++){
			DrawHandle(iter->x, iter->y, 5);
			DrawHandle(iter->x, iter->y+200, 5);
			DrawHandle(iter->x, iter->y+400, 5);
		}
		
		glColor3f(0.0f, 0.0f, 0.0f);
		glBegin(GL_LINE_STRIP);
		{
			for(auto iter=kcurve.begin();iter!=kcurve.end();iter++){
				glVertex2f(iter->x, iter->y+200);
			}
		}
		glEnd();
		glFlush();
		
		glColor3b(1.0, 0.0, 0.0);
		glBegin(GL_LINE_STRIP);
		{
			for(auto iter=curve.begin();iter!=curve.end();iter++){
				std::vector<Sample> tcurve=CurveCut(kcurve, *iter);
				if(tcurve.size()==0){
					continue;
				}
				float width=(tcurve.end()-1)->x-tcurve.begin()->x;
				float height=(tcurve.end()-1)->y-tcurve.begin()->y;
				Vector4f c=CubicCurveLinearRegression(tcurve);
				Vector4f xc,yc;
				xc<<0,1-c(0)-c(1),c(0),c(1);
				yc<<0,1-c(2)-c(3),c(2),c(3);
				for(float t=0;t<=1;t+=0.01){
					double x=width	*(xc(0)+xc(1)*t+xc(2)*t*t+xc(3)*t*t*t)+tcurve.begin()->x;
					double y=height	*(yc(0)+yc(1)*t+yc(2)*t*t+yc(3)*t*t*t)+tcurve.begin()->y;
					glVertex2f(x, y+400);
				}
			}
		}
		glEnd();
		glFlush();
		glutSwapBuffers();
	}
}

void MouseMoveFunc(int x, int y){
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(0.0, 0.0, 0.0);
	Sample temp(x,800-y);
	curve.push_back(temp);
	glBegin(GL_LINE_STRIP);
	{
		for(auto iter=curve.begin();iter!=curve.end();iter++){
			glVertex2i(iter->x, iter->y);
		}
	}
	glEnd();
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	{
		glVertex2i(0, 200);
		glVertex2i(800, 200);
	}
	glEnd();
	glFlush();
	glutSwapBuffers();
}

void DisplayFunc(void){
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	glVertex2i(0, 200);
	glVertex2i(800, 200);
	glEnd();
	glFlush();
	glutSwapBuffers();
}

int main(int argc, char * argv[]) {
	// insert code here...
	testinit(argc, argv, 0, 0, 1600, 800);
	texinit("/Users/t_wangju/Workplace/LinearWarp/chessboard.jpg");
	glutDisplayFunc(DisplayFunc);
	glutMouseFunc(MouseClickFunc);
	glutMotionFunc(MouseMoveFunc);
//	glutKeyboardFunc(KeyboardFunc);
	
	glutMainLoop();
	return 0;
}
