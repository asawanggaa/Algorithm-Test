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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <GLUT/GLUT.h>
#include <OpenGL/OpenGL.h>

#include "path/PredictedPath.h"
#include "path/Eigen/Dense"

GLuint tex;

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace PredictedPath;

std::vector<Vector3d> CurveToCubic(std::vector<Sample> curve){
	std::vector<Vector3d> answer;
	return answer;
}



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
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img.ptr());
	
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
	
}
std::vector<Sample> curve;
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
		std::vector<Sample> smoothcurve=PredictedPath::SolvePath(curve,10,4000);
		std::vector<Vector3d> MCI;
		std::cout<<curve.size()<<std::endl;glBegin(GL_LINES);
		{
			glVertex2i(400, 0);
			glVertex2i(400, 800);
			glVertex2i(0, 400);
			glVertex2i(800, 400);
		}
		glEnd();
		glBegin(GL_LINE_STRIP);
		{
			for(auto iter=curve.begin();iter!=curve.end();iter++){
				glVertex2f(iter->x, iter->y);
			}
		}
		glEnd();
		glBegin(GL_LINE_STRIP);
		{
			for(auto iter=smoothcurve.begin();iter!=smoothcurve.end();iter++){
				glVertex2f(iter->x, iter->y+400);
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
		glVertex2i(400, 0);
		glVertex2i(400, 800);
		glVertex2i(0, 400);
		glVertex2i(800, 400);
	}
	glEnd();
	glFlush();
	glutSwapBuffers();
}

void DisplayFunc(void){
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);
	glVertex2i(400, 0);
	glVertex2i(400, 800);
	glVertex2i(0, 400);
	glVertex2i(800, 400);
	glEnd();
	glFlush();
	glutSwapBuffers();
}

int main(int argc, char * argv[]) {
	// insert code here...
	testinit(argc, argv, 0, 0, 800, 800);
	texinit("/Users/t_wangju/Workplace/LinearWarp/chessboard.jpg");
	glutDisplayFunc(DisplayFunc);
	glutMouseFunc(MouseClickFunc);
	glutMotionFunc(MouseMoveFunc);
//	glutKeyboardFunc(KeyboardFunc);
	
	glutMainLoop();
	return 0;
}
