#include <opencv2/opencv.hpp>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

using namespace cv;

void CVUIShow(double* kp_s, double* ki_s, double* kd_s, double* kp_d, double* ki_d, double* kd_d) {
	Mat dashBoard = Mat::zeros(cv::Size(800, 400), CV_8UC3);
	cvui::init("pid");
	waitKey(10);

	cvui::window(dashBoard, 0, 0, 800, 200, "speed");
	
	cvui::trackbar(dashBoard, 10, 20, 750, kp_s, 0., 1.);
	cvui::trackbar(dashBoard, 10, 70, 750, ki_s, 0., 1.);
	cvui::trackbar(dashBoard, 10, 120, 750, kd_s, 0., 1.);
	
	cvui::window(dashBoard, 0, 200, 800, 200, "direction");

	cvui::trackbar(dashBoard, 10, 220, 750, kp_d, 0., 50.);
	cvui::trackbar(dashBoard, 10, 270, 750, ki_d, 0., 1.);
	cvui::trackbar(dashBoard, 10, 320, 750, kd_d, 0., 100.);
	

	cvui::update();
	cvui::imshow("pid", dashBoard);
}

void CVUIShow_2(double* previewPointSpeed,int* delta/*,double* directionErrorPoint*/) {
	Mat dashBoard = Mat::zeros(cv::Size(400, 250), CV_8UC3);
	cvui::init("control");
	waitKey(10);

	cvui::window(dashBoard, 0, 0, 400, 200, "k");
	cvui::trackbar(dashBoard, 10, 20, 350, previewPointSpeed, 0., 300.);
	cvui::window(dashBoard, 0, 80, 400, 200, "delta");
	cvui::trackbar(dashBoard, 10, 100, 350, delta, 0, 50);
	// cvui::window(dashBoard, 0, 160, 400, 200, "directionErrorPoint");
	// cvui::trackbar(dashBoard, 10, 180, 350, directionErrorPoint, 0., 50.);

	cvui::update();
	cvui::imshow("control", dashBoard);

}


void CVUIShow_3(double* r1, double* r1_min,double* r1_max, double* r2, double* r2_min, double* r2_max, double* r3, double* r3_min, double* r3_max, double* else_min, double* else_max) {
	Mat dashBoard = Mat::zeros(cv::Size(800, 400), CV_8UC3);
	cvui::init("speed");
	waitKey(10);

	cvui::window(dashBoard, 0, 0, 800, 400, "r");
	cvui::trackbar(dashBoard, 10, 20, 750, r1, 0., 350.);
	cvui::trackbar(dashBoard, 10, 70, 300, r1_min, 0., 350.);
	cvui::trackbar(dashBoard, 390, 70, 300, r1_max, 0., 350.);
	cvui::trackbar(dashBoard, 10, 120, 750, r2, 0., 350.); 
	cvui::trackbar(dashBoard, 10, 170, 300, r2_min, 0., 350.);
	cvui::trackbar(dashBoard, 390, 170, 300, r2_max, 0., 350.);
	cvui::trackbar(dashBoard, 10, 220, 750, r3, 0., 350.);
	cvui::trackbar(dashBoard, 10, 270, 300, r3_min, 0., 350.);
	cvui::trackbar(dashBoard, 390, 270, 300, r3_max, 0., 350.);
	cvui::trackbar(dashBoard, 10, 320, 300, else_min, 0., 350.);
	cvui::trackbar(dashBoard, 390, 320, 300, else_max, 0., 350.);

	cvui::update();
	cvui::imshow("speed", dashBoard);
}