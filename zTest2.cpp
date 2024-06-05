#include "opencv2/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

/// UART Communication: PI <--> Cortex:
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <string>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <unistd.h>

using namespace cv;
using namespace std;

int thresh = 100;
RNG rng(12345);

double largest_area = 0;
double min_area = 500;
int largest_contour_index = 0;
Rect bounding_rect;
double as[100];
int ascount = 0;
int WIDTH;
int HEIGHT;

int fd ;
int count;
std::string message;
void comu1(float x);
void takeAction(int action);

// ***** Motor Control *****
int in1 = 17;
int in2 = 18;
int enA = 19;
int in3 = 26;
int in4 = 19;
int enB = 12;
void initializeMotorsAB(void);
void stopBothMotors(void);
void moveMotorA(int motorLevel);
void moveMotorB(int motorLevel);
void moveBothMotors(int levelA, int levelB);
using namespace std;

int main(int argc, char** argv)
{
    initializeMotorsAB();
    stopBothMotors();

    if (wiringPiSetup () == -1)
    {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
        return 1;
    }

	VideoCapture cap(0);
	if ( !cap.isOpened() )
	{
		cout << "Cannot open the web cam" << endl;  return -1;
	}

	namedWindow("Original", CV_WINDOW_AUTOSIZE);
	moveWindow("Original", 300, 0);
    namedWindow("Thresholded", CV_WINDOW_AUTOSIZE);
    moveWindow("Thresholded", 550, 0);
    namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	moveWindow("Contours", 300, 200);

    while ( true )
	{
		Mat imgOriginal;
		bool bSuccess = cap.read( imgOriginal );
		if ( !bSuccess )
        {
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}
		cv::resize(imgOriginal, imgOriginal, cv::Size(), 0.25, 0.25);

		Mat imgHSV, imgThresholded;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
		// red or pink
		inRange(imgHSV, Scalar(166, 120, 0), Scalar(180, 255, 255), imgThresholded);
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		Mat canny_output;
		Canny(imgThresholded, canny_output, thresh, thresh * 2, 3);

        vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

		largest_contour_index = -1;
		largest_area = 0.0;
		for ( int i = 0; i < (int) contours.size(); i++ )
		{
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
			double a = contourArea(contours[i], false);
			if ( ascount <= 99 )
			{
				as[ascount] = a;
				ascount++;
			}

			if ( a > largest_area && a > min_area )
			{
				largest_area = a;
				largest_contour_index = i;
				bounding_rect = boundingRect(contours[i]);
			}
		}

		Point objCenter(-1, -1);
		if ( largest_contour_index >= 0 )
		{
			rectangle(drawing, bounding_rect, Scalar(0, 255, 0), 1, 8, 0);
			objCenter = ( bounding_rect.br() + bounding_rect.tl() )*0.5;
            circle( drawing, objCenter, 5, Scalar(0,255,0) );
		}

        if( objCenter.x != -1 && objCenter.y != -1 )
        {
            WIDTH = drawing.size().width;
            HEIGHT = drawing.size().height;
            comu1(objCenter.x);
        }

        imshow("Original",    imgOriginal);
        imshow("Thresholded", imgThresholded);
        imshow("Contours",    drawing);

		if ( waitKey(30) == 27 || waitKey(30) == 113 ){
                cout << "ESC pressed by user " << endl; break;}
	}

	cv::destroyAllWindows();
	return 0;
}

void comu1( float x )
{
    int action = -1;
    int num_partition = 5;
    int one_partition = (int) (WIDTH/num_partition);

    if( x < one_partition * 2 )
    {
        cout << "x = " << x << "  Left" << endl;
        message = "\nLeft";
        action = 0;
    }
    else if (x >= one_partition*2 && x < 3*one_partition )
    {
        cout << "x = " << x << "  Straight" << endl;
        message = "\nStraight";
        action = 1;
    }
    else
    {
        cout << "x = " << x << "  Right" << endl;
        message = "\nRight";
        action = 2;
    }

    //takeAction(action);
}

void initializeMotorsAB(void)
{
    wiringPiSetupGpio();
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(enA, PWM_OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	pinMode(enB, PWM_OUTPUT);
}

void stopBothMotors(void)
{
	digitalWrite(in1, 0);
	digitalWrite(in2, 0);
	pwmWrite(enA, 0); sleep(1);

	digitalWrite(in3, 0);
	digitalWrite(in4, 0);
	pwmWrite(enB, 0); sleep(1);
}

void moveMotorA(int motorLevel)
{
    if(motorLevel > 0)  //forward
    {
        digitalWrite(in1, 0);
        digitalWrite(in2, 1);
    }
    else{           //backward
        digitalWrite(in1, 1);
        digitalWrite(in2, 0);
    }
	pwmWrite(enA, abs(motorLevel));
	sleep(1);
}

void moveMotorB(int motorLevel)
{
    if(motorLevel > 0)  //forward
    {
        digitalWrite(in3, 0);
        digitalWrite(in4, 1);
    }
    else{               //backward
        digitalWrite(in3, 1);
        digitalWrite(in4, 0);
    }
	pwmWrite(enB, abs(motorLevel));
	sleep(1);
}

void takeAction(int action)
{
    switch (action){
    case 0: // rotate left
        moveMotorA(10);
    case 1: // move straight
        moveBothMotors(10, 10);
    case 2: // rotate right
        moveMotorB(10);
    default:
        stopBothMotors();
    }
}

void moveBothMotors(int levelA, int levelB)
{
    if(levelA > 0)  //forward
    {
        digitalWrite(in1, 0);
        digitalWrite(in2, 1);
    }
    else{           //backward
        digitalWrite(in1, 1);
        digitalWrite(in2, 0);
    }

    if(levelB > 0)  //forward
    {
        digitalWrite(in3, 0);
        digitalWrite(in4, 1);
    }
    else{           //backward
        digitalWrite(in3, 1);
        digitalWrite(in4, 0);
    }

	pwmWrite(enA, abs(levelA));
	pwmWrite(enB, abs(levelB));
	sleep(1);
}
