#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <cmath>
using namespace cv;

#include <sys/time.h>


#define PI 3.14159265
#define thetaStep 10
#define Taille 0.5

#define VIDEO 1

double calculateCpuUsage(clock_t start, clock_t end) {
	double cpuUsage = static_cast<double>(end - start) / CLOCKS_PER_SEC;
	return cpuUsage * 100.0;
}


void Sobel1_threaded(Mat frame, int * filterGX,int * filterGY, int size, Mat * out, int limit, int startRow, int endRow){

  int step = std::floor(size/2);
  float sumX,sumY;
  
  clock_t threadStart = clock();
  
  for(int y=startRow;y<endRow;y++){
    for(int x=1;x<frame.rows-1;x++){
      sumX=0;
      sumY=0;

      for(int i=0;i<size;i++){
        for(int j=0;j<size;j++){
          sumX+=filterGX[i*size+j]*frame.at<uchar>(Point(y-step+i,x-step+j));
          sumY+=filterGY[i*size+j]*frame.at<uchar>(Point(y-step+i,x-step+j));
        }
      }
      float magnitude = sqrt(pow(sumX,2)+pow(sumY,2))/4;
      out->at<uchar>(Point(y,x)) = magnitude;

      if(magnitude<limit)
        out->at<uchar>(Point(y,x)) = 0;
      //else
        //out->at<uchar>(Point(y,x)) = 0;

    }
  }
  threshold(*out, *out, limit, 255, THRESH_BINARY);
  
  clock_t threadEnd = clock();
  double cpuUsage = calculateCpuUsage(threadStart, threadEnd);
  std::cout << "Thread " << std::this_thread::get_id() << " CPU Usage " << cpuUsage << "%" << std::endl;
}

void Sobel1(Mat frame, int * filterGX,int * filterGY, int size, Mat * out, int limit) {
	
	//Max de thread
	int numThreads = std::thread::hardware_concurrency();
	numThreads = 4;
	printf("Nb threads = %d\n", numThreads);
	int rowsPerThread = frame.cols / numThreads;
	
	std::vector<std::thread> threads(numThreads);
	
	for(int i = 0; i < numThreads; i++) {
		int startRow = (i == 0) ? 1 : i * rowsPerThread;
		int endRow = (i == numThreads - 1) ? frame.cols - 1 : (i + 1) * rowsPerThread;
		threads[i] = std::thread(Sobel1_threaded, frame, filterGX, filterGY, size, out, limit, startRow, endRow);
	}
	
	for(int i = 0; i < numThreads; i++) {
		threads[i].join();
	}
}

int diff_ms(timeval t1, timeval t2)
{
    return (((t1.tv_sec - t2.tv_sec) * 1000000) +
            (t1.tv_usec - t2.tv_usec))/1000;
}

// Convert RGB image to grayscale using the luminosity method
void RGBtoGrayScale(Mat rgb, Mat* grayscale){
  std::vector<Mat> channels(3);
  split(rgb, channels);
  *grayscale = (0.07*channels[0] + 0.72*channels[1] + 0.21*channels[2]);
}

double calculateAngle(int x1, int y1, int x2, int y2){
	double dx = static_cast<double>(x2 - x1);
	double dy = static_cast<double>(y2 - y1);
	
	double angleRad = atan2(dy, dx);
	
	double angleDeg = angleRad * 180.0 / M_PI;
	
	angleDeg -= 90.0;
	
	if(angleDeg > 90.0) {
		angleDeg -= 180.0;
	} else if(angleDeg < -90.0) {
		angleDeg += 180.0;
	}
	
	return angleDeg * (-1);
	
}

void simpleHough_threaded(Mat frame, Mat* acc, Mat *f, int startRow, int endRow){
  clock_t threadStart = clock();
	
  int channels = frame.channels();
  int nRows = frame.rows;
  int nCols = frame.cols * channels;
  //const uchar* image = frame.ptr();
  int step = (int)frame.step;
  //int stepacc = (int)acc->step;
  if (frame.isContinuous())
  {
      nCols *= nRows;
      nRows = 1;
  }

  int i,j;
  double rho;
  for( i = startRow; i < endRow; i++ ){
    for( j = 0; j < frame.cols; j++ ){
      if(frame.data[i * step + j]!=0){
        for(int theta=0;theta<180; theta+=thetaStep){
          rho = j*cos((double)theta*PI/180)+i*sin((double)theta*PI/180);
          if(rho!=0)
            acc->at<ushort>(Point(cvRound(rho) ,(int)cvRound(theta/thetaStep)))+=1;
        }
      }
    }
  }
  
  clock_t threadEnd = clock();
  double cpuUsage = calculateCpuUsage(threadStart, threadEnd);
  std::cout << "Thread " << std::this_thread::get_id() << " CPU Usage " << cpuUsage << "%" << std::endl;
}


double simpleHough(Mat frame, Mat* acc, Mat *f, float max_height_percent){
	//Max de thread (4 sur la raspberry)
	int numThreads = std::thread::hardware_concurrency();
	
	int max_height = max_height_percent*frame.rows;
	
	int rowsPerThread = (frame.rows - max_height) / numThreads;
	
	std::vector<std::thread> threads(numThreads);
	
	
	for(int i = 0; i < numThreads; i++) {
		int startRow = (i == 0) ? max_height : i * rowsPerThread + max_height;
		int endRow = (i == numThreads - 1) ? frame.rows : (i + 1) * rowsPerThread + max_height;
		threads[i] = std::thread(simpleHough_threaded, frame, acc, f, startRow, endRow);
	}
	
	for(int i = 0; i < numThreads; i++) {
		threads[i].join();
	} 
	
	cv::Point min_loc, max_loc;
	cv::Point min_loc_old, max_loc_old;
	double min, max;
	cv::minMaxLoc(*acc, &min, &max, &min_loc_old, &max_loc_old);

	Point pt1, pt2;
	double a ,b;
	double x0, y0;
	double theta;
	double angleTotal = 0.0;
	double angle;
	double angleMoyen = 0.0;
	int nbAngles = 0;
	//int step = (int)frame.step;
	int stepacc = (int)acc->step;

  if (max == 0) {
      // Aucune ligne détectée, retourner une valeur par défaut pour l'angle
      return 0.0;
  }

	acc->data[max_loc_old.y * stepacc +max_loc_old.x]=0;
	for(int i=0;i<4;i++){
		cv::minMaxLoc(*acc, &min, &max, &min_loc, &max_loc);
		if(abs(max_loc_old.x-max_loc.x)>5 || abs(max_loc_old.y-max_loc.y)>5){ //might be interesting to use that ....
			theta = (double)max_loc.y*thetaStep;
			a = cos(theta*PI/180); //compute hough inverse transform from polar to cartesian
			b = sin(theta*PI/180);
			x0 = a*max_loc.x;
			y0 = b*max_loc.x;
			pt1.x = cvRound(x0 + 1000*(-b)); //compute first point belonging to the line
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b)); //compute second point
			pt2.y = cvRound(y0 - 1000*(a));
			line( *f, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
			angle = calculateAngle(pt1.x, pt1.y, pt2.x, pt2.y);
			angleTotal += angle;
			nbAngles ++;
			acc->at<ushort>(Point(max_loc.x ,max_loc.y))=0;
			max_loc_old.x = max_loc.x;
			max_loc_old.y = max_loc.y;
		}
		else{
			acc->at<ushort>(Point(max_loc.x ,max_loc.y))=0;
			i--;
		}
	}
	if(nbAngles > 0){
		angleMoyen = angleTotal / nbAngles;
	}
	return std::round(angleMoyen);
}



int main(int argc, char** argv)
{
  timeval start, end;
  printf("OK !");
  char name[50];
  Mat frame;
  Mat canny;

  #if VIDEO
  VideoCapture cap(1); // open the default camera
  if(!cap.isOpened()){  // check if we succeeded
      printf("Error capture");
      return -1;
  }
  cap >> frame;
  #endif
  #if VIDEO==0
  //Mat frame;
  frame = imread(argv[1], IMREAD_COLOR);

  if(! frame.data )                              // Check for invalid input
  {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    #endif


    int x = frame.rows;
    int y = frame.cols;
    Size pt = Size(sqrt(pow(x,2)+pow(y,2))*2,ceil(180/thetaStep));
    printf("rows : %d, cols : %d \n",frame.rows, frame.cols);
    Mat acc = Mat::zeros(pt, CV_16UC1);
    printf("rho : %d, theta : %d \n",acc.cols, acc.rows); //size of the accumulation matrix
    Mat grayscale = Mat::zeros(Size(frame.rows,frame.cols),CV_8UC1);
    Mat sobel = Mat::zeros(Size(frame.cols,frame.rows),CV_8UC1);
    //int filterGX[9] = {-1,0,1,-2,0,2,-1,0,1};
    //int filterGY[9] = {-1,-2,-1,0,0,0,1,2,1};
    //int convSize = 3;
    namedWindow("Color Frame",1);
    namedWindow("Sobel Frame",1);

    #if VIDEO==0 //if image mode selected, must have image path as first argument
    RGBtoGrayScale(frame,&grayscale);
    GaussianBlur(grayscale, grayscale, Size(9,9), 2, 2);
    //Sobel1(grayscale,filterGX,filterGY,convSize, &sobel,25);
    Canny( grayscale, sobel, 60, 60*3,3);
    int count=0;
    count=countNonZero(sobel);
    printf("nb_pixel = %d\n", count);
    double angle = 0.0;
    angle = simpleHough(sobel,&acc,&frame, 0.5);
    
    std::cout << "Angle à suivre = " << angle << " degrés" << std::endl;
	
    imshow( "Color Frame", frame );  // Show our image with hough line detection
    imshow( "Sobel Frame", sobel );
    //imshow("Acc Frame", acc);
    waitKey(0); //wait for key pressed in order to propely close all opencv windows
    #else //if video selected, no needs of arguments in the program call
    for(;;)
    {		
      clock_t mainStart = clock();
      
      Mat res;
      gettimeofday(&start,NULL);
      cap >> frame; // get a new frame from camera
      acc = Mat::zeros(pt, CV_16UC1); //16 bits for accumulatio matrix
      RGBtoGrayScale(frame,&grayscale);
      GaussianBlur(grayscale, grayscale, Size(9,9), 1.5, 1.5);
      //Sobel1(grayscale,filterGX,filterGY,3, &sobel,20); //sobel filter, not optimized though
      Canny( grayscale, sobel, 60, 60*3,3); //opencv canny filter, use to compare performances
      double angle = 0.0;
      angle = simpleHough(sobel,&acc,&frame, Taille);
      std::cout << "Angle à suivre = " << angle << " degrés" << std::endl;
      gettimeofday(&end,NULL);
      int ms = diff_ms(end,start);
          
      clock_t mainEnd = clock();
      double cpuUsage = calculateCpuUsage(mainStart, mainEnd);
      std::cout << "Main Thread CPU Usage " << cpuUsage << "%" << std::endl;
		
      //normalize(acc,acc,0,255,NORM_MINMAX, CV_16UC1); //normalize mat, use at your discretion
      sprintf (name, "fps : %f", 1/((double)ms/1000));
      putText(frame, name, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(254,20,25), 1, CV_AA);

      imshow( "Color Frame", frame );  // Show image with Hough line detection
      imshow( "Sobel Frame", sobel );  // show filter image
      //imshow("Acc Frame", acc);
      if(waitKey(30) >= 0)break;
    }
    #endif
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
