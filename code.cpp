#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

// Mat img;
Mat frame;
Mat RGBframe;
Mat Grayframe;
Mat Grayframe_not;
Mat Threshframe;
Mat Edgeframe;
Mat Finalframe;
Mat FinalframeDuplicate;
Mat Matrix;
Mat BirdEyeframe;
Mat RoILane;
Point2f RoI_points[] = {Point2f(78,145), Point2f(310,145), Point2f(13,195), Point2f(360,195)};  // Region of interest points (to modify)
Point2f BEP_points[] = {Point2f(100,0), Point2f(280,0), Point2f(100,240), Point2f(280,240)};		// Bird eye points
RaspiCam_Cv Camera;
vector<int> histogramLane;
int frameWidth = 400;
int LeftLanePos;
int RightLanePos;
int lineCenter;
int Result;
int frameCenter;
stringstream ss;
stringstream ss1, ss2, ss3;
int dist_Stop;

CascadeClassifier stop_cascade;
Mat frame_stop, RoI_stop, gray_stop;
vector<Rect> stop;

void SetupCamera(int argc, char **argv, RaspiCam_Cv &Camera){
	Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv, 0));
}

void Capture(){
	Camera.grab();
	Camera.retrieve(frame);
	cvtColor(frame, RGBframe, COLOR_BGR2RGB);
	cvtColor(frame, frame_stop, COLOR_BGR2RGB);
}

void RegionOfInterest(){
	Matrix = getPerspectiveTransform(RoI_points, BEP_points);
	warpPerspective(RGBframe, BirdEyeframe, Matrix, Size(360, 240));
	
}

void ThresholdFilter(){
	cvtColor(BirdEyeframe, Grayframe, COLOR_RGB2GRAY);
	inRange(Grayframe, 120, 250, Threshframe); 			
	Canny(Grayframe, Edgeframe, 300, 400, 3, false); 	
	add(Threshframe, Edgeframe, Finalframe);
	cvtColor(Finalframe, Finalframe, COLOR_GRAY2RGB);
	cvtColor(Finalframe, FinalframeDuplicate, COLOR_RGB2BGR);   //used in histrogram function only
}

void Histogram()
{
    histogramLane.resize(400);
    histogramLane.clear();
    
    for(int i=0; i<360; i++)       //frame.size().width = 400
    {
	RoILane = FinalframeDuplicate(Rect(i,140,1,100));
	divide(255, RoILane, RoILane);
	histogramLane.push_back((int)(sum(RoILane)[0])); 
    }
}

void LaneFinder(){

    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histogramLane.begin(), histogramLane.begin() + 150);
    LeftLanePos = distance(histogramLane.begin(), LeftPtr); 
    
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histogramLane.begin() + 250, histogramLane.end());
    RightLanePos = distance(histogramLane.begin(), RightPtr);
    
    line(Finalframe, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 255,0), 2);
    line(Finalframe, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2); 
}

// Calibration
void LaneCenter()
{
    lineCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos;
    frameCenter = 184;  									// tbedel for calibration
    
    line(Finalframe, Point2f(lineCenter,0), Point2f(lineCenter,240), Scalar(0,255,0), 3);           // Road center
    line(Finalframe, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);			// Frame center

    Result = lineCenter-frameCenter;
}
		
void StopDetection()
{
	if(!stop_cascade.load("//home//pi//Desktop//cascade.xml"))
	{
		cout << "Unable to open the file" << endl;
	}
	RoI_stop = frame_stop(Rect(0,0,400,240));
    cvtColor(RoI_stop, gray_stop, COLOR_RGB2GRAY);
    equalizeHist(gray_stop, gray_stop);
    stop_cascade.detectMultiScale(gray_stop, stop);
    
    for(int i=0; i<stop.size(); i++)
    {
	Point P1(stop[i].x, stop[i].y);
	Point P2(stop[i].x + stop[i].width, stop[i].y + stop[i].height);
	
	rectangle(RoI_stop, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_stop, "Stop Sign", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);
	dist_Stop = (-0.38)*(P2.x-P1.x) + 55.84;
	
    ss.str(" ");
    ss.clear();
    ss<<"D = "<<dist_Stop<<"cm";
    putText(RoI_stop, ss.str(), Point2f(1,130), 0,1, Scalar(0,0,255), 2);
    
    if(dist_Stop <= 20){
	ss.str(" ");
    ss.clear();
    ss<<"Stop";
    putText(RoI_stop, ss.str(), Point2f(1,230), 0,1, Scalar(0,255,0), 2);
    
	}
	}
	
}

int main(int argc, char **argv){
	wiringPiSetup();
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
	SetupCamera(argc, argv, Camera);
	cout << "Connecting to camera" << endl;
	if(!Camera.open()){
		cout << "Failed to connect!" << endl;
		return -1;
	}
	cout << "Camera ID  : " << Camera.getId() << endl;
	
	while(1){
		auto start = std::chrono::system_clock::now();
		Capture();
		RegionOfInterest();
		ThresholdFilter();
		Histogram();
		LaneFinder();
		LaneCenter();
		StopDetection();
		
		ss.str(" ");
		ss.clear();
		ss<<"Result = "<<Result;
		ss1.str(" ");
		ss1.clear();
		ss1<<"Forward";
		ss2.str(" ");
		ss2.clear();
		ss2<<"Turn right";
		ss3.str(" ");
		ss3.clear();
		ss3<<"Turn left";
		
		
		if(Result <= 10 && Result >= -10){
			putText(RGBframe, ss1.str(), Point2f(1,100), 0,1, Scalar(230,0,255), 2);
		} else if(Result < 10){
			putText(RGBframe, ss3.str(), Point2f(1,100), 0,1, Scalar(230,0,255), 2);
		} else if(Result > 10){
			putText(RGBframe, ss2.str(), Point2f(1,100), 0,1, Scalar(230,0,255), 2);
		}
		
		
		
		
		/**** Display ****/
		namedWindow("Stop Sign", WINDOW_KEEPRATIO);
		moveWindow("Stop Sign", 1280, 580);
		resizeWindow("Stop Sign", 640, 480);
		imshow("Stop Sign", RoI_stop);
		
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_time = end - start;
		float t = elapsed_time.count();
		int fps = 1/t;
		cout << "frame per second : " << fps << endl; 
		imshow("Video", RGBframe);
		waitKey(1);
		
	}
	
	return 0;
}
