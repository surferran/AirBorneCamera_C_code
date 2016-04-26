//// airborn camera system to detect movements in the recorded video
//  last update : 21/03/16
// Usage :
//	change file names in : \our_C_code\src\fback.cpp
//	base_out_file_path, rec_file_name

/////TODO: check to disable auto-ident

//#define TUTUR_AND_TRIALS_MODE		
//#define REPLACE_MAIN_WITH_EXTERNAL  	//related to the above DEFINE

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

#include "app_globals.h"

// desired functions:
//   set_Mask()  - ignore, in every aspect, from pixels in this. borders, or constant broadcast texts and signs.
//					alarm (in yellow) about suspected pixels as not related to source video.

//#include <ctime>				// ref by : http://stackoverflow.com/questions/2808398/easily-measure-elapsed-time 
#include "fback.cpp"

#ifdef TUTUR_AND_TRIALS_MODE
//#include "another\SLIC.cpp"

// ref by: http://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html
// input line can be : C:\OpenCV\sources\samples\data\aero1.jpg 20
#include "c:\OpenCV\sources\samples\cpp\tutorial_code\core\how_to_scan_images\how_to_scan_images.cpp"

#endif

#ifndef REPLACE_MAIN_WITH_EXTERNAL
int main(int argc, char** argv)
{
	//TODO: add print of app parameters to console, or log.

#ifdef TUTUR_AND_TRIALS_MODE
	//run_test_slic();
//	do_slic(argc,argv);
	return 0;
#endif

	do_DOF(1,argv, App_Parameters.flags.read_from_file);
	cvWaitKey(0);


	destroyAllWindows();	// despite the camera will be deinitialized AUTOmatically in VideoCapture destructor
							// using c:\OpenCV\build\include\opencv2\highgui.hpp
	return 0;
}
#endif