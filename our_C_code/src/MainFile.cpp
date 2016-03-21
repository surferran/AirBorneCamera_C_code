//// airborn camera system to detect movements in the recorded video
//  last update : 21/03/16
// Usage :
//	change file names in : \our_C_code\src\fback.cpp
//	base_out_file_path, rec_file_name


#define READ_FROM_FILE true

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//#include <ctime>				// ref by : http://stackoverflow.com/questions/2808398/easily-measure-elapsed-time 
#include "fback.cpp"



int main(int argc, char** argv)
{
	do_DOF(1,argv, READ_FROM_FILE);
	cvWaitKey(0);


	destroyAllWindows();	// despite the camera will be deinitialized AUTOmatically in VideoCapture destructor
							// using c:\OpenCV\build\include\opencv2\highgui.hpp
	return 0;
}