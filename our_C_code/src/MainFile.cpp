//// airborn camera system to detect movements in the recorded video
//  last update : 21/06/16
// Usage :
//	change file names in : \our_C_code\src\??.cpp
//		base_out_file_path, rec_file_name

// desired functions:
//   set_Mask()  - ignore, in every aspect, from pixels in this. borders, or constant broadcast texts and signs.
//					alarm (in yellow) about suspected pixels as not related to source video.

#include "composed_algorithm.hpp"


int main(int argc, char** argv)
{
	//	run_test_slic(); // for debug only. testing functionallity through intermidiate util.

	process_video_segmentation_algorithm(1,argv, App_Parameters.flags.read_from_file);
	
	cvWaitKey(0);  // allow user to notify the final state of the display windows

	destroyAllWindows();	// despite the camera will be deinitialized AUTOmatically in VideoCapture destructor
							// using c:\OpenCV\build\include\opencv2\highgui.hpp
	return 0;
}
