/* file: app_globals.h */
#ifndef _APP_GLOBALS_H_ 
#define _APP_GLOBALS_H_

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

////////////////////////////////////////////////////////////////

struct Alg_parameters {
	double	lambda_m				= 0.7; // assing note-reff number..
};

struct App_flags {
	bool	read_from_file			= true;   // true-read video file. false - for live capture from camera(0)
	// not used //bool	convert_RGB_2_Gray		= true;	// need for preparing rgb(or BRG) frames to optical flow calculation
	bool	measure_actions_timing	= true;		// to measure spesific calculations timings
	bool	record_outputs			= false;	// save frames into physical .avi files.
	bool	export_frames_to_Mat	= false;	// save frames into seperate .mat files. // for only optical flow (and slic?) now
	bool	do_frame_resize			= true;		// allow to choose if resizing input-frame or not

	bool	allow_debug_plots_1		= false;		// plots in section calc_motion_boundaries() begining
	bool	allow_debug_plots_2		= false;		// plots in section calc_motion_boundaries() middle
//false;//
};

////////////////////////////////////////////////////////////////

struct Main_App_parameters {
	App_flags		flags;
	int				number_of_frames_to_process = 30;		// in relation to video FPS. normally 30fps.
	Alg_parameters	algs_params;

};

static const Main_App_parameters App_Parameters;


/// storage matrices to keep all alg outputs for each frame. current and previous
// needs to keep arrays
struct Storage_4_frames {
	long	struct_size= 0;    //	the length of the built struct. it is the number of processed frames.
	Mat		DOF;				//	output of optical flow of current and previous frames.
	Slic	SPixels;			//	output of superpixels SLIC of current frame.
};

#endif  //_APP_GLOBALS_H_