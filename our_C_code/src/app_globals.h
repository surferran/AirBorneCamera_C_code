/* file: app_globals.h */
#ifndef _APP_GLOBALS_H_ 
#define _APP_GLOBALS_H_

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

////////////////////////////////////////////////////////////////

struct Alg_parameters {
	double	lambda_magnitude		=	0.7; // number from article example code. it is used in equation (1) of the article.
	double	lamda_theta				=	1;	 // number from article example code. it is used in equation (2) of the article.
	double	mid_level				=	0.5; // binaric split level;

											 //  can be about 0.7 to 1.5, or else..
	//double	b_p_m_low_level			=	0.25;//	from article example code. in article it is not written
	//double	b_p_m_high_level		=	0.6; //	from article example code. in article it is T in equation (3)
	
	double	b_p_m_low_level			=	0.0105;//	from article example code. in article it is not written
	double	b_p_m_high_level		=	0.03; //	from article example code. in article it is T in equation (3)
	///double 	T2_threshold			=   0.077;//0.20; // 0.05 
	double 	BP_minimizing_factor	=   0.005;//0.1

};

struct App_flags {
	bool	read_from_file			= true;   // true-read video file. false - for live capture from camera(0)
	bool	measure_actions_timing	= true;		// to measure spesific calculations timings
	bool	record_outputs			= false;	// save frames into physical .avi files.
	bool	export_frames_to_Mat	= true;	// TODO: save frames into seperate .mat files. // for only optical flow now
	bool	do_frame_resize			= true;		// allow to choose if resizing input-frame or not

	int		resize_factor			=	1;		// make 1/factor
    int		frame_resize_W			=	320;	// Width  
	int		frame_resize_H			=	240;	// Height

	bool	allow_debug_plots_1		= false;		// plots in section calc_motion_boundaries() begining
	bool	allow_debug_plots_2		= false;		// plots in section calc_motion_boundaries() middle
};

////////////////////////////////////////////////////////////////

struct Main_App_parameters {
	App_flags		flags;
	int				number_of_frames_to_process = 30;		// in relation to video FPS. normally 30fps.
	Alg_parameters	algs_params;
};

static const Main_App_parameters App_Parameters;

// storage matrices to keep all alg outputs for each frame. current and previous
/* the structure is relevant for F_i (for pairs of Frame_i + Frame_i+1) */
struct Storage_4_frames {
	long	struct_size= 0;    //	the length of the built struct. it is the number of processed frames.
	Mat		DOF;				//	output of optical flow of current and previous frames.
	Slic	SPixels;			//	output of superpixels SLIC of current frame.
	Mat		Votes;				// 
};

#endif  //_APP_GLOBALS_H_