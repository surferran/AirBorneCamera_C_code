#ifndef FUNCTIONS_FILE1_HPP
#define FUNCTIONS_FILE1_HPP 
//#include "composed_algorithm.hpp"

#include "opencv2/opencv.hpp" 
using namespace cv;
using namespace std;
#include "app_globals.h"

void help_app();

// add some vectors accodring to scale factor , to the original image //
void drawOptFlowMap(	const Mat& flow , Mat& cflowmap , int step,		double vecFactor , const Scalar& color);

// algorithm functions. section 3.1 in the article //
void calc_bpm(Mat& flow_grad_mag, Mat& result)   ;

// algorithm functions. section 3.1 in the article // 
void calc_bpTheta( Mat& alpha_input,Mat& flowX ,Mat& flowY ,Mat& out_bpTheta );

// algorithm functions. section 3.1 in the article // 
void calc_bp_total(Mat& bpm, Mat& bpTheta, Mat& bp_Out) ;

// algorithm functions. section 3.1 in the article //
// calculates votes for horizontal and vertical directions //
void calc_votes_1(Mat& bp, Mat& out1 );

// algorithm functions. section 3.1 in the article //
// calculates votes for diagonal right and left directions //
void calc_votes_2(Mat& bp, Mat& out3)  ;

// algorithm functions. section 3.1 in the article //
// result is inside outside maps
void calc_total_8_votes(Mat& out1, Mat& out2, Mat& out3, Mat& out4, Mat& totalVotes) ;

// algorithm functions. section 3.1 in the article //
void calc_motion_boundaries(const Mat &flow, int *frame_prev_number, Mat &totalVotes);

// preperation for the algorithm input. //
// calls the segmentation function of SLIC //
void slic_for_frame(IplImage *image  , Slic &slic, long startingOffset)   ;

/* function present in calcintegralIntersections.cpp */
void calcintegralIntersections(  int height,  int width,  unsigned char* input ,
	unsigned char *output);

void copy_binaricMat_to_ucharArray(int w, int h,  Mat & bp ,  unsigned char * out_array);

void copy_ucharArray_to_binaricMat(int w, int h, unsigned char * in_array , Mat & bp  );

#endif