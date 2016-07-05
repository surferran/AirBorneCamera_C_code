#ifndef FUNCTIONS_FILE1_HPP
#define FUNCTIONS_FILE1_HPP 
#include "composed_algorithm.hpp"

void help_fback();

// add some vectors accodring to scale factor , to the original image //
void drawOptFlowMap(	const Mat& flow , Mat& cflowmap , int step,		double vecFactor , const Scalar& color);

// algorithm functions. section 3.1 in the article //
Mat& calc_bpm(Mat& I)  ;

// algorithm functions. section 3.1 in the article //
bool calc_bpTheta( Mat& I ,Mat& out_bpTheta ) ;

// algorithm functions. section 3.1 in the article //
Mat& calc_bp_total(Mat& bpm, Mat& bpTheta) ;

// algorithm functions. section 3.1 in the article //
// calculates votes for horizontal and vertical directions //
Mat& calc_votes_1(Mat& bp, Mat& out1 );

// algorithm functions. section 3.1 in the article //
// calculates votes for diagonal right and left directions //
Mat& calc_votes_2(Mat& bp, Mat& out3)  ;

// algorithm functions. section 3.1 in the article //
// result is inside outside maps
Mat& calc_total_8_votes(Mat& out1, Mat& out2, Mat& out3, Mat& out4, Mat& totalVotes) ;

// algorithm functions. section 3.1 in the article //
void calc_motion_boundaries(const Mat &flow, Mat &totalVotes);

// preperation for the algorithm input. //
// calls the segmentation function of SLIC //
void slic_for_frame(IplImage *image  , Slic &slic, long startingOffset)   ;

#endif