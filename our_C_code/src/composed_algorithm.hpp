#ifndef COMPOSED_ALGORITHM_HPP
#define COMPOSED_ALGORITHM_HPP 

#include <iostream>
#include "opencv2/opencv.hpp"
#include "Slic.h"

using namespace cv;
using namespace std;

#include "app_globals.h"
#include "ourConverted\calcSpatialConnections.hpp"
#include "ourConverted\calcTemporalConnections.hpp"
#include "ourConverted\calcSuperpixelInRatio.hpp"

///#include "some_utils\writeMat.hpp"   //TODO: check how to include this and operate the function
//#include <ctime>				// ref by : http://stackoverflow.com/questions/2808398/easily-measure-elapsed-time 

static void help_fback();

// add some vectors accodring to scale factor , to the original image //
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double vecFactor, const Scalar& color);

// algorithm functions. section 3.1 in the article //
Mat& calc_bpm(Mat& I);

// algorithm functions. section 3.1 in the article //
bool calc_bpTheta( Mat& I ,Mat& out_bpTheta ) ;

// algorithm functions. section 3.1 in the article //
Mat& calc_bp_total(Mat& bpm, Mat& bpTheta);

// algorithm functions. section 3.1 in the article //
// calculates votes for horizontal and vertical directions //
Mat& calc_votes_1(Mat& bp, Mat& out1 );

// algorithm functions. section 3.1 in the article //
// calculates votes for diagonal right and left directions //
Mat& calc_votes_2(Mat& bp, Mat& out3) ;

// algorithm functions. section 3.1 in the article //
// result is inside outside maps
Mat& calc_total_8_votes(Mat& out1, Mat& out2, Mat& out3, Mat& out4, Mat& totalVotes);


// algorithm functions. section 3.1 in the article //
void calc_motion_boundaries(const Mat &flow);


// preperation for the algorithm input. //
// calls the segmentation function of SLIC //
void slic_for_frame(IplImage *image  , Slic &slic)   ;


// algorithm functions. section 3.2 in the article //
// calcualte spatial and temporal functions //
///void calc_pairwisePotentials();
void calc_pairwisePotentials(Slic *segmented_slic,Slic *prev_segmented_slic, Mat &flow, long long superPixels_accumulated,
								double *pairWise_Weight);

int process_video_segmentation_algorithm(int, char**, bool vid_from_file);

#endif