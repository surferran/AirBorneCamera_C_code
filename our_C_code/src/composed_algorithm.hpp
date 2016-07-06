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
#include "functions_file1.hpp"

///
#include "some_utils\writeMat.hpp"   //TODO: check how to include this and operate the function
//#include <ctime>				// ref by : http://stackoverflow.com/questions/2808398/easily-measure-elapsed-time 

// algorithm functions. section 3.1 in the article //
void calc_motion_boundaries(const Mat &flow);

// algorithm functions. section 3.2 in the article //
// calcualte spatial and temporal functions //
void calc_pairwisePotentials(Slic *segmented_slic , Slic *prev_segmented_slic , Mat &flow , long long superPixels_accumulated,
								double *pairWise_Weight);

void calc_unary_potentials(Slic *segmented_slic , Mat &frame_Votes);


void copy_binaricMat_to_ucharArray(int w, int h,  Mat & bp ,  unsigned char * out_array);

void copy_ucharArray_to_binaricMat(int w, int h, unsigned char * in_array , Mat & bp  );

#endif