#ifndef COMPOSED_ALGORITHM_HPP
#define COMPOSED_ALGORITHM_HPP 

#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;  
using namespace std;

#include "app_globals.h"   
#include "ourConverted\calcSpatialConnections.hpp"
#include "ourConverted\calcTemporalConnections.hpp"
#include "ourConverted\calcSuperpixelInRatio.hpp"
#include "functions_file1.hpp"

//#include <ctime>				// ref by : http://stackoverflow.com/questions/2808398/easily-measure-elapsed-time 

// algorithm functions. section 3.1 in the article //
void calc_motion_boundaries(const Mat &flow);

// algorithm functions. section 3.2 in the article //
// calcualte spatial and temporal functions //

void calc_pairwisePotentials(Slic *segmented_slic , Slic *prev_segmented_slic , Mat &flow , 
								long /*long*/ superPixels_accumulated,	double *pairWise_Weight); 

void calc_unary_potentials(Slic *segmented_slic , Mat &frame_Votes);


#endif