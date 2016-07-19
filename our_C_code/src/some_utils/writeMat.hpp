#ifndef WRITE_MAT_HPP
#define WRITE_MAT_HPP

/* Author: Philip G. Lee <rocketman768@gmail.com>
http://code.opencv.org/issues/1342
* I, Philip G. Lee, hereby disclaim any Copyright that I might hold
 * by default.
 */
 
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/*!
 *  \author Philip G. Lee <rocketman768@gmail.com>
 *  Write \b mat into \b filename
 *  in uncompressed .mat format (Level 5 MATLAB) for Matlab.
 *  The variable name in matlab will be \b varName. If
 *  \b bgr2rgb is true and there are 3 channels, swaps 1st and 3rd
 *  channels in the output. This is needed because OpenCV matrices
 *  are bgr, while Matlab is rgb. This has been tested to work with
 *  3-channel single-precision floating point matrices, and I hope
 *  it works on other types/channels, but not exactly sure.
 *  Documentation at <http://www.mathworks.com/help/pdf_doc/matlab/matfile_format.pdf>
 */ 
 
//#define _CRT_SECURE_NO_WARNINGS  // in order to disable:  error C4996: 'fopen': This function or variable may be unsafe. Consider using fopen_s instead. To disable deprecation, use _CRT_SECURE_NO_WARNINGS. See online help for details.
//#pragma warning(disable : 4996)

// added writeMode 0 for create, 1 for append
void writeMat( cv::Mat const& mat, const char* filename, const char* varName = "A", bool bgr2rgb = true , int writeMode = 0);

#endif