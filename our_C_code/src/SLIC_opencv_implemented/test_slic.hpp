/*
 * test_slic.cpp.
 *
 * Written by: Pascal Mettes.
 *
 * This file creates an over-segmentation of a provided image based on the SLIC
 * superpixel algorithm, as implemented in slic.h and slic.cpp.
 
 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>
using namespace std;*/

//// using the *curtesy* of https://github.com/PSMM/SLIC-Superpixels
// .. like to improve with this one : https://github.com/ClaudiuGeorgiu/VideoSLIC  !! sounds good intro
//       the paper : https://www.google.co.il/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&uact=8&ved=0ahUKEwiK0p-FpPzMAhVLWxoKHVxCCJAQFggdMAA&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel7%2F97%2F6917086%2F06920066.pdf%3Farnumber%3D6920066&usg=AFQjCNFkEWougFit0WGPU4_yjLatlYqg0w&sig2=upxeCFyaaz15HU83CMohzg&bvm=bv.123325700,d.ZGg


#ifndef SLIC_CPP_H
#define SLIC_CPP_H
//
#include "slic.h"

int run_test_slic(/*int argc, char *argv[]*/) {
    /* Load the image and convert to Lab colour space. */
    IplImage *image = cvLoadImage("dog.png", 1);
    IplImage *lab_image = cvCloneImage(image);
    cvCvtColor(image, lab_image, CV_BGR2Lab);
    
    /* Yield the number of superpixels and weight-factors from the user. */
    int w = image->width, h = image->height;
    int nr_superpixels = 400;//atoi(argv[2]);
    int nc = 40;//atoi(argv[3]);

    double step = sqrt((w * h) / (double) nr_superpixels);
    
    /* Perform the SLIC superpixel algorithm. */
    Slic slic;
    slic.generate_superpixels(lab_image, step, nc);
    slic.create_connectivity(lab_image);
    
    /* Display the contours and show the result. */
    slic.display_contours(image, CV_RGB(255,0,0));
    cvShowImage("result", image);
    cvWaitKey(0);
    cvSaveImage("myoutput.png", image);

	return 0;
}

#endif