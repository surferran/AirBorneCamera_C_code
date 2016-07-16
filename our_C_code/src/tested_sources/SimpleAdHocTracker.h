/*
 * SimpleAdHocTracker.h
 *
 *  Created on: Mar 15, 2015
 *      Author: roy_shilkrot
 *
 *  The MIT License (MIT)
 *
 *  Copyright (c) 2015 Roy Shilkrot
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE. *
 */

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <vector>

using namespace cv;
using namespace std;


class SimpleAdHocTracker {
    Ptr<FeatureDetector>    detector;
    bool                    bootstrapping;
    vector<KeyPoint>        bootstrap_kp;
    vector<KeyPoint>        trackedFeatures;
    vector<Point3d>         trackedFeatures3D;
    Mat                     prevGray;
    Mat                     camMat;
    bool                    canCalcMVM;
    Mat                     raux,taux;
    Mat                     cvToGl;
    Mat_<double>            modelview_matrix;
    Size                    frameSize;

public:
    SimpleAdHocTracker(const Ptr<FeatureDetector>&, const Mat& cam);
    void bootstrap(const Mat&);
    void bootstrapTrack(const Mat&);
    void track(const Mat&);
    void process(const Mat&, bool newmap = false);
    bool canCalcModelViewMatrix() const;
    void calcModelViewMatrix(Mat_<double>& modelview_matrix);
    bool triangulateAndCheckReproj(const Mat& P, const Mat& P1);
    bool cameraPoseAndTriangulationFromFundamental(Mat_<double>& P, Mat_<double>& P1);
    bool DecomposeEtoRandT(Mat_<double>& E, Mat_<double>& R1, Mat_<double>& R2, Mat_<double>& t1, Mat_<double>& t2);

    const vector<KeyPoint>& getTrackedFeatures() const {
        return trackedFeatures;
    }
    const vector<Point3d>& getTracked3DFeatures() const {
        return trackedFeatures3D;
    }
};