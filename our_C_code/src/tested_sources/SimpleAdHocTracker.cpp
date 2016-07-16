/*
 * SimpleAdHocTracker.cpp
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
 
 #include "SimpleAdHocTracker.h"
 #include "util.h"

SimpleAdHocTracker::SimpleAdHocTracker(const Ptr<FeatureDetector>& d, const Mat& cam):
detector(d),bootstrapping(false),canCalcMVM(false)
{
    cam.convertTo(camMat,CV_64F);

    cvToGl = Mat::zeros(4, 4, CV_64F);
    cvToGl.at<double>(0, 0) = 1.0f;
    cvToGl.at<double>(1, 1) = -1.0f; // Invert the y axis
    cvToGl.at<double>(2, 2) = -1.0f; // invert the z axis
    cvToGl.at<double>(3, 3) = 1.0f;

}

void SimpleAdHocTracker::bootstrap(const cv::Mat& img) {
    //Detect first features in the image (clear any current tracks)
    assert(!img.empty() && img.channels() == 3);

    bootstrap_kp.clear();
    detector->detect(img,bootstrap_kp);

    trackedFeatures = bootstrap_kp;

    cvtColor(img, prevGray, CV_BGR2GRAY);

    bootstrapping = true;
}

bool SimpleAdHocTracker::DecomposeEtoRandT(
                                           Mat_<double>& E,
                                           Mat_<double>& R1,
                                           Mat_<double>& R2,
                                           Mat_<double>& t1,
                                           Mat_<double>& t2)
{
    //Using HZ E decomposition

    SVD svd(E,SVD::MODIFY_A);

    //check if first and second singular values are the same (as they should be)
    double singular_values_ratio = fabsf(svd.w.at<double>(0) / svd.w.at<double>(1));
    if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
    if (singular_values_ratio < 0.7) {
        cerr << "singular values of essential matrix are too far apart\n";
        return false;
    }

    Matx33d W(0,-1,0,   //HZ 9.13
              1,0,0,
              0,0,1);
    Matx33d Wt(0,1,0,
               -1,0,0,
               0,0,1);
    R1 = svd.u * Mat(W) * svd.vt; //HZ 9.19
    R2 = svd.u * Mat(Wt) * svd.vt; //HZ 9.19
    t1 = svd.u.col(2); //u3
    t2 = -svd.u.col(2); //u3
    return true;
}


bool SimpleAdHocTracker::triangulateAndCheckReproj(const Mat& P, const Mat& P1) {
    //undistort
    Mat normalizedTrackedPts,normalizedBootstrapPts;
    undistortPoints(Points<float>(trackedFeatures), normalizedTrackedPts, camMat, Mat());
    undistortPoints(Points<float>(bootstrap_kp), normalizedBootstrapPts, camMat, Mat());

    //triangulate
    Mat pt_3d_h(4,trackedFeatures.size(),CV_32FC1);
    cv::triangulatePoints(P,P1,normalizedBootstrapPts,normalizedTrackedPts,pt_3d_h);
    Mat pt_3d; convertPointsFromHomogeneous(Mat(pt_3d_h.t()).reshape(4, 1),pt_3d);
    //    cout << pt_3d.size() << endl;
    //    cout << pt_3d.rowRange(0,10) << endl;

    vector<uchar> status(pt_3d.rows,0);
    for (int i=0; i<pt_3d.rows; i++) {
        status[i] = (pt_3d.at<Point3f>(i).z > 0) ? 1 : 0;
    }
    int count = countNonZero(status);

    double percentage = ((double)count / (double)pt_3d.rows);
    cout << count << "/" << pt_3d.rows << " = " << percentage*100.0 << "% are in front of camera";
    if(percentage < 0.75)
        return false; //less than 75% of the points are in front of the camera


    //calculate reprojection
    cv::Mat_<double> R = P(cv::Rect(0,0,3,3));
    Vec3d rvec(0,0,0); //Rodrigues(R ,rvec);
    Vec3d tvec(0,0,0); // = P.col(3);
    vector<Point2f> reprojected_pt_set1;
    projectPoints(pt_3d,rvec,tvec,camMat,Mat(),reprojected_pt_set1);
//    cout << Mat(reprojected_pt_set1).rowRange(0,10) << endl;
    vector<Point2f> bootstrapPts_v = Points<float>(bootstrap_kp);
    Mat bootstrapPts = Mat(bootstrapPts_v);
//    cout << bootstrapPts.rowRange(0,10) << endl;

    double reprojErr = cv::norm(Mat(reprojected_pt_set1),bootstrapPts,NORM_L2)/(double)bootstrapPts_v.size();
    cout << "reprojection Error " << reprojErr;
    if(reprojErr < 5) {
        vector<uchar> status(bootstrapPts_v.size(),0);
        for (int i = 0;  i < bootstrapPts_v.size(); ++ i) {
            status[i] = (norm(bootstrapPts_v[i]-reprojected_pt_set1[i]) < 20.0);
        }

        trackedFeatures3D.clear();
        trackedFeatures3D.resize(pt_3d.rows);
        pt_3d.copyTo(Mat(trackedFeatures3D));

        keepVectorsByStatus(trackedFeatures,trackedFeatures3D,status);
        cout << "keeping " << trackedFeatures.size() << " nicely reprojected points";
        bootstrapping = false;
        return true;
    }
    return false;
}

bool SimpleAdHocTracker::cameraPoseAndTriangulationFromFundamental(Mat_<double>& P, Mat_<double>& P1) {
    //find fundamental matrix
    double minVal,maxVal;
    vector<Point2f> trackedFeaturesPts = Points<float>(trackedFeatures);
    vector<Point2f> bootstrapPts = Points<float>(bootstrap_kp);
    cv::minMaxIdx(trackedFeaturesPts,&minVal,&maxVal);
    vector<uchar> status;
    Mat F = findFundamentalMat(trackedFeaturesPts, bootstrapPts, FM_RANSAC, 0.006 * maxVal, 0.99, status);
    int inliers_num = countNonZero(status);
    cout << "Fundamental keeping " << inliers_num << " / " << status.size();
    keepVectorsByStatus(trackedFeatures,bootstrap_kp,status);

    if(inliers_num > min_inliers) {
        //Essential matrix: compute then extract cameras [R|t]
        Mat_<double> E = camMat.t() * F * camMat; //according to HZ (9.12)

        //according to http://en.wikipedia.org/wiki/Essential_matrix#Properties_of_the_essential_matrix
        if(fabsf(determinant(E)) > 1e-07) {
            cout << "det(E) != 0 : " << determinant(E);
            return false;
        }

        Mat_<double> R1(3,3);
        Mat_<double> R2(3,3);
        Mat_<double> t1(1,3);
        Mat_<double> t2(1,3);
        if (!DecomposeEtoRandT(E,R1,R2,t1,t2)) return false;

        if(determinant(R1)+1.0 < 1e-09) {
            //according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
            cout << "det(R) == -1 ["<<determinant(R1)<<"]: flip E's sign";
            E = -E;
            if (!DecomposeEtoRandT(E,R1,R2,t1,t2)) return false;
        }
        if(fabsf(determinant(R1))-1.0 > 1e-07) {
            cerr << "det(R) != +-1.0, this is not a rotation matrix";
            return false;
        }

        Mat P = Mat::eye(3,4,CV_64FC1);

        //TODO: there are 4 different combinations for P1...
        Mat_<double> P1 = (Mat_<double>(3,4) <<
                           R1(0,0),   R1(0,1),    R1(0,2),    t1(0),
                           R1(1,0),   R1(1,1),    R1(1,2),    t1(1),
                           R1(2,0),   R1(2,1),    R1(2,2),    t1(2));
        cout << "P1\n" << Mat(P1) << endl;

        bool triangulationSucceeded = true;
        if(!triangulateAndCheckReproj(P,P1)) {
            P1 = (Mat_<double>(3,4) <<
                  R1(0,0),   R1(0,1),    R1(0,2),    t2(0),
                  R1(1,0),   R1(1,1),    R1(1,2),    t2(1),
                  R1(2,0),   R1(2,1),    R1(2,2),    t2(2));
            cout << "P1\n" << Mat(P1) << endl;

            if(!triangulateAndCheckReproj(P,P1)) {
                Mat_<double> P1 = (Mat_<double>(3,4) <<
                                   R2(0,0),   R2(0,1),    R2(0,2),    t2(0),
                                   R2(1,0),   R2(1,1),    R2(1,2),    t2(1),
                                   R2(2,0),   R2(2,1),    R2(2,2),    t2(2));
                cout << "P1\n" << Mat(P1) << endl;

                if(!triangulateAndCheckReproj(P,P1)) {
                    Mat_<double> P1 = (Mat_<double>(3,4) <<
                                       R2(0,0),   R2(0,1),    R2(0,2),    t1(0),
                                       R2(1,0),   R2(1,1),    R2(1,2),    t1(1),
                                       R2(2,0),   R2(2,1),    R2(2,2),    t1(2));
                    cout << "P1\n" << Mat(P1) << endl;

                    if(!triangulateAndCheckReproj(P,P1)) {
                        cerr << "can't find the right P matrix\n";
                        triangulationSucceeded = false;
                    }
                }

            }

        }
        return triangulationSucceeded;
    }
    return false;
}

void SimpleAdHocTracker::bootstrapTrack(const Mat& img) {
    //Track detected features
    if(prevGray.empty()) { cerr << "can't track: empty prev frame"; return; }

    {
        vector<Point2f> corners;
        vector<uchar> status; vector<float> errors;
        Mat currGray; cvtColor(img, currGray, CV_BGR2GRAY);
        calcOpticalFlowPyrLK(prevGray,currGray,Points<float>(trackedFeatures),corners,status,errors,cv::Size(11,11));
        currGray.copyTo(prevGray);

        if(countNonZero(status) < status.size() * 0.8) {
            cerr << "tracking failed";
            bootstrapping = false;
            return;
        }

        trackedFeatures = KeyPoints(corners);
        keepVectorsByStatus(trackedFeatures,bootstrap_kp,status);
    }
    cout << trackedFeatures.size() << " features survived optical flow";
    assert(trackedFeatures.size() == bootstrap_kp.size());

    //verify features with a homography
    Mat inlier_mask, homography;
    if(trackedFeatures.size() >= 4) {
        homography = findHomography(Points<float>(trackedFeatures),
                                    Points<float>(bootstrap_kp),
                                    CV_RANSAC,
                                    ransac_thresh,
                                    inlier_mask);
    }

    int inliers_num = countNonZero(inlier_mask);
    cout << inliers_num << " features survived homography";
    if(inliers_num != trackedFeatures.size() && inliers_num >= 4 && !homography.empty()) {
        keepVectorsByStatus(trackedFeatures,bootstrap_kp,inlier_mask);
    } else if(inliers_num < min_inliers) {
        cout << "not enough features survived homography.";
        bootstrapping = false;
        return;
    }

    vector<KeyPoint> bootstrap_kp_orig = bootstrap_kp;
    vector<KeyPoint> trackedFeatures_orig = trackedFeatures;

    //Attempt at 3D reconstruction (triangulation) if conditions are right
    Mat rigidT = estimateRigidTransform(Points<float>(trackedFeatures),Points<float>(bootstrap_kp),false);
    if(norm(rigidT.col(2)) > 100) {
        //camera motion is sufficient

        Mat_<double> P,P1;
        bool triangulationSucceeded = cameraPoseAndTriangulationFromFundamental(P,P1);

        if(triangulationSucceeded) {
            //triangulation succeeded, test for coplanarity
            Mat trackedFeatures3DM(trackedFeatures3D);
            trackedFeatures3DM = trackedFeatures3DM.reshape(1,trackedFeatures3D.size());

            //PCA will determine if most of the points are on plane
            cv::PCA pca(trackedFeatures3DM,Mat(),CV_PCA_DATA_AS_ROW);

            int num_inliers = 0;
            cv::Vec3d normalOfPlane = pca.eigenvectors.row(2);
            normalOfPlane = cv::normalize(normalOfPlane);
            cv::Vec3d x0 = pca.mean;
            double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));

            vector<uchar> status(trackedFeatures3D.size(),0);
            for (int i=0; i<trackedFeatures3D.size(); i++) {
                Vec3d w = Vec3d(trackedFeatures3D[i]) - x0;
                double D = fabs(normalOfPlane.dot(w));
                if(D < p_to_plane_thresh) {
                    num_inliers++;
                    status[i] = 1;
                }
            }
            cout << num_inliers << "/" << trackedFeatures3D.size() << " are coplanar";
            bootstrapping = ((double)num_inliers / (double)(trackedFeatures3D.size())) < 0.75;
            if(!bootstrapping) {
                //enough features are coplanar, keep them and flatten them on the XY plane
                keepVectorsByStatus(trackedFeatures3D,trackedFeatures,status);

                //the PCA has the major axes of the plane
                Mat projected = pca.project(trackedFeatures3DM);
                projected.col(2).setTo(0);
                projected.copyTo(trackedFeatures3DM);

            } else {
                cerr << "not enough features are coplanar";
                bootstrap_kp = bootstrap_kp_orig;
                trackedFeatures = trackedFeatures_orig;
            }
        }
    }

    //Setup for another iteration or handover the new map to the tracker.
}

void SimpleAdHocTracker::track(const cv::Mat &img) {
    //Track detected features
    if(prevGray.empty()) { cerr << "can't track: empty prev frame"; return; }

    {
        vector<Point2f> corners;
        vector<uchar> status; vector<float> errors;
        Mat currGray; cvtColor(img, currGray, CV_BGR2GRAY);
        calcOpticalFlowPyrLK(prevGray,currGray,Points<float>(trackedFeatures),corners,status,errors,cv::Size(11,11));
        currGray.copyTo(prevGray);

        if(countNonZero(status) < status.size() * 0.8) {
            cerr << "tracking failed";
            bootstrapping = false;
            canCalcMVM = false;
            return;
        }

        trackedFeatures = KeyPoints(corners);
        keepVectorsByStatus(trackedFeatures,trackedFeatures3D,status);
    }

    canCalcMVM = (trackedFeatures.size() >= min_inliers);

    if(canCalcMVM) {
        //Perform camera pose estimation for AR
        cv::Mat Rvec,Tvec;
        cv::solvePnP(Points<double,float>(trackedFeatures3D), Points<float>(trackedFeatures), camMat, Mat(), raux, taux, !raux.empty());
        raux.convertTo(Rvec,CV_32F);
        taux.convertTo(Tvec ,CV_64F);

        Mat Rot(3,3,CV_32FC1);
        Rodrigues(Rvec, Rot);

        // [R | t] matrix
        Mat_<double> para = Mat_<double>::eye(4,4);
        Rot.convertTo(para(cv::Rect(0,0,3,3)),CV_64F);
        Tvec.copyTo(para(cv::Rect(3,0,1,3)));
        para = cvToGl * para;

        //        cout << para << endl;

        Mat(para.t()).copyTo(modelview_matrix); // transpose to col-major for OpenGL
    }
}

void SimpleAdHocTracker::process(const Mat& img, bool newmap) {
    if(newmap) {
        cout << "bootstrapping\n";
        bootstrap(img);
        bootstrapping = true;
    } else if(bootstrapping) {
        cout << "bootstrap tracking ("<< trackedFeatures.size() << ")\n";
        bootstrapTrack(img);
    } else if(!newmap && !bootstrapping) {
        track(img);
    }
}

bool SimpleAdHocTracker::canCalcModelViewMatrix() const {
    return canCalcMVM;
}
void SimpleAdHocTracker::calcModelViewMatrix(Mat_<double>& mvm) {
    modelview_matrix.copyTo(mvm);
}