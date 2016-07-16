/*
 * util.cpp
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
 
template<typename T>
vector<cv::Point> Pointsi(const vector<cv::Point_<T> >& points) {
    vector<cv::Point> res;
    for(unsigned i = 0; i < points.size(); i++) {
        res.push_back(cv::Point(points[i].x,points[i].y));
    }
    return res;
}
template<typename T,typename V>
vector<Point_<V> > Points(const vector<Point_<T> >& points) {
    vector<Point_<V> > res;
    for(unsigned i = 0; i < points.size(); i++) {
        res.push_back(Point_<V>(points[i].x,points[i].y));
    }
    return res;
}
template<typename T,typename V>
vector<Point3_<V> > Points(const vector<Point3_<T> >& points) {
    vector<Point3_<V> > res;
    for(unsigned i = 0; i < points.size(); i++) {
        res.push_back(Point3_<V>(points[i].x,points[i].y,points[i].z));
    }
    return res;
}
template<typename T>
vector<Point_<T> > Points(const vector<KeyPoint>& keypoints)
{
    vector<Point_<T> > res;
    for(unsigned i = 0; i < keypoints.size(); i++) {
        res.push_back(Point_<T>(keypoints[i].pt.x,keypoints[i].pt.y));
    }
    return res;
}
void drawBoundingBox(Mat& image, const vector<Point2f>& bb, const Scalar& color = Scalar(0,0,255))
{
    for(unsigned i = 0; i < bb.size(); i++) {
        line(image, bb[i], bb[(i + 1) % bb.size()], color, 2);
    }
}
template<typename T, typename V>
void keepVectorsByStatus(vector<T>& f1, vector<V>& f2, const vector<uchar>& status) {
    vector<T> oldf1 = f1;
    vector<V> oldf2 = f2;
    f1.clear();
    f2.clear();
    for (int i = 0; i < status.size(); ++i) {
        if(status[i])
        {
            f1.push_back(oldf1[i]);
            f2.push_back(oldf2[i]);
        }
    }
}
template<typename T, typename V, typename K>
void keepVectorsByStatus(vector<T>& f1, vector<V>& f2, vector<K>& f3, const vector<uchar>& status) {
    vector<T> oldf1 = f1;
    vector<V> oldf2 = f2;
    vector<K> oldf3 = f3;
    f1.clear();
    f2.clear();
    f3.clear();
    for (int i = 0; i < status.size(); ++i) {
        if(status[i])
        {
            f1.push_back(oldf1[i]);
            f2.push_back(oldf2[i]);
            f3.push_back(oldf3[i]);
        }
    }
}
template<typename T>
vector<KeyPoint> KeyPoints(const vector<Point_<T> >& points) {
    vector<KeyPoint> res;
    for(unsigned i = 0; i < points.size(); i++) {
        res.push_back(KeyPoint(points[i],1,0,0));
    }
    return res;
}