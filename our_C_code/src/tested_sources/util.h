/*
 * util.h
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
vector<cv::Point> Pointsi(const vector<cv::Point_<T> >& points);
template<typename T,typename V>
vector<Point_<V> > Points(const vector<Point_<T> >& points);
template<typename T,typename V>
vector<Point3_<V> > Points(const vector<Point3_<T> >& points);
template<typename T>
vector<Point_<T> > Points(const vector<KeyPoint>& keypoints);
void drawBoundingBox(Mat& image, const vector<Point2f>& bb, const Scalar& color = Scalar(0,0,255));
template<typename T, typename V>
void keepVectorsByStatus(vector<T>& f1, vector<V>& f2, const vector<uchar>& status);
template<typename T, typename V, typename K>
void keepVectorsByStatus(vector<T>& f1, vector<V>& f2, vector<K>& f3, const vector<uchar>& status);
template<typename T>
vector<KeyPoint> KeyPoints(const vector<Point_<T> >& points);
