//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/videoio/videoio.hpp"
//#include "opencv2/highgui/highgui.hpp"
//TODO: change this file name from 'fback' to 'DOF'

#include <iostream>

using namespace cv;
using namespace std;

#include "app_globals.h"

#include "writeMat.cpp"

static void help_fback()
{
    cout <<
            "\nThis section implements ---dense optical flow--- algorithm by Gunnar Farneback\n"
             << endl;
}
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double vecFactor, const Scalar& color)
{
	float Mag2=0;

    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
			Mag2 = vecFactor*fxy.x*fxy.x*vecFactor + vecFactor*fxy.y*fxy.y*vecFactor ;              // magnitude squared
			if (Mag2>1550){
				line(cflowmap, Point(x,y), Point(cvRound(x+ vecFactor*fxy.x), cvRound(y+ vecFactor*fxy.y)),
					 color);
				circle(cflowmap, Point(x,y), 2, color, -1);
			}
		}
}

void getFlowGrad(const Mat& mag_grad, const Mat& angle,
					float vecSize)
{
	double der_mag = 0, ang, ang_nom, delta_ang,temp;
	double b_m_p, b_theta_p, lambda_m = 0.7, lambda_t = 1;
	int mask = 10;
	Point2f fxy;
	//Mat cflowgrad;
	UMat cflowgrad = UMat::zeros(mag_grad.size(), CV_32F);

	//flow.copyTo(cflowgrad);

	//Vec3b white(255,255,255), black(0, 0, 0);
	uchar white(255), black(0);

	for (int y = 0; y < mag_grad.rows ; y++)
		for (int x = 0; x < mag_grad.cols ; x++)
		{
			//fxy = mag_grad.at<Point2f>(y, x);
			//
			////der_mag = norm(Mat(fxy));
			//der_mag = sqrtf( fxy.x*fxy.x + fxy.y*fxy.y);	//change .at to LUT or regular array indeces. for performance.
			//ang_nom = atan2(fxy.y, fxy.x);
			ang_nom=0;

			b_m_p = 1 - exp(-lambda_m*der_mag);
			//cflowgrad.at<Vec3b>(Point(x, y)) = black;
			cflowgrad.col(x).row(y) = black;
			if (x > mask && y > mask && x<mag_grad.cols - mask && y < mag_grad.rows - mask)
			{
				// find the biggest angle difference for the b_theta_p
				delta_ang = 0;
				for (int ang1 = -1; ang1 < 1; ang1++)
				{
					for (int ang2 = -1; ang2 < 1; ang2++)
					{
						if (ang1 != 0 || ang2 != 0)
						{
							fxy = angle.at<Point2f>(y + ang1, x + ang2);
							ang = atan2(fxy.y, fxy.x);
							temp = pow(abs(ang - ang_nom), 2);
							if (delta_ang < temp)
								delta_ang = temp;
						}
					}
				}
				b_theta_p = 1 - exp(-lambda_t*delta_ang);
				
				if (b_m_p > vecSize) //T-threshold
				{
					cflowgrad.col(x).row(y) = b_m_p * white;
					if (b_theta_p > 0.25)
					{
						cflowgrad.col(x).row(y) = b_m_p * b_theta_p * white;
					}
				}
				/*else
				{				
						cflowgrad.at<Vec3b>(Point(x, y)) = 0.1 * white;
					}
				}*/
			}
			
		}

	imshow("flow2", cflowgrad);
}

void imshow2channel(Mat& input_matrix)
{
	cv::Mat xy[2]; //X,Y
	cv::split(input_matrix, xy);

	//calculate angle and magnitude
	cv::Mat magnitude, angle;
	cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

	//translate magnitude to range [0;1]
	double mag_max;
	cv::minMaxLoc(magnitude, 0, &mag_max);
	magnitude.convertTo(magnitude, -1, 1.0 / mag_max);

	//build hsv image
	cv::Mat _hsv[3], hsv;
	_hsv[0] = angle;
	_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
	_hsv[2] = magnitude;
	cv::merge(_hsv, 3, hsv);

	//convert to BGR and show
	cv::Mat bgr;//CV_32FC3 matrix
	cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	cv::imshow("sobel optical flow trial internet", bgr);
}

void showFlowSobel4gradients(const Mat &flow){
	//http://stackoverflow.com/questions/8507629/is-there-a-quick-and-easy-way-in-opencv-to-compute-the-gradient-of-an-image
	//http://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=sobel
	//http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/sobel_derivatives/sobel_derivatives.html

	//http://www.swarthmore.edu/NatSci/mzucker1/opencv-2.4.10-docs/doc/tutorials/imgproc/imgtrans/sobel_derivatives/sobel_derivatives.html

	//extraxt x and y channels
	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	//calculate angle and magnitude
	cv::Mat magnitude, angle;
	cv::Mat magnitude_grad, angle_grad;
	cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);


	//Sobel( magnitude, magnitude_grad, CV_32F, 0, 1, 3/*, scale, delta, BORDER_DEFAULT*/ );
	Laplacian( magnitude, magnitude_grad, CV_32F);

	//Sobel( angle, angle_grad, CV_32F, 0,1, 3/*, scale, delta, BORDER_DEFAULT*/ );
	Laplacian( angle, angle_grad, CV_32F);


	////////// section only for display intermidiate motion bouderies //////////
	//translate magnitude to range [0;1]
	double mag_max;
	cv::minMaxLoc(magnitude, 0, &mag_max);
	magnitude.convertTo(magnitude, -1, 1.0 / mag_max);

	//translate magnitude to range [0;1]
	double mag_grad_max;
	cv::minMaxLoc(magnitude_grad, 0, &mag_grad_max);
	magnitude_grad.convertTo(magnitude_grad, -1, 1.0 / mag_grad_max);

	//build hsv image
	cv::Mat _hsv[3], hsv;
	_hsv[0] = angle_grad;
	_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
	_hsv[2] = magnitude_grad;
	cv::merge(_hsv, 3, hsv);

	//convert to BGR and show
	cv::Mat bgr;//CV_32FC3 matrix
	cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
	cv::imshow("optical flow _grad trial internet", bgr);
	////////// end of section only for display intermidiate motion bouderies //////////

	getFlowGrad(magnitude_grad,angle, 0.5 )	;
}

int do_DOF(int, char**, bool vid_from_file)
{
	string	base_out_file_path	= "../work_files/optical_flow_as_Mat/";
	string	framesCounterStr	= ""	, base_file_name = "" , file_full_name="", file_suffix = ".mat";	//base_file_name outMAt_
	int		stream_frame_index	= 0;
	//Size	newSize(225, 300);			//(_Tp _width, _Tp _height);
	//Size	newSize(300, 225);			//(_Tp _width, _Tp _height);
	//Size	newSize(400, 300);			//(_Tp _width, _Tp _height);
	Size	newSize(400, 225);			//(_Tp _width, _Tp _height);

	VideoCapture cap;
	if (!vid_from_file)
	    cap = VideoCapture(0);
	else
	{		
		//	char			rec_file_name[150] = "C:\\Users\\Ran_the_User\\Documents\\GitHub\\AirBorneCamera_A\\Selected article\\FastVideoSegment_Files\\Data\\inputs\\mySample\\2_movement1_several_cars.00.avi";
		//  char			rec_file_name[150] = "C:\\Users\\Ran_the_User\\Documents\\GitHub\\AirBorneCamera_A\\Selected article\\FastVideoSegment_Files\\Data\\inputs\\mySample\\MOVI0024.avi";
		char			rec_file_name[150] = "../work_files/cars.avi";
		cap					= VideoCapture(rec_file_name);
	}

    help_fback();
    if( !cap.isOpened() )
        return -1;

    Mat flow, cflow, frame, cflow2;
    UMat gray, prevgray, uflow;
	double t ; //for timings
	double measure_times[100000]; // for keeping times. // this is a trial item
	int frame_counter = 0 ;

    namedWindow("flow", 1);
	namedWindow("flow2", 1);

    for(;;)
    {
        cap >> frame;
		if (frame.empty())
			break;
		// TODO: add test for empty frame. break if empty
		frame_counter++;
	///	resize(frame, frame, newSize , 0, 0, INTER_CUBIC); 

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        if( !prevgray.empty() )
        {
//			clock_t begin = clock();

			if (App_Parameters.flags.measure_actions_timing)
				t = (double)getTickCount();

			/* calc optical flow between two frames */
            calcOpticalFlowFarneback(prevgray, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0); // 'uflow' is the DOF matrix result

			if (App_Parameters.flags.measure_actions_timing){
				t								= 1000*((double)getTickCount() - t)/getTickFrequency();
				measure_times[frame_counter]	= t;
			}
//			clock_t end = clock();
//			double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//			printf("**MyProgram::after time= %d\n", elapsed_secs);

			/* manipulate the frame for user display. into 'cflow' matrix */
            cvtColor(prevgray, cflow, COLOR_GRAY2BGR);	// previous step video frame -> 'cflow'
			// TODO: add frame counter on top of image to display. in corner
            uflow.copyTo(flow);
			cflow.copyTo(cflow2);
            drawOptFlowMap(flow, cflow, 10/*16*/, 15, Scalar(0, 255, 0)); // every 16 pixels flow is displayed. 
		/*	getFlowGrad(flow, cflow2, 0.85);*/
			showFlowSobel4gradients(flow);
            imshow("flow", cflow);
		//	imshow("flow2", cflow2);

			if (App_Parameters.flags.export_frames_to_Mat) {
				//  TODO: 
				//  add saving the mat to im file
				// add saving those two into accumulating aray file. for multi frame recording.

				file_full_name = base_out_file_path + base_file_name + std::to_string(++stream_frame_index) + file_suffix;

				const char * c = file_full_name.c_str();
				writeMat(flow, c, "DOFframe", true, 0); //get returned byte . send number of images to be saved
			}
        }

        if(waitKey(1.0)>=0)
            break;
        std::swap(prevgray, gray);
    }
	
	//sum(measure_times);
	double sum=0, avg_time=0;
	for (int i=2; i<frame_counter; i++)//index 0,1 are not populated in the time array
		sum	+=	measure_times[i];
	avg_time	=	sum / frame_counter; 

	cout << "Time of optical flow in "	<< frame_counter << " frames: " << sum << " milliseconds."<< endl;
	cout << "average Time is: " << avg_time << " milliseconds."<< endl;
	//TODO: add printout in ~performance graph.

    return 0;
}
