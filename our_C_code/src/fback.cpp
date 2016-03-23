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
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);			
            line(cflowmap, Point(x,y), Point(cvRound(x+ vecFactor*fxy.x), cvRound(y+ vecFactor*fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
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

    Mat flow, cflow, frame;
    UMat gray, prevgray, uflow;
	double t ; //for timings
	double measure_times[100000]; // for keeping times. // this is a trial item
	int frame_counter = 0 ;

    namedWindow("flow", 1);

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
            drawOptFlowMap(flow, cflow, 10/*16*/, 15, Scalar(0, 255, 0)); // every 16 pixels flow is displayed. 

            imshow("flow", cflow);

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
