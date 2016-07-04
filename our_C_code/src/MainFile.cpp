//// airborn camera system to detect movements in the recorded video
//  last update : 21/06/16
// Usage :
//	change file names in code
//		base_out_file_path, rec_file_name

// desired functions:
//   set_Mask()  - ignore, in every aspect, from pixels in this. borders, or constant broadcast texts and signs.
//					alarm (in yellow) about suspected pixels as not related to source video.

#include "composed_algorithm.hpp"
#include "functions_file1.cpp"


int main(int argc, char** argv)
{
	//	run_test_slic(); // for debug only. testing functionallity through intermidiate util.

	///process_video_segmentation_algorithm(1,argv, App_Parameters.flags.read_from_file);

	// TODO: try add constant grid for image windows.
	// http://code.opencv.org/projects/opencv/wiki/DisplayManyImages
	// http://stackoverflow.com/questions/5089927/show-multiple-2-3-4-images-in-the-same-window-in-opencv

	bool vid_from_file =  App_Parameters.flags.read_from_file;

	string	base_out_file_path	= "../work_files/optical_flow_as_Mat/";
	string	framesCounterStr	= ""	, base_file_name = "" , file_full_name="", file_suffix = ".mat";	//base_file_name outMAt_
	int		stream_frame_index	= 0;
	//Size	newSize(225, 300);			//(_Tp _width, _Tp _height);
	//Size	newSize(300, 225);			//(_Tp _width, _Tp _height);
	//Size	newSize(400, 300);			//(_Tp _width, _Tp _height);
	//Size	newSize(400, 225);			//(_Tp _width, _Tp _height);///
	//Size	newSize(320, 240);			//(_Tp _width, _Tp _height);
	Size	newSize(160, 120);			//(_Tp _width, _Tp _height);

	VideoCapture cap;
	vid_from_file = true; 
	bool READNG_FROM_WEB = false ;
	if (!vid_from_file)
		if (READNG_FROM_WEB)
			cap.open("http://192.168.1.102:8080/");
		else
			cap = VideoCapture(0);
	else
	{		
		char			rec_file_name[150] = "C:\\Users\\Ran_the_User\\Documents\\GitHub\\AirBorneCamera_A\\Selected article\\FastVideoSegment_Files\\Data\\inputs\\mySample\\2_movement1_several_cars.00.avi";
		//	  char			rec_file_name[150] = "C:\\Users\\Ran_the_User\\Documents\\GitHub\\AirBorneCamera_A\\Selected article\\FastVideoSegment_Files\\Data\\inputs\\mySample\\MOVI0024.avi";
		//char			rec_file_name[150] = "../work_files/cars.avi";
		//char			rec_file_name[150] = "../work_files/car1.MP4";
		//char			rec_file_name[150] = "../work_files/4.mov";
		//char			rec_file_name[150] = "../work_files/car2.mov";
		//char			rec_file_name[150] = "../work_files/dogs.mp4";
		//char			rec_file_name[150] = "../work_files/dogs.mp4";
		cap					= VideoCapture(rec_file_name);
	}

	help_fback();
	if( !cap.isOpened() )
		return -1;

	Mat		frame, prevframe , dummy_MatFrame;
	UMat	gray,  prevgray, uflow;
	Mat		flow,  cflow,    cflow2;
	Mat		frame_votes;
	double	t ; //for timings
	double	measure_times[100000]; // for keeping times. // this is a trial item

									////Storage_4_frames frames_Storage; 

	int					frame_counter = 0 ;
	long long			superPixels_accumulated =0; // accumulated through frames.
	long				current_frame_sPixelsNum = 0;
	long				startingOffset	=0;
	vector<long>		sPixels_bounds;
	vector< vec2dd >	frames_sPixels_centers;

	IplImage	IpImage ;   // for passing to SLIC function
	Slic		SlicOutput , prevSlicOutput;

	namedWindow("flow", 1);
	///namedWindow("flow2", 1);

	for(;;)
	{
		cap >> frame;
		if (frame.empty())
			break;

		frame_counter++;
		///	
		/* current frame processes */
		resize   (frame, frame, newSize , 0, 0, INTER_CUBIC); 
		cvtColor (frame, gray, COLOR_BGR2GRAY);

		//////////////////////////*  SLIC *///////////////////////////
		/* do the SLIC algo. on this frame */
		///// MAT conversion by ref http://answers.opencv.org/question/14088/converting-from-iplimage-to-cvmat-and-vise-versa/ 
		dummy_MatFrame	=  frame.clone();
		IpImage			=  dummy_MatFrame;			//	sets a pointer to the Mat. clone() directly doesn't work well.
		startingOffset	=	superPixels_accumulated;
		slic_for_frame(&IpImage, SlicOutput, startingOffset) ;

		/* --- do the super-pixels handling - actions inteended for alg. part 3.1 --- */
		/* to get makeSuperpixelIndexUnique , and getSuperpixelStats*/ // lines 103..107 in 'm' script
		current_frame_sPixelsNum	 = SlicOutput.return_num_of_superpixels();
		superPixels_accumulated		+= current_frame_sPixelsNum;
		sPixels_bounds			.push_back(superPixels_accumulated) ;	// if x labels then labels[0..x-1]. next is [x..y-1],.. (y=x1+x2+..)
																		//    for frame [0,1,..]
		frames_sPixels_centers	.push_back( SlicOutput.return_centers() );

		if( !prevgray.empty() )
		{ 
			//////////////////////////*  optical flow *///////////////////////////
			if (App_Parameters.flags.measure_actions_timing)
				t = (double)getTickCount();

			/* calc optical flow between two frames */
			calcOpticalFlowFarneback(prevgray, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0); // 'uflow' is the DOF matrix result

			if (App_Parameters.flags.measure_actions_timing){
				t								= 1000*((double)getTickCount() - t)/getTickFrequency();
				measure_times[frame_counter]	= t;
			}

			/* manipulate the frame for user display. into 'cflow' matrix */
			cvtColor(prevgray, cflow, COLOR_GRAY2BGR);	// previous step video frame -> 'cflow'
														// TODO: add frame counter on top of image to display. in corner
			uflow.copyTo(flow);			// copy UMAT to MAT
			cflow.copyTo(cflow2);
			drawOptFlowMap(flow, cflow, 10/*16*/, 15, Scalar(0, 255, 0)); // every 16 pixels flow is displayed. 
			imshow("flow", cflow);


			//////////////////////////*  algorithm section 3.1 *///////////////////////////

			/* calculate votes for this frame. by optical flow input */
			// returns motion boundaries as 'frame_votes'
			calc_motion_boundaries(flow, frame_votes);////////////////////////////

			//////////////////////////*  algorithm section 3.2 *///////////////////////////

			///			float			*inRatios						= new float		   [num_of_sPixels];
			//calc_inRatios();
			/* run pairwise potenrials */
			// use the current and previous frames. and flow result for those. and other inputs.

			// calculate the pairWise potentials wights for the 2 sub-sequent frames.
			// return the relevant part for the major E() function.
			double pairWise_Weight	=	0;
			calc_pairwisePotentials( &SlicOutput ,&prevSlicOutput, flow, superPixels_accumulated, &pairWise_Weight); 
			// TODO: in order to use those for more than 2 frames at a time - one must keep all X frames in a global struct
			//			and pass them to that main function. 
			//			need also to prepare-back the inside-used functions for dealing with sevevral frames.
			//			mainly for the temporal option.

			// 			calc_unary_potentials(frame_votes, );

			/* optional section according to user parameter input */
			if (App_Parameters.flags.export_frames_to_Mat) {
				//  TODO: 
				//  add saving the mat to im file
				// add saving those two into accumulating aray file. for multi frame recording.

				file_full_name = base_out_file_path + base_file_name + std::to_string(++stream_frame_index) + file_suffix;

				const char * c = file_full_name.c_str();
				//writeMat(flow, c, "DOFframe", true, 0); //get returned byte . send number of images to be saved
			}

		}

		if(waitKey(100)>=0) //1
			break;
		/// can do just copy. not swap
		std::swap(prevgray , gray);
		std::swap(prevframe, frame);
		prevSlicOutput = SlicOutput;
	}

	//sum(measure_times);
	double sum=0, avg_time=0;
	for (int i=2; i<frame_counter; i++)//index 0,1 are not populated in the time array
		sum	+=	measure_times[i];
	avg_time	=	sum / frame_counter; 

	cout << "Time of optical flow in "	<< frame_counter << " frames: " << sum << " milliseconds."<< endl;
	cout << "average Time is: " << avg_time << " milliseconds."<< endl;
	//TODO: add printout in ~performance graph.
	
	cvWaitKey(0);  // allow user to notify the final state of the display windows

	destroyAllWindows();	// despite the camera will be deinitialized AUTOmatically in VideoCapture destructor
							// using c:\OpenCV\build\include\opencv2\highgui.hpp
	return 0;
}
