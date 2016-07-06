//// airborn camera system to detect movements in the recorded video
//  last update : 06/07/2016
// Usage :
//	change file names in code

#include "composed_algorithm.hpp"
#include "some_utils\writeMat.hpp" 

Storage_4_frames			alg_database_current;
vector<Storage_4_frames>	alg_DB_vec;

int main(int argc, char** argv)
{
	/* user operational flags must be adjusted hard coded in */
	bool vid_from_file	=  App_Parameters.flags.read_from_file;
	int  vid_resize_W	=  App_Parameters.flags.frame_resize_W;
	int  vid_resize_H	=  App_Parameters.flags.frame_resize_H; 
	int  resize_factor	=  App_Parameters.flags.resize_factor; 

	string	base_out_file_path	= "../work_files/optical_flow_as_Mat/";
	string	framesCounterStr	= ""	, base_file_name = "" , file_full_name="", file_suffix = ".mat";	//base_file_name outMAt_
	int		stream_frame_index	= 0;

	Size	newSize(vid_resize_W / resize_factor, vid_resize_H / resize_factor);

	VideoCapture cap; 

	if (!vid_from_file)
		cap = VideoCapture(0);
	else
	{	
		//		char			rec_file_name[150] = "../work_files/matlab_Aid/square01.avi";
		//			char			rec_file_name[150] = "../work_files/matlab_Aid/triangle.avi";
		//char			rec_file_name[150] = "../work_files/car2.mov";
		//char			rec_file_name[150] = "../work_files/car2.mov";
		//	
		char			rec_file_name[150] = "../work_files/matlab_Aid/square001.avi";

		//			char			rec_file_name[150] = "../work_files/matlab_Aid/circle.avi";

		cap					= VideoCapture(rec_file_name);
	}
	if( !cap.isOpened() )
		return -1;

	help_app();
	
	Mat		frame, prevframe , dummy_MatFrame;
	UMat	gray,  prevgray, uflow;
	Mat		flow,  cflow,    cflow2;
	Mat		frame_votes;
	double	t ;						//for timings
	double	measure_times[100000]; // for keeping times. // this is a trial item									

	int					frame_counter = 0 ;
	long long			superPixels_accumulated =0; // accumulated through frames.
	long				current_frame_sPixelsNum = 0;
	long				startingOffset	=0;
	vector<long>		sPixels_bounds;
	vector< vec2dd >	frames_sPixels_centers;

	IplImage	IpImage ;   // for passing to SLIC function
	Slic		SlicOutput , prevSlicOutput;

	namedWindow("flow", 1); 

	for(;;)
	{
		cap >> frame;
		if (frame.empty())
			break;

		frame_counter++;
		
		/* current frame processes */
		if (App_Parameters.flags.do_frame_resize)
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

			// calculate the pairWise potentials weights for the 2 sub-sequent frames.
			// return the relevant part for the major E() function.
			double pairWise_Weight	=	0;
			calc_pairwisePotentials( &SlicOutput ,&prevSlicOutput, flow, superPixels_accumulated, &pairWise_Weight); 
			// TODO: in order to use those for more than 2 frames at a time - one must keep all X frames in a global struct
			//			and pass them to that main function. 
			//			need also to prepare-back the inside-used functions for dealing with sevevral frames.
			//			mainly for the temporal option.

				//Slic *segmented_slic
			calc_unary_potentials(  &SlicOutput ,  frame_votes );

			/* optional section according to user parameter input */
			if (App_Parameters.flags.export_frames_to_Mat) {
				//  TODO: 
				//  add saving the mat to im file
				//  add saving those two into accumulating aray file. for multi frame recording.
				// check https://msdn.microsoft.com/query/dev14.query?appId=Dev14IDEF1&l=EN-US&k=k(LNK2019);k(vs.output)&rd=true
				/***************** save the current Optical flow matrix ******************/
				base_file_name = "flow_";
				file_full_name = base_out_file_path + base_file_name + std::to_string(++stream_frame_index) + file_suffix;

				const char * cF = file_full_name.c_str();
				const char * st = "DOFframe";

				//	
				writeMat(flow, cF, st); //get returned byte . send number of images to be saved

				/****************** save the current votes matrix ******************/
				base_file_name = "votes_";
				file_full_name = base_out_file_path + base_file_name + std::to_string(  stream_frame_index) + file_suffix;

				const char * cV = file_full_name.c_str();
				
				//
				writeMat(frame_votes, cV, "inMaps"); //get returned byte . send number of images to be saved

				/* save also the current SLIC matrix ?*/
			}

		}

		if(waitKey(100)>=0) //1
			break;

		/* store the current frame data */	
			alg_database_current.SPixels	= SlicOutput;
		if( !prevgray.empty() ) {
			alg_database_current.DOF		= flow.clone();	
			alg_database_current.Votes		= frame_votes.clone();
		}
		alg_DB_vec						.push_back(alg_database_current);

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
