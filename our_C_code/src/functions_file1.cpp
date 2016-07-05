#include "functions_file1.hpp"

void help_fback()
{
	cout <<
		"\ncontents: \n implementation of ---dense optical flow--- algorithm by Gunnar Farneback\n" <<
		"\n and SuperPixels by SLIC method, customed for OpenCV framework\n" <<
		"\n + video segmantation algorithem according to selected article algorithm\n "
		<< endl;
}

// add some vectors accodring to scale factor , to the original image //
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
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

// algorithm functions. section 3.1 in the article //
Mat& calc_bpm(Mat& I)  //I is magnitude //float32
{
	float	der_mag ,	b_m_p,		lambda_m = 0.7; // can be about 0.7 to 1.5, or else..

													// accept only char type matrices
	CV_Assert(I.depth() != sizeof(float));

	int channels	= I.channels();
	int nRows		= I.rows;
	int nCols		= I.cols * channels;  //=I.stride;
										  //int nCols = sizeof(I.type());
										  //int nDims = I.dims;
	int i,j;
	float* p;
	for( i = 0; i < nRows; ++i)
	{
		p = I.ptr<float>(i);
		for ( j = 0; j < nCols ; ++j )
		{		
			der_mag		= abs(p[j]);
			p[j]		= 1 - exp(-lambda_m * der_mag);	
		}
	}
	//p is the new b_m_p matrix. running over the original I matrix
	return I;
}

// algorithm functions. section 3.1 in the article //
bool calc_bpTheta( Mat& I ,Mat& out_bpTheta )  //I is angle //float32
{
	float 	ang_diff,		ang_nom,			delta_ang_max,		temp,
		b_theta_p,			lambda_t = 1;
	int		mask = 10;

	out_bpTheta = I.clone();

	// accept only char type matrices
	CV_Assert(I.depth() != sizeof(float));

	int channels	= I.channels();
	int nRows		= I.rows;
	int nCols		= I.cols * channels;  //=I.stride;
										  //int nCols = sizeof(I.type());
										  //int nDims = I.dims;
	int i,j;
	float* p;
	float* p_1,*p1;
	float* pT;
	////p = I.ptr<float>(0.0);
	for( i = 1; i < nRows-1; ++i)
	{
		p = I.ptr<float>(i);
		p_1 = I.ptr<float>(i-1);
		p1  = I.ptr<float>(i+1);
		pT = out_bpTheta.ptr<float>(i);
		for ( j = 1; j < nCols-1 ; ++j )
		{		
			b_theta_p = 0;
			if ( (j > mask) && (i > mask) && (j < nCols - mask) && (i < nRows - mask) )
			{
				// find the biggest angle difference for the b_theta_p
				delta_ang_max	= 0;
				ang_nom			= p[j];
				//// remember that j,i+1 smaller then size of matrix cols,rows  . just keep mask>1 always
				//for (int ang1 = -1; ang1 < 1; ang1++)
				//{
				//	for (int ang2 = -1; ang2 < 1; ang2++)
				//	{
				//		if (ang1 != 0 || ang2 != 0)
				//		{
				//			ang = p[i + ang1 , j + ang2 ];      /// TODO: verify correctness of this!
				//			temp = pow(abs(ang - ang_nom), 2);
				//			if (delta_ang_max < temp)
				//				delta_ang_max = temp;
				//		}
				//	}
				//}

				// check the side elements
				ang_diff = ((p[j-1]-p[j])*(p[j-1]-p[j])) ; 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				ang_diff = ((p[j+1]-p[j])*(p[j+1]-p[j])) ; 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				// check the above elements
				ang_diff = ((p_1[j-1]-p[j])*(p_1[j-1]-p[j])) ; 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				ang_diff = ((p_1[j]  -p[j])*(p_1[j]  -p[j])); 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				ang_diff = ((p_1[j+1]-p[j])*(p_1[j+1]-p[j])) ; 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				// check the buttom elements
				ang_diff = ((p1[j-1]-p[j])*(p1[j-1]-p[j])) ; 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				ang_diff = ((p1[j]  -p[j])*(p1[j]  -p[j])); 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				ang_diff = ((p1[j+1]-p[j])*(p1[j+1]-p[j])) ; 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;

				b_theta_p = 1 - exp(-lambda_t * delta_ang_max);

			}
			pT[j] =  b_theta_p;
		}
	}
	//p is the new b_m_p matrix. running over the original I matrix
	return true;
}

// algorithm functions. section 3.1 in the article //
Mat& calc_bp_total(Mat& bpm, Mat& bpTheta)  //I is magnitude //float32
{
	float	T_threshold_top	= 0.023;//0.025;  //0.08
	float	T_threshold_low	= 0.007;
	float	T2_threshold	= 0.077;//0.20; // 0.05

									// accept only char type matrices
	CV_Assert(bpm.depth() != sizeof(float));

	int channels	= bpm.channels();
	int nRows		= bpm.rows;
	int nCols		= bpm.cols * channels;  //=bpm.stride;
											//int nCols = sizeof(bpm.type());
											//int nDims = bpm.dims;
	int i,j;
	float* p;
	float* pT;
	for( i = 0; i < nRows; ++i)
	{
		p  = bpm.ptr<float>(i);
		pT = bpTheta.ptr<float>(i);
		for ( j = 0; j < nCols ; ++j )
		{		
			if ( (p[j] <= T_threshold_top) && (p[j] > T_threshold_low) )
			{
				p[j] *= pT[j];
			}	
			p[j] = p[j] > T2_threshold ? 1.0 : 0.0 ;
		}
	}
	//p is the new b_p matrix. running over the original bpm matrix
	// this is equition 3 in article, and figure 1f.
	return bpm;
}

// algorithm functions. section 3.1 in the article //
// calculates votes for horizontal and vertical directions //
Mat& calc_votes_1(Mat& bp, Mat& out1 )
{								  // accept only char type matrices
	CV_Assert(bp.depth() != sizeof(float));

	out1 = Mat::zeros(bp.size(),bp.type());

	int channels	= bp.channels();//
	int nRows		= bp.rows;
	int nCols		= bp.cols * channels; 

	int i,j,
		cuts,
		odd_or_one;

	float* p;
	float* pO;

	for( i = 0; i < nRows; ++i)
	{
		cuts	= 0;
		odd_or_one=0;
		p		= bp.ptr<float>(i);
		pO		= out1.ptr<float>(i);
		for ( j = 0+1 ; j < nCols ; ++j )   // consider treat the mask value
		{		
			if ( (p[j] > 0.5) && (p[j-1] <= 0.5) )  // pass a boundery
			{
				cuts ++;
				odd_or_one= odd_or_one==1 ? 0: 1; // not 1-odd_or_one to prevent num.error problem
			}
			pO[j] = odd_or_one;//cuts;			
		}
	}
	//p is the new b_p matrix. running over the original bpm matrix
	// this is calcultion of the horizontal votes
	return out1;
}

// algorithm functions. section 3.1 in the article //
// calculates votes for diagonal right and left directions //
Mat& calc_votes_2(Mat& bp, Mat& out3)  // out_i is Si
{								  // accept only char type matrices?
	CV_Assert(bp.depth() != sizeof(float));

	out3 = Mat::zeros(bp.size(),bp.type());

	if (!bp.isContinuous())
	{
		// http://docs.opencv.org/3.1.0/d3/d63/classcv_1_1Mat.html#aff83775c7fc1479de5f4a8c4e67fe361 
		cout << " indeces design ERROR - no continues stoarge... (in 'calc_votes_2') "; //because of matrix/pointer indexing
		return out3;  
	}

	int channels	= bp.channels();//
	int nRows		= bp.rows;
	int nCols		= bp.cols * channels; 

	int i,j,
		k,w,
		cuts ,
		odd_or_one;

	float *p;		// pointer to the current line
	float *pl_1;//, *pl1; // pointers for the previous and next lines
	float *pO;

	//p		= bp.ptr<float>(0);		// pointer to initial row. 1st element.
	//pO		= out3.ptr<float>(0);	// -"-

	// first section - go over all columns from left to right of the matrix.
	for( j = 2; j < nCols; ++j)
	{
		cuts	= 0;	
		odd_or_one=0;
		k=j;
		for ( i = 1 ; i < nRows ; ++i )    // 0(Zero) line is the assumed initial boundary condition, as background
		{		
			pl_1	= bp.ptr<float>(i-1);
			p		= bp.ptr<float>(i);
			pO		= out3.ptr<float>(i);   //TODO: test this section fro performance issues.
			k=k-1;
			if (k==0) {
				pO[k] = 0;
				break;
			}
			if ( (p[k] > 0.5) && (pl_1[k+1] <= 0.5) )  // pass a boundery
			{
				cuts ++;
				odd_or_one= odd_or_one==1 ? 0: 1; // not 1-odd_or_one to prevent num.error problem
			}
			pO[k] = odd_or_one;//cuts;			
		}
	}

	// second part - go over the lines , with the statring point in the right-most column
	j = nCols - 1;
	for (w=1; w<nRows-1; ++w) 
	{		
		cuts	= 0;	
		odd_or_one=0;	
		k=j;
		for ( i = w ; i < nRows ; ++i )   
		{		
			pl_1	= bp.ptr<float>(i-1);
			p		= bp.ptr<float>(i);
			pO		= out3.ptr<float>(i);   //TODO: test this section fro performance issues.
			k=k-1;
			if (k==0) {
				pO[k] = 0;
				break;
			}
			if ( (p[k] > 0.5) && (pl_1[k+1] <= 0.5) )  // pass a boundery
			{
				cuts ++;
				odd_or_one= odd_or_one==1 ? 0: 1;
			}
			pO[k] =  odd_or_one;//cuts;				
		}
	}

	//p is the new b_p matrix. running over the original bpm matrix
	// this is calcultion of the diagonal votes
	return out3;
}

// algorithm functions. section 3.1 in the article //
// result is inside outside maps
Mat& calc_total_8_votes(Mat& out1, Mat& out2, Mat& out3, Mat& out4, Mat& totalVotes)  // out_i is Si. all matrices should be in same sizes (assumed. not verified)
{								 
	totalVotes = Mat::zeros(out1.size(), out1.type());

	int channels	= out1.channels();//
	int nRows		= out1.rows;
	int nCols		= out1.cols * channels; 

	int i,j,
		votes ,
		odd_or_one;

	float *p1,*p2,*p3,*p4;	// i-th line pointers
	float *tmp21,*tmp22,*tmp31,*tmp32,*tmp41,*tmp42;	// dynamic indexed line pointers
	float *pO;

	// row and column 0 cannot be 1-s , so starting from indeces 1 (not zero)
	for( i = 1; i < nRows; ++i)
	{
		p1		= out1.ptr<float>(i);
		p2		= out2.ptr<float>(i);
		p3		= out3.ptr<float>(i);
		p4		= out4.ptr<float>(i);
		pO		= totalVotes.ptr<float>(i);  

		tmp21	= out2.ptr<float>(i-1);
		tmp22	= out2.ptr<float>(nRows-1);

		tmp31	= out3.ptr<float>(i-1);

		tmp41	= out4.ptr<float>(i-1);

		for ( j = 1 ; j < nCols ; ++j )   // TODO: fix later for the last column . now finishes 1 colun before that
		{
			// calc votes per pixel. minimum for total vote is 5 of 8
			// odd number of crossings is meaning the pixel is inside a moving body
			votes	= 0;	

			//horizontal in direction 1
			if (p1[j-1]==1) votes++;
			//horizontal in direction 2
			if ((p1[nCols-1]-p1[j])!=0) votes++;

			//vetical in direction 1
			if (tmp21[j]==1) votes++;
			//vertical in direction 2
			if ((tmp22[j]-p2[j])!=0) votes++;

			//diagonal 1st in direction 1
			if (tmp31[j+1]==1) votes++;
			//diagonal 1st in direction 2
			if (j >= nRows - i)						// the farther part of the matrix
			{
				tmp32	= out3.ptr<float>(nRows-1);
				if ((tmp32[j-(nRows - i)]-p3[j])!=0) votes++;
			}
			else                                    // the closer part of the matrix columns. take the 'transpose' element as the end element
			{
				tmp32	= out3.ptr<float>(j+i);
				if ((tmp32[0]-p3[j])!=0) votes++;
			}

			//diagonal 2st in direction 1
			if (tmp41[j-1]==1) votes++;
			////diagonal 2st in direction 2
			//if (j <= nRows - i)						// the farther part of the matrix
			//{
			//	tmp42	= out4.ptr<float>(nRows-1);
			//	if ((tmp42[j+( nRows - i)]-p4[j])!=0) votes++;
			//}
			//else                                    // the closer part of the matrix columns. take the 'transpose' element as the end element
			//{
			//	tmp42	= out4.ptr<float>(nCols-j-1);
			//	if ((tmp42[nCols-1]-p4[j])!=0) votes++;
			//}


			if (votes>4)
				pO[j] = 1; // else is 0(zero) by default
		}
	}



	//p is the new b_p matrix. running over the original bpm matrix
	// this is calcultion of the diagonal votes
	return totalVotes;
}

// algorithm functions. section 3.1 in the article //
void calc_motion_boundaries(const Mat &flow, Mat &totalVotes){
	//extraxt x and y channels
	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	//calculate angle and magnitude
	cv::Mat magnitude, angle;
	cv::Mat magnitude_grad, angle_grad;
	cv::cartToPolar(xy[0], xy[1], magnitude, angle, false);
	//// TODO: put these plots under 'debug_level_flags' to allow imidiaate plot enable/disable

	if (App_Parameters.flags.allow_debug_plots_1)
	{
		imshow("magnitude"	,magnitude);
		imshow("angle"		,angle);
	}

	Laplacian( magnitude, magnitude_grad, CV_32F);
	Laplacian( angle	, angle_grad	, CV_32F);

	double mag_max,			angle_max;
	double mag_grad_max,	angle_grad_max;

	
	////////// section only for display intermidiate motion bouderies //////////
	//translate magnitude to range [0;1]
	cv::minMaxLoc(magnitude, 0, &mag_max);
	magnitude.convertTo(magnitude, -1, 1.0 / mag_max);

	//translate magnitude to range [0;1]
	cv::minMaxLoc(magnitude_grad, 0, &mag_grad_max);
	magnitude_grad.convertTo(magnitude_grad, -1, 1.0 / mag_grad_max);

	cv::minMaxLoc(angle, 0, &angle_max);
	angle.convertTo(angle, -1, 1.0 / angle_max);

	//translate magnitude to range [0;1]
	cv::minMaxLoc(angle_grad, 0, &angle_grad_max);
	angle_grad.convertTo(angle_grad, -1, 1.0 / angle_grad_max);

	////normalized plots
	if (App_Parameters.flags.allow_debug_plots_2)
	{
		imshow("magnitude normalized"		,magnitude); 
		imshow("angle normalized"			,angle);
		imshow("magnitude grad normalized"	,magnitude_grad); 
		imshow("angle grad normalized"		,angle_grad);

		////build hsv image
		Mat _hsv[3], hsv;
		_hsv[0] = angle_grad;
		_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
		_hsv[2] = magnitude_grad;
		merge(_hsv, 3, hsv);

		////convert to BGR and show
		Mat bgr;	//CV_32FC3 matrix
		cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
		imshow("optical flow _grad trial internet", bgr);
	}
	//////////// end of section only for display intermidiate motion bouderies //////////
	

	Mat tmp,		tmp2,
		bp,
		out1,		out2,		out3,		out4;

	magnitude_grad	=	calc_bpm(magnitude_grad);
	imshow("new mag grad as B_P_M ",magnitude_grad);

	calc_bpTheta(angle, tmp);  
	imshow("new angle as B_P_TH",tmp);

	bp =   calc_bp_total(magnitude_grad, tmp);

	{
		minMaxLoc(bp, 0, &mag_grad_max);
		bp.convertTo(bp, -1, 1.0 / mag_grad_max);
	}

	imshow(" B_P",bp);

	////medianBlur	(bp,	bp,	3);		//9
	//erode			(bp,	bp,	Mat());
	//dilate		(bp,	bp,	Mat());

	//dilate		(bp,	bp,	Mat());
	//medianBlur	(bp,	bp,	5);//9
	//erode		(bp,	bp,	Mat()); ///

	//imshow("new B_P",bp);

	/* pass to function of 'calc_S_matrices' */
	// calculate votes for several directions :
	out1 = calc_votes_1(bp,out1);			//horizontal DIRECTION
											//imshow("new vote1",out1);

	transpose(bp, tmp); // original bp is now transposed into 'tmp'
	out2 = calc_votes_1(tmp,out2);			//vertical DIRECTION
	transpose(out2, out2);	// transpose back the outcome
							//imshow("new vote2",out2);

	out3 = calc_votes_2(bp, out3);	// calc diagonal crosses 
									//imshow("new vote3",out3);

	float *p;
	float *pO;
	tmp2 = bp.clone();
	////set tmp2 as pb flipped in columns
	//for ( int i = 0 ; i < bp.rows ; ++i )
	//{
	//	p	= bp.ptr<float>(i);
	//	pO	= tmp2.ptr<float>(i);
	//	for ( int j = 0 ; j < bp.cols ; ++j )
	//	{		
	//		pO[j]=p[bp.cols-1-j];		
	//	}
	//}
	flip(bp,tmp2,1);
	///imshow("tmp bp flippedd",tmp2);
	tmp2 = calc_votes_2(tmp2, out4);	// calc diagonal crosses 
										//out4 = out4.clone();
										///imshow("tmp2 output",tmp2);
										//// set the output flipped again										
										//for ( int i = 0 ; i < tmp2.rows ; ++i )
										//{
										//	p	= tmp2.ptr<float>(i);
										//	pO	= out4.ptr<float>(i);
										//	for ( int j = 0 ; j < tmp2.cols ; ++j )
										//	{		
										//		pO[j]=p[tmp2.cols-1-j];		
										//	}
										//}
	flip(tmp2,out4,1);
	//imshow("new vote4",out4);

	totalVotes= calc_total_8_votes(out1,out2,out3,out4,totalVotes);
	imshow("total votes",totalVotes);


}


// preperation for the algorithm input. //
// calls the segmentation function of SLIC //
void slic_for_frame(IplImage *image  , Slic &slic, long startingOffset)   
{
	int nr_superpixels	= 200;//1500;	// current values are what used in the article// 400;
	int nc				= 30;	// current values are what used in the article // 40;

								/* convert BGR to Lab colour space. */
	IplImage *lab_image = cvCloneImage(image);
	cvCvtColor(image, lab_image, CV_BGR2Lab);

	/* Yield the number of superpixels and weight-factors from the user. */
	int		w		= image->width, 
		h		= image->height;
	double	step	= sqrt((w * h) / (double) nr_superpixels) ;


	/* Perform the SLIC superpixel algorithm. */
	slic.generate_superpixels(lab_image, step, nc,  startingOffset); // 3 1st variables are the original version

																	 // this one is nice only for debug mode..
	slic.display_contours	(image, CV_RGB(255,0,0));	// final output of this: puts the resultant binary map 'istaken' on the 'image'
	slic.display_center_grid(image, CV_RGB(0,250,0));	// final output of this: puts the resultant binary map 'istaken' on the 'image'
	cvShowImage("slic segmentation", image);

}
