#include "functions_file1.hpp"

void help_app()
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
void calc_bpm(Mat& flow_grad_mag, Mat& result)   
{
	float	der_mag ,	b_m_p;
	double 	lambda_m = App_Parameters.algs_params.lambda_magnitude; 							

	int channels	= flow_grad_mag.channels();
	int nRows		= flow_grad_mag.rows;
	int nCols		= flow_grad_mag.cols * channels; 

	int i,j;
	float* p;
	float* pOut;
	for( i = 0; i < nRows; ++i)
	{
		p		= flow_grad_mag	.ptr<float>(i);
		pOut	= result		.ptr<float>(i);
		for ( j = 0; j < nCols ; ++j )
		{		
			der_mag		= abs(p[j]);
			pOut[j]		= 1 - exp(-lambda_m * der_mag);	
		}
	} 
}

// algorithm functions. section 3.1 in the article //
void calc_bpTheta( Mat& alpha_input,Mat& flowX ,Mat& flowY ,Mat& out_bpTheta )  
{
	double	lambda_t	=	App_Parameters.algs_params.lamda_theta;
	float 	ang_diff,		delta_ang_max,		temp,
			b_theta_p;
	///int		mask = 2;	//10

	out_bpTheta		= Mat::zeros(alpha_input.size(),alpha_input.type());

	int channels	= alpha_input.channels();
	int nRows		= alpha_input.rows;
	int nCols		= alpha_input.cols * channels;		

	int		i,j;
	float* pIn;
	float* p_1,*p1;
	float* pOut;

	float* pFlowX;
	float* pFlowY;

	for( i = 1; i < nRows-1; ++i)
	{
		pIn	= alpha_input.ptr<float>(i);		// pointer to current line of alpha_input matrix
		p_1 = alpha_input.ptr<float>(i-1);		// previous line
		p1  = alpha_input.ptr<float>(i+1);		// next line
		pOut= out_bpTheta.ptr<float>(i);		// pointer to current element in output matrix
		pFlowX = flowX.ptr<float>(i);
		pFlowY = flowY.ptr<float>(i);
		for ( j = 1; j < nCols-1 ; ++j )
		{		
			b_theta_p = 0;
			/* make difference between angles value 0 because angle or because no O.flow */
			if ((pFlowX[j]==0) && (pFlowY[j]==0)) 
				pIn[j] = 99;
			///if ( (j > mask) && (i > mask) && (j < nCols - mask) && (i < nRows - mask) )
			{
				// find the biggest angle difference for the b_theta_p
				delta_ang_max	= 0;
				//ang_nom			= pIn[j];	// element alpha(i,j)
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

				/// check diffs only on Horizontal and Vertical directions. no need for the diagonals !
				// check the side elements
				ang_diff = ((pIn[j-1]-pIn[j])*(pIn[j-1]-pIn[j])) ; 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				ang_diff = ((pIn[j+1]-pIn[j])*(pIn[j+1]-pIn[j])) ; 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ;
				// check the above elements 
				ang_diff = ((p_1[j]  -pIn[j])*(p_1[j]  -pIn[j])); 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ; 
				// check the buttom elements 
				ang_diff = ((p1[j]  -pIn[j])*(p1[j]  -pIn[j])); 	if (ang_diff>delta_ang_max) delta_ang_max = ang_diff ; 

				b_theta_p = 1 - exp(-lambda_t * delta_ang_max);

			}
			pOut[j] =  b_theta_p;
		}
	} 
}

// algorithm functions. section 3.1 in the article //
void calc_bp_total(Mat& bpm, Mat& bpTheta, Mat& bp_Out)   
{	
	float	T_threshold_low		= App_Parameters.algs_params.b_p_m_low_level;
	float	T_threshold_top		= App_Parameters.algs_params.b_p_m_high_level;
	float	minimizing_factor	= App_Parameters.algs_params.BP_minimizing_factor;
	//float	T2_threshold	= App_Parameters.algs_params.T2_threshold;
	float	split_level			= App_Parameters.algs_params.mid_level ;

	int channels	= bpm.channels();
	int nRows		= bpm.rows;
	int nCols		= bpm.cols * channels;  

	int i,j;
	float* pM;
	float* pT;
	float* pOut;
	for( i = 0; i < nRows; ++i)
	{
		pM		= bpm		.ptr<float>(i);
		pT		= bpTheta	.ptr<float>(i);
		pOut	= bp_Out	.ptr<float>(i);
		
		for ( j = 0; j < nCols ; ++j )
		{		
			if (pM[j] <= T_threshold_low)
				pOut[j] = pM[j] *minimizing_factor;			// according to article code. it doen't written in the paper

			if ( (pM[j] <= T_threshold_top) && (pM[j] > T_threshold_low) )			
				pOut[j] = pM[j]*pT[j];	

			if (pM[j] > T_threshold_top)
				pOut[j] = pM[j] ;

			if (pOut[j] > split_level)
				pOut[j] = 1 ;
			else
				pOut[j] = 0 ;
		}
	} 	 
}

// algorithm functions. section 3.1 in the article //
// calculates votes for horizontal and vertical directions //
void calc_votes_1(Mat& bp, Mat& out1 )
{					
	float	split_level		= App_Parameters.algs_params.mid_level;
	out1 = Mat::zeros(bp.size(),bp.type());

	int channels	= bp.channels();//
	int nRows		= bp.rows;
	int nCols		= bp.cols * channels; 

	int i,j,
		cuts,
		odd_or_one;

	float* p;
	float* pO;

	/* normal progress direction is from left to right of the matrix */
	for( i = 0; i < nRows; ++i)
	{
		cuts	= 0;
		odd_or_one=0;
		p		= bp.ptr<float>(i);
		pO		= out1.ptr<float>(i);
		for ( j = 0+1 ; j < nCols ; ++j )   // consider treat the mask value
		{		
			if ( (p[j] > split_level) && (p[j-1] <= split_level) )  // pass a boundery
			{
				cuts ++;
				odd_or_one= odd_or_one==1 ? 0: 1; // not 1-odd_or_one to prevent num.error problem
			}
			pO[j] = odd_or_one;//cuts;			
		}
	} 
}

// algorithm functions. section 3.1 in the article //
// calculates votes for diagonal right and left directions //
void calc_votes_2(Mat& bp, Mat& out3)   
{								
	float	split_level		= App_Parameters.algs_params.mid_level;
	out3 = Mat::zeros(bp.size(),bp.type());

	if (!bp.isContinuous())
	{ 
		cout << " indeces design ERROR - no continues stoarge... (in 'calc_votes_2') "; //because of matrix/pointer indexing
	}

	int channels	= bp.channels();//
	int nRows		= bp.rows;
	int nCols		= bp.cols * channels; 

	int i,j,
		k,w,
		cuts ,
		odd_or_one;

	float *p;		// pointer to the current line
	float *p_1;//, *pl1; // pointers for the previous and next lines
	float *pO;

	// first section - go over all columns from left to right of the matrix.
	for( j = 1; j < nCols; ++j)//2
	{
		cuts	= 0;	
		odd_or_one=0;
		k=j;
		for ( i = 1 ; i < nRows ; ++i )    // 0(Zero) line is the assumed initial boundary condition, as background
		{		
			p_1		= bp.ptr<float>(i-1);
			p		= bp.ptr<float>(i);
			pO		= out3.ptr<float>(i);   //TODO: test this section for performance issues.
			k=k-1;
			if (k==0) {
				pO[k] = 0;
				break;
			}
			if ( (p[k] > split_level) && (p_1[k+1] <= split_level) )  // pass a boundery
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
			p_1	= bp.ptr<float>(i-1);
			p		= bp.ptr<float>(i);
			pO		= out3.ptr<float>(i);   //TODO: test this section fro performance issues.
			k=k-1;
			if (k==0) {
				pO[k] = 0;
				break;
			}
			if ( (p[k] > split_level) && (p_1[k+1] <= split_level) )  // pass a boundery
			{
				cuts ++;
				odd_or_one= odd_or_one==1 ? 0: 1;
			}
			pO[k] =  odd_or_one;//cuts;				
		}
	}
}

// algorithm functions. section 3.1 in the article //
// result is inside outside maps
void calc_total_8_votes(Mat& out1, Mat& out2, Mat& out3, Mat& out4, Mat& totalVotes)  // out_i is Si. all matrices should be in same sizes (assumed. not verified)
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
			///	if ((tmp32[0]-p3[j])!=0) votes++;
			}

			//diagonal 2nd in direction 1
			if (tmp41[j-1]==1) votes++;
			//diagonal 2nd in direction 2
			if (j <= nRows - i)						// the farther part of the matrix
			{
				tmp42	= out4.ptr<float>(nRows-1);
			///	if ((tmp42[j+( nRows - i)]-p4[j])!=0) votes++;
			}
			else                                    // the closer part of the matrix columns. take the 'transpose' element as the end element
			{
				tmp42	= out4.ptr<float>(nCols-j-1);
			///	if ((tmp42[nCols-1]-p4[j])!=0) votes++;
			}


			if (votes>4)
				pO[j] = 1; // else is 0(zero) by default
		}
	}

}

// algorithm functions. section 3.1 in the article //
void calc_motion_boundaries(const Mat &flow, Mat &totalVotes){
	//extraxt x and y channels
	cv::Mat xy[2];	
	cv::split(flow, xy);	//X,Y

	/* imshow cannot plot 2channel Mat. 
		we are seperating channels also for calculations */
	/* calculate the gradient of the flow field */
	cv::Mat flow_grad_X;
	cv::Mat flow_grad_Y;
		// laplacian  is 2nd derivative..!
	//Laplacian( xy[0], flow_grad_X, CV_32F);	//consider using Sobel only
	//Laplacian( xy[1], flow_grad_Y, CV_32F);

	Mat src, src_gray;
	Mat grad;

	int ksize = 1;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_32F;//CV_16S;//

	// sobel gives 1st derivatives

	/*CV_EXPORTS_W void Sobel( InputArray src, OutputArray dst, int ddepth,
		int dx, int dy, int ksize = 3,
		double scale = 1, double delta = 0,
		int borderType = BORDER_DEFAULT );*/
	Sobel(xy[0],flow_grad_X,ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT ); // Scharr - without the '3' param
	Sobel(xy[1],flow_grad_Y,ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT ); //

	///cv::split(flow, xy);	//X,Y

	if (App_Parameters.flags.allow_debug_plots_1)
	{
		imshow("flow_grad_X"	,flow_grad_X);
		imshow("flow_grad_Y"	,flow_grad_Y);
	}

	//calculate angle and magnitude
	cv::Mat flow_grad_magnitude, angle;

	phase(xy[0], xy[1], angle);					// angle in radians
	magnitude(flow_grad_X, flow_grad_Y, flow_grad_magnitude);

	if (App_Parameters.flags.allow_debug_plots_1)
	{
		imshow("flow_grad_magnitude"	,flow_grad_magnitude);
		imshow("angle"					,angle);
	}
	
	Mat tmp,	tmp2,
		bpm,	bptheta,	bp,
		S1,		S2,			S3,		S4;

	bpm		= Mat::zeros(angle.size(), angle.type());
	bptheta = Mat::zeros(angle.size(), angle.type());
	bp		= Mat::zeros(angle.size(), angle.type());

	/* ****** gradient magnitude to BpM ****** */
	calc_bpm(flow_grad_magnitude, bpm);		// calculate equation (1) in the article
	imshow("B_P_M as flow_grad_magnitude",bpm);

	/* ****** angle theta BpTheta ****** */	
	calc_bpTheta(angle, xy[0], xy[1] , bptheta);				// calculate equation (2) in the article
	imshow("B_P_TH as angle function",bptheta);

	/* getting a binaric Bp matrix */ 
	calc_bp_total(bpm, bptheta, bp);
	imshow("B_P total",bp);

	/////////////////////////////////////////
	Size	sz		= bp.size();
	int		h		= sz.height;
	int		w		= sz.width;
	long	vec_size = h*w;
	unsigned char *flattened_bp				= new unsigned char [vec_size];
	unsigned char *flattened_total_votes	= new unsigned char [vec_size];
	totalVotes								= Mat::zeros(bp.size(), bp.type());

	copy_binaricMat_to_ucharArray( w,  h,  bp , flattened_bp) ;
	
	calcintegralIntersections(h,w , flattened_bp, flattened_total_votes );
	
	copy_ucharArray_to_binaricMat(w,h, flattened_total_votes, totalVotes) ;
	imshow("total votes REF",totalVotes);

	delete flattened_bp;
	delete flattened_total_votes;
	//////////////////////////////////////////

	/************* part of 'calc_S_matrices' ******************/
	// calculate votes for several directions :
	calc_votes_1(bp,S1);			//horizontal DIRECTION
											//imshow("new vote1",out1);

	transpose(bp, tmp); // original bp is now transposed into 'tmp'
	calc_votes_1(tmp,S2);			//vertical DIRECTION
	transpose(S2, S2);	// transpose back the outcome
							//imshow("new vote2",out2);

	calc_votes_2(bp, S3);	// calc diagonal crosses 
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
	calc_votes_2(tmp2, S4);	// calc diagonal crosses 
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
	flip(tmp2,S4,1);
	//imshow("new vote4",out4);

	calc_total_8_votes(S1,S2,S3,S4,totalVotes);
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
