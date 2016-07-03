#include "composed_algorithm.hpp"

static void help_fback()
{
    cout <<
		"\ncontents: \n implementation of ---dense optical flow--- algorithm by Gunnar Farneback\n" <<
		"\n and SuperPixels by SLIC method, customed for OpenCV framework\n" <<
		"\n + video segmantation algorithem according to selected article algorithm\n "
             << endl;
}

// add some vectors accodring to scale factor , to the original image //
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
			if ((p1[nCols]-p1[j])!=0) votes++;

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
void calc_motion_boundaries(const Mat &flow){
	//extraxt x and y channels
	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	//calculate angle and magnitude
	cv::Mat magnitude, angle;
	cv::Mat magnitude_grad, angle_grad;
	cv::cartToPolar(xy[0], xy[1], magnitude, angle, false);
	//// TODO: put these plots under 'debug_level_flags' to allow imidiaate plot enable/disable
	//imshow("magnitude",magnitude);
	//imshow("angle",angle);
	
	//Sobel( magnitude, magnitude_grad, CV_32F, 0, 1, 3/*, scale, delta, BORDER_DEFAULT*/ );
	Laplacian( magnitude, magnitude_grad, CV_32F);
	//Sobel( angle, angle_grad, CV_32F, 0,1, 3/*, scale, delta, BORDER_DEFAULT*/ );
	Laplacian( angle, angle_grad, CV_32F);
	
	double mag_max,			angle_max;
	double mag_grad_max,	angle_grad_max;
	
	{
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
		//imshow("magnitude normalized",magnitude); 
		//imshow("angle normalized",angle);
		//imshow("magnitude grad normalized",magnitude_grad); 
		//imshow("angle grad normalized",angle_grad);

		////build hsv image
		//cv::Mat _hsv[3], hsv;
		//_hsv[0] = angle_grad;
		//_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
		//_hsv[2] = magnitude_grad;
		//cv::merge(_hsv, 3, hsv);

		////convert to BGR and show
		//cv::Mat bgr;//CV_32FC3 matrix
		//cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
		//cv::imshow("optical flow _grad trial internet", bgr);
		//////////// end of section only for display intermidiate motion bouderies //////////
	}

	Mat tmp,
		tmp2,
		bp,
		out1,
		out2,
		out3,
		out4,
		totalVotes;

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

		////medianBlur	(bp,	bp,	3);//9
		//erode		(bp,	bp,	Mat());
		//dilate		(bp,	bp,	Mat());

		dilate		(bp,	bp,	Mat());
		medianBlur	(bp,	bp,	5);//9
		erode		(bp,	bp,	Mat()); ///

	imshow("new B_P",bp);

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

void copy_vectors_to_array(int w, int h,  Slic *slic_obj, unsigned int * out_array)
{
	// consider http://stackoverflow.com/questions/13237490/how-to-use-a-2d-vector-of-pointers

	int* tmp = NULL;// slic_obj->return_pointer_to_clusters_column();

	for (int j=0; j<w; j++)
	{	
		tmp = slic_obj->return_pointer_to_clusters_column(j);  /*vec2dd in_vec*/		
		for(int i=0; i<h; i++)
		{
			out_array[j*h + i] = tmp[i];  // 			point = j * height + i;
		}
	}
}

void copy_floatMat_to_shortArray(int w, int h,  Mat & flow , short * out_array)
{
	float* tmp = NULL;
	float* tmpX = NULL;
	float* tmpY = NULL;

	unsigned int elements = h* w;
/*
	const Point2f& fxy = flow.at<Point2f>(y, x);
	Mag2 = vecFactor*fxy.x*fxy.x*vecFactor + vecFactor*fxy.y*fxy.y*vecFactor ;       */

	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	///cv::cartToPolar(xy[0], xy[1], magnitude, angle, false);


	for (int i=0; i<h; i++)
	{	
		tmp =  flow.ptr<float>(i);
		tmpX = xy[0].ptr<float>(i);
		tmpY = xy[1].ptr<float>(i);
		for(int j=0; j<w; j++)
		{

			const Point2f& fxy = flow.at<Point2f>(i, j);
			//out_array[i+ j*h] = (short)tmp[j];  // this should be fine for a Mat.
												/// all lines are flatten in sequence
			out_array[i+ j*h]				= (short)tmpX[j];  // this should be fine for a Mat.
			out_array[i+ j*h + elements]	= (short)tmpY[j];  // this should be fine for a Mat.
		}
	}

	
}

void function_to_calc_weight_part_1(unsigned int	*sources, unsigned int	*targets, unsigned int vectors_length , vec2dd centeres ,
										vector<double> color_dist, vector<double> center_dist )
{
	//vector<double> spatial_color_dist;
	//vector<double> spatial_center_dist;
	//vector<double> temporal_color_dist;
	double sum=0;
	double betaSpatial = 0;

	for (int i=0; i< vectors_length; i++)
	{
		int source_label = sources[i];
		int target_label = targets[i];
		double dist1 = 0,
				dist2, dist3,
			dist_colors,
			dist_centers; 
		dist1 = ( centeres[source_label][0] - centeres[target_label][0] );
		dist1 = dist1 * dist1;
		dist2 = ( centeres[source_label][1] - centeres[target_label][1] );
		dist2 = dist2 * dist2;
		dist3 = ( centeres[source_label][2] - centeres[target_label][2] );
		dist3 = dist3 * dist3;
		dist_colors = dist1 + dist2 + dist3;

		dist1 = ( centeres[source_label][3] - centeres[target_label][3] );
		dist1 = dist1 * dist1;
		dist2 = ( centeres[source_label][4] - centeres[target_label][4] );
		dist2 = dist2 * dist2;
	
		dist_centers = sqrt(dist1 + dist2);		

		sum += dist_colors / dist_centers ;
	}
	betaSpatial = 0.5 / (sum/vectors_length) ; // 0.5 / mean

}

// algorithm functions. section 3.2 in the article //
// calcualte spatial and temporal functions //
void calc_pairwisePotentials(Slic *segmented_slic,Slic *prev_segmented_slic, Mat &flow, long long superPixels_accumulated)
{
	//similar to the original code and article algorithm.
	// calculate V,W  matrices.
	/*
	get list of  [ sSource, sDestination ] as result of getSpatialConnections by inputs of ( superpixels, labels )
	get list of  [ tSource, tDestination] and [ tConnections ] as result of getTemporalConnections by ( flow, superpixels, labels );
	calculate superpixels factors by colour distance(in spacial, and temporal), and geometrical centers distances
	finish calculations according to the article equations 8,9 */
	/*getSpatialConnections using the superpixel segmentation */
	

	//convert segmented_slic.clusters to required input format
	size_t	W				= segmented_slic->return_clusters_size();		// assumed same for current and previous frames
	size_t	H				= segmented_slic->return_clusters_size2();		// -"-
	long	num_of_sPixels	= segmented_slic->return_num_of_superpixels() ;
	unsigned int w = W;
	unsigned int h = H;
	///unsigned int eNum = num_of_sPixels;

		// ref: convert vector to array pointer by : http://stackoverflow.com/questions/1733143/converting-between-c-stdvector-and-c-array-without-copying
		//		or : http://stackoverflow.com/questions/6946217/pointer-to-a-vector
		//		& consider this: http://www.cprogramming.com/tutorial/stl/iterators.html
		//		& c this at end : http://stackoverflow.com/questions/6734472/how-to-get-a-pointer-to-a-2d-stdvectors-contents

	int				vec_size						= W*H;
	short			* converted_Mat_flow			= new short		   [vec_size*2]; // for 2 channels of the OpticalFlow data
	unsigned int	* converted_vector2d_slic		= new unsigned int [vec_size];
	unsigned int	* converted_vector2d_prevSlic	= new unsigned int [vec_size];
	unsigned int	*spatial_sources				= new unsigned int [vec_size];
	unsigned int	*spatial_targets				= new unsigned int [vec_size];
	unsigned int	*temporal_sources				= new unsigned int [vec_size];
	unsigned int	*temporal_targets				= new unsigned int [vec_size];
	float			*connectionRatio				= new float		   [vec_size];
	unsigned int	spatial_vectors_len				= 0;
	unsigned int	temporal_vectors_len			= 0;

	vector<double> spatial_color_dist;
	vector<double> spatial_center_dist;
	vector<double> temporal_color_dist , dummy;
	/////////

	copy_vectors_to_array(W,H,segmented_slic,converted_vector2d_slic); // returns 1D array for Mat representation. so arr[i][j] is for arr[row1 row2 .. rowN] flattened.

	calcSpatialConnections(converted_vector2d_slic , H, W, num_of_sPixels
								, spatial_sources, spatial_targets, &spatial_vectors_len);

	/////////

	copy_vectors_to_array(W,H,prev_segmented_slic,converted_vector2d_prevSlic); // returns 1D array for Mat representation. so arr[i][j] is for arr[row1 row2 .. rowN] flattened.
	copy_floatMat_to_shortArray(W,H,flow,converted_Mat_flow); // returns 1D array for Mat representation. so arr[i][j] is for arr[row1 row2 .. rowN] flattened.

	if ( !(flow.isContinuous() ) )
	{
		// http://docs.opencv.org/3.1.0/d3/d63/classcv_1_1Mat.html#aff83775c7fc1479de5f4a8c4e67fe361 
		cout << " indeces design ERROR - no continues stoarge... (in 'calc_votes_2') "; //because of matrix/pointer indexing
		return ;  
	}

	calcTemporalConnections(converted_Mat_flow, converted_vector2d_slic, H, W, converted_vector2d_prevSlic , superPixels_accumulated,
									temporal_sources, temporal_targets, connectionRatio , &temporal_vectors_len );
		 
	/* calculate the weight factors */

	function_to_calc_weight_part_1(spatial_sources, spatial_targets, spatial_vectors_len, prev_segmented_slic->return_centers() 
									,spatial_color_dist, spatial_center_dist   ); 

	function_to_calc_weight_part_1(temporal_sources, temporal_targets, temporal_vectors_len, segmented_slic->return_centers() 
									,temporal_color_dist, dummy   ); 
	
	//double tmp = sum (centeres[source_label][0] - centeres[target_label][0] );

	//float spatial_centers_dis = sqrt(pow(sum(centers_source_s - centers_targets_s),2)); //TODO:check validity of ^2/2
	//float spatial_colour_dis  = sqrt(pow(sum(colours_source_s - colours_targets_s),2)); //TODO:check validity of ^2/2
	//float temporal_colour_dis = sqrt(pow(sum(colours_source_t - colours_targets_t),2)); //TODO:check validity of ^2/2

	//double beta = ...  // according article, and 'computePairwisePotentials' function in main script.
	// wights..
	// set output as params.spatialWeight * sWeights ,	params.temporalWeight * tWeights 


	if ( converted_Mat_flow	)			delete converted_Mat_flow;
	if (converted_vector2d_slic) 		delete converted_vector2d_slic; 
	if (converted_vector2d_prevSlic) 	delete converted_vector2d_prevSlic; 
	if (spatial_sources)				delete spatial_sources; 
	if (spatial_targets)				delete spatial_targets; 
	if (temporal_sources)				delete temporal_sources; 
	if (temporal_targets)				delete temporal_targets; 
	if (connectionRatio)				delete connectionRatio;

}

// algorithm functions. section 3.2 in the article //
// calcualte spatial and temporal functions //
void calc_unary_potentials(Slic *segmented_slic,Slic *prev_segmented_slic, Mat &flow, long long superPixels_accumulated)
{
	
	//convert segmented_slic.clusters to required input format
	size_t	W				= segmented_slic->return_clusters_size();		// assumed same for current and previous frames
	size_t	H				= segmented_slic->return_clusters_size2();		// -"-
	long	num_of_sPixels	= segmented_slic->return_num_of_superpixels() ;
	unsigned int w = W;
	unsigned int h = H;


	int				vec_size						= W*H;
	short			* converted_Mat_flow			= new short		   [vec_size];
	unsigned int	* converted_vector2d_slic		= new unsigned int [vec_size];
	unsigned int	* converted_vector2d_prevSlic	= new unsigned int [vec_size];
	unsigned int	*spatial_sources				= new unsigned int [vec_size];
	unsigned int	*spatial_targets				= new unsigned int [vec_size];
	unsigned int	*temporal_sources				= new unsigned int [vec_size];
	unsigned int	*temporal_targets				= new unsigned int [vec_size];
	float			*connectionRatio				= new float		   [vec_size];

	/*
		accumulateInOutMap
		pre-settings 4 locationUnaries - global
		pre-settings 4 locationUnaries - per frame
		build fadeout matrix 
			weights( i ) = exp( - params.fadeout * ( i - middle ) ^ 2 );
		
		prepare segmenting by

			fgNodeWeights = masks( foregroundMasks ) .* ...
			weights( ids( foregroundMasks ) );
			bgNodeWeights = ( 1 - masks( backgroundMasks ) ) .* ...
			weights( ids( backgroundMasks ) );
		
		findUniqueColourWeights

		fitGMM..

		getUnaryAppearance : by 'pdf' functions
		unaryPotentials as Log ..

		maxflow_mex_optimisedWrapper




	 */
	if ( converted_Mat_flow	)			delete converted_Mat_flow;
	if (converted_vector2d_slic) 		delete converted_vector2d_slic; 
	if (converted_vector2d_prevSlic) 	delete converted_vector2d_prevSlic; 
	if (spatial_sources)				delete spatial_sources; 
	if (spatial_targets)				delete spatial_targets; 
	if (temporal_sources)				delete temporal_sources; 
	if (temporal_targets)				delete temporal_targets; 
	if (connectionRatio)				delete connectionRatio;

}

// TODO: try add constant grid for image windows.
// http://code.opencv.org/projects/opencv/wiki/DisplayManyImages
// http://stackoverflow.com/questions/5089927/show-multiple-2-3-4-images-in-the-same-window-in-opencv



int process_video_segmentation_algorithm(int, char**, bool vid_from_file)
{
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

	Mat		frame, prevframe;
    UMat	gray,  prevgray, uflow;
    Mat		flow,  cflow,    cflow2;
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
		IpImage			=  frame;
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
			calc_motion_boundaries(flow);////////////////////////////

			 //////////////////////////*  algorithm section 3.2 *///////////////////////////

			/* run pairwise potenrials */
			// use the current and previous frames. and flow result for those. and other inputs.
			calc_pairwisePotentials( &SlicOutput ,&prevSlicOutput, flow, superPixels_accumulated); 

		////	calc_unary_potentials();

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

    return 0;
}
