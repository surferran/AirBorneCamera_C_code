#include "composed_algorithm.hpp"
//#include "functions_file1.cpp"

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

		/* dis(Si,Sj) for different(unique) i,j labels */
		dist1 = ( centeres[source_label][3] - centeres[target_label][3] );
		dist1 = dist1 * dist1;
		dist2 = ( centeres[source_label][4] - centeres[target_label][4] );
		dist2 = dist2 * dist2;
	
		dist_centers = sqrt(dist1 + dist2);		

		/* sum of V(i,j) elements*/
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
////////////////////////////////////////////////////////////////////////////////////////

// algorithm functions. section 3.2 in the article //
// calcualte spatial and temporal functions //
void calc_inRatios(Slic *segmented_slic, Mat &votes, float  * inRatios)
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

	/*vector<double> spatial_color_dist;
	vector<double> spatial_center_dist;
	vector<double> temporal_color_dist , dummy;*/
	/////////

	copy_vectors_to_array(W,H,	segmented_slic,	converted_vector2d_slic); // returns 1D array for Mat representation. so arr[i][j] is for arr[row1 row2 .. rowN] flattened.
	///copy_floatMat_to_shortArray(W,H,flow,converted_Mat_flow); // returns 1D array for Mat representation. so arr[i][j] is for arr[row1 row2 .. rowN] flattened.

///	calcSuperpixelInRatio(converted_vector2d_slic, h, w,&num_of_sPixels, votes, inRatios);

	if ( converted_Mat_flow	)			delete converted_Mat_flow;
	if (converted_vector2d_slic) 		delete converted_vector2d_slic; 
	if (converted_vector2d_prevSlic) 	delete converted_vector2d_prevSlic; 
	if (spatial_sources)				delete spatial_sources; 
	if (spatial_targets)				delete spatial_targets; 
	if (temporal_sources)				delete temporal_sources; 
	if (temporal_targets)				delete temporal_targets; 
	if (connectionRatio)				delete connectionRatio;

}
////////////////////////////////////////////////////////////////////////////////////////

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
