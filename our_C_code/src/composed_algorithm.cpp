#include "composed_algorithm.hpp"


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

	cv::Mat xy[2]; //X,Y
	cv::split(flow, xy);

	for (int i=0; i<h; i++)
	{	
		tmp =  flow.ptr<float>(i);
		tmpX = xy[0].ptr<float>(i);
		tmpY = xy[1].ptr<float>(i);
		for(int j=0; j<w; j++)
		{
			const Point2f& fxy = flow.at<Point2f>(i, j); 
			out_array[i+ j*h]				= (short)tmpX[j];  // this should be fine for a Mat.
			out_array[i+ j*h + elements]	= (short)tmpY[j];  // this should be fine for a Mat.
		}
	}

	
}

void copy_binaricMat_to_ucharArray(int w, int h,  Mat & bp ,  unsigned char * out_array)
{
	float* tmp = NULL;  

	for (int i=0; i<h; i++)
	{	
		tmp =  bp.ptr<float>(i);
		for(int j=0; j<w; j++)
		{
			out_array[i+ j*h] = (unsigned char)tmp[j];  
		}
	}
}
void copy_ucharArray_to_binaricMat(int w, int h, unsigned char * in_array , Mat & bp  )
{
	float* tmp = NULL;  

	for (int i=0; i<h; i++)
	{	
		tmp =  bp.ptr<float>(i);
		for(int j=0; j<w; j++)
		{ 
			tmp[j] = (float)in_array[i+ j*h];
		}
	}
}

/* calculate the color and centers-geometric distances of the SuperPixels,
	according to the Spatial connections input of sources and targets.
	the centers list contains both LAB data and XY data for each sPixel(label).
*/
void calc_Spatial_distances_and_Weights(unsigned int	*sources, unsigned int	*targets, 
										unsigned int vectors_length , vec2dd centers , double *alpha2,
										double *color_dist, double *center_dist , double *Vij , double *sum_Out)
{ 
	double			sum				= 0;
	double			betaSpatial		= 0; 

	if (vectors_length<1) 
		return;				// TODO: may add error notice

	for (int i=0; i< vectors_length; i++)
	{
		int source_label = sources[i];
		int target_label = targets[i];
		double	dist1 , dist2 , dist3 ;

		/* col(Si,Sj) for different(unique) i,j labels */
		dist1 = ( centers[source_label][0] - centers[target_label][0] );
		dist1 = dist1 * dist1;
		dist2 = ( centers[source_label][1] - centers[target_label][1] );
		dist2 = dist2 * dist2;
		dist3 = ( centers[source_label][2] - centers[target_label][2] );
		dist3 = dist3 * dist3;
		color_dist[i] = dist1 + dist2 + dist3;
		 
		/* dis(Si,Sj) for different(unique) i,j labels */
		dist1 = ( centers[source_label][3] - centers[target_label][3] );
		dist1 = dist1 * dist1;
		dist2 = ( centers[source_label][4] - centers[target_label][4] );
		dist2 = dist2 * dist2;
	
		center_dist[i] = sqrt(dist1 + dist2);		

		/* sum of V(i,j) elements*/
		sum += color_dist[i] / center_dist[i] ;
	}

	double mean = (sum/vectors_length) ;
	betaSpatial = 0.5 / mean ; 

	/*Vij_list */
	*sum_Out	=	0 ;
	for (unsigned int i=0; i< vectors_length; i++)
	{
		Vij[i]		=	exp( -betaSpatial * color_dist[i] ) / center_dist[i] ;
		*sum_Out   +=	(*alpha2) * Vij[i];
	}

}


/* calculate the color distances of the SuperPixels,
according to the Temporal connections input of sources and targets. connectionRatios are given as PHI function.
the centers list contains both LAB data and XY data for each sPixel(label).
*/
void calc_Temporal_distances_and_Weights(	unsigned int	*sources, unsigned int	*targets, float	*connectionRatios, 
											unsigned int vectors_length , vec2dd centers , double *alpha3,
											double* color_dist , double *Wij, double *sum_Out)
{ 
	double sum			= 0;
	double betaTemporal	= 0;

	if (vectors_length<1) 
		return;				// TODO: may add error notice

	for (int i=0; i< vectors_length; i++)
	{
		int		source_label = sources[i];
		int		target_label = targets[i]; 
		double	dist1 , dist2 , dist3;

		/* col(Si,Sj) for different(unique) i,j labels */
		dist1 = ( centers[source_label][0] - centers[target_label][0] );
		dist1 = dist1 * dist1;
		dist2 = ( centers[source_label][1] - centers[target_label][1] );
		dist2 = dist2 * dist2;
		dist3 = ( centers[source_label][2] - centers[target_label][2] );
		dist3 = dist3 * dist3;
		color_dist[i] = dist1 + dist2 + dist3;	

		/* sum of W(i,j) elements*/
		sum += color_dist[i] * connectionRatios[i];
	}

	double mean = (sum/vectors_length) ;
	betaTemporal = 0.5 / mean ; 

	/*Wij_list */
	*sum_Out	=	0;
	for (int i=0; i< vectors_length; i++)
	{
		Wij[i]		=	exp( -betaTemporal * color_dist[i] ) * connectionRatios[i] ;
		*sum_Out   +=	(*alpha3) * Wij[i];
	}
}
 
                   
// algorithm functions. section 3.2 in the article //
// calcualte spatial and temporal functions //
void calc_pairwisePotentials(Slic *segmented_slic,Slic *prev_segmented_slic, Mat &flow, 
									/*unsigned */long /*long*/ superPixels_accumulated,	double *pairWise_Weight)
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
	size_t		W				= segmented_slic->return_clusters_size();		// assumed same for current and previous frames
	size_t		H				= segmented_slic->return_clusters_size2();		// -"-
	unsigned long long	num_of_sPixels	= segmented_slic->return_num_of_superpixels() ; 

		// ref: convert vector to array pointer by : http://stackoverflow.com/questions/1733143/converting-between-c-stdvector-and-c-array-without-copying
		//		or : http://stackoverflow.com/questions/6946217/pointer-to-a-vector
		//		& consider this: http://www.cprogramming.com/tutorial/stl/iterators.html
		//		& c this at end : http://stackoverflow.com/questions/6734472/how-to-get-a-pointer-to-a-2d-stdvectors-contents

	long			vec_size						= W*H;
	short			* converted_Mat_flow			= new short		   [vec_size*2]; // for 2 channels of the OpticalFlow data
	unsigned int	* converted_vector2d_slic		= new unsigned int [vec_size];
	unsigned int	* converted_vector2d_prevSlic	= new unsigned int [vec_size];
	unsigned int	*spatial_sources				= new unsigned int [vec_size];	// preparing maximum possible size
	unsigned int	*spatial_targets				= new unsigned int [vec_size];	//
	unsigned int	*temporal_sources				= new unsigned int [vec_size];	//
	unsigned int	*temporal_targets				= new unsigned int [vec_size];	//
	float			*connectionRatio				= new float		   [vec_size];	//
	unsigned int	spatial_vectors_len				= 0;
	unsigned int	temporal_vectors_len			= 0;

	
	/////////

	copy_vectors_to_array(W,H,segmented_slic,converted_vector2d_slic); // returns 1D array for Mat representation. so arr[i][j] is for arr[row1 row2 .. rowN] flattened.

	calcSpatialConnections(	converted_vector2d_slic , H, W, num_of_sPixels ,
							spatial_sources, spatial_targets, &spatial_vectors_len);

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

	double	*spatial_color_dist		=	new double [spatial_vectors_len] ,
			*spatial_center_dist	=	new double [spatial_vectors_len] ,
			*temporal_color_dist	=	new double [temporal_vectors_len] ,
			*Vij					=	new double [spatial_vectors_len] ,/*sWeights*/
			*Wij					=	new double [temporal_vectors_len] ;/*tWeights*/

	double	alpha2	=	1,		// TODO: must put those outside. get them as user parameters.
			alpha3	=	1;

	double	Spatial_part	=0,
			Temporal_part	=0;

	calc_Spatial_distances_and_Weights(	spatial_sources , spatial_targets , spatial_vectors_len , 
										prev_segmented_slic->return_centers() ,	&alpha2 ,
										spatial_color_dist , spatial_center_dist , Vij, &Spatial_part  ); 

	calc_Temporal_distances_and_Weights(temporal_sources , temporal_targets , connectionRatio ,
										temporal_vectors_len , segmented_slic->return_centers() , &alpha3 ,
										temporal_color_dist , Wij , &Temporal_part ); 
	 
	 
	// later on the outcome of this part will be 
	//  alpha2 * sum(Vij) + alpha3 * sum(Wij)
	*pairWise_Weight	=	Spatial_part + Temporal_part ;

	if ( converted_Mat_flow	)			delete [] converted_Mat_flow;
	if (converted_vector2d_slic) 		delete [] converted_vector2d_slic; 
	if (converted_vector2d_prevSlic) 	delete [] converted_vector2d_prevSlic; 
	if (spatial_sources)				delete [] spatial_sources; 
	if (spatial_targets)				delete [] spatial_targets; 
	if (temporal_sources)				delete [] temporal_sources; 
	if (temporal_targets)				delete [] temporal_targets; 
	if (connectionRatio)				delete [] connectionRatio;

	if (spatial_color_dist)				delete [] spatial_color_dist;
	if (spatial_center_dist)			delete [] spatial_center_dist;
	if (temporal_color_dist)			delete [] temporal_color_dist;
	if (Vij)							delete [] Vij;
	if (Wij)							delete [] Wij;

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
	///unsigned int eNum = num_of_sPixels;

	// ref: convert vector to array pointer by : http://stackoverflow.com/questions/1733143/converting-between-c-stdvector-and-c-array-without-copying
	//		or : http://stackoverflow.com/questions/6946217/pointer-to-a-vector
	//		& consider this: http://www.cprogramming.com/tutorial/stl/iterators.html
	//		& c this at end : http://stackoverflow.com/questions/6734472/how-to-get-a-pointer-to-a-2d-stdvectors-contents

	int				vec_size						= W*H;
	short			* converted_Mat_Votes			= new short		   [vec_size*2]; // for 2 channels of the OpticalFlow data
	unsigned int	* converted_vector2d_slic		= new unsigned int [vec_size];
	unsigned int	* converted_vector2d_prevSlic	= new unsigned int [vec_size];
	unsigned int	*spatial_sources				= new unsigned int [vec_size];
	unsigned int	*spatial_targets				= new unsigned int [vec_size];
	unsigned int	*temporal_sources				= new unsigned int [vec_size];
	unsigned int	*temporal_targets				= new unsigned int [vec_size];
	float			*connectionRatio				= new float		   [vec_size];
	unsigned int	spatial_vectors_len				= 0;
	unsigned int	temporal_vectors_len			= 0;
 

	copy_vectors_to_array(W,H,	segmented_slic,	converted_vector2d_slic); // returns 1D array for Mat representation. so arr[i][j] is for arr[row1 row2 .. rowN] flattened.
//	copy_floatMat_to_shortArray(W,H, votes , converted_Mat_Votes); // returns 1D array for Mat representation. so arr[i][j] is for arr[row1 row2 .. rowN] flattened.

//	calcSuperpixelInRatio(converted_vector2d_slic, H, W, &num_of_sPixels, converted_Mat_Votes, inRatios );

	if ( converted_Mat_Votes	)		delete [] converted_Mat_Votes;
	if (converted_vector2d_slic) 		delete [] converted_vector2d_slic; 
	if (converted_vector2d_prevSlic) 	delete [] converted_vector2d_prevSlic; 
	if (spatial_sources)				delete [] spatial_sources; 
	if (spatial_targets)				delete [] spatial_targets; 
	if (temporal_sources)				delete [] temporal_sources; 
	if (temporal_targets)				delete [] temporal_targets; 
	if (connectionRatio)				delete [] connectionRatio;

}
////////////////////////////////////////////////////////////////////////////////////////

// algorithm functions. section 3.2 in the article //
// calcualte spatial and temporal functions //
void calc_unary_potentials(Slic *segmented_slic, Mat &frame_Votes)
{
	
	//convert segmented_slic.clusters to required input format
	size_t	W				= segmented_slic->return_clusters_size();		// assumed same for current and previous frames
	size_t	H				= segmented_slic->return_clusters_size2();		// -"-
	long	num_of_sPixels	= segmented_slic->return_num_of_superpixels() ;
	

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
	if ( converted_Mat_flow	)			delete [] converted_Mat_flow;
	if (converted_vector2d_slic) 		delete [] converted_vector2d_slic; 
	if (converted_vector2d_prevSlic) 	delete [] converted_vector2d_prevSlic; 
	if (spatial_sources)				delete [] spatial_sources; 
	if (spatial_targets)				delete [] spatial_targets; 
	if (temporal_sources)				delete [] temporal_sources; 
	if (temporal_targets)				delete [] temporal_targets; 
	if (connectionRatio)				delete [] connectionRatio;

}
