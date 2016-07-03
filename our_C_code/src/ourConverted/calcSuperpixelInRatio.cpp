/** Function to compute the ratio of inside-points in each superpixel
/** Expected inputs:
			1 - Superpixel label map: a HxW uint16 matrix specifying
				the superpixel each pixel belongs to
			2 - In-Out map: a HxW boolean matrix containing specifying
				whether a pixel is in (true) or out (false)
		Optional inputs:
			3 - Number of superpixels

		Outputs:
			1 - Superpixel in-ratio: an Nx1 array of type single, containing
				the ratio of 'inside' pixels in each superpixel. N corresponds
				to the number of 
*/

#include <memory.h> 

void calcSuperpixelInRatio(unsigned int *superpixels, int height, int width,const unsigned short *labels, bool *inOutMap,
						float * output)
{
	
	///float *output[width][height];
	const int labels_size = *labels;
	//unsigned int spixelSize[labels_size];	//TODO: make sure to zero this

	unsigned int	*spixelSize				= new unsigned int [labels_size];

	memset(spixelSize, 0, sizeof(spixelSize) );
	/** Get the number of superpixels */	
	/** If number of superpixels is not given, find the largest label */
	/** We can get the labels value from outside --------- Yair */

	
	/** Compute the superpixel metric matrix */
	int point;
	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			point = j * height + i;
			if( inOutMap[ point ] )
			{
				/** Note: superpixel labels are 0 to N-1 */
				output[ superpixels[ point ]  ]++;
			}
			spixelSize[ superpixels[ point ] ]++;
		}
	}

	for( int label = 0; label < labels_size; label++ )
	{
		if( spixelSize[ label ] > 0 )
			output[ label ] = output[ label ] / float( spixelSize[ label ] );
	}

}
