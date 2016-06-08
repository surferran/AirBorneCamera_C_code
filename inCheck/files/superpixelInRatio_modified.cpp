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

#include <matrix.h>

float * superpixelInRatio(unsigned short *superpixels, int height, int width, bool *inOutMap)
{
	
	float *output[width][height];
	unsigned int *spixelSize[width][height];
	
	/** Get the number of superpixels */	
	/** If number of superpixels is not given, find the largest label */
	/** We can get the labels value from outside --------- Yair */
	labels = 0;
	int point;
	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			point = j * height + i;
			if( superpixels[ point ] > labels )
				labels = superpixels[ point ];
		}
	}
	
	/** Compute the superpixel metric matrix */
	int point;
	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			point = j * height + i;
			if( inOutMap[ point ] )
			{
				/** Note: superpixel labels are 1 to N */
				output[ superpixels[ point ] - 1 ]++;
			}
			spixelSize[ superpixels[ point ] - 1 ]++;
		}
	}

	for( int label = 0; label < labels; label++ )
	{
		if( spixelSize[ label ] > 0 )
			output[ label ] = output[ label ] / float( spixelSize[ label ] );
	}
	
	return output;

}
