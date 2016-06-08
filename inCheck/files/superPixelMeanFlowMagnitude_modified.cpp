/** Function to compute the mean optical flow magnitude in each superpixel

/** Expected inputs:
			1 - Flow: a HxWx2 int16 matrix containing the optical flow
				between two pairs of frames 
			2 - Superpixel Map Previous: a HxW uint16 super pixel label map
		Optional inputs:
			3 - Number of superpixels in frame
*/

#include <matrix.h>
#include <cmath>

float *output = SuperpixelMeanFloeMagnitude(short *flow, unsigned short *labelsMap, int height, int width)
void mexFunction( int nlhs, mxArray *plhs[], int nrhs,
	const mxArray *prhs[] )
{
	
	// height and width of the labels map
	
	/** Get the number of superpixels in each frame */
	/** If number of superpixels is not given, find the largest label */
	labels = 0;
	int point;
	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			point = j * height + i;
			if( labelsMap[ point ] > labels )
				labels = labelsMap[ point ];
		}
	}	
	
	unsigned int *pixelCount = new unsigned int[ labels ];
	for( int i = 0; i < labels; i++ )
		pixelCount[ i ] = 0;
	
	/** Compute the connectivity matrix */
	int point;
	int label;
	int HxW = height * width;
	float magnitude;
	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			point = j * height + i;

			label = labelsMap[ point ] - 1;
			/** Make sure not to account for unlabeled pixels: label = 0 */
			if( ( label > -1 ) )
			{
				pixelCount[ label ]++;
				magnitude = sqrt( pow( (float)(flow[ point ]), 2.0 ) + pow( (float)(flow[ point + HxW ]), 2.0 ) );
				output[ label ] += magnitude;
			}
		}
	}
	
	for( int i = 0; i < labels; i++ )
	{
		if( pixelCount[ i ] )
			output[ i ] /= (float)( pixelCount[ i ] );
	}

	delete [] pixelCount;
	
	return output;
}
