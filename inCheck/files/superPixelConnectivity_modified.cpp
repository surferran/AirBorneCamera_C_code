/** Function to compute the superpixel connections between two subsequent frames
		based on the optical flow
/** Expected inputs:
			1 - Flow: a HxWx2 int16 matrix containing the optical flow
				between two pairs of frames 
			2 - Superpixel Map Previous: a HxW uint16 super pixel label map
				for the first frame
			3 - Superpixel Map Next: a HxW uint16 super pixel label map
				for the second frame
		Optional inputs:
			4 - Number of superpixels in first frame
			5 - Number of superpixels in second frame
			
*/

#include <matrix.h>

unsigned int *output = superpixelsConnectivity(short *flow, unsigned short *previousFrame,
											unsigned short *nextFrame,int height, int width)
void mexFunction( int nlhs, mxArray *plhs[], int nrhs,
	const mxArray *prhs[] )
{
	// height and width of frame 	
	/** Get the number of superpixels in each frame */
	int labelsPrevious;
	int labelsNext;

	/** If number of superpixels is not given, find the largest label */
	labelsPrevious = 0;
	labelsNext = 0;
	int point;
	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			point = j * height + i;
			if( previousFrame[ point ] > labelsPrevious )
				labelsPrevious = previousFrame[ point ];
			
			if( nextFrame[ point ] > labelsNext )
				labelsNext = nextFrame[ point ];
		}
	}
	
	/** Compute the connectivity matrix */
	int point, nextPoint;
	int xNext, yNext;
	int previousLabel, nextLabel;
	int HxW = height * width;
	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			point = j * height + i;
			xNext = i + flow[ point ];
			yNext = j + flow[ point + HxW ];
			if( xNext > -1 && xNext < height &&
				yNext > -1 && yNext < width )
			{
				nextPoint = yNext * height + xNext;
				previousLabel = previousFrame[ point ] - 1;
				nextLabel = nextFrame[ nextPoint ] - 1;
				/** Make sure not to account for unlabeled pixels: label = 0 */
				if( ( previousLabel > -1 ) && ( nextLabel > -1 ) )
					output[ previousLabel + nextLabel * labelsPrevious ]++;
			} 
		}
	}

	return output;
}
