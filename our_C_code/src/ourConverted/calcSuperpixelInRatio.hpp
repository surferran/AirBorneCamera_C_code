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
#ifndef SP_IN_RATIOS_HPP
#define SP_IN_RATIOS_HPP 

void calcSuperpixelInRatio(unsigned int *superpixels, int height, int width,const unsigned short *labels, bool *inOutMap,
							float * output);

#endif