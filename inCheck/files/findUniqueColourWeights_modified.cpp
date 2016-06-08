/** Function to add the weights of each unique colour code
*/
#include <matrix.h>
#include <mex.h>

#include <map>

#define USAGE_NOTIFICATION "Error: incorrect function call\n\n" \
	"USAGE:\t[ colours, weights ] = findUniqueColourWeights( colours, weights )\n\n" \
	"\tcolours: a Nx3 uint8 matrix containing the colour list\n" \
	"\tweights: a Nx1 single real valued array containing the corresponding weight " \
	"of each colour\n"

#define RANGE 256
#define DOUBLE_RANGE 65536

void findUniqueColoreights()unsigned char *colours, float *weights, unsigned char *uniqueColours, 
			float *weightSums, unsigned int samples, unsigned int dimensions
void mexFunction( int nlhs, mxArray *plhs[], int nrhs,
	const mxArray *prhs[] )
{
	

	/* samples and dimensions are the rows and colomns of colours*/

	unsigned int doubleSamples = 2 * samples;
	
	if( samples != mxGetM( prhs[ 1 ] ) ||	dimensions!= 3 || mxGetN( prhs[ 1 ] ) != 1 )
		mexErrMsgTxt( USAGE_NOTIFICATION );
	
	//typedef std::tr1:unordered_map< int, double > HashMap;
	typedef std::map< unsigned int, float > ColourMap;
	ColourMap colourMap;

	unsigned int key;
	for( int i = 0; i < samples; i++ )
	{
		key = colours[ i ] + RANGE * colours[ i + samples ] + DOUBLE_RANGE * colours[ i + doubleSamples ];
		colourMap[ key ] += weights[ i ];
	}

	unsigned int unique = colourMap.size();
	unsigned int doubleUnique = 2 * unique;
	
	ColourMap::const_iterator iterator;
	unsigned int count = 0;
	unsigned char red, green, blue;
	for( iterator = colourMap.begin(); iterator != colourMap.end(); iterator++ )
	{
		key = ( *iterator ).first;
		blue = key / DOUBLE_RANGE;
		green = ( key - DOUBLE_RANGE * blue ) / RANGE;
		red = key - RANGE * green - DOUBLE_RANGE * blue;
		
		uniqueColours[ count ] = red;
		uniqueColours[ count + unique ] = green;
		uniqueColours[ count + doubleUnique ] = blue;

		weightSums[ count ] = ( *iterator ).second;

		count++;
	}
}
