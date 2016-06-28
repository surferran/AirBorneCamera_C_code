
#include "calcTemporalConnections.hpp"

void calcTemporalConnections( 	short		 *flow,
								unsigned int *superpixelMap, 
								unsigned int  height, 
								unsigned int  width, 
								unsigned int *superpixelMapNext, 
								unsigned long long superpixels,
									unsigned int *sources,
									unsigned int *targets,
									float		 *connectionRatio  )
{
	
	unsigned int frames, /*height, width, */point, pointNext, superpixel, superpixelNext, elements, count, xNext, yNext;
    unsigned long long /*superpixels,*/ connection;
	/*short *flow;float *connectionRatio;*/
	unsigned int /**superpixelMap, *superpixelMapNext, *sources, *targets,*/ *numSuperpixelConnections;

	std::map<unsigned long long, float> connections;

	numSuperpixelConnections = new unsigned int[ superpixels ];
	for( unsigned int superpixel = 0; superpixel < superpixels; superpixel++ )
		numSuperpixelConnections[ superpixel ] = 0;

	elements = height * width;

	{

		for( unsigned int i = 0; i < height; i++ )
		{
			for( unsigned int j = 0; j < width; j++ )
			{
				point = j * height + i;
				xNext = i + flow[ point ];
				yNext = j + flow[ point + elements ];

				// Make sure that the flow does not lead outside the frame boundaries
				if( xNext < 0 || xNext >= height || yNext < 0 || yNext >= width )
					continue;

				pointNext = yNext * height + xNext;

				superpixel		= superpixelMap[ point ] - 1;
				superpixelNext	= superpixelMapNext[ pointNext ] - 1;

				connection = superpixel + superpixels * superpixelNext;
				connections[ connection ]++;
				numSuperpixelConnections[ superpixel ]++;
			}
		}
	}

	// arrays size wil b connections.size() rows on 1 column.
	
	count = 0;
	for( std::map<unsigned long long, float>::iterator i = connections.begin(); i != connections.end(); i++ )
	{
		connection = i->first;

		targets[ count ]			= connection / superpixels;
		sources[ count ]			= connection % superpixels;
		connectionRatio[ count ]	= i->second / numSuperpixelConnections[ sources[ count ] ];

		count++;
	}
	
}

