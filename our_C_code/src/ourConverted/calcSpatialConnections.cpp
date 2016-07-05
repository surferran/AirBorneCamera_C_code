
#include "calcSpatialConnections.hpp"

void calcSpatialConnections(unsigned int	   *superpixelMap,	// the superpixels segmantation matrix
							unsigned int		height,			// the H of the frame
							unsigned int		width,				// the W of the frame
							unsigned long /*long*/	superpixels, // total number of s.pixels
								unsigned int *sources,				// output of sPixels sources
								unsigned int *targets,				// output of sPixels targets as the sources neighbours				
								unsigned int *vectors_length)
{
	unsigned int		point, pointNext, superpixel, superpixelNext, count;
    unsigned long long	connection;

	std::set<unsigned long long> connections;	

	for( unsigned int i = 0; i < height; i++ )
	{
		for( unsigned int j = 0; j < width; j++ )
		{
			point = j * height + i;
			superpixel = superpixelMap[ point ] /*- 1*/;

			if( i > 0 )
			{
				pointNext = j * height + ( i - 1 );
				superpixelNext = superpixelMap[ pointNext ] /*- 1*/;

				if( superpixel < superpixelNext )
					connections.insert( superpixel + superpixels * superpixelNext );
			}

			if( j > 0 )
			{
				pointNext = ( j - 1 ) * height + i;
				superpixelNext = superpixelMap[ pointNext ] /*- 1*/;

				if( superpixel < superpixelNext )
					connections.insert( superpixel + superpixels * superpixelNext );
			}

			if( i < height - 1 )
			{
				pointNext = j * height + ( i + 1 );
				superpixelNext = superpixelMap[ pointNext ] /*- 1*/;

				if( superpixel < superpixelNext )
					connections.insert( superpixel + superpixels * superpixelNext );
			}

			if( j < width - 1 )
			{
				pointNext = ( j + 1 ) * height + i;
				superpixelNext = superpixelMap[ pointNext ] /*- 1*/;

				if( superpixel < superpixelNext )
					connections.insert( superpixel + superpixels * superpixelNext );
			}

			if( i > 0 && j > 0 )
			{
				pointNext = ( j - 1 ) * height + ( i - 1 );
				superpixelNext = superpixelMap[ pointNext ] /*- 1*/;

				if( superpixel < superpixelNext )
					connections.insert( superpixel + superpixels * superpixelNext );
			}

			if( i > 0 && j < width - 1 )
			{
				pointNext = ( j + 1 ) * height + ( i - 1 );
				superpixelNext = superpixelMap[ pointNext ] /*- 1*/;

				if( superpixel < superpixelNext )
					connections.insert( superpixel + superpixels * superpixelNext );
			}

			if( i < height - 1 && j > 0 )
			{
				pointNext = ( j - 1 ) * height + ( i + 1 );
				superpixelNext = superpixelMap[ pointNext ] /*- 1*/;

				if( superpixel < superpixelNext )
					connections.insert( superpixel + superpixels * superpixelNext );
			}

			if( i < height - 1 && j < width - 1 )
			{
				pointNext = ( j + 1 ) * height + ( i + 1 );
				superpixelNext = superpixelMap[ pointNext ] /*- 1*/;

				if( superpixel < superpixelNext )
					connections.insert( superpixel + superpixels * superpixelNext );
			}
		}
	}


	count = 0;
	for( std::set<unsigned long long>::iterator i = connections.begin(); i != connections.end(); i++ )
	{
		connection = *i;

		targets[ count ] = connection / superpixels;
		sources[ count ] = connection % superpixels;

		count++;
	}
	*vectors_length = count;
}
