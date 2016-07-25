#ifndef TEMPORAL_CONNECTIONS_HPP
#define TEMPORAL_CONNECTIONS_HPP  

#include <map>

void calcTemporalConnections( 	short		 *flow,
								unsigned int *superpixelMap, 
								unsigned int  height, 
								unsigned int  width, 
								unsigned int *superpixelMapNext, 
								unsigned long /*long*/ superpixels,
									unsigned int *sources,
									unsigned int *targets,
									float		 *connectionRatio ,	
									unsigned int *vectors_length);
#endif


