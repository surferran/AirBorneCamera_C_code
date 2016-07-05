#ifndef SPATIAL_CONNECTIONS_HPP
#define SPATIAL_CONNECTIONS_HPP 

#include <iostream>
#include <set>

void calcSpatialConnections(unsigned int *superpixelMap,	// the superpixels segmantation matrix
							unsigned int height,			// the H of the frame
							unsigned int width,				// the W of the frame
							unsigned long /*long*/ superpixels, // total number of s.pixels
								unsigned int *sources,				// output of sPixels sources
								unsigned int *targets,				// output of sPixels targets as the sources neighbours	
								unsigned int *vectors_length);
#endif