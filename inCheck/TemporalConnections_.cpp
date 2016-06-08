/** Function to compute the temporal connections between subsequent frames
%
%    Copyright (C) 2013  Anestis Papazoglou
%
%    You can redistribute and/or modify this software for non-commercial use
%    under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
%    For commercial use, contact the author for licensing options.
%
%    Contact: a.papazoglou@sms.ed.ac.uk */

/** Expected inputs:
			1 - Flow: a cell array of length frames-1. Each cell i contains a HxWx2
				int16 matrix containing the optical flow between frames i and i+1.
			2 - Superpixels: a cell array of length frames. Each cell i contains a
				HxW uint32 superpixel superpixel map of frame i.
			3 - Number of superpixels: a double value containing the total number of
				superpixel superpixels.

		Outputs:
			1 - Source superpixels:
			2 - Target superpixels:
			3 - Connected pixels:
*/

#include <matrix.h>

#include <map>

void TemporalConnections( unsigned int *superpixelMap, unsigned int height, unsigned int width, unsigned int *superpixelMapNext, 
						unsigned int *sources, unsigned int*targets )
{
	unsigned int frames, height, width, point, pointNext, superpixel, superpixelNext, elements, count, xNext, yNext;
    unsigned long long superpixels, connection;
	short *flow;
	unsigned int *superpixelMap, *superpixelMapNext, *sources, *targets, *numSuperpixelConnections;
	float *connectionRatio;

	std::map<unsigned long long, float> connections;

	numSuperpixelConnections = new unsigned int[ superpixels ];
	for( unsigned int superpixel; superpixel < superpixels; superpixel++ )
		numSuperpixelConnections[ superpixel ] = 0;

	for( unsigned int frame = 0; frame < frames - 1; frame++ )
	{
		superpixelMap = ( unsigned int * )( mxGetData( mxGetCell( prhs[ 1 ], frame ) ) );
		superpixelMapNext = ( unsigned int * )( mxGetData( mxGetCell( prhs[ 1 ], frame + 1) ) );
		flow = ( short * )( mxGetData( mxGetCell( prhs[ 0 ], frame ) ) );

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

				superpixel = superpixelMap[ point ] - 1;
				superpixelNext = superpixelMapNext[ pointNext ] - 1;

				connection = superpixel + superpixels * superpixelNext;
				connections[ connection ]++;
				numSuperpixelConnections[ superpixel ]++;
			}
		}
	}

	mxArray *sourcesMxArray = mxCreateNumericMatrix( connections.size(), 1, mxUINT32_CLASS, mxREAL );
	mxArray *targetsMxArray = mxCreateNumericMatrix( connections.size(), 1, mxUINT32_CLASS, mxREAL );
	mxArray *connectionRatioMxArray = mxCreateNumericMatrix( connections.size(), 1, mxSINGLE_CLASS, mxREAL );

	sources = ( unsigned int * )mxGetData( sourcesMxArray );
	targets = ( unsigned int * )mxGetData( targetsMxArray );
	connectionRatio = ( float * )mxGetData( connectionRatioMxArray );

	count = 0;
	for( std::map<unsigned long long, float>::iterator i = connections.begin(); i != connections.end(); i++ )
	{
		connection = i->first;

		targets[ count ] = connection / superpixels;
		sources[ count ] = connection % superpixels;
		connectionRatio[ count ] = i->second / numSuperpixelConnections[ sources[ count ] ];

		count++;
	}

	plhs[ 0 ] = sourcesMxArray;
	plhs[ 1 ] = targetsMxArray;
	plhs[ 2 ] = connectionRatioMxArray;

}

