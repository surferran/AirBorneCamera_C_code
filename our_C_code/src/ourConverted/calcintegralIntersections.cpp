
#define min( a, b ) ( ( a ) < ( b ) ? ( a ) : ( b ) )


/* input flattened matrix is 'bp' , 
	output is the total votes flattened matrix (binaric) */
void calcintegralIntersections(  int height,  int width,  unsigned char* input ,
									unsigned char *outputData)
{
	unsigned char **integralMatrix;

	integralMatrix = new unsigned char*[ height ];
	for( short int i = 0; i < height; i++ )
		integralMatrix[ i ] = new unsigned char[ width ];

	unsigned char integral;

	/** Horizontal integral line **/
	for( short int i = 0; i < height; i++ )
	{
		integralMatrix[ i ][ 0 ] = 0;
		for( short int j = 1; j < width; j++ )
		{
			if( input[ j * height + i ] > input[ ( j - 1 ) * height + i ] )
				integralMatrix[ i ][ j ] = integralMatrix[ i ][ j - 1 ] + 1;
			else
				integralMatrix[ i ][ j ] = integralMatrix[ i ][ j - 1 ];
		}
	}

	/** Horizontal lines check **/
	for( int i = 0; i < height; i++ )
	{
		for( int j = 1; j < width; j++ )
		{
			if( integralMatrix[ i ][ j - 1 ] % 2 )
				outputData[ j * height + i ]++;
		}
	}

	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width - 1; j++ )
		{
			if( ( integralMatrix[ i ][ width - 1 ] - integralMatrix[ i ][ j ] ) % 2 )
				outputData[ j * height + i ]++;
		}
	}

	/** Vertical integral line **/
	for( short int j = 0; j < width; j++ )
	{
		integralMatrix[ 0 ][ j ] = 0;
		for( short int i = 1; i < height; i++ )
		{
			if( input[ j * height + i ] > input[ j * height + ( i - 1 ) ] )
				integralMatrix[ i ][ j ] = integralMatrix[ i - 1 ][ j ] + 1;
			else
				integralMatrix[ i ][ j ] = integralMatrix[ i - 1 ][ j ];
		}
	}

	/** Vertical lines check **/
	for( int i = 1; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			if( integralMatrix[ i - 1 ][ j ] % 2 )
				outputData[ j * height + i ]++;
		}
	}

	for( int i = 0; i < height - 1; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			if( ( integralMatrix[ height - 1 ][ j ] - integralMatrix[ i ][ j ] ) % 2 )
				outputData[ j * height + i ]++;
		}
	}

	/** Diagonal downward integral line **/
	for( short int start = 0; start < height; start++ )
	{
		short int stop = min( height - start, width );		
		short int i = start;
		short int j = 0;
		
		integralMatrix[ i ][ j ] = 0;
		for( short int step = 1; step < stop; step++ )
		{
			i = start + step;
			j = step;

			if( input[ j * height + i ] > input[ ( j - 1 ) * height + ( i - 1 ) ] )
				integralMatrix[ i ][ j ] = integralMatrix[ i - 1 ][ j - 1 ] + 1;
			else
				integralMatrix[ i ][ j ] = integralMatrix[ i - 1 ][ j - 1 ];
		}
	}

	for( short int start = 1; start < width; start++ )
	{
		short int stop = min( height, width - start );
		short int i = 0;
		short int j = start;

		integral = input[ j * height ];
		integralMatrix[ i ][ j ] = 0;
		for( short int step = 1; step < stop; step++ )
		{
			i = step;
			j = start + step;

			if( input[ j * height + i ] > input[ ( j - 1 ) * height + ( i - 1 ) ] )
				integralMatrix[ i ][ j ] = integralMatrix[ i - 1 ][ j - 1 ] + 1;
			else
				integralMatrix[ i ][ j ] = integralMatrix[ i - 1 ][ j - 1 ];
		}
	}

	/** Diagonal downward lines check **/
	for( int i = 1; i < height; i++ )
	{
		for( int j = 1; j < width; j++ )
		{
			if( integralMatrix[ i - 1 ][ j - 1 ] % 2 )
				outputData[ j * height + i ]++;
		}
	}

	for( int i = 0; i < height - 1; i++ )
	{
		for( int j = 0; j < width - 1; j++ )
		{
			short int endx;
			short int endy;

			if( height - i < width - j )
			{
				endx = height - 1;
				endy = j + height - 1 - i;
			}
			else
			{
				endx = i + width - 1 - j;
				endy = width - 1;
			}

			if( ( integralMatrix[ endx ][ endy ] - integralMatrix[ i ][ j ] ) % 2 )
				outputData[ j * height + i ]++;
		}
	}

	/** Diagonal upward integral line **/
	for( short int start = height - 1; start > -1; start-- )
	{
		short int stop = min( start + 1, width );
		short int i = start;
		short int j = 0;

		integralMatrix[ i ][ j ] = 0;
		for( short int step = 1; step < stop; step++ )
		{
			i = start - step;
			j = step;

			if( input[ j * height + i ] > input[ ( j - 1 ) * height + ( i + 1 ) ] )
				integralMatrix[ i ][ j ] = integralMatrix[ i + 1 ][ j - 1 ] + 1;
			else
				integralMatrix[ i ][ j ] = integralMatrix[ i + 1 ][ j - 1 ];
		}
	}

	for( short int start = 1; start < width; start++ )
	{
		short int stop = min( height, width - start ); 		
		short int i = height - 1;
		short int j = start;

		integralMatrix[ i ][ j ] = 0;
		for( short int step = 1; step < stop; step++ )
		{
			i = height - 1 - step;
			j = start + step;

			if( input[ j * height + i ] > input[ ( j - 1 ) * height + ( i + 1 ) ] )
				integralMatrix[ i ][ j ] = integralMatrix[ i + 1 ][ j - 1 ] + 1;
			else
				integralMatrix[ i ][ j ] = integralMatrix[ i + 1 ][ j - 1 ];
		}
	}

	/** Diagonal upward lines check **/
	for( int i = height - 2; i > - 1; i-- )
	{
		for( int j = 1; j < width; j++ )
		{
			if( integralMatrix[ i + 1 ][ j - 1 ] % 2 )
				outputData[ j * height + i ]++;
		}
	}
	
	for( int i = height - 1; i > 0; i-- )
	{
		for( int j = 0; j < width - 1; j++ )
		{
			short int endx;
			short int endy;

			if( i < width - 1 - j )
			{
				endx = 0;
				endy = j + i;
			}
			else
			{
				endx = i - width + 1 + j;
				endy = width - 1;
			}

			if( ( integralMatrix[ endx ][ endy ] - integralMatrix[ i ][ j ] ) % 2 )
				outputData[ j * height + i ]++;
		}
	}

	/** Deallocate memory */
	for( short int i = 0; i < height; i++ )
	{
		delete [] integralMatrix[ i ];
	}
	delete [] integralMatrix;

	/* summarize Votes>4 */
	long current_ndx = 0;
	for( int i = 0; i < height; i++ )
	{
		for( int j = 0; j < width; j++ )
		{
			current_ndx = j * height + i ;
			if( outputData[ current_ndx ] > 4 )
				outputData[ current_ndx ] = 1 ;
			else
				outputData[ current_ndx ] = 0 ;

		}
	} 
}
 