
#include "integral_volume.h"

IntegralVolume::IntegralVolume( float *dataIn, int width, int height, int depth ){

	m_width = width;
	m_height = height;
	m_depth = depth;

	m_iVolume_32S = NULL;
	m_iVolume_64F = new float [width * height * depth];

	m_radius = 1;

	//first element
	m_iVolume_64F[0] = dataIn[0];
	for(int k=1;k<m_depth;k++)
		m_iVolume_64F[k*m_width*m_height] = dataIn[k*m_width*m_height]+	m_iVolume_64F[(k-1)*m_width*m_height];

	//first row
	for(int i=1; i<m_width; i++)
		m_iVolume_64F[i] = dataIn[i] + m_iVolume_64F[i-1];
	
	for(int k=1; k<m_depth; k++)
		for(int i=1; i<m_width; i++)
			m_iVolume_64F[k*m_width*m_height+i] = dataIn[k*m_width*m_height+i] + m_iVolume_64F[k*m_width*m_height+i-1] + m_iVolume_64F[(k-1)*m_width*m_height+i] - m_iVolume_64F[(k-1)*m_width*m_height+i-1];

	//remaining rows
	
	for(int k=0;k<m_depth;k++)
	{
		for(int j=1; j<m_height; j++)
		{
			//first element
			if(k==0)
				m_iVolume_64F[j*m_width] = dataIn[j*m_width] + m_iVolume_64F[(j-1)*m_width];
			else
				m_iVolume_64F[k*m_width*m_height+j*m_width] = dataIn[k*m_width*m_height+j*m_width] + m_iVolume_64F[k*m_width*m_height+(j-1)*m_width] + m_iVolume_64F[(k-1)*m_width*m_height + j*m_width] - m_iVolume_64F[(k-1)*m_width*m_height + (j-1)*m_width];
			//remaining elements
			for(int i=1; i<m_width; i++)
			{
				if(k==0)
					m_iVolume_64F[j*m_width + i] = dataIn[j*m_width + i]+ m_iVolume_64F[(j-1)*m_width + i] + m_iVolume_64F[j*m_width + i - 1] - m_iVolume_64F[(j-1)*m_width + i - 1];
				else
					m_iVolume_64F[k*m_width*m_height + j*m_width + i] = dataIn[k*m_width*m_height + j*m_width + i] + m_iVolume_64F[k*m_width*m_height +(j-1)*m_width + i] + m_iVolume_64F[k*m_width*m_height + j*m_width + i - 1] - m_iVolume_64F[k*m_width*m_height + (j-1)*m_width + i - 1] + m_iVolume_64F[(k-1)*m_width*m_height + j*m_width + i] - m_iVolume_64F[(k-1)*m_width*m_height + (j-1)*m_width + i] - m_iVolume_64F[(k-1)*m_width*m_height + j*m_width + i-1] + m_iVolume_64F[(k-1)*m_width*m_height + (j-1)*m_width + i-1];
			}
		}
	}	
}



IntegralVolume::IntegralVolume( int *dataIn, int width, int height, int depth ){

	m_width = width;
	m_height = height;
	m_depth = depth;

	m_iVolume_64F = NULL;
	m_iVolume_32S = new int [width * height * depth];

	m_radius = 1;

	//first element
	m_iVolume_32S[0] = dataIn[0];
	for(int k=1;k<m_depth;k++)
		m_iVolume_32S[k*m_width*m_height] = dataIn[k*m_width*m_height]+	m_iVolume_32S[(k-1)*m_width*m_height];

	//first row
	for(int i=1; i<m_width; i++)
		m_iVolume_32S[i] = dataIn[i] + m_iVolume_32S[i-1];
	
	for(int k=1; k<m_depth; k++)
		for(int i=1; i<m_width; i++)
			m_iVolume_32S[k*m_width*m_height+i] = dataIn[k*m_width*m_height+i] + m_iVolume_32S[k*m_width*m_height+i-1] + m_iVolume_32S[(k-1)*m_width*m_height+i] - m_iVolume_32S[(k-1)*m_width*m_height+i-1];

	//remaining rows
	
	for(int k=0;k<m_depth;k++)
	{
		for(int j=1; j<m_height; j++)
		{
			//first element
			if(k==0)
				m_iVolume_32S[j*m_width] = dataIn[j*m_width] + m_iVolume_32S[(j-1)*m_width];
			else
				m_iVolume_32S[k*m_width*m_height+j*m_width] = dataIn[k*m_width*m_height+j*m_width] + m_iVolume_32S[k*m_width*m_height+(j-1)*m_width] + m_iVolume_32S[(k-1)*m_width*m_height + j*m_width] - m_iVolume_32S[(k-1)*m_width*m_height + (j-1)*m_width];
			//remaining elements
			for(int i=1; i<m_width; i++)
			{
				if(k==0)
					m_iVolume_32S[j*m_width + i] = dataIn[j*m_width + i]+ m_iVolume_32S[(j-1)*m_width + i] + m_iVolume_32S[j*m_width + i - 1] - m_iVolume_32S[(j-1)*m_width + i - 1];
				else
					m_iVolume_32S[k*m_width*m_height + j*m_width + i] = dataIn[k*m_width*m_height + j*m_width + i] + m_iVolume_32S[k*m_width*m_height +(j-1)*m_width + i] + m_iVolume_32S[k*m_width*m_height + j*m_width + i - 1] - m_iVolume_32S[k*m_width*m_height + (j-1)*m_width + i - 1] + m_iVolume_32S[(k-1)*m_width*m_height + j*m_width + i] - m_iVolume_32S[(k-1)*m_width*m_height + (j-1)*m_width + i] - m_iVolume_32S[(k-1)*m_width*m_height + j*m_width + i-1] + m_iVolume_32S[(k-1)*m_width*m_height + (j-1)*m_width + i-1];
			}
		}
	}
}


void IntegralVolume::getCubeFromCenter ( int x, int y, int z, int radius, float &res){
	//lower vertexes
	int v1 = (z+radius)*m_width*m_height+(y+radius)*m_width + x+radius;
	int v2 = (z+radius)*m_width*m_height+(y+radius)*m_width + x-radius-1;
	int v3 = (z+radius)*m_width*m_height+(y-radius-1)*m_width + x+radius;
	int v4 =(z+radius)*m_width*m_height+(y-radius-1)*m_width + x-radius-1;
	//upper vertexes
	int v5 = (z-radius-1)*m_width*m_height+(y+radius)*m_width + x+radius;
	int v6 = (z-radius-1)*m_width*m_height+(y+radius)*m_width + x-radius-1;
	int v7 = (z-radius-1)*m_width*m_height+(y-radius-1)*m_width + x+radius;
	int v8 =(z-radius-1)*m_width*m_height+(y-radius-1)*m_width + x-radius-1;

	res = m_iVolume_64F[v1] - m_iVolume_64F[v2]-m_iVolume_64F[v3]+m_iVolume_64F[v4] - m_iVolume_64F[v5]+m_iVolume_64F[v6]+m_iVolume_64F[v7]-m_iVolume_64F[v8];
}

void IntegralVolume::getCubeFromCenter (int x, int y, int z, float &res){

	//lower vertexes
	int v1 = (z+m_radius)*m_width*m_height+(y+m_radius)*m_width + x+m_radius;
	int v2 = (z+m_radius)*m_width*m_height+(y+m_radius)*m_width + x-m_radius-1;
	int v3 = (z+m_radius)*m_width*m_height+(y-m_radius-1)*m_width + x+m_radius;
	int v4 =(z+m_radius)*m_width*m_height+(y-m_radius-1)*m_width + x-m_radius-1;
	//upper vertexes
	int v5 = (z-m_radius-1)*m_width*m_height+(y+m_radius)*m_width + x+m_radius;
	int v6 = (z-m_radius-1)*m_width*m_height+(y+m_radius)*m_width + x-m_radius-1;
	int v7 = (z-m_radius-1)*m_width*m_height+(y-m_radius-1)*m_width + x+m_radius;
	int v8 =(z-m_radius-1)*m_width*m_height+(y-m_radius-1)*m_width + x-m_radius-1;

	res = m_iVolume_64F[v1] - m_iVolume_64F[v2]-m_iVolume_64F[v3]+m_iVolume_64F[v4] - m_iVolume_64F[v5]+m_iVolume_64F[v6]+m_iVolume_64F[v7]-m_iVolume_64F[v8];

}


void IntegralVolume::getCubeFromCenter ( int x, int y, int z, int radius, int &res){

	//lower vertexes
	int v1 = (z+radius)*m_width*m_height+(y+radius)*m_width + x+radius;
	int v2 = (z+radius)*m_width*m_height+(y+radius)*m_width + x-radius-1;
	int v3 = (z+radius)*m_width*m_height+(y-radius-1)*m_width + x+radius;
	int v4 =(z+radius)*m_width*m_height+(y-radius-1)*m_width + x-radius-1;
	//upper vertexes
	int v5 = (z-radius-1)*m_width*m_height+(y+radius)*m_width + x+radius;
	int v6 = (z-radius-1)*m_width*m_height+(y+radius)*m_width + x-radius-1;
	int v7 = (z-radius-1)*m_width*m_height+(y-radius-1)*m_width + x+radius;
	int v8 =(z-radius-1)*m_width*m_height+(y-radius-1)*m_width + x-radius-1;

	res = m_iVolume_32S[v1] - m_iVolume_32S[v2]-m_iVolume_32S[v3]+m_iVolume_32S[v4] - m_iVolume_32S[v5]+m_iVolume_32S[v6]+m_iVolume_32S[v7]-m_iVolume_32S[v8];

}

void IntegralVolume::getCubeFromCenter (int x, int y, int z, int &res){

	//lower vertexes
	int v1 = (z+m_radius)*m_width*m_height+(y+m_radius)*m_width + x+m_radius;
	int v2 = (z+m_radius)*m_width*m_height+(y+m_radius)*m_width + x-m_radius-1;
	int v3 = (z+m_radius)*m_width*m_height+(y-m_radius-1)*m_width + x+m_radius;
	int v4 =(z+m_radius)*m_width*m_height+(y-m_radius-1)*m_width + x-m_radius-1;
	//upper vertexes
	int v5 = (z-m_radius-1)*m_width*m_height+(y+m_radius)*m_width + x+m_radius;
	int v6 = (z-m_radius-1)*m_width*m_height+(y+m_radius)*m_width + x-m_radius-1;
	int v7 = (z-m_radius-1)*m_width*m_height+(y-m_radius-1)*m_width + x+m_radius;
	int v8 =(z-m_radius-1)*m_width*m_height+(y-m_radius-1)*m_width + x-m_radius-1;

	res = m_iVolume_32S[v1] - m_iVolume_32S[v2]-m_iVolume_32S[v3]+m_iVolume_32S[v4] - m_iVolume_32S[v5]+m_iVolume_32S[v6]+m_iVolume_32S[v7]-m_iVolume_32S[v8];
}

void IntegralVolume::getRectangleFromCorner ( int x, int y, int z, int sidex, int sidey, int sidez, float &res){
	
	//lower vertexes
	int v1 = (z+sidez-1)*m_width*m_height+(y+sidey-1)*m_width + x+sidex-1;
	int v2 = (z+sidez-1)*m_width*m_height+(y+sidey-1)*m_width + x-1;
	int v3 = (z+sidez-1)*m_width*m_height+(y-1)*m_width + x+sidex-1;
	int v4 =(z+sidez-1)*m_width*m_height+(y-1)*m_width + x-1;
	//upper vertexes
	int v5 = (z-1)*m_width*m_height+(y+sidey-1)*m_width + x+sidex-1;
	int v6 = (z-1)*m_width*m_height+(y+sidey-1)*m_width + x-1;
	int v7 = (z-1)*m_width*m_height+(y-1)*m_width + x+sidex-1;
	int v8 =(z-1)*m_width*m_height+(y-1)*m_width + x-1;

	res = m_iVolume_64F[v1] - m_iVolume_64F[v2]-m_iVolume_64F[v3]+m_iVolume_64F[v4] - m_iVolume_64F[v5]+m_iVolume_64F[v6]+m_iVolume_64F[v7]-m_iVolume_64F[v8];
}

void IntegralVolume::getRectangleFromCorner ( int x, int y, int z, int sidex, int sidey, int sidez, int &res){

	//lower vertexes
	int v1 = (z+sidez-1)*m_width*m_height+(y+sidey-1)*m_width + x+sidex-1;
	int v2 = (z+sidez-1)*m_width*m_height+(y+sidey-1)*m_width + x-1;
	int v3 = (z+sidez-1)*m_width*m_height+(y-1)*m_width + x+sidex-1;
	int v4 =(z+sidez-1)*m_width*m_height+(y-1)*m_width + x-1;
	//upper vertexes
	int v5 = (z-1)*m_width*m_height+(y+sidey-1)*m_width + x+sidex-1;
	int v6 = (z-1)*m_width*m_height+(y+sidey-1)*m_width + x-1;
	int v7 = (z-1)*m_width*m_height+(y-1)*m_width + x+sidex-1;
	int v8 =(z-1)*m_width*m_height+(y-1)*m_width + x-1;

	res = m_iVolume_32S[v1] - m_iVolume_32S[v2]-m_iVolume_32S[v3]+m_iVolume_32S[v4]-m_iVolume_32S[v5]+m_iVolume_32S[v6]+m_iVolume_32S[v7]-m_iVolume_32S[v8];

}

IntegralVolume::~IntegralVolume(){

	if ( m_iVolume_64F != NULL)
		delete [] m_iVolume_64F;

	if ( m_iVolume_32S != NULL)
		delete [] m_iVolume_32S;

}
