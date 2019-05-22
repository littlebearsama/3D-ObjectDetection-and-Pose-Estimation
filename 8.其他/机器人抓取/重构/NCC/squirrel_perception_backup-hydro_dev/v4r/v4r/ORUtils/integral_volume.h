
	
// ����������������������������������������
// |  Thomas Ricci, Federico Tombari      |
// |  CVLab, University of Bologna, 2012  |
// ����������������������������������������


#ifndef INTEGRAL_VOLUME_H_
#define INTEGRAL_VOLUME_H_

#include <stdlib.h>
#include <pcl/pcl_exports.h>

#ifdef _MSC_VER
#ifdef FAAT_UTILS_EXPORTS
#define FAAT_UTILS_API __declspec(dllexport)
#else
#define FAAT_UTILS_API __declspec(dllimport)
#endif
#else
#define FAAT_UTILS_API
#endif

class FAAT_UTILS_API IntegralVolume{

protected:

	int m_radius;

	int m_width;
	int m_height;
	int m_depth;

	float *m_iVolume_64F;
	int *m_iVolume_32S;

public:

	IntegralVolume( float *dataIn, int width, int height, int depth );
	IntegralVolume( int *dataIn, int width, int height, int depth );
	~IntegralVolume ();

	void getCubeFromCenter ( int x, int y, int z, float &res);
	void getCubeFromCenter ( int x, int y, int z, int radius, float &res);

	void getCubeFromCenter ( int x, int y, int z, int &res);
	void getCubeFromCenter ( int x, int y, int z, int radius, int &res);

	void getRectangleFromCorner ( int x, int y, int z, int sidex, int sidey, int sidez, float &res);
	void getRectangleFromCorner ( int x, int y, int z, int sidex, int sidey, int sidez, int &res);

	void setRadius ( int radius){m_radius = radius;};
	int getRadius () { return m_radius; };

	int * getPointer() {
	  return m_iVolume_32S;
	};
};

#endif


