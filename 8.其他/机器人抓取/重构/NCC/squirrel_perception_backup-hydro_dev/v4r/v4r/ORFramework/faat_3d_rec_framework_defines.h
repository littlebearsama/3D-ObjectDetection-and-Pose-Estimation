#include <pcl/common/common.h>

#ifndef FAAT_3D_REC_FRAMEWORK_DEFINES_H
#define FAAT_3D_REC_FRAMEWORK_DEFINES_H

#ifdef _MSC_VER
#ifdef FAAT_3D_FRAMEWORK_EXPORTS
#define FAAT_3D_FRAMEWORK_API __declspec(dllexport)
#else
#define FAAT_3D_FRAMEWORK_API __declspec(dllimport)
#endif
#else
#define FAAT_3D_FRAMEWORK_API
#endif

struct IndexPoint
{
  int idx;
};

//This stuff is needed to be able to make the SHOT histograms persistent
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<352>,
    (float[352], histogram, histogram352)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<1344>,
    (float[1344], histogram, histogram1344)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint,
    (int, idx, idx)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<128>,
    (float[128], histogram, histogramSIFT)
)

namespace faat_pcl
{
    namespace rec_3d_framework
    {
        enum FeatureType
        {
            SIFT = 0x01, // 00000001
            ESF = 0x02, // 00000010
            SHOT  = 0x04, // 00000100
            OURCVFH  = 0x08,  // 00001000
            FPFH = 0x10  // 00010000
        };
    }
}

#endif
