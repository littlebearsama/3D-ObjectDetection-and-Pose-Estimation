#ifndef PCL_2_TOMGINE_H
#define PCL_2_TOMGINE_H

#include "v4r/TomGine/tgTomGineThread.h"

#undef Success

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace TomGine
{

  class tgPCL2TomGine
  {
  public:

    static tgModel
    convert (pcl::PolygonMesh &mesh);
  };

}

#endif
