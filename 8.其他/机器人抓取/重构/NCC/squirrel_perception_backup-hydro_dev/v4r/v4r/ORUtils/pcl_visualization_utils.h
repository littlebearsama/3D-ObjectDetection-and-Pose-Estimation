#ifndef PCL_VISUALIZATION_UTILS_H
#define PCL_VISUALIZATION_UTILS_H


#include <pcl/visualization/cloud_viewer.h>
#include <string>

namespace faat_pcl
{
  namespace utils
  {
      std::vector<int> visualization_framework (pcl::visualization::PCLVisualizer::Ptr vis, int number_of_views, int number_of_subwindows_per_view);
  }
}
#endif // PCL_VISUALIZATION_UTILS_H

