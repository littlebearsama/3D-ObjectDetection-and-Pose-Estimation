#include "pcl_visualization_utils.h"

namespace faat_pcl
{
  namespace utils
  {
      std::vector<int> visualization_framework (pcl::visualization::PCLVisualizer::Ptr vis, int number_of_views, int number_of_subwindows_per_view)
      {
        std::vector<int> viewportNr (number_of_views * number_of_subwindows_per_view, 0);

        for (int i = 0; i < number_of_views; i++)
        {
          for (int j = 0; j < number_of_subwindows_per_view; j++)
          {
            vis->createViewPort (float (i) / number_of_views, float (j) / number_of_subwindows_per_view, (float (i) + 1.0) / number_of_views,
                                 float (j + 1) / number_of_subwindows_per_view, viewportNr[number_of_subwindows_per_view * i + j]);

            vis->setBackgroundColor (float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                                     float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view,
                                     float (j * ((i % 2) / 10.0 + 1)) / number_of_subwindows_per_view, viewportNr[number_of_subwindows_per_view * i + j]);

            vis->removeAllShapes(viewportNr[i * number_of_subwindows_per_view + j]);
            std::stringstream window_id;
            window_id << "(" << i << ", " << j << ")";
            vis->addText (window_id.str (), 10, 10, window_id.str (), viewportNr[i * number_of_subwindows_per_view + j]);
          }
        }
        return viewportNr;
      }
  }
}
