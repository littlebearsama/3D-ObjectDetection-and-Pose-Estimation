/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file ColorHistogram.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#ifndef SURFACE_COLOR_HISTOGRAM_HH
#define SURFACE_COLOR_HISTOGRAM_HH

#include <vector>
#include <opencv2/opencv.hpp>
#include "v4r/PCLAddOns/PCLUtils.h"

//@ep: TODO: this should be inheritance and not case clause

namespace surface
{

/**
 * @brief Class ColorHistogram
 */
class ColorHistogram
{

public:
  typedef boost::shared_ptr<ColorHistogram> Ptr;
  
  struct Color3C
  {
    int ch1; // R/B/Y
    int ch2; // G/G/U
    int ch3; // B/R/V
  };
  
  enum ColorModel {
    YUV_MODEL = 0,
    RGB_MODEL,
    BGR_MODEL,
  };

  enum HistogramType {
    HIST_AVERAGE_HIST = 0,
    HIST_2D = 1,
    HIST_3D = 2,
  };
  
private:
  int nrBins;
  int colorModel;                          /// 0 ... yuv / 1 ... rgb 
  int histogramType;
  double maxVal;
  
  double UVthreshold;
  bool useUVthreshold;
  
  int width, height;
  
  bool computed;
  bool have_input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  bool have_indices;
  pcl::PointIndices::Ptr indices;
  
  cv::Mat_<double> hist;

  bool init();
  
  bool buildHistogram();
  bool buildHistogramAverage();
  bool buildHistogram2D();
  bool buildHistogram3D();

  void getYUV(pclA::RGBValue &color, Color3C &convColor);
  void getRGB(pclA::RGBValue &color, Color3C &convColor);
  bool getColor(pclA::RGBValue color, Color3C &convColor);

  double compareAverage(ColorHistogram::Ptr ch);
  double compare2D(ColorHistogram::Ptr ch);
  double compare3D(ColorHistogram::Ptr ch);

  void printHistogramAverage();
  void printHistogram2D();
  void printHistogram3D();
  
public:
  
  ColorHistogram(int _nr_bins, double _UVthreshold = 0.);
  // sets input cloud
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
  // sets indices
  void setIndices(pcl::PointIndices::Ptr _indices);
  void setIndices(std::vector<int> &_indices);
  void setIndices(cv::Rect rect);
  
  void setColorModel(int _colorModel) {colorModel = _colorModel;};
  void setHistogramType(int _histogramType) {histogramType = _histogramType;};
  void setBinNumber(int _nrBins) {nrBins = _nrBins;};
  void setUseUVThreshold(bool _useUVthreshold) {useUVthreshold = _useUVthreshold;};
  void setUVThreshold(double _UVthreshold) {UVthreshold = _UVthreshold;};
  void setMaxVal(double _maxVal) {maxVal = _maxVal;}
  
  int getBinNumber() {return nrBins;};
  int getColorModel() {return colorModel;};
  int getHistogramType() {return histogramType;};
  bool getUseUVThreshold() {return useUVthreshold;};
  double getUVThreshold() {return UVthreshold;};
  double getMaxVal() {return maxVal;};
  bool getComputed() {return computed;};
  const cv::Mat getHist() const {return hist;};
  
  void compute();
  
  double compare(ColorHistogram::Ptr ch);
  
  void printHistogram();
};

}

#endif

