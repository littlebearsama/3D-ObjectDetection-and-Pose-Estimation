/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1040 Vienna, Austria
 *    potapova(at)acin.tuwien.ac.at
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

#include "utils.hpp"

namespace EPUtils
{

int readFiles(const std::string &directoryName, std::vector<std::string> &names)
{
  boost::filesystem::path directory(directoryName);
  // check if directory exists
  if(exists(directory))
  {
    // if not a directory, but regular file
    if(is_regular_file(directory))
    {
      names.push_back(directoryName);
      return(0);
    }
    // if normal directory
    if(is_directory(directory))
    {
      for(boost::filesystem::directory_iterator end, it(directory); it != end ; ++it)
      {
        if(is_regular_file(*it))
        {
          names.push_back(it->path().directory_string());
        }
      }
      std::sort(names.begin(),names.end());
      return(0);
    }
    return(1);
  }
  else
    return(2);
}

void readPolygons(std::vector<std::vector<cv::Point> > &polygons, std::string filename)
{
  int x,y;
  std::ifstream indata;

  indata.open(filename.c_str()); // opens the file
  if(!indata)
  {
    // file couldn't be opened
    printf("ERROR: file %s could not be opened.\n",filename.c_str());
    return;
  }

  polygons.clear();
  int n_polygons;
  indata >> n_polygons;
  
  if(n_polygons <= 0)
    return;
  
  polygons.resize(n_polygons);

  // keep reading until end-of-file
  for(int k = 0; k < n_polygons; ++k)
  {
    int n = 0;
    indata >> n;
    std::vector<cv::Point> pol;
    pol.clear();
    pol.reserve(n);
    for(int i = 0; i < n; ++i)
    {
      indata >> x >> y;
      pol.push_back(cv::Point(x,y));
    }
    polygons.at(k) = pol;
  }
  indata.close();
  return;
}

void writePolygons(std::vector<std::vector<cv::Point> > &polygons, std::string &str)
{
  std::ofstream outdata;
  
  outdata.open(str.c_str()); // opens the file
  if(!outdata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << str << " could not be opened." << std::endl;
    return;
  }
  
  outdata << polygons.size() << std::endl;
  
  // keep reading until end-of-file
  for(unsigned int k = 0; k < polygons.size(); ++k)
  {
    outdata << polygons.at(k).size() << " ";
    for(unsigned int i = 0; i < polygons.at(k).size(); ++i)
    {
      outdata << polygons.at(k).at(i).x << " " << polygons.at(k).at(i).y << " ";
    }
    outdata << std::endl;
  }
  outdata.close();
  return;
}

void readCenters(std::vector<cv::Point> &centers, std::string &str)
{
  int x,y;
  std::ifstream indata;

  // open file
  indata.open(str.c_str());
  if(!indata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << str << " could not be opened." << std::endl;
    return;
  }
  
  int n_points = 0;
  if(!indata.eof())
  {
    indata >> n_points;
  }
  centers.resize(n_points);
  
  for(int k = 0; k < n_points; ++k)
  {
    indata >> x >> y;
    centers.at(k) = cv::Point(x,y);
  }
  indata.close();
  return;
  
}

void readRectangles(std::vector<cv::Rect> &rectangles, std::string &str)
{
  int x1, y1, x2, y2;
  std::string line;
  std::ifstream indata;

  indata.open(str.c_str()); // opens the file
  if(!indata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << str << " could not be opened." << std::endl;
    return;
  }

  // keep reading until end-of-file
  while ( !indata.eof() )
  {
    // keep reading until end-of-file
    indata >> x1 >> y1 >> x2 >> y2 >> line;
    rectangles.push_back(cv::Rect(x1,y1,x2-x1,y2-y1));
  }
  indata.close();
  return;
}

void readAttentionPoints(std::vector<std::vector<cv::Point> > &attentionPoints, std::string &str)
{
  std::string line;
  std::ifstream indata;

  //attentionPoints.reserve(200);

  indata.open(str.c_str()); // opens the file
  if(!indata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << str << " could not be opened." << std::endl;
    return;
  }

  int n_points;
  indata >> n_points;
  attentionPoints.resize(n_points);

  //while ( !indata.eof() )
  for(int j = 0; j < n_points; ++j)
  {
    int n = 0;
    indata >> line >> n;
    //indata >> n;
    //std::cerr << line << std::endl;
    if(n >= 0)
    {
      std::vector<cv::Point> points;
      points.resize(n);

      for(int i = 0; i < n; ++i)
      {
        indata >> points.at(i).x >> points.at(i).y;
      }

      attentionPoints.at(j) = points;
    }
  }

  indata.close();
  return;
}

//revision
void readAttentionPoints(std::vector<cv::Point> &attentionPoints, std::string &str)
{
  std::ifstream indata;

  // open the file
  indata.open(str.c_str());
  if(!indata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << str << " could not be opened." << std::endl;
    return;
  }

  int n_points;
  indata >> n_points;
  attentionPoints.resize(n_points);

  for(int j = 0; j < n_points; ++j)
  {
    cv::Point point;
    indata >> point.x >> point.y;
    attentionPoints.at(j) = point;
  }

  indata.close();
  return;
}

void writeAttentionPoints(std::vector<cv::Point> attentionPoints, std::string &str)
{
  std::ofstream outdata;
  
  outdata.open(str.c_str()); // opens the file
  if(!outdata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << str << " could not be opened." << std::endl;
    return;
  }
  
  outdata << attentionPoints.size() << std::endl;
  
  // keep reading until end-of-file
  for(unsigned int i = 0; i < attentionPoints.size(); ++i)
  {
    outdata << attentionPoints.at(i).x << " " << attentionPoints.at(i).y << std::endl;
  }
  
  outdata.close();
  return;
}
//end revision

void readAttentionPointsAndContours(std::vector<cv::Point> &attentionPoints,
                                    std::vector<std::vector<cv::Point> > &contours, std::string &str)
{
  //std::cerr << str << std::endl;
  //std::string line;
  std::ifstream indata;

  //attentionPoints.reserve(50);

  indata.open(str.c_str()); // opens the file
  if(!indata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << str << " could not be opened." << std::endl;
    return;
  }

  int numberOfAttentionPoints;
  indata >> numberOfAttentionPoints;
  attentionPoints.resize(numberOfAttentionPoints);
  contours.resize(numberOfAttentionPoints);

  for(int i = 0; i < numberOfAttentionPoints; ++i)
  {
    indata >> attentionPoints.at(i).x;
    indata >> attentionPoints.at(i).y;

    int numberOfPointsInContour;
    indata >> numberOfPointsInContour;
    contours.at(i).resize(numberOfPointsInContour);

    for(int j = 0; j < numberOfPointsInContour; ++j)
    {
      indata >> contours.at(i).at(j).x;
      indata >> contours.at(i).at(j).y;
    }
  }

  indata.close();
  return;
}

/**
 * reads ground truth segmentation from the text file
 * */
//ep:begin revision 18-07-2014
void readAnnotationsFromFile(std::vector<std::vector<cv::Point> > &polygons, std::string filename)
{
  polygons.clear();
  readPolygons(polygons,filename);
}
//ep:end revision 18-07-2014

void saturation(cv::Mat &map, float max_value)
{
  for(int y = 0; y < map.rows; ++y)
  {
    for(int x = 0; x < map.cols; ++x)
    {
      if(map.at<float>(y,x) > max_value)
      {
	map.at<float>(y,x) = max_value;
      }
    }
  }
}

void calculatePolygonsCenters(std::vector<std::vector<cv::Point> > &polygons,
                              std::vector<cv::Point> &attentionPoints,
                              std::vector<int> &attentionPointsBelongTo)
{
  attentionPoints.clear();
  attentionPoints.resize(polygons.size());
  attentionPointsBelongTo.clear();
  attentionPointsBelongTo.resize(polygons.size());
  for(unsigned int i = 0; i < polygons.size(); ++i)
  {
    float x = 0;
    float y = 0;
    for(unsigned int j = 0; j < polygons.at(i).size(); ++j)
    {
      x += polygons.at(i).at(j).x;
      y += polygons.at(i).at(j).y;
    }
    x /= polygons.at(i).size();
    y /= polygons.at(i).size();
    attentionPoints.at(i).x = (int)x;
    attentionPoints.at(i).y = (int)y;
    attentionPointsBelongTo.at(i) = i+1;
  }
}

double PolygonArea(std::vector<cv::Point> &polygon)
{
  int i,j;
  double area = 0;

  int N = polygon.size();

  for( i=0; i<N; i++)
  {
    j = (i + 1) % N;
    area += polygon.at(i).x * (-polygon.at(j).y);
    area -= (-polygon.at(i).y) * polygon.at(j).x;
  }

  area /= 2;
  return(area < 0 ? -area : area);
}

void Centroid(std::vector<cv::Point> &polygon, cv::Point &center)
{
  int i,j;
  //double area = PolygonArea(polygon);
  //std::cerr << area << std::endl;

  int N = polygon.size();

  double center_x = 0;
  double center_y = 0;
  double area = 0;

  for( i=0; i<N; i++)
  {
    j = (i + 1) % N;
    double term_x = polygon.at(i).x + polygon.at(j).x;
    double term_y = polygon.at(i).y + polygon.at(j).y;
    double term_xy = polygon.at(i).x * polygon.at(j).y - polygon.at(i).y * polygon.at(j).x;
    center_x += term_x*term_xy;
    center_y += term_y*term_xy;
    area += term_xy;
  }

  area = area/2;
  center_x = center_x/(6*area);
  center_y = center_y/(6*area);
  //std::cerr << center_x << " " << center_y << std::endl;
  center.x = (center_x > 0 ? center_x : -center_x);
  center.y = (center_y > 0 ? center_y : -center_y);
}

void makeGaborFilter(cv::Mat &filter0, cv::Mat &filter90, float angle, float stddev, float elongation, int filterSize,
                     int filterPeriod)
{

  float major_stddev = stddev;
  float minor_stddev = major_stddev * elongation;
  //float max_stddev = std::max(major_stddev,minor_stddev);

  int sz = filterSize/2; 

  float PI = 3.14159;
  float rtDeg = PI / 180 * angle;

  float omega = 2 * PI / filterPeriod;
  float co = cos(rtDeg);
  float si = -sin(rtDeg);
  float major_sigq = 2 * major_stddev * major_stddev;
  float minor_sigq = 2 * minor_stddev * major_stddev;

  std::vector<float> vco;
  vco.resize(2*sz+1);
  std::vector<float> vsi;
  vsi.resize(2*sz+1);
  for(int i = -sz; i<=sz; ++i)
  {
    vco.at(i+sz) = i*co;
    vsi.at(i+sz) = i*si;
  }

  filter0 = cv::Mat_<float>::zeros(2*sz+1,2*sz+1);
  filter90 = cv::Mat_<float>::zeros(2*sz+1,2*sz+1);
  
  for(int i = -sz; i<=sz; ++i)
  {
    for(int j = -sz; j<=sz; ++j)
    {
      float temp1 = vco.at(i+sz) + vsi.at(j+sz);
      
      float major = temp1;
      float major2 = temp1 * temp1;
      
      float temp2 = vsi.at(i+sz) - vco.at(j+sz);
      float minor = temp2;
      float minor2 = minor * minor;
      
      float phase0 = exp(-major2/major_sigq - minor2/minor_sigq);
      
      float psi0 = 0*PI/180;
      filter0.at<float>(i+sz,j+sz) = cos(omega*major + psi0)*phase0;
      float psi90 = 90*PI/180;
      filter90.at<float>(i+sz,j+sz) = cos(omega*major + psi90)*phase0;
    }
  }
  
  cv::Scalar mean0 = cv::mean(filter0);
  cv::Scalar mean90 = cv::mean(filter90);
  
  filter0 = filter0 - mean0(0);
  filter90 = filter90 - mean90(0);
  
  cv::Mat temp;
  cv::multiply(filter0,filter0,temp);
  cv::Scalar sum0 = cv::sum(temp);
  cv::multiply(filter90,filter90,temp);
//  cv::Scalar sum90 = cv::sum(temp);
  
  filter0 = filter0 / sqrt(sum0(0));
  filter90 = filter90 / sqrt(sum0(0));
}

void makeGaborKernel2D(cv::Mat &kernel, float &max_sum, float theta, float bandwidth, float lambda, float sigma, float phi, 
                       float gamma)
{
  float PI = 3.14159;
  
  theta = PI*theta/180;
  
  float slratio = (1/PI) * sqrt( (log(2)/2) ) * ( (pow(2.0,bandwidth) + 1) / (pow(2.0,bandwidth) - 1) );
  //BANDWITH = 1 will result in the slratio = 0.56

  //printParameter(slratio,"slratio");
  
  if (sigma == 0)
    sigma = slratio * lambda;
  else if (lambda == 0)
    lambda = sigma / slratio;

  //printParameter(sigma,"sigma");
  //printParameter(lambda,"lambda");
  
  int n = 0;
  if ((gamma <= 1) && (gamma > 0))
    n = ceil(2.5*sigma/gamma);
  else
    n = ceil(2.5*sigma);
  
  //printParameter(gamma,"gamma");
  //printParameter(n,"n");

  kernel = cv::Mat_<float>::zeros(2*n+1,2*n+1);
  
  float co = cos(theta);
  float si = sin(theta);
  
  std::vector<float> vco;
  vco.resize(2*n+1);
  std::vector<float> vsi;
  vsi.resize(2*n+1);
  for(int i = -n; i<=n; ++i)
  {
    vco.at(i+n) = i*co;
    vsi.at(i+n) = i*si;
  }
  
  //xp =  x * cos(theta) + y * sin(theta);
  //yp = -x * sin(theta) + y * cos(theta);

  float gamma2 = gamma*gamma;
  float b = 1 / (2*sigma*sigma);
  float a = b / PI;
  float f = 2*PI/lambda;
  
  //printParameter(gamma2,"gamma2");
  //printParameter(b,"b");
  //printParameter(a,"a");
  //printParameter(f,"f");

  for(int i = -n; i<=n; ++i) //x
  {
    for(int j = -n; j<=n; ++j) //y
    {
      float xp =  vco.at(i+n) + vsi.at(j+n);
      float yp = -vsi.at(i+n) + vco.at(j+n);
      
      kernel.at<float>(j+n,i+n) = a*exp(-b*(xp*xp + gamma2*yp*yp))*cos(f*xp+phi);
    }
  }
  
  //printMat(kernel,"kernel");
  
  float pos = 0;
  float neg = 0;
  for(int i = -n; i <= n; ++i)
  {
    for(int j = -n; j <= n; ++j)
    {
      float value = kernel.at<float>(j+n,i+n);
      if(value > 0)
      {
	pos += value;
      }
      else if(value < 0)
      {
	neg += value;
      }
    }
  }
  
  //printParameter(pos,"pos");
  //printParameter(neg,"neg");
  
  float pos2 = 0;
  float neg2 = 0;
  
  float meansum = (pos - neg)/2;
  //printParameter(meansum,"meansum");
  if(meansum > 0)
  {
    pos = pos / meansum;
    neg = -neg / meansum;
    
    //printParameter(pos,"pos");
    //printParameter(neg,"neg");
    
    for(int i = -n; i <= n; ++i) //x
    {
      for(int j = -n; j <= n; ++j) //y
      {
        float value = kernel.at<float>(j+n,i+n);
	
	//printParameter(value,"value");
        
	if(value > 0)
        {
	  //printParameter(pos,"pos");
	  value *= neg;
	  pos2 += value;
        }
        else if(value < 0)
        {
	  //printParameter(neg,"neg");
	  value *= pos;
	  neg2 += value;
        }
        
        //printParameter(value,"value");
        
        kernel.at<float>(j+n,i+n) = value;
      }
    }
  }
  // print kernel
  //printMat(kernel,"kernel");
  
  //std::cerr << "pos2 = " << pos2 << std::endl;
  //std::cerr << "neg2 = " << neg2 << std::endl;
  
  max_sum = std::max(pos2,-neg2);
  
}

void saveDistribution(std::vector<float> dist, std::string filename)
{
  std::ofstream outdata;
  std::string filename2 = filename + "_.m";
  // open file
  outdata.open(filename2.c_str());
  if(!outdata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << filename2 << " could not be opened." << std::endl;
    return;
  }
  
  outdata << dist.size() << std::endl << filename << "=[";
  
  for(unsigned int i = 0; i < dist.size(); ++i)
  {
    outdata << dist.at(i) << std::endl;
  }
  outdata << "];" << std::endl;
  outdata.close();
}

void readDistribution(std::vector<float> &dist, std::string filename)
{
  std::ifstream indata;
  
  // open file
  indata.open(filename.c_str());
  if(!indata)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << filename << " could not be opened." << std::endl;
    return;
  }
  
  int n;
  std::string temp;
  indata >> n >> filename >> temp;
  dist.resize(n);
  
  for(unsigned int i = 0; i < dist.size(); ++i)
  {
    indata >> dist.at(i);
  }
  indata.close();
}

} //namespace EPUtils
