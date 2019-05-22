 // modified by Ekaterina Potapovas
 /* * * * * * * * * * * * * * * * * * * * * * * *
 *              Symmetry_map               	*
 *						*
 * Author:   Dominik Kohl			*
 * Email:    e0726126@sudent.tuwien.ac.at	*
 * Date:     15 June 2011 			*
 * Supervisor: Ekaterina Potapova		*
 *						*
 * Based on the Code from Gert Kootstra		*
 * and Niklas Bergstrom [1] and [2]		*
 * (http://www.csc.kth.se/~kootstra/) 		*
 *						*
 *						*
 * * * * * * * * * * * * * * * * * * * * * * * */


 /*
 * DESCRIPTION
 *
 * This is software for the bottom-up detection of unknown objects.
 * It is based on a symmetry-saliency map, as described in [1] and [2]
 *
 * Symmetry is an important Gestalt principle and can be used for
 * figure-ground segregation or to find the centerpoint of an object.
 *
 * The code works with ROS and OpenCV 2.1 or higher and IPP 7.0
 * To get the symmetry-map all usable OpenCV instructions are used
 * and with IPP for speed-optimisation.
 *
 *
 * REFERENCES
 *
 * [1] Kootstra, G., Bergstr&ouml;m, N., & Kragic, D. (2010). Using Symmetry to
 * Select Fixation Points for Segmentation. To be presented at the
 * International Conference on Pattern Recognition (ICPR), August 23-26,
 * 2010, Istanbul, Turkey.
 *
 * [2] Kootstra, G. & Schomaker, L.R.B. (2009) Using Symmetrical
 * Regions-of-Interest to Improve Visual SLAM. In: Proceedings of the
 * International Conference on Intelligent RObots and Systems (IROS),
 * pp. 930-935, Oct 11-15, 2009, St. Louis, USA. doi:
 * 10.1109/IROS.2009.5354402.
*/

#include "SymmetryMap.hpp"

namespace AttentionModule
{

SymmetryMap::SymmetryMap():
BaseMap()
{
  reset();
}

SymmetryMap::~SymmetryMap()
{
}

void SymmetryMap::reset()
{
  BaseMap::reset();
  
  R1 = 7;
  R2 = 17;
  S = 16;
  
  mapName = "SymmetryMap";
}

void SymmetryMap::print()
{
  BaseMap::print();
  printf("[%s]: R1                 = %d\n",mapName.c_str(),R1);
  printf("[%s]: R2                 = %d\n",mapName.c_str(),R1);
  printf("[%s]: S                  = %d\n",mapName.c_str(),R1);
}

void SymmetryMap::setR1(int R1_)
{
  R1 = R1_;
  calculated = false;
  printf("[INFO]: %s: R1: %d.\n",mapName.c_str(),R1);
}

void SymmetryMap::setR2(int R2_)
{
  R2 = R2_;
  calculated = false;
  printf("[INFO]: %s: R2: %d.\n",mapName.c_str(),R2);
}

void SymmetryMap::setS(int S_)
{
  S = S_;
  calculated = false;
  printf("[INFO]: %s: S: %d.\n",mapName.c_str(),S);
}

int SymmetryMap::getR1()
{
  return(R1);
}

int SymmetryMap::getR2()
{
  return(R2);
}

int SymmetryMap::getS()
{
  return(S);
}

int SymmetryMap::checkParameters()
{
  if(!haveImage)
  {
    printf("[ERROR]: %s: No image set.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if( (width == 0) || (height == 0) || (image.rows == 0) || (image.cols == 0) )
  {
    printf("[ERROR]: %s: Seems like image is empty.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if((image.cols != width) || (image.rows != height))
  {
    printf("[ERROR]: %s: Problem with image sizes.\n",mapName.c_str());
    return(AM_IMAGE);
  }

//   if(image.channels() != 3)
//   {
//     printf("[ERROR]: %s: Image should have 3 channels.\n",mapName.c_str());
//     return(AM_IMAGE);
//   }
  
  if(!haveMask)
    mask = cv::Mat_<uchar>::ones(height,width);
  
  if((mask.cols != width) || (mask.rows != height))
  {
    mask = cv::Mat_<uchar>::ones(height,width);
  }

  return(AM_OK);
}

void SymmetryMap::initialize()
{
  //LOG
  /*int logArSize = 10000;
  cv::Mat logAr = cv::Mat_<float>::zeros(logArSize,1);
  for(int i=0; i < logArSize; ++i)
  {
    logAr.at<float>(i,1) = log( 1 + sqrt(72)*(float)i/(logArSize-1) );
  }*/

  // Make a Gaussian distance weight array
  int lengthDW = 2 * ( (R2*2)*(R2*2) ) + 1;
  distanceWeight.resize(lengthDW);
  for(int i=0; i<lengthDW; i++)
  {
    distanceWeight.at(i) = (1/(S*sqrt(2*M_PI))) * exp( -i / (2*S*S));
  }

  //PixelAngels
  
  //remove if exist
  pixelAngles = cv::Mat_<float>::zeros(R2*4+1,R2*4+1);

  for(int y=-R2*2; y<R2*2+1; y++)
  {
    for(int x=-R2*2; x<R2*2+1; x++)
    {
      pixelAngles.at<float>(y+R2*2,x+R2*2) = atan2(y, x);
    }
  }
  
  // Make cos-function from -4pi - 4pi
  cosArSize = 10000;
  cosAr = cv::Mat_<float>::zeros(2*cosArSize+1,1);
  for(int i = -cosArSize; i < cosArSize+1; ++i)
  {
    cosAr.at<float>(i+cosArSize,1) = cos(4*M_PI*(float)i/cosArSize);
  }

  // END of LookUp-Tabel Calculation
}

int SymmetryMap::calculate()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);

  printf("[INFO]: %s: Computation started.\n",mapName.c_str());
  
  initialize();
  
  cv::Mat image_cur;
  if(image.channels() > 1)
    cv::cvtColor(image,image_cur,CV_RGB2GRAY);
  else
    image.copyTo(image_cur);
  
  symmetryMap(image_cur,width,height,R1,R2,S,map);

  EPUtils::normalize(map,normalization_type);

  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());
  return(AM_OK);
}


int SymmetryMap::calculatePyramidSimple()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Simple pyramid started.\n",mapName.c_str());
  
  initialize();
  
  SimplePyramid::Ptr pyramid( new SimplePyramid() );
  
  pyramid->setStartLevel(1);
  pyramid->setMaxLevel(5);
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  
  //insert appropriate
  cv::Mat image_cur;
  if(image.channels() > 1)
    cv::cvtColor(image,image_cur,CV_RGB2GRAY);
  else
    image.copyTo(image_cur);
  
  pyramid->setImage(image_cur);
  pyramid->buildPyramid();
  pyramid->print();

  combinePyramid(pyramid);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int SymmetryMap::combinePyramid(BasePyramid::Ptr pyramid)
{
  for(unsigned int i = pyramid->getStartLevel(); i <= (unsigned int)pyramid->getMaxLevel(); ++i)
  {
    printf("[INFO]: %s: Computating feature map for level %d.\n",mapName.c_str(),i);

    // start creating parameters
    cv::Mat current_image;
    if(!pyramid->getImage(i,current_image))
    {
      printf("[ERROR]: Something went wrong! Can't get image for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    int current_width = pyramid->getWidth(i);
    if(current_width <= 0)
    {
      printf("[ERROR]: Something went wrong! Can't get width for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    int current_height = pyramid->getHeight(i);
    if(current_height <= 0)
    {
      printf("[ERROR]: Something went wrong! Can't get height for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    cv::Mat current_map;
    symmetryMap(current_image,current_width,current_height,R1,R2,S,current_map);
    
    if(!pyramid->setFeatureMap(i,current_map))
    {
     printf("[ERROR]: Something went wrong! Can't set feature map for level %d!\n",i);
     return(AM_CUSTOM);
    }
    
    printf("[INFO]: %s: Feature map at level %d is set.\n",mapName.c_str(),i);
  }
  // combine saliency maps
  pyramid->combinePyramid(true);
  
  if(!pyramid->getMap(map))
  {
    printf("[ERROR]: Something went wrong! Can't get saliency map from the pyramid!\n");
    return(AM_CUSTOM);
  }
  return(AM_OK);
}

void SymmetryMap::symmetryMap(cv::Mat &image_cur, int image_width, int image_height, int R1, int R2, int S, cv::Mat &map_cur)
{

  //Variables for calculation of the symmetry_pixel
  int x0, y0, x1, y1, d;
  int dy, dy1, dy2, dx, dx1, dx2;
  float dX, dY, symV;
  float angle, g0, g1, gwf;
  float totalSym=0;

  //Calculate the symmetry for all used scales
  cv::Mat image_X;
  cv::Mat image_Y;

  cv::Sobel(image_cur,image_X,CV_32F,1,0);
  cv::Sobel(image_cur,image_Y,CV_32F,0,1);

  // Calculate the Angle and Magnitude for every Pixel with cartToPolar
  cv::Mat image_magnitude, image_angle;
  cv::cartToPolar(image_X,image_Y,image_magnitude,image_angle,false);

  map_cur = cv::Mat_<float>::zeros(image_height,image_width);

  for(int y = 0; y < image_height; ++y)  // START of Iteration over all Pixel ***************************
  {
    for(int x = 0; x < image_width; ++x)
    {
      // Excluding the borders, since the gradients there are not vaild
      dy1 = std::max(R2 - y +1, 1);
      dy2 = std::max(y + R2 + 1 - image_height + 1, 1);
      dy  = std::max(dy1, dy2);

      dx1 = std::max(R2 - x +1 ,1);
      dx2 = std::max(x + R2 + 1 - image_width + 1, 1);
      dx  = std::max(dx1, dx2);

      // Reset for next Iteration
      symV = 0;
      totalSym = 0;


      // ------------------------
      // |                      |
      // |                      |
      // |      *********       |
      // |      *       *       |
      // |      *       * R1    | R2
      // |      *       *       |
      // |      *********       |
      // |                      |
      // |                      |
      // ------------------------

      for(int j=dy; j < (R2+1); j++)  // Start Iteration over the Mask (Boundary Box) --------------------------------------------
      {
        for(int i = dx; i < (R2*2+1 - dx); i++)
        {
          if((j >= R2) && (i >= R2))  // When at the center of the mask, break
            break;

          if( !((j>(R2-R1)) && (j<(R2+R1)) && (i>(R2-R1)) && (i<(R2+R1))) )
          {
            x0 = x - R2 + i;
            y0 = y - R2 + j;
            x1 = x + R2 - i;
            y1 = y + R2 - j;
            dX = x1 - x0;
            dY = y1 - y0;
            d = (int)rint(dX*dX+dY*dY);  // L2 distance
            // Get the angle of the line between the two pixels use LookUp-table
            angle = pixelAngles.at<float>((y1-y0)+R2*2,(x1-x0)+R2*2);

            // Subtract the angle between the two pixels from the gradient angles to get the normalized angles
            g0 = image_angle.at<float>(y0,x0) - angle;
            g1 = image_angle.at<float>(y1,x1) - angle;

            // Calculate the strength of both gradient magnitudes

            //Use normal logarithmus
            gwf = log( ( 1+image_magnitude.at<float>(y0,x0) ) * ( 1+image_magnitude.at<float>(y1,x1) ) );
            //Use LookUp-Table
            symV = (1 - cosAr.at<float>((int)(cosArSize*(g0 + g1)/(4*M_PI))  + cosArSize,1)) *
                   (1 - cosAr.at<float>((int)(cosArSize*(g0 - g1)/(4*M_PI))  + cosArSize,1)) * gwf * distanceWeight.at(d);

            totalSym += symV;   //Add to the center Point
            // END Iteration over the Mask ------------------------------------------------------------
         }
       }
     }
     //Save the symmetry information of this pixel in the correct scale image
     map_cur.at<float>(y,x) = totalSym;
     // END of Iteration over all Pixel **************************************************************************************
   }
 }
 
 return;
}   

} //namespace AttentionModule