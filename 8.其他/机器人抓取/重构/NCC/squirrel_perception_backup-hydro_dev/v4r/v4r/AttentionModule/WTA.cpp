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

/**
 * @brief this implementation was inspired by http://www.saliencytoolbox.net
 */

#include "WTA.hpp"

namespace AttentionModule
{

void defaultLeakyIntFire(LIF& lif)
{
  lif.timeStep = 0.0001;                  // seconds
  lif.Eleak = 0;                          // Volts
  lif.Eexc = 100e-3;                      // Volts
  lif.Einh = -20e-3;                      // Volts
  lif.Gleak = 1e-8;                       // Siemens
  lif.Gexc = 0;                           // Siemens
  lif.Ginh = cv::Mat_<float>::zeros(1,1); // Siemens
  lif.GinhDecay = 1;                      // Siemens
  lif.Ginput = 5e-8;                      // Siemens
  lif.Vthresh = 0.001;                    // Volts
  lif.C = 1e-9;                           // Farad
  lif.time = 0;                           // seconds
  lif.V = cv::Mat_<float>::zeros(1,1);    // Volts
  lif.I = cv::Mat_<float>::zeros(1,1);    // Ampere
  lif.DoesFire = 1;                       // binary
}

void defaultParams(Params& params)
{
  params.IORdecay = 0.9999;//
  params.smOutputRange = 1.0000e-009;//
  params.noiseAmpl = 1.0000e-017;//
  params.noiseConst = 1.0000e-014;//
  params.useRandom = true;//
  params.foaSize = 64;//64;
  params.mapLevel = 5;//
  params.useCentering = false;
  params.useMorphologyOpenning = false;
}

void initializeWTA(WTA& wta, const cv::Mat& salmap, Params& salParams)
{
  defaultLeakyIntFire(wta.sm);
  wta.sm.C = 5e-8;
  wta.sm.Einh = 0;
  wta.sm.Eexc = 0;
  wta.sm.Gleak = 1e-7;
  wta.sm.Ginh = cv::Mat_<float>::zeros(salmap.rows,salmap.cols);
  wta.sm.GinhDecay = salParams.IORdecay;
  wta.sm.DoesFire = 0;
  wta.sm.I = cv::Mat_<float>::zeros(salmap.rows,salmap.cols);
  srand(time(NULL));
  for(int i = 0; i < salmap.rows; i++)
  {
    for(int j = 0; j < salmap.cols; j++)
    {
      wta.sm.I.at<float>(i,j) = salmap.at<float>(i,j)*salParams.smOutputRange +
                                 salParams.noiseAmpl*(((double)rand())/RAND_MAX) +
                                 salParams.noiseConst;
    }
  }
  wta.sm.V = cv::Mat_<float>::zeros(salmap.rows,salmap.cols);

  defaultLeakyIntFire(wta.exc);
  wta.exc.I = cv::Mat_<float>::zeros(salmap.rows,salmap.cols);
  wta.exc.V = cv::Mat_<float>::zeros(salmap.rows,salmap.cols);
  wta.exc.Ginh = cv::Mat_<float>::zeros(1,1);
  wta.exc.Ginh.at<float>(0,0) = 1e-2;

  defaultLeakyIntFire(wta.inhib);
}

void evolveLeakyIntFire(LIF& lif, float t, cv::Mat& spikes)
{
  float dt = t - lif.time;
  float ginh;

  //integrate
  for(int i = 0; i < lif.V.rows; i++)
  {
    for(int j = 0; j < lif.V.cols; j++)
    {
      if((lif.Ginh.rows == 1) && (lif.Ginh.cols == 1))
      {
        ginh = lif.Ginh.at<float>(0,0);
      }
      else
      {
        ginh = lif.Ginh.at<float>(i,j);
      }
      lif.V.at<float>(i,j) = lif.V.at<float>(i,j) + dt/lif.C * (lif.I.at<float>(i,j) - lif.Gleak*(lif.V.at<float>(i,j) - lif.Eleak) -
                                                                lif.Gexc*(lif.V.at<float>(i,j) - lif.Eexc) -
                                                                ginh*(lif.V.at<float>(i,j) - lif.Einh));
    }
  }

  //clamp potentials that are lower than Einh
  for(int i = 0; i < lif.V.rows; i++)
  {
    for(int j = 0; j < lif.V.cols; j++)
    {
      if(lif.V.at<float>(i,j) < lif.Einh)
      {
        lif.V.at<float>(i,j) = lif.Einh;
      }

      //spikes.at<float>(i,j) = 0;
      //if((lif.V.at<float>(i,j) > lif.Vthresh) && (lif.DoesFire))
      //{
      //  spikes.at<float>(i,j) = 1;
      //  lif.V.at<float>(i,j) = 0;
      //}
    }
  }

  //let Ginh decay (for IOR to wear off)
  for(int i = 0; i < lif.Ginh.rows; i++)
  {
    for(int j = 0; j < lif.Ginh.cols; j++)
    {
      lif.Ginh.at<float>(i,j) = lif.Ginh.at<float>(i,j) * lif.GinhDecay;
    }
  }

  // reset units that have just fired
  spikes = cv::Mat_<float>::zeros(lif.V.rows,lif.V.cols);
  for(int i = 0; i < lif.V.rows; i++)
  {
    for(int j = 0; j < lif.V.cols; j++)
    {
      //spikes.at<float>(i,j) = 0;
      if((lif.V.at<float>(i,j) > lif.Vthresh) && (lif.DoesFire))
      {
        spikes.at<float>(i,j) = 1;
        lif.V.at<float>(i,j) = 0;
      }
    }
  }

  lif.time = t;
}

void evolveWTA(WTA& wta, cv::Point& winner)
{
  float time = wta.exc.time + wta.exc.timeStep;
  winner.x = -1;
  winner.y = -1;

  // first evolve the sm
  cv::Mat spikes = cv::Mat_<float>::zeros(wta.sm.V.rows,wta.sm.V.cols);
  evolveLeakyIntFire(wta.sm,time,spikes);

  // set the input into the excitatory WTA neurons to the output of the sm
  for(int i = 0; i < wta.exc.I.rows; i++)
  {
    for(int j = 0; j < wta.exc.I.cols; j++)
    {
      wta.exc.I.at<float>(i,j) = wta.sm.V.at<float>(i,j) * wta.exc.Ginput;
    }
  }

  //evolve the excitatory neurons of the WTA network
  spikes = cv::Mat_<float>::zeros(wta.exc.V.rows,wta.exc.V.cols);
  evolveLeakyIntFire(wta.exc,time,spikes);

  //erase any inhibitions we might have had
  //for(int i = 0; i < wta.exc.Ginh.rows; ++i)
  //{
  //  for(int j = 0; j < wta.exc.Ginh.cols; ++j)
  //  {
  //    wta.exc.Ginh.at<float>(i,j) = 0;
  //  }
  //}
  wta.exc.Ginh.at<float>(0,0) = 0;

  // did anyone fire?
  bool done = false;
  for(int j = 0; j < spikes.cols; j++)
  {
    if(done)
      break;

    for(int i = 0; i < spikes.rows; i++)
    {
      if(done)
	break;

      if(spikes.at<float>(i,j) > 0)
      {
        winner.x = j;
        winner.y = i;
        wta.inhib.Gexc = wta.inhib.Gleak * 10;
	done = true;

        /*int ttt = 0;
        for(int i = 0; i < spikes.rows; ++i)
        {
            for(int j = 0; j < spikes.cols; ++j)
            {
                if(spikes.at<float>(i,j) > 0)
                {
                    ttt++;
                }
            }
        }
        std::cerr << ttt << std::endl;*/
        //cv::Mat temp;
        //spikes.copyTo(temp);
        //cv::resize(spikes,temp,cv::Size(spikes.cols*10,spikes.rows*10));
        //showImgaeForDebug("spikes",temp);
        //cv::waitKey();
      }
    }
  }

  // evolve the inhibitory interneuron
  spikes = cv::Mat_<float>::zeros(wta.inhib.V.rows,wta.inhib.V.cols);
  evolveLeakyIntFire(wta.inhib,time,spikes);
  if (spikes.at<float>(0,0))
  {
    // trigger global inhibition
    wta.exc.Ginh = 0.01;
    // no need to be excited anymore
    wta.inhib.Gexc = 0;
  }
}

bool fastSegmentMap(cv::Mat& resultMap, cv::Mat& map, cv::Point& seedPoint)
{
  float thresh = 0.50;
  float eps = 255*0.001;

  float seedVal = map.at<float>(seedPoint.y,seedPoint.x);
  resultMap = cv::Mat_<uchar>::zeros(map.rows,map.cols);

  if(seedVal < eps)
    return(false);

  cv::Mat bw = cv::Mat_<uchar>::zeros(map.rows,map.cols);
  for(int i = 0; i < bw.rows; ++i)
  {
    for(int j = 0; j < bw.cols; ++j)
    {
      if((map.at<float>(i,j)/seedVal) > thresh)
        bw.at<uchar>(i,j) = 1;
    }
  }
  std::cerr << "fastSegmentMap - 1" << std::endl;
  resultMap = cv::Mat_<uchar>::zeros(bw.rows,bw.cols);
  std::vector<cv::Point> points;
  points.resize(bw.rows * bw.cols);
  std::cerr << "fastSegmentMap - 2" << std::endl;
  long int ii = -1;
  if(bw.at<uchar>(seedPoint.y,seedPoint.x))
  {
    //points.push_back(seedPoint);
    ii++;
    points[ii] = seedPoint;
    bw.at<uchar>(seedPoint.y,seedPoint.x) = 0;
    resultMap.at<uchar>(seedPoint.y,seedPoint.x) = 255;
  }
  std::cerr << "fastSegmentMap - 3" << std::endl;
  while(ii>=0 /*points.size()*/)
  {
    cv::Point p = points.at(ii/*points.size()-1*/);
    ii--;
    //points.pop_back();
    //resultMap.at<float>(p.y,p.x) = 1;
    //bw.at<float>(p.y,p.x) = 0;
    if((p.y - 1) >= 0)
    {
      if(bw.at<uchar>(p.y-1,p.x))
      {
        cv::Point pp(p.x,p.y-1);
	//points.push_back(pp);
	ii++;
	points.at(ii) = pp;
	bw.at<uchar>(p.y-1,p.x) = 0;
	resultMap.at<uchar>(p.y-1,p.x) = 255;
      }
    }
    if((p.y + 1) < resultMap.rows)
    {
      if(bw.at<uchar>(p.y+1,p.x))
      {
        cv::Point pp(p.x,p.y+1);
	//points.push_back(pp);
	ii++;
	points.at(ii) = pp;
	bw.at<uchar>(p.y+1,p.x) = 0;
	resultMap.at<uchar>(p.y+1,p.x) = 255;
      }
    }
    if((p.x - 1) >= 0)
    {
      if(bw.at<uchar>(p.y,p.x-1))
      {
        cv::Point pp(p.x-1,p.y);
	//points.push_back(pp);
	ii++;
	points.at(ii) = pp;
	bw.at<uchar>(p.y,p.x-1) = 0;
	resultMap.at<uchar>(p.y,p.x-1) = 255;
      }
    }
    if((p.x + 1) < resultMap.cols)
    {
      if(bw.at<uchar>(p.y,p.x+1))
      {
        cv::Point pp(p.x+1,p.y);
	//points.push_back(cv::Point(p.x+1,p.y));
	ii++;
	points.at(ii) = pp;
	bw.at<uchar>(p.y,p.x+1) = 0;
	resultMap.at<uchar>(p.y,p.x+1) = 255;
      }
    }
  }
  return(true);
}

bool estimateShape(cv::Mat& binMap, cv::Mat& segmentedMap, cv::Mat& shapeMap, cv::Mat& salmap, cv::Point& winner, Params& params, cv::Mat &image)
{
  if(!fastSegmentMap(binMap,salmap,winner))
    return(false);
  std::cerr << "estimateShape - 1" << std::endl;

  // check that we actually segmented something, but not too big (< 10%)
  cv::Scalar Number = cv::sum(binMap); // sum
  cv::imshow("slkdjf",binMap);
  std::cerr << "sum = " << Number[0] << std::endl;
  float areaRatio = ((Number[0]) / (binMap.rows * binMap.cols));

  if ((areaRatio > 0) & (areaRatio < 0.1))
  {
    // this guy looks good - let's keep him!

    // for the IOR mask, we don't want to smooth the shape
    cv::Mat iorMask;
    //binMap.copyTo(iorMask);
    //se = [[0 0 1 0 0];[0 1 1 1 0];[1 1 1 1 1];[0 1 1 1 0];[0 0 1 0 0]];
    std::cerr << "estimateShape - 2" << std::endl;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5));
    std::cerr << "estimateShape - 3" << std::endl;
    cv::dilate(binMap,iorMask,kernel);
    std::cerr << "estimateShape - 4" << std::endl;
    // for the binary map, erode the shape a bit
    cv::Mat tmp, tmp2;
    //binMap.copyTo(tmp);
    bool done = false;
    cv::morphologyEx(binMap,tmp2,cv::MORPH_OPEN,kernel);
    std::cerr << "estimateShape - 5" << std::endl;
    cv::morphologyEx(tmp2,tmp,cv::MORPH_CLOSE,kernel);
    std::cerr << "estimateShape - 6" << std::endl;
    cv::Mat newMap;
    if (tmp.at<uchar>(winner.y,winner.x) > 0)
    {
      cv::Scalar tmp_sum = cv::sum(tmp);
      if (tmp_sum[0] > 0)
      {
        tmp.copyTo(newMap);
        done = true;
      }
    }
    else
    {
      //se = [[0 1 0];[1 1 1];[0 1 0]];
      kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3));
      cv::morphologyEx(binMap,tmp2,cv::MORPH_OPEN,kernel);
      cv::morphologyEx(tmp2,tmp,cv::MORPH_CLOSE,kernel);
      cv::Scalar tmp_sum = cv::sum(tmp);
      if ((tmp.at<uchar>(winner.y,winner.x) > 0) && (tmp_sum[0] > 0))
      {
        tmp.copyTo(newMap);
        done = true;
      }
    }
    if(done)
    {
      binMap = cv::Mat_<uchar>::zeros(newMap.rows,newMap.cols);
      std::vector<cv::Point> points;
      points.resize(binMap.rows * binMap.cols);
      long int ii = 0;
      //points.push_back(winner);
      points.at(ii) = winner;
      while(ii>=0/*points.size()*/)
      {
        cv::Point p = points[ii/*points.size()-1*/];
	ii--;
        //points.pop_back();
        binMap.at<uchar>(p.y,p.x) = 1;
        if((p.y - 1) >= 0)
        {
          if(newMap.at<uchar>(p.y-1,p.x) && !binMap.at<uchar>(p.y-1,p.x))
	  {
	    //points.push_back(cv::Point(p.x,p.y-1));
	    cv::Point pp(p.y-1,p.x);
	    ii++;
	    points.at(ii) = pp;
	  }
        }
        if((p.y + 1) < binMap.rows)
        {
          if(newMap.at<uchar>(p.y+1,p.x) && !binMap.at<uchar>(p.y+1,p.x))
	  {
	    //points.push_back(cv::Point(p.x,p.y+1));
	    cv::Point pp(p.y+1,p.x);
	    ii++;
	    points.at(ii) = pp;
	  }
        }
        if((p.x - 1) >= 0)
        {
          if(newMap.at<uchar>(p.y,p.x-1) && !binMap.at<uchar>(p.y,p.x-1))
	  {
	    //points.push_back(cv::Point(p.x-1,p.y));
	    cv::Point pp(p.y,p.x-1);
	    ii++;
	    points.at(ii) = pp;
	  }
        }
        if((p.x + 1) < binMap.cols)
        {
          if(newMap.at<uchar>(p.y,p.x+1) && !binMap.at<uchar>(p.y,p.x+1))
	  {
	    //points.push_back(cv::Point(p.x+1,p.y));
	    cv::Point pp(p.y,p.x+1);
	    ii++;
	    points.at(ii) = pp;
	  }
        }
      }
    }
    else
    {
      return(false);
    }
  }
  else
  {
    return(false);
  }

  // The segmented map is just winning map * binary map
  segmentedMap = cv::Mat_<float>::zeros(salmap.rows,salmap.cols);
  for(int i = 0; i < salmap.rows; ++i)
  {
    for(int j = 0; j < salmap.cols; ++j)
    {
      segmentedMap.at<float>(i,j) = salmap.at<float>(i,j) * binMap.at<float>(i,j);
    }
  }

  // The shape map is a smoothed version of the binary map
  cv::resize(binMap,shapeMap,cv::Size(image.cols,image.rows),0,0,cv::INTER_NEAREST);
  cv::GaussianBlur(shapeMap,shapeMap,cv::Size(15,15),0);

  return(true);
}

void applyIOR(WTA& wta, cv::Point& winner, Params& params)
{
  diskIOR(wta,winner,params);
}

void diskIOR(WTA& wta, cv::Point& winner, Params& params)
{
  int x = winner.x;
  int y = winner.y;
  float pampl = 0.1 * wta.sm.V.at<float>(y,x);
  float mampl = 0.0001 * pampl;

  // foaSize is interpreted as radius here, not as diameter
  float psdev = 0.3 * params.foaSize / (pow(2.0,params.mapLevel));

  float msdev = 4.0 * psdev;
  float d;
  //cv::Mat g = cv::Mat_<float>::zeros(wta.sm.V.rows,wta.sm.V.cols);
  float g;
  for(int i = 0; i < wta.sm.Ginh.rows; ++i)
  {
    for(int j = 0; j < wta.sm.Ginh.cols; ++j)
    {
      d = (i-y)*(i-y) + (j-x)*(j-x);
      //g.at<float>(i,j) = pampl * exp(-0.5 * d / (psdev*psdev)) - mampl * exp(-0.5 * d / (msdev*msdev));
      g = pampl * exp(-0.5 * d / (psdev*psdev)) - mampl * exp(-0.5 * d / (msdev*msdev));
      wta.sm.Ginh.at<float>(i,j) = wta.sm.Ginh.at<float>(i,j) + g;
    }
  }

  //wta.sm.Ginh = wta.sm.Ginh + g;
}

/*void shapeIOR(WTA& wta, cv::Point& winner, Params& params, cv::Mat& binaryMap, cv::Mat& binMap)
{
  float minVal = binaryMap.at<float>(0,0);
  float maxVal = minVal;
  //ROS_ERROR("Step 0");
  //cv::minMaxLoc(binaryMap,&minVal,&maxVal);
  for(int i = 0; i < binaryMap.rows; ++i)
  {
    for(int j = 0; j < binaryMap.cols; ++j)
    {
      if(binaryMap.at<float>(i,j) > maxVal)
        maxVal = binaryMap.at<float>(i,j);
      if(binaryMap.at<float>(i,j) < minVal)
        minVal = binaryMap.at<float>(i,j);
    }
  }
  //ROS_ERROR("Step 00");
  if (maxVal == 0)
  {
    diskIOR(wta,winner,params);
    return;
  }
  //ROS_ERROR("Step 01");
  int x = winner.x;
  int y = winner.y;
  float ampl = 0.1 * wta.sm.V.at<float>(y,x);
  //ROS_ERROR("Step 02");
  //cv::resize(binaryMap,binMap,wta.sm.V.size());
  binMap = cv::Mat_<double>::zeros(binaryMap.rows,binaryMap.cols);
  binaryMap.copyTo(binMap);
  //ROS_ERROR("Step 03");
  wta.sm.Ginh = wta.sm.Ginh + ampl * binMap;
  //ROS_ERROR("Step 03");
}*/

void winnerToImgCoords(cv::Point& win2, cv::Point& winner, Params& params, cv::Mat& img, const cv::Mat& _salmap)
{
  srand ( time(NULL) );
  float x = winner.x;
  float y = winner.y;
  if(params.useRandom)
  {
    x = x + (((float)rand())) / RAND_MAX;
    y = y + (((float)rand())) / RAND_MAX;
  }
  win2.x = (int)(x * (pow(2,params.mapLevel-1)));
  win2.y = (int)(y * (pow(2,params.mapLevel-1)));

  // center results
  if(params.useCentering)
  {
    int step = pow(2,params.mapLevel-1);
    int num = 0;
    float x_new = 0;
    float y_new = 0;
    for(int i = std::max(0,win2.y-step); i < std::min(_salmap.rows,win2.y+step); ++i)
    {
      for(int j = std::max(0,win2.x-step); j < std::min(_salmap.cols,win2.x+step); ++j)
      {
	if(_salmap.at<float>(i,j) > 0)
	{
	  num = num + 1;
	}
      }
    }
    for(int i = std::max(0,win2.y-step); i < std::min(_salmap.rows,win2.y+step); ++i)
    {
      for(int j = std::max(0,win2.x-step); j < std::min(_salmap.cols,win2.x+step); ++j)
      {
	if(_salmap.at<float>(i,j) > 0)
	{
	  y_new = y_new + ((float)i)/num;
	  x_new = x_new + ((float)j)/num;
	}
      }
    }

    win2.x = (int)(x_new);
    win2.y = (int)(y_new);
  }
}

bool SecondRound(cv::Point winner, cv::Mat &existing_points)
{
  if(existing_points.at<uchar>(winner.y,winner.x) > 0)
  {
    return true;
  }
    
  existing_points.at<uchar>(winner.y,winner.x) = 255;
  return(false);
}

void plotSalientLocation(cv::Point& win2, cv::Point& lastWinner, cv::Mat& img, Params& params, int pointNumber)
{
  int y2 = lastWinner.y;
  cv::circle(img,win2,3,cv::Scalar(255,0,0),-1);
  // put text
  char text[4];
  sprintf(text,"%d",pointNumber);
  cv::putText(img,text,cv::Point(win2.x,win2.y-5),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(255,0,0));
  bool doLine = ((y2 != -1) ? true : false);
  if (doLine)
  {
    cv::line(img,lastWinner,win2,cv::Scalar(255,0,0),2);
  }
  
}

void UpdateWinner2(cv::Mat &salmap, cv::Point &winner)
{
  //double seedValue = salmap.at<double>(winner.y,winner.x);
  std::vector<cv::Point> points;
  points.resize(salmap.rows*salmap.cols);
  points.at(0) = winner;
  double total_x = winner.x;
  double total_y = winner.y;
  double total = 1;
  long int i = 0;
  long int points_size = 1;
  cv::Mat processed = cv::Mat_<uchar>::zeros(salmap.rows,salmap.cols);
  processed.at<uchar>(winner.y,winner.x) = 1;
  while(i < points_size)
  {
    //4-element
    int x,y;
    x = points.at(i).x;
    y = points.at(i).y;
    i++;
    if(((x-1) >= 0) && (salmap.at<double>(y,x-1) > salmap.at<double>(y,x)) && (!processed.at<uchar>(y,x-1)))
    {
      total_x += x-1;
      total_y += y;
      total += 1;
      points.at(points_size) = cv::Point(x-1,y);
      points_size++;
      processed.at<uchar>(y,x-1) = 1;
    }
    if(((x+1) < salmap.cols) && (salmap.at<double>(y,x+1) > salmap.at<double>(y,x)) && (!processed.at<uchar>(y,x+1)))
    {
      total_x += x+1;
      total_y += y;
      total += 1;
      points.at(points_size) = cv::Point(x-1,y);
      points_size++;
      processed.at<uchar>(y,x+1) = 1;
    }
    if(((y-1) >= 0) && (salmap.at<double>(y-1,x) > salmap.at<double>(y,x)) && (!processed.at<uchar>(y-1,x)))
    {
      total_x += x;
      total_y += y-1;
      total += 1;
      points.at(points_size) = cv::Point(x,y-1);
      points_size++;
      processed.at<uchar>(y-1,x) = 1;
    }
    if(((y+1) < salmap.rows) && (salmap.at<double>(y+1,x) > salmap.at<double>(y,x)) && (!processed.at<uchar>(y+1,x)))
    {
      total_x += x;
      total_y += y+1;
      total += 1;
      points.at(points_size) = cv::Point(x,y+1);
      points_size++;
      processed.at<uchar>(y+1,x) = 1;
    }
  }
  winner.x = (int)(total_x/total);
  winner.y = (int)(total_y/total);
}

void UpdateWinner(cv::Mat &salmap, cv::Point &winner, Params& params)
{
  int delta = 2*pow(2,params.mapLevel-1);
  int x = winner.x;
  int y = winner.y;
  double total = 0;
  double total_x = 0;
  double total_y = 0;
  for(int i = -delta/2; i <= delta-delta/2; ++i)
  {
    for(int j = -delta/2; j <= delta-delta/2; ++j)
    {
      if((x+j >=0) && (x+j < salmap.rows) && (y+i >=0) && (y+i < salmap.cols))
      {
	if(salmap.at<double>(y+i,x+j) > 0)
	{
	  total += 1;
	  total_x += x+j;
	  total_y += y+i;
	}
      }
    }
  }
  winner.x = (int)(total_x/total);
  winner.y = (int)(total_y/total);
}

int CalculateWTA(cv::Mat& img, cv::Mat& _salmap, std::vector<cv::Point>& attented_points, int AttentionPointsNumber, Params &params)
{
  cv::Mat temp;
  _salmap.copyTo(temp);

  // perform morphological operations if necessary
  if(params.useMorphologyOpenning)
  {
    // calculate the size of the kernel
    int kernel_size = pow(2,params.mapLevel-1);
    cv::Mat element = cv::Mat_<uchar>::ones(kernel_size,kernel_size);
    cv::erode(_salmap,temp,element);
  }

  // resize to the necessary level
  cv::Mat salmap;
  cv::resize(temp,salmap,cv::Size(_salmap.cols/(pow(2,params.mapLevel-1)),_salmap.rows/(pow(2,params.mapLevel-1))));

  cv::Mat existing_points = cv::Mat_<uchar>::zeros(salmap.rows,salmap.cols);
  
  WTA wta;
  initializeWTA(wta,salmap,params);

  cv::Point lastWinner;
  lastWinner.x = -1;
  lastWinner.y = -1;

  bool done = false;
  cv::Point win2;
  // number of iterations
  int Ammount = 0;

  while(!done)
  {
    cv::Point winner(-1,-1);

    int cur_time = 0;
    while (winner.x == -1)
    {
      cur_time += 1;
      evolveWTA(wta,winner);
      //if(cur_time > 2500)
	//return(0);
    }
    // trigger inhibition of return
    applyIOR(wta,winner,params);

    // convert the winner's location to image coordinates
    winnerToImgCoords(win2,winner,params,img,_salmap);
    // update winner location
    //UpdateWinner(_salmap,win2,params);
    // print winner points
    //std::cerr << "winner(" << winner.y << "," << winner.x << ")" << std::endl;
    //std::cerr << "win2(" << win2.y << "," << win2.x << ")" << std::endl;
    // plot the currently attended region into the image figure
    
    if(SecondRound(winner,existing_points))
    {
      break;
    }
    
    plotSalientLocation(win2,lastWinner,img,params,Ammount);
    lastWinner = win2;

    Ammount++;
    
    if((AttentionPointsNumber > 0) && (Ammount >= AttentionPointsNumber))
    {
      done = true;
    }
    attented_points.push_back(cv::Point(win2)); // x - col, y - row
    
    //cv::imshow("wta.sm.Ginh",wta.sm.Ginh);
    //cv::waitKey(10);
    
  }
  
  return(0);
}

void PrintImage(const cv::Mat &img)
{
    for(int i = 0; i < img.rows; ++i)
    {
        for(int j = 0; j < img.cols; ++j)
        {
            std::cout << img.at<float>(i,j) << " ";
        }
        std::cout << std::endl;
    }
}

} //namespace AttentionModule
