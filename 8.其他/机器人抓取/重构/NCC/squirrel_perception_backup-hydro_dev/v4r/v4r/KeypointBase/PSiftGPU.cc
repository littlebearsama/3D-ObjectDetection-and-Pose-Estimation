/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "PSiftGPU.hh"

namespace kp 
{

using namespace std;

PSiftGPU::PSiftGPU(Parameter p, cv::Ptr<SiftGPU> _sift, cv::Ptr<SiftMatchGPU> _matcher, int memSize)
{
  gpuMemSize = memSize;
  param=p;

  if (_sift.empty())
  {
    //init sift
    const char * argv[] = {"-m", "-fo","-1", "-s", "-v", "1", "-pack"};

    int argc = sizeof(argv)/sizeof(char*);
    sift = new SiftGPU();
    sift->ParseParam(argc, (char**)argv);

    //create an OpenGL context for computation
    if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
      throw runtime_error ("PSiftGPU::PSiftGPU: No GL support!");
  }
  else
  {
    sift = _sift;
  }
  if (_matcher.empty())
  {
    gpuMemSize = 4096;
    matcher = new SiftMatchGPU(4096);
    matcher->VerifyContextGL();
  }
  else
  {
    matcher = _matcher;
  }
}

PSiftGPU::~PSiftGPU()
{
}




/************************************** PRIVATE ************************************/
/**
 * TransformToRootSIFT
 * computes the square root of the L1 normalized SIFT vectors.
 * Then the Euclidean distance is equivalent to using the Hellinger kernel 
 *  to compare the original SIFT vectors
 */
void PSiftGPU::TransformToRootSIFT(DataMatrix2Df &descriptors) const
{
  float norm;

  for (int i=0; i<descriptors.rows; i++)
  {
    Eigen::Map<Eigen::VectorXf> desc(&descriptors(i,0), descriptors.cols);
    norm = desc.lpNorm<1>();
    desc.array() /= norm;
    desc.array() = desc.array().sqrt();
  }
}


/**
 * TransformToRootSIFT
 * computes the square root of the L1 normalized SIFT vectors.
 * Then the Euclidean distance is equivalent to using the Hellinger kernel 
 *  to compare the original SIFT vectors
 */
void PSiftGPU::TransformToRootSIFT(cv::Mat& descriptors) const
{
  float norm;

  for (int i=0; i<descriptors.rows; i++)
  {
    Eigen::Map<Eigen::VectorXf> desc(&descriptors.at<float>(i,0), descriptors.cols);
    norm = desc.lpNorm<1>();
    desc.array() /= norm;
    desc.array() = desc.array().sqrt();
  }
}

void PSiftGPU::detectImpl( const cv::Mat& image, vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) const
{
  keypoints.clear();
  cv::Mat grayImage;
  if( image.type() != CV_8U ) cv::cvtColor( image, grayImage, CV_BGR2GRAY );
  else grayImage = image;

  SiftGPU *pSift = (SiftGPU*)&(*sift);
  pSift->CreateContextGL();
  pSift->VerifyContextGL();

  if(pSift->RunSIFT(grayImage.cols,grayImage.rows,grayImage.ptr<uchar>(0),GL_LUMINANCE,GL_UNSIGNED_BYTE))
  {
    int num = pSift->GetFeatureNum();
    if (num>0)
    {
      vector<SiftGPU::SiftKeypoint> ks(num);

      pSift->GetFeatureVector(&ks[0], NULL);

      //copy sift
      if (mask.size() != image.size() || mask.type()!=CV_8U)
      {
        keypoints.resize(num);
        for (int i=0; i<num; i++)
        {
          keypoints[i] = cv::KeyPoint(ks[i].x,ks[i].y, ks[i].s*3, -ks[i].o*180/M_PI, 1,0,i);
        }
      }
      else
      {
        for (int i=0; i<num; i++)
          if (mask.at<uchar>((int)(ks[i].y+.5),(int)(ks[i].x+.5)) > 0)
          {
            keypoints.push_back(cv::KeyPoint(ks[i].x,ks[i].y, ks[i].s*3, -ks[i].o*180/M_PI, 1,0,keypoints.size()));
          }
      }
    }else cout<<"No SIFT found"<<endl;
  }else throw runtime_error ("PSiftGPU::Detect: SiftGPU Error!");

}

/**
 * compute descriptors for given keypoints
 */
void PSiftGPU::computeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, 
                 cv::Mat& descriptors) const
{
  if (keypoints.size()==0)
    return;

  cv::Mat grayImage;
  if( image.type() != CV_8U ) cv::cvtColor( image, grayImage, CV_BGR2GRAY );
  else grayImage = image;

  vector<SiftGPU::SiftKeypoint> ks(keypoints.size());
  for (unsigned i=0; i<ks.size(); i++)
  {
    ks[i].x = keypoints[i].pt.x;
    ks[i].y = keypoints[i].pt.y;
    ks[i].s = keypoints[i].size / 3.;
    ks[i].o = -keypoints[i].angle*M_PI/180.;
  }

  SiftGPU *pSift = (SiftGPU*)&(*sift);
  pSift->CreateContextGL();
  pSift->VerifyContextGL();

  pSift->SetKeypointList(ks.size(), &ks[0], 0);

  if(pSift->RunSIFT(grayImage.cols,grayImage.rows,grayImage.ptr<uchar>(0),GL_LUMINANCE,GL_UNSIGNED_BYTE))
  {
    int num = pSift->GetFeatureNum();
    if (num==(int)keypoints.size())
    {
      descriptors = cv::Mat(num,128,CV_32F);
      pSift->GetFeatureVector(NULL, descriptors.ptr<float>(0));
      if (param.computeRootSIFT) TransformToRootSIFT(descriptors);
    }
    else cout<<"No SIFT found"<<endl;
  }
  else throw runtime_error ("PSiftGPU::computeImpl: SiftGPU Error!");
}

/**
 * knnMatchImpl
 * mask and compactResult are not supported
 */
void PSiftGPU::knnMatchImpl( const cv::Mat& queryDescriptors, vector<vector<cv::DMatch> >& matches,
                 int k, const vector<cv::Mat>& masks, bool compactResult) 
{
  matches.clear();
  if (trainDescCollection.size()==0 || queryDescriptors.rows == 0)
    return;

  cv::Mat tmpTrainDesc, tmpQueryDesc;
  int num, numTrain=0;
  int (*match_buf)[2] = new int[(int)queryDescriptors.rows][2];

  for (unsigned i=0; i<trainDescCollection.size(); i++)
    numTrain += trainDescCollection[i].rows;

  vector<unsigned> ltView, ltKey;

  if (gpuMemSize < (int)queryDescriptors.rows)
  { 
    gpuMemSize=(int)queryDescriptors.rows;
    matcher->SetMaxSift((int)queryDescriptors.rows);
  }
  if (gpuMemSize < (int)numTrain)
  {
    gpuMemSize=numTrain;
    matcher->SetMaxSift(numTrain);
  }

  // prepare data structure
  if (queryDescriptors.isContinuous()) tmpQueryDesc = queryDescriptors;
  else queryDescriptors.copyTo(tmpQueryDesc);

  if (trainDescCollection.size()==1)
  {
    if (trainDescCollection[0].isContinuous()) tmpTrainDesc = trainDescCollection[0];
    else trainDescCollection[0].copyTo(tmpTrainDesc);
  }
  else
  {
    unsigned z=0;
    tmpTrainDesc = cv::Mat(numTrain,128,CV_32F);
    ltView.resize(numTrain), ltKey.resize(numTrain);

    for (unsigned i=0; i<trainDescCollection.size(); i++)
    {
      for (int j=0; j<trainDescCollection[i].rows; j++, z++)
      {
        tmpTrainDesc.row(z) = trainDescCollection[i].row(j);
        ltView[z] = i;
        ltKey[z] = j;
      }
    }
  }
  
  matcher->SetDescriptors(0, tmpQueryDesc.rows, tmpQueryDesc.ptr<float>(0));  // image keypoints
  matcher->SetDescriptors(1, tmpTrainDesc.rows, tmpTrainDesc.ptr<float>(0));  // codebook

  num = matcher->GetSiftMatch(tmpQueryDesc.rows, match_buf, param.distmax,
                              param.ratiomax, param.mutual_best_match);
    
  matches.resize(num,vector<cv::DMatch>(1));
  if (ltView.size()>0)
  {
    for (unsigned i=0; i<(unsigned)num; i++)
    {
      matches[i][0] = cv::DMatch( match_buf[i][0], ltKey[match_buf[i][1]],ltView[match_buf[i][1]], 
                                  Distance128(tmpQueryDesc.ptr<float>(match_buf[i][0]),
                                              tmpTrainDesc.ptr<float>(match_buf[i][1])) );
    }
  }
  else
  {
    for (unsigned i=0; i<(unsigned)num; i++)
    {
      matches[i][0] = cv::DMatch( match_buf[i][0], match_buf[i][1], 
                                  Distance128(tmpQueryDesc.ptr<float>(match_buf[i][0]),
                                              tmpTrainDesc.ptr<float>(match_buf[i][1])) );
    }
  }

  delete[] match_buf;
}

/**
 * radiusMatchImpl
 */
void PSiftGPU::radiusMatchImpl(const cv::Mat& queryDescriptors,vector<vector<cv::DMatch> >& matches,
             float maxDistance, const vector<cv::Mat>& masks, bool compactResult )
{
  matches.clear();
  if (trainDescCollection.size()==0 || queryDescriptors.rows == 0)
    return;

  cv::Mat tmpTrainDesc, tmpQueryDesc;
  int num, numTrain=0;
  int (*match_buf)[2] = new int[(int)queryDescriptors.rows][2];

  for (unsigned i=0; i<trainDescCollection.size(); i++)
    numTrain += trainDescCollection[i].rows;

  vector<unsigned> ltView, ltKey;

  if (gpuMemSize < (int)queryDescriptors.rows)
  { 
    gpuMemSize=(int)queryDescriptors.rows;
    matcher->SetMaxSift((int)queryDescriptors.rows);
  }
  if (gpuMemSize < (int)numTrain)
  {
    gpuMemSize=numTrain;
    matcher->SetMaxSift(numTrain);
  }

  // prepare data structure
  if (queryDescriptors.isContinuous()) tmpQueryDesc = queryDescriptors;
  else queryDescriptors.copyTo(tmpQueryDesc);

  if (trainDescCollection.size()==1)
  {
    if (trainDescCollection[0].isContinuous()) tmpTrainDesc = trainDescCollection[0];
    else trainDescCollection[0].copyTo(tmpTrainDesc);
  }
  else
  {
    unsigned z=0;
    tmpTrainDesc = cv::Mat(numTrain,128,CV_32F);
    ltView.resize(numTrain), ltKey.resize(numTrain);

    for (unsigned i=0; i<trainDescCollection.size(); i++)
    {
      for (int j=0; j<trainDescCollection[i].rows; j++, z++)
      {
        tmpTrainDesc.row(z) = trainDescCollection[i].row(j);
        ltView[z] = i;
        ltKey[z] = j;
      }
    }
  }
  
  matcher->SetDescriptors(0, tmpQueryDesc.rows, tmpQueryDesc.ptr<float>(0));  // image keypoints
  matcher->SetDescriptors(1, tmpTrainDesc.rows, tmpTrainDesc.ptr<float>(0));  // codebook

  num = matcher->GetSiftMatch(tmpQueryDesc.rows, match_buf, maxDistance,
                            param.ratiomax, param.mutual_best_match);
    
  matches.resize(num,vector<cv::DMatch>(1));
  if (ltView.size()>0)
  {
    for (unsigned i=0; i<(unsigned)num; i++)
    {
      matches[i][0] = cv::DMatch( match_buf[i][0], ltKey[match_buf[i][1]],ltView[match_buf[i][1]], 
                                  Distance128(tmpQueryDesc.ptr<float>(match_buf[i][0]),
                                              tmpTrainDesc.ptr<float>(match_buf[i][1])) );
    }
  }
  else
  {
    for (unsigned i=0; i<(unsigned)num; i++)
    {
      matches[i][0] = cv::DMatch( match_buf[i][0], match_buf[i][1], 
                                  Distance128(tmpQueryDesc.ptr<float>(match_buf[i][0]),
                                              tmpTrainDesc.ptr<float>(match_buf[i][1])) );
    }
  }

  delete[] match_buf;

}



/************************************** PUBLIC ************************************/

/**
 * compute descriptors for given keypoints
 */
void PSiftGPU::compute(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints, DataMatrix2Df &descriptors)
{
  descriptors.clear();
  if (keypoints.size()==0)
    return;

  cv::Mat grayImage;
  if( image.type() != CV_8U ) cv::cvtColor( image, grayImage, CV_BGR2GRAY );
  else grayImage = image;

  vector<SiftGPU::SiftKeypoint> ks(keypoints.size());
  for (unsigned i=0; i<ks.size(); i++)
  {
    ks[i].x = keypoints[i].pt.x;
    ks[i].y = keypoints[i].pt.y;
    ks[i].s = keypoints[i].size / 3.;
    ks[i].o = -keypoints[i].angle*M_PI/180;
  }

  SiftGPU *pSift = (SiftGPU*)&(*sift);
  pSift->CreateContextGL();
  pSift->VerifyContextGL();

  pSift->SetKeypointList(ks.size(), &ks[0]);

  if(pSift->RunSIFT(grayImage.cols,grayImage.rows,grayImage.ptr<uchar>(0),GL_LUMINANCE,GL_UNSIGNED_BYTE))
  {
    int rows = pSift->GetFeatureNum();

    if (rows==(int)keypoints.size())
    {
      descriptors.resize(rows,128);
      pSift->GetFeatureVector(NULL, &descriptors[0]);
      if (param.computeRootSIFT) TransformToRootSIFT(descriptors);
    }
    else {descriptors.clear(); cout<<"No SIFT found"<<endl;}
  }
  else throw runtime_error ("PSiftGPU::computeImpl: SiftGPU Error!");
}


/**
 * detect
 */
void PSiftGPU::detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, DataMatrix2Df &descriptors, const cv::Mat& mask )
{
  descriptors.clear();
  keypoints.clear();
  cv::Mat grayImage;
  if( image.type() != CV_8U ) cv::cvtColor( image, grayImage, CV_BGR2GRAY );
  else grayImage = image;

  SiftGPU *pSift = (SiftGPU*)&(*sift);
  pSift->CreateContextGL();
  pSift->VerifyContextGL();

  if(pSift->RunSIFT(grayImage.cols,grayImage.rows,grayImage.ptr<uchar>(0),GL_LUMINANCE,GL_UNSIGNED_BYTE))
  {
    int rows = pSift->GetFeatureNum();
    if (rows>0)
    {
      vector<SiftGPU::SiftKeypoint> ks(rows);
      descriptors.resize(rows,128);
      
      pSift->GetFeatureVector(&ks[0], &descriptors[0]);

      //copy sift
      if (mask.size() != image.size() || mask.type()!=CV_8U)
      {
        keypoints.resize(rows);
        for (int i=0; i<rows; i++)
        {
          keypoints[i] = cv::KeyPoint(ks[i].x,ks[i].y, ks[i].s*3, -ks[i].o*180/M_PI, 1,0,i);
        }
      }
      else
      {
        for (int i=0; i<rows; i++)
          if (mask.at<uchar>((int)(ks[i].y+.5),(int)(ks[i].x+.5)) > 0)
          {
            keypoints.push_back(cv::KeyPoint(ks[i].x,ks[i].y, ks[i].s*3, -ks[i].o*180/M_PI, 1,0,keypoints.size()));
          }
      }
    }else cout<<"No SIFT found"<<endl;
  }else throw runtime_error ("PSiftGPU::Detect: SiftGPU Error!");
}

/**
 * compute dog and sift descriptor
 */
void PSiftGPU::detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask)
{
  detectImpl(image, keypoints, mask);
}

/**
 * compute descriptors for given keypoints
 */
void PSiftGPU::compute(const cv::Mat& image, vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors )
{
  computeImpl(image, keypoints, descriptors);
}

/**
 * match descriptors on gpu
 * no mask used
 */
void PSiftGPU::match( const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, vector<cv::DMatch>& matches, const cv::Mat& mask)
{
  add( vector<cv::Mat>(1, trainDescriptors) );
  vector<vector<cv::DMatch> > knnMatches;
  knnMatchImpl(queryDescriptors, knnMatches, 1, vector<cv::Mat>(1,mask), true);

  // copy matches
  matches.clear();
  for (unsigned i=0; i<knnMatches.size(); i++)
    for (unsigned j=0; j<knnMatches[i].size(); j++)
      matches.push_back(knnMatches[i][j]);
}

/**
 * add train descriptors for matching
 */
/*void PSiftGPU::add( const vector<cv::Mat>& descriptors )
{
cout<<"add"<<endl;
  for (unsigned i=0; i<descriptors.size(); i++)
    trainDescriptors.push_back(descriptors[i]);
}*/

/**
 * clear train descriptors of matching storage
 */
/*void PSiftGPU::clear()
{
  trainDescriptors.clear();
}*/

/**
 * clone
 */
cv::Ptr<cv::DescriptorMatcher> PSiftGPU::clone( bool emptyTrainData ) const 
{
  PSiftGPU* tmpMatcher = new PSiftGPU(param, sift,matcher, gpuMemSize);

  return tmpMatcher;
}

}

