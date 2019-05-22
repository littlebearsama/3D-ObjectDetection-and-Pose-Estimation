#include "local_recognizer.hpp"
#include "faat_3d_rec_framework_defines.h"

template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::Histogram<352> >;
template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<352> >;
template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<128> >;
//template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<1344> >;
//template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::FPFHSignature33>;
//template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::FPFHSignature33>;

template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZ, pcl::Histogram<352> >;
template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::Histogram<128> >;
//template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::Histogram<352> >;
//template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::Histogram<1344> >;
//template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZ, pcl::FPFHSignature33>;
//template class PCL_EXPORTS faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::FPFHSignature33>;
