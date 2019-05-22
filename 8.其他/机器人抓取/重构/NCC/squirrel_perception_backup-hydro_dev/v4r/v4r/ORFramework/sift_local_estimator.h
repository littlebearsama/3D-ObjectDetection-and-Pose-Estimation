/*
 * shot_local_estimator.h
 *
 *  Created on: Mar 24, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_REC_FRAMEWORK_SIFT_ESTIMATOR_H_
#define FAAT_PCL_REC_FRAMEWORK_SIFT_ESTIMATOR_H_

#include "local_estimator.h"
#include "faat_3d_rec_framework_defines.h"
#include <pcl/io/pcd_io.h>
#include <v4rexternal/SiftGPU/src/SiftGPU/SiftGPU.h>
#include <GL/glut.h>
#include <v4r/ORUtils/pcl_opencv.h>

namespace faat_pcl
{
namespace rec_3d_framework
{

template<typename PointInT, typename FeatureT>
class FAAT_3D_FRAMEWORK_API SIFTLocalEstimation : public LocalEstimator<PointInT, FeatureT>
{

    typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
    typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;

    using LocalEstimator<PointInT, FeatureT>::support_radius_;
    using LocalEstimator<PointInT, FeatureT>::normal_estimator_;
    using LocalEstimator<PointInT, FeatureT>::keypoint_extractor_;
    using LocalEstimator<PointInT, FeatureT>::adaptative_MLS_;
    using LocalEstimator<PointInT, FeatureT>::keypoint_indices_;
    pcl::PointIndices indices_;
    cv::Ptr<SiftGPU> sift;

public:

    size_t getFeatureType() const
    {
        return SIFT;
    }

    bool
    estimate (const PointInTPtr & in, PointInTPtr & keypoints, FeatureTPtr & signatures, std::vector<float> & scales)
    {

        keypoint_indices_.indices.clear();
        if(indices_.indices.size() == 0)
        {
            indices_.indices.resize(in->points.size());
            for(size_t i=0; i < indices_.indices.size(); i++)
            {
                indices_.indices[i] = i;
            }
        }

        keypoints.reset(new pcl::PointCloud<PointInT>);

        pcl::PointCloud<int> mask_cloud;
        mask_cloud.width = in->width;
        mask_cloud.height = in->height;
        mask_cloud.points.resize(in->width * in->height);
        for(size_t i=0; i < mask_cloud.points.size(); i++)
            mask_cloud.points[i] = 0;

        for(size_t i=0; i < indices_.indices.size(); i++)
            mask_cloud.points[indices_.indices[i]] = 1;

        cv::Mat_ < cv::Vec3b > colorImage;
        PCLOpenCV::ConvertPCLCloud2Image<PointInT> (in, colorImage);
        cv::Mat grayImage;
        cv::cvtColor (colorImage, grayImage, CV_BGR2GRAY);

        cv::Mat descriptors;
        std::vector<SiftGPU::SiftKeypoint> ks;

        SiftGPU *pSift = (SiftGPU*)&(*sift);

        if (sift->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
            throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");

        sift->VerifyContextGL();
        if (sift->RunSIFT (grayImage.cols, grayImage.rows, grayImage.ptr<uchar> (0), GL_LUMINANCE, GL_UNSIGNED_BYTE))
        {
            int num = sift->GetFeatureNum ();
            if (num > 0)
            {
                ks.resize(num);
                descriptors = cv::Mat(num,128,CV_32F);
                pSift->GetFeatureVector(&ks[0], descriptors.ptr<float>(0));
            }
            else std::cout<<"No SIFT found"<< std::endl;
        }
        else
            throw std::runtime_error ("PSiftGPU::Detect: SiftGPU Error!");

        //use indices_ to check if the keypoints and feature should be saved
        //compute SIFT keypoints and SIFT features
        //backproject sift keypoints to 3D and save in keypoints
        //save signatures

        scales.resize(ks.size());
        signatures->resize (ks.size ());
        signatures->width = static_cast<int> (ks.size ());
        signatures->height = 1;
        keypoints->points.resize(ks.size());
        int kept = 0;
        for(size_t i=0; i < ks.size(); i++)
        {
            int u,v;
            v = (int)(ks[i].y+.5);
            u = (int)(ks[i].x+.5);
            if(u >= 0 && v >= 0 && u < mask_cloud.width && v < mask_cloud.height && mask_cloud.at(u,v))
            {
                if(pcl_isfinite(in->at(u,v).z) && pcl_isfinite(in->at(u,v).x) && pcl_isfinite(in->at(u,v).y))
                {
                    keypoints->points[kept] = in->at(u,v);
                    keypoint_indices_.indices.push_back(v * in->width + u);
                    assert((v * in->width + u) < (in->points.size()));
                    for (int k = 0; k < 128; k++)
                        signatures->points[kept].histogram[k] = descriptors.at<float>(i,k);

                    scales[kept] = ks[i].s;
                    kept++;
                }
            }
        }

        signatures->width = kept;
        signatures->resize(kept);
        keypoints->points.resize(kept);
        scales.resize(kept);

        std::cout << "Number of SIFT features:" << kept << std::endl;
        indices_.indices.clear();

        return true;
    }


    bool
    estimate (const cv::Mat_ < cv::Vec3b > colorImage, std::vector<SiftGPU::SiftKeypoint> & ks, FeatureTPtr & signatures, std::vector<float> & scales)
    {
        //          sift_keypoints_.indices.clear();
        //          if(indices_.indices.size() == 0)
        //          {
        //            indices_.indices.resize(in->points.size());
        //            for(size_t i=0; i < indices_.indices.size(); i++)
        //            {
        //              indices_.indices[i] = i;
        //            }
        //          }

        //          keypoints.reset(new pcl::PointCloud<PointInT>);

        //          pcl::PointCloud<int> mask_cloud;
        //          mask_cloud.width = in->width;
        //          mask_cloud.height = in->height;
        //          mask_cloud.points.resize(in->width * in->height);
        //          for(size_t i=0; i < mask_cloud.points.size(); i++)
        //            mask_cloud.points[i] = 0;

        //for(size_t i=0; i < indices_.indices.size(); i++)
        //  mask_cloud.points[indices_.indices[i]] = 1;

        cv::Mat grayImage;
        cv::cvtColor (colorImage, grayImage, CV_BGR2GRAY);

        cv::Mat descriptors;

        SiftGPU *pSift = (SiftGPU*)&(*sift);

        if (sift->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
            throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");

        sift->VerifyContextGL();
        if (sift->RunSIFT (grayImage.cols, grayImage.rows, grayImage.ptr<uchar> (0), GL_LUMINANCE, GL_UNSIGNED_BYTE))
        {
            int num = sift->GetFeatureNum ();
            if (num > 0)
            {
                ks.resize(num);
                descriptors = cv::Mat(num,128,CV_32F);
                pSift->GetFeatureVector(&ks[0], descriptors.ptr<float>(0));
            }
            else std::cout<<"No SIFT found"<< std::endl;
        }
        else
            throw std::runtime_error ("PSiftGPU::Detect: SiftGPU Error!");

        //use indices_ to check if the keypoints and feature should be saved
        //compute SIFT keypoints and SIFT features
        //backproject sift keypoints to 3D and save in keypoints
        //save signatures

        scales.resize(ks.size());
        signatures->resize (ks.size ());
        signatures->width = static_cast<int> (ks.size ());
        signatures->height = 1;
        //keypoints->points.resize(ks.size());
        //          int kept = 0;
        for(size_t i=0; i < ks.size(); i++)
        {
            //int u,v;
            //            v = (int)(ks[i].y+.5);
            //            u = (int)(ks[i].x+.5);
            //            if(u >= 0 && v >= 0 && u < mask_cloud.width && v < mask_cloud.height && mask_cloud.at(u,v))
            //            {
            //              if(pcl_isfinite(in->at(u,v).z) && pcl_isfinite(in->at(u,v).x) && pcl_isfinite(in->at(u,v).y))
            //              {
            //keypoints->points[kept] = in->at(u,v);
            //sift_keypoints_.indices.push_back(v * in->width + u);
            //assert((v * in->width + u) < (in->points.size()));
            for (int k = 0; k < 128; k++)
                signatures->points[i].histogram[k] = descriptors.at<float>(i,k);

            scales[i] = ks[i].s;
            //kept++;
            //              }
            //            }
        }

        //signatures->width = kept;
        //signatures->resize(kept);
        //keypoints->points.resize(kept);
        //scales.resize(kept);

        std::cout << "Number of SIFT features:" << ks.size() << std::endl;
        indices_.indices.clear();

        return true;
    }

    SIFTLocalEstimation (cv::Ptr<SiftGPU> _sift = cv::Ptr<SiftGPU> ())
    {
        if (_sift.empty ())
        {
            //init sift

            static char kw[][16] = {"-m", "-fo", "-1", "-s", "-v", "1", "-pack"};
            char * argv[] = {kw[0], kw[1], kw[2], kw[3],kw[4],kw[5],kw[6], NULL};

            int argc = sizeof(argv) / sizeof(char*);
            sift = new SiftGPU ();
            sift->ParseParam (argc, argv);

            //create an OpenGL context for computation
            if (sift->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
                throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");
        }
        else
        {
            sift = _sift;
        }
    }

    bool estimate(const PointInTPtr & in, FeatureTPtr & signatures)
    {
        //fill keypoints with indices_, all points at indices_[i] should be valid
        std::vector<SiftGPU::SiftKeypoint> ks;
        if(indices_.indices.size() == 0)
        {
            PCL_ERROR("indices are empty\n");
            return false;
        }

        ks.resize(indices_.indices.size());
        for(size_t i=0; i < indices_.indices.size(); i++)
        {
            int u,v;
            ks[i].y = (float)(indices_.indices[i] / in->width);
            ks[i].x = (float)(indices_.indices[i] % in->width);
            ks[i].s = 1.f;
            ks[i].o = 0.f;
        }

        std::cout << "Number of keypoints:" << ks.size() << std::endl;

        cv::Mat_ < cv::Vec3b > colorImage;
        PCLOpenCV::ConvertPCLCloud2Image<PointInT> (in, colorImage);
        cv::Mat grayImage;
        cv::cvtColor (colorImage, grayImage, CV_BGR2GRAY);

        if (sift->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
            throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");

        sift->VerifyContextGL();

        SiftGPU *pSift = (SiftGPU*)&(*sift);
        pSift->SetKeypointList(ks.size(), &ks[0], 0);
        sift->VerifyContextGL();

        cv::Mat descriptors;
        if(pSift->RunSIFT(grayImage.cols,grayImage.rows,grayImage.ptr<uchar>(0),GL_LUMINANCE,GL_UNSIGNED_BYTE))
        {
            int num = pSift->GetFeatureNum();
            if (num==(int)ks.size())
            {
                descriptors = cv::Mat(num,128,CV_32F);
                pSift->GetFeatureVector(NULL, descriptors.ptr<float>(0));
            }
            else std::cout<<"No SIFT found"<< std::endl;
        }

        signatures->resize (ks.size ());
        signatures->width = static_cast<int> (ks.size ());
        signatures->height = 1;
        for(size_t i=0; i < ks.size(); i++)
        {
            for (int k = 0; k < 128; k++)
                signatures->points[i].histogram[k] = descriptors.at<float>(i,k);
        }

        return true;
    }

    bool
    estimate (const PointInTPtr & in, PointInTPtr & processed, PointInTPtr & keypoints, FeatureTPtr & signatures)
    {

        keypoint_indices_.indices.clear();
        if(indices_.indices.size() == 0)
        {
            indices_.indices.resize(in->points.size());
            for(size_t i=0; i < indices_.indices.size(); i++)
            {
                indices_.indices[i] = i;
            }
        }

        processed.reset(new pcl::PointCloud<PointInT>);
        keypoints.reset(new pcl::PointCloud<PointInT>);

        pcl::PointCloud<int> mask_cloud;
        mask_cloud.width = in->width;
        mask_cloud.height = in->height;
        mask_cloud.points.resize(in->width * in->height);
        for(size_t i=0; i < mask_cloud.points.size(); i++)
            mask_cloud.points[i] = 0;

        for(size_t i=0; i < indices_.indices.size(); i++)
            mask_cloud.points[indices_.indices[i]] = 1;

        cv::Mat_ < cv::Vec3b > colorImage;
        PCLOpenCV::ConvertPCLCloud2Image<PointInT> (in, colorImage);
        cv::Mat grayImage;
        cv::cvtColor (colorImage, grayImage, CV_BGR2GRAY);

        cv::Mat descriptors;
        std::vector<SiftGPU::SiftKeypoint> ks;

        SiftGPU *pSift = (SiftGPU*)&(*sift);

        if (sift->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED)
            throw std::runtime_error ("PSiftGPU::PSiftGPU: No GL support!");

        sift->VerifyContextGL();
        if (sift->RunSIFT (grayImage.cols, grayImage.rows, grayImage.ptr<uchar> (0), GL_LUMINANCE, GL_UNSIGNED_BYTE))
        {
            int num = sift->GetFeatureNum ();
            if (num > 0)
            {
                ks.resize(num);
                descriptors = cv::Mat(num,128,CV_32F);
                pSift->GetFeatureVector(&ks[0], descriptors.ptr<float>(0));
            }
            else std::cout<<"No SIFT found"<< std::endl;
        }
        else
            throw std::runtime_error ("PSiftGPU::Detect: SiftGPU Error!");

        //use indices_ to check if the keypoints and feature should be saved
        //compute SIFT keypoints and SIFT features
        //backproject sift keypoints to 3D and save in keypoints
        //save signatures

        signatures->resize (ks.size ());
        signatures->width = static_cast<int> (ks.size ());
        signatures->height = 1;
        keypoints->points.resize(ks.size());
        int kept = 0;
        for(size_t i=0; i < ks.size(); i++)
        {
            int u,v;
            v = (int)(ks[i].y+.5);
            u = (int)(ks[i].x+.5);
            if(u >= 0 && v >= 0 && u < mask_cloud.width && v < mask_cloud.height && mask_cloud.at(u,v))
            {
                if(pcl_isfinite(in->at(u,v).z) && pcl_isfinite(in->at(u,v).x) && pcl_isfinite(in->at(u,v).y))
                {
                    keypoints->points[kept] = in->at(u,v);
                    keypoint_indices_.indices.push_back(v * in->width + u);
                    assert((v * in->width + u) < (in->points.size()));
                    for (int k = 0; k < 128; k++)
                        signatures->points[kept].histogram[k] = descriptors.at<float>(i,k);
                    kept++;
                }
            }
        }

        signatures->width = kept;
        signatures->resize(kept);
        keypoints->points.resize(kept);
        pcl::copyPointCloud(*in, indices_, *processed);
        std::cout << "Number of SIFT features:" << kept << std::endl;
        indices_.indices.clear();

        return true;
    }

    void
    setIndices (const pcl::PointIndices & p_indices)
    {
        indices_ = p_indices;
    }

    void
    setIndices(const std::vector<int> & p_indices)
    {
        indices_.indices = p_indices;
    }

    bool acceptsIndices() const
    {
        return true;
    }

private:

};
}
}

#endif /* REC_FRAMEWORK_SHOT_LOCAL_ESTIMATOR_H_ */
