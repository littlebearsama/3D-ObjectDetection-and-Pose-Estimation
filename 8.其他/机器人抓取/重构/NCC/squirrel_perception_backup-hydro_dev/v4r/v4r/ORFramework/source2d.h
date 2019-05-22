#ifndef MODEL2D_H
#define MODEL2D_H

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>


namespace bf = boost::filesystem;

namespace faat_pcl
{
namespace rec_3d_framework
{
class Model2D
{
public:
    std::string id_;
    std::string class_;
    boost::shared_ptr<cv::Mat>view_;
    std::string view_filename_;

    Model2D()
    {
    }

    bool
    operator== (const Model2D &other) const
    {
        return (id_ == other.id_) && (class_ == other.class_);
    }
};


class Source2D
{
    typedef boost::shared_ptr<Model2D> Model2DTPtr;
protected:
    std::string path_;
    boost::shared_ptr<std::vector<Model2DTPtr> > models_;
    bool load_into_memory_;

public:
    Source2D()
    {
        load_into_memory_ = true;
    }

    void
    getFoldersInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths)
    {
        bf::directory_iterator end_itr;
        for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
        {
            //check if its a directory, else ignore
            if (bf::is_directory (*itr))
            {
#if BOOST_FILESYSTEM_VERSION == 3
                std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
                std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
                relative_paths.push_back (path);
            }
        }
    }

    void
    getFilesInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
    {
        bf::directory_iterator end_itr;
        for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
        {
            //check if its a directory, then ignore
            if (bf::is_directory (*itr))
            {

            }
            else
            {
                std::vector < std::string > strs;
#if BOOST_FILESYSTEM_VERSION == 3
                std::string file = (itr->path ().filename ()).string();
#else
                std::string file = (itr->path ()).filename ();
#endif

                boost::split (strs, file, boost::is_any_of ("."));
                std::string extension = strs[strs.size () - 1];

                if (extension.compare (ext) == 0)
                {
#if BOOST_FILESYSTEM_VERSION == 3
                    std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
                    std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

                    relative_paths.push_back (path);
                }
            }
        }
    }

    boost::shared_ptr<std::vector<Model2DTPtr> >
    getModels ()
    {
        return models_;
    }

    bool
    getModelById (std::string & model_id, Model2DTPtr & m)
    {
        typename std::vector<Model2DTPtr>::iterator it = models_->begin ();
        while (it != models_->end ())
        {
            if (model_id.compare ((*it)->id_) == 0)
            {
                m = *it;
                return true;
            } else
            {
                it++;
            }
        }

        return false;
    }

    void
    setLoadIntoMemory(bool b)
    {
      load_into_memory_ = b;
    }

    boost::shared_ptr<std::vector<Model2DTPtr> >
    getModels (std::string & model_id)
    {
        typename std::vector<Model2DTPtr>::iterator it = models_->begin ();
        while (it != models_->end ())
        {
            if (model_id.compare ((*it)->id_) != 0)
            {
                it = models_->erase (it);
            }
            else
            {
                it++;
            }
        }

        return models_;
    }

    void
    setPath (std::string & path)
    {
        path_ = path;
    }

    void
    loadInMemorySpecificModel(std::string & dir, Model2D & model)
    {
        std::stringstream pathmodel;
        pathmodel << dir << "/" << model.class_ << "/" << model.id_;
        cv::Mat image = cv::imread(model.view_filename_, CV_LOAD_IMAGE_COLOR);

        std::string directory, filename;
        char sep = '/';
#ifdef _WIN32
        sep = '\\';
#endif

        size_t position = model.view_filename_.rfind(sep);
        if (position != std::string::npos)
        {
            directory = model.view_filename_.substr(0, position);
            filename = model.view_filename_.substr(position+1, model.view_filename_.length()-1);
        }

        *(model.view_) =image;
    }

    void
    loadOrGenerate (std::string & model_path, Model2D & model)
    {
        model.view_.reset (new cv::Mat() );
        model.view_filename_ = model_path;

        if(load_into_memory_)
        {
            loadInMemorySpecificModel(model_path, model);
        }
    }

    void
    generate ()
    {
        models_.reset (new std::vector<Model2DTPtr>);

        //get models in directory
        std::vector < std::string > folders;
        std::string start = "";
        bf::path dir = path_;
        std::string ext_v[] = {"jpg", "JPG", "png", "PNG", "bmp", "BMP", "jpeg", "JPEG"};

        getFoldersInDirectory (dir, start, folders);
        std::cout << "There are " << folders.size() << " folders. " << std::endl;

        for (size_t i = 0; i < folders.size (); i++)
        {
            std::stringstream class_path;
            class_path << path_ << "/" << folders[i];
            bf::path class_dir = class_path.str();
            for(size_t ext_id=0; ext_id < sizeof(ext_v)/sizeof(ext_v[0]); ext_id++)
            {
                std::vector < std::string > filesInRelFolder;
                getFilesInDirectory (class_dir, start, filesInRelFolder, ext_v[ext_id]);
                std::cout << "There are " <<  filesInRelFolder.size() << " files in folder " << folders[i] << ". " << std::endl;

                for (size_t kk = 0; kk < filesInRelFolder.size (); kk++)
                {
                    Model2DTPtr m(new Model2D());
                    m->class_ = folders[i];
                    m->id_ = filesInRelFolder[kk];

                    std::stringstream model_path;
                    model_path << class_path.str() << "/" << filesInRelFolder[kk];
                    std::string path_model = model_path.str ();
                    std::cout << "Calling loadOrGenerate path_model: " << path_model << ", m_class: " << m->class_ << ", m_id: " << m->id_ << std::endl;
                    loadOrGenerate (path_model, *m);

                    models_->push_back (m);
                }
            }
        }
    }
};
}
}

#endif // MODEL2D_H
