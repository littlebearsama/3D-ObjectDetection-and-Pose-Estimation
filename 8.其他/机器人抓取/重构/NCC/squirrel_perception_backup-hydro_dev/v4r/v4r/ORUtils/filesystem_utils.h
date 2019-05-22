/*
 * filesystem_utils.h
 *
 *  Created on: Mar 15, 2013
 *      Author: aitor
 */

#ifndef FILESYSTEM_UTILS_H_
#define FILESYSTEM_UTILS_H_

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/common/common.h>
#include <fstream>
#include <boost/regex.hpp>

namespace bf = boost::filesystem;

namespace faat_pcl
{
  namespace utils
  {

    void
    getFilesInDirectory (bf::path & path_with_views,
                           std::vector<std::string> & view_filenames,
                           const std::string & pattern);

    void
    getFilesInDirectoryRecursive (bf::path & path_with_views,
                                    std::string & rel_path_so_far,
                                    std::vector<std::string> & view_filenames,
                                    const std::string & pattern);

    void
    getFilesInDirectory (   bf::path & dir,
                            std::string & rel_path_so_far,
                            std::vector<std::string> & relative_paths,
                            std::string & ext);

    bool
    writeMatrixToFile (std::string file, Eigen::Matrix4f & matrix);

    bool
    readMatrixFromFile (std::string file, Eigen::Matrix4f & matrix);

    bool
    readMatrixFromFile (std::string file, Eigen::Matrix4f & matrix, int padding);
  }
}
#endif /* FILESYSTEM_UTILS_H_ */
