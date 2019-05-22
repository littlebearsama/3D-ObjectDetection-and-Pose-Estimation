/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_WARP_PATCH_HOMOGRAPHY_HPP
#define KP_WARP_PATCH_HOMOGRAPHY_HPP


namespace kp 
{

/** 
 * getInterpolated
 * get a bilinear interpolated pixel
 */
template <typename T>
inline T getInterpolated(const cv::Mat_<unsigned char> &im, const Eigen::Matrix<T,2,1> &pt)
{
  int xt = (int) pt[0];
  int yt = (int) pt[1];
  T ax = pt[0] - xt;
  T ay = pt[1] - yt;

  return ( (1.-ax) * (1.-ay) * im(yt,xt) +
            ax     * (1.-ay) * im(yt,xt+1) +
           (1.-ax) *  ay     * im(yt+1,xt) +
            ax     *  ay     * im(yt+1,xt+1) );
}

/** 
 * getInterpolated
 * get a bilinear interpolated pixel
 */
template <typename T>
inline T getInterpolated(const unsigned char *im, int rows, int cols, const T pt[2])
{
  int xt = (int) pt[0];
  int yt = (int) pt[1];
  T ax = pt[0] - xt;
  T ay = pt[1] - yt;

  const unsigned char *ptr_y0 = im+yt*cols+xt;
  const unsigned char *ptr_y1 = im+(yt+1)*cols+xt;

  return ( (1.-ax) * (1.-ay) * *ptr_y0 +
            ax     * (1.-ay) * ptr_y0[1] +
           (1.-ax) *  ay     * *ptr_y1 +
            ax     *  ay     * ptr_y1[1] );
}


/**
 * mapPoint
 */
template <typename T>
inline void mapPoint(const Eigen::Matrix<T,2,1> &pt_in, const Eigen::Matrix<T,3,3> &H, Eigen::Matrix<T,2,1> &pt_out)
{
  pt_out[0] = H(0,0)*pt_in[0] + H(0,1)*pt_in[1] + H(0,2);
  pt_out[1] = H(1,0)*pt_in[0] + H(1,1)*pt_in[1] + H(1,2);
  float t = H(2,0)*pt_in[0] + H(2,1)*pt_in[1] + H(2,2);
  pt_out[0] /= t;
  pt_out[1] /= t;
}

/**
 * mapPoint
 */
template<typename TN1, typename TN2, typename TN3>
inline void mapPoint(const TN1 &in_x, const TN1 &in_y, const TN2 H[9], TN3 &out_x, TN3 &out_y)
{
  out_x = H[0]*in_x + H[1]*in_y + H[2];
  out_y = H[3]*in_x + H[4]*in_y + H[5];
  TN3 t = H[6]*in_x + H[7]*in_y + H[8];
  out_x /= t;
  out_y /= t;
}

/**
 * warpPatchHomography
 */
template <typename T>
bool warpPatchHomography(const cv::Mat_<unsigned char> &im, const Eigen::Matrix<T,3,3> &H, cv::Mat_<unsigned char> &patch)
{
  Eigen::Matrix<T,2,1> pt;

  for (int v=0; v<patch.rows; v++)
  {
    for (int u=0; u<patch.cols; u++)
    {
      mapPoint(Eigen::Matrix<T,2,1>(u,v), H, pt);

      if ((int)pt[0] < 0 || (int)pt[1] < 0 || int(pt[0])+1 >= im.cols || int(pt[1])+1 >= im.rows)
        return false;

      patch(v,u) = getInterpolated(im, pt);
    }
  }

  return true;
}

/**
 * warpPatchHomography
 */
template <typename T>
bool warpPatchHomography(const unsigned char *im, int im_rows, int im_cols, const T H[9], unsigned char *patch, int p_rows, int p_cols)
{
  T pt[2];

  for (int v=0; v<p_rows; v++)
  {
    for (int u=0; u<p_cols; u++)
    {
      mapPoint(u,v, H, pt[0], pt[1]);

      if ((int)pt[0]<0 || (int)pt[1]<0 || int(pt[0])+1>=im_cols || int(pt[1])+1>=im_rows)
      {
        return false;
      }

      patch[v*p_cols+u] = getInterpolated(im, im_rows, im_cols, pt);
    }
  }

  return true;
}

} //--END--

#endif




