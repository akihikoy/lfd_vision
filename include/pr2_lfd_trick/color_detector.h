//-------------------------------------------------------------------------------------------
/*! \file    color_detector.h
    \brief   Color detector.
    \author  Akihiko Yamaguchi, xakiyam@gmail.com
    \version 0.1
    \date    Jun.05, 2014
*/
//-------------------------------------------------------------------------------------------
#ifndef color_detector_h
#define color_detector_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // CV_BGR2HSV
#include <vector>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------


//===========================================================================================
/* Detector of specific colors from a source image. */
class TColorDetector
//===========================================================================================
{
public:
  TColorDetector()
    : color_code_            (CV_BGR2HSV),
      using_blur_            (true),
      gaussian_kernel_size_  (7,7),
      gaussian_sigma_x_      (2.5),
      gaussian_sigma_y_      (2.5),
      dilations_erosions_    (2),
      lookup_table_          (256, 1, CV_8UC3)
    {
      lookup_table_= cv::Scalar(0,0,0);
    }
  ~TColorDetector()  {}

  /*! Setup colors to be detected.
    \param colors      Colors to be detected.
    \param col_radius  Threshold of each color.  If col_radius[i]<0, we treate as 0==256 (i.e. cyclic).  */
  void SetupColors(const std::vector<cv::Vec3b> &colors, const cv::Vec3s &col_radius);

  /*! Detect specific colors from the source image, and return the mask image (0 or 255).
    \param src_img  Input image.  */
  cv::Mat Detect(const cv::Mat &src_img);


  int ColorCode() const {return color_code_;}
  bool UsingBlur() const {return using_blur_;}
  const cv::Size& GaussianKernelSize() const {return gaussian_kernel_size_;}
  const double& GaussianSigmaX() const {return gaussian_sigma_x_;}
  const double& GaussianSigmaY() const {return gaussian_sigma_y_;}
  int DilationsErosions() const {return dilations_erosions_;}

  void SetColorCode(int v)  {color_code_= v;}
  void SetUsingBlur(bool v)  {using_blur_= v;}
  void SetGaussianKernelSize(const cv::Size &v)  {gaussian_kernel_size_= v;}
  void SetGaussianSigmaY(const double &v)  {gaussian_sigma_y_= v;}
  void SetGaussianSigmaX(const double &v)  {gaussian_sigma_x_= v;}
  void SetDilationsErosions(int v)  {dilations_erosions_= v;}

protected:

  // Parameters

  //! Color conversion code where the colors are detected, like CV_BGR2HSV, CV_BGR2Lab.
  int color_code_;
  //! If true, the Gaussian blur is applied to the input image.
  bool using_blur_;
  //! Kernel size of the Gaussian blur.
  cv::Size gaussian_kernel_size_;
  //! X-std dev of the Gaussian blur.
  double gaussian_sigma_x_;
  //! Y-std dev of the Gaussian blur.
  double gaussian_sigma_y_;
  //! Size of dilations/erosions.
  int dilations_erosions_;


  cv::Mat lookup_table_;

};
//-------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // color_detector_h
//-------------------------------------------------------------------------------------------
