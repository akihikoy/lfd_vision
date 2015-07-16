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
#include <list>
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
  TColorDetector(const TColorDetector &src)
    : color_code_            (src.color_code_           ),
      using_blur_            (src.using_blur_           ),
      gaussian_kernel_size_  (src.gaussian_kernel_size_ ),
      gaussian_sigma_x_      (src.gaussian_sigma_x_     ),
      gaussian_sigma_y_      (src.gaussian_sigma_y_     ),
      dilations_erosions_    (src.dilations_erosions_   ),
      lookup_table_          (256, 1, CV_8UC3)
    {
      src.lookup_table_.copyTo(lookup_table_);
    }
  ~TColorDetector()  {}

  /*! Setup colors to be detected.
    \param colors      Colors to be detected.
    \param col_radius  Threshold of each color.  If col_radius[i]<0, we treate as 0==256 (i.e. cyclic).  */
  void SetupColors(const std::vector<cv::Vec3b> &colors, const cv::Vec3s &col_radius);

  /*! Detect specific colors from the source image, and return the mask image (0 or 255).
    \param src_img  Input image.  */
  cv::Mat Detect(const cv::Mat &src_img) const;


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


//===========================================================================================
/* Multiple color detector. */
class TMultipleColorDetector
//===========================================================================================
{
public:
  TMultipleColorDetector();

  void Setup(int num_detectors);

  void Reset();

  // Do detection
  void Detect(const cv::Mat &frame, int mode, bool verbose=true);
  void Draw(cv::Mat &img_draw) const;

  void CameraWindowMouseCallback(int active_col_index, int event, int x, int y, int flags);
  void MaskWindowMouseCallback(int active_col_index, int event, int x, int y, int flags);

  void LoadColors(int index, const std::string &file_name);
  void SaveColors(int index, const std::string &file_name);

  const TColorDetector& Detector(int index) const {return col_detectors_[index];}
  TColorDetector& RefDetector(int index)  {return col_detectors_[index];}

  const double& BlockAreaMin() const {return block_area_min_;}
  void SetBlockAreaMin(const double &v)  {block_area_min_= v;}

  // Get information
  int Size() const {return col_detectors_.size();}
  const double& Ratio(int index) const {return data_ratio_[index];}
  const double& Area(int index) const {return data_area_[index];}
  cv::Point Center(int index) const {return cv::Point(CenterX(index), CenterY(index));}
  const double& CenterX(int index) const {return data_center_xy_[2*index];}
  const double& CenterY(int index) const {return data_center_xy_[2*index+1];}
  cv::Rect Bound(int index) const {return cv::Rect(data_bound_[4*index+0], data_bound_[4*index+1], data_bound_[4*index+2], data_bound_[4*index+3]);}

  // Raw data access
  const std::vector<double>& DataRatio() const {return data_ratio_;}  // [ratio] * size
  const std::vector<double>& DataArea() const {return data_area_;}  // [area] * size
  const std::vector<double>& DataCenterXY() const {return data_center_xy_;}  // [x,y] * size
  const std::vector<double>& DataBound() const {return data_bound_;}  // [x,y,lx,ly] * size

  const std::vector<int>&  NumsBlocks() const {return nums_blocks_;}
  const std::list<double>& BlocksArea() const {return blocks_area_;}  // [area] * sum(nums_blocks_)
  const std::list<double>& BlocksCenterXY() const {return blocks_center_xy_;}  // [x,y] * sum(nums_blocks_)

  // Set camera window for mouse event
  void SetCameraWindow(cv::Mat &camera_frame)  {camera_window_= &camera_frame;}

private:
  std::vector<TColorDetector>  col_detectors_;
  std::vector<std::vector<cv::Vec3b> >  detect_colors_;
  std::vector<int> nonzero_base_;
  std::vector<cv::Mat>  mask_imgs_;

  // Only largest contours
  std::vector<double> data_ratio_;
  std::vector<double> data_area_;
  std::vector<double> data_center_xy_;  // [x,y] * size
  std::vector<double> data_bound_;  // [x,y,lx,ly] * size

  // All contours of all detectors
  std::vector<int>  nums_blocks_;
  std::list<double> blocks_area_;  // [area] * sum(nums_blocks_)
  std::list<double> blocks_center_xy_;  // [x,y] * sum(nums_blocks_)

  cv::Mat  *camera_window_;
  cv::Vec3s  col_radius_;
  double block_area_min_;  // Minimum value of block area

};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // color_detector_h
//-------------------------------------------------------------------------------------------
