#include <opencv2/opencv.hpp>

#ifndef UTILS_H
#define UTILS_H

using namespace cv;

class Subdiv2DIndex : public Subdiv2D
{
public:
  Subdiv2DIndex(Rect rectangle);
  Subdiv2DIndex();
  //Source code of Subdiv2D: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/subdivision2d.cpp#L762
  //The implementation tweaks getTrianglesList() so that only the indice of the triangle inside the image are returned
  void
  getTrianglesIndices(std::vector<int> &ind) const;
};

inline float CosineAngle(cv::Mat &n1, cv::Mat &n2)
{
  return acos(n1.dot(n2) / (norm(n1) * norm(n2)));
}

#endif
