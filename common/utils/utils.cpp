//
// Created by sarkar on 19.06.15.
//
#include "utils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/functional/hash.hpp>

using namespace std;
using namespace cv;

namespace od
{


  std::vector<std::string> myglob(const std::string &pat)
  {
    using namespace std;
    glob_t glob_result;
    glob(pat.c_str(), GLOB_TILDE, NULL, &glob_result);
    vector<string> ret;
    for(unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
      ret.push_back(string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return ret;
  }

  void normL2(cv::Mat &descriptors)
  {
    for (int r = 0; r < descriptors.rows; r++)
    {
      float norm=0;
      for (int c = 0; c < descriptors.cols; c++) norm+=(descriptors.at<float>(r, c)*descriptors.at<float>(r, c));
      norm = 1.0/sqrt(norm);
      for (int c = 0; c < descriptors.cols; c++) descriptors.at<float>(r, c)*=norm;
    }
  }

  cv::Mat makeCanvasMultiImage(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows)
  {
    int N = vecMat.size();
    nRows  = nRows > N ? N : nRows;
    int edgeThickness = 10;
    int imagesPerRow = ceil(double(N) / nRows);
    int resizeHeight = floor(2.0 * ((floor(double(windowHeight - edgeThickness) / nRows)) / 2.0)) - edgeThickness;
    int maxRowLength = 0;

    std::vector<int> resizeWidth;
    for (int i = 0; i < N;) {
      int thisRowLen = 0;
      for (int k = 0; k < imagesPerRow; k++) {
        double aspectRatio = double(vecMat[i].cols) / vecMat[i].rows;
        int temp = int( ceil(resizeHeight * aspectRatio));
        resizeWidth.push_back(temp);
        thisRowLen += temp;
        if (++i == N) break;
      }
      if ((thisRowLen + edgeThickness * (imagesPerRow + 1)) > maxRowLength) {
        maxRowLength = thisRowLen + edgeThickness * (imagesPerRow + 1);
      }
    }
    int windowWidth = maxRowLength;
    cv::Mat canvasImage(windowHeight, windowWidth, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int k = 0, i = 0; i < nRows; i++) {
      int y = i * resizeHeight + (i + 1) * edgeThickness;
      int x_end = edgeThickness;
      for (int j = 0; j < imagesPerRow && k < N; k++, j++) {
        int x = x_end;
        cv::Rect roi(x, y, resizeWidth[k], resizeHeight);
        cv::Size s = canvasImage(roi).size();
        // change the number of channels to three
        cv::Mat target_ROI(s, CV_8UC3);
        if (vecMat[k].channels() != canvasImage.channels()) {
          if (vecMat[k].channels() == 1) {
            cv::cvtColor(vecMat[k], target_ROI, CV_GRAY2BGR);
          }
        }
        cv::resize(target_ROI, target_ROI, s);
        if (target_ROI.type() != canvasImage.type()) {
          target_ROI.convertTo(target_ROI, canvasImage.type());
        }
        target_ROI.copyTo(canvasImage(roi));
        x_end += resizeWidth[k] + edgeThickness;
      }
    }
    return canvasImage;
  }

  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.75;
  int thickness_font = 2;
  Scalar red(0, 0, 255);
  Scalar green(0,255,0);
  Scalar blue(255,0,0);
  Scalar yellow(0,255,255);

  void drawTextTopLeft(cv::Mat image, std::string text, cv::Scalar color)
  {
    cv::putText(image, text, cv::Point(25,50), fontFace, fontScale, color, thickness_font, 16);
  }

  cv::Mat makeCanvasMultiImages(std::vector<cv::Mat>& imgs, cv::Size cellSize, std::vector<std::string> messages)
  {

    bool printmssg = false;
    if (messages.size() == imgs.size()) printmssg = true;

    float nImgs=imgs.size();
    int   imgsInRow=ceil(sqrt(nImgs));     // You can set this explicitly
    int   imgsInCol=ceil(nImgs/imgsInRow); // You can set this explicitly

    int resultImgW=cellSize.width*imgsInRow;
    int resultImgH=cellSize.height*imgsInCol;

    Mat resultImg=Mat::zeros(resultImgH,resultImgW,CV_8UC3);
    int ind=0;
    Mat tmp;
    for(int i=0;i<imgsInCol;i++)
    {
      for(int j=0;j<imgsInRow;j++)
      {
        if(ind<imgs.size())
        {
          int cell_row=i*cellSize.height;
          int cell_col=j*cellSize.width;

          resize(imgs[ind], tmp, cellSize);

          if (printmssg) drawTextTopLeft(tmp, messages[ind], yellow);

          tmp.copyTo(resultImg(Range(cell_row,cell_row+cellSize.height),Range(cell_col,cell_col+cellSize.width)));
        }
        ind++;
      }
    }

    return resultImg;
  }


  cv::Scalar getHashedColor(std::string name, int constadd = 100)
  {
    boost::hash<std::string> string_hash;
    int hashed = string_hash(name);

    return CV_RGB((hashed + constadd) % 255, (hashed + 2*constadd) % 255, (hashed + 3*constadd) % 255);
  }

  cv::Mat makeCanvasMultiImage1(std::vector<cv::Mat>& vecMat, int windowHeight, int nRows)
  {
    int N = vecMat.size();
    nRows  = nRows > N ? N : nRows;
    int edgeThickness = 10;
    int imagesPerRow = ceil(double(N) / nRows);
    int resizeHeight = floor(2.0 * ((floor(double(windowHeight - edgeThickness) / nRows)) / 2.0)) - edgeThickness;
    int maxRowLength = 0;

    std::vector<int> resizeWidth;
    for (int i = 0; i < N;) {
      int thisRowLen = 0;
      for (int k = 0; k < imagesPerRow; k++) {
        double aspectRatio = double(vecMat[i].cols) / vecMat[i].rows;
        int temp = int( ceil(resizeHeight * aspectRatio));
        resizeWidth.push_back(temp);
        thisRowLen += temp;
        if (++i == N) break;
      }
      if ((thisRowLen + edgeThickness * (imagesPerRow + 1)) > maxRowLength) {
        maxRowLength = thisRowLen + edgeThickness * (imagesPerRow + 1);
      }
    }
    int windowWidth = maxRowLength;
    cv::Mat canvasImage(windowHeight, windowWidth, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int k = 0, i = 0; i < nRows; i++) {
      int y = i * resizeHeight + (i + 1) * edgeThickness;
      int x_end = edgeThickness;
      for (int j = 0; j < imagesPerRow && k < N; k++, j++) {
        int x = x_end;
        cv::Rect roi(x, y, resizeWidth[k], resizeHeight);
        cv::Size s = canvasImage(roi).size();
        // change the number of channels to three
        cv::Mat target_ROI(s, CV_8UC3);
        if (vecMat[k].channels() != canvasImage.channels()) {
          if (vecMat[k].channels() == 1) {
            cv::cvtColor(vecMat[k], target_ROI, CV_GRAY2BGR);
          }
        }
        cv::resize(target_ROI, target_ROI, s);
        if (target_ROI.type() != canvasImage.type()) {
          target_ROI.convertTo(target_ROI, canvasImage.type());
        }
        target_ROI.copyTo(canvasImage(roi));
        x_end += resizeWidth[k] + edgeThickness;
      }
    }
    return canvasImage;
  }
}

