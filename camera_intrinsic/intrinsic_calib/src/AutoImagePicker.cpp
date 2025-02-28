/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include "AutoImagePicker.hpp"

#define CHESSBOARD_MIN_AREA_PORTION 0.04
#define IMAGE_MARGIN_PERCENT 0.1
#define CHESSBOARD_MIN_AREA_CHANGE_PARTION_THRESH 0.005
#define CHESSBOARD_MIN_MOVE_THRESH 0.08
#define CHESSBOARD_MIN_ANGLE_CHANGE_THRESH 5.0
#define CHESSBOARD_MIN_ANGLE 40.0
#define MAX_SELECTED_SAMPLE_NUM 45
#define REGION_MAX_SELECTED_SAMPLE_NUM 15

AutoImagePicker::AutoImagePicker(const int &img_width, const int &img_height,
                                 const int &board_width,
                                 const int &board_height) {
  img_width_ = img_width;
  img_height_ = img_height;
  board_width_ = board_width;
  board_height_ = board_height;
  // area_box_是一个二维数组，宽高与图像宽高相同，初始化为5
  area_box_ = std::vector<std::vector<int>>(img_height_,
                                            std::vector<int>(img_width_, 5));
  // encourage edge position 
  // 为了鼓励边缘位置，将边缘的像素值设置为1
  int d_imgh = static_cast<int>(IMAGE_MARGIN_PERCENT * img_height);
  int d_imgw = static_cast<int>(IMAGE_MARGIN_PERCENT * img_width);
  for (int i = d_imgh; i < img_height - d_imgh; i++) {
    for (int j = d_imgw; j < img_width - d_imgw; j++) {
      area_box_[i][j] = 1; // 中心区域设置为1 边缘位置默认为5
    }
  }
  candidate_board_.clear(); // 候选棋盘格
  board_square_box_.clear(); // 
}
// 输入为检测到的角点
bool AutoImagePicker::addImage(const std::vector<cv::Point2f> &image_corners) {
  cv::Point p1 = image_corners[0];
  cv::Point p2 = image_corners[0 + board_width_ - 1];
  cv::Point p3 = image_corners[image_corners.size() - board_width_];
  cv::Point p4 = image_corners[image_corners.size() - 1];
  BoardSquare current_square(p1, p2, p3, p4);
  if (!checkValidity(current_square))
    return false;
  if (checkMoveThresh(current_square) || checkAreaThresh(current_square)) {
    // current_square.index = candidate_board_.size();
    candidate_board_.push_back(current_square);
    this->fillAreaBox(current_square);
  } else {
    if (!checkPoseAngleThresh(current_square))
      return false;
    candidate_board_.push_back(current_square);
    this->fillAreaBox(current_square);
  }
  return true;
}

bool AutoImagePicker::status() {
  if (candidate_board_.size() >= MAX_SELECTED_SAMPLE_NUM) {
    std::cout << "[ImagePicker] Enough selected images.\n";
    return true;
  }
  return false;
}

bool AutoImagePicker::checkValidity(const BoardSquare &board) {
  if (board.area < CHESSBOARD_MIN_AREA_PORTION * img_width_ * img_height_)
    return false; // board is too far
  if (board.angle_left_top < CHESSBOARD_MIN_ANGLE ||
      board.angle_right_top < CHESSBOARD_MIN_ANGLE ||
      board.angle_left_bottom < CHESSBOARD_MIN_ANGLE ||
      board.angle_right_bottom < CHESSBOARD_MIN_ANGLE)
    return false; // board is too inclined
  return true;
}

bool AutoImagePicker::checkMoveThresh(const BoardSquare &board) {
  for (size_t i = 0; i < candidate_board_.size(); i++) {
    auto candidate = candidate_board_[i];
    double dx = board.midpoint.x - candidate.midpoint.x;
    double dy = board.midpoint.y - candidate.midpoint.y;
    double dist = sqrt(dx * dx + dy * dy);
    // 自适应性考虑，最终阈值 ≈ 图像特征长度的8%
    // (width + height) / 2代表了图像尺寸的一个特征长度
    if (dist <
        CHESSBOARD_MIN_MOVE_THRESH * double(img_width_ + img_height_) / 2.0)
      return false;
  }
  return true;
}

bool AutoImagePicker::checkAreaThresh(const BoardSquare &board) {
  int score = 0;
  // 计算棋盘格的得分 处于边缘的棋盘格得分更高
  for (int i = std::max(0, int(board.min_y));
       i < std::min(img_height_, int(board.max_y)); i++) {
    for (int j = std::max(0, int(board.min_x));
         j < std::min(img_width_, int(board.max_x)); j++) {
      score += area_box_[i][j];
    }
  }
  if (score <
      CHESSBOARD_MIN_AREA_CHANGE_PARTION_THRESH * img_width_ * img_height_)
    return false;
  else
    return true;
}

bool AutoImagePicker::checkPoseAngleThresh(const BoardSquare &board) {
  for (size_t i = 0; i < candidate_board_.size(); i++) {
    auto candidate = candidate_board_[i];
    double dx = board.midpoint.x - candidate.midpoint.x;
    double dy = board.midpoint.y - candidate.midpoint.y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist <
        CHESSBOARD_MIN_MOVE_THRESH * double(img_width_ + img_height_) / 2.0) {
      if (fabs(board.angle_left_top - candidate.angle_left_top) <
              CHESSBOARD_MIN_ANGLE_CHANGE_THRESH &&
          fabs(board.angle_right_top - candidate.angle_right_top) <
              CHESSBOARD_MIN_ANGLE_CHANGE_THRESH &&
          fabs(board.angle_left_bottom - candidate.angle_left_bottom) <
              CHESSBOARD_MIN_ANGLE_CHANGE_THRESH &&
          fabs(board.angle_right_bottom - candidate.angle_right_bottom) <
              CHESSBOARD_MIN_ANGLE_CHANGE_THRESH)
        return false;
    }
  }
  return true;
}

int AutoImagePicker::getAreaScore(const BoardSquare &board) {
  int score = 0;
  for (int i = std::max(0, int(board.min_y));
       i < std::min(img_height_, int(board.max_y)); i++) {
    for (int j = std::max(0, int(board.min_x));
         j < std::min(img_width_, int(board.max_x)); j++) {
      score += area_box_[i][j];
    }
  }
  return score;
}

void AutoImagePicker::fillAreaBox(const BoardSquare &board) {
  for (int i = std::max(0, int(board.min_y));
       i < std::min(img_height_, int(board.max_y)); i++) {
    for (int j = std::max(0, int(board.min_x));
         j < std::min(img_width_, int(board.max_x)); j++) {
      area_box_[i][j] = 0;
    }
  }
}