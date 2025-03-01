//
// Created by ashkore on 2023/9/22.
//

#ifndef SMART_CAR_CAMERA_MY_CV2_H
#define SMART_CAR_CAMERA_MY_CV2_H

#include "config.h"

void my_cv2_threshold(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t threshold, uint8_t maxvalue);

void my_cv2_gaussian_blur_3x3(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2,uint16_t y2);

void my_cv2_calculateGradient_sobel_5x5(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void my_cv2_nonMaximumSuppression(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void my_cv2_doubleThreshold(uint8_t *dst_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t lowThreshold, uint8_t highThreshold);

void my_cv2_checkConnectivity(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void my_cv2_Canny(uint8_t *dst_image, uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void my_cv2_gaussian_blur_5x5(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void my_cv2_enhanceAlphaBeta(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, float alpha, float beta);

void my_cv2_fix_center_high_light(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, float factor);

void my_cv2_equalization_hist(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
#endif //SMART_CAR_CAMERA_MY_CV2_H
