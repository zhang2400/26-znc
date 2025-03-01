//
// Created by ashkore on 2023/9/22.
//

// 文件简介     本文件模拟了opencv中的一些函数

#include "my_cv2.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将image二值化
// 参数说明     *image          图像数组指针
// 参数说明     width       图像宽度
// 参数说明     height      图像高度
// 参数说明     threshold       二值化阈值
// 参数说明     maxvalue        最大值
// 返回参数     void
// 备注信息     用于将图像二值化
//-------------------------------------------------------------------------------------------------------------------
void my_cv2_threshold(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t threshold, uint8_t maxvalue) {
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            if (*(image + i * image_width + j) > threshold) {
                *(image + i * image_width + j) = maxvalue;
            } else {
                *(image + i * image_width + j) = 0;
            }

        }
    }
}

// 高斯模糊，3x3，用于图像平滑，去噪，但是会使图像变模糊
void my_cv2_gaussian_blur_3x3(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2,uint16_t y2) {
    uint8_t kernel[3][3] = {
            {1, 2, 1},
            {2, 4, 2},
            {1, 2, 1}
    };
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            int sum = 0;
            for (uint16_t k = 0; k < 3; k++) {
                for (uint16_t l = 0; l < 3; l++) {
                    sum += *(src_image + (i + k - 1) * image_width + j + l - 1) * kernel[k][l];
                }
            }
            *(dst_image + i * image_width + j) = (uint8_t) (sum / 16);
        }
    }
}

// 计算梯度，用于canny边缘检测
void my_cv2_calculateGradient_sobel_5x5(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    int16_t sobel_x[5][5] = {
            {-1, 0, 2, 0, -1},
            {-4, 0, 8, 0, -4},
            {-6, 0, 12, 0, -6},
            {-4, 0, 8, 0, -4},
            {-1, 0, 2, 0, -1}
    };
    int16_t sobel_y[5][5] = {
            {-1, -4, -6, -4, -1},
            {0, 0, 0, 0, 0},
            {2, 8, 12, 8, 2},
            {0, 0, 0, 0, 0},
            {-1, -4, -6, -4, -1}
    };
    for (uint16_t i = y1 + 2; i < y2 - 2; i++) {
        for (uint16_t j = x1 + 2; j < x2 - 2; j++) {
            int sum_x = 0;
            int sum_y = 0;
            for (uint16_t k = 0; k < 5; k++) {
                for (uint16_t l = 0; l < 5; l++) {
                    sum_x += *(src_image + (i + k - 2) * image_width + j + l - 2) * sobel_x[k][l];
                    sum_y += *(src_image + (i + k - 2) * image_width + j + l - 2) * sobel_y[k][l];
                }
            }
            *(dst_image + i * image_width + j) = (uint8_t) (sqrt(sum_x * sum_x + sum_y * sum_y) / 4);
        }
    }
}

void my_cv2_calculateGradient_Laplacian_3x3(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    int16_t laplacian[3][3] = {
            {0, 1, 0},
            {1, -4, 1},
            {0, 1, 0}
    };
    for (uint16_t i = y1 + 1; i < y2 - 1; i++) {
        for (uint16_t j = x1 + 1; j < x2 - 1; j++) {
            int sum = 0;
            for (uint16_t k = 0; k < 3; k++) {
                for (uint16_t l = 0; l < 3; l++) {
                    sum += *(src_image + (i + k - 1) * image_width + j + l - 1) * laplacian[k][l];
                }
            }
            *(dst_image + i * image_width + j) = (uint8_t) (sum / 4);
        }
    }
}

void my_cv2_calculateGradient_PreWitt_3x3(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    int16_t prewitt_x[3][3] = {
            {-1, 0, 1},
            {-1, 0, 1},
            {-1, 0, 1}
    };
    int16_t prewitt_y[3][3] = {
            {-1, -1, -1},
            {0, 0, 0},
            {1, 1, 1}
    };
    for (uint16_t i = y1 + 1; i < y2 - 1; i++) {
        for (uint16_t j = x1 + 1; j < x2 - 1; j++) {
            int sum_x = 0;
            int sum_y = 0;
            for (uint16_t k = 0; k < 3; k++) {
                for (uint16_t l = 0; l < 3; l++) {
                    sum_x += *(src_image + (i + k - 1) * image_width + j + l - 1) * prewitt_x[k][l];
                    sum_y += *(src_image + (i + k - 1) * image_width + j + l - 1) * prewitt_y[k][l];
                }
            }
            *(dst_image + i * image_width + j) = (uint8_t) (sqrt(sum_x * sum_x + sum_y * sum_y) / 3);
        }
    }
}

// 非极大值抑制，用于canny边缘检测
void my_cv2_nonMaximumSuppression(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            if(i == 0 || i == y2 - 1 || j == 0 || j == x2 - 1) {
                *(dst_image + i * image_width + j) = 0;
                continue;
            }
            int angle = (int)(atan2(*(src_image + i * image_width + j + 1) - *(src_image + i * image_width + j - 1), *(src_image + (i + 1) * image_width + j) - *(src_image + (i - 1) * image_width + j)) * 180 / 3.1415926);
            if ((angle > -22.5 && angle <= 22.5) || (angle > 157.5 && angle <= -157.5)) {
                if (*(src_image + i * image_width + j) > *(src_image + i * image_width + j + 1) && *(src_image + i * image_width + j) > *(src_image + i * image_width + j - 1)) {
                    *(dst_image + i * image_width + j) = *(src_image + i * image_width + j);
                } else {
                    *(dst_image + i * image_width + j) = 0;
                }
            } else if ((angle > 22.5 && angle <= 67.5) || (angle > -157.5 && angle <= -112.5)) {
                if (*(src_image + i * image_width + j) > *(src_image + (i - 1) * image_width + j + 1) && *(src_image + i * image_width + j) > *(src_image + (i + 1) * image_width + j - 1)) {
                    *(dst_image + i * image_width + j) = *(src_image + i * image_width + j);
                } else {
                    *(dst_image + i * image_width + j) = 0;
                }
            } else if ((angle > 67.5 && angle <= 112.5) || (angle > -112.5 && angle <= -67.5)) {
                if (*(src_image + i * image_width + j) > *(src_image + (i - 1) * image_width + j) && *(src_image + i * image_width + j) > *(src_image + (i + 1) * image_width + j)) {
                    *(dst_image + i * image_width + j) = *(src_image + i * image_width + j);
                } else {
                    *(dst_image + i * image_width + j) = 0;
                }
            } else {
                if (*(src_image + i * image_width + j) > *(src_image + (i - 1) * image_width + j - 1) &&
                    *(src_image + i * image_width + j) > *(src_image + (i + 1) * image_width + j + 1)) {
                    *(dst_image + i * image_width + j) = *(src_image + i * image_width + j);
                } else {
                    *(dst_image + i * image_width + j) = 0;
                }
            }
        }
    }
}

void my_cv2_doubleThreshold(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t lowThreshold, uint8_t highThreshold){
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            if (*(image + i * image_width + j) > highThreshold) {
                *(image + i * image_width + j) = 255;
            } else if (*(image + i * image_width + j) > lowThreshold) {
                *(image + i * image_width + j) = 127;
            } else {
                *(image + i * image_width + j) = 0;
            }
        }
    }
}

void my_cv2_checkConnectivity(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            if (*(image + i * image_width + j) == 127) {
                if (*(image + (i - 1) * image_width + j - 1) == 255 || *(image + (i - 1) * image_width + j) == 255 || *(image + (i - 1) * image_width + j + 1) == 255 ||
                    *(image + i * image_width + j - 1) == 255 || *(image + i * image_width + j + 1) == 255 ||
                    *(image + (i + 1) * image_width + j - 1) == 255 || *(image + (i + 1) * image_width + j) == 255 || *(image + (i + 1) * image_width + j + 1) == 255) {
                    *(image + i * image_width + j) = 255;
                } else {
                    *(image + i * image_width + j) = 0;
                }
            }
        }
    }
}

// canny边缘检测
void my_cv2_Canny(uint8_t *dst_image, uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
//    uint8_t gradient[40][94];
    my_cv2_gaussian_blur_3x3((uint8_t *) dst_image, (uint8_t *) src_image, image_width, x1, y1, x2, y2);
    my_cv2_calculateGradient_sobel_5x5((uint8_t *) src_image, (uint8_t *) dst_image, image_width, x1, y1, x2, y2);
    my_cv2_nonMaximumSuppression((uint8_t *)dst_image,(uint8_t *)src_image, image_width, x1, y1, x2, y2);
}

// 高斯模糊，5x5，用于图像平滑，去噪，但是会使图像变模糊
void my_cv2_gaussian_blur_5x5(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint8_t kernel[5][5] = {
            {1, 4, 6, 4, 1},
            {4, 16, 24, 16, 4},
            {6, 24, 36, 24, 6},
            {4, 16, 24, 16, 4},
            {1, 4, 6, 4, 1}
    };
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            int sum = 0;
            for (uint16_t k = 0; k < 5; k++) {
                for (uint16_t l = 0; l < 5; l++) {
                    sum += *(src_image + (i + k - 2) * image_width + j + l - 2) * kernel[k][l];
                }
            }
            *(dst_image + i * image_width + j) = (uint8_t) (sum / 256);
        }
    }
}

// 增强对比度和亮度，可能会使图像变得过曝
void my_cv2_enhanceAlphaBeta(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, float alpha, float beta) {
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            int enhance = (int) ((*(image + i * image_width + j) - 127) * alpha + beta);
            int result =  enhance + 127;
            if(result > 255) {
                result = 255;
            } else if(result < 0) {
                result = 0;
            }
            *(image + i * image_width + j) = (uint8_t) result;
        }
    }
}

// 修复摄像头传回的图像中间亮四周暗的问题，可能会丢失四周图像细节
void my_cv2_fix_center_high_light(uint8_t *image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, float factor) {
    int center_x = (x1 + x2) / 2;
    int center_y = (y1 + y2) / 2;
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            int distance_x = center_x - j;
            int distance_y = center_y - i;
            float distance = sqrt(distance_x * distance_x + distance_y * distance_y);
            *(image + i * image_width + j) += distance * factor;
        }
    }
}

void my_cv2_equalization_hist(uint8_t *dst_image, const uint8_t *src_image, uint16_t image_width, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    int hist[256] = {0};
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            hist[*(src_image + i * image_width + j)]++;
        }
    }
    int sum = 0;
    int lut[256] = {0};
    for (int i = 0; i < 256; i++) {
        sum += hist[i];
        lut[i] = (int) (sum * 255 / (x2 - x1) / (y2 - y1));
    }
    for (uint16_t i = y1; i < y2; i++) {
        for (uint16_t j = x1; j < x2; j++) {
            *(dst_image + i * image_width + j) = (uint8_t) lut[*(src_image + i * image_width + j)];
        }
    }
}
