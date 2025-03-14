//
// Created by RUPC on 2022/9/20.
//
#include "rev_perspective.h"

uint8_t gray_pers_image[RESULT_H][RESULT_W];
uint8_t contrast_pers_image[RESULT_H][RESULT_W];
uint8_t binary_pers_image[RESULT_H][RESULT_W];
uint8_t gray_binary_pers_image[RESULT_H][RESULT_W];
double change_un_Mat[3][3] ={{-1.828243,1.594090,-66.994795},{0.042251,0.281081,-48.918301},{0.000288,0.011527,-0.859113}};
uint8_t* mapping_table[RESULT_H][RESULT_W];

// 建立透视变换指针表
void InitLookupTable(void) {
    for (int i = 0; i < RESULT_W; i++) {
        for (int j = 0; j < RESULT_H; j++) {
            int local_x = (int) ((change_un_Mat[0][0] * i + change_un_Mat[0][1] * j + change_un_Mat[0][2])
                                 / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2]));
            int local_y = (int) ((change_un_Mat[1][0] * i + change_un_Mat[1][1] * j + change_un_Mat[1][2])
                                 / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2]));
            if (local_x >= 0 && local_y >= 0 && local_y < LQU_CAM_H && local_x < LQU_CAM_W) {
                mapping_table[j][i] = &LQU_CAM_image[local_y][local_x];
            } else {
                mapping_table[j][i] = nullptr;
            }
        }
    }
}

// 直接查表
void ImagePerspective(void) {
    for (int i = 0; i < RESULT_W; i++) {
        for (int j = 0; j < RESULT_H; j++) {
            if (mapping_table[j][i] != nullptr) {
                gray_pers_image[j][i] = *mapping_table[j][i];
            } else {
                gray_pers_image[j][i] = 0;
            }
        }
    }
}