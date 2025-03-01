//
// Created by RUPC on 2022/9/20.
//
#include "rev_perspective.h"

uint8_t gray_pers_image[RESULT_H][RESULT_W];
uint8_t contrast_pers_image[RESULT_H][RESULT_W];
uint8_t binary_pers_image[RESULT_H][RESULT_W];
uint8_t gray_binary_pers_image[RESULT_H][RESULT_W];
double change_un_Mat[3][3] ={{-0.143541,0.172249,-11.856459},{-0.000000,0.005383,-3.581340},{-0.000000,0.001794,-0.193780}};
uint8_t* mapping_table[RESULT_H][RESULT_W];

// 建立透视变换指针表
void InitLookupTable(void) {
    for (int i = 0; i < RESULT_W; i++) {
        for (int j = 0; j < RESULT_H; j++) {
            int local_x = (int) ((change_un_Mat[0][0] * i * 2 + change_un_Mat[0][1] * j * 2 + change_un_Mat[0][2])
                                 / (change_un_Mat[2][0] * i * 2 + change_un_Mat[2][1] * j * 2 + change_un_Mat[2][2]));
            int local_y = (int) ((change_un_Mat[1][0] * i * 2 + change_un_Mat[1][1] * j * 2 + change_un_Mat[1][2])
                                 / (change_un_Mat[2][0] * i * 2 + change_un_Mat[2][1] * j * 2 + change_un_Mat[2][2]));
            if (local_x >= 0 && local_y >= 0 && local_y < MT9V03X_HS && local_x < MT9V03X_WS) {
                mapping_table[j][i] = &mt9v03x_image[local_y][local_x];
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