//
// Created by ashkore on 2023/9/23.
//

#include "image_process.h"

uint8_t    mt9v03x_image[MT9V03X_HS][MT9V03X_WS];
uint8_t    LQU_CAM_image[LQU_CAM_H][LQU_CAM_W];
uint8_t    gray_image[60][80];
uint8_t    binary_image[60][80];
uint8_t    binary_image_bak[60][80];
uint8_t    contrast_image[60][80];
uint8_t    gray_binary_image[60][80];
float left_corner_angle;  // 左拐角角度，用于extern到其他文件
float right_corner_angle;  // 右拐角角度，用于extern到其他文件
uint8_t left_border[100][2];  // 左边界
uint8_t right_border[100][2];  // 右边界
int left_dirs[100];  // 左爬线方向
int right_dirs[100];  // 右爬线方向
uint8_t left_border_pers[100][2];  // 左边界
uint8_t right_border_pers[100][2];  // 右边界
int left_dirs_pers[100];  // 左爬线方向
int right_dirs_pers[100];  // 右爬线方向
uint8_t left_border_single[100][2];  // 左边界单Y
uint8_t right_border_single[100][2];  // 右边界单Y
uint8_t middle_line[100][2];  // 中线
uint8_t middle_line_single[100][2];  // 中线单Y
uint8_t middle_line_pers[100][2];  // 中线
uint8_t middle_line_single_pers[100][2];  // 中线单Y
uint8_t distance_middle_line[100][2];  // 最长白列中线
uint8_t distance_middle_line_pers[100][2];  // 透视最长白列中线
uint8_t left_distance[100][2];  // 左距离
uint8_t right_distance[100][2];  // 右距离
uint8_t left_distance_line[100][2];  // 左距离线
uint8_t right_distance_line[100][2];  // 右距离线
uint8_t left_distance_pers[100][2];  // 透视左距离
uint8_t right_distance_pers[100][2];  // 透视右距离
uint8_t left_distance_line_pers[100][2];  // 透视左距离线
uint8_t right_distance_line_pers[100][2];  // 透视右距离线
uint8_t distances[100];  // 距离
uint8_t distances_pers[100];  // 透视距离
uint8_t narrow_line[100][2];  // 窄线(障碍物)

int max_middle_line_height = 0;
int left_border_index = 0;
int left_border_index_pers = 0;
int left_border_single_index = 0;
int right_border_index = 0;
int right_border_index_pers = 0;
int right_border_single_index = 0;
int left_skip_index = 0;
int right_skip_index = 0;
int middle_line_index = 0;
int middle_line_index_pers = 0;
int middle_line_single_index = 0;
int middle_line_single_index_pers = 0;
int left_skip_index_pers = 0;
int right_skip_index_pers = 0;
int distance_index = 0;
int distance_index_pers = 0;
int distance_middle_line_index = 0;
int distance_middle_line_index_pers = 0;
int narrow_line_index = 0;

int bottom_start_x;
int bottom_end_x;
int bottom_start_x_pers;
int bottom_end_x_pers;

int left_reach_edge = 0;
int right_reach_edge = 0;
int t1,t2;

int x_left;
int x_right;

uint8_t road_color = 0;  // 道路颜色
uint8_t border_color = 255;  // 边界颜色

Max_White_Column max_white_column;  // 最大白色列
Max_White_Column max_white_column_pers;
//Max_White_Row max_white_row;  // 最大白色行

int lost_x1;
int lost_x2;
int lost_y1;
int lost_y2;
int left_lost_count;
int right_lost_count;
int left_lost_dir = 0;
int right_lost_dir = 0;

int left_border_x = 0;
int right_border_x = 0;
int left_border_y = 0;
int right_border_y = 0;

int fix_x1 = 0;
int fix_x2 = 0;
int fix_y1 = 0;
int fix_y2 = 0;
int fix_x2_last = 0;

int garage_count = 0;

extern int cornering;

extern VOFA vofa_udp;
void bottom_start_end_x_get(){
    int maxStartIndex = -1;
    int maxEndIndex = -1;
    int currentStartIndex = -1;
    int currentEndIndex = -1;
    int maxLength = 0;
    int currentLength = 0;
    for (int i = 0; i < 80; i++) {
        if (gray_binary_image[58][i] == 255) {
            if (currentStartIndex == -1) {
                currentStartIndex = i;
            }
            currentEndIndex = i;
            currentLength = currentEndIndex - currentStartIndex;
            if (currentLength > maxLength) {
                maxLength = currentLength;
                maxStartIndex = currentStartIndex;
                maxEndIndex = currentEndIndex;
            }
        } else {
            currentStartIndex = -1;
        }
    }
    bottom_start_x = maxStartIndex;
    bottom_end_x = maxEndIndex;
}

void bottom_start_end_x_get_pers(){
    int maxStartIndex = -1;
    int maxEndIndex = -1;
    int currentStartIndex = -1;
    int currentEndIndex = -1;
    int maxLength = 0;
    int currentLength = 0;
    for (int i = 0; i < 40; i++) {
        if (gray_binary_pers_image[57][i] == 255) {
            if (currentStartIndex == -1) {
                currentStartIndex = i;
            }
            currentEndIndex = i;
            currentLength = currentEndIndex - currentStartIndex;
            if (currentLength > maxLength) {
                maxLength = currentLength;
                maxStartIndex = currentStartIndex;
                maxEndIndex = currentEndIndex;
            }
        } else {
            currentStartIndex = -1;
        }
    }
    bottom_start_x_pers = maxStartIndex;
    bottom_end_x_pers = maxEndIndex;
}

void max_white_column_get(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
    int16_t i;
    int16_t j;
    int16_t left_max_white_count = 0;
    int16_t right_max_white_count = 0;
    x1 += 2;
    x2 -= 2;
    if(flag.found_garage == false) {
        for (i = x1; i < x2; i++) {
            int16_t white_count = 0;
            for (j = y2; j >= y1; j--) {
                if (binary_image_bak[j][i] == 0) {
                    white_count++;
                } else {
                    break;
                }
            }
            if (white_count > left_max_white_count) {
                left_max_white_count = white_count;
                max_white_column.start_y = y2;
                max_white_column.end_y = y2 - white_count + 1;
                max_white_column.left_x = i;
                max_white_column.left_height = left_max_white_count;
            }
        }
        for (i = x2; i > x1; i--) {
            int16_t white_count = 0;
            for (j = y2; j >= y1; j--) {
                if (binary_image_bak[j][i] == 0) {
                    white_count++;
                } else {
                    break;
                }
            }
            if (white_count > right_max_white_count) {
                right_max_white_count = white_count;
                max_white_column.right_x = i;
                max_white_column.right_height = right_max_white_count;
            }
        }
    } else {
        for (i = x1; i < x2; i++) {
            int16_t white_count = 0;
            for (j = y2; j >= y1; j--) {
                if (gray_binary_image[j][i] == 255) {
                    white_count++;
                } else {
                    break;
                }
            }
            if (white_count > left_max_white_count) {
                left_max_white_count = white_count;
                max_white_column.start_y = y2;
                max_white_column.end_y = y2 - white_count + 1;
                max_white_column.left_x = i;
                max_white_column.left_height = left_max_white_count;
            }
        }
        for (i = x2; i > x1; i--) {
            int16_t white_count = 0;
            for (j = y2; j >= y1; j--) {
                if (gray_binary_image[j][i] == 255) {
                    white_count++;
                } else {
                    break;
                }
            }
            if (white_count > right_max_white_count) {
                right_max_white_count = white_count;
                max_white_column.right_x = i;
                max_white_column.right_height = right_max_white_count;
            }
        }
    }
}

void max_white_column_get_pers(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
    int16_t i;
    int16_t j;
    int16_t left_max_white_count = 0;
    int16_t right_max_white_count = 0;
    x1 += 2;
    x2 -= 2;
    for (i = x1; i < x2; i++) {
        int16_t white_count = 0;
        for (j = y2; j >= y1; j--) {
            if (binary_pers_image[j][i] == 0) {
                white_count++;
            } else {
                break;
            }
        }
        if (white_count > left_max_white_count) {
            left_max_white_count = white_count;
            max_white_column_pers.start_y = y2;
            max_white_column_pers.end_y = y2 - white_count + 1;
            max_white_column_pers.left_x = i;
            max_white_column_pers.left_height = left_max_white_count;
        }
    }
    for (i = x2; i > x1; i--) {
        int16_t white_count = 0;
        for (j = y2; j >= y1; j--) {
            if (binary_pers_image[j][i] == 0) {
                white_count++;
            } else {
                break;
            }
        }
        if (white_count > right_max_white_count) {
            right_max_white_count = white_count;
            max_white_column_pers.right_x = i;
            max_white_column_pers.right_height = right_max_white_count;
        }
    }
}

//void max_white_row_get(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
//    int16_t i;
//    int16_t j;
//    int16_t top_max_white_count = 0;
//    int16_t bottom_max_white_count = 0;
//    for(i = y1; i < y2; i++){
//        int16_t white_count = 0;
//        int16_t start_x = x1;
//        while(gray_binary_image[i][start_x] == 0) start_x++;
//        for(j = start_x; j < x2; j++){
//            if(gray_binary_image[i][j] == 1){
//                white_count++;
//            } else {
//                break;
//            }
//        }
//        if(white_count > top_max_white_count){
//            top_max_white_count = white_count;
//            max_white_row.start_x = start_x;
//            max_white_row.end_x = j;
//            max_white_row.top_y = i;
//            max_white_row.top_width = top_max_white_count;
//        }
//    }
//    for(i = y2; i > y1; i--){
//        int16_t white_count = 0;
//        j = x1;
//        while(gray_binary_image[i][j] == 0) j++;
//        for(; j < x2; j++){
//            if(gray_binary_image[i][j] == 1){
//                white_count++;
//            } else {
//                break;
//            }
//        }
//        if(white_count > bottom_max_white_count){
//            bottom_max_white_count = white_count;
//            max_white_row.bottom_y = i;
//            max_white_row.bottom_width = bottom_max_white_count;
//        }
//    }
//}

int check_crossroad(){
    flag.found_crossroad = false;
    if(left_lost_count > 3 && right_lost_count > 3 && left_lost_count + right_lost_count > 12 && counter.drive_in_left_roundabout == 0 && counter.drive_in_right_roundabout == 0){
        flag.found_crossroad = true;
        counter.drive_in_left_roundabout = 0;
        counter.drive_in_right_roundabout = 0;
        std::cout << "found crossroad" << std::endl;
        return true;
    }
    return false;
}

int check_roundabout(){
    flag.found_left_roundabout = 0;
    flag.found_right_roundabout = 0;
    int target_left_lost_count = 10;
    int target_right_lost_count = 10;
    if(left_lost_count > target_left_lost_count && right_lost_count == 0 && right_lost_dir == 0){
        flag.found_left_roundabout = 1;
        std::cout << "found left roundabout" << std::endl;
    }
    if(right_lost_count > target_right_lost_count && left_lost_count == 0 && left_lost_dir == 0){
        flag.found_right_roundabout = 1;
        std::cout << "found right roundabout" << std::endl;
    }
    return flag.found_left_roundabout || flag.found_right_roundabout;
}

int check_ramp(){
    flag.found_ramp = false;
    if((left_lost_count < 4 && right_lost_count < 4
        && abs(max_white_column.left_x - max_white_column.right_x) < 10
        && max_white_column.end_y >= 20
        && distances[35] == 0
        && abs(distance_middle_line[0][0] - distance_middle_line[20][0]) < 8)){
        flag.found_ramp = true;
    }
    return flag.found_ramp;
}


int check_obstacle(){
    flag.found_obstacle = false;

    if(narrow_line_index > 10){
        flag.found_obstacle = true;
        float Ax;
        float Ay;
        float Bx;
        float By;
        float Cx;
        float Cy;
        uint8 Aindex = 60 - narrow_line[0][1] - 4;
        uint8 Bindex = 60 - narrow_line[narrow_line_index/2][1] - 2;
        uint8 Cindex = 60 - narrow_line[narrow_line_index - 1][1] + 4;
        Bx = left_distance_line_pers[Bindex][0];
        By = left_distance_line_pers[Bindex][1];
        Ax = left_distance_line_pers[Aindex][0];
        Ay = left_distance_line_pers[Aindex][1];
        Cx = left_distance_line_pers[Cindex][0];
        Cy = left_distance_line_pers[Cindex][1];
//        narrow_line[narrow_line_index][0] = Ax;
//        narrow_line[narrow_line_index++][1] = Ay;
//        narrow_line[narrow_line_index][0] = Bx;
//        narrow_line[narrow_line_index++][1] = By;
//        narrow_line[narrow_line_index][0] = Cx;
//        narrow_line[narrow_line_index++][1] = Cy;
        float left_dis = distance_to_line(Ax, Ay, Bx, By, Cx, Cy);
        float left_ang = get_angle(Ax, Ay, Bx, By, Cx, Cy);
        Bx = right_distance_line_pers[Bindex][0];
        By = right_distance_line_pers[Bindex][1];
        Ax = right_distance_line_pers[Aindex][0];
        Ay = right_distance_line_pers[Aindex][1];
        Cx = right_distance_line_pers[Cindex][0];
        Cy = right_distance_line_pers[Cindex][1];
//        narrow_line[narrow_line_index][0] = Ax;
//        narrow_line[narrow_line_index++][1] = Ay;
//        narrow_line[narrow_line_index][0] = Bx;
//        narrow_line[narrow_line_index++][1] = By;
//        narrow_line[narrow_line_index][0] = Cx;
//        narrow_line[narrow_line_index++][1] = Cy;
        float right_dis = distance_to_line(Ax, Ay, Bx, By, Cx, Cy);
        float right_ang = get_angle(Ax, Ay, Bx, By, Cx, Cy);
        if(left_dis > right_dis){
            flag.advance_avoid_obstacle_dir = -1;
        } else {
            flag.advance_avoid_obstacle_dir = 1;
        }
    } else {
        flag.advance_avoid_obstacle_dir = 0;
    }
    return flag.found_obstacle;
}

int check_garage() {
    garage_count = 0;
    int flag_next = 0;
    int distance = 0;
    int start_y;
    int end_y;
    flag.found_garage = false;
    start_y = 45;
    end_y = 20;
    int _bottom_start_x = bottom_start_x;
    int _bottom_end_x = bottom_end_x;
    // if(_bottom_end_x - _bottom_start_x < 35){
    //     _bottom_start_x = 17;
    //     _bottom_end_x = 77;
    // }
    for(int i = start_y; i > end_y; i--) {
        for (int j = _bottom_start_x; j < _bottom_end_x; j++) {
            if (gray_binary_image[i][j] == 0) {
                if (flag_next == 0 && distance > 0 && distance < 4) {
                    garage_count++;
                    flag_next = 1;
                }
                distance = 0;
            } else {
                flag_next = 0;
                distance++;
            }
        }
    }
    if(garage_count > 25){
        flag.found_garage = true;
        std::cout<<"found garage count:"<<garage_count<<std::endl;
    }
    return flag.found_garage;
}

// 抹除左上方道路
// 将图像左上方的道路抹除，即像素点置为黑色，并返回抹除的像素点个数
void erase_top_left_road(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
    if (x1<0)x1 = 0;
    if (x1>=80)x1 = 80;
    float slope = (float)(y2 - y1) / (x2 - x1);
    float b = y1 - slope * x1;
    for(int i = y1; i < y2; i++){
        int x_end = (i - b) / slope;
        binary_image[i][x_end - 2] = 255;
        binary_image[i][x_end - 1] = 255;
        binary_image[i][x_end] = 255;
    }
}

// 抹除右上方道路
// 将图像右上方的道路抹除，即像素点置为黑色，并返回抹除的像素点个数
void erase_top_right_road(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
    if (x1<0)x1 = 0;
    if (x1>=80)x1 = 80;
    float slope = (float)(y2 - y1) / (x2 - x1);
    float b = y1 - slope * x1;
    for(int i = y1; i < y2; i++){
        int x_start = (i - b) / slope;
        binary_image[i][x_start] = 255;
        binary_image[i][x_start + 1] = 255;
        binary_image[i][x_start + 2] = 255;
    }
}

void fix_left_break(int16_t y1, int16_t y2){
    for(int i = y1; i < y2; i++){
        int x_end = right_distance_line[i][0] - road_distances[i];
        binary_image[60 - i][x_end - 2] = 255;
        binary_image[60 - i][x_end - 1] = 255;
        binary_image[60 - i][x_end] = 255;
    }
}

void fix_right_break(int16_t y1, int16_t y2){
    for(int i = y1; i < y2; i++){
        int x_start = left_distance_line[i][0] + road_distances[i];
        binary_image[60 - i][x_start] = 255;
        binary_image[60 - i][x_start + 1] = 255;
        binary_image[60 - i][x_start + 2] = 255;
    }
}

void fix_crossroad(){
}

//2x2压缩图像
void compress_image(uint8_t *Dst, const uint8_t *Src){
    uint16_t i;
    uint16_t j;
    uint16_t compress_width = MT9V03X_W / 2;
    uint16_t compress_height = MT9V03X_H / 2;
    for(i = 0; i < compress_height; i++){
        for(j = 0; j < compress_width; j++){
            Dst[i * compress_width + j] = (Src[i*2*MT9V03X_W + j*2] + Src[i*2*MT9V03X_W + j*2+1] + Src[(i*2+1)*MT9V03X_W + j*2] + Src[(i*2+1)*MT9V03X_W + j*2+1]) / 4;
        }
    }
}

void get_lost_count() {
    left_lost_count = 0;
    right_lost_count = 0;
    left_lost_dir = 0;
    right_lost_dir = 0;
    int target_distance = 10;
    int target_lost_y1 = 13;

    if (lost_x1 == 0 || lost_y1 == 0 || lost_x2 == 0 || lost_y2 == 0 || max_white_column.left_height < 43 || lost_y1 < target_lost_y1) {
        return;
    }
    if(lost_x2 - lost_x1 == 0){
        for(int y = lost_y1; y > lost_y2; y--){
            int left_dist = 0;
            int right_dist = 0;
            for(int x = lost_x1; x >= 0; x--){
                if(binary_image_bak[y][x] == 0){
                    left_dist++;
                    if(left_dist > road_distances[60 - y] / 2 + target_distance){
                        left_lost_count++;
                        break;
                    }
                } else {
                    break;
                }
            }
            for(int x = lost_x1; x < 80; x++){
                if(binary_image_bak[y][x] == 0){
                    right_dist++;
                    if(right_dist > road_distances[60 - y] / 2 + target_distance){
                        right_lost_count++;
                        break;
                    }
                } else {
                    break;
                }
            }
        }
    } else {
        float slope = (float)(lost_y2 - lost_y1) / (float)(lost_x2 - lost_x1);
        float b = lost_y1 - slope * lost_x1;
        for (int y = lost_y1; y > lost_y2; y--) {
            int left_dist = 0;
            int right_dist = 0;
            for (int x = (y - b) / slope; x >= 0; x--) {
                if (binary_image_bak[y][x] == 0) {
                    left_dist++;
                    if(left_dist > road_distances[60 - y] / 2 + target_distance){
                        left_lost_count++;
                        break;
                    }
                } else {
                    break;
                }
            }
            for (int x = (y - b) / slope; x < 80; x++) {
                if (binary_image_bak[y][x] == 0) {
                    right_dist++;
                    if(right_dist > road_distances[60 - y] / 2 + target_distance){
                        right_lost_count++;
                        break;
                    }
                } else {
                    break;
                }
            }
        }
    }
    for(int dir_index = left_skip_index - 10; dir_index < left_skip_index; dir_index++){
        int left_dir = left_dirs[dir_index];
        if(left_dir == 4 || left_dir == 7){
            left_lost_dir++;
        }
    }
    for(int dir_index = right_skip_index - 10; dir_index < right_skip_index; dir_index++){
        int right_dir = right_dirs[dir_index];
        if(right_dir == 5 || right_dir == 6){
            right_lost_dir++;
        }
    }
}

void get_narrow_line(){
    memset(narrow_line, 0, sizeof(narrow_line));
    narrow_line_index = 0;
    if(max_white_column_pers.left_height < 19) return;
    int max_y = max_white_column_pers.left_height - 5;
    if(max_y > 40) max_y = 40;
    for(int q = 2; q < max_y; q++){
        if(distances_pers[q] < 15){
            int _x = distance_middle_line_pers[q][0];
            int _y = distance_middle_line_pers[q][1];
            if(narrow_line_index > 0 && abs(_y - narrow_line[narrow_line_index - 1][1]) > 3) break;
            narrow_line[narrow_line_index][0] = _x;
            narrow_line[narrow_line_index][1] = _y;
            narrow_line_index++;
        }
    }
}

void get_distance_line(){
    memset(left_distance_line, 0, sizeof(left_distance_line));
    memset(right_distance_line, 0, sizeof(right_distance_line));
    memset(left_distance, 0, sizeof(left_distance));
    memset(right_distance, 0, sizeof(right_distance));
    memset(distances, 0, sizeof(distances));
    memset(distance_middle_line, 0, sizeof(distance_middle_line));
    int left_x = max_white_column.left_x;
    int right_x = max_white_column.right_x;
    distance_index = 0;
    distance_middle_line_index = 0;
    lost_x1 = 0;
    lost_x2 = 0;
    lost_y1 = 0;
    lost_y2 = 0;
    for(int y = max_white_column.start_y - 1; y >= max_white_column.end_y; y--){
        for(int x = left_x; x >= 0; x--) {
            if (binary_image_bak[y][x] == 255 || x == 1) {
                left_distance_line[distance_index][0] = x;
                left_distance_line[distance_index][1] = y;
                left_distance[distance_index][0] = left_x - x;
                left_distance[distance_index][1] = y;
                break;
            }
        }
        for(int x = right_x; x < 80; x++) {
            if (binary_image_bak[y][x] == 255 || x == 79) {
                right_distance_line[distance_index][0] = x;
                right_distance_line[distance_index][1] = y;
                right_distance[distance_index][0] = x - right_x;
                right_distance[distance_index][1] = y;
                break;
            }
        }
        distances[distance_index] = (right_distance_line[distance_index][0] - left_distance_line[distance_index][0]);
        if(distances[distance_index] < road_distances[distance_index] + 7 && y > 7){
            distance_middle_line[distance_middle_line_index][0] = (right_distance_line[distance_index][0] + left_distance_line[distance_index][0]) / 2;
            distance_middle_line[distance_middle_line_index][1] = y;
            distance_middle_line_index++;
            if(lost_x1 != 0 && lost_y1 != 0 && lost_x2 == 0 && lost_y2 == 0){
                lost_x2 = (right_distance_line[distance_index][0] + left_distance_line[distance_index][0]) / 2;
                lost_y2 = y;
            }
        } else {
            if(lost_x1 == 0 && lost_y1 == 0 && lost_x2 == 0 && lost_y2 == 0){
                lost_x1 = (right_distance_line[distance_index-1][0] + left_distance_line[distance_index-1][0]) / 2;
                lost_y1 = y;
            }
        }
        distance_index++;
    }
}

void get_distance_line_pers(){
    memset(left_distance_line_pers, 0, sizeof(left_distance_line_pers));
    memset(right_distance_line_pers, 0, sizeof(right_distance_line_pers));
    memset(left_distance_pers, 0, sizeof(left_distance_pers));
    memset(right_distance_pers, 0, sizeof(right_distance_pers));
    memset(distances_pers, 0, sizeof(distances_pers));
    memset(distance_middle_line_pers, 0, sizeof(distance_middle_line_pers));
    int left_x = max_white_column_pers.left_x;
    int right_x = max_white_column_pers.right_x;
    distance_index_pers = 0;
    distance_middle_line_index_pers = 0;
    for(int y = max_white_column_pers.start_y - 1; y >= max_white_column_pers.end_y; y--){
        for(int x = left_x; x >= 0; x--) {
            if (binary_pers_image[y][x] == 255 || x == 1) {
                left_distance_line_pers[distance_index_pers][0] = x;
                left_distance_line_pers[distance_index_pers][1] = y;
                left_distance_pers[distance_index_pers][0] = left_x - x;
                left_distance_pers[distance_index_pers][1] = y;
                break;
            }
        }
        for(int x = right_x; x < 40; x++) {
            if (binary_pers_image[y][x] == 255 || x == 38) {
                right_distance_line_pers[distance_index_pers][0] = x;
                right_distance_line_pers[distance_index_pers][1] = y;
                right_distance_pers[distance_index_pers][0] = x - right_x;
                right_distance_pers[distance_index_pers][1] = y;
                break;
            }
        }
        distances_pers[distance_index_pers] = (right_distance_line_pers[distance_index_pers][0] - left_distance_line_pers[distance_index_pers][0]);
        distance_middle_line_pers[distance_middle_line_index_pers][0] = (right_distance_line_pers[distance_index_pers][0] + left_distance_line_pers[distance_index_pers][0]) / 2;
        distance_middle_line_pers[distance_middle_line_index_pers][1] = y;
        distance_middle_line_index_pers++;
        distance_index_pers++;
    }
}

uint8_t get_otsu_threshold(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint8_t *image) {
#define GrayScale 256
    int Pixel_Max = 0;
    int Pixel_Min = 255;
    uint16_t width = x2 - x1;
    uint16_t height = y2 - y1;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    uint8_t threshold = 120;
    const uint8_t* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32_t gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = y1; i < y2; i += 2)
    {
        for (j = x1; j < x2; j += 2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
            if(data[i * width + j]>Pixel_Max)   Pixel_Max=data[i * width + j];
            if(data[i * width + j]<Pixel_Min)   Pixel_Min=data[i * width + j];
        }
    }

    //计算每个像素值的点在整幅图像中的比例

    for (i = Pixel_Min; i < Pixel_Max; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    }

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = Pixel_Min; j < Pixel_Max; j++)
    {

        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += (float)j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp= gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
//        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8_t)j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }

    return threshold;
}

// 补边
void draw_rectan(){
    uint8_t i;
    border_color = 255;
    for (i = 0; i < 60; i++){
        binary_image[i][0] = border_color;
        binary_image[i][80 - 1] = border_color;
        binary_image[i][80 - 2] = border_color;
    }
    for (i = 0; i < 80; i++){
        binary_image[0][i] = border_color;
    }
}

int get_border_line(int detect_count_max) {
    int left_x, left_y, right_x, right_y;  // 爬线时坐标
    left_border_index = 0;  // 爬线时左边界索引
    right_border_index = 0;  // 爬线时右边界索引
    left_skip_index = 0;  // 爬线时左边界跳过索引
    right_skip_index = 0;  // 爬线时右边界跳过索引
    left_reach_edge = 0;
    right_reach_edge = 0;
    int left_y_index = 10;
    int right_y_index = 10;
    int left_dir;  // 爬线时左边界方向
    int right_dir;  // 爬线时右边界方向
    int detect_count;  // 爬线时检测次数
    int flaghighl=false;
    int flaghighr=false;
    flag.left_sec_border = false;
    flag.right_sec_border = false;
    counter.left_img_fix = 0;
    counter.right_img_fix = 0;
    counter.skip_left_border = 0;
    counter.skip_right_border = 0;

    road_color = 0;
    border_color = 255;

    memset(left_border, 0, sizeof(left_border));
    memset(right_border, 0, sizeof(right_border));
    memset(left_dirs, 0, sizeof(left_dirs));
    memset(right_dirs, 0, sizeof(right_dirs));
    memset(middle_line, 0, sizeof(middle_line));
    memset(middle_line_single, 0, sizeof(middle_line_single));
    // 初始边界点在道路内
    int first_x;
    int first_y = 50;
    for (first_x = (bottom_start_x + bottom_end_x) / 2; first_x >= 0; first_x--) {
        if (binary_image[first_y][first_x] == road_color && binary_image[first_y][first_x - 1] != road_color) {
            left_border[left_border_index][0] = first_x;
            left_border[left_border_index][1] = first_y;
            break;
        }
    }

    for (first_x = (bottom_start_x + bottom_end_x) / 2; first_x < 80; first_x++) {
        if (binary_image[first_y][first_x] == road_color && binary_image[first_y][first_x + 1] != road_color) {
            right_border[right_border_index][0] = first_x;
            right_border[right_border_index][1] = first_y;
            break;
        }
    }
    left_x = left_border[left_border_index][0];
    left_y = left_border[left_border_index][1];
    right_x = right_border[right_border_index][0];
    right_y = right_border[right_border_index][1];
    left_dir = -1;  // 重置左边界方向
    right_dir = -1;  // 重置右边界方向
    // vofa_udp.printf(":%d,%d,%d,%d,%d,%d\n",left_x,left_y,right_x,right_y,bottom_start_x,bottom_end_x);
    for(detect_count = 0; detect_count < detect_count_max; detect_count++) {
        // 左边界
        if(detect_count > 15 && sqrt((left_x - left_border[left_border_index- 5][0]) * (left_x - left_border[left_border_index - 5][0]) + (left_y - left_border[left_border_index - 5][1]) * (left_y - left_border[left_border_index - 5][1])) < 4){
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
        } else if((detect_count > 25 && left_x < 4 && left_y > 25)
            || (left_x < 6 && left_y > 21 && counter.skip_left_border > 0 && detect_count > 15 )) {
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
            flag.left_sec_border = false;
            counter.right_img_fix = right_border_x - 40;
        }else if (left_y < 10){
            if (flaghighl==false)flaghighl=true;
        } else if (left_dir != 3 && binary_image[left_y - 1][left_x + 1] == road_color &&
                   binary_image[left_y - 1][left_x] == border_color){  // 右上黑，方向3，正上白
            left_y = left_y - 1;
            left_x = left_x + 1;
            left_dir = 6;
            left_y_index++;
        } else if (left_dir != 4 && binary_image[left_y][left_x + 1] == road_color &&
                   binary_image[left_y - 1][left_x + 1] == border_color){  // 正右黑，方向4，右上白
            left_x = left_x + 1;
            left_dir = 5;
        } else if (left_dir != 1 && binary_image[left_y - 1][left_x] == road_color &&
                   binary_image[left_y - 1][left_x - 1] == border_color){  // 正上黑，方向1，左上白
            left_y = left_y - 1;
            left_dir = 0;
            left_y_index++;
            if (left_x < 4 && detect_count > 20) {
                counter.skip_left_border++;
            }
        } else if (left_dir != 2 && binary_image[left_y - 1][left_x - 1] == road_color &&
                   binary_image[left_y][left_x - 1] == border_color){  // 左上黑，方向2，正右白
            left_y = left_y - 1;
            left_x = left_x - 1;
            left_dir = 7;
            left_y_index++;
        } else if (left_dir !=5 && binary_image[left_y][left_x - 1] == road_color &&
                   binary_image[left_y + 1][left_x - 1] == border_color){  // 正左黑，方向5，左下白
            left_x = left_x - 1;
            left_dir = 4;
        } else if (left_dir != 6 && binary_image[left_y + 1][left_x - 1] == road_color &&
                   binary_image[left_y + 1][left_x] == border_color){  // 左下黑，方向6，正下白
            left_y = left_y + 1;
            left_x = left_x - 1;
            left_dir = 3;
            left_y_index--;
        } else if (left_dir != 7 && binary_image[left_y + 1][left_x + 1] == road_color &&
                   binary_image[left_y][left_x + 1] == border_color){  // 右下黑，方向7，正右白
            left_y = left_y + 1;
            left_x = left_x + 1;
            left_dir = 2;
            left_y_index--;
        } else if (left_dir != 0 && binary_image[left_y + 1][left_x] == road_color &&
                   binary_image[left_y + 1][left_x + 1] == border_color){  // 正下黑，方向8，右下白
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
        } else {
            break;
        }

        if (left_border_index != 0){
            left_border[left_border_index][0] = left_x;
            left_border[left_border_index][1] = left_y;
        }
        left_border_index++;
        left_dirs[left_border_index] = left_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }

        // 右边界
        if(detect_count > 15 && sqrt((right_x - right_border[right_border_index-5][0]) * (right_x - right_border[right_border_index-5][0]) + (right_y - right_border[right_border_index-5][1]) * (right_y - right_border[right_border_index-5][1])) < 4){
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
        } else if((detect_count > 25 && right_x > 76 && right_y > 25)
            || (right_x > 74 && right_y > 21 && counter.skip_right_border > 0 && detect_count > 15)) {
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
            flag.right_sec_border = false;
            counter.left_img_fix = 40 - left_border_x;
        } else if (right_y < 10){
            if (flaghighr==false)flaghighr=true;
        } else if(right_dir != 2 && binary_image[right_y - 1][right_x - 1] == road_color &&
                  binary_image[right_y - 1][right_x] == border_color) {  // 左上黑，方向2，正上白
            right_y = right_y - 1;
            right_x = right_x - 1;
            right_dir = 7;
            right_y_index++;
        } else if(right_dir != 5 && binary_image[right_y][right_x - 1] == road_color &&
                  binary_image[right_y - 1][right_x - 1] == border_color){  // 正左黑，方向5，正上白
            right_x = right_x - 1;
            right_dir = 4;
        } else if(right_dir != 1 && binary_image[right_y - 1][right_x] == road_color &&
                  binary_image[right_y - 1][right_x + 1] == border_color){  // 正上黑，方向1，右上白
            right_y = right_y - 1;
            right_dir = 0;
            right_y_index++;
            if (right_x > 76 && detect_count > 20) {
                counter.skip_right_border++;
            }
        } else if (right_x < 80 && right_dir != 3 && binary_image[right_y - 1][right_x + 1] == road_color &&
                   binary_image[right_y][right_x + 1] == border_color){  // 右上黑，方向3，正右白
            right_y = right_y - 1;
            right_x = right_x + 1;
            right_dir = 6;
            right_y_index++;
        } else if(right_dir != 4 && binary_image[right_y][right_x + 1] == road_color &&
                  binary_image[right_y + 1][right_x + 1] == border_color){  // 正右黑，方向4，正下白
            right_x = right_x + 1;
            right_dir = 5;
        } else if(right_dir != 6 && binary_image[right_y + 1][right_x - 1] == road_color &&
                  binary_image[right_y][right_x - 1] == border_color){  // 左下黑，方向6，正左白
            right_y = right_y + 1;
            right_x = right_x - 1;
            right_dir = 3;
        } else if(right_dir != 7 && binary_image[right_y + 1][right_x + 1] == road_color &&
                  binary_image[right_y + 1][right_x] == border_color) {  // 右下黑，方向7，正下白
            right_y = right_y + 1;
            right_x = right_x + 1;
            right_dir = 2;
        } else if(right_dir != 0 && binary_image[right_y + 1][right_x] == road_color &&
                  binary_image[right_y + 1][right_x - 1] == border_color){  // 正下黑，方向8，左下白
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
        } else {
            break;
        }

        right_border_index++;
        if (right_border_index != 0){
            right_border[right_border_index][0] = right_x;
            right_border[right_border_index][1] = right_y;
        }
        right_dirs[right_border_index] = right_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }
        if (flaghighl&&flaghighr) {
            break;
        }
        left_border_x = left_x;
        left_border_y = left_y;
        right_border_x = right_x;
        right_border_y = right_y;
    }
    detect_count_max = detect_count;

    // printf("%d,%d,%d,%d\r\n",flag.left_sec_border,flag.right_sec_border,counter.skip_left_border,counter.skip_right_border);
    // 计算中线
    middle_line_index = 0;
    if (flag.left_sec_border == false && flag.right_sec_border == false
        || (counter.left_img_fix < 1 && counter.right_img_fix < 1)) {
        for(int q = 0; q < detect_count_max; q++) {
            middle_line[q][0] = (left_border[q][0] + right_border[q][0]) / 2;
            middle_line[q][1] = (left_border[q][1] + right_border[q][1]) / 2;
            middle_line_index++;
        }
    }else {
        get_border_line_sec(detect_count_max);
    }

    middle_line_single_index = 0;
    int middle_line_current_y = -1;
    for(int q = 0; q < detect_count_max; q++){
        if(middle_line[q][1] != middle_line_current_y && middle_line[q][1] != 0){
            middle_line_single[middle_line_single_index][0] = middle_line[q][0];
            middle_line_single[middle_line_single_index][1] = middle_line[q][1];
            middle_line_single_index++;
            middle_line_current_y = middle_line[q][1];
        }
    }
    for(int q = 0; q < detect_count_max; q++) {
        // printf("%d,%d,%d\n",q,middle_line[q][0],middle_line[q][1]);
    }
    return detect_count_max;
}

int get_border_line_sec(int detect_count_max) {
    int left_x, left_y, right_x, right_y;  // 爬线时坐标
    left_border_index = 0;  // 爬线时左边界索引
    right_border_index = 0;  // 爬线时右边界索引
    left_skip_index = 0;  // 爬线时左边界跳过索引
    right_skip_index = 0;  // 爬线时右边界跳过索引
    left_reach_edge = 0;
    right_reach_edge = 0;
    int left_y_index = 9;
    int right_y_index = 9;
    int left_dir;  // 爬线时左边界方向
    int right_dir;  // 爬线时右边界方向
    int detect_count;  // 爬线时检测次数
    int flaghighl=false;
    int flaghighr=false;

    road_color = 0;
    border_color = 255;

    memset(left_border, 0, sizeof(left_border));
    memset(right_border, 0, sizeof(right_border));
    memset(left_dirs, 0, sizeof(left_dirs));
    memset(right_dirs, 0, sizeof(right_dirs));
    memset(middle_line, 0, sizeof(middle_line));
    memset(middle_line_single, 0, sizeof(middle_line_single));
    // 初始边界点在道路内
    int first_x;
    int first_y = 50;
    for (first_x = (bottom_start_x + bottom_end_x) / 2; first_x >= 0; first_x--) {
        if (binary_image[first_y][first_x] == road_color && binary_image[first_y][first_x - 1] != road_color) {
            left_border[left_border_index][0] = first_x;
            left_border[left_border_index][1] = first_y;
            break;
        }
    }

    for (first_x = (bottom_start_x + bottom_end_x) / 2; first_x < 80; first_x++) {
        if (binary_image[first_y][first_x] == road_color && binary_image[first_y][first_x + 1] != road_color) {
            right_border[right_border_index][0] = first_x;
            right_border[right_border_index][1] = first_y;
            break;
        }
    }
    left_x = left_border[left_border_index][0];
    left_y = left_border[left_border_index][1];
    right_x = right_border[right_border_index][0];
    right_y = right_border[right_border_index][1];
    left_dir = -1;  // 重置左边界方向
    right_dir = -1;  // 重置右边界方向
    // vofa_udp.printf(":%d,%d,%d,%d,%d,%d\n",left_x,left_y,right_x,right_y,bottom_start_x,bottom_end_x);
    for(detect_count = 0; detect_count < detect_count_max; detect_count++) {
        // 左边界
        if(detect_count > 15 && sqrt((left_x - left_border[left_border_index- 5][0]) * (left_x - left_border[left_border_index - 5][0]) + (left_y - left_border[left_border_index - 5][1]) * (left_y - left_border[left_border_index - 5][1])) < 4){
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
        } else if( (detect_count > 10 && left_x < 3)) {
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
        }else if (left_y < 20){
            if (flaghighl==false)flaghighl=true;
        } else if (left_dir != 3 && binary_image[left_y - 1][left_x + 1] == road_color &&
                   binary_image[left_y - 1][left_x] == border_color){  // 右上黑，方向3，正上白
            left_y = left_y - 1;
            left_x = left_x + 1;
            left_dir = 6;
            left_y_index++;
        } else if (left_dir != 4 && binary_image[left_y][left_x + 1] == road_color &&
                   binary_image[left_y - 1][left_x + 1] == border_color){  // 正右黑，方向4，右上白
            left_x = left_x + 1;
            left_dir = 5;
        } else if (left_dir != 1 && binary_image[left_y - 1][left_x] == road_color &&
                   binary_image[left_y - 1][left_x - 1] == border_color){  // 正上黑，方向1，左上白
            left_y = left_y - 1;
            left_dir = 0;
            left_y_index++;
        } else if (left_dir != 2 && binary_image[left_y - 1][left_x - 1] == road_color &&
                   binary_image[left_y][left_x - 1] == border_color){  // 左上黑，方向2，正右白
            left_y = left_y - 1;
            left_x = left_x - 1;
            left_dir = 7;
            left_y_index++;
        } else if (left_dir !=5 && binary_image[left_y][left_x - 1] == road_color &&
                   binary_image[left_y + 1][left_x - 1] == border_color){  // 正左黑，方向5，左下白
            left_x = left_x - 1;
            left_dir = 4;
        } else if (left_dir != 6 && binary_image[left_y + 1][left_x - 1] == road_color &&
                   binary_image[left_y + 1][left_x] == border_color){  // 左下黑，方向6，正下白
            left_y = left_y + 1;
            left_x = left_x - 1;
            left_dir = 3;
            left_y_index--;
        } else if (left_dir != 7 && binary_image[left_y + 1][left_x + 1] == road_color &&
                   binary_image[left_y][left_x + 1] == border_color){  // 右下黑，方向7，正右白
            left_y = left_y + 1;
            left_x = left_x + 1;
            left_dir = 2;
            left_y_index--;
        } else if (left_dir != 0 && binary_image[left_y + 1][left_x] == road_color &&
                   binary_image[left_y + 1][left_x + 1] == border_color){  // 正下黑，方向8，右下白
            if(left_skip_index == 0){
                left_skip_index = left_border_index;
            }
        } else {
            break;
        }

        if (left_border_index != 0) {
            left_border[left_border_index][0] = left_x;
            left_border[left_border_index][1] = left_y;
            if (flag.right_sec_border){
                middle_line[left_border_index][0] = left_x + road_distances_fix[left_y_index] / 2 + static_cast<int>(0.016 * left_y_index * counter.left_img_fix);
                middle_line[left_border_index][1] = left_y;
            }
            // printf("index:%d,x:%d,y:%d,:y_index:%d,road:%d,mid%d\n",left_border_index,left_x,left_y,left_y_index,road_distances_fix[left_y_index],middle_line[left_border_index][0]);
        }
        left_border_index++;
        left_dirs[left_border_index] = left_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }

        // 右边界
        if(detect_count > 15 && sqrt((right_x - right_border[right_border_index-5][0]) * (right_x - right_border[right_border_index-5][0]) + (right_y - right_border[right_border_index-5][1]) * (right_y - right_border[right_border_index-5][1])) < 4){
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
        } else if((detect_count > 10 && right_x > 76)) {
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
        } else if (right_y < 20){
            if (flaghighr==false)flaghighr=true;
        } else if(right_dir != 2 && binary_image[right_y - 1][right_x - 1] == road_color &&
                  binary_image[right_y - 1][right_x] == border_color) {  // 左上黑，方向2，正上白
            right_y = right_y - 1;
            right_x = right_x - 1;
            right_dir = 7;
            right_y_index++;
        } else if(right_dir != 5 && binary_image[right_y][right_x - 1] == road_color &&
                  binary_image[right_y - 1][right_x - 1] == border_color){  // 正左黑，方向5，正上白
            right_x = right_x - 1;
            right_dir = 4;
        } else if(right_dir != 1 && binary_image[right_y - 1][right_x] == road_color &&
                  binary_image[right_y - 1][right_x + 1] == border_color){  // 正上黑，方向1，右上白
            right_y = right_y - 1;
            right_dir = 0;
            right_y_index++;
        } else if (right_x < 80 && right_dir != 3 && binary_image[right_y - 1][right_x + 1] == road_color &&
                   binary_image[right_y][right_x + 1] == border_color){  // 右上黑，方向3，正右白
            right_y = right_y - 1;
            right_x = right_x + 1;
            right_dir = 6;
            right_y_index++;
        } else if(right_dir != 4 && binary_image[right_y][right_x + 1] == road_color &&
                  binary_image[right_y + 1][right_x + 1] == border_color){  // 正右黑，方向4，正下白
            right_x = right_x + 1;
            right_dir = 5;
        } else if(right_dir != 6 && binary_image[right_y + 1][right_x - 1] == road_color &&
                  binary_image[right_y][right_x - 1] == border_color){  // 左下黑，方向6，正左白
            right_y = right_y + 1;
            right_x = right_x - 1;
            right_dir = 3;
            right_y_index--;
        } else if(right_dir != 7 && binary_image[right_y + 1][right_x + 1] == road_color &&
                  binary_image[right_y + 1][right_x] == border_color) {  // 右下黑，方向7，正下白
            right_y = right_y + 1;
            right_x = right_x + 1;
            right_dir = 2;
            right_y_index--;
        } else if(right_dir != 0 && binary_image[right_y + 1][right_x] == road_color &&
                  binary_image[right_y + 1][right_x - 1] == border_color){  // 正下黑，方向8，左下白
            if(right_skip_index == 0){
                right_skip_index = right_border_index;
            }
        } else {
            break;
        }

        right_border_index++;
        if (right_border_index != 0){
            right_border[right_border_index][0] = right_x;
            right_border[right_border_index][1] = right_y;
            if (flag.left_sec_border){
                middle_line[right_border_index][0] = right_x - road_distances_fix[right_y_index] / 2 - static_cast<int>(0.016 * right_y_index * counter.right_img_fix);;
                middle_line[right_border_index][1] = right_y;
            }
        }
        right_dirs[right_border_index] = right_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }
        if (flaghighl&&flaghighr) {
            break;
        }
        left_border_x = left_x;
        left_border_y = left_y;
        right_border_x = right_x;
        right_border_y = right_y;
    }
    detect_count_max = detect_count;

    return detect_count_max;
}

int get_border_line_pers(int detect_count_max) {
    int left_x, left_y, right_x, right_y;  // 爬线时坐标
    left_border_index_pers = 0;  // 爬线时左边界索引
    right_border_index_pers = 0;  // 爬线时右边界索引
    left_skip_index_pers = 0;  // 爬线时左边界跳过索引
    right_skip_index_pers = 0;  // 爬线时右边界跳过索引
    int left_dir;  // 爬线时左边界方向
    int right_dir;  // 爬线时右边界方向
    int detect_count;  // 爬线时检测次数


    road_color = 0;
    border_color = 255;


    memset(left_border_pers, 0, sizeof(left_border_pers));
    memset(right_border_pers, 0, sizeof(right_border_pers));
    memset(left_dirs_pers, 0, sizeof(left_dirs_pers));
    memset(right_dirs_pers, 0, sizeof(right_dirs_pers));
    // 初始边界点在道路内
    int first_x;
    int first_y = 57;
    for (first_x = (bottom_start_x_pers + bottom_end_x_pers) / 2; first_x > 0; first_x--) {
        if (binary_pers_image[first_y][first_x] == road_color && binary_pers_image[first_y][first_x - 1] != road_color) {
            left_border_pers[left_border_index_pers][0] = first_x;
            left_border_pers[left_border_index_pers][1] = first_y;
            break;
        }
    }

    for (first_x = (bottom_start_x_pers + bottom_end_x_pers)/2; first_x < 40; first_x++) {
        if (binary_pers_image[first_y][first_x] == road_color && binary_pers_image[first_y][first_x + 1] != road_color) {
            right_border_pers[right_border_index_pers][0] = first_x;
            right_border_pers[right_border_index_pers][1] = first_y;
            break;
        }
    }
    left_x = left_border_pers[left_border_index_pers][0];
    left_y = left_border_pers[left_border_index_pers][1];
    right_x = right_border_pers[right_border_index_pers][0];
    right_y = right_border_pers[right_border_index_pers][1];
    left_dir = -1;  // 重置左边界方向
    right_dir = -1;  // 重置右边界方向
    for(detect_count = 0; detect_count < detect_count_max; detect_count++) {
        // 左边界
        if(detect_count > 15 && sqrt((left_x - left_border_pers[left_border_index_pers-5][0]) * (left_x - left_border_pers[left_border_index_pers-5][0]) + (left_y - left_border_pers[left_border_index_pers-5][1]) * (left_y - left_border_pers[left_border_index_pers-5][1])) < 4){
            if(left_skip_index == 0){
                left_skip_index = left_border_index_pers;
            }
        } else if(left_y < 3 || left_x < 3 || left_x > 37) {
            if(left_skip_index == 0){
                left_skip_index = left_border_index_pers;
            }
        } else if (left_dir != 3 && binary_pers_image[left_y - 1][left_x + 1] == road_color &&
                   binary_pers_image[left_y - 1][left_x] == border_color){  // 右上黑，方向3，正上白
            left_y = left_y - 1;
            left_x = left_x + 1;
            left_dir = 6;
        } else if (left_dir != 4 && binary_pers_image[left_y][left_x + 1] == road_color &&
                   binary_pers_image[left_y - 1][left_x + 1] == border_color){  // 正右黑，方向4，右上白
            left_x = left_x + 1;
            left_dir = 5;
        } else if (left_dir != 1 && binary_pers_image[left_y - 1][left_x] == road_color &&
                   binary_pers_image[left_y - 1][left_x - 1] == border_color){  // 正上黑，方向1，左上白
            left_y = left_y - 1;
            left_dir = 0;
        } else if (left_dir != 2 && binary_pers_image[left_y - 1][left_x - 1] == road_color &&
                   binary_pers_image[left_y][left_x - 1] == border_color){  // 左上黑，方向2，正右白
            left_y = left_y - 1;
            left_x = left_x - 1;
            left_dir = 7;
        } else if (left_dir !=5 && binary_pers_image[left_y][left_x - 1] == road_color &&
                   binary_pers_image[left_y + 1][left_x - 1] == border_color){  // 正左黑，方向5，左下白
            left_x = left_x - 1;
            left_dir = 4;
        } else if (left_dir != 6 && binary_pers_image[left_y + 1][left_x - 1] == road_color &&
                   binary_pers_image[left_y + 1][left_x] == border_color){  // 左下黑，方向6，正下白
            left_y = left_y + 1;
            left_x = left_x - 1;
            left_dir = 3;
        } else if (left_dir != 7 && binary_pers_image[left_y + 1][left_x + 1] == road_color &&
                   binary_pers_image[left_y][left_x + 1] == border_color){  // 右下黑，方向7，正右白
            left_y = left_y + 1;
            left_x = left_x + 1;
            left_dir = 2;
        } else if (left_dir != 0 && binary_pers_image[left_y + 1][left_x] == road_color &&
                   binary_pers_image[left_y + 1][left_x + 1] == border_color){  // 正下黑，方向8，右下白
            if(left_skip_index == 0){
                left_skip_index = left_border_index_pers;
            }
        } else {
            break;
        }

        left_border_index_pers++;
        left_border_pers[left_border_index_pers][0] = left_x;
        left_border_pers[left_border_index_pers][1] = left_y;
        left_dirs[left_border_index_pers] = left_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }

        // 右边界
        if(detect_count > 15 && sqrt((right_x - right_border_pers[right_border_index_pers-5][0]) * (right_x - right_border_pers[right_border_index_pers-5][0]) + (right_y - right_border_pers[right_border_index_pers-5][1]) * (right_y - right_border_pers[right_border_index_pers-5][1])) < 4){
            if(right_skip_index == 0){
                right_skip_index = right_border_index_pers;
            }
        } else if(right_y < 3 || right_x > 37 || right_x < 3) {
            if(right_skip_index == 0){
                right_skip_index = right_border_index_pers;
            }
        } else if(right_dir != 2 && binary_pers_image[right_y - 1][right_x - 1] == road_color &&
                  binary_pers_image[right_y - 1][right_x] == border_color) {  // 左上黑，方向2，正上白
            right_y = right_y - 1;
            right_x = right_x - 1;
            right_dir = 7;
        } else if(right_dir != 5 && binary_pers_image[right_y][right_x - 1] == road_color &&
                  binary_pers_image[right_y - 1][right_x - 1] == border_color){  // 正左黑，方向5，正上白
            right_x = right_x - 1;
            right_dir = 4;
        } else if(right_dir != 1 && binary_pers_image[right_y - 1][right_x] == road_color &&
                  binary_pers_image[right_y - 1][right_x + 1] == border_color){  // 正上黑，方向1，右上白
            right_y = right_y - 1;
            right_dir = 0;
        } else if (right_dir != 3 && binary_pers_image[right_y - 1][right_x + 1] == road_color &&
                   binary_pers_image[right_y][right_x + 1] == border_color){  // 右上黑，方向3，正右白
            right_y = right_y - 1;
            right_x = right_x + 1;
            right_dir = 6;
        } else if(right_dir != 4 && binary_pers_image[right_y][right_x + 1] == road_color &&
                  binary_pers_image[right_y + 1][right_x + 1] == border_color){  // 正右黑，方向4，正下白
            right_x = right_x + 1;
            right_dir = 5;
        } else if(right_dir != 6 && binary_pers_image[right_y + 1][right_x - 1] == road_color &&
                  binary_pers_image[right_y][right_x - 1] == border_color){  // 左下黑，方向6，正左白
            right_y = right_y + 1;
            right_x = right_x - 1;
            right_dir = 3;
        } else if(right_dir != 7 && binary_pers_image[right_y + 1][right_x + 1] == road_color &&
                  binary_pers_image[right_y + 1][right_x] == border_color) {  // 右下黑，方向7，正下白
            right_y = right_y + 1;
            right_x = right_x + 1;
            right_dir = 2;
        } else if(right_dir != 0 && binary_pers_image[right_y + 1][right_x] == road_color &&
                  binary_pers_image[right_y + 1][right_x - 1] == border_color){  // 正下黑，方向8，左下白
            if(right_skip_index == 0){
                right_skip_index = right_border_index_pers;
            }
        } else {
            break;
        }
        right_border_index_pers++;
        right_border_pers[right_border_index_pers][0] = right_x;
        right_border_pers[right_border_index_pers][1] = right_y;
        right_dirs[right_border_index_pers] = right_dir;

        if(left_x == right_x && left_y == right_y){
            break;
        }
    }
    detect_count_max = detect_count;

    // 计算中线
    memset(middle_line_pers, 0, sizeof(middle_line_pers));
    middle_line_index_pers = 0;
    for(int q = 0; q < detect_count_max; q++){
        middle_line_pers[q][0] = (left_border_pers[q][0] + right_border_pers[q][0]) / 2;
        middle_line_pers[q][1] = (left_border_pers[q][1] + right_border_pers[q][1]) / 2;
        middle_line_index_pers++;
    }

    memset(middle_line_single_pers, 0, sizeof(middle_line_single_pers));
    middle_line_single_index_pers = 0;
    int middle_line_current_y = -1;
    for(int q = 0; q < detect_count_max; q++){
        if(middle_line_pers[q][1] != middle_line_current_y && middle_line_pers[q][1] != 0){
            middle_line_single_pers[middle_line_single_index_pers][0] = middle_line_pers[q][0];
            middle_line_single_pers[middle_line_single_index_pers][1] = middle_line_pers[q][1];
            middle_line_single_index_pers++;
            middle_line_current_y = middle_line_pers[q][1];
        }
    }

    return detect_count_max ;
}

void get_max_middle_line_height(){
    max_middle_line_height = 0;
    cornering = 0;
    if(middle_line_index_pers < 1)middle_line_index_pers = 1;
    for(int q = middle_line_index_pers - 1; q > 0; q--){
        if(middle_line_pers[q][0] > 14 && middle_line_pers[q][0] < 34 &&
        DISTANCE(left_border_pers[q][0],left_border_pers[q][1],right_border_pers[q][0],right_border_pers[q][1]) < 10){
            max_middle_line_height = 60 - middle_line_pers[q][1];
            int x1 = middle_line_pers[q][0];
            int y1 = middle_line_pers[q][1];
            int x2 = middle_line_pers[middle_line_index_pers - 1][0];
            int y2 = middle_line_pers[middle_line_index_pers - 1][1];
            if(max_white_column.left_height > 35 && abs(y1 - y2) < 15 && counter.drive_in_left_roundabout == 0 && counter.drive_in_right_roundabout == 0) {
                if (x2 - x1 > 6) cornering = 1;
                if (x2 - x1 < -6) cornering = -1;
            } else {
                cornering = 0;
            }
            break;
        }
    }
    max_middle_line_height = gray_binary_pers_image[44][23];
}

// 计算A到BC构成的直线的距离
float distance_to_line(float Ax, float Ay, float Bx, float By, float Cx, float Cy){
    float A = Cy - By;
    float B = Bx - Cx;
    float C = Cx * By - Bx * Cy;
    if(A == 0 && B == 0){
        return 0;
    }
    return fabsf(A * Ax + B * Ay + C) / sqrtf(A * A + B * B);
}

// 计算ABC三点的夹角，其中B点为端点
float get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy) {
    // 余弦定理求角度
    float AB = sqrt((Bx - Ax) * (Bx - Ax) + (By - Ay) * (By - Ay));
    float BC = sqrt((Bx - Cx) * (Bx - Cx) + (By - Cy) * (By - Cy));
    float AC = sqrt((Cx - Ax) * (Cx - Ax) + (Cy - Ay) * (Cy - Ay));
    float angle = acos((AB * AB + BC * BC - AC * AC) / (2 * AB * BC)) * 180 / 3.1415926;
    // 以BA为0度，逆时针为角度正方向，判断角度是否大于180度，若大于180度则取负值
    if (((Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax)) < 0) {
        angle = -angle;
    }
    return angle;
}

uint8_t is_left_corner(float left_Ax, float left_Ay, float left_Bx, float left_By, float left_Cx, float left_Cy,
                     float right_Ax, float right_Ay, float right_Bx, float right_By, float right_Cx, float right_Cy){
/*C---B
 *  \ |
 *    A
 * */
    // 第一步判断角度
    float _left_angle = get_angle(left_Ax, left_Ay, left_Bx, left_By, left_Cx, left_Cy);
    float _right_angle = get_angle(right_Ax, right_Ay, right_Bx, right_By, right_Cx, right_Cy);
    if(_left_angle > -110 && _left_angle < -70 && _right_angle > -120){  // 左角度在-75~-125度之间，右角度大于-120
        // 第二步判断斜率和B点位置
        float slope_AC = (left_Cy - left_Ay) / (left_Cx - left_Ax);  // 斜率大于0（防止十字路口检测到其他拐点）
        float det = left_Ax * (left_By - left_Cy) - left_Ay * (left_Bx - left_Cx) + left_Bx * left_Cy - left_By * left_Cx;  // 判断B点在AC直线的下方（防止十字路口检测到其他拐点）
        if(slope_AC > 0 && det < 0 && left_By > 10){
            // 第三步获取左边的最高点坐标值，代入原图像检测上方左右3个像素点是否存在边界
            int left_highest_y = left_By;
            int left_highest_x = left_Bx;
            // 防止越界
            if(left_highest_x < 6) left_highest_x = 6;
            int left_has_border = 0;
            int i = 1;
            for(int j = -3; j <= 3; j++){
                if(binary_image[left_highest_y - i][left_highest_x + j] == border_color){
                    left_has_border++;
                }
            }
            if(left_has_border == 0){  // 左边上方没找到边界
                int diff = left_By - left_Cy;  // B点减C点的y坐标小于2（防止检测到环岛中间的圈）
                if(diff < 2){
                    flag.left_B_higher_than_C = true;
                } else {
                    flag.left_B_higher_than_C = false;
                }
                left_corner_angle = _left_angle;
                return 1;
            }
        }
    }
    return 0;
}

uint8_t is_right_corner(float right_Ax, float right_Ay, float right_Bx, float right_By, float right_Cx, float right_Cy,
                      float left_Ax, float left_Ay, float left_Bx, float left_By, float left_Cx, float left_Cy){
/*B---C
 *| /
 *A
 * */
    // 第一步判断角度
    float _right_angle = get_angle(right_Ax, right_Ay, right_Bx, right_By, right_Cx, right_Cy);
    float _left_angle = get_angle(left_Ax, left_Ay, left_Bx, left_By, left_Cx, left_Cy);
    if(_right_angle < 110 && _right_angle > 70 && _left_angle < 120){  // 右角度在75~125度之间，左角度小于120
        // 第二步判断斜率和B点位置
        float slope_AC = (right_Cy - right_Ay) / (right_Cx - right_Ax);  // 斜率小于0（防止十字路口检测到其他拐点）
        float det = right_Ax * (right_By - right_Cy) - right_Ay * (right_Bx - right_Cx) + right_Bx * right_Cy - right_By * right_Cx;  // 判断B点在AC直线的下方（防止十字路口检测到其他拐点）
        if(slope_AC < 0 && det > 0 && right_By > 10){
            // 第三步获取右边的最高点坐标值，代入原图像检测上方左右3个像素点是否存在边界
            int right_highest_y = right_By;
            int right_highest_x = right_Bx;
            // 防止越界
            if(right_highest_x > 88) right_highest_x = 88;
            int right_has_border = 0;
            int i = 1;
            for(int j = -3; j <= 3; j++){
                if(binary_image[right_highest_y - i][right_highest_x + j] == border_color){
                    right_has_border++;
                }
            }
            if(right_has_border == 0){  // 右边上方没找到边界
                int diff = right_By - right_Cy;  // B点减C点的y坐标小于2（防止检测到环岛中间的圈）
                if(diff < 2){
                    flag.right_B_higher_than_C = true;
                } else {
                    flag.right_B_higher_than_C = false;
                }
                right_corner_angle = _right_angle;
                return 1;
            }
        }
    }
    return 0;
}

void calculate_contrast(uint8_t *dst_image, const uint8_t *src_image, int16_t image_w, int16_t image_h) {
    for (int y = 0; y < image_h; y++) {
        for (int x = 1; x < image_w - 1; x++) {
            // 采样左右2个像素
            int pixel1 = src_image[y * image_w + (x - 1)];
            int pixel2 = src_image[y * image_w + (x + 1)];

            // 计算像素之间的差异和总和
            int diff = pixel1 - pixel2;
            int sum = pixel1 + pixel2;

            // 计算差比和并放大255倍
            // (sum != 0)避免除以0
            uint8_t contrast = (sum != 0) ? abs(diff * 255 / sum) : 0;
            dst_image[y * image_w + x] = contrast;
        }
    }
}

void calculate_contrast_x8(uint8_t *dst_image, const uint8_t *src_image, int16_t image_w, int16_t image_h) {
    for (int y = 1; y < image_h - 1; y++) {
        for (int x = 1; x < image_w - 1; x++) {
            int pixel1 = src_image[y * image_w + (x - 1)];
            int pixel2 = src_image[y * image_w + (x + 1)];
            int pixel3 = src_image[(y - 1) * image_w + x];
            int pixel4 = src_image[(y + 1) * image_w + x];
            int pixel5 = src_image[(y - 1) * image_w + (x - 1)];
            int pixel6 = src_image[(y - 1) * image_w + (x + 1)];
            int pixel7 = src_image[(y + 1) * image_w + (x - 1)];
            int pixel8 = src_image[(y + 1) * image_w + (x + 1)];


            int diff1 = pixel1 - pixel2;
            int diff2 = pixel3 - pixel4;
            int diff3 = pixel5 - pixel6;
            int diff4 = pixel7 - pixel8;
            int sum1 = pixel1 + pixel2;
            int sum2 = pixel3 + pixel4;
            int sum3 = pixel5 + pixel6;
            int sum4 = pixel7 + pixel8;

            int contrast1 = (sum1 != 0) ? abs(diff1 * 80 / sum1) : 0;
            int contrast2 = (sum2 != 0) ? abs(diff2 * 80 / sum2) : 0;
            int contrast3 = (sum3 != 0) ? abs(diff3 * 32 / sum3) : 0;
            int contrast4 = (sum4 != 0) ? abs(diff4 * 32 / sum4) : 0;
            dst_image[y * image_w + x] = contrast1 + contrast2 + contrast3 + contrast4;
        }
    }
}

void tcp_draw_border_line(cv::Mat& image, int x, int y, const uint8_t line[][2], cv::Scalar color) {
    int i = 0;

    while (line[i][0] != 0 || line[i][1] != 0) {
        const int px = x + line[i][0];
        const int py = y + line[i][1];

        if (px >= 0 && px < image.cols && py >= 0 && py < image.rows) {
            image.at<cv::Vec3b>(py, px) = cv::Vec3b(
                color[0],  // Blue
                color[1],  // Green
                color[2]   // Red
            );
        }
        i++;
    }
}

void tcp_draw_real_border_line(cv::Mat& image, int x, int y, const uint8_t line[][2], cv::Scalar color) {
    int i = incision;

    while (i < incision + IMAGE_VALID_NUM && (line[i][0] != 0 || line[i][1] != 0)) {
        const int px = x + line[i][0];
        const int py = y + line[i][1];

        if (px >= 0 && px < image.cols && py >= 0 && py < image.rows) {
            image.at<cv::Vec3b>(py, px) = cv::Vec3b(
                color[0],  // Blue
                color[1],  // Green
                color[2]   // Red
            );
        }
        i++;
    }
}

void cover_car_head(){
    for(int x = 30; x < 53; x++){
        for(int y = 47; y < 60; y++){
            gray_image[y][x] = gray_image[y-1][x];
        }
    }
}

void cover_car_head_pers(){
    for(int x = 0; x < 40; x++){
        for(int y = 58; y < 60; y++){
            gray_pers_image[y][x] = gray_pers_image[y-1][x];
        }
    }
}

void outbounds_detection(void){
    for(int i = 59;i > 0;i--) {
        if(gray_binary_image[i][40] == 255) {
            blind_line = 60 - i;
        }
        else {
            break;
        }
    }
}