// Copyright (c) 2021 xxx.Co.Ltd. All rights reserved.
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#define eps 1e-10

class Point3d {
 public:
    double x, y, z;
    Point3d() {}
    Point3d(double x_, double y_, double z_) :x(x_), y(y_), z(z_) {}
    friend double operator* (const Point3d& p, const Point3d& q) { return p.x * q.x + p.y * q.y + p.z * q.z; }   // 向量的点乘
    friend Point3d operator* (double s, const Point3d& p) { return Point3d(p.x * s, p.y * s, p.z * s); }    // 向量乘以常数
    friend Point3d operator/ (const Point3d& p, double s) { return Point3d(p.x / s, p.y / s, p.z / s); }    // 向量除以常数
    friend Point3d operator- (const Point3d& p, const Point3d& q) { return Point3d(p.x - q.x, p.y - q.y, p.z - q.z); } // 向量的差
    friend Point3d operator+ (const Point3d& p, const Point3d& q) { return Point3d(p.x + q.x, p.y + q.y, p.z + q.z); } // 向量的和
    friend bool operator == (const Point3d& p, const Point3d& q) {       // 两个点近似相等
        return fabs(p.x - q.x) + fabs(p.y - q.y) + fabs(p.z - q.z) <= eps;
    }
};

class Edge {    // 边
 public:
    Point3d st, ed;
    Edge() {}
    Edge(const Point3d& p0_, const Point3d& p1_) : st(p0_), ed(p1_) {}
    friend bool operator == (const Edge& e1, const Edge& e2) {       // 判断两条边是否相等
        return ((e1.st == e2.st) && (e1.ed == e2.ed)) || ((e1.st == e2.ed) && (e1.ed == e2.st));
    }
};

class Triangle {    // 三角形
 public:
    Point3d p0, p1, p2;      // 三角形的顶点
    int r, g, b;    // 三角形的颜色
    Triangle() {}
    Triangle(const Point3d& p0_, const Point3d& p1_, const Point3d& p2_) : p0(p0_), p1(p1_), p2(p2_), r(0), g(0), b(0) {}
    friend bool operator==(const Triangle& left, const Triangle& right) {      // 判断两三角形是否相等
        return (left.p0 == right.p0 || left.p0 == right.p1 || left.p0 == right.p2)
            && (left.p1 == right.p0 || left.p1 == right.p1 || left.p1 == right.p2)
            && (left.p2 == right.p0 || left.p2 == right.p1 || left.p2 == right.p2);
    }
};

bool neibor_triangle(const Triangle& left, const Triangle& right) {
    int same = 0; // 相同点数
    if (left.p0 == right.p0 || left.p0 == right.p1 || left.p0 == right.p2) same++;
    if (left.p1 == right.p0 || left.p1 == right.p1 || left.p1 == right.p2) same++;
    if (left.p2 == right.p0 || left.p2 == right.p1 || left.p2 == right.p2) same++;
    if (same == 2) return true;
    return false;
}

void calc_inversion(std::vector<std::vector<double>>& matrix_A, std::vector<std::vector<double>>& inv_A, int len) {
    inv_A.resize(len);

    // 对输入矩阵进行扩容，构建增广矩阵
    for (int i = 0; i < len; i++) {
        matrix_A[i].resize(2 * len);
        inv_A[i].resize(len);
        matrix_A[i][len + i] = 1;
    }

    // 根据初等变换法求解逆矩阵
    for (int i = 0; i < len; i++) {
        if (matrix_A[i][i] == 0) {    // 如果A矩阵主对角线上元素为0，则和以后的第i列元素不为0的进行交换
            for (int j = i + 1; j < len; j++) {
                if (matrix_A[j][i] != 0) {
                    std::vector<double> temp;
                    temp = matrix_A[i];
                    matrix_A[i] = matrix_A[j];
                    matrix_A[j] = temp;
                    break;
                }
            }
        }
        for (int j = 0; j < len; j++) {
            if (i != j) {
                if (matrix_A[j][i] != 0) { // 只有当其不为零时进行计算，否则不计算
                    double coefficient = matrix_A[j][i] / matrix_A[i][i];
                    for (int k = i; k < 2 * len; k++) {
                        matrix_A[j][k] -= coefficient * matrix_A[i][k];
                    }
                }
            }
        }
        double coefficient = matrix_A[i][i];
        for (int j = i; j < 2 * len; j++) {
            matrix_A[i][j] /= coefficient;
        }
    }
    //截取后半部分为逆矩阵
    for (int i = 0; i < len; i++) {
        for (int j = 0; j < len; j++) {
            inv_A[i][j] = matrix_A[i][j + len];
        }
    }
}

/* 参数载入 */
bool part_params_load(int& N, const std::string& k_input_filename, std::vector<Triangle>& triangles) {
    FILE* fp;
    fp = fopen(k_input_filename.c_str(), "r");
    if (fp == NULL) {
        printf("The file was not opened!!\n");
        exit(0);
    } else {
        fscanf(fp, "%d\n", &N);
        triangles.resize(N);
        for (int i = 0; i < N; i++) {
            fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d\n", &triangles[i].p0.x, &triangles[i].p0.y, &triangles[i].p0.z,
                &triangles[i].p1.x, &triangles[i].p1.y, &triangles[i].p1.z, &triangles[i].p2.x, &triangles[i].p2.y, &triangles[i].p2.z,
                &triangles[i].r, &triangles[i].g, &triangles[i].b);
        }
    }
    fclose(fp);
    return 1;
}

void calc_unsure_color(const int N, std::vector<Triangle>& triangles) {
    // 首先统计不确定平面的序号
    std::vector<int> unsure;
    for (int i = 0; i < N; i++) {
        if (triangles[i].r == 255 && triangles[i].g == 255 && triangles[i].b == 0) {
            unsure.emplace_back(i);
        }
    }
    // 寻找不确定三角形的邻域(3个)
    std::map<int, std::vector<int>> unsure_neighbor;
    for (auto num : unsure) {
        for (int i = 0; i < N; i++) {
            if (i != num) {
                if (neibor_triangle(triangles[i], triangles[num])) unsure_neighbor[num].emplace_back(i);
            }
        }
    }

    // ---------------------算法开始--------------------- //
    // 预处理，利用均值，过滤掉周围三角形均是已知的
    for (const auto& num : unsure) {
        int flag = 0;
        double sum_r = 0, sum_g = 0, sum_b = 0;
        for (auto& neib : unsure_neighbor[num]) {
            if (!unsure_neighbor.count(neib)) {  // 如果一个面的连通域内的面是已知的，将其颜色加入计算
                flag++;
                sum_r += triangles[neib].r;
                sum_g += triangles[neib].g;
                sum_b += triangles[neib].b;
            }
        }
        if (flag == 3) {
            triangles[num].r = static_cast<int>(round(sum_r / 3));
            triangles[num].g = static_cast<int>(round(sum_g / 3));
            triangles[num].b = static_cast<int>(round(sum_b / 3));
            unsure_neighbor.erase(num);
        }
    }
    // 利用非齐次线性方程组，求解预处理后依然未知的面
    int unsure_num = unsure_neighbor.size();
    std::vector<std::vector<double>> matrix_A(unsure_num, std::vector<double>(unsure_num));        // 系数矩阵
    std::vector<double> matrix_B_r(unsure_num), matrix_B_g(unsure_num), matrix_B_b(unsure_num);    // 构建不同颜色的B矩阵
    std::map<int, int> unsure_to_x;        // 按顺序定义到未知面的映射
    int x_index = 0;
    for (auto& un : unsure_neighbor) {   // 将未知面从小到大按顺序编号0，1，2
        unsure_to_x[un.first] = x_index;
        x_index++;
    }
    for (auto& cur : unsure_neighbor) {
        int row_num = unsure_to_x[cur.first];
        matrix_A[row_num][row_num] = 3;    // 主对角线上元素均为3
        double sum_r = 0, sum_g = 0, sum_b = 0;
        for (auto& it : cur.second) {
            if (!unsure_neighbor.count(it)) { // 如果一个面的连通域内的面是已知的，将其颜色加入计算
                sum_r += triangles[it].r;
                sum_g += triangles[it].g;
                sum_b += triangles[it].b;
            } else {
                matrix_A[row_num][unsure_to_x[it]] = -1;   // 如果一个面的连通域内的面是未知的，其A矩阵对应系数为-1
            }
            matrix_B_r[row_num] = sum_r;
            matrix_B_g[row_num] = sum_g;
            matrix_B_b[row_num] = sum_b;
        }
    }
    // 利用初等行变换，求解A矩阵的逆
    std::vector<std::vector<double>> inv_matrix_A;
    calc_inversion(matrix_A, inv_matrix_A, unsure_num);

    // 将得到的逆矩阵和B矩阵相乘，得到输出
    int index = 0;
    for (auto& it : unsure_to_x) {
        double total_red = 0;
        double total_green = 0;
        double total_blue = 0;
        for (int i = 0; i < unsure_num; i++) {
            total_red += inv_matrix_A[index][i] * matrix_B_r[i];
            total_green += inv_matrix_A[index][i] * matrix_B_g[i];
            total_blue += inv_matrix_A[index][i] * matrix_B_b[i];
        }
        index++;
        triangles[it.first].r = static_cast<int>(round(total_red));
        triangles[it.first].g = static_cast<int>(round(total_green));
        triangles[it.first].b = static_cast<int>(round(total_blue));
    }
}

bool data_to_txt(const std::vector<Triangle>& triangles, const std::string k_output_filename) {
    std::ofstream file;
    file.open(k_output_filename, std::ios::out);
    if (!file) {
        printf("The outfile was not opened!!\n");
        exit(0);
    }
    int size = triangles.size();
    file << size << std::endl;
    for (int i = 0; i < size; i++) {
        file << triangles[i].p0.x << " " << triangles[i].p0.y << " " << triangles[i].p0.z << " "
            << triangles[i].p1.x << " " << triangles[i].p1.y << " " << triangles[i].p1.z << " "
            << triangles[i].p2.x << " " << triangles[i].p2.y << " " << triangles[i].p2.z << " "
            << triangles[i].r << " " << triangles[i].g << " " << triangles[i].b << std::endl;
    }
    file.close();
    return 1;
}

int main(int argc, char* argv[]) {
    // 此为原始数据文件路径，提交时请勿修改，擅自修改会导致运行找不到文件而报错
    const std::string k_input_filename {argv[1]};

    // 此为结果文件路径，提交时请勿修改，擅自修改会导致评测系统无法获取结果文件
    const std::string k_output_filename {argv[2]};

    int N;       // 三角形总数
    std::vector<Triangle> triangles;    // 三角形集合
    if (!part_params_load(N, k_input_filename, triangles)) return 0;  // 导入数据
    calc_unsure_color(N, triangles);                   // 求解不确定的面
    if (!data_to_txt(triangles, k_output_filename)) return 0;    // 输出到文件
    return 0;
}
