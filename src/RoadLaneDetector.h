#pragma once
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class RoadLaneDetector
{
private:
    double previous_heading_error = 0.0; // 이전 헤딩 에러


public:

    vector<int> lx, rx;
    vector<int> ly, ry;
    // x, y 값을 저장할 벡터 생성
    vector<int> x_vals;
    vector<int> y_vals;
    vector<int> y_vals_left, y_vals_right;
vector<int> histogram;
Mat transform_matrix;
//    vector<double> left_coeffs;
//     vector<double> right_coeffs;
double calculateRadius(double a, double b, double x);
vector<double> polyfitGradientDescent(const vector<int>& x_vals, const vector<int>& y_vals, double learning_rate, int iterations);
Mat Reverse_transformed(Mat result , Mat transfrom_matrix);
void processLaneLine(Mat& img_frame, const Vec4f& line);
double calculateSlope(const Point& p1, const Point& p2);
Mat bird_eye_view (Mat img_frame);
vector <int> Start_lane_detection (Mat mask);
Mat sliding_window(Mat img_frame,Mat mask,int left_base,int right_base);
Mat img_filter(Mat transformed_frame);
 const Vec4f li (vector<int> left_positions ,vector<int> y_vals);
     vector<double> polyfit(const vector<int>& x_vals, const vector<int>& y_vals, int degree);
double polynomial(double x, const vector<double>& coeffs);
// double calculateCurvature(double a, double b, double x);

double secondDerivative(const function<double(double)>& func, double t, double h);
double derivative(const function<double(double)>& func, double t, double h);
double calculateCurvature(const vector<double>& coeffs, double x);
};