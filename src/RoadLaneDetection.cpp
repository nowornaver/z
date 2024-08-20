#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "RoadLaneDetector.h"
#include <iostream>
#include <string>
#include <vector>
// L = 75cm
double RoadLaneDetector::derivative(const function<double(double)>& func, double t, double h) {
    return (func(t + h) - func(t - h)) / (2 * h);
}

// 2차 미분 계산 함수
double RoadLaneDetector::secondDerivative(const function<double(double)>& func, double t, double h) {
    return (func(t + h) - 2 * func(t) + func(t - h)) / (h * h);
}

// 곡률 계산 함수
double RoadLaneDetector:: calculateCurvature(const vector<double>& coeffs, double x) {
    double a0 = coeffs[0]; //상수
    double a1 = coeffs[1]; //1차
    double a2 = coeffs[2]; //2차

    double first_derivative = a1 + 2 * a2 * x; //1차 도함수
    double second_derivative = 2 * a2; //2차 도함수

    double curvature = abs(second_derivative * first_derivative) / abs(pow(first_derivative , 3));

    return curvature;
}
double RoadLaneDetector::calculateRadius(double a, double b, double x) {
    double numerator = 1 + (2 * a * x + b) * (2 * a * x + b);  // (1 + (2ax + b)^2)
    double denominator = 2 * fabs(a);  // 2|a|

    double radius = numerator / denominator;

    return radius; 
    }
// double RoadLaneDetector::calculateCurvature(double a, double b, double x) {
//     double radius = calculateRadius(a, b, x);
//     double curvature = 1.0 / radius;  // 커브쳐는 반경의 역수

//     return curvature;
// }

vector<double> RoadLaneDetector::polyfitGradientDescent(const vector<int>& x_vals, const vector<int>& y_vals, double learning_rate, int iterations) {
    int n = x_vals.size();
    double a = 1.0, b = 0.0, c = 0.0; // 초기 계수값

    for (int iter = 0; iter < iterations; iter++) {
        double da = 0.0, db = 0.0, dc = 0.0;

        for (int i = 0; i < n; i++) {
            double x = x_vals[i];
            double y = y_vals[i];
            double y_pred = a * x * x + b * x + c;
            double error = y_pred - y;

            da += error * x * x;  // a에 대한 편미분
            db += error * x;      // b에 대한 편미분
            dc += error;          // c에 대한 편미분
        }

        // 평균화된 그래디언트
        da /= n;
        db /= n;
        dc /= n;

        // 파라미터 업데이트
        a -= learning_rate * da;
        b -= learning_rate * db;
        c -= learning_rate * dc;
    }

    return {a, b, c}; // 최적의 a, b, c를 반환
}
vector<double> RoadLaneDetector::polyfit(const vector<int>& x_vals, const vector<int>& y_vals, int degree) {
      vector<double> coeffs_vector;

   if (!x_vals.empty() && !y_vals.empty()) {
   
    int n = x_vals.size(); //n 행의 수 
    // cout <<"n = " <<n <<endl; 
    Mat X(n, degree + 1, CV_64F); //degree+1 열
    Mat Y(n, 1, CV_64F);

    for (int i = 0; i < n; i++) {
        Y.at<double>(i, 0) = static_cast<double>(y_vals[i]);
        for (int j = 0; j <= degree; j++) {
            X.at<double>(i, j) = pow(x_vals[i], j); //x_vals[i]^j
        }
    }

    // (X^T * X) * a = X^T * Y 를 풀어서 회귀계수 a를 구함
    Mat Xt = X.t(); //행렬 x의 전치행렬 
    Mat XtX = Xt * X;
    Mat XtY = Xt * Y;
    Mat coeffs = XtX.inv() * XtY;

    for (int i = 0; i < coeffs.rows; i++) {
        coeffs_vector.push_back(coeffs.at<double>(i, 0));
    }
   
    return coeffs_vector; 
    }
    return coeffs_vector;
    
}
Mat RoadLaneDetector::Reverse_transformed(Mat result , Mat transfrom_matrix) {
Mat a ; 
cv::warpPerspective(result, a, transfrom_matrix, cv::Size(640, 480));



    return a;
}
void RoadLaneDetector::processLaneLine(Mat& img_frame, const Vec4f& line) {
    // 기울기와 절편 추출
    float slope = line[1] / line[0]; // 기울기
    float intercept = line[3] - slope * line[2]; // 절편

    // x, y 값을 저장할 벡터 생성


    // 직선 위의 점 생성
    for (int x = 0; x < img_frame.cols; x++) {
        int y = static_cast<int>(slope * x + intercept); // y = mx + b
        if (y >= 0 && y < img_frame.rows) { // 유효한 y 범위 체크
            x_vals.push_back(x);
            y_vals.push_back(y);
        }
    }
  for (size_t i = 0; i < x_vals.size() - 1; i++) {
        cv::line(img_frame, Point(x_vals[i], y_vals[i]), Point(x_vals[i + 1], y_vals[i + 1]), cv::Scalar(0, 255, 0), 1);
    }

  
}
 const Vec4f RoadLaneDetector::li (vector<int> left_positions ,vector<int> y_vals) {
    if (!left_positions.empty() && !y_vals.empty()) {
        vector<Point2f> points;
        for (size_t i = 0 ; i<left_positions.size() ; i ++) {
            points.push_back(Point2f(left_positions[i] , y_vals[i]));
        }

        Vec4f line ; 
        cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);    
        


        return line;
        }
    
    return Vec4f(0,0,0,0);

    
 }

Mat RoadLaneDetector::bird_eye_view (Mat img_frame) {

int width = 640;
int height = 480;
		Point2f src_vertices[4];

    src_vertices[0] = Point(width*0.02 , height*0.8);  // Bottom-left
    src_vertices[1] = Point(width * 0.85, height*0.8);  // Bottom-right
    src_vertices[2] = Point(width * 0.56, height * 0.64);  // Top-right
    src_vertices[3] = Point(width * 0.25, height * 0.64);  // Top-left
	// cv::circle(img_frame,src_vertices[0],5,(0,0,255),-1);
	// cv::circle(img_frame,src_vertices[1],5,(0,0,255),-1);
	// cv::circle(img_frame,src_vertices[2],5,(0,0,255),-1);
	// cv::circle(img_frame,src_vertices[3],5,(0,0,255),-1);
	 Point2f dst_vertices[4];
    dst_vertices[0] = Point(width * 0.3,height);  // Bottom-left
    dst_vertices[1] = Point(width * 0.8, height);  // Bottom-right
    dst_vertices[2] = Point(width*0.8, 0);  // Top-right
    dst_vertices[3] = Point(width*0.3, 0);  // Top-left
    // cv::circle(img_frame,dst_vertices[0],5,(0,0,255),-1);
	// cv::circle(img_frame,dst_vertices[1],5,(0,0,255),-1);
	// cv::circle(img_frame,dst_vertices[2],5,(0,0,255),-1);
	// cv::circle(img_frame,dst_vertices[3],5,(0,0,255),-1);
	Mat matrix = cv::getPerspectiveTransform(src_vertices,dst_vertices);
    transform_matrix = cv::getPerspectiveTransform(dst_vertices,src_vertices);
	Mat transformed_frame;
cv::warpPerspective(img_frame, transformed_frame, matrix, cv::Size(640, 480));
    Mat mask = Mat::zeros(img_frame.size(), CV_8UC1);
    Point poly[1][4];

    poly[0][0] = dst_vertices[0];
    poly[0][1] = dst_vertices[1];
    poly[0][2] = dst_vertices[2];
    poly[0][3] = dst_vertices[3];
    const Point* pts[1] = { poly[0] };
    int npts = 4;

    // 다각형 영역을 1로 채우기
    cv::fillPoly(mask, pts, &npts, 1, Scalar(255));
 Mat masked_frame;
    cv::bitwise_and(transformed_frame, transformed_frame, masked_frame, mask);
    return masked_frame; 
}
vector<int> RoadLaneDetector::Start_lane_detection(Mat mask) {

    vector<int> lane(2, 0); // 크기가 2인 벡터로 초기화 (왼쪽과 오른쪽 차선 위치)
 histogram.resize(mask.cols, 0); // 멤버 변수 histogram 초기화
         for (int i = 0; i < mask.cols; i++) {
            histogram[i] = countNonZero(mask.col(i));
        }

        // 히스토그램 출력
  
		int midpoint = histogram.size() / 2; //midpoint = 320

// // 4. 왼쪽 차선의 시작 위치를 찾습니다.
int left_base = 0;
for (int i = 0; i < midpoint; i++) {
    if (histogram[i] > histogram[left_base]) {
        left_base = i;
        lane[0] = left_base;
    }
}

// // 5. 오른쪽 차선의 시작 위치를 찾습니다.
int right_base = midpoint;
for (int i = midpoint; i < histogram.size(); i++) {
    if (histogram[i] > histogram[right_base]) {
        right_base = i;
        lane[1]= right_base;
    }
}


    return lane;
}

Mat RoadLaneDetector::sliding_window(Mat img_frame, Mat mask, int left_base, int right_base) {
    Mat msk = mask.clone();
    int y = mask.rows;
if (left_base!=0 && right_base!=0) {
    while (y > 0) {
        // 왼쪽 차선 범위 설정
        int left_start = max(0, left_base - 50);  // 윈도우 너비 확장
        int left_end = min(mask.cols, left_base + 50);  // 윈도우 너비 확장
        if (left_start >= left_end) break; // 유효한 범위가 없으면 종료

        Mat img_left = mask.rowRange(y - 120, y).colRange(left_start, left_end);  // 윈도우 높이 확장 (120 픽셀로 증가)
        vector<vector<Point>> contours_left;
        findContours(img_left, contours_left, RETR_TREE, CHAIN_APPROX_SIMPLE);
        
        // 왼쪽 차선의 중심 계산
        if (!contours_left.empty()) {
            Moments M = moments(contours_left[0]);
            if (M.m00 != 0) {
                int cx = static_cast<int>(M.m10 / M.m00);
                lx.push_back(left_start + cx);
                x_vals.push_back(left_start+cx);

                left_base = left_start + cx;
                y_vals_left.push_back(y - 60); // y 값 추가 (중앙으로)
                y_vals.push_back(y-60);
                drawContours(img_frame, contours_left, -1, Scalar(255, 123, 0), 2); // 왼쪽 차선 그리기
            }
        }

        // 오른쪽 차선 범위 설정
        int right_start = max(0, right_base - 50);  // 윈도우 너비 확장
        int right_end = min(mask.cols, right_base + 50);  // 윈도우 너비 확장
        if (right_start >= right_end) break; // 유효한 범위가 없으면 종료

        Mat img_right = mask.rowRange(y - 120, y).colRange(right_start, right_end);  // 윈도우 높이 확장 (120 픽셀로 증가)
        vector<vector<Point>> contours_right;
        findContours(img_right, contours_right, RETR_TREE, CHAIN_APPROX_SIMPLE);
        
        // 오른쪽 차선의 중심 계산
        if (!contours_right.empty()) {
            Moments M = moments(contours_right[0]);
            if (M.m00 != 0) {
                int cx = static_cast<int>(M.m10 / M.m00);
                rx.push_back(right_start + cx);
                x_vals.push_back(right_start+cx);
                right_base = right_start + cx;
                y_vals.push_back(y-60);
                y_vals_right.push_back(y - 60); // y 값 추가 (중앙으로)
                drawContours(img_frame, contours_right, -1, Scalar(255, 0, 0), 2); // 오른쪽 차선 그리기
            }
        }

        // 현재 윈도우 표시
        rectangle(msk, Point(left_base - 50, y), Point(left_base + 100, y - 120), Scalar(255, 255, 255), 2);
        rectangle(msk, Point(right_base - 50, y), Point(right_base + 100, y - 120), Scalar(255, 255, 255), 2);

        // 다음 윈도우로 이동
        y -= 120;  // 윈도우가 큰 만큼 더 많이 이동
    }
}
    return msk;
}

Mat RoadLaneDetector::img_filter (Mat transformed_frame) {



namedWindow("Trackbars");

    // 트랙바 생성
   
    // 트랙바 값 읽기

        Mat hsv_transformed_frame;
		
        cvtColor(transformed_frame, hsv_transformed_frame, COLOR_BGR2HSV);
    int l_h = getTrackbarPos("L - H", "Trackbars");
    int l_s = getTrackbarPos("L - S", "Trackbars");
    int l_v = getTrackbarPos("L - V", "Trackbars");
    int u_h = getTrackbarPos("U - H", "Trackbars");
    int u_s = getTrackbarPos("U - S", "Trackbars");
    int u_v = getTrackbarPos("U - V", "Trackbars");

    Mat hls_transformed_frame;
    cvtColor(transformed_frame,hls_transformed_frame,COLOR_BGR2HLS);


    int l_h_hls = getTrackbarPos("L - H1", "Trackbars");
    int l_l = getTrackbarPos("L - L1", "Trackbars");
    int l_s_hls = getTrackbarPos("L - S1", "Trackbars");
    int u_h_hls = getTrackbarPos("U - H1", "Trackbars");
    int u_l = getTrackbarPos("U - L1", "Trackbars");
    int u_s_hls = getTrackbarPos("U - S1", "Trackbars");

    Mat mask1;

    Scalar lower1(l_h_hls , l_l , l_s_hls);
    Scalar upper1(u_h_hls,u_l,u_s_hls);
    inRange(hls_transformed_frame , lower1 , upper1,mask1);
    imshow("hls_trans",hls_transformed_frame);
    imshow("iss",mask1);
        // Create mask using the trackbar values
        Mat mask;
        Scalar lower(l_h, l_s, l_v);
        Scalar upper(u_h, u_s, u_v);
        cv::inRange(hsv_transformed_frame,lower, upper,mask);
Mat combined;
    bitwise_or(mask, mask1, combined);


return combined;
}



