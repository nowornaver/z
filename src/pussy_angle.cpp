#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "RoadLaneDetector.h"
#include <fstream>  // 파일 입출력을 위한 헤더 추가
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace cv;
using namespace std;

void on_trackbar(int, void*) {}

int l_h = 0, l_s = 0, l_v = 100; // 초기값 설정
int u_h = 255, u_s = 255, u_v = 255;
int l_h_hls = 0, l_l = 0, l_s_hls = 100; // HLS 하한 값
int u_h_hls = 255, u_l = 255, u_s_hls = 255; // HLS 상한 값
void createTrackbars() {
    // 트랙바 생성
    namedWindow("Trackbars");
    createTrackbar("L - H", "Trackbars", &l_h, 179, on_trackbar);
    createTrackbar("L - S", "Trackbars", &l_s, 255, on_trackbar);
    createTrackbar("L - V", "Trackbars", &l_v, 255, on_trackbar);
    createTrackbar("U - H", "Trackbars", &u_h, 179, on_trackbar);
    createTrackbar("U - S", "Trackbars", &u_s, 255, on_trackbar);
    createTrackbar("U - V", "Trackbars", &u_v, 255, on_trackbar);



	createTrackbar("L - H1", "Trackbars", &l_h, 179, on_trackbar);
    createTrackbar("L - L1", "Trackbars", &l_s, 255, on_trackbar);
    createTrackbar("L - S1", "Trackbars", &l_v, 255, on_trackbar);
    createTrackbar("U - H1", "Trackbars", &u_h, 179, on_trackbar);
    createTrackbar("U - L1", "Trackbars", &u_s, 255, on_trackbar);
    createTrackbar("U - S1", "Trackbars", &u_v, 255, on_trackbar);
}
double steering_left_angle=0;
double steering_left_angle_degrees=0 ;
double steering_right_angle=0;
double steering_right_angle_degrees=0;
double steering_angle=0;
int main(int argc , char **argv)
{

    ofstream csv_file1("polynomial_coefficients.csv");
    if (csv_file1.is_open()) {
        csv_file1 << "Frame,Polynomial Degree,Left Coefficients,Right Coefficients\n";
    } else {
        cerr << "Unable to open file: polynomial_coefficients.csv" << endl;
        return -1;
    }
    ofstream csv_file("lane_coordinates.csv");
    if (!csv_file.is_open()) {
        cerr << "Error: Could not open the file 'lane_coordinates.csv' for writing." << endl;
        return -1;
    }

    csv_file << "Left X,Left Y,Right X,Right Y\n";  // CSV 헤더

	RoadLaneDetector roadLaneDetector;

	Mat img_frame, img_filter, img_edges, img_mask, img_lines, img_result;
	Mat bird_of_eye_view;
	vector<Vec4i> lines;
	vector<vector<Vec4i>> separated_lines;
	vector<Point> lane;
	string dir;
	vector<double> polyregre;
    VideoCapture video("/dev/video2", cv::CAP_V4L2);
	// VideoCapture video("test_video.mp4");
		if (!video.isOpened())
	{
		cout << "동영상 파일을 열 수 없습니다. \n" << endl;
		getchar();
		return -1;
	}

	video.read(img_frame);
	
	if (img_frame.empty())
	{
		cout << "영상 프레임이 비어 있습니다.\n";
		return -1;
	}

	VideoWriter writer;
	int codec = VideoWriter::fourcc('X', 'V', 'I', 'D'); // 원하는 코덱 선택
	double fps = 25.0;                                    // 프레임
	string filename = "./result.avi";                     // 결과 파일

	writer.open(filename, codec, fps, img_frame.size(), true);
	if (!writer.isOpened())
	{
		cout << "출력을 위한 비디오 파일을 열 수 없습니다. \n";
		return -1;
	}
	// get() 함수로 카메라 또는 동영상 파일로부터 여러 정보를 받아 올 수 있음. 매개변수는 열거형 상수 Properties 가 들어감
    ros::init(argc, argv, "push_angle_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);
    geometry_msgs::Twist push;
    ros::Publisher publisher_push = n.advertise<geometry_msgs::Twist>("push", 1000);

int frame_count = 0 ;
	while (ros::ok)
	{
    frame_count++;
		// 1. 원본 영상을 읽어온다.
		if (!video.read(img_frame))
			break;
		        // 이미지 크기를 1280x720으로 리사이즈
        cv::resize(img_frame, img_frame, cv::Size(640,480));
int width = 640;
int height = 480;

	Mat transformed_frame = roadLaneDetector.bird_eye_view(img_frame);
	Mat mask = roadLaneDetector.img_filter(transformed_frame);     // Canny(img_filter, img_edges, 50, 150); //확인완료
		    //     Mat mask_bottom_half = mask.rowRange(mask.rows / 2, mask.rows);  //mask.rows = 480
				// imshow("mask_bottom_half",mask_bottom_half);
		vector <int> l = roadLaneDetector.Start_lane_detection(mask);
		        Mat msk = mask.clone(); // mask의 복사본 생성
	Mat result = roadLaneDetector.sliding_window(transformed_frame,mask,l[0],l[1]);
    for (size_t i = 0; i < roadLaneDetector.lx.size(); i++) {
            csv_file << roadLaneDetector.lx[i] << "," << roadLaneDetector.y_vals_left[i] << ",";
            if (i < roadLaneDetector.rx.size()) {
                csv_file << roadLaneDetector.rx[i] << "," << roadLaneDetector.y_vals_right[i] << "\n";
            } else {
                csv_file << ",\n";
            }
        }
	// Mat result1= roadLaneDetector.Reverse_transformed(result,roadLaneDetector.transform_matrix);
	// imshow("result1",result1);
 const double WHEELBASE = 0.75; // 75 cm

 if (roadLaneDetector.y_vals.size() % 8 ==0 && roadLaneDetector.x_vals.size() %8 ==0){
	vector<double> left_coeffs = roadLaneDetector.polyfit(roadLaneDetector.lx, roadLaneDetector.y_vals_left, 2);
	vector<double> right_coeffs = roadLaneDetector.polyfit(roadLaneDetector.rx, roadLaneDetector.y_vals_right, 2);
	//coeffs[0] = 상수 coeffs[1]= 1차 coeffs[2] = 2차
 if (csv_file1.is_open()) {
            for (size_t i = 0; i < left_coeffs.size(); ++i) {
                csv_file1 << frame_count << "," << i << "," << left_coeffs[i] << "," << right_coeffs[i] << "\n";
            }
        }
	// cout <<"right_Coeffs" <<right_coeffs[2] <<endl;
    for (double x : roadLaneDetector.lx) {
        double curvature = roadLaneDetector.calculateCurvature(left_coeffs, static_cast<double>(x));
steering_left_angle = atan(WHEELBASE * curvature); // 결과는 라디안 단위
steering_left_angle_degrees = steering_left_angle * (180.0 / M_PI);
        // 결과 출력
        cout.precision(4);
        cout << "Calculated curvature at x = " << x << " is: " << curvature << endl;
    }
	    for (double x : roadLaneDetector.rx) {
        double curvature = roadLaneDetector.calculateCurvature(right_coeffs, static_cast<double>(x));
steering_right_angle = atan(WHEELBASE * curvature); // 결과는 라디안 단위
steering_right_angle_degrees = steering_right_angle * (180.0 / M_PI);
        // 결과 출력
        cout.precision(4);
        cout << "Calculated curvature at x = " << x << " is: " << curvature << endl;
    }
steering_angle = (steering_left_angle_degrees + steering_right_angle_degrees)/2;
 }

push.angular.z = steering_angle ;
publisher_push.publish(push);
cout <<"Steering_angle" <<steering_angle<<endl;

// 곡률 (단위: 1/미터)

// 방향각 계산


// 결과를 디그리로 변환
		// imshow("img_frame",img_frame);
		// imshow("transformed_frame",transformed_frame);
		imshow("mask",mask);
		imshow("result",result);
	writer.write(img_frame);
		// esc 키 종료
		if (waitKey(100) == 27){
	
			break; 
			
			}

	}



	video.release();
	writer.release();
	
	return 0;
}
