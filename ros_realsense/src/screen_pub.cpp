#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RGB_Image_Publisher");
  ros::NodeHandle nh;

  // 토픽으로 이미지를 발행하는 Publisher 지정
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("RGB_Image", 1);

  // 원하는 해상도 설정
  int target_width = 640;
  int target_height = 360;

  // 원하는 화면 영역의 좌표와 크기 설정
  int capture_x = 0;  // 시작 x 좌표 설정
  int capture_y = 20;  // 시작 y 좌표 설정
  //1,2번 모니터중 왼쪽 모니터만을 출력함 (더블모니터 사용)
  int capture_width = 1920;  
  int capture_height = 1040;

  // 루프 주기를 30Hz로 설정
  ros::Rate loop_rate(30);

  // 특정 디스플레이에 연결
  Display* display = XOpenDisplay(":1"); // 0번 디스플레이 선택
  if (!display) {
    ROS_ERROR("Cannot open X display");
    return 1;
  }
  
  // 선택한 디스플레이의 루트 윈도우 지정
  Window root = DefaultRootWindow(display);

  while (ros::ok())
  {
    // 원하는 화면 영역 캡처
    XImage* ximage = XGetImage(display, root, capture_x, capture_y, capture_width, capture_height, AllPlanes, ZPixmap);
    cv::Mat full_resolution(capture_height, capture_width, CV_8UC4, ximage->data);
    
    // 이미지 크기 조정
    cv::Mat resized_image;
    cv::resize(full_resolution, resized_image, cv::Size(target_width, target_height));

    // OpenCV Mat 형식의 이미지를 ROS Image 메시지로 변환
    cv_bridge::CvImage img_bridge;
    img_bridge.header = std_msgs::Header();
    img_bridge.encoding = sensor_msgs::image_encodings::BGRA8; // BGRA 형식으로 변경
    img_bridge.image = resized_image;

    // ROS Image 메시지로 변환
    sensor_msgs::ImagePtr img_msg = img_bridge.toImageMsg();

    // ROS Image 메시지를 토픽으로 발행
    image_pub.publish(img_msg);

    // 메모리 해제
    XDestroyImage(ximage);

    // 루프 주기를 유지하기 위해 sleep
    loop_rate.sleep();

    // ROS 콜백 함수를 호출하여 노드가 종료되지 않도록 함
    ros::spinOnce();
  }

  XCloseDisplay(display);

  return 0;
}
