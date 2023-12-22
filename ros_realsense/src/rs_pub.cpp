#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>

class RealsensePub
{
//Private Member 변수를 생성 
private:

  ros::NodeHandle nh;
  ros::Publisher ColorImage;
  ros::Publisher DepthImage;
  rs2::pipeline pipe;

  //깊이 프레임을 Color Frame을 기준으로 정렬을 수행
  rs2::align align{RS2_STREAM_COLOR};
  
  //발행되는 img 토픽의 기본 너비와 높이 값을 지정 
  int width = 640;
  int height = 480;

  //RGB Image Publish함
  void ColorImgPub(const rs2::video_frame& color_frame, int width, int height)
  {
   //cv_bridge를 통해 OpenCV 객체를 생성
      cv_bridge::CvImage color_img;
      //이미지 메세지 헤더를 설정
      color_img.header = std_msgs::Header();
      //Color 이미지를 8비트 RGB 형식으로 인코딩
      color_img.encoding = sensor_msgs::image_encodings::RGB8;

      //이미지 데이터를 변환하고 ROS 이미지 메시지로 변환 후 이미지 토픽을 발행 
      color_img.image = cv::Mat(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
      sensor_msgs::ImagePtr color_img_msg = color_img.toImageMsg();
      ColorImage.publish(color_img_msg);    
  }

  //동일하게 깊이 이미지 정보를 발행하는 함수
  void DepthImgPub(const rs2::frame& depth_frame, int width, int height)
  {   
      cv_bridge::CvImage depth_img;
      depth_img.header = std_msgs::Header();

     //깊이 이미지는 16비트의 부호 없는 정수 형식이 기본 인코딩
      depth_img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;   
      depth_img.image = cv::Mat(cv::Size(width, height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
      sensor_msgs::ImagePtr depth_img_msg = depth_img.toImageMsg();
      DepthImage.publish(depth_img_msg);
  }

//Public Member : Class 생성자 AlignImgPub 함수

public:
    RealsensePub() : nh("~")
    { 
        //색상 이미지와 깊이 이미지를 각각 발행함
        ColorImage = nh.advertise<sensor_msgs::Image>("/RGB_Image", 1);
        DepthImage = nh.advertise<sensor_msgs::Image>("/Depth_Image", 1);
        pipe.start();
    }

  //정렬된 이미지를 발행하는 함수 (main 함수에서 호출)
  void AlignImgPub()

    {   //RealSense 파이프라인에서 프레임을 기다린 이후 
        rs2::frameset frames = pipe.wait_for_frames();
        //생성된 정렬에 대한 객체를 통해 프레임을 정렬 
        rs2::frameset aligned_frames = align.process(frames);

        // 정렬된 프레임셋에서 컬러 프레임과 깊이 프레임 가져오기
        rs2::video_frame color_frame = aligned_frames.get_color_frame();
        rs2::frame depth_frame = aligned_frames.get_depth_frame();

        //가져온 컬러 프레임과 깊이 프레임을 발행하는 함수
        ColorImgPub(color_frame, width, height);
        DepthImgPub(depth_frame, width, height);
    }


};

//코드의 Main 함수 
int main(int argc, char** argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "Image_Publisher");
    RealsensePub RealsensePub;
    //성공 시 터미널에 메세지 출력 
    ROS_INFO("Publish SUCCESS!!");
  
    //30hz를 주기로 이미지를 발행
    ros::Rate loop_rate(30);
    while (ros::ok())

    {
      RealsensePub.AlignImgPub();

    }
    



    return 0;
}
