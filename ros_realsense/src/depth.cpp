#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

//CSRT 알고리즘에 대한 Class 생성 
class Depth_Publisher
{
// Private 멤버 변수 

public:
// 해당 생성자 선언 
Depth_Publisher() 
{ //발행되는 원본 이미지와 ROI 영역에 대한 시작점과 끝점을 수신
    depth_image_sub = nh.subscribe("/Depth_Image", 1, &Depth_Publisher::DepthCallback, this);
    current_point_sub = nh.subscribe("/Object_tracking/point", 1, &Depth_Publisher::PointCallback, this);
//Tracking에 따른 중심좌표와 속도에 대한 정보를 발행 
    depth_pub = nh.advertise<std_msgs::UInt16>("Object_tracking/depth", 1);

}

//private 멤버 함수
private:

    //Tracking에 필요할 원본 이미지와 정보를 구독 
    ros::NodeHandle nh;
    ros::Subscriber depth_image_sub;
    ros::Subscriber current_point_sub;   
    //Tracking하고 있는 Object의 중심좌표와 속도를 Publish함
    ros::Publisher depth_pub;
    
    ushort depth_value = 0;
    Point2f current_center;
    int current_width = 0;
    bool depth_start = false;

void PointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // 받은 메시지에서 좌표 정보와 너비 값을 추출
  
    current_center.x = msg->x;
    current_center.y = msg->y;
    current_width = msg->z;
    depth_start = true;

    
    if (current_width < 1)
    {
        depth_start = false;
    }
    
}
    


void DepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //수신받은 이미지를 OpenCV 이미지로 변환
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat normalized_depth;
    Mat color_depth;
    ushort neighbor_depth = 0;
    int depth_radius = current_width; //좌표로 대치 
    //주변 깊이 값을 계산할 영역을 추적 영역에 비례해 지정


    // 깊이에 대한 값을 정규화시켜 출력 -> 깊이 값에 대해 직관적으로 확인
    normalize(cv_ptr->image, normalized_depth, 0, 255, NORM_MINMAX, CV_8U);
    // 해당 깊이 이미지를 가시광선 영역으로 시각화 해줌 (빨간색이 가까운 영역)
    applyColorMap(normalized_depth, color_depth, COLORMAP_RAINBOW);  
    //빨간색 영역이 먼 영역으로 하면 해당 코드를 주석 해제 
    // applyColorMap(normalized_depth, color_depth, COLORMAP_JET);
             
    if (depth_start)
    {   
     // 최신 현위치 중심값을 가져오는 부분
        depth_value = cv_ptr->image.at<ushort>(current_center);

        //해당 위치의 Depth 정보가 없다면  
        if (depth_value < 1)
        {   
            //원점을 기준으로 반경을 좌/우측으로 나눠줌 
            int left_sum = 0;
            int left_count = 0;
            int right_sum = 0;
            int right_count = 0;
            //갯수 임계값 지정 
            int count_threshold = 10;
   
            // 좌/우측 x축 끝 경계를 지름을 기준으로 정의해줌
            int left_boundary = current_center.x - depth_radius;
            int right_boundary = current_center.x + depth_radius;

            // -radius에서 +radius까지 x,y좌표에 대해 반복문을 실행(이중반복문) 
            for (int dx = -depth_radius; dx <= depth_radius; ++dx)
            {  
                for (int dy = -depth_radius; dy <= depth_radius; ++dy)
                {
                    // 현 중심 위치에 해당 좌표 변화량을 더해서 이웃 좌표로 설정
                    Point2f neighbor_point = current_center + Point2f(dx, dy);
                    
                    // 모든 주변 영역이 Window 영역 안에 존재할 경우 -> OpenCV 에러 방지 
                    if (neighbor_point.x >= 0 && neighbor_point.x < cv_ptr->image.cols &&
                        neighbor_point.y >= 0 && neighbor_point.y < cv_ptr->image.rows)
                    {   
                        //이웃의 깊이 값을 다 넣어줌 
                        neighbor_depth = cv_ptr->image.at<ushort>(neighbor_point);
                        
                        //깊이 데이터가 존재하는 영역에 대해서만
                        if (neighbor_depth != 0)
                        {   
                            // 좌측의 픽셀에 해당대면 좌측의 값에 저장 
                             if (left_boundary < neighbor_point.x && neighbor_point.x < current_center.x)
                            {
                                left_sum += neighbor_depth;
                                left_count++;
                            } 
                            // 우측에 픽셀에 해당되면 우측 값에 저장 
                            else if (current_center.x < neighbor_point.x && neighbor_point.x < right_boundary)
                            {
                                right_sum += neighbor_depth;
                                right_count++;
                            }
                         
                         }
                    }
                }
            }
            

            //오류 방지를 위해 2개의 영역 모두 임계값을 넘어갈 시에 
            if (right_count > count_threshold && left_count > count_threshold)
            {
                if (right_sum / right_count > left_sum / left_count)
                {
                    depth_value = right_sum / right_count;
                }
                else
                {
                    depth_value = left_sum / left_count;
                }
            }

            else if (right_count > count_threshold)
            {
                depth_value = right_sum / right_count;
            }
            else if (left_count > count_threshold)
            {
                depth_value = left_sum / left_count;
            }
            else
            {
                depth_value = -1;
                putText(color_depth, "NO DEPTH DATA", Point(current_center), FONT_HERSHEY_SIMPLEX, 1 , Scalar(255,255,255), 2);
            }

        }
        //RGB 이미지에서 지정된 ROI 영역을 Detph에서도 지정해줌
        circle(color_depth, current_center, depth_radius, Scalar(200,200,200), 2);
        circle(color_depth, current_center, 3 , Scalar(0, 0, 0), 2);
        PublishDepth();
    }
      
        // Display colored depth image with ROI
        imshow("Depth Image", color_depth);
        waitKey(30);
    }

void PublishDepth()
{
    std_msgs::UInt16 depth_msg;
    depth_msg.data = depth_value;
    depth_pub.publish(depth_msg);
    ROS_INFO("Point:(%.0f, %.0f) Depth :  %d", current_center.x, current_center.y, depth_value);
}

};
    

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Depth_publisher");
    Depth_Publisher roi_display;
    ros::Rate loop_rate(30);
    ROS_INFO("Starting..");
    
    while (ros::ok()) 
    {
        // 'Esc' 키를 누르면 프로그램 종료
        int key = waitKey(10);

        if (key == 27) 
        {
            destroyAllWindows();
            break;
        }
        
        // ROS 이벤트 처리 및 주기 대기
        ros::spinOnce();
        loop_rate.sleep();
    }
    // 창 닫기
    ROS_INFO("Shutting down the Depth node..");
    return 0;
        
}