#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>


using namespace cv;
using namespace std;

//CSRT 알고리즘에 대한 Class 생성 
class vel_Pub 
{

// Private 멤버 변수 
private:

    ros::NodeHandle nh;
    ros::Subscriber current_point_sub;
    ros::Publisher velocity_Pub;

    Point2f current_center; 
    Point2f velocity;

    float current_width;
    float width_ratio;
    
    Rect2d tracked_rect;

    vector<ros::Time> prev_time;
    vector<Point2f> prev_point;
    vector<Point2f> predict_point;
    vector<float> prev_widths;
    
    Mat mask_image;


public:
    vel_Pub() : prev_point(), predict_point(), width_ratio(0)
    {
        current_point_sub = nh.subscribe("Object_tracking/point", 1, &vel_Pub::PointCallback, this);
        velocity_Pub = nh.advertise<geometry_msgs::Vector3>("Object_tracking/velocity", 1);
        mask_image = Mat::zeros(Size(640, 480), CV_8UC3);
    }


//private 멤버 함수
private:

void PointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // 받은 메시지에서 좌표 정보와 너비 값을 추출
    current_center.x = msg->x;
    current_center.y = msg->y;
    current_width = msg->z;
    
   if (current_width != 0)
    {
        CalWidth();
        CalVelocity();
        // 너비 값을 벡터에 추가
   
    }
}

// CalWidth 함수 수정
void CalWidth()
{
    int max_vector_size = 10; 
    prev_widths.push_back(current_width);

    if (prev_widths.size() >= max_vector_size)
    {
        float prev_width = prev_widths[0]; 

        width_ratio = current_width / prev_width;
    }

    // 현재 너비 값을 벡터에 추가
    prev_widths.push_back(current_width);
    if (prev_widths.size() > max_vector_size)
    {
        prev_widths.erase(prev_widths.begin());
    }
}

void CalVelocity()
{
    const int max_point = 100;
    const int prev_num = 6;

    ros::Time current_time = ros::Time::now();

    prev_point.push_back(current_center);
    prev_time.push_back(current_time);

    Point2f displacement_vector = current_center - prev_point[prev_point.size() - prev_num];
    double time_diff = (current_time - prev_time[prev_time.size() - prev_num]).toSec();

    velocity.x = (displacement_vector.x / time_diff);
    velocity.y = (displacement_vector.y / time_diff);

    double predict_time = 0.5; 

    Point2f predicted_position = current_center + Point2f(velocity.x * predict_time, velocity.y * predict_time);
    predict_point.push_back(predicted_position);

    if (prev_point.size() > max_point)
    {
        prev_point.erase(prev_point.begin());
        predict_point.erase(predict_point.begin());
    }

 Mat visualization = mask_image.clone();

    // 이전에 그려진 원들을 유지하면서 현재 보정된 포인트를 덮어씌웁니다.
for (const Point2f& future_point : predict_point)
    {
        circle(visualization, future_point, 3, Scalar(200, 0, 108), 2);
    }

    for (const Point2f& actual_point : prev_point)
    {
        circle(visualization, actual_point, 3, Scalar(0, 255, 0), 2);
    }


    imshow("Point Tracker", visualization);

    Velocity_publisher();
}

void Velocity_publisher()
{
    geometry_msgs::Vector3 object_velocity;
    object_velocity.x = velocity.x;
    object_velocity.y = velocity.y;
    object_velocity.z = width_ratio;

    velocity_Pub.publish(object_velocity);
    ROS_INFO("Velocity: (%.1f, %.1f) dW : %.2f",velocity.x, velocity.y, width_ratio);
}

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Center_Record_Node");
    vel_Pub roi_display;
    ros::Rate loop_rate(10);
    ROS_INFO("Starting..");
    
    while (ros::ok()) 
    {
        // 'Esc' 키를 누르면 프로그램 종료
        int key = waitKey(30);

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
    ROS_INFO("Shutting down the tracking node..");
    return 0;
        
}