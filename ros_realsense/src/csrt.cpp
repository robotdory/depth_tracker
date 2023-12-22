#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

//CSRT 알고리즘에 대한 Class 생성 
class CSR_Tracker
{

// Private 멤버 변수 
private:

    //Tracking에 필요할 원본 이미지와 정보를 구독 
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Subscriber roi_sub;
    ros::Subscriber roi_start_sub;
    ros::Subscriber roi_end_sub;

    //Tracking하고 있는 Object의 중심좌표와 속도를 Publish함
    ros::Publisher current_point_pub;
    ros::Time matching_time; //reset time 
    
    Mat original_image; //구독된 원본 이미지를 저장할 MAT객체
    Mat roi_image;

    //Tracking 시작 및 Update에 필요한 Bool 함수 
    bool roi_ready = false;
    bool track_start = true; 
    bool track_reset =false; 

    Point roi_start; //ROI 영역을 저장할 Point 객체 생성
    Point roi_end;

    Ptr<Tracker> tracker; // tracker 객체 
    Rect2d tracked_rect;
    Point2f current_center;  
    float matching_threshold = 0.4;


public:
// 해당 생성자 선언 
CSR_Tracker() 
{ //발행되는 원본 이미지와 ROI 영역에 대한 시작점과 끝점을 수신
    image_sub = nh.subscribe("ROI_Detection/Original_Image", 1, &CSR_Tracker::ImageCallback, this);
    roi_sub = nh.subscribe("ROI_Detection/ROI_Image", 1, &CSR_Tracker::ROICallback, this);
    roi_start_sub = nh.subscribe("ROI_Detection/roi_Start", 1, &CSR_Tracker::StartCallback, this);
    roi_end_sub = nh.subscribe("ROI_Detection/roi_End", 1, &CSR_Tracker::EndCallback, this);

//Tracking에 따른 중심좌표와 속도에 대한 정보를 발행 
    current_point_pub = nh.advertise<geometry_msgs::Point>("Object_tracking/point", 1);
}


//private 멤버 함수
private:

//이미지 콜백에 대한 함수 
void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    //OpenCV 형식으로 해당 이미지를 저장 
    original_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    //원본 이미지가 존재한다면 
    if (!original_image.empty())
    {   
        //노이즈 제거를 위한 가우시안 블러처리 
        GaussianBlur(original_image, original_image, Size(5, 5), 0);

        if (track_start && roi_ready)
        {
            StartTracking();
        }

        if (!track_start)
        {
             bool track_update = tracker->update(original_image, tracked_rect);

            if (track_update)
            {
                UpdateTracking();
            }
            else
            {
                track_reset = true;
            }

            //추적에 실패하거나 영역 주변의 깊이 값이 도출이 안돼면    
            if(track_reset)
            {   
                ResetTracking();
            }                   
        }
        imshow("CSRT Tracking", original_image);
        waitKey(20);
    }
}

void StartTracking()
{
    tracker = TrackerCSRT::create(); // CSRT 알고리즘 트래커를 생성 
    Rect2d roi_rect(roi_start, roi_end); // 2차원 
    tracker->init(original_image, roi_rect);
    track_start = false;
    
}
void UpdateTracking()
{
    const int max_point = 100;  // Maximum number of tracked points
    //프레임 간격 +1 로 지정 
    const int prev_num = 6;
    // Draw rectangle around tracked object and get current time
    rectangle(original_image, tracked_rect, Scalar(0, 255, 0), 2);
    ros::Time current_time = ros::Time::now();
    // Calculate current center point
    current_center = Point2f(tracked_rect.x + tracked_rect.width / 2, tracked_rect.y + tracked_rect.height / 2);


    // Create and publish ROS messages for center point and velocity
    geometry_msgs::Point object_point;
    object_point.x = current_center.x;
    object_point.y = current_center.y;
    object_point.z = tracked_rect.width;

    current_point_pub.publish(object_point);

    ROS_INFO("Center: (%.0f, %.0f), Width: %.0f", object_point.x, object_point.y, object_point.z);
}
 void ResetTracking() 
 {  
    // 템플릿 매칭을 위해 원본과 ROI 이미지를 그레이스케일로 변환
    Mat original_gray;
    Mat roi_gray;

    cvtColor(original_image, original_gray, COLOR_BGR2GRAY);
    cvtColor(roi_image, roi_gray, COLOR_BGR2GRAY);
    if (!roi_gray.empty())
    {
        // 템플릿 매칭 수행 (Tutorial에서 가장 정확도가 우수하다고 알려진 정규화된 상관계수 이용)
        
        Mat matching_result;
        matchTemplate(original_gray, roi_gray, matching_result, TM_CCOEFF_NORMED);

        // 최적의 템플릿 매칭 위치 찾기
        double min_value, max_value;
        Point minLoc, maxLoc;
        minMaxLoc(matching_result, &min_value, &max_value, &minLoc, &maxLoc);
  
        // 일치 신뢰도가 특정 임계값을 초과하면 찾은거로 간주
        if (max_value > matching_threshold)
        {   
            //최대 신뢰 값을 가지는 영역에 대해 위치를 대입 
            roi_start = Point(maxLoc.x, maxLoc.y);
            roi_end = Point(maxLoc.x + roi_gray.cols, maxLoc.y + roi_gray.rows);
            rectangle(original_image, Rect(roi_start, roi_end), Scalar(255, 0, 139), 2);
            matching_threshold = max_value; //임계 값 대입 
            ROS_INFO("matching_max_value : %.2f", matching_threshold); //값을 확인하기 위해 터미널에 출력
            matching_time = ros::Time::now();
            StartTracking();
        }

        //해당 좌표를 토대로 Tracker 알고리즘 재실행           
        if ((ros::Time::now() - matching_time).toSec() < 3.0)
        {
            putText(original_image, "Template Matching...", Point(50,50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,139), 2);
            
        }
        else if ((ros::Time::now() - matching_time).toSec() < 4.0)
        {                           
            putText(original_image, "Finish Template Matching..", Point(50,50), FONT_HERSHEY_SIMPLEX, 1, Scalar(139,0,255), 2);

        }
        else
        {      
        track_reset = false;
        matching_threshold = 0.4; //임계값 변수 초기화 
        }             
    }

 }

void ROICallback(const sensor_msgs::ImageConstPtr& msg)
{
    roi_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
     if (!roi_image.empty())
    {   
  
    }
}

void StartCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    roi_start = Point(msg->x, msg->y);
    roi_ready = true;
    track_start = true;
}



void EndCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    roi_end = Point(msg->x, msg->y);
    roi_ready = true;
    track_start = true; // ROI 변경 
}

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Tracker_Node");
    CSR_Tracker roi_display;
    ros::Rate loop_rate(30);
    ROS_INFO("Starting..");
    
    while (ros::ok()) 
    {
        // 'Esc' 키를 누르면 프로그램 종료
        int key = waitKey(1);

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