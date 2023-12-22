#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
// 헤더파일 불러오기 

using namespace cv;
using namespace std;

// OpticalFlow를 진행하는 CLASS 생성
class OpticalFlow
{
public:
// OpticalFlow 생성자 선언 
    OpticalFlow() : nh("~")
    {
        sub = nh.subscribe("/RGB_Image", 1, &OpticalFlow::imageCallback, this);
        point_pub = nh.advertise<geometry_msgs::Point>("Point", 1);
        speed_pub = nh.advertise<geometry_msgs::Point>("Speed", 1);

    }
    
// 관련한 Private 멤버 함수 선언
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher point_pub; 
    ros::Publisher speed_pub;
    ros::Time prev_time;

    Mat prev_img; //이전 프레임을 담는 함수 
    vector<Point2f> prev_points;  //이전 프레임의 Point를 담는 벡터
    map<int, float> prev_speeds;
    map<int, float> prev_angles;
    
    //img callback 함수를 선언 
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {   //OpenCV 형식으로 해당 이미지 토픽을 현재 프레임 이미지에 저장 
            Mat current_img = cv_bridge::toCvCopy(msg, "bgr8")->image; 
            //해당 이미지를 Grayscale로 변환해줌 -> Optical Flow의 원활한 구현
            Mat gray_img;
            cvtColor(current_img, gray_img, COLOR_BGR2GRAY);

            //추적중에 있는 포인트 좌표가 존재한다면 
            if (!prev_points.empty())
            {   //OpticalFlow를 갱신하는 함수를 실행
                updateOpticalFlow(current_img, gray_img);
            }
            else
            {   //비어있을 시에는 해당 Optical Flow를 시작함
                startOpticalFlow(gray_img);
            }
            //Optical Flow를 포함한 이미지를 시각화 
            imshow("Optical Flow", current_img);
            waitKey(1);
        }
        //오류 문구 발생 
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("ERROR! Please Check your Code");

        }
    }

    //OpticalFlow를 갱신하는 함수 
    void updateOpticalFlow(const Mat& current_img, const Mat& gray_img)
    {   //알고리즘에 필요한 벡터 선언 
         vector<Point2f> current_points;
         vector<uchar> status;
         vector<float> error; //추적 오차를 저장하는 벡터
        // Lucas-Kanade방식으로 Optical Flow 알고리즘을 수행 
        calcOpticalFlowPyrLK(prev_img, gray_img, prev_points, current_points, status, error);

        int point_num = 0; //Point의 수를 세는 정수형 변수 
        
        //변경되지 않는 변수들을 const int로 선언하고 초기화          
        const int dist_threshold = 10; //검출하기 위한 Point들을 선언 
        const int point_threshold = 5;  //이하의 값에서 재구동을 위해 변수를 선언
        const int hz  = 30; //해당 코드의 주사율을 저장하는 변수 (30hz)

        for (int i = 0; i < prev_points.size(); ++i)
        {
            if (status[i] == 1)
            {   //추적에 성공한다면 1로 표현
                point_num++;
                //이전 프레임과 현재 프레임의 좌표를 저장 
                Point2f prev_pt = prev_points[i];
                Point2f current_pt = current_points[i];
                //이전 프레임에서 현제 프레임으로 이동한다면 orange 색으로 화살표로 표시 
                arrowedLine(current_img, prev_pt, current_pt, Scalar(000, 140, 250), 3);

                // 현재 포인트와 이전 포인트의 유클리드 거리를 계산
                float distance = norm(current_pt - prev_pt);
                
                // 각속도의 계산을 위한 변수들 지정 
                float prev_angle = prev_angles[i];
                float dx = current_pt.x - prev_pt.x; //x의 변화율
                float dy = current_pt.y - prev_pt.y; //y의 변화율                            
                float angle = atan2(dy, dx);  // 두 점 간의 각도 계산
                float dangle = angle - prev_angle;
                prev_angles[i] = angle;

                //속도가 임계 값 이상이라면
                if (distance >= dist_threshold)  
                {  //좌표 메세지를 발행 (현재 좌표와 프레임당 픽셀)
                    geometry_msgs::Point optical_flow_info;
                    optical_flow_info.x = current_pt.x;
                    optical_flow_info.y = current_pt.y;
                    optical_flow_info.z = distance;

                    point_pub.publish(optical_flow_info);
                    //Publish된 메세지를 터미널에 출력 (시각화를 위해 출력 자리수를 제어)
                    ROS_INFO("Position INFO: x = %.0f, y = %.0f , Distance = %.2f ", optical_flow_info.x, optical_flow_info.y, optical_flow_info.z);

                //veloicity에 대한 메세지 값
                geometry_msgs::Point vel_msg;               
                vel_msg.x = dx * hz; //x축에 대한 속도 값 
                vel_msg.y = dy * hz ; //y축에 대한 속도 값 
                vel_msg.z = dangle * hz; //각속도 

                speed_pub.publish(vel_msg);
                //Publish된 메세지를 터미널에 출력 (시각화를 위해 출력 자리수를 제어)
                 ROS_INFO("Velocity INFO: V_x = %.1f (pixel/s), V_y = %.1f (pixel/s), V_ang = %.3f (rad/s), ", vel_msg.x, vel_msg.y, vel_msg.z);
                
                }               
            }
        }
        //임계 값보다 point들이 더 적다면 재구동 
        if (point_num < point_threshold)
        {
            ROS_INFO("Only %d Point(s) detected. Restart optical flow...", point_num);
            startOpticalFlow(gray_img);
        }
        //아닐 경우는 현재 프레임의 이미지를 이전 이미지 프레임에 대입 
            else
        {
            prev_points = current_points;
            prev_img = gray_img.clone();
        }
    }
    
    //Opticalflow를 시작(및 재시작) 함수를 지정 
    void startOpticalFlow(const Mat& gray_img)
    {   //이전 이미지를 현재 이미지로 갱신 
        prev_img = gray_img.clone();

        //특징점을 찾는 함수 구동 (최대 검출, 품질, 포인트간 거리)를 설정
        // shi-tomasi 방식으로 Corner Detection하려면 아래를 주석 해제
        // goodFeaturesToTrack(prev_img, prev_points, 200, 0.3, 7);

        
        // Harris 방식으로 Corner Detection를 하려면 아래를 주석 해제
        goodFeaturesToTrack(prev_img, prev_points, 200, 0.2, 0.5, noArray(), 2, true, 0.04);
    }
};

//함수의 main 함수 
int main(int argc, char** argv)
{   //노드 생성 및 초기화 
    ros::init(argc, argv, "Optical_Flow");
    OpticalFlow optical_flow;

    ROS_INFO("running...SUCCESS!!");
    while (ros::ok())
    {
        ros::spinOnce();

        if (cv::waitKey(1) == 27)
        {
            ROS_INFO("Goodbye!");
            break;
        }
    }

    return 0;
}
