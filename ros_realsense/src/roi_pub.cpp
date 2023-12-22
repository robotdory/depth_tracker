    #include <ros/ros.h>
    #include <sensor_msgs/Image.h>
    #include <cv_bridge/cv_bridge.h>
    #include <opencv2/highgui/highgui.hpp>
    #include <geometry_msgs/Point.h>

    //헤더파일 불러오기 

    using namespace cv;
    using namespace std;

    // ROIDetection에 대한 CLASS 생성
    class ROIDetection
    {
    public:

        ROIDetection()
            : selecting_roi(false), roi_show(false), roi_ready(false), roi_point(false)
        {
            // RGB 이미지를 구독한하는 Subscriber 생성
            image_sub = nh.subscribe("RGB_Image", 1, &ROIDetection::imageCallback, this);
            
            // 원본 영상과 ROI 영역을 발행할 Publisher 생성
            original_pub = nh.advertise<sensor_msgs::Image>("ROI_Detection/Original_Image", 1);
            roi_pub = nh.advertise<sensor_msgs::Image>("ROI_Detection/ROI_Image", 1);
            roi_start_pub = nh.advertise<geometry_msgs::Point>("ROI_Detection/roi_Start", 1);
            roi_end_pub = nh.advertise<geometry_msgs::Point>("ROI_Detection/roi_End", 1);
        
            // 마우스 이벤트 처리를 위한 콜백 함수 설정
            namedWindow("Original Image");
            setMouseCallback("Original Image", ROIDetection::onMouse, this);
        }

    private:
        // 마우스 처리를 static 함수 on mouse 선언
        static void onMouse(int event, int x, int y, int flags, void* param);

        // 이미지 콜백 함수 선언
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        // ROI 영역의 시작점과 끝점을 저장할 Point 선언
        Point roi_start, roi_end;

        // ROI 영역 선택에 필요한 bool 변수 선언
        bool selecting_roi;
        bool roi_show; //ROI 영역을 표시할 사각형 표시에 대한 변수
        bool roi_ready; //ROI 영역 준비 및 발행에 대한 변수
        bool roi_point;
        bool roi_error = false;


        //img 토픽을 구독하고 발행하는 변수 선언
        ros::NodeHandle nh;
        ros::Subscriber image_sub;

        ros::Publisher roi_pub;
        ros::Publisher original_pub;
        ros::Publisher roi_start_pub;  
        ros::Publisher roi_end_pub;   


        // 현재 처리 중인 원본 이미지를 담은 OpenCV MAT 객체
        Mat original_image;
        Mat pub_only_image;
    };

    // ROIDetection 클래스의 생성자


    // 마우스 이벤트 처리 콜백 함수
    void ROIDetection::onMouse(int event, int x, int y, int flags, void* param)
    {
        ROIDetection* self = static_cast<ROIDetection*>(param);
        // 왼쪽 마우스 버튼이 눌렸을 때, ROI 영역의 시작점 설정
        
        //마우스의 포인터가 영역 밖으로 벗어났을 시 클릭을 할 수 없도록 설정 
        //각각의 윈도우 밖으로 나가는 4가지 경우를 or 연산    
        if (x < 0 || x >= self->original_image.cols || y < 0 || y >= self->original_image.rows)
            {
            ROS_INFO("Endpoint is out of Window. Please Try again. ");

            return;
            
            }

                if (event == EVENT_LBUTTONDOWN) {
                    self->roi_error = false;
                    self->roi_start = Point(x, y);
                    self->selecting_roi = true;
                    self->roi_show = false; //사각형 표시 변수 
                    self->roi_ready = false; //ROI 발행 변수
                    self->roi_point = false;
                }
                
                else if (event == EVENT_MOUSEMOVE) {
                    // 마우스가 움직이는 동안, ROI 영역 크기 설정
                    if (self->selecting_roi) {
                        self->roi_end = Point(x, y);
                        self->roi_show = true; //사각형 표시 
                        self->roi_point = false;
                    }
                }

                else if (event == EVENT_LBUTTONUP) {
                    self->roi_end = Point(x, y);
                    self->selecting_roi = false;

                    if (self->roi_start == self->roi_end)
                    {
                        //start점과 end점이 같다면 해당 변수들을 초기화
                        
                        self->roi_error = true;

                        ROS_INFO("ERROR! Please Select ROI again.");  
                    } 
                    
                    else {
                    self->roi_error = false;
                    self->roi_show = true;
                    self->roi_ready = true; //roi 발행 준비 완료 
                    self->roi_point = true;
                    }   
                }
    }

    // 이미지 콜백 함수
    void ROIDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {   
        if(!roi_error)
        {
            // 수신된 이미지를 OpenCV 형식의 bgr 이미지로 변환 
            original_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            //흰색 영역이 표시되지 않게 Publish하기 위해 2개를 저장 
            pub_only_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // roi_show가 true라면 
            if (roi_show) 
            {
                // ROI 영역을 사각형으로 표시하고 이를 추출 
                Rect roi_rect(roi_start, roi_end);
                Mat roi_image = original_image(roi_rect).clone();

                // ROI 영역을 두꺼운 흰색 사각형으로 표시
                rectangle(original_image, roi_rect, Scalar(255, 255, 255), 2);
           
                if (roi_ready) 
                { // roi_ready가 참일 때
                    // ROI_Image 창을 띄우고 해당 이미지를 보여줌 
                    namedWindow("ROI Image");
                    imshow("ROI Image", roi_image);
                    
                    // 영역이 선택되면 roi 영역의 이미지를 Publish 함 
                    sensor_msgs::ImagePtr roi_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi_image).toImageMsg();
                    roi_pub.publish(roi_img_msg);
                    
                    // ROI 시작점을 발행
                    geometry_msgs::Point roi_start_msg;
                    roi_start_msg.x = roi_start.x;
                    roi_start_msg.y = roi_start.y;
                    roi_start_msg.z = 0;  
                    roi_start_pub.publish(roi_start_msg);
                
                    
                    // ROI 끝점을 발행
                    geometry_msgs::Point roi_end_msg;
                    roi_end_msg.x = roi_end.x;
                    roi_end_msg.y = roi_end.y;
                    roi_end_msg.z = 0;  
                    roi_end_pub.publish(roi_end_msg);
                    
                    if (roi_point) 
                        { // 처음 roi_point가 true가 될 때만 출력

                        ROS_INFO("ROI Start Point: (%d, %d)", roi_start.x, roi_start.y);
                        ROS_INFO("ROI End Point: (%d, %d)", roi_end.x, roi_end.y); 
                        roi_ready = false; 
                        // roi 발행 준비 완료 
                    }
            
        
                }
                  
            }
        }
        else
        { 
          return; 
        }
        
        
        // bool 함수의 true, false 여부와 상관없이 원본 이미지에 대해서는 송출 및 출력
        sensor_msgs::ImagePtr original_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_only_image).toImageMsg();
        original_pub.publish(original_img_msg);
        imshow("Original Image", original_image);

    }

   int main(int argc, char** argv)
{
    // ROS 노드 초기화 및 이름 지정 
    ros::init(argc, argv, "ROI_publisher");
    ROIDetection roi_detection;

    // 정보 출력 및 메인 루프 시작
    ROS_INFO ("Publish Success!!");
    ROS_INFO ("Select ROI please.");

    // FPS 측정을 위한 변수 및 타이머 설정
    int frame_count = 0;
    double start_time = ros::Time::now().toSec();

    // 30hz 단위로 루프를 실행하여 ROS 이벤트 처리
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

        // FPS 계산 및 출력
        frame_count++;
        double current_time = ros::Time::now().toSec();
        double elapsed_time = current_time - start_time;
        if (elapsed_time >= 1.0)  // Print FPS every second츠
        {
            double fps = frame_count / elapsed_time;
            ROS_INFO("FPS: %.2f", fps);
            start_time = current_time;
            frame_count = 0;
        }
    }
    ROS_INFO("Shutting down the ROI Publisher..");

    // 창 닫기
    return 0;
}
