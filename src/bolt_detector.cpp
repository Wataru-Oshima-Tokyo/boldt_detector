 #include <ros/ros.h>

 // Include opencv2

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>


 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include "std_msgs/String.h"
 #include "std_srvs/Empty.h"
 #include "std_msgs/Bool.h"
 #include <vector>
 #include <camera_pkg_msgs/Coordinate.h>
 #include "_Detector.h"
 #include <map>

// #include <camera_pkg/Camera_CV.h>
 #define IMG_HEIGHT (640)
 #define IMG_WIDTH (480)
 #define rep(i,a,b) for(int i=a;i<b;i++)
 #define fore(i,a) for(auto &i:a)
 using namespace std;
 using namespace cv;


struct timespec start, stop;
double fstart, fstop;

class CAMERA_CV{
  public:
    Mat src, src_gray, src_hsv, dst, detected_edges, mask;
    Mat depth;
    ros::Publisher pub;
    ros::Subscriber image_sub, depth_sub, mg400_dsth;
    ros::NodeHandle nh;
    ros::ServiceServer detection_start, detection_stop;
    int lowThreshold;
    // int low_c[3] = {17, 123, 121};
    // int high_c[3] ={37, 143, 201};
    int low_c[3] = {0, 0, 0};
    int high_c[3] = {0, 0, 0};
    const int max_c[3] = {179, 255, 255};
    std::string HSV[3] = {"H","S","V"};
    // int _MIN_DH =15, _MIN_DS = 60, _MIN_DV = 60;
    // int _MAX_DH = 15, _MAX_DS = 150, _MAX_DV = 60;
    void CannyThreshold(int, void*);
    void MaskThreshold(int, void*);
    void DrawCircle(int, void*);
    void Coordinate_Publisher(int x, int y);
    void mouseEvent(int event, int x, int y, int flags, void* userdata);
    camera_pkg_msgs::Coordinate MaskThreshold(int, int);
    // Mat getDepth();
    const std::string OPENCV_WINDOW = "Image window";
    virtual void image_callback(const sensor_msgs::ImageConstPtr&);
    virtual void depth_callback(const sensor_msgs::ImageConstPtr&);
    virtual bool detection_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool detection_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    virtual bool mg400_work_callback(const std_msgs::Bool& msg);
    // Topics
    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    const std::string MG400_TOPIC = "mg400/working";
    std::string cmd = "L";
    // const std::string DEPTH_TOPIC = "/camera/depth/color/image_raw";
    const std::string PUBLISH_TOPIC = "/camera_pkg/coordinate";
    const std::string DETECTION_START_SRV = "/bolt_detection/start";
    const std::string DETECTION_STOP_SRV = "/bolt_detection/stop";
    CAMERA_CV();
    ~CAMERA_CV();
    bool getRun(); 
    bool mg400_working = false;
    const int max_lowThreshold = 100;
    const std::string window_name = "Edge Map";
    std::vector<Point2i> positions;
    bool RUN = false;
private:
    
    bool start_call = true;
    bool stop_call = false;
    const int ratio = 3;
    //set the kernel size 3
    const int kernel_size = 3;
};


CAMERA_CV::CAMERA_CV(){
  ros::NodeHandle private_nh("~");
  private_nh.param("image_topic", IMAGE_TOPIC, std::string("/color/image_rect_raw"));
  private_nh.param("depth_topic", DEPTH_TOPIC, std::string("/depth/image_rect_raw"));
  lowThreshold = 6;
};

CAMERA_CV::~CAMERA_CV(){};

bool CAMERA_CV::getRun(){
  return RUN;
}


 bool CAMERA_CV::detection_start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  //  cout << "start calibration" << endl;
   RUN = true;
   return RUN;

 }

 bool CAMERA_CV::detection_stop_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  //  cout << "stop calibration" << endl;
   RUN = false;
   return RUN;
 }

void CAMERA_CV::Coordinate_Publisher(int x, int y){
    camera_pkg_msgs::Coordinate coordinate;
    //  Mat* _depth = &depth;
     std::string temp="L";
     double z=0.0;
     z = depth.at<uint16_t>((uint16_t)y,(uint16_t)x);
    if(!temp.empty()){
       if(z>0 && z <1200){
          coordinate.t = temp;
          coordinate.x = x;
          coordinate.y = y;
          coordinate.z = z;

          pub.publish(coordinate);
       }else{
         cout << "z value is not valid please try again." << endl;
       }

     }
}

bool CAMERA_CV::mg400_work_callback(const std_msgs::Bool& msg){
    mg400_working = msg.data;
}

void CAMERA_CV::depth_callback(const sensor_msgs::ImageConstPtr& msg){
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    // ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    depth = cv_ptr->image;

}

 void CAMERA_CV::image_callback(const sensor_msgs::ImageConstPtr& msg){
    clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    // ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    src = cv_ptr->image;
    dst.create(src.size(), src.type());
    //cvtColor(src, src_gray, COLOR_BGR2GRAY);
    cvtColor(src, src_hsv, COLOR_BGR2HSV);
    
    // namedWindow(window_name, WINDOW_AUTOSIZE );
    // CannyThreshold(0, 0);

 }



void mouseEvent(int event, int x, int y, int flags, void* userdata)
{
     CAMERA_CV *cc = (CAMERA_CV*)userdata;
    //  ros::Publisher* _pub = cc->pub;
    //  _cc.pub = _cc.nh.advertise<std_msgs::String>(_cc.PUBLISH_TOPIC, 1000);
     camera_pkg_msgs::Coordinate coordinate;
    //  Mat* _depth = &depth;
     
    //  cout << color[0] << " "<< color[1] << " " << color[2] <<endl; 
     if  ( event == EVENT_LBUTTONDOWN )
     {
		  cc->cmd = "L";
          cc->RUN =!cc->RUN;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cc->cmd = "R";
          cc->RUN =false;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cc->cmd = "M";
          cc->RUN =false;
     }
     if(cc->cmd != ""){
          coordinate.t = cc->cmd;
          coordinate.x = 0;
          coordinate.y = 0;
          coordinate.z = 0;
          cc->pub.publish(coordinate);
       }
}



int main( int argc, char** argv )
{

   ros::init(argc, argv, "work_with_camera_start");
   CAMERA_CV cc;
   _DETECTOR dtc;
   // Initialize the ROS Node "roscpp_example"
   ros::Rate loop_rate(20);
   
   cc.image_sub = cc.nh.subscribe(cc.IMAGE_TOPIC, 1000, &CAMERA_CV::image_callback, &cc);
   cc.depth_sub = cc.nh.subscribe(cc.DEPTH_TOPIC, 1000, &CAMERA_CV::depth_callback, &cc);
   cc.mg400_dsth = cc.nh.subscribe(cc.MG400_TOPIC,1000, &CAMERA_CV::mg400_work_callback, &cc);
   cc.detection_start = cc.nh.advertiseService(cc.DETECTION_START_SRV, &CAMERA_CV::detection_start_service, &cc);
   cc.detection_stop = cc.nh.advertiseService(cc.DETECTION_STOP_SRV, &CAMERA_CV::detection_stop_service, &cc);
   cc.pub = cc.nh.advertise<camera_pkg_msgs::Coordinate>(cc.PUBLISH_TOPIC, 1000);
   std_srvs::Empty _emp;
   while(ros::ok()){
      if(!cc.src.empty()){
         cc.positions = dtc.detect(std::make_shared<cv::Mat>(cc.src));
            if(cc.getRun()){
                for(auto position: cc.positions){
                    // printf("x: %d, y: %d\n", position.x, position.y);
                    cv::circle(cc.src, cv::Point(position.x,position.y), 4, cv::Scalar(157, 99, 83));
                    cc.Coordinate_Publisher(position.x, position.y);
                }   
            }
        setMouseCallback("src", mouseEvent, &cc);
        clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
        std::string fps= "FPS: " + std::to_string(1/(fstop-fstart));
        std::string mode="";
        std::string cmd_exp="L:start/stop R: xy_calibration M: z_calibration";
        if(cc.cmd =="L"){
            if(cc.RUN)
                mode ="Executing";
            else 
                mode ="Pausing...";
        }else if (cc.cmd == "R"){
            mode ="xy_calibration";
        }else if (cc.cmd == "M"){
            mode ="z_calibration";
        }
        putText(cc.src, //target image
          fps, //text
          Point(10, 30), //top-left position
          FONT_HERSHEY_DUPLEX,
          1.0,
          Scalar(118, 185, 0), //font color
          2);
        putText(cc.src, mode, Point(10, 60), FONT_HERSHEY_DUPLEX,1.0,Scalar(0, 0, 255), 2);
        putText(cc.src, cmd_exp, Point(10, 90), FONT_HERSHEY_DUPLEX, 1.0, Scalar(255,0, 0), 2);
        imshow( "src", cc.src);
        waitKey(3);
      }

      ros::spinOnce();
      loop_rate.sleep();
   }
  destroyAllWindows();
  return 0;
}
