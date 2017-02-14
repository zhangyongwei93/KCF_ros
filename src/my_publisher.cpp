#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  

using namespace std;
using namespace cv;

Mat frame;

//char flag = 0;

int main(int argc, char* argv[]) 
{

    ros::init(argc, argv, "KCF_ground_node");  
    ros::NodeHandle nh;  

    image_transport::ImageTransport it(nh);  
    image_transport::Publisher pub = it.advertise("camera/image", 1); 

    string arg=argv[1];
    VideoCapture cap(arg);
    if (!cap.isOpened())
    {          
            cap.open(atoi(arg.c_str()));
            printf("Opening camera\n");  
    }

    ros::Rate loop_rate(50);  
    while (nh.ok()) 
    { 
             cap >> frame;
             imshow("image",frame);
             cvWaitKey(1);

             sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
             pub.publish(msg);

          //   if ( flag == 0 )
         //    {
          //        pub.publish(msg);
	//}

        //    if ( waitKey(1) == 'b' )
         //   	    flag = 1;
      //      if ( waitKey(1) == 13 || waitKey(1) == 32 )
        //    	    flag = 0;

            ros::spinOnce();  
            loop_rate.sleep(); 
    }
}
