#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>  

#include "kcftracker.hpp"
#include "KCF/BoundingBox.h"
#include "std_msgs/Bool.h"

using namespace std;
using namespace cv;

Mat frame;

VideoWriter writer("/home/zhao/Videos/video_KCF_01.avi", CV_FOURCC('M', 'J', 'P', 'G'), 20, Size(640, 480)); 
 
//received image callback
void imageCallback(const sensor_msgs::ImageConstPtr& cam_image){
cv_bridge::CvImagePtr cv_ptr;
try
{
     cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
}

catch (cv_bridge::Exception& e)
{
     ROS_ERROR("cv_bridge exception:%s",e.what());
     return;
}

     frame= cv_ptr->image;
     imshow("tracker",frame);
     cvWaitKey(1);
     writer << frame; 
}


int main(int argc, char **argv)  
{  
     // Published ROI 
     KCF::BoundingBox box;
     // Published image stop flag
     std_msgs::Bool stopflag ;
      // ROI selector
     BoxExtractor ROIbox;

     bool Stopflag  = false;

     ros::init(argc, argv, "KCF_ground_node");  
     ros::NodeHandle nh;    

     image_transport::ImageTransport it(nh);  
     image_transport::Subscriber sub = it.subscribe("KCF/image", 1, imageCallback); 
      
     ros::Publisher stopflag_pub = nh.advertise<std_msgs::Bool>("KCF/stopflag", 1000);
     ros::Publisher boundingbox_pub  = nh.advertise<KCF::BoundingBox>("KCF/boundingbox", 1000);

     ros::Rate loop_rate(50);  
     while (nh.ok()) 
     { 
           if ( waitKey(1) == 'b' )
           {
                 Stopflag = true; 
                 stopflag.data = Stopflag;
                 stopflag_pub.publish( stopflag );
           }

           if ( Stopflag )
           {
                 //select ROI
                 Rect roi = ROIbox.extract("tracker", frame);

                 //quit if ROI was not selected
                 if (roi.width == 0 || roi.height == 0)
                       break;

                //publish ROI 
                 box.x = roi.x;
                 box.y = roi.y;
                 box.width = roi.width;
                 box.height = roi.height;
                 boundingbox_pub.publish( box );

                 Stopflag = false;
           }

           if (waitKey(1) == 27) 
                 return 0;

           ros::spinOnce();  
           loop_rate.sleep(); 
    }
} 
