#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>

#include "kcftracker.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>    
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "KCF/BoundingBox.h"
#include "std_msgs/Bool.h"

using namespace std;
using namespace cv;

Mat frame;
Rect roi;
bool stopflag = false;
bool init_flag = false;
bool begintrack_flag = false;

// ROI callback
void BoxMessageReceived(const KCF::BoundingBox& KCFbox)
{
    roi.x = KCFbox.x;
    roi.y = KCFbox.y;
    roi.width = KCFbox.width;
    roi.height = KCFbox.height;

    init_flag = true;
}

// StopFlag callback
void StopMessageReceived(const std_msgs::Bool& flag)
{
    stopflag  = flag.data;
}


int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "KCF_UAV_node");  
    ros::NodeHandle nh;   

    image_transport::ImageTransport it(nh);  
    image_transport::Publisher image_pub = it.advertise( "KCF/image", 1); 

    ros::Subscriber stopflag_sub = nh.subscribe ( "KCF/stopflag" ,1000, &StopMessageReceived ) ;
    ros::Subscriber boundingbox_sub = nh.subscribe ( "KCF/boundingbox" ,1000, &BoxMessageReceived ) ;
    
    // Open Camera 
    string arg=argv[1];
    VideoCapture cap(arg);
    if (!cap.isOpened())
    {          
          cap.open(atoi(arg.c_str()));
          printf("Opening Camera\n");  
    }

    bool HOG = true;
    bool FIXEDWINDOW = true;
    bool MULTISCALE = true;
    bool SILENT = true;
    bool LAB = true;

     // Create KCFTracker object
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
     // Tracker results
    Rect result;

    ros::Rate loop_rate(50);  
    while (nh.ok()) 
    { 
           if ( !stopflag )
           {
                 cap >> frame;
                 if ( !begintrack_flag )
                 {
                       //imshow("image",frame);
                       cvWaitKey(1);
                 }
           }

           if ( init_flag )
           {
  	             // initialize the tracker
      	      tracker.init((Rect)roi, frame);

      	      printf("Start the tracking process, press ESC to quit.\n");
                 stopflag = false;
                 init_flag = false;
      	      begintrack_flag = true;
           }

           if ( begintrack_flag )
           { 
                  double t = (double)cvGetTickCount();

                  // stop the program if no more images
            	if (frame.rows == 0 || frame.cols == 0)
            	        break;

      	       // update the tracking result
      	       result = tracker.update(frame);

      	       // draw the tracked object
      	       rectangle(frame, result, Scalar(255, 0, 0), 2, 1);

      	       // show image with the tracked object
      	      // imshow("image", frame);

      	       //Í³¼Æ´¦Àí1Ö¡Ê±¼ä
      	       t = (double)cvGetTickCount() - t;
                  cout << "cost time: " << t / ((double)cvGetTickFrequency()*1000.) << endl;

      	       //quit on ESC button
      	       if (waitKey(1) == 27) 
      	              return 0;
       }

         //publish image 
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
        image_pub.publish(msg);

        ros::spinOnce();  
        loop_rate.sleep(); 
    }
}


