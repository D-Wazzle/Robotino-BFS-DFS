  #include <ros/ros.h>
  #include <std_msgs/String.h>
  #include <image_transport/image_transport.h>
  #include <cv_bridge/cv_bridge.h>
  #include <sensor_msgs/image_encodings.h>
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/opencv.hpp>
  #include <std_msgs/MultiArrayLayout.h>
  #include <std_msgs/MultiArrayDimension.h>
  #include <std_msgs/Int32MultiArray.h>


  namespace enc = sensor_msgs::image_encodings;

  static const char WINDOW[] = "Image window";

  class ImageConverter
  {
    
    // declare all of our node handelers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pos_pub_;
    
    
  public:
    //  declare variables to be use later
    IplImage* imgTracking;
    int lastX;
    int lastY;
    ros::NodeHandle n;
    
    
    ImageConverter()
      : it_(nh_)
    {
      image_pub_ = it_.advertise("/image_processed", 1);
      pos_pub_= n.advertise<std_msgs::Int32MultiArray>("/obj_pos", 2);
      image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);

      cv::namedWindow(WINDOW);
    }

    ~ImageConverter()
    {
      cv::destroyWindow(WINDOW);
    }
    
    // same as in linked tutorial
    IplImage* GetThresholdedImage(IplImage* imgHSV){       
      IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
      
      // below we select the HSV values.  Thes are the values for the ball I am using on my robotino's webcam
      cvInRangeS(imgHSV, cvScalar(23,80,50), cvScalar(38,2556,256), imgThresh); 
      return imgThresh;
  }
    // same as in linked tutorial
    void trackObject(IplImage* imgThresh){
      // Calculate the moments of 'imgThresh'
      CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
      cvMoments(imgThresh, moments, 1);
      double moment10 = cvGetSpatialMoment(moments, 1, 0);
      double moment01 = cvGetSpatialMoment(moments, 0, 1);
      double area = cvGetCentralMoment(moments, 0, 0);

      // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
      if(area>100){
	  // calculate the position of the ball
	  int posX = moment10/area;
	  int posY = moment01/area;        
	  
	if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
	  {
	      // Draw a yellow line from the previous point to the current point
	      cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,0,255), 4);
	  }

	  lastX = posX;
	  lastY = posY;
	std::cout << "X:pos" << posX <<std::endl;
	std::cout << "Y:pos" << posY <<std::endl;
	
	// the code belosw is used to send a size 2 array to indicate the position
	// sent as rostopic /obj_pos
	std_msgs::Int32MultiArray array;
	array.data.clear();
	array.data.push_back(posX);
	array.data.push_back(posY);
	pos_pub_.publish(array);
	  
      }

      free(moments); 
  }

    

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      // get the pointer to the open cv image
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }

      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){

	      
	      IplImage* frame=new IplImage(cv_ptr->image);  // in our case the source of the image is different
	      // than that of the tutorial so we and to select frame from our cv_pointer we got from our open cv link
	      
	      imgTracking=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 3);
	      cvZero(imgTracking); //covert the image, 'imgTracking' to black
		    
      
	      frame=cvCloneImage(frame); 
	      
	      cvSmooth(frame, frame, CV_GAUSSIAN,3,3); //smooth the original image using Gaussian kernel

	      IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3); 
	      cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
	      IplImage* imgThresh = GetThresholdedImage(imgHSV);
	    
	      cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN,3,3); //smooth the binary image using Gaussian kernel
	      
	    //track the possition of the ball
	    trackObject(imgThresh);

	      // Add the tracking image and the frame
	    cvAdd(frame, imgTracking, frame);

	    cvShowImage("Ball", imgThresh);           
	    cvShowImage("Video", frame);
	    
	    //Clean up used images
	    cvReleaseImage(&frame);  // we created a pointer so we need to clean it up before we 
				      // exit the scope of the if statement
	    cvReleaseImage(&imgHSV);
	    cvReleaseImage(&imgThresh);            
	    cvReleaseImage(&frame);
	    
	
      }
      cv::imshow(WINDOW, cv_ptr->image);
      cv::waitKey(3);
      
      image_pub_.publish(cv_ptr->toImageMsg());
    }
  };

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "vision");
    ImageConverter ic;
    ros::spin();
    return 0;
  }
