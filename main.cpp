 #include <iostream> // include all necessary libraries
 #include <string>
 #include <list> 
 #include <fstream>
 #include <vector>
 #include<cmath>

 #include <bits/stdc++.h> 
 #include <visp3/core/vpCameraParameters.h>
 #include <visp3/core/vpCameraParameters.h>
 #include <visp3/gui/vpDisplayGDI.h>
 #include <visp3/gui/vpDisplayX.h>
 #include <visp3/io/vpImageIo.h>
 #include <visp3/sensor/vpRealSense2.h>
 #include <visp3/robot/vpRobotAfma6.h> 
 #include <visp3/detection/vpDetectorAprilTag.h>
 #include <visp3/visual_features/vpFeatureBuilder.h>
 #include <visp3/visual_features/vpFeaturePoint.h>
 #include <visp3/vs/vpServo.h>
 #include <visp3/vs/vpServoDisplay.h>
 #include <visp3/gui/vpPlot.h>
 

 #include <visp3/core/vpXmlParserCamera.h>
 #include "opencv2/imgcodecs.hpp"
 #include "opencv2/highgui.hpp"
 #include "opencv2/imgproc.hpp"
 #include "opencv2/imgcodecs.hpp"
 #include "opencv2/highgui.hpp"
#if VISP_HAVE_OPENCV_VERSION >= 0x040000
 #include <opencv2/core/core.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/calib3d/calib3d.hpp>
#elif VISP_HAVE_OPENCV_VERSION >= 0x020300
 #include <opencv2/core/core.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
#endif

 using namespace std;
 using namespace cv;
 
// class for reading intrinsic and extrinsic parameters
class Parameters{
  public:

  vpCameraParameters cam;
  vpXmlParserCamera p;
  vpPoseVector ePc;
  string x,y;

  Parameters (string x1,string y1){      
  x=x1;
  y=y1;
  }
     
  vpPoseVector  get_extrinsics(){      
  std::string opt_eMc_filename = x ;
  if (!opt_eMc_filename.empty()) {      
  ePc.loadYAML(opt_eMc_filename, ePc);
  }
  
  else{ 
  std::cout << "Warning, opt_eMc_filename is empty! Use hard coded values." << "\n";
  }

  std::cout << "The extrinsic parameters are: " << endl;
  for (int i=0;i<6;i++){
  std::cout<< ePc[i]<< endl;
  }  
  return ePc;  
  }  

  tuple<double,double,double,double,double,double> get_intrinsics(){    
  vpCameraParameters::vpCameraParametersProjType projModel; // Projection model
  // Use a perspective projection model without distortion
  projModel = vpCameraParameters::perspectiveProjWithDistortion;

  if (p.parse(cam,y, "Camera", projModel,640,480) != vpXmlParserCamera::SEQUENCE_OK){       
  std::cout << "Cannot found Camera" << std::endl;
  } 
          
  cam.printParameters(); // cout the parameters
  double px = cam.get_px(); // Get the camera parameters for the model without distortion
  double py = cam.get_py();
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  double kud=cam.get_kud();
  double kdu=cam.get_kdu();

  return {px,py,u0,v0,kud,kdu};
  }
 };
 

// class for hough circle detection
class Detect{
            
public:

    int x,y;
    bool debug_flag;
    std::list<Point> center_list;
        
    Detect ( int x1, int y1, bool z1){
    x=x1;
    y=y1;
    debug_flag=z1;    
    }

    cv::Mat detection(cv::Mat fname){   
    if (debug_flag==true){        
    // Loads an image
    Mat src = fname;
    
    // Check if image is loaded fine
    if(src.empty()){           
    printf(" Error opening image\n");
    printf(" Program Arguments: [image_name -- default %s] \n", fname);
    }
    
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    vector<Vec3f> circles;
    HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, x, y // change the last two parameters (min_radius & max_radius) to detect larger circles
                );             
                
    
    for( size_t i = 0; i < circles.size(); i++ ){                   
    Vec3i c = circles[i];
    Point center = Point(c[0], c[1]);
    // circle center
    circle( src, center, 1, Scalar(0,100,100), 3, LINE_AA);
    // std::cout << center << endl;
    // std::cout<<"center is of type: "<<typeid(center).name()<<endl;
    center_list.push_back (center);
    // circle outline
    int radius = c[2];
    circle( src, center, radius, Scalar(255,0,255), 3, LINE_AA);
    }
                
                
    int size = center_list.size();
    cout << "The list contains " << size << " elements"<< endl;
   
    bool check = imwrite("Detected circles.jpg", src);

    if (check == false){ 
                    
    std::cout << "Mission - Saving the image, FAILED" << endl;   
    }          
    std::cout << "Successfully saved the image. " << endl;
    imshow("detected circles", src);
    waitKey();
    return src;
    }
    
    else{
    std::cout<<"Flag is off, turn it on to start detecting"<<endl;
    }

    }
    
    std::list<Point> get_centers()
    {
        return center_list;
    }
    
    
}; 
 
  
// To display the trajectory, this function is not currently used , since the code is not directly connected to the joints of the robot 
void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                               std::vector<vpImagePoint> *traj_vip)
 {
   for (size_t i = 0; i < vip.size(); i++) {
     if (traj_vip[i].size()) {
       // Add the point only if distance with the previous > 1 pixel
       if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.) {
         traj_vip[i].push_back(vip[i]);
       }
     }
     else {
       traj_vip[i].push_back(vip[i]);
     }
   }
   for (size_t i = 0; i < vip.size(); i++) {
     for (size_t j = 1; j < traj_vip[i].size(); j++) {
       vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
     }
   }
 }


// Function to print list datatype
void print(std::list<Point> const &list){

    for (auto const& i: list){ 
    std::cout << i << "\n";
    }

}


 
 
int main(int argc, char **argv)
 {
     
           // read extrinsics and intrinsics
           vpPoseVector ext;
           std::fstream myfile1("./location.txt", std::ios_base::in);
           string fname1,fname2;
           myfile1 >> fname1 >> fname2 ;
           std::cout<< fname1 << endl;
           std::cout<< fname2 <<endl;     
           Parameters parameters(fname1,fname2);
           ext=parameters.get_extrinsics();
           auto [Px,Py,U0,V0,Kud,Kdu]= parameters.get_intrinsics();
           // std::cout<<""<< endl;
           // std::cout<< Px << endl;    
           vpHomogeneousMatrix eMc(ext);
           std::cout<< "The homogenous  matrix is :" << endl;
           std::cout << "eMc:\n" << eMc << "\n";
           vpCameraParameters cam(Px, Py, U0, V0,Kud,Kdu);
           std::cout << "cam:\n" << cam << "\n";
           cv::Mat K = (cv::Mat_<double>(3, 3) << cam.get_px(), 0, cam.get_u0(), 0, cam.get_py(), cam.get_v0(), 0, 0, 1);
           cv::Mat D = (cv::Mat_<double>(4, 1) << cam.get_kud(), 0, 0, 0);
           
           //vpRobotAfma6 robot;
           // Ideal position where the centroid shold be 
           double distance;
           double xl1=327 * 0.000065;
           double yl1=236 * 0.000065;
           //double xl2=0;
           //double yl2=0;

        /****************************************************************************
         // YOUR CODE HERE to initialize camera
         // YOUR CODE Here to init video feed and check camera opened successfully
          i.e.:
            VideoCapture cap(0); 
            if(!cap.isOpened()){
                cout << "Error opening video stream" << endl;
                return -1;
                }
        *****************************************************************************/
           //VideoCapture cap(0); 
           VideoCapture cap("3.avi"); 
           if(!cap.isOpened()){
                cout << "Error opening video stream" << endl;
                return -1;
            }
       /****************************************************************************           
        // YOUR CODE HERE TO write the video file
           
           int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
           int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
           VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));
 
       *****************************************************************************/
           int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
           int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
           VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));
           
          // Read the min, max radius and threshold value
           std::fstream myfile ("./config.txt", std::ios_base::in);
           float min, max, threshold;
           
           myfile >> min >> max>> threshold ;
           threshold= threshold/100;
           std::cout<< min << endl;
           std::cout<<max<<endl;
           Mat frame;
           float min_dist;



           do{
            /****************************************************************************
            // YOUR CODE HERE to grab camera image
            // YOUR CODE HERE to convert the img into an opencv matrix
            // the camera frame must be provided here.
            i.e.:
            frame = camerafeed.convert_to_cv_mat
            *****************************************************************************/
            cap >> frame;

           float frame_width= frame.size().width * 0.000065;
           float frame_height=frame.size().height * 0.000065;

            

            float min1=((( threshold * frame_height)*( threshold * frame_height)) + ((threshold * frame_width)*(threshold * frame_width)));
            min_dist=sqrt(min1);
            cout<<"Minimum Distance"<<endl;
            cout<< min_dist<< endl;
            // If the frame is empty, break immediately
            if (frame.empty()){
                break;
            }               

            std::list<Point> centers;
            cv::Mat imageUndistorted;
            cv::undistort(frame, imageUndistorted, K,D); // Undistort image
            cv::Mat processedImage;
 
            std::cout<< "RUN HOUGH CIRCLE DETECTION" <<endl;
            Detect Detect(min,max,true); // Run hough circle detection on the frame (image)
            processedImage=Detect.detection(imageUndistorted);
            
            std::cout<< "ready to get centroid!!!" <<endl;
            centers=Detect.get_centers(); // Get centroids
            std::vector<Point> center_vector(centers.begin(), centers.end());
            print(centers);

            //vpImage<vpRGBa> renderedImage;
            //vpImageConvert::convert(processedImage, renderedImage);
            //vpImageIo::write(renderedImage, "detected-circles.jpg");       // save the image with the circles drawn on it      

            int size = centers.size();
            
            std::cout<<size<<endl;
            
            std::vector<vpPoint> point(size);       //Add those centroids as features
            for(int i=0;i<size;i++){
                auto x1=center_vector[i].x * 0.000065;
                std::cout<<x1<<endl;
                auto y1=center_vector[i].y * 0.000065;
                std::cout<<y1<<endl;
                point[i].setWorldCoordinates(x1,y1,0);
                //point.push_back(vpPoint(x1, y1, 0));
            } 

            auto xl2=center_vector[0].x * 0.000065;
            auto yl2=center_vector[0].y * 0.000065;

            distance = sqrt(pow(xl2 - xl1, 2) + pow(yl2 - yl1, 2) * 1.0);
            printf("the distance is : %f \r\n", distance);
            // Servo
            //vpHomogeneousMatrix cdMc, cMo, oMo;

            // Desired pose used to compute the desired features
            vpHomogeneousMatrix cdMo (0,0,0.068,0,0,0); 
            //vpHomogeneousMatrix cdMo (0,0,0.123,0,0,0);
            //vpHomogeneousMatrix cdMo;
            //cdMo[0][0] = 0.2230    ;cdMo[0][1] = 0.8881   ; cdMo[0][2] = -0.4020   ; cdMo[0][3] = -0.0004514;
            //cdMo[1][0] =   -0.3032   ; cdMo[1][1] = -0.3288   ; cdMo[1][2] =  -0.8944    ; cdMo[1][3] = 0.0000206;
            //cdMo[2][0] =-0.9265     ; cdMo[2][1] = 0.3213    ; cdMo[2][2] =   0.1959    ; cdMo[2][3] = 0.0004860;
            // Initial pose of the camera
            //vpHomogeneousMatrix cMo(-0.137, 0.3, -0.3, vpMath::rad(7.3), vpMath::rad(-1.07), vpMath::rad(58.4));

            
            vpServo task;
            task.setServo(vpServo::EYEINHAND_CAMERA);
            task.setInteractionMatrixType(vpServo::CURRENT);
            task.setLambda(0.5);
            vpFeaturePoint p[size], pd[size];
            for (unsigned int i = 0; i < size; i++) {
                point[i].track(cdMo);
                vpFeatureBuilder::create(pd[i], point[i]);
                //point[i].track(cMo);
                //vpFeatureBuilder::create(p[i], point[i]); 
                task.addFeature(p[i], pd[i]);
            }
            vpColVector v(6);
            
            std::cout<<"compute velocity in camera frame!"<<endl;
            v = task.computeControlLaw();     // Compute velocity in camera frame     
            cout<< v <<endl;
            vpHomogeneousMatrix command;
            // vpColVector cmd(6);

            vpHomogeneousMatrix velocity(v[0],v[1],v[2],v[3],v[4],v[5]);

            //command= velocity * eMc.inverse();
            command= eMc * velocity;
            // command=eMc * velocity;
            cout<< "Print Command Matrix"<< endl;
            cout<<command<<endl;
            cout<<"-------------"<<endl;
            
            vpPoseVector cmd(command);
            cout<<"Print command velocity"<< endl;
            cout<<cmd<<endl;
            cout<<"--------------"<<endl;

           /************************************** 
           if (distance<min_dist)
           {
            
            // YOUR CODE HERE TO CLOSE CAMERA
            // Your code here to save the video (if any)
            
             printf("find the hole!!\r\n");
             cap.release();
             video.release();
            
            break;
           }
           ***************************************/

            /**************************************
            // YOUR CODE HERE to control the screw machine
            i.e.,:
              x = cmd[0] // m/s
              y = cmd[1] // m/s
              z = cmd [2] // m/s
             ***************************************/ 
              printf("x: %f \r\n", cmd[0]);
              printf("y: %f \r\n", cmd[1]);
              printf("z: %f \r\n", cmd[2]);
              
   
           }while( distance > min_dist);

          cap.release();
          video.release();

     
 }
