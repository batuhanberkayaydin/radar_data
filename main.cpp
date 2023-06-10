
#include <iostream>
#include "chrono"

#ifdef __cplusplus
extern "C" {
#endif

#include "decode_ethernet.h"

#ifdef __cplusplus
}
#endif

#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

#include <chrono>
#include <cstdint>
#include <iostream>

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {

    
    std::ofstream cvsWriter;
    cvsWriter.open("./"+ std::to_string(timeSinceEpochMillisec()) +".csv");

    cv::VideoCapture capture(3);
    // capture.set(cv::CAP_PROP_FPS, 30);
    // capture.set(cv::CAP_PROP_FRAME_WIDTH, 1920) ;
    // capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080) ;

    if (!capture.isOpened()) {

        std::cout << "Camera Source Not Found!";
    }
    cv::Mat testmat;
    capture >> testmat;
    // cv::flip(testmat, testmat, 1);
    // testmat = cv::imread("middle-of-road.jpg");

    initRadarData();

    std::vector<std::string> startHourSplitted, endHourSplitted, testHourSplitted;

   
    while (true)
    {

        capture >> testmat;

        getRadarData();
        
        for (size_t i = 0; i < 10; i++)
        {
            int objId = i;
            

            std::string testStr = printSMSObject_V3_1(i);
            boost::split(startHourSplitted, testStr, boost::is_any_of(","));

            float x_coord = stof(startHourSplitted[0]);
            float y_coord = stof(startHourSplitted[1]);
            float z_coord = stof(startHourSplitted[2]);
            std::string speed_out = startHourSplitted[3];
            // std::cout << "Speed : " << speed_out << std::endl;
            std::string heading_out = startHourSplitted[4];
            // std::cout << "Heading : " << heading_out << std::endl;
            std::string length_out = startHourSplitted[5];
            // std::cout << "length : "  << length_out << std::endl;
            // float f_length_out = stof(startHourSplitted[4]);
            // float f_quality_out = stof(startHourSplitted[5]);
            // float f_speedAbs_out = stoi(startHourSplitted[5]);
            //  float f_speedAbs_out = 0;
            //  float f_quality_out = 0;
            // float f_accel_out = stof(startHourSplitted[7]);
            


            if (x_coord && y_coord && z_coord != 0)
            { 

                cvsWriter << timeSinceEpochMillisec() << objId << ", " << x_coord << ", " << y_coord << ", " << z_coord << ", "<< speed_out << ", "<< heading_out << ", "<< length_out << ", "  << "\n";
                // std::cout << "Test2 : " <<  std::endl;

            }
            
        }
        cv::imwrite("./1/" + std::to_string(timeSinceEpochMillisec())+ ".jpeg", testmat );

        // video.write(testmat);
        cv::imshow("test", testmat);
        // Press  ESC on keyboard to exit
        char c=(char)cv::waitKey(25);
        if(c==27)
        break;

    }
    cvsWriter.close();
    // video.release(); 
    capture.release();
    

}
