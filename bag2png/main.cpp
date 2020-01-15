#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <opencv/highgui.h>
#include <string>
#include <thread>


int main(int argc, char* argv[] ) {
    rs2::colorizer color_map;
    int number = 0;
    char key;
    std::string file_name = argv[1];
    std::string out_file_name = argv[2];
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device_from_file(file_name.c_str());
    //cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);
    //rs2::device device = pipe.get_active_profile().get_device();
    //rs2::playback playback = device.as<rs2::playback>();
    rs2::frameset frames;
    cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
    while(true)
    {
        if(pipe.poll_for_frames(&frames))
        {
            //std::cout<<"4"<<std::endl;

            rs2::frameset frame = pipe.wait_for_frames();
            rs2::frame color_frame = frame.get_color_frame();
            std::stringstream output_names;


            cv::Mat color_mat = cv::Mat(cv::Size(1280, 720), CV_8UC3, const_cast<void *>( color_frame.get_data()));
            //cv::Mat ir =cv::Mat(cv::Size(1280, 720), CV_16UC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::cvtColor(color_mat,color_mat,CV_RGB2BGR);
            cv::imshow("Color", color_mat);
            //cv::cvtColor(color_mat,color_mat,CV_RGB2BGR);
            key = cv::waitKey(1);

            number++;
            
            if (key == 's')
            {
                number++;
                cv::imwrite(out_file_name,color_mat);
                std::cout << "save ply_file" << std::endl;
                
            }
            

            if (key == 'q') break;
        }
    }

    pipe.stop();


    return 0;

}
