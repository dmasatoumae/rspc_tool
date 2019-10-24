#include<iostream>
#include<librealsense2/rs.hpp>
#include<opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include<string>

int main(int argc,char* argv[])
{
    char key;
    int number=0;
    std::string file_name = argv[1];
    std::string output_name ="Outputs";

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_device_from_file(file_name.c_str());
    pipe.start(cfg);

    rs2::frameset frames;
    rs2::points points;
    rs2::pointcloud pc;
    cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE);
    while(true)
    {
        if(pipe.poll_for_frames(&frames))
        {
            auto frame = pipe.wait_for_frames();
            auto depth_frame = frame.get_depth_frame();
            auto color_frame = frame.get_color_frame();
            
            cv::Mat color_mat = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, const_cast<void *>( color_frame.get_data()));
            cv::Mat ir(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16SC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);


            points = pc.calculate(depth_frame);
            pc.map_to(color_frame);
            points.export_to_ply("test.ply",color_frame);
            cv::cvtColor(color_mat,color_mat,CV_RGB2BGR);
            ir.convertTo(ir, CV_8U, -255.0/10000.0, 255.0);
            cv::equalizeHist(ir, ir);
            //cv::applyColorMap(ir, ir, cv::COLORMAP_JET);

            cv::imshow("Depth", ir);
            cv::imshow("Color", color_mat);
            key = cv::waitKey(0);

            if(key =='s')
            {
                output_name =std::to_string(number)+".ply";
                points.export_to_ply(output_name,color_frame);
                number++;
                std::cout<<"save ply file"<<std::endl;
            }
            if (key == 'q')break;
        }
    }
    pipe.stop();
    return 0;
}

