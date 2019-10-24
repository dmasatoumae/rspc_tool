#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <opencv/highgui.h>
#include <string>
#include <thread>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

/*

auto pipeline_connect(auto pipelines,auto ctx)
{
    
    
    for(auto&& dev : ctx.query_devices())
    {
        std::cout<<"pipeline connect start"<<std::endl;
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
    }
    int number_of_rs=pipelines.size();
    std::cout<<pipelines.size()<<" recognize"<<endl;
    
    return pipelines;
}
*/
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();
	int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(
		texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto tex_coords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	for (int i = 0; i < points.size(); ++i)
	{

        if((vertices[i].z)<2.5){
            //std::cout<<vertices[i].z<<std::endl;
		    cloud->points[i].x = vertices[i].x;
		    cloud->points[i].y = vertices[i].y;
            cloud->points[i].z = vertices[i].z;

        }

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);
        if((vertices[i].z)<2.5){
            //std::cout<<vertices[i].z<<std::endl;
            cloud->points[i].r = std::get<0>(current_color);
		    cloud->points[i].g = std::get<1>(current_color);
		    cloud->points[i].b = std::get<2>(current_color);

            

        }

        
	}

	return cloud;
}

//なぜか関数としてフィルタをかけるとRGBが消える
rs2::depth_frame points_filter(rs2::depth_frame depth_frame)
{


    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);
    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
    rs2::temporal_filter temp_filter;
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
    rs2::hole_filling_filter hf_filter;
    hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);

            
    depth_frame = dec_filter.process(depth_frame);
    depth_frame = depth_to_disparity.process(depth_frame);
    depth_frame = spat_filter.process(depth_frame);
    depth_frame = temp_filter.process(depth_frame);
    depth_frame = disparity_to_depth.process(depth_frame);
    depth_frame = hf_filter.process(depth_frame);
            


    return depth_frame;
}

int main(int argc, char* argv[] ) {
    std::cout<<"1"<<std::endl;
    rs2::colorizer color_map;
    int number = 0;
    char key;
    
    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);
    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
    rs2::temporal_filter temp_filter;
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
    rs2::hole_filling_filter hf_filter;
    hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);
    
    std::string file_name = argv[1];
    //std::string output_names = "Outputs";
    //std::stringstream output_names;
    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_device_from_file(file_name.c_str());
    //cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    //rs2::device device = pipe.get_active_profile().get_device();
    //rs2::playback playback = device.as<rs2::playback>();
    rs2::frameset frames;
    rs2::points points;
    rs2::pointcloud pc;
    cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    while(true)
    {
        if(pipe.poll_for_frames(&frames))
        {
            //std::cout<<"4"<<std::endl;

            rs2::frameset frame = pipe.wait_for_frames();
            auto depth_frame = frame.get_depth_frame();
            rs2::frame color_frame = frame.get_color_frame();
            //下のdepth_frameだとバグるUbuntu18.04
            //rs2::frame depth_frame = frame.get_depth_frame().apply_filter(color_map);
            //
            //rs2::frame depth_frame = color_map(frame.get_depth_frame());
            
            depth_frame = dec_filter.process(depth_frame);
            depth_frame = depth_to_disparity.process(depth_frame);
            depth_frame = spat_filter.process(depth_frame);
            depth_frame = temp_filter.process(depth_frame);
            depth_frame = disparity_to_depth.process(depth_frame);
            depth_frame = hf_filter.process(depth_frame);
            
            //depth_frame=points_filter(depth_frame);
            std::stringstream output_names;


            cv::Mat color_mat = cv::Mat(cv::Size(1280, 720), CV_8UC3, const_cast<void *>( color_frame.get_data()));
            cv::Mat ir =cv::Mat(cv::Size(1280, 720), CV_16UC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);
            //std::cout<<"6"<<std::endl;

            //std::cout<<"6.2"<<std::endl;
            points = pc.calculate(depth_frame);
            pc.map_to(color_frame);

            //std::cout<<"6.4"<<std::endl;
            

            cv::cvtColor(color_mat,color_mat,CV_RGB2BGR);
            //points.export_to_ply("test.ply",color_frame);

            ir.convertTo(ir, CV_8U, -255.0/10000.0, 255.0);
            cv::equalizeHist(ir, ir);
            cv::applyColorMap(ir, ir, cv::COLORMAP_JET);
            //std::cout<<"7"<<std::endl;
            cv::imshow("Depth", ir);
            cv::imshow("Color", color_mat);
            output_names<<file_name<<number<<".ply";
            //std::this_thread::sleep_for(std::chrono::seconds(8));
            //sleep(6);
            //points.export_to_ply(output_names.str(),color_frame);

            key = cv::waitKey(1);

            number++;
            
            if (key == 's')
            {
                //output_names = std::to_string(number)+".ply";
                points.export_to_ply("1.ply",color_frame);
                number++;
                std::cout << "save ply_file" << std::endl;
                auto save_point = points_to_pcl(points,color_frame);
                pcl::io::savePLYFileBinary("1pcl.ply",*save_point);
            }
            

            if (key == 'q') break;
        }
    }

    pipe.stop();


    return 0;

}
