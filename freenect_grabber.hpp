#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <libfreenect/libfreenect.hpp>

#include <stdio.h>
#include <iostream>
#include "omp.h"


class Mutex {
public:
        Mutex() {
                pthread_mutex_init( &m_mutex, NULL );
        }
        void lock() {
                pthread_mutex_lock( &m_mutex );
        }
        void unlock() {
                pthread_mutex_unlock( &m_mutex );
        }
private:
        pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
public:


        std::vector<uint16_t> m_buffer_depth;
        std::vector<uint8_t> m_buffer_video;
        std::vector<uint16_t> m_gamma;
        Mutex m_rgb_mutex;
        Mutex m_depth_mutex;
        bool m_new_rgb_frame;
        bool m_new_depth_frame;

        uint16_t getDepthBufferSize16() 
        {
            return getDepthBufferSize()/2;
        }

        MyFreenectDevice(freenect_context *_ctx, int _index)
                : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(getDepthBufferSize()),m_buffer_video(getVideoBufferSize()), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false)
        {
    
            for( unsigned int i = 0 ; i < 2048 ; i++) {
                    float v = i/2048.0;
                    v = std::pow(v, 3)* 6;
                    m_gamma[i] = v*6*256;
            }


        }
        // Do not call directly even in child
        void VideoCallback(void* _rgb, uint32_t timestamp) {
                //std::cout << "RGB callback" << std::endl;
                m_rgb_mutex.lock();
                uint8_t* rgb = static_cast<uint8_t*>(_rgb);
                std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
                m_new_rgb_frame = true;
                m_rgb_mutex.unlock();
        };
        // Do not call directly even in child
        void DepthCallback(void* _depth, uint32_t timestamp) {
                //std::cout << "Depth callback" << std::endl;
            /*
                m_depth_mutex.lock();
                uint16_t* depth = static_cast<uint16_t*>(_depth);
                for( unsigned int i = 0 ; i < 640*480 ; i++) {
                        int pval = m_gamma[depth[i]];
                        int lb = pval & 0xff;
                        switch (pval>>8) {
                        case 0:
                                m_buffer_depth[3*i+0] = 255;
                                m_buffer_depth[3*i+1] = 255-lb;
                                m_buffer_depth[3*i+2] = 255-lb;
                                break;
                        case 1:
                                m_buffer_depth[3*i+0] = 255;
                                m_buffer_depth[3*i+1] = lb;
                                m_buffer_depth[3*i+2] = 0;
                                break;
                        case 2:
                                m_buffer_depth[3*i+0] = 255-lb;
                                m_buffer_depth[3*i+1] = 255;
                                m_buffer_depth[3*i+2] = 0;
                                break;
                        case 3:
                                m_buffer_depth[3*i+0] = 0;
                                m_buffer_depth[3*i+1] = 255;
                                m_buffer_depth[3*i+2] = lb;
                                break;
                        case 4:
                                m_buffer_depth[3*i+0] = 0;
                                m_buffer_depth[3*i+1] = 255-lb;
                                m_buffer_depth[3*i+2] = 255;
                                break;
                        case 5:
                                m_buffer_depth[3*i+0] = 0;
                                m_buffer_depth[3*i+1] = 0;
                                m_buffer_depth[3*i+2] = 255-lb;
                                break;
                        default:
                                m_buffer_depth[3*i+0] = 0;
                                m_buffer_depth[3*i+1] = 0;
                                m_buffer_depth[3*i+2] = 0;
                                break;
                        }
                }
                m_new_depth_frame = true;
                m_depth_mutex.unlock();
                */
                m_depth_mutex.lock();
                uint16_t* depth = static_cast<uint16_t*>(_depth);
                // was getVideoBufferSize()
                std::copy(depth, depth+getDepthBufferSize(), m_buffer_depth.begin());
                m_new_depth_frame = true;
                m_depth_mutex.unlock();
        }
        bool getRGB(std::vector<uint8_t> &buffer) {
                m_rgb_mutex.lock();
                if(m_new_rgb_frame) {
                        buffer.swap(m_buffer_video);
                        m_new_rgb_frame = false;
                        m_rgb_mutex.unlock();
                        return true;
                } else {
                        m_rgb_mutex.unlock();
                        return false;
                }
        }
 
        bool getDepth(std::vector<uint16_t> &buffer) {
                m_depth_mutex.lock();
                if(m_new_depth_frame) {
                        buffer.swap(m_buffer_depth);
                        m_new_depth_frame = false;
                        m_depth_mutex.unlock();
                        return true;
                } else {
                        m_depth_mutex.unlock();
                        return false;
                }
        }
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> 
class freenectGrabber
{
public:
    Freenect::Freenect myfreenect;
    MyFreenectDevice* device;
    std::vector<uint16_t> depth_map;
    std::vector<uint8_t> rgb;



    freenectGrabber(){
        depth_map.resize(640*480*4);
        rgb.resize(640*480*4);
        freenect_video_format requested_format = FREENECT_VIDEO_RGB;
        device = &myfreenect.createDevice<MyFreenectDevice>(0);
        device->setDepthFormat(FREENECT_DEPTH_REGISTERED);
        device->setVideoFormat(requested_format);
        device->startDepth();
        device->startVideo();
    }

    ~freenectGrabber(){
        device->stopVideo();
        device->stopDepth();
    }

    typename pcl::PointCloud<PointT>::Ptr 
    get_point_cloud(int distance, bool colored) 
    {

        //get rgb and depth data
        while(!device -> getDepth(depth_map)){}
        while(!device -> getRGB(rgb)){}


        
        int depth_width = 640;
        int depth_height = 480;

        //create the empty Pointcloud
        boost::shared_ptr<pcl::PointCloud<PointT>> cloud (new pcl::PointCloud<PointT>);

        //initialize the PointCloud height and width
        //cloud->height = std::max (image_height, depth_height);
        //cloud->width = std::max (image_width, depth_width);

        //allow infinite values for points coordinates
        cloud->is_dense = false;



        //set camera parameters for kinect
        double focal_x_depth = 585.187492217609;//5.9421434211923247e+02;
        double focal_y_depth = 585.308616340665;//5.9104053696870778e+02;
        double center_x_depth = 322.714077555293;//3.3930780975300314e+02;
        double center_y_depth = 248.626108676666;//2.4273913761751615e+02;

        float bad_point = std::numeric_limits<float>::quiet_NaN ();
        #pragma omp parallel for
        for (unsigned int y = 0; y < depth_height; ++y)
            for ( unsigned int x = 0; x < depth_width; ++x){
                PointT ptout;
                uint16_t dz = depth_map[y*depth_width + x];
                if (abs(dz) < distance){
                    // project
                    Eigen::Vector3d ptd((x - center_x_depth) * dz / focal_x_depth, (y - center_y_depth) * dz/focal_y_depth, dz);
                    // assign output xyz
                    
                        ptout.x = ptd.x()*0.001f;
                        ptout.y = ptd.y()*0.001f;
                        ptout.z = ptd.z()*0.001f;
                    
                    if(colored){
                        uint8_t r = rgb[(y*depth_width + x)*3];
                        uint8_t g = rgb[(y*depth_width + x)*3 + 1];
                        uint8_t b = rgb[(y*depth_width + x)*3 + 2];

                        ptout.rgba = pcl::PointXYZRGB(r, g, b).rgba; //assign color 
                        //ptout.rgba = pcl::PointXYZRGB(0, 0, 0).rgba;

                    } else
                        ptout.rgba = pcl::PointXYZRGB(0, 0, 0).rgba;
                        #pragma omp critical
                        cloud->points.push_back(ptout); //assigns point to cloud   
                } 
                
            }
            /*
        cloud->sensor_origin_.setZero();
        cloud->sensor_orientation_.w () = 0.0;
        cloud->sensor_orientation_.x () = 1.0;
        cloud->sensor_orientation_.y () = 0.0;
        cloud->sensor_orientation_.z () = 0.0;
        */
        cloud->height = 1;
        cloud->width = cloud->points.size();
        return (cloud);
    }
};