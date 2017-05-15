/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 *  Author: Marsette Vona
 *  Author: Dimitrios Kanoulas (dkanoulas@gmail.com)
 */

#ifndef MVKINFU_KINFU_APP_ROS_H
#define MVKINFU_KINFU_APP_ROS_H

#include "cuda/containers.h"
#include <rxkinfu/kinfu_tracker.h>
#include <rxkinfu/tsdf_volume_host.h>
#include <rxkinfu/image_view.h>
#include <rxkinfu/cloud_view.h>

#ifdef HAVE_IMUCAM
#include <imucam/imucam.h>
#endif

// ROS headers
#include <ros/ros.h>
#include <ros/node_handle.h>

// SENSOR_MSGS headers
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>

// PCL headers
#include <pcl/io/openni2_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

namespace rxkinfu
{
  /** \brief TBD
    *
    * \author Dimitrios Kanoulas
    */
  class KinfuAppRos
  {
    public:
      /** \brief Contructor that runs setup() with the same input params.
        *
        * \param[in] volume_size_meters the size of the volume in meters
        * \param[in] en_icp whether ICP is enabled
        * \param[in] en_vizualization whether vizualization is enabled
        * \param[in] show_current_cloud whether to show the current point cloud
        * \param[in] show_raycast whether to show the raycast image
        * \param[in] show_depth whether to show the depth image
        * \param[in] truncation_scaling TBD
        * \param[in] eval_mode TBD
        * \param[in] live TBD
        * \param[in] trigger TBD
        * \param[in] capture TBD
        */
      KinfuAppRos (const Eigen::Vector3f &volume_size_meters,
                   bool en_icp, bool en_vizualization,
                   bool show_current_cloud, bool show_raycast, bool show_depth,
                   bool truncation_scaling, bool eval_mode,
                   bool live, bool trigger,
                   boost::shared_ptr<pcl::Grabber> capture);

      
      /** \brief Contructor that runs setup() with the same input params.
        *
        * \param[in] argc the number of input parameters
        * \param[in] argv the input parameters
        * \param[in] live TBD
        * \param[in] trigger TBD
        * \param[in] capture TBD
        */
      KinfuAppRos (int argc, char **argv, bool live, bool trigger,
                   boost::shared_ptr<pcl::Grabber> capture);

      
      /** \brief Default contructor that runs setup() with the same input params.
        *
        * \param[in] argc the number of input parameters
        * \param[in] argv the input parameters
        * \param[in] node the ROS rxkinfu node created in the main function
        */
      KinfuAppRos (int argc, char **argv, ros::NodeHandle node);
      
      
      /** \brief Destructor. */
      virtual ~KinfuAppRos() {}

      /** \brief Print the KinFu help options. */
      virtual void
      printHelp();
      
      /** \brief Print the rxKinFu help options.
        *
        * \param[in] pfx TBD
        * \param[in] with_inputs TBD
        */
      static void
      usageHelp (const char *pfx = "", bool with_inputs = true);
      
      /** \brief Print the rxKinFu status. */
      virtual int
      printStatus();
      
      /** \brief Setup the KinFu.
        *
        * \param[in] volume_size_meters TBD
        * \param[in] en_icp TBD
        * \param[in] en_vizualization TBD
        * \param[in] show_current_cloud TBD
        * \param[in] show_raycast TBD
        * \param[in] show_depth TBD
        * \param[in] truncation_scaling TBD
        * \param[in] eval_mode TBD
        * \param[in] live TBD
        * \param[in] trigger TBD
        * \param[in] capture TBD
        */
      virtual void
      setup (const Eigen::Vector3f &volume_size_meters,
             bool en_icp, bool en_vizualization,
             bool show_current_cloud,
             bool show_raycast, bool show_depth,
             bool truncation_scaling, bool eval_mode,
             bool live, bool trigger,
             boost::shared_ptr<pcl::Grabber> capture);

      /** \brief Setup the KinFu.
        *
        * \param[in] argc TBD
        * \param[in] argv TBD
        * \param[in] live TBD
        * \param[in] trigger TBD
        * \param[in] capture TBD
        */
      virtual void
      setup (int argc, char **argv, bool live, bool trigger,
             boost::shared_ptr<pcl::Grabber> capture);

      /** \brief Setup the KinFu.
        *
        * \param[in] argc TBD
        * \param[in] argv TBD
        */
      virtual void
      setup (int argc, char **argv);

      /** \brief Setup the KinFu vizualization.
        *
        * \param[in] show_current_cloud TBD
        * \param[in] show_raycast TBD
        * \param[in] show_depth TBD
        */
      virtual void
      setupViz (bool show_current_cloud, bool show_raycast, bool show_depth);

      virtual void setupKinfu (const Eigen::Vector3f &volume_size_meters,
                               bool en_icp, bool truncation_scaling);

      virtual pcl::Grabber* makeCapture (int argc, char **argv,
                                         bool &live, bool &trigger);

      virtual KinfuTracker* makeKinfuTracker ();

      virtual TsdfVolumeHost<float, short>* makeTsdfVolumeHost();

      virtual ImageView* makeImageView(bool show_raycast, bool show_depth);
      virtual SceneCloudView* makeSceneCloudView(bool show_cloud);
      virtual CurrentCloudView* makeCurrentCloudView(bool show_cloud);

      virtual void toggleCameraMode();
      virtual void setIndependentCameraMode();
      virtual void setBubbleCameraMode();
      virtual void setKinectCameraMode();
    
      virtual void togglePaused();
      virtual void setPaused(bool en);
      virtual void setSingleStep(bool en);

      virtual void toggleShowCurentCloudInScene();
      virtual void setShowCurrentCloudInScene(bool show);

      virtual void toggleShowBubbleCloud();
      virtual void setShowBubbleCloud(bool show);

      #define TOGGLE_SHOW(foo, Foo)            \
        virtual void toggleShow##Foo();      \
        virtual void setShow##Foo(bool show);
          
      TOGGLE_SHOW(camera,Camera)
      TOGGLE_SHOW(gravity,Gravity)
      TOGGLE_SHOW(down,Down)
      TOGGLE_SHOW(velocity,Velocity)
      TOGGLE_SHOW(heading,Heading)
      TOGGLE_SHOW(bubble,Bubble)
      TOGGLE_SHOW(bubble_frusta,BubbleFrusta)

      #undef TOGGLE_SHOW

      virtual void setEstimateDownFromGravity(bool en);
      virtual void setEstimateHeadingFromVelocity(bool en);
      virtual void setEstimateHeadingFromCam(bool en);
      virtual void setMovingVolumeVelocityAvgThresh(float thresh);
      virtual void setMovingVolumeVelocityAvgWeight(float weight);
      virtual void setSceneCloudTriangleSize(int ts);

      virtual void setBubbleCloudTriangleSize(int ts);
      virtual void setBubbleSize(float const *sz);
      virtual void setBubbleResolution(float const *sz);
      virtual void setBubbleOffset(const Eigen::Vector3f &o);
      virtual void initBubble();

      virtual bool execKinfu(double timestamp);

      virtual void execScan();

      virtual void execBubbleRaycast(const Eigen::Affine3f &bubble_pose);
      
      virtual void bubbleCamBackproject(float &x, float &y, Eigen::Vector3f p,
                                        int face = 0);

      virtual void execExt() {}

      virtual void execViz(bool has_data, bool has_image,
                          const Eigen::Affine3f &bubble_pose);

      virtual void spinViewers(int iterations = 1);

      virtual void copyDepth(const boost::shared_ptr<pcl::io::openni2::DepthImage>&
                            data);

      //cookie=0 when called from a gfx window, cookie=1 when called from console
      virtual void keyCB(const pcl::visualization::KeyboardEvent &e,
                        void *cookie);
      
      /** \brief Mouse callback function. */
      virtual void
      mouseCB (const pcl::visualization::MouseEvent &e, void *cookie);
      
      /** \brief Point Picking callback function. */
      virtual void
      pointCB (const pcl::visualization::PointPickingEvent &e, void *cookie);
      
      /** \brief Callback function for the openni-based depth images. */
      virtual void
      depthCB (const boost::shared_ptr<pcl::io::openni2::DepthImage>& data);
      
      /** \brief Callback function for the openni-based point cloud. */
      void
      pointCloudCB(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&
                   data);
      
      /** \brief Callback function for the depth images. */
      virtual void
      depthImageCB (const sensor_msgs::ImageConstPtr& msg);
      
      #ifdef HAVE_IMUCAM
      virtual void frameCB(const imucam::Frame::ConstPtr& frame);
      #endif

      virtual boost::signals2::connection startCapture();

      virtual Eigen::Affine3f getBubblePose();

      virtual void updateMovingVolumeVectors();

      virtual bool shouldQuit();

      virtual bool grab();

      virtual bool quit();

      virtual void quit(bool q);
      
      virtual void setupMovingVolume(int argc, char **argv);
      
      virtual bool setInitialCameraPose(int argc, char **argv);
      
      boost::shared_ptr<pcl::Grabber> getGrabber() { return capture; }
      
      boost::shared_ptr<KinfuTracker> getKinfu() { return kinfu; }
      
      TsdfVolumeHost<float, short>::Ptr getTsdfVolumeHost()
      { return tsdf_volume_host; }
      
      static void
      setCudaDevice (int argc, char **argv);
      
      static std::vector<std::string>
      getPCDFilesInDir(const std::string& directory);
      
      /** \brief Publiser of the ROS topics such as:
        *        -- the bubble point cloud
        * 
        */
      virtual void
      publishROSTopics ();
      
      /** \brief The main loop that runs in fixed framerate. */
      virtual void
      mainLoop ();
        
    protected:
      /** \brief Whether to print the status of the kinfu: frame, capture, app,
        * and tracker info.
        */
      bool print_status;
      
      /** \brief The ROS node that handles the rxkinfu. */
      ros::NodeHandle node_;
      
      /** \brief Camera original point cloud publisher. */
      ros::Publisher original_pc_pub_;
      
      /** \brief Bubble raycasted point cloud publisher. */
      ros::Publisher bubble_pc_pub_;
      
      /** \brief Test depth image publisher. */
      ros::Publisher depth_image_pub_;
      
      /** \brief Depth image subscriber. */
      ros::Subscriber depth_image_sub_;
    
      boost::shared_ptr<pcl::Grabber> capture;
      
      boost::shared_ptr<KinfuTracker> kinfu;
      
      boost::shared_ptr<SceneCloudView> scene_cloud_view;
      boost::shared_ptr<ImageView> image_view;
      boost::shared_ptr<CurrentCloudView> current_cloud_view;
      
      boost::mutex quit_mutex;
      bool quit_now;
      
      bool scan, scan_mesh, scan_volume;
      bool independent_camera, live_grabber, triggered;
      
      TsdfVolumeHost<float, short>::Ptr tsdf_volume_host;
      
      /** \brief The TSDF raycasted point cloud, after download request.*/
      pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud;
      
      boost::mutex data_mutex; //TBD doc what is guarded by this
      boost::condition_variable data_ready_cond;
      
      int dropped_frames, max_data_queue;
      double timestamp, timestamp_was;
      KinfuTracker::DepthMap depth_device;
      
      /** \brief The input depth data. */
      std::queue<boost::shared_ptr<unsigned short> > source_depth_data;
      
      /** \brief The input depth data for the GPU. */
      std::queue<PtrStepSz<const unsigned short> > source_depth;
      
      std::vector<unsigned short> viz_depth_data;
      PtrStepSz<const unsigned short> viz_depth;
      
      bool viz, eval, timing, paused, single_step;
      
      float velocity_avg_thresh, velocity_avg_weight;
      
      bool reset_camera_pending;
      bool show_current_cloud_in_scene;
      int  scene_cloud_triangle_size, bubble_cloud_triangle_size;
      
      bool show_bubble_cloud, show_bubble, show_bubble_frusta;
      bool estimate_down_from_gravity;
      bool estimate_heading_from_velocity, estimate_heading_from_cam;
      
      Eigen::Vector3f gravity, down, velocity, heading;
      
      Eigen::Vector3f bubble_offset;
      float bubble_size[6], bubble_resolution[6];
      std::string bubble_cloud_name[6], bubble_cam_name[6];
      Eigen::AngleAxisf bubble_aa[6];
      int bubble_cols[6], bubble_rows[6];
      float bubble_cx[6], bubble_cy[6], bubble_fx[6], bubble_fy[6];
      Eigen::Vector3f bubble_camera_loc;
      Eigen::Affine3f bubble_camera_viewpoint;
      Eigen::AngleAxisf bubble_rot;
      RayCaster::Ptr bubble_raycaster_ptr[6];
      
      /** \brief The 2D array device for the bubble box raycasted point clouds. */
      DeviceArray2D<pcl::PointXYZ> bubble_cloud_device;

      /** \brief A pointer to the point cloud raycasted from the bubble box. */
      pcl::PointCloud<pcl::PointXYZ>::Ptr bubble_rc_cloud_ptr[6];
      
      /** \brief A pointer to the original raycasted point cloud. */
      pcl::PointCloud<pcl::PointXYZ>::Ptr original_rc_pc_ptr;
      
      /** \brief A pointer to the original point cloud. */
      pcl::PointCloud<pcl::PointXYZ>::Ptr original_pc_ptr;

      
      bool show_camera;
      float cam_near, cam_far;
      
      bool show_gravity, show_down, show_velocity, show_heading;
      
      bool bubble_camera;
      
      int last_status_num_newlines;
      
      float frame_ms, grab_ms, upload_ms, track_ms, viz_ms;
      float scan_ms, bubble_raycast_ms, exec_ext_ms, accounted_ms, exec_ms;
      
      int execs, frame_num;
      float fps;
      float latency, max_latency;
      
      #ifdef HAVE_IMUCAM    
        std::queue<imucam::Frame::ConstPtr> source_frame;
        imucam::Frame::ConstPtr current_frame;
      #endif

    private:
      KinfuAppRos(const KinfuAppRos& other); // Disable
      KinfuAppRos& operator=(const KinfuAppRos&); // Disable
  };
}

#endif