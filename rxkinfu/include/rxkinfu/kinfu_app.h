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
 *  Author: Dimitrios Kanoulas (dkanoulas@gmail.com), Marsette Vona
 */

#ifndef MVKINFU_KINFU_APP_H
#define MVKINFU_KINFU_APP_H

#include "kinfu_tracker.h"
#include "cuda/containers.h"
#include "tsdf_volume_host.h"
#include "image_view.h"
#include "cloud_view.h"

#ifdef HAVE_IMUCAM
#include <imucam/imucam.h>
#endif

#include <pcl/io/openni2_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

namespace rxkinfu { 

  class KinfuApp {
    
  public:
    
    KinfuApp(const Eigen::Vector3f &volume_size_meters,
             bool en_icp, bool en_vizualization,
             bool show_current_cloud, bool show_raycast, bool show_depth,
             bool truncation_scaling, bool eval_mode,
             bool live, bool trigger,
             boost::shared_ptr<pcl::Grabber> capture);

    KinfuApp(int argc, char **argv, bool live, bool trigger,
             boost::shared_ptr<pcl::Grabber> capture);

    KinfuApp(int argc, char **argv);
    
    virtual ~KinfuApp() {}

    virtual void setup(const Eigen::Vector3f &volume_size_meters,
                       bool en_icp, bool en_vizualization,
                       bool show_current_cloud,
                       bool show_raycast, bool show_depth,
                       bool truncation_scaling, bool eval_mode,
                       bool live, bool trigger,
                       boost::shared_ptr<pcl::Grabber> capture);

    virtual void setup(int argc, char **argv, bool live, bool trigger,
                       boost::shared_ptr<pcl::Grabber> capture);

    virtual void setup(int argc, char **argv);

    virtual void setupViz(bool show_current_cloud,
                          bool show_raycast, bool show_depth);

    virtual void setupKinfu(const Eigen::Vector3f &volume_size_meters,
                            bool en_icp, bool truncation_scaling);

    virtual pcl::Grabber* makeCapture(int argc, char **argv,
                                      bool &live, bool &trigger);

    virtual KinfuTracker* makeKinfuTracker();

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

#define TOGGLE_SHOW(foo, Foo)                                           \
    virtual void toggleShow##Foo();                                     \
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

    virtual void depthCB(const boost::shared_ptr<pcl::io::openni2::DepthImage>&
                         data);

#ifdef HAVE_IMUCAM
    virtual void frameCB(const imucam::Frame::ConstPtr& frame);
#endif

    virtual boost::signals2::connection startCapture();

    virtual Eigen::Affine3f getBubblePose();

    virtual void updateMovingVolumeVectors();

    virtual bool shouldQuit();

    virtual bool grab();

    virtual int printStatus();

    virtual void mainLoop();

    virtual bool quit();

    virtual void quit(bool q);

    virtual void printHelp();
    
    virtual void setupMovingVolume(int argc, char **argv);

    virtual bool setInitialCameraPose(int argc, char **argv);

    boost::shared_ptr<pcl::Grabber> getGrabber() { return capture; }

    boost::shared_ptr<KinfuTracker> getKinfu() { return kinfu; }

    TsdfVolumeHost<float, short>::Ptr getTsdfVolumeHost()
    { return tsdf_volume_host; }

    //cookie=0 when called from a gfx window, cookie=1 when called from console
    virtual void keyCB(const pcl::visualization::KeyboardEvent &e,
                       void *cookie);
    
    /** \brief Mouse callback function. */
    virtual void mouseCB(const pcl::visualization::MouseEvent &e, void *cookie);
    
    /** \brief Point Picking callback function. */
    virtual void pointCB(const pcl::visualization::PointPickingEvent &e,
                         void *cookie);

    static void usageHelp(const char *pfx = "", bool with_inputs = true);

    static void setCudaDevice(int argc, char **argv);

    static std::vector<std::string>
      getPCDFilesInDir(const std::string& directory);

  protected:
    
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud;
    
    boost::mutex data_mutex; //TBD doc what is guarded by this
    boost::condition_variable data_ready_cond;
   
    int dropped_frames, max_data_queue;
    double timestamp, timestamp_was;
    
    KinfuTracker::DepthMap depth_device;
    //KinfuTracker::View color_device_; //davidjones: ADDED FOR COLOR
    
    std::queue<boost::shared_ptr<unsigned short> > source_depth_data;
    std::queue<PtrStepSz<const unsigned short> > source_depth;

    std::vector<unsigned short> viz_depth_data;
    PtrStepSz<const unsigned short> viz_depth;

#ifdef HAVE_IMUCAM    
    std::queue<imucam::Frame::ConstPtr> source_frame;
    imucam::Frame::ConstPtr current_frame;
#endif

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr bubble_cloud_ptr[6];
    DeviceArray2D<pcl::PointXYZ> bubble_cloud_device;

    bool show_camera;
    float cam_near, cam_far;

    bool show_gravity, show_down, show_velocity, show_heading;
    
    bool bubble_camera;

    bool print_status;

    int last_status_num_newlines;

    float frame_ms, grab_ms, upload_ms, track_ms, viz_ms;
    float scan_ms, bubble_raycast_ms, exec_ext_ms, accounted_ms, exec_ms;

    int execs, frame_num;
    float fps;
    float latency, max_latency;

  private:
    KinfuApp(const KinfuApp& other); // Disable
    KinfuApp& operator=(const KinfuApp&); // Disable
  };
}

#endif
