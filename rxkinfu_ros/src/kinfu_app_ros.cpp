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

#include "kinfu_app_ros.h"

#define DEF_VOL_SZ_M 3.0f
#define INITIAL_CAM_SETBACK 0.6f
#define TSDF_TRUNC_DIST_M 0.030f
#define ICP_FILTER_DIST_THRESH_M 0.1f
#define ICP_FILTER_ANGLE_THRESH_DEG 20.0f
#define CAMERA_MOVEMENT_THRESHOLD 0.001f
#define TSDF_TRUNC_RATIO 0.01f
#define MAX_TRI_SZ 8
#define DEF_BUBBLE_SZ 1.0f
#define DEF_BUBBLE_RES 100
#define DEF_VELOCITY_AVG_THRESH 0.2f
#define DEF_VELOCITY_AVG_WEIGHT 0.1f
#define DATA_WAIT_MS 100
#define FPS_SKIP 5
#define DEF_FRAME_MS 33
#define SIMULATE_DROPPED_FRAMES 0
#define SPIN_SLEEP_MS 10
#define DEF_MAX_DATA_QUEUE 10
#define DEF_BUB_CAM_LOC 7,-3,-3
#define DEF_CAM_NEAR 0.5f
#define DEF_CAM_FAR 5.0f

#include <iostream>

#include <boost/filesystem.hpp>

#ifdef HAVE_IMUCAM
#include <imucam/fmt.h>
#endif

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/exceptions.h>
#include <pcl/common/angles.h>

using namespace Eigen;
using namespace pcl;
using namespace rxkinfu;
namespace pc = pcl::console;

// setup the Range Sensor frame and topic
const std::string RANGE_SENSOR_FRAME = "/camera_rgb_optical_frame";
const std::string RANGE_SENSOR_TOPIC = "/camera/depth_registered/points";

/** \brief MACRO for framerate printing. */
#define FPS_CALC(_WHAT_) \
do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
  { \
    std::cout << "Average framerate("<< _WHAT_ << "): " \
    << double(count)/double(now - last) << " Hz" <<  std::endl; \
    count = 0; \
    last = now; \
  } \
}while(false)

///////////////////////////////////////////////////////////////////////////////
KinfuAppRos::KinfuAppRos (const Vector3f &volsz, bool icp,
                          bool viz, bool cc, bool rv, bool dv,
                          bool ts, bool ev, bool live, bool trig,
                          boost::shared_ptr<pcl::Grabber> cap)
{
  setup (volsz, icp, viz, cc, rv, dv, ts, ev, live, trig, cap);
}

///////////////////////////////////////////////////////////////////////////////
KinfuAppRos::KinfuAppRos (int argc, char **argv,
                          bool live, bool trig,
                          boost::shared_ptr<pcl::Grabber> cap)
{
  setup (argc, argv, live, trig, cap);
}

///////////////////////////////////////////////////////////////////////////////
KinfuAppRos::KinfuAppRos (int argc, char **argv, ros::NodeHandle node)
{
  node_ = node; // set the ROS node
  setup (argc, argv);
}

///////////////////////////////////////////////////////////////////////////////
void KinfuAppRos::printHelp ()
{
  std::cout
    << std::endl
    << "    h    : print this help\n"
    << "  esc,q  : quit\n"
    << "    t    : download cloud on next frame\n"
    << "    a    : download mesh on next frame\n"
    << "    m    : toggle cloud exctraction mode\n"
    << "    n    : toggle normals extraction\n"
    << "    i    : toggle independent camera mode\n"
    << "    b    : toggle volume bounds\n"
    << "    c    : clear clouds and meshes from viewer\n"
    << "  "
    << SceneCloudView::PCD_BIN << "," << SceneCloudView::PCD_ASCII << ","
    << SceneCloudView::PLY
    << "  : save last cloud as binary PCD/ascii PCD/PLY\n"
    << "   " << SceneCloudView::MESH_PLY << "," << SceneCloudView::MESH_VTK
    << "   : save last mesh as PLY/VTK\n"
    << "    x    : also download TSDF when taking cloud\n"
    << "    v    : save last TSDF\n"
    << "    d    : toggle show current cloud/mesh in scene view"
    << ", cycle mesh resolution\n"
    << "    k    : toggle show camera\n"
    << "    +    : toggle show bubble\n"
    << "    z    : toggle show bubble frusta\n"
    << "    y    : toggle show bubble cloud/mesh, cycle mesh resolution\n"
    << "   spc   : toggle pause\n"
    << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
void KinfuAppRos::usageHelp (const char *pfx, bool with_inputs)
{
  std::cout
    << pfx << "  -h : print this message\n"
    << pfx << "  -gpu <CUDA device num> (default 0)\n"
    << pfx << "  -cc : show current frame cloud\n"
    << pfx << "  -ts : scale trunc dist & raycaster on vol size\n"
    << pfx << "  -noicp : disable icp\n"
    << pfx << "  -noviz : disable visualization\n"
    << pfx << "  -eval : save all poses for evaluation\n"
    << pfx << "  -campos <x,y,z> : initial camera position in meters\n"
    << pfx << "  -camaa <angle,x,y,z> : initial camera angle [rad]@(x, y, z)\n"
    << pfx << "  -vs <s>|<w,h,d> : TSDF volume size in meters\n"
    << pfx << "  -indcam : start with independent graphics camera\n"
    << pfx << "  -pause : start paused\n"
    << pfx << "  -mdq <n> : set max data queue size (default "
    << DEF_MAX_DATA_QUEUE << ")\n"
    << pfx << "  -step : single step mode\n"
    << pfx << "  -ndv : no depth camera view\n"
    << pfx << "  -nrv : no raycast view\n"
    << pfx << "  -sc [ts] : show current cloud (as mesh with tri size ts)\n"
    << pfx << "  -drawcam [near [far]]: show camera frustum\n";

  std::cout
    << pfx << "  -mvcnn : moving volume check nearest neighbor\n"
    << pfx << "  -mvcv : moving volume check valid\n"
    << pfx << "  -mvfc : moving volume fixed camera mode\n"
    << pfx << "  -mvfd : moving volume fix down then forward mode\n"
    << pfx << "  -mvff : moving volume fix forward then down mode\n"
    << pfx << "  -mvdthresh <t> : moving volume dist thresh [m] or inf\n"
    << pfx << "  -mvathresh <t> : moving volume angle thresh [rad] or inf\n"
    << pfx << "  -downgrav : set moving volume down from IMU gravity\n"
    << pfx << "  -headvel : set moving volume heading from estimated velocity\n"
    << pfx << "  -headcam: set moving volume heading from camera +z\n"
    << pfx << "  -vthresh <t> : moving volume velocity running avg thresh\n"
    << pfx << "  -vweight <w> : moving volume velocity running avg weight\n"
    << pfx << "  -drawvel : show estimated velocity vector\n"
    << pfx << "  -drawhead : show heading vector\n"
    << pfx << "  -drawgrav : show gravity vector\n"
    << pfx << "  -drawdown : show down vector\n"
    << pfx << "  -bc [ts] : show bubble cloud (as mesh with tri size ts)\n"
    << pfx << "  -b : show moving volume bubble\n"
    << pfx << "  -bf : show moving volume bubble frusta\n"
    << pfx << "  -bo <x,y,z> : set moving volume bubble offset in meters\n"
    << pfx << "   next two args take 1-6 floats like this: "
    << "bottom,front,left,right,back,top\n"
    << pfx << "  -bs [...] : set moving volume bubble dimensions in meters\n"
    << pfx << "  -br [...] : set moving volume bubble resolutions in px/m\n"
    << pfx << "  -bubcamloc [x,y,z] set bubble fixed camera location (default "
    << Vector3f(DEF_BUB_CAM_LOC).transpose() << ")\n"
    << pfx << "  -bubcam : start with bubble-fixed camera\n"
    << pfx << "  -print_status : print the kinfu status\n";

  if (with_inputs)
  {
    std::cout
      << pfx << "  -dev <openni device num> (default 0)\n"
      << pfx << "  -oni <file>\n"
      << pfx << "  -pcd <file or dir> [-pcd_fps N (default 15.0)]\n";
#ifdef HAVE_IMUCAM
    std::cout << pfx << "  -imucam_grabber [\"arg1 arg2 ...\"]\n";
    std::cout << pfx << "  -imucam_grabber args:\n";
    std::string pfx2_str = FMT(pfx << "    ");
    const char *pfx2 = pfx2_str.c_str();
    imucam::Grabber::usageShort(pfx2, pfx2);
    imucam::Grabber::usageHelp(pfx2);
    std::cout << pfx << "  -imucam_reader [\"arg1 arg2 ...\"]\n";
    std::cout << pfx << "  -imucam_reader args:\n";
    imucam::Reader::usageShort(pfx2, pfx2);
    std::cout << pfx2 << "[-untriggered]\n";
    imucam::Reader::usageHelp(pfx2);
    std::cout << pfx2 << "-untriggered : free-run mode, may drop frames\n";
#endif
  }
}

///////////////////////////////////////////////////////////////////////////////
int KinfuAppRos::printStatus ()
{
  int queued_frames, dropped_frames = 0;
  {
    boost::unique_lock<boost::mutex> lock(data_mutex);
    dropped_frames = KinfuAppRos::dropped_frames; 
    queued_frames = source_depth.size();
  }
  
  float total_ms_tracker, total_ms_volume_xform,
        max_ms_tracker, max_ms_volume_xform,
        execute_ms, create_maps_ms, icp_ms, integrate_ms,
        raycast_ms, transform_tsdf_ms, shift_tsdf_ms;
  kinfu->getTimings (total_ms_tracker, total_ms_volume_xform,
                     max_ms_tracker, max_ms_volume_xform,
                     execute_ms, create_maps_ms, icp_ms, integrate_ms,
                     raycast_ms, transform_tsdf_ms, shift_tsdf_ms);
  
  int num_resets, num_volume_xforms;
  kinfu->getStats (num_resets, num_volume_xforms);
  
  pc::print_info ("frame: %6d %6.1ffps timestamp diff %3.fms, "
                  "%3.fms accounted (upld+trk+bubble)\n",
                  frame_num, fps, frame_ms, accounted_ms);
  pc::print_info ("capture: %2d queued frames, %3d dropped frames, "
                  "%9.fms est latency\n",
                  queued_frames, dropped_frames, latency); 
  pc::print_info ("app: %2.fms grab, %1.fms upld, %3.fms track, "
                  "%2.fms bubble, %4.fms ext, %4.fms viz\n",
                  grab_ms, upload_ms, track_ms, bubble_raycast_ms, exec_ext_ms,
                  viz_ms);
  pc::print_info ("tracker: %3.fms maps, %3.fms icp, "
                  "%3.fms integrate, %3.fms raycast, %3d resets\n",
                  create_maps_ms, icp_ms, integrate_ms, raycast_ms,
                  num_resets);
  pc::print_info ("moving vol: %3.fms transform, %3.fms shift, "
                  "%3d xforms, %3.fms max\r",
                  transform_tsdf_ms, shift_tsdf_ms,
                  num_volume_xforms, max_ms_volume_xform);

  return 4; //number of newlines printed
}

///////////////////////////////////////////////////////////////////////////////
void KinfuAppRos::setup (const Vector3f &volsz, bool en_icp,
                         bool en_viz, bool cc, bool rv, bool dv,
                         bool ts, bool ev, bool live, bool trig,
                         boost::shared_ptr<pcl::Grabber> cap)
{
  if (!cap) throw std::runtime_error("no capture object");
    capture = cap;

  quit(false);

  max_data_queue = DEF_MAX_DATA_QUEUE;
  dropped_frames = 0;

  timestamp = timestamp_was = NAN;

  scan = scan_mesh = scan_volume = false;
  independent_camera = false;

  timing = false;
  paused = single_step = false;

  velocity_avg_thresh = DEF_VELOCITY_AVG_THRESH;
  velocity_avg_weight = DEF_VELOCITY_AVG_WEIGHT; 

  reset_camera_pending = false;

  show_current_cloud_in_scene = false;
  scene_cloud_triangle_size = bubble_cloud_triangle_size = 0;

  show_bubble_cloud = show_bubble = show_bubble_frusta = false;

  estimate_down_from_gravity = false;
  estimate_heading_from_velocity = estimate_heading_from_cam = false;

  gravity = Vector3f::UnitY(); down = Vector3f::UnitY();
  velocity = Vector3f::Zero(); heading = Vector3f::UnitZ();

  bubble_offset = Vector3f::Zero();
  for (int i = 0; i < 6; i++)
  {
    bubble_size[i] = DEF_BUBBLE_SZ;
    bubble_resolution[i] = DEF_BUBBLE_RES;
  }
  bubble_camera_loc = Vector3f(DEF_BUB_CAM_LOC);
  initBubble();

  show_camera = false;
  cam_near = DEF_CAM_NEAR; cam_far = DEF_CAM_FAR;
  show_gravity = show_down = false;
  show_velocity = show_heading = false;
  
  bubble_camera = false;

  print_status = false;
  last_status_num_newlines = 0;
  
  frame_ms = grab_ms = upload_ms = track_ms = viz_ms = 0;
  scan_ms = bubble_raycast_ms = accounted_ms = exec_ext_ms = exec_ms = 0;

  execs = 0;
  frame_num = -1;
  fps = 0;
  latency = max_latency = 0;

  viz = en_viz; eval = ev;

  live_grabber = live;
  triggered = trig;

  tsdf_volume_host.reset(makeTsdfVolumeHost());
  tsdf_cloud.reset(new PointCloud<PointXYZI>);

  kinfu.reset(makeKinfuTracker());
  setupKinfu(volsz, en_icp, ts);

  setupViz(cc, rv, dv);
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setup (int argc, char **argv, bool live, bool trig,
                    boost::shared_ptr<pcl::Grabber> cap)
{
  setCudaDevice(argc, argv);

  bool icp = !pc::find_switch(argc, argv, "-noicp");
  bool viz = !pc::find_switch(argc, argv, "-noviz");
  bool rv = !pc::find_switch(argc, argv, "-nrv");
  bool dv = !pc::find_switch(argc, argv, "-ndv");
  bool cc = pc::find_switch(argc, argv, "-cc");
  bool ts = pc::find_switch(argc, argv, "-ts");
  bool ev = pc::find_switch(argc, argv, "-eval");

  if (!icp) pc::print_highlight("ICP disabled\n");
  if (!viz) pc::print_highlight("visualization disabled\n");
  if (ts) pc::print_highlight("truncation scaling enabled\n");
  if (ev) pc::print_highlight("evaluation mode, will save poses on exit\n");

  std::vector<float> volarg;
  Vector3f vs = Vector3f::Constant(DEF_VOL_SZ_M);
  pc::parse_x_arguments(argc, argv, "-vs", volarg);
  if (volarg.size() > 0) vs(0) = volarg[0];
  if (volarg.size() > 1) { vs(1) = volarg[1]; } else { vs(1) = vs(0); }
  if (volarg.size() > 2) { vs(2) = volarg[2]; } else { vs(2) = vs(1); }
  pc::print_highlight("volume size x,y,z = %6.3fm, %6.3fm, %6.3fm\n",
                      vs(0), vs(1), vs(2));
  
  setup(vs, icp, viz, cc, rv, dv, ts, ev, live, trig, cap);

  setupMovingVolume(argc, argv);
  
  setInitialCameraPose(argc, argv);

  pc::parse_argument(argc, argv, "-mdq", max_data_queue);
  pc::print_highlight("max data queue size %d frames\n", max_data_queue);

  if (pc::find_switch(argc, argv, "-indcam"))
  {
    pc::print_highlight("independent camera mode, hit I to toggle\n");
    setIndependentCameraMode();
  }

  if (pc::find_switch(argc, argv, "-bubcam"))
  {
    pc::print_highlight("bubble-fixed camera mode, hit I to toggle\n");
    setBubbleCameraMode();
  }

  std::vector<float> loc;
  if (pc::parse_x_arguments(argc, argv, "-bubcamloc", loc) > 0)
  {
    for (unsigned int i = 0; i < loc.size() && i < 3; i++)
      bubble_camera_loc(i) = loc[i];
    
    pc::print_highlight("bubble camera location %6.3f, %6.3f, %6.3f\n",
                        bubble_camera_loc(0), bubble_camera_loc(1),
                        bubble_camera_loc(2));
  }

  if (pc::find_switch(argc, argv, "-pause"))
  {
    setPaused(true);
    pc::print_highlight("started paused - hit spacebar to start\n");
  }

  if (pc::find_switch(argc, argv, "-step"))
  {
    setSingleStep(true); setPaused(true);
    pc::print_highlight("started single step - hit spacebar for each frame\n");
  }

  setShowCamera(pc::find_switch(argc, argv, "-drawcam"));

  std::vector<float> nf;
  if (pc::parse_x_arguments(argc, argv, "-drawcam", nf) > 0)
  {
    if (nf.size() >= 1) cam_near = nf[0];
    if (nf.size() >= 2) cam_far = nf[1];
    pc::print_highlight("cam near %6.3f, far %6.3f\n", cam_near, cam_far);
  }

  std::vector<int> tsz;
  if ((pc::parse_x_arguments(argc, argv, "-sc", tsz) > 0) ||
      pc::find_switch(argc, argv, "-sc"))
  {
    if (tsz.size() > 0)
      setSceneCloudTriangleSize((tsz[0] > MAX_TRI_SZ) ? MAX_TRI_SZ :
                                (tsz[0] < 0) ? 0 : tsz[0]);
    setShowCurrentCloudInScene(true);
  }

  tsz.clear();
  if ((pc::parse_x_arguments(argc, argv, "-bc", tsz) > 0) ||
      pc::find_switch(argc, argv, "-bc"))
  {
    if (tsz.size() > 0)
      setBubbleCloudTriangleSize((tsz[0] > MAX_TRI_SZ) ? MAX_TRI_SZ :
                                 (tsz[0] < 0) ? 0 : tsz[0]);
    setShowBubbleCloud(true);
  }

  setShowBubble(pc::find_switch(argc, argv, "-b"));

  setShowBubbleFrusta(pc::find_switch(argc, argv, "-bf"));

  std::vector<float> barg;
  float bsz[] = {DEF_BUBBLE_SZ, DEF_BUBBLE_SZ, DEF_BUBBLE_SZ,
                 DEF_BUBBLE_SZ, DEF_BUBBLE_SZ, DEF_BUBBLE_SZ};
  if (pc::parse_x_arguments(argc, argv, "-bs", barg) > 0)
  {
    for (unsigned int i = 0; i < barg.size() && i < 6; i++) bsz[i] = barg[i];
    pc::print_highlight("bubble size "
                        "bottom=%6.3f front=%6.3f left=%6.3f right=%6.3f "
                        "back=%6.3f top=%6.3f\n",
                        bsz[0], bsz[1], bsz[2], bsz[3], bsz[4], bsz[5]);
  }
  setBubbleSize(bsz);
  
  barg.clear();
  float bres[] = {DEF_BUBBLE_RES, DEF_BUBBLE_RES, DEF_BUBBLE_RES,
                  DEF_BUBBLE_RES, DEF_BUBBLE_RES, DEF_BUBBLE_RES};
  if (pc::parse_x_arguments(argc, argv, "-br", barg) > 0)
  {
    for (unsigned int i = 0; i < barg.size() && i < 6; i++) bres[i] = barg[i];
    pc::print_highlight("bubble resolution "
                        "bottom=%3.f front=%3.f left=%3.f right=%3.f "
                        "back=%3.f top=%3.f\n",
                        bres[0], bres[1], bres[2], bres[3], bres[4], bres[5]);
  }
  setBubbleResolution(bres);

  barg.clear();
  Vector3f bofs = Vector3f::Zero();
  if (pc::parse_x_arguments(argc, argv, "-bo", barg) > 0)
  {
    for (unsigned int i = 0; i < barg.size() && i < 3; i++) bofs(i) = barg[i];
    pc::print_highlight("bubble offset = %6.3f, %6.3f, %6.3f\n",
                        bofs(0), bofs(1), bofs(2));
  }
  setBubbleOffset(bofs);
  
  initBubble(); //again now that some other things are set up
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setup (int argc, char **argv)
{
  bool live = false, trig = false;

  boost::shared_ptr<pcl::Grabber> cap(makeCapture(argc, argv, live, trig));

  pc::print_highlight("%s; %s\n",
                      (live) ? "grabbing from device" : "grabbing from file",
                      (trig) ? "triggered" : "untriggered");

  setup(argc, argv, live, trig, cap);
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setupKinfu (const Vector3f &volsz, bool icp, bool ts)
{
  kinfu->volume().setSize(volsz);
  
  Matrix3f R = Matrix3f::Identity();
  // * AngleAxisf( deg2rad(-30.f), Vector3f::UnitX());
  Vector3f t = volsz*0.5f - Vector3f(0, 0, volsz(2))*INITIAL_CAM_SETBACK;
  
  Affine3f pose = Translation3f(t) * AngleAxisf(R);
  
  kinfu->setInitalCameraPose(pose);

  kinfu->volume().setTsdfTruncDist(TSDF_TRUNC_DIST_M); //m

  kinfu->setIcpCorespFilteringParams
    (ICP_FILTER_DIST_THRESH_M, sin(deg2rad(ICP_FILTER_ANGLE_THRESH_DEG)));

  //kinfu->setDepthTruncationForICP(5.f); //m

  kinfu->setCameraMovementThreshold(CAMERA_MOVEMENT_THRESHOLD);
  
  if (!icp) kinfu->disableIcp();
  
  if (ts) kinfu->volume().setTsdfTruncDist(volsz(0)*TSDF_TRUNC_RATIO);
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setupMovingVolume (int argc, char **argv)
{
  if (pc::find_switch(argc, argv, "-mvcnn"))
  {
    pc::print_highlight("moving volume check nearest neighbor mode\n");
    kinfu->setMovingVolumeCheckNN(true);
  }

  if (pc::find_switch(argc, argv, "-mvcv"))
  {
    pc::print_highlight("moving volume check valid mode\n");
    kinfu->setMovingVolumeCheckValid(true);
  }

  if (pc::find_switch(argc, argv, "-mvfc"))
  {
    pc::print_highlight("moving volume fix camera mode\n");
    kinfu->setMovingVolumePolicy(KinfuTracker::FIX_CAMERA_IN_VOLUME);
  }

  if (pc::find_switch(argc, argv, "-mvfd"))
  {
    pc::print_highlight("moving volume fix down then forward\n");
    kinfu->setMovingVolumePolicy(KinfuTracker::FIX_DOWN_THEN_FWD_IN_VOLUME);
  }

  if (pc::find_switch(argc, argv, "-mvff"))
  {
    pc::print_highlight("moving volume fix forward then down\n");
    kinfu->setMovingVolumePolicy(KinfuTracker::FIX_FWD_THEN_DOWN_IN_VOLUME);
  }

  if (pc::find_switch(argc, argv, "-downgrav"))
  {
    pc::print_highlight("moving volume estimating down from gravity\n");
    setEstimateDownFromGravity(true);
  }
  
  if (pc::find_switch(argc, argv, "-headvel"))
  {
    pc::print_highlight("moving volume estimating heading from velocity\n");
    setEstimateHeadingFromVelocity(true);
  }
  
  if (pc::find_switch(argc, argv, "-headcam"))
  {
    pc::print_highlight("moving volume estimating heading as camera +z\n");
    setEstimateHeadingFromCam(true);
  }
  
  float thresh;
  if (pc::parse_argument(argc, argv, "-mvdthresh", thresh) > 0)
  {
    pc::print_highlight("moving volume distance threshold %fm\n", thresh);
    kinfu->setMovingVolumeDistanceThresh(thresh);
  }

  if (pc::parse_argument(argc, argv, "-mvathresh", thresh) > 0)
  {
    pc::print_highlight("moving volume angle threshold %frad\n", thresh);
    kinfu->setMovingVolumeAngleThresh(thresh);
  }
  
  if (pc::parse_argument(argc, argv, "-vthresh", thresh) > 0)
  {
    pc::print_highlight("moving volume velocity running average thresh %fm/s\n",
                        thresh);
    setMovingVolumeVelocityAvgThresh(thresh);
  }

  float weight;
  if (pc::parse_argument(argc, argv, "-vweight", weight) > 0)
  {
    pc::print_highlight("moving volume velocity running average weight %f\n",
                        weight);
    setMovingVolumeVelocityAvgWeight(weight);
  }

  setShowGravity(pc::find_switch(argc, argv, "-drawgrav"));
  setShowDown(pc::find_switch(argc, argv, "-drawdown"));
  setShowHeading(pc::find_switch(argc, argv, "-drawhead"));
  setShowVelocity(pc::find_switch(argc, argv, "-drawvel"));
}

////////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setupViz (bool cc, bool rv, bool dv)
{
  scene_cloud_view.reset (makeSceneCloudView (viz));
  scene_cloud_view->registerKeyboardCallback (&KinfuAppRos::keyCB, *this);
  scene_cloud_view->registerMouseCallback (&KinfuAppRos::mouseCB, *this);
  scene_cloud_view->registerPointPickingCallback (&KinfuAppRos::pointCB, *this);

  image_view.reset(makeImageView(viz && rv, viz && dv));
  image_view->registerKeyboardCallback(&KinfuAppRos::keyCB, *this);

  if (cc)
  {
    current_cloud_view.reset(makeCurrentCloudView(viz));
    current_cloud_view->setViewerPose(kinfu->getCameraPose());
    current_cloud_view->registerKeyboardCallback(&KinfuAppRos::keyCB, *this);
    current_cloud_view->registerMouseCallback (&KinfuAppRos::mouseCB, *this);
    current_cloud_view->registerPointPickingCallback (&KinfuAppRos::pointCB, *this);
  }
  
  if (viz)
    scene_cloud_view->toggleCube(kinfu->volume().getSize());
}

///////////////////////////////////////////////////////////////////////////////
KinfuTracker*
KinfuAppRos::makeKinfuTracker ()
{
  return new KinfuTracker ();
}

///////////////////////////////////////////////////////////////////////////////
TsdfVolumeHost<float, short>*
KinfuAppRos::makeTsdfVolumeHost ()
{
  return new TsdfVolumeHost<float, short>();
}

///////////////////////////////////////////////////////////////////////////////
ImageView*
KinfuAppRos::makeImageView (bool show_raycast, bool show_depth)
{
  return new ImageView (show_raycast, show_depth);
}

///////////////////////////////////////////////////////////////////////////////
SceneCloudView*
KinfuAppRos::makeSceneCloudView (bool show_cloud)
{
  return new SceneCloudView(show_cloud);
}

///////////////////////////////////////////////////////////////////////////////
CurrentCloudView*
KinfuAppRos::makeCurrentCloudView(bool show_cloud)
{
  return new CurrentCloudView(show_cloud);
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::toggleCameraMode ()
{
  if (!independent_camera) setIndependentCameraMode ();
  else if (!bubble_camera) setBubbleCameraMode ();
  else setKinectCameraMode ();
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setIndependentCameraMode ()
{
  if (!independent_camera || bubble_camera)
  {
    reset_camera_pending = true;
    if (kinfu && scene_cloud_view)
      scene_cloud_view->setViewerPose(kinfu->getCameraPose());
    independent_camera = true;
    bubble_camera = false;
    //std::cout << "Independent camera mode\n";
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setBubbleCameraMode ()
{
  if (!independent_camera || !bubble_camera)
  {
    independent_camera = bubble_camera = true;
    //std::cout << "Bubble-fixed camera mode\n";
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setKinectCameraMode ()
{
  if (independent_camera || bubble_camera)
  {
    independent_camera = bubble_camera = false;
    //std::cout << "Kinect-fixed camera mode\n";
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::togglePaused ()
{
  setPaused(!paused);
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setPaused (bool en)
{
  if (paused != en)
  {
    paused = en;
    //if (!single_step)
    //std::cout << "Pause mode: " << (en ?  "on" : "off") << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setSingleStep (bool en)
{
  if (single_step != en)
  {
    single_step = en;
    //std::cout << "Single step: " << (en ?  "on" : "off") << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::toggleShowCurentCloudInScene ()
{
  if (show_current_cloud_in_scene)
  {
    if (scene_cloud_triangle_size >= MAX_TRI_SZ)
    {
      setShowCurrentCloudInScene(false);
    }
    else
    {
      scene_cloud_triangle_size++;
      //std::cout << "Scene cloud triangle size "
      //          << scene_cloud_triangle_size << std::endl;
    }
  }
  else
  {
    scene_cloud_triangle_size = 0;
    setShowCurrentCloudInScene(true);
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setShowCurrentCloudInScene (bool show)
{
  if (show_current_cloud_in_scene != show)
  {
    show_current_cloud_in_scene = show;
    //std::cout << "Scene cloud " << (show ? "shown" : "hidden") << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::toggleShowBubbleCloud ()
{
  if (show_bubble_cloud)
  {
    if (bubble_cloud_triangle_size >= MAX_TRI_SZ)
    {
      setShowBubbleCloud(false);
    }
    else
    {
      bubble_cloud_triangle_size++;
      //std::cout << "Bubble cloud triangle size "
      //          << bubble_cloud_triangle_size << std::endl;
    }
  }
  else
  {
    bubble_cloud_triangle_size = 0;
    setShowBubbleCloud(true);
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setShowBubbleCloud (bool show)
{
  if (show_bubble_cloud != show)
  {
    show_bubble_cloud = show;
    initBubble();
    //std::cout << "Bubble cloud " << (show ? "shown" : "hidden") << std::endl;
  }
}

#define TOGGLE_SHOW(foo, Foo)                                           \
  void KinfuAppRos::toggleShow##Foo() { setShow##Foo(!show_##foo); }    \
  void KinfuAppRos::setShow##Foo(bool show) {                           \
    if (show_##foo != show) {                                           \
      show_##foo = show;                                                \
      /* std::cout << #Foo << " " << (show ? "shown\n" : "hidden\n") */ \
    }                                                                   \
  }

TOGGLE_SHOW(camera,Camera)
TOGGLE_SHOW(gravity,Gravity)
TOGGLE_SHOW(down,Down)
TOGGLE_SHOW(velocity,Velocity)
TOGGLE_SHOW(heading,Heading)
TOGGLE_SHOW(bubble,Bubble)
TOGGLE_SHOW(bubble_frusta,BubbleFrusta)

#undef TOGGLE_SHOW

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setEstimateDownFromGravity (bool en)
{
  estimate_down_from_gravity = en;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setEstimateHeadingFromVelocity (bool en)
{
  estimate_heading_from_velocity = en;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setEstimateHeadingFromCam (bool en)
{
  estimate_heading_from_cam = en;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setMovingVolumeVelocityAvgThresh (float thresh)
{
  velocity_avg_thresh = thresh;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setMovingVolumeVelocityAvgWeight (float weight)
{
  velocity_avg_weight = weight;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setSceneCloudTriangleSize (int ts)
{ scene_cloud_triangle_size = ts; }

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setBubbleCloudTriangleSize (int ts)
{
  bubble_cloud_triangle_size = ts;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setBubbleSize (float const *sz)
{
  for (int i = 0; i < 6; i++)
    bubble_size[i] = sz[i];
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setBubbleResolution (float const *sz)
{
  for (int i = 0; i < 6; i++)
    bubble_resolution[i] = sz[i];
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::setBubbleOffset (const Vector3f &o)
{
  bubble_offset = o;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::initBubble ()
{ 
  const std::string bflrkt = "bflrkt";
  for (int i = 0; i < 6; i++)
  {
    std::stringstream sstr;
    sstr << "bubble_cloud_" << bflrkt[i];
    bubble_cloud_name[i] = sstr.str();
    sstr.clear();
    sstr << "bubble_cam_" << bflrkt[i];
    bubble_cam_name[i] = sstr.str();
  }
  
  enum { B = 0, F = 1, L = 2, R = 3, K = 4, T = 5 };
  float *s = bubble_size, *r = bubble_resolution;
  int i; float sz, res;
  
  //bottom
  i = B; sz = s[i]; res = r[i];
  bubble_aa[i] = AngleAxisf(-0.5*M_PI, Vector3f::UnitX());
  bubble_cols[i] = static_cast<int>((s[L]+s[R])*res);
  bubble_rows[i] = static_cast<int>((s[K]+s[F])*res);
  bubble_cx[i] = s[L]*res; bubble_cy[i] = s[F]*res;
  bubble_fx[i] = bubble_fy[i] = sz*res;

  //front
  i = F; sz = s[i]; res = r[i];
  bubble_aa[i] = AngleAxisf(0, Vector3f::UnitX());
  bubble_cols[i] = static_cast<int>((s[L]+s[R])*res);
  bubble_rows[i] = static_cast<int>((s[B]+s[T])*res);
  bubble_cx[i] = s[L]*res; bubble_cy[i] = s[T]*res;
  bubble_fx[i] = bubble_fy[i] = sz*res;
    
  //left
  i = L; sz = s[i]; res = r[i];
  bubble_aa[i] = AngleAxisf(-0.5*M_PI, Vector3f::UnitY());
  bubble_cols[i] = static_cast<int>((s[K]+s[F])*res);
  bubble_rows[i] = static_cast<int>((s[B]+s[T])*res);
  bubble_cx[i] = s[K]*res; bubble_cy[i] = s[T]*res;
  bubble_fx[i] = bubble_fy[i] = sz*res;
  
  //right
  i = R; sz = s[i]; res = r[i];
  bubble_aa[i] = AngleAxisf(0.5*M_PI, Vector3f::UnitY());
  bubble_cols[i] = static_cast<int>((s[K]+s[F])*res);
  bubble_rows[i] = static_cast<int>((s[B]+s[T])*res);
  bubble_cx[i] = s[F]*res; bubble_cy[i] = s[T]*res;
  bubble_fx[i] = bubble_fy[i] = sz*res;
    
  //back
  i = K; sz = s[i]; res = r[i];
  bubble_aa[i] = AngleAxisf(M_PI, Vector3f::UnitY());
  bubble_cols[i] = static_cast<int>((s[L]+s[R])*res);
  bubble_rows[i] = static_cast<int>((s[B]+s[T])*res);
  bubble_cx[i] = s[R]*res; bubble_cy[i] = s[T]*res;
  bubble_fx[i] = bubble_fy[i] = sz*res;
  
  //top
  i = T; sz = s[i]; res = r[i];                       
  bubble_aa[i] = AngleAxisf(0.5*M_PI, Vector3f::UnitX());
  bubble_cols[i] = static_cast<int>((s[L]+s[R])*res);
  bubble_cx[i] = s[L]*res; bubble_cy[i] = s[K]*res;
  bubble_rows[i] = static_cast<int>((s[K]+s[F])*res);
  bubble_fx[i] = bubble_fy[i] = sz*res;
  
  for (int i = 0; i < 6; i++)
  {
    if (r[i] > 0)
    {
      bubble_raycaster_ptr[i].
        reset(new RayCaster(bubble_rows[i], bubble_cols[i], //+1?
                            bubble_fx[i], bubble_fy[i],
                            bubble_cx[i], bubble_cy[i]));
      bubble_rc_cloud_ptr[i].reset(new PointCloud<PointXYZ>);
    }
    else
    {
      bubble_raycaster_ptr[i] = RayCaster::Ptr();
      bubble_rc_cloud_ptr[i] = PointCloud<PointXYZ>::Ptr();
    }
  }
  
  Vector3f cam_z = -bubble_camera_loc.normalized();
  Vector3f cam_x = cam_z.cross(-Vector3f::UnitY());
  Vector3f cam_y = cam_z.cross(cam_x);
  bubble_camera_viewpoint.translation() = bubble_camera_loc;
  bubble_camera_viewpoint.linear().col(0) = cam_x;
  bubble_camera_viewpoint.linear().col(1) = cam_y;
  bubble_camera_viewpoint.linear().col(2) = cam_z;

  if (kinfu)
  {
    Quaternionf q (kinfu->getCameraPose(0).rotation());
    bubble_rot = q.inverse();
  }
}

///////////////////////////////////////////////////////////////////////////////
bool
KinfuAppRos::execKinfu (double timestamp)
{
  return kinfu->execute (depth_device, timestamp);
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::execScan ()
{

  if (scan || scan_mesh)
  {
    setShowCurrentCloudInScene(false);
    setShowBubbleCloud(false);
  }

  if (scan)
  {
    scan = false;
    
    if (viz) scene_cloud_view->show(*kinfu);
    
    if (scan_volume)
    {
      std::cout << "Downloading TSDF volume from device ..." << std::flush;
      
      kinfu->volume().downloadTsdfAndWeighs
        (tsdf_volume_host->volumeWriteable(),
         tsdf_volume_host->weightsWriteable());
      
      tsdf_volume_host->setHeader(kinfu->volume().getResolution(),
                                  kinfu->volume().getSize());
      
      std::cout << "Done, " << tsdf_volume_host->size() << " voxels\n";
      
      std::cout << "Converting to TSDF cloud ..." << std::flush;
      tsdf_volume_host->convertToTsdfCloud(tsdf_cloud);
      std::cout << "Done, " << tsdf_cloud->size() << " points\n";
      
    }
    else
    {
      std::cout << "tsdf volume download is disabled\n";
    }
  }
  
  if (scan_mesh)
  {
    scan_mesh = false;
    scene_cloud_view->showMesh(*kinfu);
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::execBubbleRaycast (const Eigen::Affine3f &bubble_pose)
{
  for (int i = 0; i < 6; i++)
  {
    if (bubble_resolution[i] > 0)
    {
      Affine3f frustum_pose = bubble_pose;
      frustum_pose.rotate(bubble_aa[i]);
      
      bubble_raycaster_ptr[i]->run(kinfu->volume(), frustum_pose);
      
      convertMapToOranizedCloud(bubble_raycaster_ptr[i]->getVertexMap(),
                                bubble_cloud_device);
      
      int c;
      bubble_cloud_device.download(bubble_rc_cloud_ptr[i]->points, c);
      bubble_rc_cloud_ptr[i]->width = bubble_cloud_device.cols();
      bubble_rc_cloud_ptr[i]->height = bubble_cloud_device.rows();
      bubble_rc_cloud_ptr[i]->is_dense = false;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::bubbleCamBackproject (float &x, float &y, Eigen::Vector3f p,
                                   int face)
{
  Affine3f cam_pose = getBubblePose();
  cam_pose.rotate(bubble_aa[face]);
  p = cam_pose.inverse()*p;
  x = bubble_fx[face]*p.x()/p.z() + bubble_cx[face];
  y = bubble_fy[face]*p.y()/p.z() + bubble_cy[face];
}

///////////////////////////////////////////////////////////////////////////////
void KinfuAppRos::execViz (bool has_data, bool has_image,
                           const Affine3f &bubble_pose)
{
  // Set viewer camera pose
  if (bubble_camera)
    scene_cloud_view->setViewerPose(bubble_pose * bubble_camera_viewpoint);
  else if (!independent_camera)
    scene_cloud_view->setViewerPose(kinfu->getCameraPose());
  
  // Show incoming depth image
  if (has_data) image_view->showDepth(viz_depth);
  //  else image_view->showGeneratedDepth(kinfu, kinfu->getCameraPose());

  // Show raycast image
  if (has_image)
  {
    Affine3f pose = scene_cloud_view->getViewerPose();
    image_view->show(*kinfu, (independent_camera) ? &pose : 0);
  }

  // Show current cloud
  if (current_cloud_view)
    current_cloud_view->show(*kinfu);    

  if (show_current_cloud_in_scene)
  {
    if (independent_camera && image_view->getRayCaster())
      scene_cloud_view->showCloud(*(image_view->getRayCaster()),
                                  "current_cloud", scene_cloud_triangle_size);
    else scene_cloud_view->showCloud(*kinfu, "current_cloud",
                                     scene_cloud_triangle_size);
  }
  else
  {
    scene_cloud_view->removeCloud("current_cloud");
  }
  
  // Show depth camera frustum
  if (show_camera && independent_camera)
    scene_cloud_view->showCamera(kinfu->getCameraPose(), "depth_cam",
                                 cam_near, cam_far,
                                 0.1f); //axes lengths
  else
    scene_cloud_view->removeCamera("depth_cam");

  // Show bubble cloud: visualize the point cloud extracted from the buuble box
  if (show_bubble_cloud)
  {
    // There are 6 different clouds to be extracted from the bubble box, one
    // per its surface (planes)
    for (int i = 0; i < 6; i++)
    {
      if (bubble_resolution[i] > 0)
        scene_cloud_view->showCloud(bubble_rc_cloud_ptr[i], bubble_cloud_name[i],
                                    bubble_cloud_triangle_size);
      else
        scene_cloud_view->removeCloud(bubble_cloud_name[i]);
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
      scene_cloud_view->removeCloud(bubble_cloud_name[i]);
  }

  // Show bubble frusta
  if (show_bubble_frusta)
  {
    for (int i = 0; i < 6; i++)
    {
      if (bubble_resolution[i] > 0)
      {
        Affine3f frustum_pose = bubble_pose;
        frustum_pose.rotate(bubble_aa[i]);
        scene_cloud_view->showCamera(frustum_pose, bubble_cam_name[i],
                                     0.1f, bubble_size[i], //near, far
                                     0, //axes
                                     1, 0, 1, //rgb
                                     bubble_rows[i], bubble_cols[i],
                                     bubble_fx[i], bubble_fy[i],
                                     bubble_cx[i], bubble_cy[i]);
      }
      else
      {
        scene_cloud_view->removeCamera(bubble_cam_name[i]);
      }
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
      scene_cloud_view->removeCamera(bubble_cam_name[i]);
  }
  
  // Show bubble and arrows
  Vector3f bubble_loc = bubble_pose.translation();

  if (show_bubble)
    scene_cloud_view->showBox(bubble_pose, bubble_size, "bubble_box");
  else
    scene_cloud_view->removeBox("bubble_box");
    
  if (show_gravity)
    scene_cloud_view->showArrow(gravity, bubble_loc, 1, 0, 1, false,
                                "gravity_arrow");
  else
    scene_cloud_view->removeArrow("gravity_arrow");
  
  if (show_down)
    scene_cloud_view->showArrow(down, bubble_loc, 0, 1, 0, false,
                                "down_arrow");
  else
    scene_cloud_view->removeArrow("down_arrow");
  
  if (show_velocity)
    scene_cloud_view->showArrow(velocity, bubble_loc, 1, 1, 0, true,
                                "velocity_arrow");
  else
    scene_cloud_view->removeArrow("velocity_arrow");
  
  if (show_heading) 
    scene_cloud_view->showArrow(heading, bubble_loc, 0, 1, 1, false,
                                "heading_arrow");
  else scene_cloud_view->removeArrow("heading_arrow");
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::spinViewers (int iterations)
{
  for (int i = 0; !quit() && ((iterations < 0) || (i < iterations)); i++)
  {
    image_view->spinOnce(1, true);
    scene_cloud_view->spinOnce();
    
    if (current_cloud_view)
      current_cloud_view->spinOnce();
    
    if (!quit() && ((iterations < 0) || (i < (iterations-1))))
      boost::this_thread::sleep(boost::posix_time::milliseconds(SPIN_SLEEP_MS));
  }
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::copyDepth (const boost::shared_ptr<pcl::io::openni2::DepthImage>&
                        data)
{
  int w = data->getWidth(), h = data->getHeight();

  PtrStepSz<const unsigned short> depth;
  boost::shared_ptr<unsigned short> depth_data;

  depth.cols = w;
  depth.rows = h;
  depth.step = w*depth.elemSize();
  
  depth_data.reset(new unsigned short[w*h]);
  data->fillDepthImageRaw(w, h, depth_data.get());
  depth.data = depth_data.get();

  if (source_depth.size() == max_data_queue)
  {
    source_depth.pop();
    source_depth_data.pop();
    //std::cerr << "dropped frame!\n";
    dropped_frames++;
  } 
  
  source_depth.push(depth);
  source_depth_data.push(depth_data);

}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::depthCB(const boost::shared_ptr<pcl::io::openni2::DepthImage>&
                     data)
{
  //FPS_CALC ("depthCB callback");
  
  if (quit()) return;
  
  if (!live_grabber) data_mutex.lock();
  else if (!data_mutex.try_lock()) return;

  copyDepth(data);
    
  data_mutex.unlock();
  data_ready_cond.notify_one();
}

//////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::pointCloudCB(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&
                          data)
{
  FPS_CALC ("pointCloudCB callback");
  
  if (quit()) return;
  
  if (!live_grabber) data_mutex.lock();
  else if (!data_mutex.try_lock()) return;

  //TBD
  
  data_mutex.unlock();
  data_ready_cond.notify_one();
}

////////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::depthImageCB (const sensor_msgs::ImageConstPtr& msg)
{
  FPS_CALC ("depthImageCB callback");
  //depth_image_pub_.publish (msg); //test the CB depth image

  int w = msg->width, h = msg->height;
  
  PtrStepSz<const unsigned short> depth;
  depth.cols = w;
  depth.rows = h;
  depth.step = w*depth.elemSize();

  boost::shared_ptr<unsigned short> depth_data;
  depth_data.reset(new unsigned short[w*h]);  
  
  //unsigned int dt = msg->data[0];
  //unsigned short dt_new = (unsigned short) dt;
  //std::cout << "data: " << dt_new << std::endl;
  for (int i=0; i<w*h; i++)    
  {
    (depth_data.get())[i] = static_cast<unsigned short>(msg->data[i]);
  }
 
  depth.data = depth_data.get();
  
  if (source_depth.size() == max_data_queue)
  {
    std::cerr << "dropped frame!\n";
    source_depth.pop();
    source_depth_data.pop();
    dropped_frames++;
  } 
  
  source_depth.push(depth);
  source_depth_data.push(depth_data);
}


#ifdef HAVE_IMUCAM
///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::frameCB (const imucam::Frame::ConstPtr& frame)
{
  if (quit()) return;
  
  if (!live_grabber) data_mutex.lock();
  else if (!data_mutex.try_lock()) return;

  //TBD: Kanoulas
  //copyDepth(frame->depth);

  if (source_frame.size() == max_data_queue) source_frame.pop();
  source_frame.push(frame);

  data_mutex.unlock();
  data_ready_cond.notify_one();
}
#endif

///////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
KinfuAppRos::startCapture ()
{
  boost::signals2::connection c;

  typedef boost::shared_ptr<pcl::io::openni2::DepthImage> DepthImagePtr;
  typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > PointCloudPtr;

  #ifdef HAVE_IMUCAM
  
  imucam::Source::Ptr source =
    boost::dynamic_pointer_cast<imucam::Source>(capture);
  
  if (source)
  {
    boost::function<void (const imucam::Frame::ConstPtr&)> func =
      boost::bind(&KinfuAppRos::frameCB, this, _1);
    c = capture->registerCallback(func);
  }
  else
  {
    boost::function<void (const DepthImagePtr&)> func =
      boost::bind(&KinfuAppRos::depthCB, this, _1);
    c = capture->registerCallback(func);
  }
  
  #else
  
  boost::function<void (const DepthImagePtr&)> func =
    boost::bind(&KinfuAppRos::depthCB, this, _1);
  c = capture->registerCallback(func);
  
  //TBD: Point Cloud callback needs to be fixed
  //boost::function<void (const PointCloudPtr&) > cloud_cb =
  //  boost::bind (&KinfuAppRos::pointCloudCB, this, _1);
  //boost::signals2::connection cloud_connection =
  //  capture->registerCallback (cloud_cb);


  #endif
  
  if (!triggered)
    capture->start();
  
  return c;
}

///////////////////////////////////////////////////////////////////////////////
Affine3f
KinfuAppRos::getBubblePose ()
{
  Affine3f bubble_pose = kinfu->getCameraPose ();
//  if (kinfu->getMovingVolumePolicy() == KinfuTracker::FIX_CAMERA_IN_VOLUME)
//    bubble_pose.linear() = bubble_pose.linear() * bubble_rot;
//  else
  bubble_pose.linear() = Matrix3f::Identity();
  bubble_pose.translation() += bubble_offset;
  return bubble_pose;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::updateMovingVolumeVectors ()
{
#ifdef HAVE_IMUCAM
  if (current_frame && !(current_frame->um6_msgs.empty()))
  {
    Vector3f g_in_camera = 
      current_frame->getCameraToWorld()->transpose() * Vector3f::UnitZ();
    Affine3f camera_to_volume = kinfu->getCameraPose();
    gravity = camera_to_volume.linear() * g_in_camera;
  }
#endif
  
  if (estimate_down_from_gravity) down = gravity;

  int np = kinfu->getNumberOfPoses();
  if (np > 1)
  {
    //current pose of camera after tracking in current volume frame
    Affine3f cam_pose = kinfu->getCameraPose(np-1);
    double cam_time = kinfu->getTimestamp(np-1);
    
    //last camera pose in previous volume frame
    Affine3f prev_cam_pose = kinfu->getCameraPose(np-2);
    double prev_cam_time = kinfu->getTimestamp(np-2);
    
    //transform from current to previous volume frame
    Affine3f vol_rel_pose = kinfu->getRelativeVolumePose(np-1);
    
    //last camera pose in current volume frame
    prev_cam_pose = vol_rel_pose.inverse() * prev_cam_pose;
    
    velocity = cam_pose.translation() - prev_cam_pose.translation();
    velocity = velocity/(cam_time-prev_cam_time);
    
    if (estimate_heading_from_cam)
    {
      heading = cam_pose.linear().col(2);
    }
    else if (estimate_heading_from_velocity &&
             (velocity.norm() > velocity_avg_thresh))
    {
      Vector3f fwd = velocity.normalized();
      if (fwd.dot(Vector3f::UnitZ()) < 0) fwd = -fwd;
      heading = heading*(1.0f-velocity_avg_weight) + fwd*velocity_avg_weight;
      heading = heading.normalized();
    }
  }
  
  kinfu->setMovingVolumeDown(down);
  kinfu->setMovingVolumeFwd(heading);
}

///////////////////////////////////////////////////////////////////////////////
bool
KinfuAppRos::shouldQuit ()
{

  if (quit()) return true;

  if (scene_cloud_view && scene_cloud_view->wasStopped())
    return true;
  if (current_cloud_view && current_cloud_view->wasStopped())
    return true;
  if (image_view && image_view->wasStopped())
    return true;

  #ifdef HAVE_IMUCAM
  imucam::Source::Ptr source =
    boost::dynamic_pointer_cast<imucam::Source>(capture);
  if (source && source->atEnd())
    return true;
  #endif

  return false;
}

///////////////////////////////////////////////////////////////////////////////
bool
KinfuAppRos::grab ()
{
  bool has_data = false;

  if (triggered) capture->start(); //trigger grabber
  
  {
    boost::unique_lock<boost::mutex> lock (data_mutex);
  
    has_data =
      !source_depth.empty() ||
      data_ready_cond.timed_wait(lock,
                                 boost::posix_time::millisec(DATA_WAIT_MS));
    has_data = !source_depth.empty();
    
    if (has_data)
    {
      PtrStepSz<const unsigned short> depth = source_depth.front();
      
      StopWatch timer;
      depth_device.upload (depth.data, depth.step, depth.rows, depth.cols);
      upload_ms = timer.getTime ();
      
      viz_depth = depth;
      viz_depth_data.resize (viz_depth.rows*viz_depth.cols);
      memcpy(&(viz_depth_data[0]), depth.data,
             viz_depth.rows*viz_depth.cols*viz_depth.elemSize());
      viz_depth.data = &(viz_depth_data[0]);
      
      source_depth.pop ();
      source_depth_data.pop ();
      
      frame_num++; 
    }
    else
    {
      upload_ms = 0;
    }
    
#ifdef HAVE_IMUCAM
    if (has_data && !source_frame.empty())
    {
      current_frame = source_frame.front();
      source_frame.pop();
    }
#endif

  } //lock(data_mutex)
  
#ifdef HAVE_IMUCAM
  if (current_frame)
    timestamp = imucam::FPS::ptimeToUS(current_frame->time)/1e6;
#endif
  
  if (isnan(timestamp))
  {
    if (!isnan(timestamp_was)) timestamp = timestamp_was + DEF_FRAME_MS/1e3;
    else { timestamp_was = 0; timestamp = DEF_FRAME_MS/1e3; }
  }
  
  frame_ms = static_cast<float>((timestamp-timestamp_was)*1e3);
  
  timestamp_was = timestamp;

  return has_data;
}

///////////////////////////////////////////////////////////////////////////////
bool
KinfuAppRos::quit ()
{
  boost::unique_lock<boost::mutex> lock(quit_mutex);
  return quit_now;
  
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::quit (bool q)
{
  boost::unique_lock<boost::mutex> lock(quit_mutex);
  quit_now = q;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::keyCB (const pcl::visualization::KeyboardEvent &e, void *cookie)
{
  int key = e.getKeyCode();
  
  if (!(e.keyDown())) return;
  
  //base class uses p, w, s, j, c, f, e, o, g, u, s, f, l, r
  switch (key) {
  case 27: case (int)'q': quit(true); break;
  case (int)'t': scan = true; break;
  case (int)'a': scan_mesh = true; break;
  case (int)'h': printHelp(); break;
  case (int)'m': scene_cloud_view->toggleExtractionMode(); break;
  case (int)'n': scene_cloud_view->toggleNormals(); break;      
  case (int)'c': scene_cloud_view->clearClouds(true); break;
  case (int)'i': toggleCameraMode(); break;
  case (int)'b':
    scene_cloud_view->toggleCube(kinfu->volume().getSize()); break;
  case (int)'7': case (int)'8':
    scene_cloud_view->writeMesh(key-(int)'0'); break;
  case (int)'1': case (int)'2': case (int)'3':
    scene_cloud_view->writeCloud(key-(int)'0'); break;      
  case (int)'x':
    scan_volume = !scan_volume;
    std::cout << "TSDF download "
              << (scan_volume ? "enabled\n" : "disabled\n");
    break;
  case (int)'v':
    std::cout << "Saving TSDF volume to tsdf_volume.dat... " << std::flush;
    tsdf_volume_host->save("tsdf_volume.dat", true);
    std::cout << "done (" << tsdf_volume_host->size() << " voxels)\n";
    std::cout << "Saving TSDF volume to tsdf_cloud.pcd... " << std::flush;
    io::savePCDFile<PointXYZI>("tsdf_cloud.pcd", *(tsdf_cloud), true);
    std::cout << "done (" << tsdf_cloud->size() << " points)\n";
    break;
  case (int)'d': toggleShowCurentCloudInScene(); break;
  case (int)'k': toggleShowCamera(); break;
  case (int)'+': toggleShowBubble(); break;
  case (int)'z': toggleShowBubbleFrusta(); break;
  case (int)'y': toggleShowBubbleCloud(); break;
  case (int)' ': togglePaused(); break;
  default: break;
  }    
}

////////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::mouseCB (const pcl::visualization::MouseEvent &e, void *cookie)
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::pointCB (const pcl::visualization::PointPickingEvent &e,
                      void *cookie)
{
  return;
}

///////////////////////////////////////////////////////////////////////////////
void KinfuAppRos::setCudaDevice (int argc, char **argv)
{
  int device = 0;
  pc::parse_argument(argc, argv, "-gpu", device);
  rxkinfu::setDevice(device);
  rxkinfu::printShortCudaDeviceInfo(device);
  //  if (checkIfPreFermiGPU(device))
  //    std::cout << "Kinfu is supported on Fermi and Kepler only.\n";
}

///////////////////////////////////////////////////////////////////////////////
std::vector<std::string>
KinfuAppRos::getPCDFilesInDir(const std::string& directory)
{
  namespace fs = boost::filesystem;
  fs::path dir(directory);
  
  std::cout << "path: " << directory << std::endl;
  if (directory.empty() || !fs::exists(dir) || !fs::is_directory(dir))
    PCL_THROW_EXCEPTION (IOException, "No valid PCD directory given!\n");
  
  std::vector<std::string> result;
  fs::directory_iterator pos(dir);
  fs::directory_iterator end;           
  
  for(; pos != end ; ++pos)
    if (fs::is_regular_file(pos->status()))
      if (fs::extension(*pos) == ".pcd")
        result.push_back(pos->path().string());
  
  return result;  
}

///////////////////////////////////////////////////////////////////////////////
Grabber*
KinfuAppRos::makeCapture (int argc, char **argv, bool &live, bool &trig)
{
  Grabber* capture = 0;

  std::string openni_device, oni_file, pcd_dir;
#ifdef HAVE_IMUCAM
  boost::shared_ptr<std::vector<char *> > av;
#endif

  live = false; trig = false;

  try
  {
    if (pc::parse_argument(argc, argv, "-dev", openni_device) > 0)
    {
      capture = new pcl::io::OpenNI2Grabber(openni_device);
      live = true;
    }
    else if (pc::parse_argument(argc, argv, "-oni", oni_file) > 0)
    {
      capture = new ONIGrabber(oni_file, false, false);
      trig = true;
    }
    else if (pc::parse_argument(argc, argv, "-pcd", pcd_dir) > 0)
    {
      float fps_pcd = 15.0f;
      pc::parse_argument(argc, argv, "-pcd_fps", fps_pcd);
      std::vector<std::string> pcd_files = getPCDFilesInDir(pcd_dir);
      sort(pcd_files.begin(), pcd_files.end());
      capture = new PCDGrabber<PointXYZ>(pcd_files, fps_pcd, false);
      #ifdef HAVE_IMUCAM
      #define  splitArgs imucam::Interactor::splitArgs
    }
    else if ((av = splitArgs(argc,argv,"-imucam_grabber")))
    {
      capture = new imucam::Grabber(av->size(), &((*av)[0]));
      imucam::Source *source = dynamic_cast<imucam::Source*>(capture);
      source->dumpConfig();
      trig = source->triggered;
      live = true;
    }
    else if ((av = splitArgs(argc,argv,"-imucam_reader")))
    {
      if (!pc::find_switch(av->size(), &((*av)[0]), "-untriggered"))
        av->push_back((char *)"-trigger"); //default: don't drop frames
      capture = new imucam::Reader(av->size(), &((*av)[0]));
      imucam::Source *source = dynamic_cast<imucam::Source*>(capture);
      source->dumpConfig();
      trig = source->triggered;
    #undef splitArgs
    #endif
    }
    else
    {
      capture = new pcl::io::OpenNI2Grabber();
      live = true;
    }
  }
  catch (const PCLException &e)
  {
    std::cerr << "error opening depth source: " << e.what() << std::endl;
  }
  
  return capture;
}

///////////////////////////////////////////////////////////////////////////////
bool
KinfuAppRos::setInitialCameraPose (int argc, char **argv)
{
  bool setcam = false;
  std::vector<float> camarg;

  if (pc::parse_x_arguments(argc, argv, "-campos", camarg) > 0)
  {
    Vector3f pos(0, 0, 0);
    if (camarg.size() > 0) pos(0) = camarg[0];
    if (camarg.size() > 1) pos(1) = camarg[1];
    if (camarg.size() > 2) pos(2) = camarg[2];
    Affine3f pose = kinfu->getCameraPose();
    pose.translation() = pos;
    kinfu->setInitalCameraPose(pose);
    setcam = true;
  }

  camarg.clear();
  if (pc::parse_x_arguments(argc, argv, "-camaa", camarg) > 0)
  {
    float angle = 0;
    if (camarg.size() > 0) angle = camarg[0];
    Vector3f axis(0, 0, 0);
    if (camarg.size() > 1) axis(0) = camarg[1];
    if (camarg.size() > 2) axis(1) = camarg[2];
    if (camarg.size() > 3) axis(2) = camarg[3];
    AngleAxisf aa(angle, axis.normalized());
    Affine3f pose = kinfu->getCameraPose();
    pose.linear() = aa.toRotationMatrix();
    kinfu->setInitalCameraPose(pose);
    setcam = true;
  }

  Affine3f pose = kinfu->getCameraPose();
  pc::print_highlight("initial camera pose\n");
  std::cout << pose.matrix() << std::endl;

  return setcam;
}

///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::publishROSTopics ()
{
  // Original point cloud
  DeviceArray2D<pcl::PointXYZ> cloud_device;
  kinfu->getLastFrameCloud (cloud_device);
  int c; cloud_device.download(original_rc_pc_ptr->points, c);
  original_rc_pc_ptr->width = cloud_device.cols();
  original_rc_pc_ptr->height = cloud_device.rows();
  original_rc_pc_ptr->is_dense = false;
  original_rc_pc_ptr->header.frame_id = RANGE_SENSOR_FRAME; // set frame
  original_pc_pub_.publish (original_rc_pc_ptr);
  
  // For every side of the bubble box
  for (int i = 0; i < 6; i++)
  {
    if (bubble_resolution[i] > 0)
    {
      bubble_rc_cloud_ptr[i]->header.frame_id = RANGE_SENSOR_FRAME; // set frame
      bubble_pc_pub_.publish (bubble_rc_cloud_ptr[i]);
    }
  }
}


///////////////////////////////////////////////////////////////////////////////
void
KinfuAppRos::mainLoop ()
{
  // Initialize ROS
  original_pc_pub_ =
    node_.advertise<sensor_msgs::PointCloud2> ("original_point_cloud", 1);
  
  bubble_pc_pub_ =
    node_.advertise<sensor_msgs::PointCloud2> ("bubble_point_cloud", 1);
  
  depth_image_pub_ =
    node_.advertise<sensor_msgs::Image> ("input_depth_image", 1);
    
  depth_image_sub_ = node_.subscribe ("/camera/depth/image_raw", 1,
                                      &KinfuAppRos::depthImageCB, this);
 
  // Point cloud creation
  original_rc_pc_ptr = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
  
  #ifdef HAVE_IMUCAM
  imucam::NonblockingConsole nonblocking_console;
  #endif

  // Start grabber
  boost::signals2::connection c = startCapture ();

  //run loop
  int num_newlines = 0;
  while (!shouldQuit ())
  {
    StopWatch timer;
    
    bool has_data = false, has_image = false;
    
    #ifdef HAVE_IMUCAM
    //read keyboard hits from console
    int key = nonblocking_console.getch();
    if (key != EOF)
      keyCB(imucam::Viewer::makeKeyEvent(key), reinterpret_cast<void*>(1));
    #endif
    
    has_data = has_image = false;
    timestamp = NAN;
    
    if (paused)
    {
      grab_ms = upload_ms = track_ms = 0;
      goto skip_grab_and_track;
    }
    
    // Fill next image to depth_device and viz_depth
    grab_ms = timer.getTime();
    has_data = grab ();
    grab_ms = timer.getTime() - grab_ms;
    
    //estimate theoretical latency
    if (!live_grabber && has_data && (frame_ms > 0) && (accounted_ms > 0))
    {
      latency += (accounted_ms-frame_ms);
      if (latency < 0) latency = 0;
      if (latency > max_latency) max_latency = latency;
      
      if (SIMULATE_DROPPED_FRAMES && (accounted_ms > frame_ms))
      {
        std::cout << "simulating frame drop\n";
        {
          boost::unique_lock<boost::mutex> lock(data_mutex);
          KinfuAppRos::dropped_frames++;
        }
        accounted_ms -= frame_ms; if (accounted_ms < 0) accounted_ms = 0;
        goto skip_grab_and_track;
      }
    }
    
    //run KinfuTracker and update moving volume state
    track_ms = timer.getTime();
    has_image = has_data && execKinfu(timestamp); //main algorithm
    updateMovingVolumeVectors();
    track_ms = timer.getTime() - track_ms;
    
    if (single_step) paused = true;

skip_grab_and_track:

    //do any requested scans
    scan_ms = timer.getTime();
    execScan();
    scan_ms = timer.getTime() - scan_ms;
    
    //bubble raycast
    Affine3f bubble_pose = getBubblePose();
    bubble_raycast_ms = timer.getTime();
    execBubbleRaycast(bubble_pose);
    bubble_raycast_ms = timer.getTime() - bubble_raycast_ms;

    //hook for extensions
    exec_ext_ms = timer.getTime();
    execExt();
    exec_ext_ms = timer.getTime() - exec_ext_ms;

    //visualization
    viz_ms = timer.getTime();
    if (viz)
    {
      execViz(has_data, has_image, bubble_pose);

      spinViewers();
      
      if (reset_camera_pending)
      {
        scene_cloud_view->resetViewerPose();
        reset_camera_pending = false;
      }
    }
    viz_ms = timer.getTime() - viz_ms;

    // Calculate accounted time and FPS
    accounted_ms = upload_ms + track_ms + bubble_raycast_ms;
    exec_ms += timer.getTime();
    if (execs && ((execs%FPS_SKIP) == 0))
    {
      //std::cout << "exec " << execs
      //          << ", avg time = " << (exec_ms/FPS_SKIP) << "ms "
      //          << "(" << 1000.f*(FPS_SKIP/exec_ms) << "fps)\n";
      fps = 1000.f*(FPS_SKIP/exec_ms);
      exec_ms = 0;
    }
    execs++;

    // Print status
    if (print_status)
    {
      num_newlines = last_status_num_newlines = printStatus(); 
      pc::print_info("\033[%dA",num_newlines); //VT100 up n lines
      fflush(stdout);
    }
    
    // Publish the topics
    publishROSTopics ();
    
    // Spin ROS nodes
    ros::spinOnce ();

  } //run loop
  
  
  if (print_status && (execs > 0))
    pc::print_info("\033[%dB\n", num_newlines); //VT100 down n lines and newline

  last_status_num_newlines = 0;
  
  if (!triggered) capture->stop();
  
  c.disconnect();

  if (eval) kinfu->saveAllPoses();

  std::cout << "moving volume: max latency " << max_latency << "ms\n";
  kinfu->printMovingVolumeStats();

  quit(true); //signal any other running threads
}