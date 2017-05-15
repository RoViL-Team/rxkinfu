/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *
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
 */

#define DEF_VOL_SZ_M 3.0f

#include <iostream>
#include <fstream>
#include <algorithm>

#include <pcl/common/time.h>
#include "kinfu_tracker.h"
#include "cuda/device.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

using namespace rxkinfu::device;
using namespace rxkinfu;

using Eigen::AngleAxisf;
using Eigen::Array3f;
using Eigen::Vector3i;
using Eigen::Vector3f;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
KinfuTracker::KinfuTracker (int rows, int cols)
: rows_(rows), cols_(cols)
, global_time_(0)
, max_icp_distance_(0), integration_metric_threshold_(0.f), disable_icp_(false)
, create_maps_ms_(0), icp_ms_(0), integrate_ms_(0), raycast_ms_(0)
, execute_ms_(0)
, moving_volume_policy_(FIX_VOLUME)
, moving_volume_down_(0, 1, 0), moving_volume_fwd_(0, 0, 1)
, moving_volume_distance_thresh_(0.3f), moving_volume_angle_thresh_(0.05f)
, first_frame_after_reset_(true), num_resets_(0), num_volume_xforms_(0)
, moving_volume_transform_tsdf_ms_(0), moving_volume_shift_tsdf_ms_(0)
, total_ms_tracker_(0), total_ms_volume_xform_(0)
, max_ms_tracker_(0), max_ms_volume_xform_(0)
, moving_volume_checknn_(false), moving_volume_checkvalid_(false)
{
  const Vector3f volume_size = Vector3f::Constant(DEF_VOL_SZ_M);
  const Vector3i volume_resolution;
   
  tsdf_volume_ = TsdfVolume::Ptr(makeTsdfVolume(VOLUME_X, VOLUME_Y, VOLUME_Z));
  tsdf_volume_->setSize(volume_size);
  
  setDepthIntrinsics (525.f, 525.f); // default values, can be overwritten
  
  init_Rcam_ = Eigen::Matrix3f::Identity ();// * AngleAxisf(-30.f/180*3.1415926, Vector3f::UnitX());
  init_tcam_ = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

  const int iters[] = {10, 5, 4};
  std::copy (iters, iters + LEVELS, icp_iterations_);

  const float default_distThres = 0.10f; //meters
  const float default_angleThres = sin (20.f * 3.14159254f / 180.f);
  const float default_trunc_dist = 0.03f; //meters

  setIcpCorespFilteringParams (default_distThres, default_angleThres);
  tsdf_volume_->setTsdfTruncDist (default_trunc_dist);

  allocateBuffers (rows, cols);

  rmats_.reserve (30000);
  tvecs_.reserve (30000);
  stamps_.reserve (30000);

  rmats_vol_.reserve (30000);
  tvecs_vol_.reserve (30000);

  reset ();
}

TsdfVolume* KinfuTracker::makeTsdfVolume(int res_x, int res_y, int res_z) {
  tsdf_swap_.create (res_y * res_z, res_x); //TBD
  const Vector3i res(res_x, res_y, res_z);
  return new TsdfVolume(res);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::setDepthIntrinsics (float fx, float fy, float cx, float cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = (cx == -1) ? cols_/2-0.5f : cx;
  cy_ = (cy == -1) ? rows_/2-0.5f : cy;  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::setInitalCameraPose (const Eigen::Affine3f& pose)
{
  init_Rcam_ = pose.rotation ();
  init_tcam_ = pose.translation ();
  reset ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::setDepthTruncationForICP (float max_icp_distance)
{
  max_icp_distance_ = max_icp_distance;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::setCameraMovementThreshold(float threshold)
{
  integration_metric_threshold_ = threshold;  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::setIcpCorespFilteringParams (float distThreshold, float sineOfAngle)
{
  distThres_  = distThreshold; //mm
  angleThres_ = sineOfAngle;
}

void KinfuTracker::setMovingVolumePolicy (MovingVolumePolicy policy)
{ moving_volume_policy_ = policy; }

int KinfuTracker::getMovingVolumePolicy()
{ return moving_volume_policy_; }

void KinfuTracker::setMovingVolumeDown (const Vector3f &d)
{ moving_volume_down_ = d; }

void KinfuTracker::setMovingVolumeFwd (const Vector3f &f)
{ moving_volume_fwd_ = f; }

void KinfuTracker::setMovingVolumeDistanceThresh (float t)
{ moving_volume_distance_thresh_ = t; }

void KinfuTracker::setMovingVolumeAngleThresh (float t)
{ moving_volume_angle_thresh_ = t; }

void KinfuTracker::setMovingVolumeCheckNN (bool check)
{ moving_volume_checknn_ = check; }

void KinfuTracker::setMovingVolumeCheckValid (bool check)
{ moving_volume_checkvalid_ = check; }

Eigen::Affine3f KinfuTracker::getRelativeVolumePose (int time) const {

  if (rmats_.size() != rmats_vol_.size())
    std::cerr << "ERROR rmats_ and rmats_vol_ differ in size\n";

  if (tvecs_.size() != tvecs_vol_.size())
    std::cerr << "ERROR tvecs_ and tvecs_vol_ differ in size\n";

  if (time > static_cast<int>(rmats_vol_.size ()) || time < 0)
    time = rmats_vol_.size () - 1;

  Eigen::Affine3f aff = Eigen::Affine3f::Identity();

  if (time >= 0) {
    aff.linear () = rmats_vol_[time];
    aff.translation () = tvecs_vol_[time];
  }

  return (aff);
}

////////////////////////////////////////////////////////////////////////////////
bool
KinfuTracker::getFirstFrameAfterReset ()
{
  return (first_frame_after_reset_);
}

double KinfuTracker::getTimestamp(int time) const {
  if (time > static_cast<int>(stamps_.size()) || time < 0)
    time = stamps_.size() - 1;
  if (time > 0) return stamps_[time];
  else if (stamps_.size() > 1) return stamps_[1]-0.030;
  else return stamps_[0];
}

void KinfuTracker::printMovingVolumeStats () const {
  std::cout
    << "moving molume " 
    << "distance threshold = " << moving_volume_distance_thresh_
    << "m, angle threshold = " << moving_volume_angle_thresh_ << "rad\n";
  
  std::cout
    << "moving volume " 
    << "checknn = " << moving_volume_checknn_
    << ", checkvalid = " << moving_volume_checkvalid_ << std::endl;

  std::cout
    << "moving volume "
    << global_time_ << " frames, tracker avg "
    << total_ms_tracker_/static_cast<float>(global_time_) << "ms, max "
    << max_ms_tracker_ << "ms\n";

  std::cout
    << "moving volume "
    << num_volume_xforms_ << " volume transforms, avg "
    << total_ms_volume_xform_/static_cast<float>(num_volume_xforms_)
    << "ms, max " << max_ms_volume_xform_ << "ms\n";

  std::cout << num_resets_ << " resets\n";
}

void KinfuTracker::getTimings(float &total_ms_tracker,
                              float &total_ms_volume_xform,
                              float &max_ms_tracker, float &max_ms_volume_xform,
                              float &execute_ms, float &create_maps_ms,
                              float &icp_ms, float &integrate_ms,
                              float &raycast_ms,
                              float &moving_volume_transform_tsdf_ms,
                              float &moving_volume_shift_tsdf_ms) {
  total_ms_tracker = total_ms_tracker_;
  total_ms_volume_xform = total_ms_volume_xform_;
  max_ms_tracker = max_ms_tracker_; max_ms_volume_xform = max_ms_volume_xform_;
  execute_ms = execute_ms_; create_maps_ms = create_maps_ms_; icp_ms = icp_ms_;
  integrate_ms = integrate_ms_; raycast_ms = raycast_ms_;
  moving_volume_transform_tsdf_ms = moving_volume_transform_tsdf_ms_;
  moving_volume_shift_tsdf_ms = moving_volume_shift_tsdf_ms_;
}

void KinfuTracker::getStats(int &num_resets, int &num_volume_xforms) {
  num_resets = num_resets_;
  num_volume_xforms = num_volume_xforms_;
}

void KinfuTracker::updateMovingVolume (Vector3f& tcurr, Matrix3frm& Rcurr) {

  //resolve down and fwd vectors ///////////////////////////////////////////////
  Vector3f down = moving_volume_down_;
  if (((down.array() != down.array()).any()) || //any nan?
      (down.norm() < Eigen::NumTraits<float>::epsilon()))
    down = Vector3f(0, 1, 0);
  down = down.normalized();
  
  Vector3f fwd = moving_volume_fwd_;
  if (((fwd.array() != fwd.array()).any()) || //any nan?
      (fwd.norm() < Eigen::NumTraits<float>::epsilon()))
    fwd = Vector3f(0, 0, 1);
  fwd = fwd.normalized();
  
  //apply policy to determine ideal volume transform ///////////////////////////

  //[Rvol|tvol] will be the transform from the new volume frame to the
  //old volume frame (identity if no change)
  Vector3f tvol = Vector3f(0,0,0);
  Matrix3frm Rvol = Matrix3frm::Identity ();

  switch (moving_volume_policy_) {

  case FIX_VOLUME: break;

  case FIX_CAMERA_IN_VOLUME: {

    //factor current camera pose into the initial camera transform followed
    //by the volume transform
    //
    //curr = vol * init;
    //vol = curr * init^-1;
    //[Rvol|tvol] = [Rcurr|tcurr]*[init_Rcam_^T|-init_Rcam_^T*init_tcam_]
    Rvol = Rcurr * init_Rcam_.inverse();
    tvol = tcurr - Rvol * init_tcam_;

    break;
  }

  case FIX_DOWN_THEN_FWD_IN_VOLUME: {

    Vector3f d = down.normalized(), f = fwd.normalized(), r = d.cross(f);

    if (r.norm() > 0.01) {

      r = r.normalized();

      Rvol.col(0) = r;          //x = right
      Rvol.col(1) = d;          //y = down
      Rvol.col(2) = r.cross(d); //z = forward

      //factor current camera pose into a new camera pose
      //
      //curr' = [Rcurr'|init_tcam_]
      //
      //followed by the volume transform
      //
      //[Rcurr|tcurr] = [Rvol|tvol] * [Rcurr'|init_tcam_];
      //
      //the two unknowns are the translation part of the volume transfrom tvol
      //and the rotation part of the new camera pose Rcurr'
      //
      //Rcurr = Rvol * Rcurr'            -> Rcurr' = Rvol^T * Rcurr
      //tcurr = Rvol * init_tcam_ + tvol ->   tvol = tcurr - Rvol * init_tcam_
      tvol = tcurr - Rvol * init_tcam_;
    }

    break;
 }

  case FIX_FWD_THEN_DOWN_IN_VOLUME: {
    
    Vector3f d = down.normalized(), f = fwd.normalized(), r = d.cross(f);

    if (r.norm() > 0.01) {

      r = r.normalized();

      Rvol.col(0) = r;          //x = right
      Rvol.col(1) = f.cross(r); //y = down
      Rvol.col(2) = f;          //z = forward

      tvol = tcurr - Rvol * init_tcam_;
    }

    break;
  }
  } //switch moving_volume_policy_

  //move the volume iff needed /////////////////////////////////////////////////

  pcl::StopWatch timer;

  //linear and angular volume differences
  float ld = tvol.norm(), ad = rodrigues2(Rvol).norm();

  //voxel dimensions in physical units
  Vector3f voxel_size = tsdf_volume_->getVoxelSize();
  
  bool moved = false;

  moving_volume_transform_tsdf_ms_ = moving_volume_shift_tsdf_ms_ = 0;

  if ((ad > moving_volume_angle_thresh_) ||
      ((ld > moving_volume_distance_thresh_) &&
       (voxel_size.array() > moving_volume_distance_thresh_).any())) {

    pcl::StopWatch timer;

    //need to perform a full interpolating transform because rotation exceedeed
    //threshold or because translation exceeded threshold that was smaller than
    //a voxel dimension

    //convert translation to fractional grid units
    Vector3f trans = tvol.cwiseQuotient(voxel_size);

    float3& device_trans = device_cast<float3>(trans);
    Mat33& device_rot = device_cast<Mat33>(Rvol);  
    DeviceArray2D<int> tsdf_data = tsdf_volume_->data();
    transformTsdfVolume(device_trans, device_rot, tsdf_data, tsdf_swap_,
                        moving_volume_checknn_, moving_volume_checkvalid_);
    tsdf_volume_->swapData(tsdf_swap_);

    moved = true;
   
    moving_volume_transform_tsdf_ms_ = static_cast<float>(timer.getTime());

  } else if (ld > moving_volume_distance_thresh_) {

    pcl::StopWatch timer;

    //a memory copy volume shift will suffice because rotation is within
    //threshold and translation threshold is greater than or equal to all voxel
    //dimensions

    //convert translation to integer grid units
    Vector3f shift = tvol.cwiseQuotient(voxel_size);
    for (int i = 0; i < 3; i++)
      shift(i) = ((shift(i) < 0) ? ceil(shift(i)) : floor(shift(i)));

    Vector3i trans = shift.cast<int>();
    int3& device_trans = device_cast<int3>(trans);
    DeviceArray2D<int> tsdf_data = tsdf_volume_->data();
    shiftTsdfVolume(device_trans, tsdf_data, tsdf_swap_);
    tsdf_volume_->swapData(tsdf_swap_);

    //update [Rvol|tvol] for the actual transform that was applied
    tvol = shift.cwiseProduct(voxel_size);
    Rvol = Matrix3frm::Identity();

    moved = true;

    moving_volume_shift_tsdf_ms_ = static_cast<float>(timer.getTime());

  } else {

    //angle and distance both below threshold, no volume change 
    tvol = Vector3f(0, 0, 0);
    Rvol = Matrix3frm::Identity();
  }

  //fixup camera pose and do accounting if the volume moved ////////////////////
  
  if (moved) {
    
    //factor current cam pose into a new cam pose followed by the volume xform
    //
    //curr = vol * curr';
    //curr' = vol^-1 * curr;
    //
    //[Rcurr'|tcurr'] = [Rvol^T|-Rvol^T*tvol]*[Rcurr|tcurr]
    tcurr = Rvol.inverse() * (tcurr - tvol);
    Rcurr = Rvol.inverse() * Rcurr; 
    
    num_volume_xforms_++;
    float ms = static_cast<float>(timer.getTime());
    if (ms > max_ms_volume_xform_) max_ms_volume_xform_ = ms;
    total_ms_volume_xform_ += ms;
  }
  
  //always save volume transform even if identity so there is 1:1
  //correspondence between volume and camera transforms
  rmats_vol_.push_back (Rvol);
  tvecs_vol_.push_back (tvol);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
KinfuTracker::cols ()
{
  return (cols_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
KinfuTracker::rows ()
{
  return (rows_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::reset (bool clear,
                     const Vector3f& init_tvol, const Matrix3frm& init_Rvol)
{

  if (clear) {
    global_time_ = 0;
    rmats_.clear ();
    tvecs_.clear ();
    stamps_.clear ();
    rmats_vol_.clear ();
    tvecs_vol_.clear ();
    num_resets_ = 0;
    num_volume_xforms_ = 0;
    total_ms_tracker_ = 0;
    total_ms_volume_xform_ = 0;
    max_ms_tracker_ = 0;
    max_ms_volume_xform_ = 0;
  } else {
//    std::cout << "reset at time " << global_time_ <<  std::endl;
    num_resets_++;
  }

  rmats_.push_back (init_Rcam_);
  tvecs_.push_back (init_tcam_);
  stamps_.push_back (NAN);

  tsdf_volume_->reset();
  initVolume(tsdf_swap_);

  rmats_vol_.push_back (init_Rvol);
  tvecs_vol_.push_back (init_tvol);

  first_frame_after_reset_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::allocateBuffers (int rows, int cols)
{    
  depths_curr_.resize (LEVELS);
  vmaps_g_curr_.resize (LEVELS);
  nmaps_g_curr_.resize (LEVELS);

  vmaps_g_prev_.resize (LEVELS);
  nmaps_g_prev_.resize (LEVELS);

  vmaps_curr_.resize (LEVELS);
  nmaps_curr_.resize (LEVELS);

  for (int i = 0; i < LEVELS; ++i)
  {
    int pyr_rows = rows >> i;
    int pyr_cols = cols >> i;

    depths_curr_[i].create (pyr_rows, pyr_cols);

    vmaps_g_curr_[i].create (pyr_rows*3, pyr_cols);
    nmaps_g_curr_[i].create (pyr_rows*3, pyr_cols);

    vmaps_g_prev_[i].create (pyr_rows*3, pyr_cols);
    nmaps_g_prev_[i].create (pyr_rows*3, pyr_cols);

    vmaps_curr_[i].create (pyr_rows*3, pyr_cols);
    nmaps_curr_[i].create (pyr_rows*3, pyr_cols);
  }  
  depthRawScaled_.create (rows, cols);
  // see estimate transform for the magic numbers
  gbuf_.create (27, 20*60);
  sumbuf_.create (27);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void KinfuTracker::createMaps(const DepthMap& depth_raw) {

  pcl::StopWatch timer;

  //depth_raw.copyTo(depths_curr[0]);
  bilateralFilter(depth_raw, depths_curr_[0]);
  
  if (max_icp_distance_ > 0) truncateDepth(depths_curr_[0], max_icp_distance_);
  
  for (int i = 1; i < LEVELS; ++i) pyrDown(depths_curr_[i-1], depths_curr_[i]);
 
  Intr intr(fx_, fy_, cx_, cy_);

  for (int i = 0; i < LEVELS; ++i) {
    createVMap(intr(i), depths_curr_[i], vmaps_curr_[i]);
    //createNMap(vmaps_curr_[i], nmaps_curr_[i]);
    computeNormalsEigen (vmaps_curr_[i], nmaps_curr_[i]);
  }

  device::sync();

  create_maps_ms_ = static_cast<float>(timer.getTime());
}

bool KinfuTracker::execFirstFrame(const DepthMap& depth_raw, double timestamp) {

  //[Ri|ti] - pos of camera, i.e.
  //transform from camera to global coo space for (i-1)th camera pose
  Matrix3frm init_Rcam = rmats_[0];
  Vector3f   init_tcam = tvecs_[0]; 
  
  Mat33& device_Rcam = device_cast<Mat33>(init_Rcam);
  float3& device_tcam = device_cast<float3>(init_tcam);

  Matrix3frm init_Rcam_inv = init_Rcam.inverse();
  Mat33& device_Rcam_inv = device_cast<Mat33>(init_Rcam_inv);

  //integrateTsdfVolume(depth_raw, Intr(fx_, fy_, cx_, cy_), 
  //                    device_cast<const float3>(tsdf_volume_->getSize()),
  //                    device_Rcam_inv, device_tcam, trunc_dist, volume_);    
  integrateTsdfVolume(depth_raw, Intr(fx_, fy_, cx_, cy_), 
                      device_cast<const float3>(tsdf_volume_->getSize()),
                      device_Rcam_inv, device_tcam,
                      tsdf_volume_->getTsdfTruncDist(),
                      tsdf_volume_->data(), depthRawScaled_);

  for (int i = 0; i < LEVELS; ++i)
    transformMaps(vmaps_curr_[i], nmaps_curr_[i], device_Rcam, device_tcam,
                  vmaps_g_prev_[i], nmaps_g_prev_[i]);
  
  if (global_time_ > 0) { //after a reset
    rmats_.push_back(init_Rcam_);
    tvecs_.push_back(init_tcam_);
    rmats_vol_.push_back(Matrix3frm::Identity());
    tvecs_vol_.push_back(Vector3f::Zero());
    stamps_.push_back(timestamp);
  }
  
  ++global_time_;

  return false;
}

bool KinfuTracker::doICP(Matrix3frm &Rprev, Vector3f &tprev,
                         Matrix3frm &Rcurr, Vector3f &tcurr) {

  pcl::StopWatch timer;
  
  Matrix3frm Rprev_inv = Rprev.inverse(); //Rprev.t();
  
  //Mat33& device_Rprev = device_cast<Mat33> (Rprev);
  Mat33&  device_Rprev_inv = device_cast<Mat33>(Rprev_inv);
  float3& device_tprev = device_cast<float3>(tprev);
  
  for (int level_index = LEVELS-1; level_index>=0; --level_index) {
    
    int iter_num = icp_iterations_[level_index];
    
    MapArr& vmap_curr = vmaps_curr_[level_index];
    MapArr& nmap_curr = nmaps_curr_[level_index];
    
    //MapArr& vmap_g_curr = vmaps_g_curr_[level_index];
    //MapArr& nmap_g_curr = nmaps_g_curr_[level_index];
    
    MapArr& vmap_g_prev = vmaps_g_prev_[level_index];
    MapArr& nmap_g_prev = nmaps_g_prev_[level_index];
    
    for (int iter = 0; iter < iter_num; ++iter) {
      
      Mat33&  device_Rcurr = device_cast<Mat33>(Rcurr);
      float3& device_tcurr = device_cast<float3>(tcurr);
      
      Eigen::Matrix<double, 6, 6, Eigen::RowMajor> A;
      Eigen::Matrix<double, 6, 1> b;
     
      Intr intr(fx_, fy_, cx_, cy_);

      estimateCombined(device_Rcurr, device_tcurr, vmap_curr, nmap_curr,
                       device_Rprev_inv, device_tprev, intr(level_index),
                       vmap_g_prev, nmap_g_prev, distThres_, angleThres_,
                       gbuf_, sumbuf_, A.data(), b.data());
      
      //checking nullspace
      double det = A.determinant();
      
      if (fabs (det) < 1e-15 || pcl_isnan(det)) {
        
        if (pcl_isnan(det)) std::cout << "qnan" << std::endl;
        
        //reset (); with moving volume do a reset but don't clear global_time_,
        //rmats_, tvecs_, rmats_vol_, or tvecs_vol_.  And instead of pushing the
        //identity transform onto the latter two push a relative volume
        //transform based on the the previous good camera pose
        
        //factor prev like this
        //prev = vol * init;
        //vol = prev * init^-1;
        //[Rvol|tvol] = [Rprev|tprev]*[init_Rcam_^T|-init_Rcam_^T*init_tcam_]
        reset(false,  //don't clear
              -Rprev * init_Rcam_.inverse() * init_tcam_ + tprev,
              Rprev * init_Rcam_.inverse());
        
        ++global_time_; //and count this as a frame
        
        return false;
      }
      
      //float maxc = A.maxCoeff();
      
      Eigen::Matrix<float, 6, 1> result = A.llt().solve(b).cast<float>();
      //Eigen::Matrix<float, 6, 1> result = 
      //A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
      
      float alpha = result(0);
      float beta  = result(1);
      float gamma = result(2);
      
      Eigen::Matrix3f Rinc =
        (Eigen::Matrix3f)
        AngleAxisf(gamma, Vector3f::UnitZ()) *
        AngleAxisf(beta, Vector3f::UnitY()) *
        AngleAxisf(alpha, Vector3f::UnitX());
      
      Vector3f tinc = result.tail<3>();
      
      //compose
      tcurr = Rinc * tcurr + tinc;
      Rcurr = Rinc * Rcurr;
    }
  }

  icp_ms_ = static_cast<float>(timer.getTime());

  return true;
}

bool
KinfuTracker::shouldIntegrate(const Matrix3frm &Rprev, const Vector3f &tprev,
                              const Matrix3frm &Rcurr, const Vector3f &tcurr) {
  float rnorm = rodrigues2(Rcurr.inverse() * Rprev).norm();
  float tnorm = (tcurr - tprev).norm();  
  const float alpha = 1.f;
  return (rnorm + alpha * tnorm)/2 >= integration_metric_threshold_;
}

void KinfuTracker::doIntegrate(const DepthMap& depth_raw,
                               Matrix3frm &Rcurr, Vector3f &tcurr) {

  pcl::StopWatch timer;

  Matrix3frm Rcurr_inv = Rcurr.inverse();

  //integrateTsdfVolume(depth_raw, Intr(fx_, fy_, cx_, cy_), 
  //                    device_cast<const float3>(tsdf_volume_->getSize()),
  //                    device_cast<Mat33>(Rcurr_inv),
  //                    device_cast<float3>(tcurr),
  //                    trunc_dist, volume_);
  integrateTsdfVolume(depth_raw, Intr(fx_, fy_, cx_, cy_), 
                      device_cast<const float3>(tsdf_volume_->getSize()),
                      device_cast<Mat33>(Rcurr_inv), device_cast<float3>(tcurr),
                      tsdf_volume_->getTsdfTruncDist(),
                      tsdf_volume_->data(), depthRawScaled_);

  integrate_ms_ = static_cast<float>(timer.getTime());
}

void KinfuTracker::doRaycast(Matrix3frm &Rcurr, Vector3f &tcurr) {

  pcl::StopWatch timer;

  raycast(Intr(fx_, fy_, cx_, cy_),
          device_cast<Mat33>(Rcurr), device_cast<float3>(tcurr),
          tsdf_volume_->getTsdfTruncDist(), 
          device_cast<const float3>(tsdf_volume_->getSize()),
          tsdf_volume_->data(), vmaps_g_prev_[0], nmaps_g_prev_[0]);
  
  for (int i = 1; i < LEVELS; ++i) {
    resizeVMap(vmaps_g_prev_[i-1], vmaps_g_prev_[i]);
    resizeNMap(nmaps_g_prev_[i-1], nmaps_g_prev_[i]);
  }
  
  device::sync();

  raycast_ms_ = static_cast<float>(timer.getTime());
}

bool
KinfuTracker::execute(const DepthMap& depth_raw, double timestamp,
                      Eigen::Affine3f *hint) {  

  pcl::StopWatch timer;

  //Bilateral filter, pyrdown, create vertex and normal maps
  createMaps(depth_raw);

  //Can't perform more on first frame
  if (first_frame_after_reset_) {
    first_frame_after_reset_ = false;
    return execFirstFrame(depth_raw, timestamp);
  }
 
  assert(global_time_ > 0);

  //[Ri|ti] - pos of camera, i.e.
  //transfrom from camera to global coo space for (i-1)th camera pose
  Matrix3frm Rprev = rmats_[global_time_ - 1]; 
  Vector3f tprev = tvecs_[global_time_ - 1]; 

  //transform to global coo for ith camera pose
  Matrix3frm Rcurr = (hint) ? hint->rotation().matrix() : Rprev;
  Vector3f tcurr = (hint) ? hint->translation().matrix() : tprev;

  //Iterative Closest Point
  if (!disable_icp_ && !doICP(Rprev, tprev, Rcurr, tcurr)) return false;

  //Integration check - We do not integrate volume if camera does not move.  
  bool integrate = disable_icp_ || shouldIntegrate(Rprev, tprev, Rcurr, tcurr);

  //Moving volume (may mutate camera pose)
  updateMovingVolume(tcurr, Rcurr);
 
  //Save new camera pose
  rmats_.push_back(Rcurr);
  tvecs_.push_back(tcurr);
  stamps_.push_back(timestamp);

  //Volume integration
  if (integrate) doIntegrate(depth_raw, Rcurr, tcurr);

  //Ray casting
  doRaycast(Rcurr, tcurr);

  execute_ms_ = static_cast<float>(timer.getTime());

  float ms = static_cast<float>(execute_ms_);
  if (ms > max_ms_tracker_) max_ms_tracker_ = ms;
  total_ms_tracker_ += ms;

  ++global_time_;

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f
KinfuTracker::getCameraPose (int time) const
{
  if (time > (int)rmats_.size () || time < 0)
    time = rmats_.size () - 1;

  Eigen::Affine3f aff;
  aff.linear () = rmats_[time];
  aff.translation () = tvecs_[time];
  return (aff);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

size_t
KinfuTracker::getNumberOfPoses () const
{
  return rmats_.size();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const TsdfVolume& 
KinfuTracker::volume() const 
{ 
  return *tsdf_volume_; 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TsdfVolume& 
KinfuTracker::volume()
{
  return *tsdf_volume_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::getImage (View& view) const
{
  //Eigen::Vector3f light_source_pose = tsdf_volume_->getSize() * (-3.f);
  Eigen::Vector3f light_source_pose = tvecs_[tvecs_.size () - 1];

  LightSource light;
  light.number = 1;
  light.pos[0] = device_cast<const float3>(light_source_pose);

  view.create (rows_, cols_);
  generateImage (vmaps_g_prev_[0], nmaps_g_prev_[0], light, view);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::getLastFrameCloud (DeviceArray2D<PointType>& cloud) const
{
  cloud.create (rows_, cols_);
  DeviceArray2D<float4>& c = (DeviceArray2D<float4>&)cloud;
  device::convert (vmaps_g_prev_[0], c);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
KinfuTracker::getLastFrameNormals (DeviceArray2D<NormalType>& normals) const
{
  normals.create (rows_, cols_);
  DeviceArray2D<float8>& n = (DeviceArray2D<float8>&)normals;
  device::convert (nmaps_g_prev_[0], n);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
KinfuTracker::disableIcp() { disable_icp_ = true; }

static void writePose(std::ofstream& stream,
                      double stamp, Eigen::Affine3f& pose) {
    Eigen::Quaternionf q(pose.rotation());
    Eigen::Vector3f t = pose.translation();
    stream << stamp << " ";
    stream << t[0] << " " << t[1] << " " << t[2] << " ";
    stream << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

void KinfuTracker::saveAllPoses(int frame_number,
                                const std::string& logfile) const {   

  size_t total = getNumberOfPoses();

  if (frame_number < 0) frame_number = (int)total;

  frame_number = std::min(frame_number, (int)total);

  std::string vol_logfile = logfile;
  vol_logfile.insert(logfile.rfind('.'), "_vol");

  std::cout << "Writing " << frame_number
            << " absolute camera poses to " << logfile <<  std::endl;
  
  std::cout << "Writing " << frame_number
            << " (rel vol pose, camera in volume pose) pairs to "
            << vol_logfile <<  std::endl;
  
  std::ofstream path_file_stream(logfile.c_str());
  path_file_stream.setf(std::ios::fixed,std::ios::floatfield);
  
  std::ofstream path_vol_file_stream(vol_logfile.c_str());
  path_vol_file_stream.setf(std::ios::fixed,std::ios::floatfield);

  Eigen::Affine3f vol_pose(Eigen::Affine3f::Identity());
  for(int i = 0; i < frame_number; ++i) {
    double stamp = getTimestamp(i);
    Eigen::Affine3f camera_pose = getCameraPose(i);
    Eigen::Affine3f rel_vol_pose = getRelativeVolumePose(i);
    vol_pose = vol_pose * rel_vol_pose;
    Eigen::Affine3f pose = vol_pose * camera_pose;
    writePose(path_file_stream, stamp, pose);
    writePose(path_vol_file_stream, stamp, rel_vol_pose);
    writePose(path_vol_file_stream, stamp, camera_pose);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3f KinfuTracker::rodrigues2(const Eigen::Matrix3f& matrix)
{
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);    
  Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();
  
  double rx = R(2, 1) - R(1, 2);
  double ry = R(0, 2) - R(2, 0);
  double rz = R(1, 0) - R(0, 1);
  
  double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
  double c = (R.trace() - 1) * 0.5;
  c = c > 1. ? 1. : c < -1. ? -1. : c;
  
  double theta = acos(c);
  
  if( s < 1e-5 )
    {
      double t;
      
      if( c > 0 )
        rx = ry = rz = 0;
      else
        {
          t = (R(0, 0) + 1)*0.5;
          rx = sqrt( std::max(t, 0.0) );
          t = (R(1, 1) + 1)*0.5;
          ry = sqrt( std::max(t, 0.0) ) * (R(0, 1) < 0 ? -1.0 : 1.0);
          t = (R(2, 2) + 1)*0.5;
          rz = sqrt( std::max(t, 0.0) ) * (R(0, 2) < 0 ? -1.0 : 1.0);
          
          if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0) )
            rz = -rz;
          theta /= sqrt(rx*rx + ry*ry + rz*rz);
          rx *= theta;
          ry *= theta;
          rz *= theta;
        }
    }
  else
    {
      double vth = 1/(2*s);
      vth *= theta;
      rx *= vth; ry *= vth; rz *= vth;
    }
  return Eigen::Vector3d(rx, ry, rz).cast<float>();
}

void KinfuTracker::mergePointNormal(const DeviceArray<pcl::PointXYZ>& cloud, const DeviceArray<pcl::Normal>& normals, DeviceArray<pcl::PointNormal>& output)
{
  const size_t size = std::min(cloud.size(), normals.size());
  output.create(size);
  
  const DeviceArray<float4>& c = (const DeviceArray<float4>&)cloud;
  const DeviceArray<float8>& n = (const DeviceArray<float8>&)normals;
  const DeviceArray<float12>& o = (const DeviceArray<float12>&)output;
  device::mergePointNormal(c, n, o);           
}

