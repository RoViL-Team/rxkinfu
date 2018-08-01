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

#ifndef MVKINFU_KINFUTRACKER_H_
#define MVKINFU_KINFUTRACKER_H_

#include "cuda/device_array.h"
#include "pixel_rgb.h"
#include "tsdf_volume.h"
#include "raycaster.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vector>

namespace rxkinfu
{
  /** \brief KinfuTracker class encapsulates implementation of Microsoft Kinect Fusion algorithm
   * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
   */
  class KinfuTracker
  {
  public:
    /** \brief Pixel type for rendered image. */
    typedef rxkinfu::PixelRGB PixelRGB;
    
    typedef DeviceArray2D<PixelRGB> View;
    typedef DeviceArray2D<unsigned short> DepthMap;
    
    typedef pcl::PointXYZ PointType;
    typedef pcl::Normal NormalType;
    
    /** \brief Number of pyramid levels */
    enum { LEVELS = 3 };
    
    /** \brief ICP Correspondences  map type */
    typedef DeviceArray2D<int> CorespMap;
    
    /** \brief Vertex or Normal Map type */
    typedef DeviceArray2D<float> MapArr;
    
    typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
    typedef Eigen::Vector3f Vector3f;

    /** \brief Constructor
     * \param[in] rows height of depth image
     * \param[in] cols width of depth image
     */
    KinfuTracker (int rows = 480, int cols = 640);
    
    virtual ~KinfuTracker() {};

    TsdfVolume* makeTsdfVolume(int res_x, int res_y, int res_z);

    /** \brief Sets Depth camera intrinsics
     * \param[in] fx focal length x 
     * \param[in] fy focal length y
     * \param[in] cx principal point x
     * \param[in] cy principal point y
     */
    virtual void
      setDepthIntrinsics (float fx, float fy, float cx = -1, float cy = -1);
    
    /** \brief Sets initial camera pose relative to volume coordiante space
     * \param[in] pose Initial camera pose
     */
    virtual void
      setInitalCameraPose (const Eigen::Affine3f& pose);
    
		/** \brief Sets truncation threshold for depth image for ICP step only! This helps 
     *  to filter measurements that are outside tsdf volume. Pass zero to disable the truncation.
     * \param[in] max_icp_distance_ Maximal distance, higher values are reset to zero (means no measurement). 
     */
    virtual void
      setDepthTruncationForICP (float max_icp_distance = 0.f);
    
    /** \brief Sets ICP filtering parameters.
     * \param[in] distThreshold distance.
     * \param[in] sineOfAngle sine of angle between normals.
     */
    virtual void
      setIcpCorespFilteringParams (float distThreshold, float sineOfAngle);
    
    /** \brief Sets integration threshold. TSDF volume is integrated iff a camera movement metric exceedes the threshold value. 
     * The metric represents the following: M = (rodrigues(Rotation).norm() + alpha*translation.norm())/2, where alpha = 1.f (hardcoded constant)
     * \param[in] threshold a value to compare with the metric. Suitable values are ~0.001          
     */
    virtual void
      setCameraMovementThreshold(float threshold = 0.001f);
    
    /** \brief Returns cols passed to ctor */
    int
      cols ();
    
    /** \brief Returns rows passed to ctor */
    int
      rows ();
    
    virtual void createMaps(const DepthMap& depth_raw);

    virtual bool execFirstFrame(const DepthMap& depth_raw, double timestamp);

    virtual bool doICP(Matrix3frm &Rprev, Vector3f &tprev,
                       Matrix3frm &Rcurr, Vector3f &tcurr);

    virtual bool shouldIntegrate
      (const Matrix3frm &Rprev, const Vector3f &tprev,
       const Matrix3frm &Rcurr, const Vector3f &tcurr);

    virtual void doIntegrate(const DepthMap& depth_raw,
                             Matrix3frm &Rcurr, Vector3f &tcurr);

    virtual void doRaycast(Matrix3frm &Rcurr, Vector3f &tcurr);

    /** \brief Processes next frame.
     * \param[in] Depth next frame with values in millimeters
     * \return true if can render 3D view.
     */
    virtual bool execute (const DepthMap& depth, double timestamp = NAN,
                          Eigen::Affine3f* hint=NULL);
    
    /** \brief Returns camera pose at given time, default the last pose
     *
     * For moving volume each camera pose is relative to the volume frame
     * that was in effect at the time.  The global camera pose is given by
     * prepending the relative volume transforms back to time 0.
     *
     * \param[in] time Index of frame for which camera pose is returned.
     * \return camera pose
     */
    virtual void
    getCameraPose (Eigen::Affine3f &cam_pose, int time = -1) const;
    
    /** \brief Returns number of poses including initial */
    virtual size_t
    getNumberOfPoses () const;
    
    /** \brief Returns TSDF volume storage */
    const TsdfVolume& volume() const;
    
    /** \brief Returns TSDF volume storage */
    TsdfVolume& volume();
    
    /** \brief Renders 3D scene to display to human
     * \param[out] view output array with image
     */
    virtual void
    getImage (View& view) const;
    
    //davidjones: MODIFIED FOR COLOR
    /** \brief Renders 3D scene to display to human
      * \param[out] view output array with image
      */
    //void
    //getImage (View& view, const View& color) const;
    
    //davidjones: MODIFIED FOR COLOR
    /** \brief Renders 3D scene to display to human
      * \param[out] view output array with image
      * \param[in] light_source_pose Pose of light source for computing illumination
      */
    //void
    //getImage (View& view, const View& color, const Eigen::Vector3f& light_source_pose) const;
        
    /** \brief Returns point cloud abserved from last camera pose
     * \param[out] cloud output array for points
     */
    virtual void
    getLastFrameCloud (DeviceArray2D<PointType>& cloud) const;
    
    /** \brief Returns point cloud abserved from last camera pose
     * \param[out] normals output array for normals
     */
    virtual void
      getLastFrameNormals (DeviceArray2D<NormalType>& normals) const;
    
    /** \brief Disables ICP forever */
    virtual void
    disableIcp();

    virtual void
    saveAllPoses (int frame_number = -1,
                    const std::string& logfile = "kinfu_poses.txt") const;

    /** Moving volume policies. */
    enum MovingVolumePolicy {
          
      /** No moving volume. */
      FIX_VOLUME,
      
      /** Transform volume as needed to keep camera at initial pose. */
      FIX_CAMERA_IN_VOLUME,

      /**
       * Rotate volume first to keep volume +y direction parallel
       * to a specified down vector, then to orient volume +z as close as
       * possible to a specified forward vector.
       *
       * The volume is also automatically translated to keep the camera at
       * its initial location.
       */
      FIX_DOWN_THEN_FWD_IN_VOLUME,
      
      /**
       * Rotate volume first to keep volume +z direction parallel
       * to a specified forward vector, then to orient volume +y as close
       * as possible to a specified down vector.
       *
       * The volume is also automatically translated to keep the camera at
       * its initial location.
       */
      FIX_FWD_THEN_DOWN_IN_VOLUME,
    };

    /** \brief Set the moving volume policy.
     *
     * The policy may be changed at any time.
     *
     * \param[in] policy the new policy, default FIX_VOLUME
     */
    virtual void
    setMovingVolumePolicy (MovingVolumePolicy policy = FIX_VOLUME);

    virtual int
    getMovingVolumePolicy();

    /** \brief Set the moving volume down vector.
     *
     * The vector may be changed at any time.
     *
     * \param[in] d the new down vector in volume frame, if zero length or
     * NaN then the default (0, 1, 0) is used
     */
    virtual void setMovingVolumeDown (const Eigen::Vector3f &d);

    /** \brief Set the moving volume forward vector.
     *
     * The vector may be changed at any time.
     *
     * \param[in] d the new forward vector in volume frame, if zero length
     * or NaN then the default (0, 0, 1) is used
     */
    virtual void setMovingVolumeFwd (const Eigen::Vector3f &f);
    
    /** \brief Set the moving volume distance threshold.
     *
     * The threshold may be changed at any time.
     *
     * \param[in] t the threshold in meters, default 0.3
     */
    virtual void setMovingVolumeDistanceThresh (float t = 0.3f);

    /** \brief Set the moving volume angle threshold.
     *
     * The threshold may be changed at any time.
     *
     * \param[in] t the threshold in radians, default 0.05
     */
    virtual void setMovingVolumeAngleThresh (float t = 0.05f);

    /** \brief Set a moving volume transform option.
     *
     * \param[in] check first check if nearest neighbor is a sentinel,
     * default false
     */
    virtual void setMovingVolumeCheckNN (bool check = false);

    /** \brief Set a moving volume transform option.
     *
     * \param[in] check check weights for invalids, default false
     */
    virtual void setMovingVolumeCheckValid (bool check = false);

    /** \brief Returns moving volume pose at given time relative to the
     * prior one.
     *
     * The total number of volume poses is the same as the number of
     * camera poses, but there may be fewer distinct volume frames because
     * a new volue frame is defined only when the relative volume pose
     * changes.  When there is no change in volume frame at a given time
     * this function returns identity.
     *
     * \param[in] time Index of input image for which pose is returned, -1 for
     * last (the default).
     *
     * \return the moving volume relative pose
     */
    virtual void
    getRelativeVolumePose (Eigen::Affine3f &rel_vol_pose, int time = -1) const;

    /** \brief Returns timestamp for indicated frame.
     *
     * Timestamps are measured in seconds since the epoch.  The timestamp of the
     * initial camera pose is estimated as 30ms before the timestamp of the
     * first processed image.
     *
     * \param[in] time Index of input image for which timestamp is returned, -1
     * for last (the default).
     *
     * \return the timestamp or NAN if no images have been processed.
     */
    virtual double getTimestamp(int time = -1) const;

    /** \brief Get whether it is the first frame after the reset. */
    virtual bool
    getFirstFrameAfterReset ();
    
    /** Print accumulated statistics about moving volume activity. */
    virtual void printMovingVolumeStats() const;

    virtual void getTimings(float &total_ms_tracker,
                            float &total_ms_volume_xform,
                            float &max_ms_tracker,
                            float &max_ms_volume_xform,
                            float &execute_ms, 
                            float &create_maps_ms,
                            float &icp_ms,
                            float &integrate_ms,
                            float &raycast_ms,
                            float &moving_volume_transform_tsdf_ms,
                            float &moving_volume_shift_tsdf_ms);

    virtual void getStats(int &num_resets, int &num_volume_xforms);

    virtual Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix);

    virtual void
    mergePointNormal(const DeviceArray<pcl::PointXYZ>& cloud, const DeviceArray<pcl::Normal>& normals, DeviceArray<pcl::PointNormal>& output);

  protected:
    
    /** \brief Height of input depth image. */
    int rows_;
    /** \brief Width of input depth image. */
    int cols_;
    /** \brief Frame counter */
    int global_time_;
    
    /** \brief Truncation threshold for depth image for ICP step */
    float max_icp_distance_;
    
    /** \brief Intrinsic parameters of depth camera. */
    float fx_, fy_, cx_, cy_;

    /** \brief Tsdf volume container. */
    TsdfVolume::Ptr tsdf_volume_;
    
    /** \brief Initial camera rotation in volume coo space. */
    Matrix3frm init_Rcam_;
    
    /** \brief Initial camera position in volume coo space. */
    Vector3f   init_tcam_;
    
    /** \brief array with IPC iteration numbers for each pyramid level */
    int icp_iterations_[LEVELS];
    /** \brief distance threshold in correspondences filtering */
    float  distThres_;
    /** \brief angle threshold in correspondences filtering. Represents max sine of angle between normals. */
    float angleThres_;
    
    /** \brief Depth pyramid. */
    std::vector<DepthMap> depths_curr_;
    /** \brief Vertex maps pyramid for current frame in global coordinate space. */
    std::vector<MapArr> vmaps_g_curr_;
    /** \brief Normal maps pyramid for current frame in global coordinate space. */
    std::vector<MapArr> nmaps_g_curr_;
    
    /** \brief Vertex maps pyramid for previous frame in global coordinate space. */
    std::vector<MapArr> vmaps_g_prev_;
    /** \brief Normal maps pyramid for previous frame in global coordinate space. */
    std::vector<MapArr> nmaps_g_prev_;
    
    /** \brief Vertex maps pyramid for current frame in current coordinate space. */
    std::vector<MapArr> vmaps_curr_;
    /** \brief Normal maps pyramid for current frame in current coordinate space. */
    std::vector<MapArr> nmaps_curr_;
    
    /** \brief Buffer for storing scaled depth image */
    DeviceArray2D<float> depthRawScaled_;
    
    /** \brief Temporary buffer for ICP */
    DeviceArray2D<double> gbuf_;
    /** \brief Buffer to store MLS matrix. */
    DeviceArray<double> sumbuf_;
    
    /** \brief Array of camera rotation matrices for each exec. */
    std::vector<Matrix3frm> rmats_;
    
    /** \brief Array of camera translations for each exec. */
    std::vector<Vector3f> tvecs_;

    /** \brief Array of timestamps (seconds since epoch) for each exec. */
    std::vector<double> stamps_;
    
    /** \brief Camera movement threshold. TSDF is integrated iff a camera movement metric exceedes some value. */
    float integration_metric_threshold_;
    
    /** \brief ICP step is completelly disabled. Inly integratio now */
    bool disable_icp_;

    /** \brief timings for last execute() */
    float create_maps_ms_, icp_ms_, integrate_ms_, raycast_ms_, execute_ms_;

    /** The current moving volume policy. */
    MovingVolumePolicy moving_volume_policy_;

    /** Moving volume guidance vector. */
    Vector3f moving_volume_down_, moving_volume_fwd_;

    /** Moving volume threshold. */
    float moving_volume_distance_thresh_, moving_volume_angle_thresh_;

    /** swap buffer for moving volume. */
    DeviceArray2D<int> tsdf_swap_;

    /** Moving volume relative rotation matrices for each moment of time. */
    std::vector<Matrix3frm> rmats_vol_;
    
    /** Moving volume relative translations for each moment of time. */
    std::vector<Vector3f> tvecs_vol_;

    /** are we yet to process the first frame after a reset */
    bool first_frame_after_reset_;

    /** Moving volume stats. */
    int num_resets_, num_volume_xforms_;

    /** Moving volume timings for last execute(). */
    float moving_volume_transform_tsdf_ms_, moving_volume_shift_tsdf_ms_;

    /** Moving volume stats. */
    float total_ms_tracker_, total_ms_volume_xform_;

    /** Moving volume stats. */
    float max_ms_tracker_, max_ms_volume_xform_;
       
    /** Moving volume transform options. */
    bool moving_volume_checknn_, moving_volume_checkvalid_;

    /** Update moving volume.
     *
     * Applies the current moving volume policy to decide if it's time to
     * move the volume.  If so the current camera pose is modified
     * accordingly.
     *
     * \param[in,out] tcurr current camera frame origin in volume
     * \param[in,out] Rcurr current camera frame basis in volume
     */
    void updateMovingVolume (Vector3f& tcurr, Matrix3frm& Rcurr);

    /** \brief Allocates all GPU internal buffers.
     * \param[in] rows_arg
     * \param[in] cols_arg          
     */
    void
      allocateBuffers (int rows_arg, int cols_arg);
    
    /** \brief Performs the tracker reset to initial state. It's used if
     * case of camera tracking fail.
     *
     * \param[in] clear whether to clear all prior moving volume and
     * camera poses, default true
     *
     * \param[in] init_tvol initial moving volume translation relative to
     * the prior volume frame, default (0,0,0)
     *
     * \param[in] init_Rvol initial moving volume rotation relative to the
     * prior volume frame, default identity
     */
    void reset (bool clear = true, //moving volume
                const Vector3f& init_tvol = Vector3f(0,0,0),
                const Matrix3frm& init_Rvol = Matrix3frm::Identity());

  private:
    KinfuTracker(const KinfuTracker& other); // Disable
    KinfuTracker& operator=(const KinfuTracker&); // Disable

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
  };
}

#endif
