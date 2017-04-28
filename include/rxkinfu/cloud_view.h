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
 */

#ifndef MVKINFU_CLOUD_VIEW_H
#define MVKINFU_CLOUD_VIEW_H

#include "kinfu_tracker.h"
#include "marching_cubes.h"
#include "tsdf_volume_host.h"
#include "raycaster.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace rxkinfu {

  class CloudView {
    
  public:
    
    CloudView(const char *name, bool show_cloud);
  
    virtual ~CloudView() {}

    virtual void setViewerPose(const Eigen::Affine3f& pose);

    virtual Eigen::Affine3f getViewerPose();

    virtual void resetViewerPose();

    virtual void showCloud(KinfuTracker &kinfu, const std::string id,
                           int triangle_size = 0);

    virtual void showCloud(RayCaster &raycaster, const std::string id,
                           int triangle_size = 0);

    virtual void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
                           const std::string id,
                           int triangle_size = 0);

    virtual void removeCloud(const std::string id);

    //58deg H, 45deg V
    //kinect: 800mm-4000mm
    //kinect for windows in near mode: 500mm-3000mm
    //carmine 1.08, xtion pro, xtion pro live: 800mm-3500mm
    //carmine 1.09: 350mm-1400mm

    virtual void showCamera(const Eigen::Affine3f &pose,
                            const std::string id = "camera",
                            float near = 0.5f, float far = 5.0f,
                            float axes = 0.2f,
                            float fr = 0, float fg = 1, float fb = 1,
                            int rows = 480, int cols = 640,
                            float fx = 525, float fy = 525,
                            float cx = -1, float cy = -1);

    virtual void removeCamera(const std::string id = "camera");

    virtual void showBox(const Eigen::Affine3f &pose, float *bflrkt = 0,
                         const std::string id = "box",
                         float axes = 0.5f,
                         float br = 1, float bg = 1, float bb = 0);

    virtual void removeBox(const std::string id = "box");

    virtual void showArrow(const Eigen::Vector3f &v,
                           const Eigen::Vector3f &p = Eigen::Vector3f::Zero(),
                           float r = 1, float g = 1, float b = 1,
                           bool display_length = false,
                           const std::string id = "arrow");

    virtual void removeArrow(const std::string id = "arrow");

    virtual bool wasStopped();

    virtual void
    registerKeyboardCallback
    (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), 
     void* cookie = 0);
    
    template<typename T> inline boost::signals2::connection
    registerKeyboardCallback
    (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*),
     T& instance, void* cookie = NULL)
    {
      return cloud_viewer->registerKeyboardCallback(callback, instance, cookie);
    }
    
    /** \brief Mouse callback function.
      *
      * \param[in] e the function that will be registered as a callback for a
      *            mouse event
      * \param[in] cookie user data that is passed to the callback
      */
    virtual void
    registerMouseCallback
    (void (*callback) (const pcl::visualization::MouseEvent&, void*),
     void* cookie = 0);

    template<typename T> inline boost::signals2::connection
    registerMouseCallback
    (void (T::*callback) (const pcl::visualization::MouseEvent&, void*),
     T& instance, void* cookie = NULL)
    {
      return cloud_viewer->registerMouseCallback (callback, instance, cookie);
    }

    /** \brief Point picking callback function.
      *
      * \param[in] e the function that will be registered as a callback for a
      *            point picking event
      * \param[in] cookie user data that is passed to the callback
      */
    virtual void
    registerPointPickingCallback
    (void (*callback) (const pcl::visualization::PointPickingEvent&, void*),
     void* cookie = 0);

    template<typename T> inline boost::signals2::connection
    registerPointPickingCallback
    (void (T::*callback) (const pcl::visualization::PointPickingEvent&, void*),
     T& instance, void* cookie = NULL)
    {
      return cloud_viewer->registerPointPickingCallback (callback, instance, cookie);
    }

    virtual void spinOnce(int time = 1, bool force_redraw = false);

    virtual pcl::visualization::PCLVisualizer::Ptr getVisualizer();

  protected:
    
    pcl::visualization::PCLVisualizer::Ptr cloud_viewer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;

    DeviceArray2D<pcl::PointXYZ> cloud_device;

    std::map<std::string, int> last_triangle_size;

  private:
    CloudView(const CloudView& other); // Disable
    CloudView& operator=(const CloudView&); // Disable
  };

  class SceneCloudView : public CloudView {
    
  public: 
    
    enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };
    
    enum { GPU_CON6 = 0, CPU_CON6 = 1, CPU_CON26 = 2 };
    
    SceneCloudView(bool show_cloud);
    
    virtual ~SceneCloudView() {}

    virtual MarchingCubes* makeMarchingCubes();

    virtual void show(KinfuTracker& kinfu);

    virtual void showMesh(KinfuTracker& kinfu);
    
    virtual void toggleCube(const Eigen::Vector3f& size);
      
    virtual void toggleExtractionMode();
    
    virtual void toggleNormals();
    
    virtual void clearClouds(bool print_message = false);
    
    virtual boost::shared_ptr<pcl::PolygonMesh>
      convertToMesh(const DeviceArray<pcl::PointXYZ>& triangles);

    virtual void writeCloud(int format) const;
  
    template<typename CloudPtr> void
      writeCloudFile(int format, const CloudPtr& cloud) const;
  
    virtual void writeMesh(int format) const;

    virtual void writePolygonMeshFile(int format,
                                      const pcl::PolygonMesh& mesh) const;

  protected:
    
    int extraction_mode;
    bool compute_normals, valid_combined;
    bool cube_added;
    
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr;
    
    DeviceArray<pcl::PointXYZ> cloud_buffer_device;
    DeviceArray<pcl::Normal> normals_device;
    
    pcl::PointCloud<pcl::PointNormal>::Ptr combined_ptr;
    DeviceArray<pcl::PointNormal> combined_device;  
    
    MarchingCubes::Ptr marching_cubes;
    DeviceArray<pcl::PointXYZ> triangles_buffer_device;
    
    boost::shared_ptr<pcl::PolygonMesh> mesh_ptr;

  private:
    SceneCloudView(const SceneCloudView& other); // Disable
    SceneCloudView& operator=(const SceneCloudView&); // Disable
  };
  
  class CurrentCloudView : public CloudView {
    
  public:
    
    CurrentCloudView(bool show_cloud);

    virtual ~CurrentCloudView() {}

    virtual void show(const KinfuTracker& kinfu);

  private:
    CurrentCloudView(const CurrentCloudView& other); // Disable
    CurrentCloudView& operator=(const CurrentCloudView&); // Disable
  };
}

#include "cloud_view.hpp"

#endif
