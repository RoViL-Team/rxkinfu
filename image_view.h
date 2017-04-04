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

#ifndef MVKINFU_IMAGE_VIEW_H
#define MVKINFU_IMAGE_VIEW_H

#include "kinfu_tracker.h"
#include "raycaster.h"

#include <pcl/visualization/image_viewer.h>

namespace rxkinfu {

  class ImageView {
    
  public:
    
    ImageView(bool show_raycast, bool show_depth);
    
    virtual ~ImageView() {}
   
    virtual RayCaster* makeRayCaster(int rows, int cols);

    virtual void show(KinfuTracker& kinfu, Eigen::Affine3f* pose = 0);
    
    virtual void showDepth(const PtrStepSz<const unsigned short>& depth);
  
    virtual void showGeneratedDepth(KinfuTracker& kinfu,
                                    const Eigen::Affine3f& pose);

    virtual bool wasStopped();

    virtual void registerKeyboardCallback
      (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), 
       void* cookie = 0);

    template<typename T> inline void
      registerKeyboardCallback
      (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), T& instance, void* cookie = NULL) {
      if (scene_viewer)
        scene_viewer->registerKeyboardCallback(callback, instance, cookie);
      if (depth_viewer)
        depth_viewer->registerKeyboardCallback(callback, instance, cookie);
    }

    virtual void spinOnce(int time = 1, bool force_redraw = false);

    RayCaster::Ptr getRayCaster() { return raycaster; }

  protected:
    
    pcl::visualization::ImageViewer::Ptr scene_viewer, depth_viewer;
    
    KinfuTracker::View view_device;
    std::vector<KinfuTracker::PixelRGB> view_host;
    
    RayCaster::Ptr raycaster;

  private:
    ImageView(const ImageView& other); // Disable
    ImageView& operator=(const ImageView&); // Disable
  };
}

#endif
