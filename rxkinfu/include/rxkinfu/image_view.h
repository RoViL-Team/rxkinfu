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
 *  Author: Marsette Vona, Dimitrios Kanoulas (dkanoulas@gmail.com)
 */

#ifndef RXKINFU_IMAGE_VIEW_H
#define RXKINFU_IMAGE_VIEW_H

#include "kinfu_tracker.h"
#include "raycaster.h"

#include <pcl/visualization/image_viewer.h>

namespace rxkinfu
{
  /** \brief Visualization of images for the ray tracing or depth.
    *
    * \author Anatoly Baskeheev, Itseez Ltd, Marsette Vona, Dimitrios Kanoulas
    */
  class ImageView
  {
    public:
      /** \brief Constructor.
        *
        * \param[in] show_raycast whether to show the raycasted image
        * \param[in] show_depth whether to show the depth image
        */
      ImageView (bool show_raycast, bool show_depth);
      
      /** \brief Empty destructor. */
      virtual ~ImageView () {}
      
      /** \brief Generate a rows-by-cols RayCaster object.
        *
        * \param[in] rows the row size of the raycaster
        * \param[in] cols the row size of the raycaster
        * 
        * \return pointer to the RayCaster object
        */
      virtual RayCaster*
      makeRayCaster (int rows, int cols);
      
      /** \brief */
      //virtual void
      //show (KinfuTracker& kinfu, KinfuTracker::View color_device_in, Eigen::Affine3f* pose = 0); //davidjones: MODIFIED FOR COLOR
      virtual void
      show (KinfuTracker& kinfu, Eigen::Affine3f* pose = 0);
      
      /** \brief Show the depth image.
        *
        * \param[in] the depth image
        */
      virtual void
      showDepth (const PtrStepSz<const unsigned short>& depth);
      
      /** \brief */
      virtual void
      showGeneratedDepth (KinfuTracker& kinfu, const Eigen::Affine3f& pose);
      
      /** \brief */
      virtual bool
      wasStopped ();
      
      /** \brief */
      virtual void
      registerKeyboardCallback
        (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), 
         void* cookie = 0);
      
      /** \brief */
      template<typename T> inline void
      registerKeyboardCallback
        (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*),
                              T& instance, void* cookie = NULL)
      {
        if (scene_viewer)
          scene_viewer->registerKeyboardCallback (callback, instance, cookie);
        if (depth_viewer)
          depth_viewer->registerKeyboardCallback (callback, instance, cookie);
      }
      
      /** \brief */
      virtual void
      spinOnce (int time = 1, bool force_redraw = false);
      
      /** \brief */
      RayCaster::Ptr
      getRayCaster() { return raycaster; }
      
    protected:
      /** \brief The raycasted scene image viewer. */
      pcl::visualization::ImageViewer::Ptr scene_viewer;
      
      /** \brief The depth scene image viewer. */
      pcl::visualization::ImageViewer::Ptr depth_viewer;
      
      /** \brief */
      KinfuTracker::View view_device;
      //KinfuTracker::View colors_device_; //davidjones: MODIFIED FOR COLOR
      
      /** \brief */
      std::vector<KinfuTracker::PixelRGB> view_host;
      
      /** \brief */
      RayCaster::Ptr raycaster;
      
      
  private:
    /** \brief */
    ImageView (const ImageView& other); // Disable
    
    /** \brief */
    ImageView& operator= (const ImageView&); // Disable
  };
}

#endif // RXKINFU_IMAGE_VIEW_H