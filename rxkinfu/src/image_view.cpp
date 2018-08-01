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

#include "image_view.h"

// TBD: change those to global variables
#define FRAME_W 640
#define FRAME_H 480
#define DEPTH_MAX 5000

// STD headers
#include <iostream>

// PCL headers
#include <pcl/visualization/common/float_image_utils.h>

using namespace pcl;
using namespace rxkinfu;
namespace pv = pcl::visualization;

///////////////////////////////////////////////////////////////////////////////
ImageView::ImageView (bool show_raycast, bool show_depth)
{
  // whether to show the raycasted image
  if (show_raycast)
  {
    scene_viewer = pv::ImageViewer::Ptr (new pv::ImageViewer);
    scene_viewer->setWindowTitle ("View from ray tracing");
    scene_viewer->setPosition (0, 0);
  }
  
  // whether to show the current depth image
  if (show_depth)
  {
    depth_viewer = pv::ImageViewer::Ptr (new pv::ImageViewer);
    depth_viewer->setWindowTitle ("Kinect depth stream");
    depth_viewer->setPosition (FRAME_W, 0);
  }
}

///////////////////////////////////////////////////////////////////////////////
RayCaster*
ImageView::makeRayCaster (int rows, int cols)
{
  return new RayCaster(rows, cols);
}

///////////////////////////////////////////////////////////////////////////////
void
ImageView::show (KinfuTracker& kinfu, Eigen::Affine3f* pose)
{
  if (!scene_viewer) return;
  
  if (!raycaster)
    raycaster = RayCaster::Ptr(makeRayCaster(kinfu.rows(), kinfu.cols()));
  
  if (pose)
  {
    raycaster->run (kinfu.volume(), *pose);
    raycaster->generateSceneView (view_device);
  }
  else
  {
    kinfu.getImage (view_device);
  }
  
  int c; view_device.download (view_host, c);
  
  scene_viewer->addRGBImage (reinterpret_cast<unsigned char*>(&view_host[0]),
                             view_device.cols(), view_device.rows());    
}

/*davidjones: MODIFIED FOR COLOR*/
///////////////////////////////////////////////////////////////////////////////
//void
//ImageView::show (KinfuTracker& kinfu, KinfuTracker::View color_device_in, Eigen::Affine3f* pose)
//{
//  if (!scene_viewer) return;
//  
//  if (!raycaster)
//    raycaster = RayCaster::Ptr(makeRayCaster(kinfu.rows(), kinfu.cols()));
//  
//  if (pose)
//  {
//    raycaster->run (kinfu.volume(), *pose);
//    raycaster->generateSceneView (view_device, color_device_in);
//  }
//  else
//  {
//    kinfu.getImage (view_device, color_device_in); //davidjones: MODIFIED FOR COLOR
//  }
//  
//  int c; view_device.download (view_host, c);
//  
//  scene_viewer->addRGBImage (reinterpret_cast<unsigned char*>(&view_host[0]),
//                             view_device.cols(), view_device.rows());    
//}

///////////////////////////////////////////////////////////////////////////////
void
ImageView::showDepth (const PtrStepSz<const unsigned short>& depth)
{
  // if depth viewer is not generated return
  if (!depth_viewer) return;
  
  //depth_viewer->showShortImage(depth.data, depth.cols, depth.rows,
  //                             0, DEPTH_MAX, true); 
  unsigned char *vbuf =
    pv::FloatImageUtils::getVisualImage (depth.data, depth.cols, depth.rows,
                                         0, DEPTH_MAX);
  depth_viewer->addRGBImage (vbuf, depth.cols, depth.rows);
  delete[] vbuf;
}

///////////////////////////////////////////////////////////////////////////////
void
ImageView::showGeneratedDepth (KinfuTracker& kinfu,
                               const Eigen::Affine3f& pose)
{
  // if depth viewer is not generated return
  if (!depth_viewer) return;
  
  // if raycasting viewer is not generated, create one
  if (!raycaster)
    raycaster = RayCaster::Ptr (new RayCaster (kinfu.rows (), kinfu.cols ()));
  
  raycaster->run (kinfu.volume(), pose);
  
  KinfuTracker::DepthMap depth; int c; std::vector<unsigned short> data;
  raycaster->generateDepthImage(depth);    
  
  depth.download(data, c);
  depth_viewer->addShortImage (&data[0], depth.cols(), depth.rows(),
                               0, DEPTH_MAX, true);
}

///////////////////////////////////////////////////////////////////////////////
bool
ImageView::wasStopped ()
{
  return
    (scene_viewer && scene_viewer->wasStopped ()) ||
    (depth_viewer && depth_viewer->wasStopped ());
}

///////////////////////////////////////////////////////////////////////////////
void
ImageView::registerKeyboardCallback
  (void (*callback) (const pv::KeyboardEvent&, void*), void* cookie)
{
  if (scene_viewer) scene_viewer->registerKeyboardCallback (callback, cookie);
  if (depth_viewer) depth_viewer->registerKeyboardCallback (callback, cookie);
}

///////////////////////////////////////////////////////////////////////////////
void
ImageView::spinOnce (int time, bool force_redraw)
{
  if (scene_viewer) scene_viewer->spinOnce (time, force_redraw);
  if (depth_viewer) depth_viewer->spinOnce (time, force_redraw);
}