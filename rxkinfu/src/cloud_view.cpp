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

#include "cloud_view.h"

#include <pcl/common/time.h>
#include <pcl/surface/organized_fast_mesh.h>

#include <vtkLineSource.h>

#define FRAME_W 640
#define FRAME_H 480
#define CLIP_MIN 0.01
//#define CLIP_MAX 10.01
#define CLIP_MAX 1000.01
#define LINE_WIDTH 2.0f

//soft ultramarine
#define CLOUD_R 128
#define CLOUD_G 163
#define CLOUD_B 201

#define CUBE_R 0
#define CUBE_G 0
#define CUBE_B 0

using namespace Eigen;
using namespace pcl;
using namespace rxkinfu;
namespace pv = pcl::visualization;

CloudView::CloudView(const char *name, bool show_cloud) { 
  if (show_cloud) {
    cloud_viewer = pv::PCLVisualizer::Ptr (new pv::PCLVisualizer(name));
    cloud_viewer->initCameraParameters();
    cloud_viewer->setSize(FRAME_W, FRAME_H);
    cloud_viewer->setCameraClipDistances(CLIP_MIN, CLIP_MAX);
    cloud_viewer->setBackgroundColor(255, 255, 255);
  }
}

void CloudView::setViewerPose(const Affine3f& pose)
{
  if (!cloud_viewer) return;
  Vector3f loc = pose*Vector3f(0,0,0);
  Vector3f look = pose.rotation()*Vector3f(0,0,1)+loc;
  Vector3f up = pose.rotation()*Vector3f(0, -1, 0);
  cloud_viewer->setCameraPosition
    (loc[0], loc[1], loc[2], look[0], look[1], look[2], up[0], up[1], up[2]);
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::getViewerPose (Eigen::Affine3f& pose)
{
  if (cloud_viewer)
  {
    pose = cloud_viewer->getViewerPose();
    Matrix3f rotation = pose.linear();
    Matrix3f axis_reorder;  
    //    axis_reorder <<
     //     0,  0,  1,
     //     -1,  0,  0,
     //     0, -1,  0;
    axis_reorder <<
      -1,  0,  0,
      0,  -1,  0,
      0,  0,  1;
    rotation = rotation*axis_reorder;
    pose.linear() = rotation;
  }
  
  return;
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::resetViewerPose ()
{
  if (cloud_viewer) cloud_viewer->resetCamera();
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::showCloud (KinfuTracker &kinfu, const std::string id,
                      int triangle_size)
{
  kinfu.getLastFrameCloud(cloud_device);

  int c; cloud_device.download(cloud_ptr->points, c);
  cloud_ptr->width = cloud_device.cols();
  cloud_ptr->height = cloud_device.rows();
  cloud_ptr->is_dense = false;

  showCloud(cloud_ptr, id, triangle_size);
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::showCloud (RayCaster &raycaster, const std::string id,
                      int triangle_size)
{
  convertMapToOranizedCloud(raycaster.getVertexMap(), cloud_device);

  int c; cloud_device.download(cloud_ptr->points, c);
  cloud_ptr->width = cloud_device.cols();
  cloud_ptr->height = cloud_device.rows();
  cloud_ptr->is_dense = false;

  showCloud(cloud_ptr, id, triangle_size);
}

///////////////////////////////////////////////////////////////////////////////
void CloudView::showCloud (PointCloud<PointXYZ>::Ptr cloud_ptr,
                           const std::string id, int triangle_size)
{
  if (!cloud_viewer) return;
  
  std::cout << "triangle_size: " << triangle_size << std::endl;
  if (triangle_size <= 0)
  {
    if (!cloud_viewer->updatePointCloud<PointXYZ>(cloud_ptr, id))
      cloud_viewer->addPointCloud<PointXYZ>(cloud_ptr, id);
  }
  else
  {
    OrganizedFastMesh<PointXYZ> ofm;
    ofm.setTrianglePixelSize(triangle_size);
    ofm.setMaxEdgeLength(0.1f);
    //ofm.setTriangulationType(OrganizedFastMesh<PointXYZ>::QUAD_MESH);
    ofm.setTriangulationType
      (OrganizedFastMesh<PointXYZ>::TRIANGLE_ADAPTIVE_CUT);
    //ofm.setTriangulationType(OrganizedFastMesh<PointXYZ>::TRIANGLE_LEFT_CUT);
    std::vector<Vertices> verts;
    ofm.setInputCloud(cloud_ptr);
    ofm.reconstruct(verts);

    //updatePolygonMesh() seems to have problems when the triangle size changes
    if (last_triangle_size.count(id) &&
        (last_triangle_size[id] != triangle_size))
      cloud_viewer->removePointCloud(id);
    last_triangle_size[id] = triangle_size;

    if (verts.size() > 3)
    {
      std::cout << "cloud_ptr size: " << cloud_ptr->points.size() << std::endl;
      if (!cloud_viewer->updatePolygonMesh<PointXYZ>(cloud_ptr, verts, id))
      {
        cloud_viewer->addPolygonMesh<PointXYZ>(cloud_ptr, verts, id);
      
        std::cout << "cloud_viewer id: " << id << std::endl;
      }
    }
    else
    {
      cloud_viewer->removePointCloud(id);
    }
  }
  
  cloud_viewer->setPointCloudRenderingProperties
    (pv::PCL_VISUALIZER_COLOR,
     CLOUD_R/255.0, CLOUD_G/255.0, CLOUD_B/255.0,
     id);
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::removeCloud (const std::string id)
{
  if (cloud_viewer) cloud_viewer->removePointCloud(id);
  last_triangle_size.erase(id);
}

///////////////////////////////////////////////////////////////////////////////
static
void addLine (vtkSmartPointer<vtkAppendPolyData> polydata,
              vtkSmartPointer<vtkUnsignedCharArray> colors,
              float x1, float y1, float z1, float x2, float y2, float z2,
              float r, float g, float b)
{
  vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
  line->SetPoint1(x1, y1, z1);
  line->SetPoint2(x2, y2, z2);
  line->Update();
  polydata->AddInputData(line->GetOutput ());
  //polydata->AddInput(line->GetOutput ());
  unsigned char rgb[] = {r*255.0, g*255.0, b*255.0};
  colors->InsertNextTupleValue(rgb);
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::showCamera (const Affine3f &pose, const std::string id,
                       float near, float far,
                       float axes, float fr, float fg, float fb,
                       int rows, int cols,
                       float fx, float fy, float cx, float cy)
{  
  //first try a pose update to an existing shape
  if (!cloud_viewer || cloud_viewer->updateShapePose(id, pose)) return;
  
  if (cx <= 0) cx = ((float) cols)/2.0f;
  if (cy <= 0) cy = ((float) rows)/2.0f;
  
  vtkSmartPointer<vtkAppendPolyData> polydata =
    vtkSmartPointer<vtkAppendPolyData>::New();
  
  vtkSmartPointer<vtkUnsignedCharArray> colors =
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
      
  //draw axes
  if (axes > 0) {
    addLine(polydata, colors, 0, 0, 0, axes, 0, 0, 1, 0, 0);
    addLine(polydata, colors, 0, 0, 0, 0, axes, 0, 0, 1, 0);
    addLine(polydata, colors, 0, 0, 0, 0, 0, axes, 0, 0, 1);
  }
      
  //draw frustum
  if (near >= 0 && far >= 0 && far >= near) {
    
    float xn[] = {0, 0}, yn[] = {0, 0}, xf[] = {0, 0}, yf[] = {0, 0};
    const float zn = near, zf = far;
    const float xx[] = {-0.5f, cols-0.5f, cols-0.5f, -0.5f};
    const float yy[] = {-0.5f, -0.5f, rows-0.5f, rows-0.5f};
    
    for (int i = 0; i < 5; i++) {
      
      xn[1] = (xx[i%4]-cx)*zn/fx; yn[1] = (yy[i%4]-cy)*zn/fy;
      xf[1] = (xx[i%4]-cx)*zf/fx; yf[1] = (yy[i%4]-cy)*zf/fy;
      
      if (i > 0) {
        addLine(polydata, colors,
                xn[0], yn[0], zn, xn[1], yn[1], zn, fr, fg, fb);
        addLine(polydata, colors,
                xf[0], yf[0], zf, xf[1], yf[1], zf, fr, fg, fb);
        addLine(polydata, colors,
                xn[0], yn[0], zn, xf[0], yf[0], zf, fr, fg, fb);
      }
      
      xn[0] = xn[1]; yn[0] = yn[1];
      xf[0] = xf[1]; yf[0] = yf[1];
    }
  }
  
  polydata->Update();
  vtkSmartPointer<vtkPolyData> line_data = polydata->GetOutput();
  line_data->GetCellData()->SetScalars(colors);
      
  cloud_viewer->addModelFromPolyData(line_data, id);
  cloud_viewer->setShapeRenderingProperties(pv::PCL_VISUALIZER_LINE_WIDTH,
                                            LINE_WIDTH, id);

  cloud_viewer->updateShapePose(id, pose);
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::removeCamera (const std::string id)
{
  if (cloud_viewer) cloud_viewer->removeShape(id);
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::showBox (const Affine3f &pose, float *bflrkt,
                    const std::string id,
                    float axes, float br, float bg, float bb)
{
  //first try a pose update to an existing shape
  if (!cloud_viewer || cloud_viewer->updateShapePose(id, pose)) return;
  
  float def_bflrkt[] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  if (!bflrkt) bflrkt = def_bflrkt;
  
  vtkSmartPointer<vtkAppendPolyData> polydata =
    vtkSmartPointer<vtkAppendPolyData>::New();
  
  vtkSmartPointer<vtkUnsignedCharArray> colors =
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
  
  //draw axes
  if (axes > 0) {
    addLine(polydata, colors, 0, 0, 0, axes, 0, 0, 1, 0, 0);
    addLine(polydata, colors, 0, 0, 0, 0, axes, 0, 0, 1, 0);
    addLine(polydata, colors, 0, 0, 0, 0, 0, axes, 0, 0, 1);
  }
  
  //draw box
  float *h = bflrkt;
  const float b = h[0], f = h[1], l = -h[2], r = h[3], k = -h[4], t = -h[5];
  float x[] = {0, 0}, y[] = {0, 0};
  const float xx[] = {l, r, r, l}, yy[] = {t, t, b, b};
  
  for (int i = 0; i < 5; i++) {
    
    x[1] = xx[i%4]; y[1] = yy[i%4];
    
    if (i > 0) {
      addLine(polydata, colors, x[0], y[0], k, x[1], y[1], k, br, bg, bb);
      addLine(polydata, colors, x[0], y[0], f, x[1], y[1], f, br, bg, bb);
      addLine(polydata, colors, x[0], y[0], k, x[0], y[0], f, br, bg, bb);
    }
    
    x[0] = x[1]; y[0] = y[1];
  }
  
  polydata->Update();
  vtkSmartPointer<vtkPolyData> line_data = polydata->GetOutput();
  line_data->GetCellData()->SetScalars(colors);
  
  cloud_viewer->addModelFromPolyData(line_data, id);
  cloud_viewer->setShapeRenderingProperties(pv::PCL_VISUALIZER_LINE_WIDTH,
                                            LINE_WIDTH, id);

  cloud_viewer->updateShapePose(id, pose);
}
 
///////////////////////////////////////////////////////////////////////////////
void
CloudView::removeBox (const std::string id)
{
  if (cloud_viewer) cloud_viewer->removeShape(id);
}

///////////////////////////////////////////////////////////////////////////////
void
CloudView::showArrow (const Vector3f &v, const Vector3f &p,
                      float r, float g, float b,
                      bool display_length, const std::string id)
{  
  if (!cloud_viewer) return;
  
  cloud_viewer->removeShape(id);

  Vector3f h = p + v;
  cloud_viewer->addArrow(PointXYZ(h.x(), h.y(), h.z()),
                         PointXYZ(p.x(), p.y(), p.z()),
                         r, g, b, display_length, id);
}
  
void CloudView::removeArrow(const std::string id)
{ if (cloud_viewer) cloud_viewer->removeShape(id); }

bool CloudView::wasStopped()
{ return cloud_viewer && cloud_viewer->wasStopped(); }

////////////////////////////////////////////////////////////////////////////////
void CloudView::registerKeyboardCallback
(void (*callback) (const pv::KeyboardEvent&, void*), void* cookie)
{
  if (cloud_viewer) cloud_viewer->registerKeyboardCallback (callback, cookie);
}

////////////////////////////////////////////////////////////////////////////////
void
CloudView::registerMouseCallback
(void (*callback) (const pv::MouseEvent&, void*), void* cookie)
{
  if (cloud_viewer) cloud_viewer->registerMouseCallback (callback, cookie);
}

////////////////////////////////////////////////////////////////////////////////
void
CloudView::registerPointPickingCallback
(void (*callback) (const pv::PointPickingEvent&, void*), void* cookie)
{
  if (cloud_viewer) cloud_viewer->registerPointPickingCallback (callback, cookie);
}

////////////////////////////////////////////////////////////////////////////////
void CloudView::spinOnce(int time, bool force_redraw)
{ if (cloud_viewer) cloud_viewer->spinOnce(time, force_redraw); }

pv::PCLVisualizer::Ptr CloudView::getVisualizer() { return cloud_viewer; }

SceneCloudView::SceneCloudView(bool show_cloud)
  : CloudView("Scene Cloud", show_cloud), extraction_mode(GPU_CON6)
  , compute_normals(false), valid_combined(false), cube_added(false) {
  
  cloud_ptr = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
  normals_ptr = PointCloud<Normal>::Ptr(new PointCloud<Normal>);
  combined_ptr = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>);
  
  if (cloud_viewer) 
    cloud_viewer->setPosition(0, FRAME_H+20);
}

MarchingCubes* SceneCloudView::makeMarchingCubes()
{ return new MarchingCubes(); }

void SceneCloudView::show(KinfuTracker& kinfu) {
  
  ScopeTime time("Point Cloud Extraction");
  
  std::cout << "Getting cloud... " << std::flush;
  
  valid_combined = false;
  size_t np = 0;
  
  if (extraction_mode != GPU_CON6) {
    kinfu.volume().fetchCloudHost(*cloud_ptr, extraction_mode == CPU_CON26);
    np = cloud_ptr->points.size();
  } else {
    DeviceArray<PointXYZ> extracted =
      kinfu.volume().fetchCloud(cloud_buffer_device);             
    if (compute_normals) {
      kinfu.volume().fetchNormals(extracted, normals_device);
      kinfu.mergePointNormal(extracted, normals_device, combined_device);
      combined_device.download(combined_ptr->points);
      combined_ptr->width = (int)(combined_ptr->points.size());
      combined_ptr->height = 1;
      np = combined_ptr->points.size();
      valid_combined = true;
    } else {
      extracted.download(cloud_ptr->points);
      cloud_ptr->width = (int)(cloud_ptr->points.size());
      cloud_ptr->height = 1;
      np = cloud_ptr->points.size();
    }
  }
  
  std::cout << "done, " << (np/1000) << "K points\n";
  
  if (cloud_viewer) {
    cloud_viewer->removeAllPointClouds();
    const std::string id("Cloud");
    if (valid_combined) {
      cloud_viewer->addPointCloud<PointNormal>(combined_ptr, id);
      cloud_viewer->addPointCloudNormals<PointNormal>(combined_ptr,
                                                      50, 0.02f, id);
    } else cloud_viewer->addPointCloud<PointXYZ>(cloud_ptr, id);
    cloud_viewer->setPointCloudRenderingProperties
      (pv::PCL_VISUALIZER_COLOR,
       CLOUD_R/255.0, CLOUD_G/255.0, CLOUD_B/255.0,
       id);
    cloud_viewer->spinOnce();
  }
}

void SceneCloudView::showMesh(KinfuTracker& kinfu) {
  
  ScopeTime time("Mesh Extraction");
  
  std::cout << "Getting mesh... " << std::flush;
  
  if (!marching_cubes) marching_cubes = MarchingCubes::Ptr(makeMarchingCubes());

  
  DeviceArray<PointXYZ> triangles =
    marching_cubes->run(kinfu.volume(), triangles_buffer_device);    
  mesh_ptr = convertToMesh(triangles);
  
  std::cout << "done, "
            << (triangles.size()/MarchingCubes::POINTS_PER_TRIANGLE/1000)
            << "K triangles\n";
  
  if (cloud_viewer) {
    cloud_viewer->removeAllPointClouds();
    if (mesh_ptr) cloud_viewer->addPolygonMesh(*mesh_ptr);
    cloud_viewer->spinOnce();
  }
}

///////////////////////////////////////////////////////////////////////////////
void
SceneCloudView::toggleCube (const Vector3f& size)
{

  if (!cloud_viewer) return;
  
  cube_added = !cube_added;

  if (!cube_added)
  {
    cloud_viewer->removeCoordinateSystem (0);
    cloud_viewer->removeShape ("cube");
  }
  else
  {
    // TBD: this does not work in kinetic/Ubuntu 16.04
    cloud_viewer->addCoordinateSystem(1.0,"cube",0);
    
    cloud_viewer->addCube (size*0.5, Quaternionf::Identity(),
                           size(0), size(1), size(2));
    
    cloud_viewer->setShapeRenderingProperties (pv::PCL_VISUALIZER_LINE_WIDTH,
                                               LINE_WIDTH, "cube");
    
    cloud_viewer->setShapeRenderingProperties
      (pv::PCL_VISUALIZER_COLOR,
       CUBE_R/255.0, CUBE_G/255.0, CUBE_B/255.0,
       "cube");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
SceneCloudView::toggleExtractionMode ()
{
  extraction_mode = (extraction_mode+1)%3;
  std::cout << "\nCloud exctraction mode ";
  switch (extraction_mode) {
  case 0: std::cout << "GPU_CON6\n"; break;
  case 1: std::cout << "CPU_CON6 (requires a lot of memory)\n"; break;
  case 2: std::cout << "CPU_CON26 (requires a lot of memory)\n"; break;
  }
}

///////////////////////////////////////////////////////////////////////////////
void
SceneCloudView::toggleNormals ()
{
  compute_normals = !compute_normals;
  std::cout << "\nCompute normals: " << (compute_normals ? "On\n" : "Off\n");
  if (compute_normals && (extraction_mode != GPU_CON6)) {
    extraction_mode = GPU_CON6;
    std::cout << "Cloud extraction mode set to GPU_CON6, "
              << "required for normal extraction\n";
  }
}

///////////////////////////////////////////////////////////////////////////////
void
SceneCloudView::clearClouds (bool print_message)
{
  if (!cloud_viewer) return;
  cloud_viewer->removeAllPointClouds();
  cloud_ptr->points.clear ();
  normals_ptr->points.clear ();    
  if (print_message) std::cout << "Clouds/Meshes cleared\n";
}

///////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<PolygonMesh>
SceneCloudView::convertToMesh (const DeviceArray<PointXYZ>& triangles)
{
  if (triangles.empty()) return boost::shared_ptr<PolygonMesh>();
  
  PointCloud<PointXYZ> cloud;
  cloud.width  = (int)triangles.size();
  cloud.height = 1;
  triangles.download(cloud.points);
  
  boost::shared_ptr<PolygonMesh> mesh_ptr(new PolygonMesh()); 
  toPCLPointCloud2(cloud, mesh_ptr->cloud);
  
  mesh_ptr->polygons.resize(triangles.size()/3);
  for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i) {
    Vertices v;
    v.vertices.push_back(i*3+0);
    v.vertices.push_back(i*3+2);
    v.vertices.push_back(i*3+1);              
    mesh_ptr->polygons[i] = v;
  }    
  
  return mesh_ptr;
}

///////////////////////////////////////////////////////////////////////////////
void
SceneCloudView::writeCloud (int format) const
{
  if (!cloud_ptr->points.empty()) {
    if (valid_combined) writeCloudFile(format, combined_ptr);
    else writeCloudFile(format, cloud_ptr);
  }
}

///////////////////////////////////////////////////////////////////////////////
void
SceneCloudView::writeMesh (int format) const
{ if (mesh_ptr) writePolygonMeshFile(format, *mesh_ptr); }

///////////////////////////////////////////////////////////////////////////////
void
SceneCloudView::writePolygonMeshFile (int format, const PolygonMesh& mesh)
  const {
  if (format == MESH_PLY) {
    std::cout << "Saving mesh to to 'mesh.ply'..." << std::flush;
    io::savePLYFile("mesh.ply", mesh);		
  } else /* MESH_VTK */ {
    std::cout << "Saving mesh to to 'mesh.vtk'..." << std::flush;
    io::saveVTKFile("mesh.vtk", mesh);    
  }  
  std::cout << "Done\n";
}


///////////////////////////////////////////////////////////////////////////////
CurrentCloudView::CurrentCloudView(bool show_cloud)
  : CloudView("Current Frame Cloud", show_cloud) {
  
  cloud_ptr = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
  
  if (cloud_viewer) {
    //    cloud_viewer->setPointCloudRenderingProperties
    //      (pv::PCL_VISUALIZER_POINT_SIZE, 1);
    cloud_viewer->setPosition(FRAME_W, FRAME_H+20);
  }
}

///////////////////////////////////////////////////////////////////////////////
void
CurrentCloudView::show (const KinfuTracker& kinfu)
{
  kinfu.getLastFrameCloud(cloud_device);

  int c; cloud_device.download(cloud_ptr->points, c);

  cloud_ptr->width = cloud_device.cols();
  cloud_ptr->height = cloud_device.rows();
  cloud_ptr->is_dense = false;

  if (cloud_viewer) {
    cloud_viewer->removeAllPointClouds();
    const std::string id("Cloud");
    cloud_viewer->addPointCloud<PointXYZ>(cloud_ptr,id);
    cloud_viewer->setPointCloudRenderingProperties
      (pv::PCL_VISUALIZER_COLOR,
       CLOUD_R/255.0, CLOUD_G/255.0, CLOUD_B/255.0,
       id);
    cloud_viewer->spinOnce();
  }
}