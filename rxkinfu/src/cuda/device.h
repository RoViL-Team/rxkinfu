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

#ifndef MVKINFU_DEVICE_H_
#define MVKINFU_DEVICE_H_

#include "device_array.h"
#include "safe_call.h"

namespace rxkinfu
{
  namespace device
  {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Types
    typedef unsigned short ushort;
    typedef DeviceArray2D<float> MapArr;
    typedef DeviceArray2D<ushort> DepthMap;
    typedef float4 PointType;

    //TSDF fixed point divisor (if old format is enabled)
    const int DIVISOR = 32767;     // SHRT_MAX;

    //Should be multiple of 32
    enum { VOLUME_X = 512, VOLUME_Y = 512, VOLUME_Z = 512 };

    /** \brief Camera intrinsics structure
      */ 
    struct Intr
    {
      float fx, fy, cx, cy;
      Intr () {}
      Intr (float fx_, float fy_, float cx_, float cy_) : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}

      Intr operator()(int level_index) const
      { 
        int div = 1 << level_index; 
        return (Intr (fx / div, fy / div, cx / div, cy / div));
      }
    };

    /** \brief 3x3 Matrix for device code
      */ 
    struct Mat33
    {
      float3 data[3];
    };

    /** \brief Light source collection
      */ 
    struct LightSource
    {
      float3 pos[1];
      int number;
    };

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Maps
  
    /** \brief Perfoms bilateral filtering of disparity map
      * \param[in] src soruce map
      * \param[out] dst output map
      */
    void 
    bilateralFilter (const DepthMap& src, DepthMap& dst);
    
	/** \brief Computes depth pyramid
      * \param[in] src source
      * \param[out] dst destination
      */
    void 
    pyrDown (const DepthMap& src, DepthMap& dst);

    /** \brief Computes vertex map
      * \param[in] intr depth camera intrinsics
      * \param[in] depth depth
      * \param[out] vmap vertex map
      */
    void 
    createVMap (const Intr& intr, const DepthMap& depth, MapArr& vmap);
    
	/** \brief Computes normal map using cross product
      * \param[in] vmap vertex map
      * \param[out] nmap normal map
      */
    void 
    createNMap (const MapArr& vmap, MapArr& nmap);
    
	/** \brief Computes normal map using Eigen/PCA approach
      * \param[in] vmap vertex map
      * \param[out] nmap normal map
      */
    void 
    computeNormalsEigen (const MapArr& vmap, MapArr& nmap);

    /** \brief Performs affine transform of vertex and normal maps
      * \param[in] vmap_src source vertex map
      * \param[in] nmap_src source vertex map
      * \param[in] Rmat Rotation mat
      * \param[in] tvec translation
      * \param[out] vmap_dst destination vertex map
      * \param[out] nmap_dst destination vertex map
      */
    void 
    transformMaps (const MapArr& vmap_src, const MapArr& nmap_src, const Mat33& Rmat, const float3& tvec, MapArr& vmap_dst, MapArr& nmap_dst);

	/** \brief Performs depth truncation
      * \param[out] depth depth map to truncation
      * \param[in] max_distance truncation threshold, values that are higher than the threshold are reset to zero (means not measurement)
      */
	void 
	truncateDepth(DepthMap& depth, float max_distance);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //   ICP 
            
    /** \brief Computation Ax=b for ICP iteration
      * \param[in] Rcurr Rotation of current camera pose guess 
      * \param[in] tcurr translation of current camera pose guess 
      * \param[in] vmap_curr current vertex map in camera coo space
      * \param[in] nmap_curr current vertex map in camera coo space
      * \param[in] Rprev_inv inverse camera rotation at previous pose
      * \param[in] tprev camera translation at previous pose
      * \param[in] intr camera intrinsics
      * \param[in] vmap_g_prev previous vertex map in global coo space
      * \param[in] nmap_g_prev previous vertex map in global coo space
      * \param[in] distThres distance filtering threshold
      * \param[in] angleThres angle filtering threshold. Represents sine of angle between normals
      * \param[out] gbuf temp buffer for GPU reduction
      * \param[out] mbuf ouput GPU buffer for matrix computed
      * \param[out] matrixA_host A
      * \param[out] vectorB_host b
      */
    void 
    estimateCombined (const Mat33& Rcurr, const float3& tcurr, const MapArr& vmap_curr, const MapArr& nmap_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
                      const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, 
                      DeviceArray2D<float>& gbuf, DeviceArray<float>& mbuf, float* matrixA_host, float* vectorB_host);


	void
	estimateCombined (const Mat33& Rcurr, const float3& tcurr, const MapArr& vmap_curr, const MapArr& nmap_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr,
                      const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres,
                      DeviceArray2D<double>& gbuf, DeviceArray<double>& mbuf, double* matrixA_host, double* vectorB_host);


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TSDF volume functions            

    /** \brief Perform tsdf volume initialization
      *  \param[out] array volume to be initialized
      */
    void
    initVolume(PtrStep<short2> array);

    //first version
    /** \brief Performs Tsfg volume uptation (extra obsolete now)
      * \param[in] depth_raw Kinect depth image
      * \param[in] intr camera intrinsics
      * \param[in] volume_size size of volume in mm
      * \param[in] Rcurr_inv inverse rotation for current camera pose
      * \param[in] tcurr translation for current camera pose
      * \param[in] trunc_dist tsdf truncation distance
      * \param[in] volume tsdf volume to be updated
      */
    void 
    integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                         const Mat33& Rcurr_inv, const float3& tcurr, float trunc_dist, PtrStep<short2> volume);

    //second version
    /** \brief Function that integrates volume if volume element contains: 2 bytes for round(tsdf*SHORT_MAX) and 2 bytes for integer weight.
      * \param[in] depth_raw Kinect depth image
      * \param[in] intr camera intrinsics
      * \param[in] volume_size size of volume in mm
      * \param[in] Rcurr_inv inverse rotation for current camera pose
      * \param[in] tcurr translation for current camera pose
      * \param[in] trunc_dist tsdf truncation distance
      * \param[in] volume tsdf volume to be updated
      * \param[out] depthRawScaled Buffer for scaled depth along ray
      */
    void 
    integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                         const Mat33& Rcurr_inv, const float3& tcurr, float trunc_dist, PtrStep<short2> volume, DeviceArray2D<float>& depthRawScaled);
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Raycast and view generation        
    /** \brief Generation vertex and normal maps from volume for current camera pose
      * \param[in] intr camera intrinsices
      * \param[in] Rcurr current rotation
      * \param[in] tcurr current translation
      * \param[in] trunc_dist volume truncation distance
      * \param[in] volume_size volume size in mm
      * \param[in] volume tsdf volume
      * \param[out] vmap output vertex map
      * \param[out] nmap output normals map
      */
    void 
    raycast (const Intr& intr, const Mat33& Rcurr, const float3& tcurr, float trunc_dist, const float3& volume_size, 
             const PtrStep<short2>& volume, MapArr& vmap, MapArr& nmap);

    /** \brief Renders 3D image of the scene
      * \param[in] vmap vetex map
      * \param[in] nmap normals map
      * \param[in] light poase of light source
      * \param[out] dst buffer where image is generated
      */
    void 
    generateImage (const MapArr& vmap, const MapArr& nmap, const LightSource& light, PtrStepSz<uchar3> dst);
    
    //davidjones: MODIFIED FOR COLOR
    /** \brief Renders 3D image of the scene
      * \param[in] vmap vetex map
      * \param[in] nmap normals map
      * \param[in] colorsmap colors map
      * \param[in] light poase of light source
      * \param[out] dst buffer where image is generated
      */
    //void
    //generateImage (const MapArr& vmap, const MapArr& nmap, PtrStepSz<uchar3> colormap,
    //               const LightSource& light, PtrStepSz<uchar3> dst);

    /** \brief Renders depth image from give pose
      * \param[in] vmap inverse camera rotation
      * \param[in] nmap camera translation
      * \param[in] light vertex map
      * \param[out] dst buffer where depth is generated
      */
    void
    generateDepth (const Mat33& R_inv, const float3& t, const MapArr& vmap, DepthMap& dst);

    /** \brief Performs resize of vertex map to next pyramid level by averaging each four points
      * \param[in] input vertext map
      * \param[out] output resized vertex map
      */
    void 
    resizeVMap (const MapArr& input, MapArr& output);
    
    /** \brief Performs resize of vertex map to next pyramid level by averaging each four normals
      * \param[in] input normal map
      * \param[out] output vertex map
      */
    void 
    resizeNMap (const MapArr& input, MapArr& output);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Cloud extraction 

    /** \brief Perform point cloud extraction from tsdf volume
      * \param[in] volume tsdf volume 
      * \param[in] volume_size size of the volume
      * \param[out] output buffer large enought to store point cloud
      * \return number of point stored to passed buffer
      */ 
    size_t 
    extractCloud (const PtrStep<short2>& volume, const float3& volume_size, PtrSz<PointType> output);

    /** \brief Performs normals computation for given poins using tsdf volume
      * \param[in] volume tsdf volume
      * \param[in] volume_size volume size
      * \param[in] input points where normals are computed
      * \param[out] output normals. Could be float4 or float8. If for a point normal can't be computed, such normal is marked as nan.
      */ 
    template<typename NormalType> 
    void 
    extractNormals (const PtrStep<short2>& volume, const float3& volume_size, const PtrSz<PointType>& input, NormalType* output);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Utility
    struct float8  { float x, y, z, w, c1, c2, c3, c4; };
    struct float12 { float x, y, z, w, normal_x, normal_y, normal_z, n4, c1, c2, c3, c4; };

    /** \brief Conversion from SOA to AOS
      * \param[in] vmap SOA map
      * \param[out] output Array of 3D points. Can be float4 or float8.
      */
    template<typename T> 
    void 
    convert (const MapArr& vmap, DeviceArray2D<T>& output);

    /** \brief Merges pcl::PointXYZ and pcl::Normal to PointNormal
      * \param[in] coud points cloud
      * \param[in] normals normals cloud
      * \param[out] output array of PointNomals.
      */
    void 
    mergePointNormal(const DeviceArray<float4>& cloud, const DeviceArray<float8>& normals, const DeviceArray<float12>& output);

    /** \brief  Check for qnan (unused now) 
      * \param[in] value
      */
    inline bool 
    valid_host (float value)
    {
      return *reinterpret_cast<int*>(&value) != 0x7fffffff; //QNAN
    }

    /** \brief synchronizes CUDA execution */
    inline 
    void 
    sync () { cudaSafeCall (cudaDeviceSynchronize ()); }


    template<class D, class Matx> D&
    device_cast (Matx& matx)
    {
      return (*reinterpret_cast<D*>(matx.data ()));
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Marching cubes implementation

    /** \brief Binds marching cubes tables to texture references */
    void 
    bindTextures(const int *edgeBuf, const int *triBuf, const int *numVertsBuf);            
    
    /** \brief Unbinds */
    void 
    unbindTextures();
    
    /** \brief Scans tsdf volume and retrieves occuped voxes
      * \param[in] volume tsdf volume
      * \param[out] occupied_voxels buffer for occuped voxels. The function fulfills first row with voxel ids and second row with number of vertextes.
      * \return number of voxels in the buffer
      */
    int
    getOccupiedVoxels(const PtrStep<short2>& volume, DeviceArray2D<int>& occupied_voxels);

    /** \brief Computes total number of vertexes for all voxels and offsets of vertexes in final triangle array
      * \param[out] occupied_voxels buffer with occuped voxels. The function fulfills 3nd only with offsets      
      * \return total number of vertexes
      */
    int
    computeOffsetsAndTotalVertexes(DeviceArray2D<int>& occupied_voxels);

    /** \brief Generates final triangle array
      * \param[in] volume tsdf volume
      * \param[in] occupied_voxels occuped voxel ids (first row), number of vertexes(second row), offsets(third row).
      * \param[in] volume_size volume size in meters
      * \param[out] output triangle array            
      */
    void
    generateTriangles(const PtrStep<short2>& volume, const DeviceArray2D<int>& occupied_voxels, const float3& volume_size, DeviceArray<PointType>& output);

    /** \brief Translate the tsdf volume by an integer number of grid units.
     *
     * The origin of volume frame is the center of the upper left corner voxel,
     * with +x "right", +y "down", and +z "in".
     *
     * \param[in] trans new volume frame origin in integer grid cell units
     * \param[in] from_volume volume to translate from
     * \param[out] to_volume volume to translate to, must not alias from_volume
     **/
    void shiftTsdfVolume (const int3& trans,
                          PtrStep<short2> from_volume,
                          PtrStep<short2> to_volume);

    /** \brief Translate and rotate the tsdf volume.
     *
     * The origin of volume frame is the center of the upper left corner voxel,
     * with +x "right", +y "down", and +z "in".
     *
     * rot is the upper left 3x3 and trans the upper right 3x1 submatrix of a
     * 4x4 homogenous transform that takes points from the to_volume frame to
     * the from_volume frame.
     *
     * \param[in] trans new volume frame origin in grid cell units
     * \parma[in] new volume frame basis
     * \param[in] from_volume volume to translate from
     * \param[out] to_volume volume to translate to, must not alias from_volume
     * \param[in] checknn first check if nearest neighbor is a sentinel,
     * default false
     * \param[in] checkvalid check weights for invalids, default false
     **/
    void transformTsdfVolume (const float3& trans, const Mat33& rot,
                              PtrStep<short2> from_volume,
                              PtrStep<short2> to_volume,
                              const bool checknn = false,
                              const bool checkvalid = false);
  }
}

#endif
