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

#include "device_cuda.h"

using namespace rxkinfu::device;

namespace rxkinfu
{
  namespace device
  {
    template<typename T>
    __global__ void
    initializeVolume (PtrStep<T> volume)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;
      
      
      if (x < VOLUME_X && y < VOLUME_Y)
      {
          T *pos = volume.ptr(y) + x;
          int z_step = VOLUME_Y * volume.step / sizeof(*pos);

#pragma unroll
          for(int z = 0; z < VOLUME_Z; ++z, pos+=z_step)
             pack_tsdf (0.f, 0, *pos);
      }
    }   
  }
}

void
rxkinfu::device::initVolume (PtrStep<short2> volume)
{
  dim3 block (32, 16);
  dim3 grid (1, 1, 1);
  grid.x = divUp (VOLUME_X, block.x);      
  grid.y = divUp (VOLUME_Y, block.y);

  initializeVolume<<<grid, block>>>(volume);
  cudaSafeCall ( cudaGetLastError () );
  cudaSafeCall (cudaDeviceSynchronize ());
}

namespace rxkinfu
{
  namespace device
  {
    struct Tsdf
    {
      enum
      {
        CTA_SIZE_X = 32, CTA_SIZE_Y = 8,
        MAX_WEIGHT = 1 << 7
      };

      mutable PtrStep<short2> volume;
      float3 cell_size;

      Intr intr;

      Mat33 Rcurr_inv;
      float3 tcurr;

      PtrStepSz<ushort> depth_raw; //depth in mm

      float trunc_dist_mm;

      __device__ __forceinline__ float3
      getVoxelGCoo (int x, int y, int z) const
      {
        float3 coo = make_float3 (x, y, z);
        coo += 0.5f;         //shift to cell center;

        coo.x *= cell_size.x;
        coo.y *= cell_size.y;
        coo.z *= cell_size.z;

        return coo;
      }

      __device__ __forceinline__ void
      operator () () const
      {
        int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
        int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

        if (x >= VOLUME_X || y >= VOLUME_Y)
          return;

        short2 *pos = volume.ptr (y) + x;
        int elem_step = volume.step * VOLUME_Y / sizeof(*pos);

        for (int z = 0; z < VOLUME_Z; ++z, pos += elem_step)
        {
          float3 v_g = getVoxelGCoo (x, y, z);            //3 // p

          //transform to curr cam coo space
          float3 v = Rcurr_inv * (v_g - tcurr);           //4

          int2 coo;           //project to current cam
          coo.x = __float2int_rn (v.x * intr.fx / v.z + intr.cx);
          coo.y = __float2int_rn (v.y * intr.fy / v.z + intr.cy);

          if (v.z > 0 && coo.x >= 0 && coo.y >= 0 && coo.x < depth_raw.cols && coo.y < depth_raw.rows)           //6
          {
            int Dp = depth_raw.ptr (coo.y)[coo.x];

            if (Dp != 0)
            {
              float xl = (coo.x - intr.cx) / intr.fx;
              float yl = (coo.y - intr.cy) / intr.fy;
              float lambda_inv = rsqrtf (xl * xl + yl * yl + 1);

              float sdf = 1000 * norm (tcurr - v_g) * lambda_inv - Dp; //mm

              sdf *= (-1);

              if (sdf >= -trunc_dist_mm)
              {
                float tsdf = fmin (1, sdf / trunc_dist_mm);

                int weight_prev;
                float tsdf_prev;

                //read and unpack
                unpack_tsdf (*pos, tsdf_prev, weight_prev);

                const int Wrk = 1;

                float tsdf_new = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
                int weight_new = min (weight_prev + Wrk, MAX_WEIGHT);

                pack_tsdf (tsdf_new, weight_new, *pos);
              }
            }
          }
        }
      }
    };

    __global__ void
    integrateTsdfKernel (const Tsdf tsdf) {
      tsdf ();
    }

    __global__ void
    tsdf2 (PtrStep<short2> volume, const float trunc_dist_mm, const Mat33 Rcurr_inv, float3 tcurr,
           const Intr intr, const PtrStepSz<ushort> depth_raw, const float3 cell_size)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x >= VOLUME_X || y >= VOLUME_Y)
        return;

      short2 *pos = volume.ptr (y) + x;
      int elem_step = volume.step * VOLUME_Y / sizeof(short2);

      float v_g_x = (x + 0.5f) * cell_size.x - tcurr.x;
      float v_g_y = (y + 0.5f) * cell_size.y - tcurr.y;
      float v_g_z = (0 + 0.5f) * cell_size.z - tcurr.z;

      float v_x = Rcurr_inv.data[0].x * v_g_x + Rcurr_inv.data[0].y * v_g_y + Rcurr_inv.data[0].z * v_g_z;
      float v_y = Rcurr_inv.data[1].x * v_g_x + Rcurr_inv.data[1].y * v_g_y + Rcurr_inv.data[1].z * v_g_z;
      float v_z = Rcurr_inv.data[2].x * v_g_x + Rcurr_inv.data[2].y * v_g_y + Rcurr_inv.data[2].z * v_g_z;

//#pragma unroll
      for (int z = 0; z < VOLUME_Z; ++z)
      {
        float3 vr;
        vr.x = v_g_x;
        vr.y = v_g_y;
        vr.z = (v_g_z + z * cell_size.z);

        float3 v;
        v.x = v_x + Rcurr_inv.data[0].z * z * cell_size.z;
        v.y = v_y + Rcurr_inv.data[1].z * z * cell_size.z;
        v.z = v_z + Rcurr_inv.data[2].z * z * cell_size.z;

        int2 coo;         //project to current cam
        coo.x = __float2int_rn (v.x * intr.fx / v.z + intr.cx);
        coo.y = __float2int_rn (v.y * intr.fy / v.z + intr.cy);


        if (v.z > 0 && coo.x >= 0 && coo.y >= 0 && coo.x < depth_raw.cols && coo.y < depth_raw.rows)         //6
        {
          int Dp = depth_raw.ptr (coo.y)[coo.x]; //mm

          if (Dp != 0)
          {
            float xl = (coo.x - intr.cx) / intr.fx;
            float yl = (coo.y - intr.cy) / intr.fy;
            float lambda_inv = rsqrtf (xl * xl + yl * yl + 1);

            float sdf = Dp - norm (vr) * lambda_inv * 1000; //mm


            if (sdf >= -trunc_dist_mm)
            {
              float tsdf = fmin (1.f, sdf / trunc_dist_mm);

              int weight_prev;
              float tsdf_prev;

              //read and unpack
              unpack_tsdf (*pos, tsdf_prev, weight_prev);

              const int Wrk = 1;

              float tsdf_new = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
              int weight_new = min (weight_prev + Wrk, Tsdf::MAX_WEIGHT);

              pack_tsdf (tsdf_new, weight_new, *pos);
            }
          }
        }
        pos += elem_step;
      }       /* for(int z = 0; z < VOLUME_Z; ++z) */
    }      /* __global__ */
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
rxkinfu::device::integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size,
                                      const Mat33& Rcurr_inv, const float3& tcurr, float trunc_dist, 
                                      PtrStep<short2> volume)
{
  Tsdf tsdf;

  tsdf.volume = volume;  
  tsdf.cell_size.x = volume_size.x / VOLUME_X;
  tsdf.cell_size.y = volume_size.y / VOLUME_Y;
  tsdf.cell_size.z = volume_size.z / VOLUME_Z;
  
  tsdf.intr = intr;

  tsdf.Rcurr_inv = Rcurr_inv;
  tsdf.tcurr = tcurr;
  tsdf.depth_raw = depth_raw;

  tsdf.trunc_dist_mm = trunc_dist*1000; //mm

  dim3 block (Tsdf::CTA_SIZE_X, Tsdf::CTA_SIZE_Y);
  dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

#if 0
   //tsdf2<<<grid, block>>>(volume, trunc_dist, Rcurr_inv, tcurr, intr, depth_raw, tsdf.cell_size);
   integrateTsdfKernel<<<grid, block>>>(tsdf);
#endif
  cudaSafeCall ( cudaGetLastError () );
  cudaSafeCall (cudaDeviceSynchronize ());
}


namespace rxkinfu
{
  namespace device
  {
    __global__ void
    scaleDepth (const PtrStepSz<ushort> depth, PtrStep<float> scaled, const Intr intr)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x >= depth.cols || y >= depth.rows)
        return;

      int Dp = depth.ptr (y)[x];

      float xl = (x - intr.cx) / intr.fx;
      float yl = (y - intr.cy) / intr.fy;
      float lambda = sqrtf (xl * xl + yl * yl + 1);

      scaled.ptr (y)[x] = Dp * lambda/1000.f; //meters
    }

    __global__ void
    tsdf23 (const PtrStepSz<float> depthScaled, PtrStep<short2> volume,
            const float trunc_dist, const Mat33 Rcurr_inv, const float3 tcurr, const Intr intr, const float3 cell_size)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x >= VOLUME_X || y >= VOLUME_Y)
        return;

      float v_g_x = (x + 0.5f) * cell_size.x - tcurr.x;
      float v_g_y = (y + 0.5f) * cell_size.y - tcurr.y;
      float v_g_z = (0 + 0.5f) * cell_size.z - tcurr.z;

      float v_g_part_norm = v_g_x * v_g_x + v_g_y * v_g_y;

      float v_x = (Rcurr_inv.data[0].x * v_g_x + Rcurr_inv.data[0].y * v_g_y + Rcurr_inv.data[0].z * v_g_z) * intr.fx;
      float v_y = (Rcurr_inv.data[1].x * v_g_x + Rcurr_inv.data[1].y * v_g_y + Rcurr_inv.data[1].z * v_g_z) * intr.fy;
      float v_z = (Rcurr_inv.data[2].x * v_g_x + Rcurr_inv.data[2].y * v_g_y + Rcurr_inv.data[2].z * v_g_z);

      float z_scaled = 0;

      float Rcurr_inv_0_z_scaled = Rcurr_inv.data[0].z * cell_size.z * intr.fx;
      float Rcurr_inv_1_z_scaled = Rcurr_inv.data[1].z * cell_size.z * intr.fy;

      float trunc_dist_inv = 1.0f / trunc_dist;

      short2* pos = volume.ptr (y) + x;
      int elem_step = volume.step * VOLUME_Y / sizeof(short2);

//#pragma unroll
      for (int z = 0; z < VOLUME_Z;
           ++z,
           v_g_z += cell_size.z,
           z_scaled += cell_size.z,
           v_x += Rcurr_inv_0_z_scaled,
           v_y += Rcurr_inv_1_z_scaled,
           pos += elem_step)
      {
        float inv_z = 1.0f / (v_z + Rcurr_inv.data[2].z * z_scaled);
        if (inv_z < 0)
            continue;

        // project to current cam
        int2 coo =
        {
          __float2int_rn (v_x * inv_z + intr.cx),
          __float2int_rn (v_y * inv_z + intr.cy)
        };

        if (coo.x >= 0 && coo.y >= 0 && coo.x < depthScaled.cols && coo.y < depthScaled.rows)         //6
        {
          float Dp_scaled = depthScaled.ptr (coo.y)[coo.x]; //meters

          float sdf = Dp_scaled - sqrtf (v_g_z * v_g_z + v_g_part_norm);

          if (Dp_scaled != 0 && sdf >= -trunc_dist) //meters
          {
            float tsdf = fmin (1.0f, sdf * trunc_dist_inv);

            //read and unpack
            float tsdf_prev;
            int weight_prev;
            unpack_tsdf (*pos, tsdf_prev, weight_prev);

            const int Wrk = 1;

            float tsdf_new = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
            int weight_new = min (weight_prev + Wrk, Tsdf::MAX_WEIGHT);

            pack_tsdf (tsdf_new, weight_new, *pos);
          }
        }
      }       // for(int z = 0; z < VOLUME_Z; ++z)
    }      // __global__

    __global__ void
    tsdf23normal_hack (const PtrStepSz<float> depthScaled, PtrStep<short2> volume,
                  const float trunc_dist, const Mat33 Rcurr_inv, const float3 tcurr, const Intr intr, const float3 cell_size)
    {
        int x = threadIdx.x + blockIdx.x * blockDim.x;
        int y = threadIdx.y + blockIdx.y * blockDim.y;

        if (x >= VOLUME_X || y >= VOLUME_Y)
            return;

        const float v_g_x = (x + 0.5f) * cell_size.x - tcurr.x;
        const float v_g_y = (y + 0.5f) * cell_size.y - tcurr.y;
        float v_g_z = (0 + 0.5f) * cell_size.z - tcurr.z;

        float v_g_part_norm = v_g_x * v_g_x + v_g_y * v_g_y;

        float v_x = (Rcurr_inv.data[0].x * v_g_x + Rcurr_inv.data[0].y * v_g_y + Rcurr_inv.data[0].z * v_g_z) * intr.fx;
        float v_y = (Rcurr_inv.data[1].x * v_g_x + Rcurr_inv.data[1].y * v_g_y + Rcurr_inv.data[1].z * v_g_z) * intr.fy;
        float v_z = (Rcurr_inv.data[2].x * v_g_x + Rcurr_inv.data[2].y * v_g_y + Rcurr_inv.data[2].z * v_g_z);

        float z_scaled = 0;

        float Rcurr_inv_0_z_scaled = Rcurr_inv.data[0].z * cell_size.z * intr.fx;
        float Rcurr_inv_1_z_scaled = Rcurr_inv.data[1].z * cell_size.z * intr.fy;

        float trunc_dist_inv = 1.0f / trunc_dist;

        short2* pos = volume.ptr (y) + x;
        int elem_step = volume.step * VOLUME_Y / sizeof(short2);

        //#pragma unroll
        for (int z = 0; z < VOLUME_Z;
            ++z,
            v_g_z += cell_size.z,
            z_scaled += cell_size.z,
            v_x += Rcurr_inv_0_z_scaled,
            v_y += Rcurr_inv_1_z_scaled,
            pos += elem_step)
        {
            float inv_z = 1.0f / (v_z + Rcurr_inv.data[2].z * z_scaled);
            if (inv_z < 0)
                continue;

            // project to current cam
            int2 coo =
            {
                __float2int_rn (v_x * inv_z + intr.cx),
                __float2int_rn (v_y * inv_z + intr.cy)
            };

            if (coo.x >= 0 && coo.y >= 0 && coo.x < depthScaled.cols && coo.y < depthScaled.rows)         //6
            {
                float Dp_scaled = depthScaled.ptr (coo.y)[coo.x]; //meters

                float sdf = Dp_scaled - sqrtf (v_g_z * v_g_z + v_g_part_norm);

                if (Dp_scaled != 0 && sdf >= -trunc_dist) //meters
                {
                    float tsdf = fmin (1.0f, sdf * trunc_dist_inv);                                              

                    bool integrate = true;
                    if ((x > 0 &&  x < VOLUME_X-2) && (y > 0 && y < VOLUME_Y-2) && (z > 0 && z < VOLUME_Z-2))
                    {
                        const float qnan = numeric_limits<float>::quiet_NaN();
                        float3 normal = make_float3(qnan, qnan, qnan);

                        float Fn, Fp;
                        int Wn = 0, Wp = 0;
                        unpack_tsdf (*(pos + elem_step), Fn, Wn);
                        unpack_tsdf (*(pos - elem_step), Fp, Wp);

                        if (Wn > 16 && Wp > 16) 
                            normal.z = (Fn - Fp)/cell_size.z;

                        unpack_tsdf (*(pos + volume.step/sizeof(short2) ), Fn, Wn);
                        unpack_tsdf (*(pos - volume.step/sizeof(short2) ), Fp, Wp);

                        if (Wn > 16 && Wp > 16) 
                            normal.y = (Fn - Fp)/cell_size.y;

                        unpack_tsdf (*(pos + 1), Fn, Wn);
                        unpack_tsdf (*(pos - 1), Fp, Wp);

                        if (Wn > 16 && Wp > 16) 
                            normal.x = (Fn - Fp)/cell_size.x;

                        if (normal.x != qnan && normal.y != qnan && normal.z != qnan)
                        {
                            float norm2 = dot(normal, normal);
                            if (norm2 >= 1e-10)
                            {
                                normal *= rsqrt(norm2);

                                float nt = v_g_x * normal.x + v_g_y * normal.y + v_g_z * normal.z;
                                float cosine = nt * rsqrt(v_g_x * v_g_x + v_g_y * v_g_y + v_g_z * v_g_z);

                                if (cosine < 0.5)
                                    integrate = false;
                            }
                        }
                    }

                    if (integrate)
                    {
                        //read and unpack
                        float tsdf_prev;
                        int weight_prev;
                        unpack_tsdf (*pos, tsdf_prev, weight_prev);

                        const int Wrk = 1;

                        float tsdf_new = (tsdf_prev * weight_prev + Wrk * tsdf) / (weight_prev + Wrk);
                        int weight_new = min (weight_prev + Wrk, Tsdf::MAX_WEIGHT);

                        pack_tsdf (tsdf_new, weight_new, *pos);
                    }
                }
            }
        }       // for(int z = 0; z < VOLUME_Z; ++z)
    }      // __global__
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
rxkinfu::device::integrateTsdfVolume (const PtrStepSz<ushort>& depth, const Intr& intr,
                                      const float3& volume_size, const Mat33& Rcurr_inv, const float3& tcurr, 
                                      float trunc_dist,
                                      PtrStep<short2> volume, DeviceArray2D<float>& depthScaled)
{
  depthScaled.create (depth.rows, depth.cols);

  dim3 block_scale (32, 8);
  dim3 grid_scale (divUp (depth.cols, block_scale.x), divUp (depth.rows, block_scale.y));

  //scales depth along ray and converts mm -> meters. 
  scaleDepth<<<grid_scale, block_scale>>>(depth, depthScaled, intr);
  cudaSafeCall ( cudaGetLastError () );

  float3 cell_size;
  cell_size.x = volume_size.x / VOLUME_X;
  cell_size.y = volume_size.y / VOLUME_Y;
  cell_size.z = volume_size.z / VOLUME_Z;

  //dim3 block(Tsdf::CTA_SIZE_X, Tsdf::CTA_SIZE_Y);
  dim3 block (16, 16);
  dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

  tsdf23<<<grid, block>>>(depthScaled, volume, trunc_dist, Rcurr_inv, tcurr, intr, cell_size);    
  //tsdf23normal_hack<<<grid, block>>>(depthScaled, volume, trunc_dist, Rcurr_inv, tcurr, intr, cell_size);

  cudaSafeCall ( cudaGetLastError () );
  cudaSafeCall (cudaDeviceSynchronize ());
}

namespace rxkinfu {
  namespace device {

    __global__ void
    shiftTsdf(const int3 trans,
              PtrStep<short2> from_volume, PtrStep<short2> to_volume) { 

      const int x = threadIdx.x + blockIdx.x * blockDim.x;
      const int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x >= VOLUME_X || y >= VOLUME_Y) return;

      const int from_x = x + trans.x, from_y = y + trans.y;
      int from_z = trans.z;

      const int from_step = from_volume.step * VOLUME_Y / sizeof(short2);
      const int to_step = to_volume.step * VOLUME_Y / sizeof(short2);

      short2* from_pos = from_x + from_volume.ptr (from_y) + from_z * from_step;
      short2* to_pos = x + to_volume.ptr (y);

      const bool skip = (from_x < 0 || from_y < 0 ||
                         from_x >= VOLUME_X || from_y >= VOLUME_Y);

      short2 sentinel, *from;
      pack_tsdf (0.0f, 0, sentinel);

      for(int z = 0; z < VOLUME_Z; ++z, ++from_z){

        if( skip || from_z < 0 || from_z >= VOLUME_Z ) from = &sentinel;
        else from = from_pos;
        
        __syncthreads(); //saves about a ms

        *to_pos = *from;

        from_pos += from_step;
        to_pos += to_step;
      }
    }
  }
}

void rxkinfu::device::shiftTsdfVolume(const int3& trans,
                                      PtrStep<short2> from_volume,
                                      PtrStep<short2> to_volume) {
  dim3 block (VOLUME_X, 1);
  dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

  shiftTsdf<<<grid, block>>>(trans, from_volume, to_volume);    

  cudaSafeCall (cudaGetLastError());
  cudaSafeCall (cudaDeviceSynchronize());
}

namespace rxkinfu {
  namespace device {

    /** \brief Linearly interpolate tsdf values.
      *
      * \param[in] t the interpolation value in [0,1]
      * \param[in] a the first tsdf value pair
      * \param[in] b the second tsdf value pair
      * \param[out] r the interpolated distance and weight are stored here
      * \param[in] checkvalid check weights for invalids
      */
    template<bool checkvalid>
    __device__ __forceinline__ void
    linterp(float t, const float2 &a, const float2 &b, float2 &r) {

      float ad = a.x, aw = a.y, bd = b.x, bw = b.y;

      if (!checkvalid || ((aw != 0) && (bw != 0)) || ((aw == 0) && (bw == 0))) {
        r.x = (1-t) * ad + t * bd;
        r.y = (1-t) * aw + t * bw;
      } else if ((aw != 0) && (t < 0.5f)) {
        r.x = ad;
        r.y = aw;
      } else if ((bw != 0) && (t > 0.5f)) {
        r.x = bd;
        r.y = bw;
      } else {
        r.x = 0;
        r.y = 0;
      }
    }

    /** \brief Linearly interpolate tsdf cells.
      *
      * \param[in] t the interpolation value in [0,1]
      * \param[in] a the first tsdf cell
      * \param[in] b the second tsdf cell
      * \param[out] r the interpolated distance and weight are stored here
      * \param[in] checkvalid check weights for invalids
      */
    template<bool checkvalid>
    __device__ __forceinline__ void
   linterp(float t, const short2 &a, const short2 &b, float2 &r) {

      float ad, bd;
      int aw, bw;

      unpack_tsdf(a, ad, aw);
      unpack_tsdf(b, bd, bw);

      linterp<checkvalid>(t, make_float2(ad, aw), make_float2(bd, bw), r);
    }


    /** \brief trilinearly interpolate in a tsdf volume.
     *
     * \param[in] from_volume the volume in which to interpolate
     * \param[in] from_step z-step in from_volume
     * \param[in] point the 3D point at which to interpolate in grid cell units
     * \param[in] checknn first check if nearest neighbor is a sentinel
     * \param[out] tsdf the interpolated distance value is written here
     * \param[out] tsdf the interpolated weight is written here
     */
    template<bool checknn, bool checkvalid>
    __device__ __forceinline__ void
    trilinterp(PtrStep<short2> &from_volume, const int from_step,
               const float3 &point, float &tsdf, int &weight) {

      int3 g, g2;
      short2 *from_pos;
      float3 diff;
      float2 buf[4];

      //default to uninitialized
      tsdf = 0.0f; weight = 0;

      //grid coords of neighborhood upper left corner
      g.x = __float2int_rd (point.x);
      g.y = __float2int_rd (point.y);
      g.z = __float2int_rd (point.z);
          
      //it's about 10ms faster to check the whole neighborhood than to check
      //each neighbor addr
      if (g.x >= 0 && g.y >= 0 && g.z >= 0 &&
          g.x < VOLUME_X - 1 && g.y < VOLUME_Y - 1 && g.z < VOLUME_Z - 1) {
        
        if (checknn) {
          
          //first check if nn is empty
          
          //seems to save an extra 5-10ms 
          
          //while it seems like this would mean 5 total fetches if the nn
          //is not empty, it is not a win to cache the nn value explicitly
          //here, probably it will be in cache when it is
          //read again below anyway.
          
          g2.x = __float2int_rn (point.x);
          g2.y = __float2int_rn (point.y);
          g2.z = __float2int_rn (point.z);
          
          from_pos = g2.x + from_volume.ptr (g2.y) + g2.z * from_step;
          unpack_tsdf(*from_pos, tsdf, weight);
       }

        if ((!checknn) || ((weight != 0) && (tsdf != 1.0f))) {
          
          //interpolate in 2x2x2 neighborhood
          
          diff.x = point.x - g.x;
          diff.y = point.y - g.y;
          diff.z = point.z - g.z;
              
          for (int i = 0; i < 4; i++) {
            from_pos = 
              g.x + 
              from_volume.ptr (g.y + (i&1)) +
              + (g.z + ((i>>1)&1)) * from_step;
            linterp<checkvalid>(diff.x, *from_pos, *(from_pos+1), buf[i]);
          }
              
          for (int i = 0; i < 2; i++)
            linterp<checkvalid>(diff.y, buf[2*i], buf[2*i+1], buf[i]);
          
          linterp<checkvalid>(diff.z, buf[0], buf[1], buf[0]);
          
          tsdf = buf[0].x;
          weight = min(__float2int_rn (buf[0].y), Tsdf::MAX_WEIGHT);
          
          //clamp normalized tsdf
          if(tsdf >= 0.0f) tsdf = fmin(1.0f, tsdf);
          else tsdf = fmax(-1.0f, tsdf);
        }
      }
    }
  }
}

namespace rxkinfu {
  namespace device {

    template<bool checknn, bool checkvalid>
    __device__ __forceinline__ void
    transformTsdf(const float3 trans, const Mat33 rot,
                  PtrStep<short2> from_volume,
                  PtrStep<short2> to_volume) {

      // Graveyard of attempted optimizations:
      //
      // * save 2x2x2 neighborhood in a thread-local buffer, then for z>0 shift
      //   the buffer geometrically and fetch only the newly opened cells
      //
      // * various shared memory explicit caching schemes
      //
      // * nearest-neighbor only (fast but quality terrible)
      //
      // * nearest and 2nd nearest neighbor only (quality terrible)
  
      const int x = threadIdx.x + blockIdx.x * blockDim.x;
      const int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x >= VOLUME_X || y >= VOLUME_Y)
        return;

      short2 *to_pos = to_volume.ptr (y) + x;

      const int from_step = from_volume.step * VOLUME_Y / sizeof(short2);
      const int to_step = to_volume.step * VOLUME_Y / sizeof(short2);

      float3 point;

      float tsdf = 0.0f;
      int weight = 0;

      for(int z = 0; z < VOLUME_Z; ++z) {

        point = (rot * make_float3(x, y, z)) + trans;

        trilinterp<checknn, checkvalid>(from_volume, from_step, point,
                   tsdf, weight);

        pack_tsdf (tsdf, weight, *to_pos);

        to_pos += to_step;
      }
    }

    __global__ void
    transformTsdfFF(const float3 trans, const Mat33 rot,
                    PtrStep<short2> from_volume,
                    PtrStep<short2> to_volume) {
      transformTsdf<false,false>(trans, rot, from_volume, to_volume);
    }

    __global__ void
    transformTsdfFT(const float3 trans, const Mat33 rot,
                    PtrStep<short2> from_volume,
                    PtrStep<short2> to_volume) {
      transformTsdf<false,true>(trans, rot, from_volume, to_volume);
    }

    __global__ void
    transformTsdfTF(const float3 trans, const Mat33 rot,
                    PtrStep<short2> from_volume,
                    PtrStep<short2> to_volume) {
      transformTsdf<true,false>(trans, rot, from_volume, to_volume);
    }

    __global__ void
    transformTsdfTT(const float3 trans, const Mat33 rot,
                    PtrStep<short2> from_volume,
                    PtrStep<short2> to_volume) {
      transformTsdf<true,true>(trans, rot, from_volume, to_volume);
    }
  }
}

void
rxkinfu::device::transformTsdfVolume(const float3& trans, const Mat33& rot,
                                     PtrStep<short2> from_volume,
                                     PtrStep<short2> to_volume,
                                     const bool checknn,
                                     const bool checkvalid) {
  dim3 block (VOLUME_X, 1);
  dim3 grid (divUp (VOLUME_X, block.x), divUp (VOLUME_Y, block.y));

  if (!checknn && !checkvalid)
    transformTsdfFF<<<grid, block>>>(trans, rot, from_volume, to_volume);
  else if (!checknn && checkvalid)
    transformTsdfFT<<<grid, block>>>(trans, rot, from_volume, to_volume);
  else if (checknn && !checkvalid)
    transformTsdfTF<<<grid, block>>>(trans, rot, from_volume, to_volume);
  else if (checknn && checkvalid)
    transformTsdfTT<<<grid, block>>>(trans, rot, from_volume, to_volume);

  cudaSafeCall (cudaGetLastError());
  cudaSafeCall (cudaDeviceSynchronize());
}

