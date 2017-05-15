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
 *  $Id: tsdf_volume.h 6459 2012-07-18 07:50:37Z dpb $
 */


#ifndef MVKINFU_TSDF_VOLUME_HOST_H_
#define MVKINFU_TSDF_VOLUME_HOST_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>

namespace rxkinfu
{
  template <typename VoxelT, typename WeightT>
  class TsdfVolumeHost
  {
  public:

    typedef boost::shared_ptr<TsdfVolumeHost<VoxelT, WeightT> > Ptr;
    typedef boost::shared_ptr<const TsdfVolumeHost<VoxelT, WeightT> > ConstPtr;

    // typedef Eigen::Matrix<VoxelT, Eigen::Dynamic, 1> VoxelTVec;
    typedef Eigen::VectorXf VoxelTVec;

    /** \brief Structure storing voxel grid resolution, volume size (in mm) and element_size of stored data */
    struct Header
    {
      Eigen::Vector3i resolution;
      Eigen::Vector3f volume_size;
      int volume_element_size, weights_element_size;

      Header ()
        : resolution (0,0,0),
          volume_size (0,0,0),
          volume_element_size (sizeof(VoxelT)),
          weights_element_size (sizeof(WeightT))
      {};

      Header (const Eigen::Vector3i &res, const Eigen::Vector3f &size)
        : resolution (res),
          volume_size (size),
          volume_element_size (sizeof(VoxelT)),
          weights_element_size (sizeof(WeightT))
      {};

      size_t
      getVolumeSize () const { return resolution[0] * resolution[1] * resolution[2]; };

      friend std::ostream&
      operator << (std::ostream& os, const Header& h)
      {
        os << "(resolution = " << h.resolution.transpose() << ", volume size = " << h.volume_size.transpose() << ")";
        return (os);
      }

      public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };

    ////////////////////////////////////////////////////////////////////////////////////////
    // Constructors

    /** \brief Default constructor */
    TsdfVolumeHost ()
      : volume_ (new std::vector<VoxelT>),
        weights_ (new std::vector<WeightT>)
    {};

    /** \brief Constructor loading data from file */
    TsdfVolumeHost (const std::string &filename)
      : volume_ (new std::vector<VoxelT>),
        weights_ (new std::vector<WeightT>)
    {
      if (load (filename))
        std::cout << "done [" << size() << "]" << std::endl;
      else
        std::cout << "error!" << std::endl;
    };

    /** \brief Set the header directly. Useful if directly writing into volume and weights */
    virtual void
    setHeader (const Eigen::Vector3i &resolution, const Eigen::Vector3f &volume_size) {
      header_ = Header (resolution, volume_size);
      if (volume_->size() != this->size())
        pcl::console::print_warn ("[TsdfVolumeHost::setHeader] Header volume size (%d) doesn't fit underlying data size (%d)", volume_->size(), size());
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    // Storage and element access

    /** \brief Loads volume from file */
    virtual bool
    load (const std::string &filename, bool binary = true);

    /** \brief Saves volume to file */
    virtual bool
    save (const std::string &filename = "tsdf_volume.dat", bool binary = true) const;

    /** \brief Returns overall number of voxels in grid */
    size_t
    size () const { return header_.getVolumeSize(); };

    /** \brief Returns the volume size in mm */
    const Eigen::Vector3f &
    volumeSize () const { return header_.volume_size; };

    /** \brief Returns the size of one voxel in mm */
    virtual Eigen::Vector3f
    voxelSize () const {
      Eigen::Array3f res = header_.resolution.array().template cast<float>();
      return header_.volume_size.array() / res;
    };

    /** \brief Returns the voxel grid resolution */
    const Eigen::Vector3i &
    gridResolution() const { return header_.resolution; };

    /** \brief Returns constant reference to header */
    const Header &
    header () const { return header_; };

    /** \brief Returns constant reference to the volume std::vector */
    const std::vector<VoxelT> &
    volume () const { return *volume_; };

    /** \brief Returns writebale(!) reference to volume */
    std::vector<VoxelT> &
    volumeWriteable () const { return *volume_; };

    /** \brief Returns constant reference to the weights std::vector */
    const std::vector<WeightT> &
    weights () const { return *weights_; };

    /** \brief Returns writebale(!) reference to volume */
    std::vector<WeightT> &
    weightsWriteable () const { return *weights_; };

    ////////////////////////////////////////////////////////////////////////////////////////
    // Functionality

    /** \brief Converts volume to cloud of TSDF values*/
    virtual void
    convertToTsdfCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const;

    /** \brief Retunrs the 3D voxel coordinate */
    template <typename PointT> void
    getVoxelCoord (const PointT &point, Eigen::Vector3i &voxel_coord)  const;

    /** \brief Retunrs the 3D voxel coordinate and point offset wrt. to the voxel center (in mm) */
    template <typename PointT> void
    getVoxelCoordAndOffset (const PointT &point, Eigen::Vector3i &voxel_coord, Eigen::Vector3f &offset) const;

    /** extracts voxels in neighborhood of given voxel */
    virtual bool
    extractNeighborhood (const Eigen::Vector3i &voxel_coord, int neighborhood_size, VoxelTVec &neighborhood) const;

    /** adds voxel values in local neighborhood */
    virtual bool
    addNeighborhood (const Eigen::Vector3i &voxel_coord, int neighborhood_size, const VoxelTVec &neighborhood, WeightT voxel_weight);

    /** averages voxel values by the weight value */
    virtual void
    averageValues ();

    /** \brief Returns and index for linear access of the volume and weights */
    virtual inline int
    getLinearVoxelIndex (const Eigen::Array3i &indices) const {
      return indices(0) + indices(1) * header_.resolution[0] + indices(2) * header_.resolution[0] * header_.resolution[1];
    }

    /** \brief Returns a vector of linear indices for voxel coordinates given in 3xn matrix */
    virtual inline Eigen::VectorXi
    getLinearVoxelIndinces (const Eigen::Matrix<int, 3, Eigen::Dynamic> &indices_matrix) const  {
      return (Eigen::RowVector3i (1, header_.resolution[0], header_.resolution[0] * header_.resolution[1]) * indices_matrix).transpose();
    }

  protected:

    typedef boost::shared_ptr<std::vector<VoxelT> > VolumePtr;
    typedef boost::shared_ptr<std::vector<WeightT> > WeightsPtr;

    Header header_;
    VolumePtr volume_;
    WeightsPtr weights_;

  private:
    TsdfVolumeHost(const TsdfVolumeHost& other); // Disable
    TsdfVolumeHost& operator=(const TsdfVolumeHost&); // Disable

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

}

#include "tsdf_volume_host.hpp"

#endif
