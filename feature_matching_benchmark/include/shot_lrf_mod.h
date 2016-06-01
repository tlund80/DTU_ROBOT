/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id$
 */

#ifndef PCL_FEATURES_SHOT_LRF_MOD_H_
#define PCL_FEATURES_SHOT_LRF_MOD_H_

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief SHOTLocalReferenceFrameEstimation estimates the Local Reference Frame used in the calculation
    * of the (SHOT) descriptor.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *     
    * This is a modified version here the z-axis is never directed anti-parallel with the surface normal
    *
    * \author Samuele Salti, Federico Tombari
    * \author Anders Glent Buch (surface normal orientation mod)
    * \ingroup features
    */
  template<typename PointInT, typename PointOutT = ReferenceFrame>
  class SHOTLocalReferenceFrameEstimationMod : public Feature<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<SHOTLocalReferenceFrameEstimationMod<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const SHOTLocalReferenceFrameEstimationMod<PointInT, PointOutT> > ConstPtr;
      /** \brief Constructor */
      SHOTLocalReferenceFrameEstimationMod ()
      {
        feature_name_ = "SHOTLocalReferenceFrameEstimationMod";
      }
      
      /** \brief Empty destructor */
      virtual ~SHOTLocalReferenceFrameEstimationMod () {}

    protected:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      //using Feature<PointInT, PointOutT>::searchForNeighbors;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::search_parameter_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Computes disambiguated local RF for a point index
        * \param[in] index the index
        * \param[out] rf reference frame to compute
        */
      float
      getLocalRF (const int &index, Eigen::Matrix3f &rf);
      float
      getLocalRFMOD (const int &index, Eigen::Matrix3f &rf);
      float
      getLocalRFNAC (const int &index, Eigen::Matrix3f &rf);

      /** \brief Feature estimation method.
        * \param[out] output the resultant features
        */
      virtual void
      computeFeature (PointCloudOut &output);
  };
}

#include <utility>

//////////////////////////////////////////////////////////////////////////////////////////////
// Compute a local Reference Frame for a 3D feature; the output is stored in the "rf" matrix
template<typename PointInT, typename PointOutT> float
pcl::SHOTLocalReferenceFrameEstimationMod<PointInT, PointOutT>::getLocalRF (const int& current_point_idx, Eigen::Matrix3f &rf) {
//    return (getLocalRFNAC (current_point_idx, rf));
    return (getLocalRFMOD (current_point_idx, rf));
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Compute a local Reference Frame for a 3D feature; the output is stored in the "rf" matrix
template<typename PointInT, typename PointOutT> float
pcl::SHOTLocalReferenceFrameEstimationMod<PointInT, PointOutT>::getLocalRFMOD (const int& current_point_idx, Eigen::Matrix3f &rf)
{
  const Eigen::Vector4f& central_point = (*input_)[current_point_idx].getVector4fMap ();
  std::vector<int> n_indices;
  std::vector<float> n_sqr_distances;

  this->searchForNeighbors (current_point_idx, search_parameter_, n_indices, n_sqr_distances);
  
  /**
   * AGB mod: Remove all points with normal in opposing direction as the center point
   */
  std::vector<int> n_indices_mod;
  std::vector<float> n_sqr_distances_mod;
  n_indices_mod.reserve (n_indices.size ());
  n_sqr_distances_mod.reserve (n_sqr_distances.size ());
  const Eigen::Vector3f& central_normal = (*input_)[current_point_idx].getNormalVector3fMap ();
  for (size_t i_idx = 0; i_idx < n_indices.size (); ++i_idx)
  {
    Eigen::Vector3f n = surface_->points[n_indices[i_idx]].getNormalVector3fMap ();
    if (central_normal.dot (n) > 0.0f)
    {
        n_indices_mod.push_back (n_indices[i_idx]);
        n_sqr_distances_mod.push_back (n_sqr_distances[i_idx]);
    }
  }
  n_indices = n_indices_mod;
  n_sqr_distances = n_sqr_distances_mod;
  
  /**
   * End AGB mod
   */

  Eigen::Matrix<double, Eigen::Dynamic, 4> vij (n_indices.size (), 4);

  Eigen::Matrix3d cov_m = Eigen::Matrix3d::Zero ();

  double distance = 0.0;
  double sum = 0.0;

  int valid_nn_points = 0;

  for (size_t i_idx = 0; i_idx < n_indices.size (); ++i_idx)
  {
    Eigen::Vector4f pt = surface_->points[n_indices[i_idx]].getVector4fMap ();
    if (pt.head<3> () == central_point.head<3> ())
          continue;

    // Difference between current point and origin
    vij.row (valid_nn_points).matrix () = (pt - central_point).cast<double> ();
    vij (valid_nn_points, 3) = 0;

    distance = search_parameter_ - sqrt (n_sqr_distances[i_idx]);

    // Multiply vij * vij'
    cov_m += distance * (vij.row (valid_nn_points).head<3> ().transpose () * vij.row (valid_nn_points).head<3> ());

    sum += distance;
    valid_nn_points++;
  }

  if (valid_nn_points < 5)
  {
    //PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Neighborhood has less than 5 vertexes. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
    rf.setConstant (std::numeric_limits<float>::quiet_NaN ());

    return (std::numeric_limits<float>::max ());
  }

  cov_m /= sum;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (cov_m);

  const double& e1c = solver.eigenvalues ()[0];
  const double& e2c = solver.eigenvalues ()[1];
  const double& e3c = solver.eigenvalues ()[2];

  if (!pcl_isfinite (e1c) || !pcl_isfinite (e2c) || !pcl_isfinite (e3c))
  {
    //PCL_ERROR ("[pcl::%s::getLocalRF] Warning! Eigenvectors are NaN. Aborting Local RF computation of feature point (%lf, %lf, %lf)\n", "SHOTLocalReferenceFrameEstimation", central_point[0], central_point[1], central_point[2]);
    rf.setConstant (std::numeric_limits<float>::quiet_NaN ());

    return (std::numeric_limits<float>::max ());
  }

  // Disambiguation
  Eigen::Vector4d v1 = Eigen::Vector4d::Zero ();
  Eigen::Vector4d v3 = Eigen::Vector4d::Zero ();
  v1.head<3> ().matrix () = solver.eigenvectors ().col (2);
  v3.head<3> ().matrix () = solver.eigenvectors ().col (0);

  int plusNormal = 0, plusTangentDirection1=0;
  for (int ne = 0; ne < valid_nn_points; ne++)
  {
    double dp = vij.row (ne).dot (v1);
    if (dp >= 0)
      plusTangentDirection1++;

    dp = vij.row (ne).dot (v3);
    if (dp >= 0)
      plusNormal++;
  }

  //TANGENT
  plusTangentDirection1 = 2*plusTangentDirection1 - valid_nn_points;
  if (plusTangentDirection1 == 0)
  {
        int points = 5; //std::min(valid_nn_points*2/2+1, 11);
        int medianIndex = valid_nn_points/2;

        for (int i = -points/2; i <= points/2; i++)
            if ( vij.row (medianIndex - i).dot (v1) > 0)
                plusTangentDirection1 ++;

        if (plusTangentDirection1 < points/2+1)
            v1 *= - 1;
    } 
  else if (plusTangentDirection1 < 0)
    v1 *= - 1;

  //Normal
  plusNormal = 2*plusNormal - valid_nn_points;
  if (plusNormal == 0)
  {
        int points = 5; //std::min(valid_nn_points*2/2+1, 11);
        int medianIndex = valid_nn_points/2;

        for (int i = -points/2; i <= points/2; i++)
            if ( vij.row (medianIndex - i).dot (v3) > 0)
                plusNormal ++;

        if (plusNormal < points/2+1)
            v3 *= - 1;
    } else if (plusNormal < 0)
    v3 *= - 1;

  rf.row (0).matrix () = v1.head<3> ().cast<float> ();
  rf.row (2).matrix () = v3.head<3> ().cast<float> ();
  rf.row (1).matrix () = rf.row (2).cross (rf.row (0));

  return (0.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Compute a local Reference Frame for a 3D feature; the output is stored in the "rf" matrix
template<typename PointInT, typename PointOutT> float
pcl::SHOTLocalReferenceFrameEstimationMod<PointInT, PointOutT>::getLocalRFNAC (const int& current_point_idx, Eigen::Matrix3f &rf)
{
  // Small value threshold
  const float tol = 1e-5f;
    
  // Find neighbors
  std::vector<int> idx;
  std::vector<float> distsq;
  this->searchForNeighbors (current_point_idx, search_parameter_, idx, distsq);
  
  // Current point and normal
  const Eigen::Vector3f& current_point = input_->points[current_point_idx].getVector3fMap ();
  const Eigen::Vector3f& current_normal = input_->points[current_point_idx].getNormalVector3fMap ();
  
  // Not valid
  if (current_point.hasNaN () || current_normal.hasNaN ()) {
      rf = Eigen::Matrix3f::Identity ();
      return (std::numeric_limits<float>::max ());
    }
  
  // Find the normal angle centroid as a vector from the current point
  Eigen::Vector3f normal_angle_centroid_dir (0.0f, 0.0f, 0.0f);
  for(size_t i = 0; i < idx.size (); ++i) {
      const Eigen::Vector3f& neighbor_point = surface_->points[idx[i]].getVector3fMap ();
      const Eigen::Vector3f& neighbor_normal = surface_->points[idx[i]].getNormalVector3fMap ();
      
      if (neighbor_point.hasNaN () || neighbor_normal.hasNaN ())
          continue;
      
      // Skip if neighbor normal is anti-parallel with current
      const float dot = current_normal.dot (neighbor_normal);
      if (dot < 0.0f)
          continue;
      
      normal_angle_centroid_dir += (1.0f - dot) * (neighbor_point - current_point);
  }
  
  // RF cannot be determined
  if (normal_angle_centroid_dir.norm () < tol) {
    rf = Eigen::Matrix3f::Identity ();
    return (std::numeric_limits<float>::max ());
  }
  
  // The z-axis is defined as the surface normal
  rf.col (2) = current_normal;
  
  // Compute x-axis as the vector projection to the tangent plane, normalized
  rf.col (0) = (normal_angle_centroid_dir - normal_angle_centroid_dir.dot (current_normal) * current_normal).normalized ();
  
  // Compute y as cross (z, x)
  rf.col (1) = rf.col (2).cross (rf.col (0));

  return (0.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SHOTLocalReferenceFrameEstimationMod<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  //check whether used with search radius or search k-neighbors
  if (this->getKSearch () != 0)
  {
    PCL_ERROR(
      "[pcl::%s::computeFeature] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch( radius ) to use this class.\n",
      getClassName().c_str ());
    return;
  }
  tree_->setSortedResults (true);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Matrix3f rf;
    if (getLocalRF ((*indices_)[i], rf) == std::numeric_limits<float>::max ())
    {
      output.is_dense = false;
    }
    // TODO: For the MOD
    PointOutT& output_rf = output[i];
    for (int d = 0; d < 3; ++d)
    {
      output_rf.x_axis[d] = rf.row (0)[d];
      output_rf.y_axis[d] = rf.row (1)[d];
      output_rf.z_axis[d] = rf.row (2)[d];
    }
    // TODO: For the NAC
//    std::memcpy (output[i].rf, rf.data (), 9 * sizeof (float));
  }
}

#endif

