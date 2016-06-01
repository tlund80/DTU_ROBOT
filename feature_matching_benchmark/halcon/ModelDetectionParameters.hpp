/********************************************************************************************************************
 *
 * @file		ModelCreationParameters.hpp
 * @author		Thomas Sølund(thso@teknologisk.dk)
 * @date		2013-01-24
 * @version		1.0
 * @brief		Creation parameters of Halcon object models.
 *
*********************************************************************************************************************/

#ifndef MODELDETECTIONPARAMETERS_HPP_
#define MODELDETECTIONPARAMETERS_HPP_

#include <halconcpp/HalconCpp.h>

using namespace HalconCpp;

//todo class implementation for parameter restriction

//* SurfaceModelDetectionParameters
/**
* Model parameters for creation of a 2D Surface Model
*
*/

struct SurfaceModelDetectionParameters
{
	SurfaceModelDetectionParameters() :
					RelSamplingDistance(HTuple(0.03)),
					ReturnResultHandle(HTuple("true")),
					KeyPointFraction(HTuple(0.5)),
					MinScore(HTuple(0)),
					num_matches(HTuple(1)),
					max_overlap_dist_rel(HTuple(0.5)),
					sparse_pose_refinement(HTuple("true")),
					score_type(HTuple("model_point_fraction")),
					pose_ref_use_scene_normals(HTuple("false")),
					dense_pose_refinement(HTuple("true")),
					pose_ref_num_steps(HTuple(5)),
					pose_ref_sub_sampling(HTuple(2)),
					pose_ref_dist_threshold_rel(HTuple(0.1)),
					pose_ref_scoring_dist_rel(HTuple(0.005))
					{}
	HTuple ObjectModel3D;
	HTuple RelSamplingDistance;
	HTuple KeyPointFraction;
	HTuple MinScore;
	HTuple ReturnResultHandle;
	//HTuple GenParamName;
	//HTuple GenParamValue;
	HTuple num_matches;
	HTuple max_overlap_dist_rel;
	//HTuple max_overlap_dist_abs;
	HTuple sparse_pose_refinement;
	HTuple score_type;
	HTuple pose_ref_use_scene_normals;
	HTuple dense_pose_refinement;
	HTuple pose_ref_num_steps;
	HTuple pose_ref_sub_sampling;
	HTuple pose_ref_dist_threshold_rel;
	HTuple pose_ref_scoring_dist_rel;
	//HTuple pose_ref_scoring_dist_abs;



};

#endif /* MODELDETECTIONPARAMETERS_HPP_ */
