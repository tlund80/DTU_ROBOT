/********************************************************************************************************************
 *
 * @file		ModelCreationParameters.hpp
 * @author		Thomas Sølund(thso@teknologisk.dk)
 * @date		2013-01-24
 * @version		1.0
 * @brief		Creation parameters of Halcon object models.
 *
*********************************************************************************************************************/

#ifndef MODELCREATIONPARAMETERS_HPP_
#define MODELCREATIONPARAMETERS_HPP_

#include <halconcpp/HalconCpp.h>

using namespace HalconCpp;

//todo class implementation for parameter restriction

//* ScaledShapeModel2DCreationParameters
/**
* Model parameters for creation of a 2D Scaled Shape Model
* for scale invariant matching
*/

struct SurfaceModelCreationParameters
{
	SurfaceModelCreationParameters() :
					RelSamplingDistance(HTuple(0.03)),
					model_invert_normals(HTuple("false")),
                    pose_ref_rel_sampling_distance(HTuple(0.01)),
					feat_step_size_rel(HTuple(0.03)),
					feat_angle_resolution(HTuple(30))
					{}
	HTuple ObjectModel3D;
	HTuple RelSamplingDistance;
	HTuple model_invert_normals;
    HTuple pose_ref_rel_sampling_distance;
	HTuple feat_step_size_rel;
	HTuple feat_angle_resolution;
};

#endif /* MODELCREATIONPARAMETERS_HPP_ */
