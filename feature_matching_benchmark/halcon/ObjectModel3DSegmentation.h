/*
 * ObjectModel3DSegmentation.h
 *
 *  Created on: Jun 5, 2013
 *      Author: thomas
 */

#ifndef OBJECTMODEL3DSEGMENTATION_H_
#define OBJECTMODEL3DSEGMENTATION_H_

#include <ros/ros.h>
#include <halconcpp/HalconCpp.h>

#include "ObjectModelAttribute.hpp"

using namespace HalconCpp;

namespace perception {

class ObjectModel3DSegmentation {
public:
	ObjectModel3DSegmentation();
	virtual ~ObjectModel3DSegmentation();

    /**
     * Set query point set
     * @param query query point set
     */
	void prepareObjectModelForSegmentation(int max_area_holes);

    /**
     * Set query point set
     * @param query query point set
     */
	void SegmentObjectModel3D(ObjectModelSegmentationParams params, HTuple& ObjectModelArray_out);

    /**
     * Set query point set
     * @param query query point set
     */
	void Fit3DPrimitivesToObjectModel(HTuple Primitive3d_type, HTuple fitting_algorithm, Primitive3DFittingParams params,  HTuple& ObjectModelArray_out);



private:
	HTuple mModel;
};

} /* namespace perception */
#endif /* OBJECTMODEL3DSEGMENTATION_H_ */
