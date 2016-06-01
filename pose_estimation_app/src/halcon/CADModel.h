/*
 * CADModel.h
 *
 *  Created on: Oct 2, 2013
 *      Author: thomas
 */

#ifndef CADMODEL_H_
#define CADMODEL_H_

#include <halconcpp/HalconCpp.h>
#include <pcl/point_types.h>

using namespace HalconCpp;

namespace perception {

class CADModel {
public:
	CADModel();
	virtual ~CADModel();





private:

	// PCL Model
	pcl::PointCloud<pcl::PointXYZ> model;

	//Model name
	std::string name;

	//Halcon model
	HTuple hModel;
};
} /* namespace perception */
#endif /* CADMODEL_H_ */
