/********************************************************************************************************************
 *
 * @file		SurfaceModelCreator.cpp
 * @author		Thomas Sølund (thso@teknologisk.dk)
 * @date		2013-04-22
 * @version		1.0
 * @brief		Class for creating Halcon surface models
 *
*********************************************************************************************************************/
#include "SurfaceModelCreator.h"
#include <pcl/console/print.h>

namespace perception {

SurfaceModelCreator::SurfaceModelCreator() {
	// TODO Auto-generated constructor stub

}

SurfaceModelCreator::~SurfaceModelCreator() {
	// TODO Auto-generated destructor stub
	try
	{
		ClearSurfaceModel(mModelID);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelCreator::~SurfaceModelCreator()", except.ErrorText().Text());
	}
}

void SurfaceModelCreator::createModel(){

	HTuple GenParamName;
	GenParamName.Append("model_invert_normals");
	GenParamName.Append("pose_ref_rel_sampling_distance");
	GenParamName.Append("feat_step_size_rel");
	GenParamName.Append("feat_angle_resolution");


	HTuple GenParamValue;
	GenParamValue.Append(mParam.model_invert_normals);
	GenParamValue.Append(mParam.pose_ref_rel_sampling_distance);
	GenParamValue.Append(mParam.feat_step_size_rel);
	GenParamValue.Append(mParam.feat_angle_resolution);

	try
	{
		CreateSurfaceModel(mParam.ObjectModel3D,mParam.RelSamplingDistance,
					       GenParamName, GenParamValue,&mModelID);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelCreator::createModel()", except.ErrorText().Text());
	}
}

void SurfaceModelCreator::getCenterOfSurfaceModel(HTuple& center_out) const{

	try
	{
		GetSurfaceModelParam(mModelID,"center",&center_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelCreator::getCenterOfSurfaceModel()", except.ErrorText().Text());
	}

}

void SurfaceModelCreator::getBounding_boxOfSurfaceModel(HTuple& Bounding_box_out) const{

	try
	{
		GetSurfaceModelParam(mModelID,"bounding_box1",&Bounding_box_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getBounding_boxOfSurfaceModel()", except.ErrorText().Text());
	}

}

void SurfaceModelCreator::getDiameterSurfaceModel(HTuple& Diameter_out) const{

	try
	{
		GetSurfaceModelParam(mModelID,"diameter",&Diameter_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getDiameterSurfaceModel()", except.ErrorText().Text());
	}

}

void SurfaceModelCreator::saveModel(const std::string& savePath){

	try
		{
		WriteSurfaceModel(mModelID, HTuple(savePath.c_str()));
		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in SurfaceModelCreator::saveModel()", except.ErrorText().Text());
		}

}

void SurfaceModelCreator::showModels()
{
	try
	{
		HTuple width = 1000;
		HTuple height = 800;

		HWindow w(0, 0, (Hlong)width, (Hlong)height);

		DispObjectModel3d(w,mModelID,HTuple(),HTuple(),HTuple("colored"),HTuple(3));

		w.Click();
		w.CloseWindow();
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelCreator::showDetectedModels()", except.ErrorText().Text());
	}
}

void SurfaceModelCreator::setParameters(const SurfaceModelCreationParameters& param){

	mParam = param;
}

} /* namespace perception */
