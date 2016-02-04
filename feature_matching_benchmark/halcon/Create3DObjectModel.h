/********************************************************************************************************************
 *
 * @file		Create3DObjectModel.h
 * @author		Thomas Sølund (thso@teknologisk.dk)
 * @date		2013-04-22
 * @version		1.0
 * @brief		Class for creating Halcon 3D object models
 *
*********************************************************************************************************************/
#ifndef CREATE3DOBJECTMODEL_H_
#define CREATE3DOBJECTMODEL_H_

#include <halconcpp/HalconCpp.h>

#include "ObjectModelAttribute.hpp"

using namespace HalconCpp;

namespace perception {

class Create3DObjectModel {
public:
	Create3DObjectModel();
	virtual ~Create3DObjectModel();

	/* Create a model from points, XYZ map or disparity map */
    void createModel(const HObject& X, const HObject& Y, const HObject& Z);
    void createModelFromPoints(const HTuple &X, const HTuple &Y, const HTuple &Z);
    void createModel2(const HObject &depth_image);
	void createModelFromDisparity(HObject& X, HObject& Y, HObject& Z);

	/*Create 3D primitives */
	void createCylinderModel(HTuple& Pose, HTuple& Radius, HTuple& MinExtend, HTuple& MaxExtend);
	void createBoxModel(HTuple& Pose, HTuple LengthX, HTuple LengthY, HTuple LengthZ);
	void createPlaneModel(HTuple& Pose, HTuple& XExtend, HTuple& YExtend);
	void createSphereModel(HTuple& Pose, HTuple& Radius);
	void createSphereCenterModel(HTuple& Radius, HTuple& x, HTuple& y ,HTuple& z);

	/** Write ObjectModel3D to a file
	  * @param path Storage path
	  * @param FileType CAD file type (om3, dxf, off, ply, obj, stl, stl_binary, stl_ascii)
	*/
	void writeModel(std::string path, std::string FileType);
	void writeModel(std::string path, std::string FileType, HTuple& model);
	void showObjectModel3D(HTuple window_width, HTuple window_height);
	void readModel(std::string path);

	void triangulateModel(HTuple& TriangulationMethod, ObjectModel3DTriangulationParams& Params, HTuple& TriangulatedObjectModel3D_out, HTuple& Information_out);

	void computeSmallestBoundingBox(HTuple Type, HTuple& Pose, HTuple& Length1, HTuple& Length2, HTuple& Length3);
	void computeSmallestBoundingSphere(HTuple& CenterPoint_out, HTuple& Radius_out);
	HTuple computeMaxDiameter();
	void computeArea(HTuple& Area_out);
	HTuple computePrincipalAxis();
	HTuple computeMeanPoint();
	HTuple compute2ndCentralMoment();
	void computeSurfaceNormals();
	void sampleObjectModel(HTuple SampleDistance);
	void affineTransformation(HTuple& TransformationPose, HTuple& TransformedModel_out);
	void rigidTransformation(HTuple& TransformationPose, HTuple& TransformedModel_out);
	void computeConvexHullOfObjectModel(HTuple& ConvexHullObjectModel_out);

	bool hasPoints();
	bool hasPointNormals();
	bool hasTriangles();
	bool hasFaces();
	bool hasLines();
	bool hasXYZMapping();
	bool hasSurfaceBasedMatchingData();
	bool hasSegmentationData();
	bool hasPrimitiveData();

	int getNumberOfPoints();
	int getNumberOfTriangles();
	int getNumberOfFaces();
	int getNumberOfLines();
	void getBoundingBox(double& min_x, double& min_y, double& min_z, double& max_x, double& max_y, double& max_z);

    HTuple get3DObjectModelAttribute(HTuple ParamName) const;
    HTuple get3DObjectModel(void) const
    {
        return mModel;
    }

private:
	HTuple mModel;
};

} /* namespace perception */
#endif /* CREATE3DOBJECTMODEL_H_ */
