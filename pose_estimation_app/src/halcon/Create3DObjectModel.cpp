/********************************************************************************************************************
 *
 * @file		Create3DObjectModel.cpp
 * @author		Thomas Sølund (thso@teknologisk.dk)
 * @date		2013-04-22
 * @version		1.0
 * @brief		Class for creating Halcon 3D object models
 *
*********************************************************************************************************************/
#include "Create3DObjectModel.h"
#include <pcl/console/print.h>

namespace perception {

Create3DObjectModel::Create3DObjectModel() {
	// TODO Auto-generated constructor stub


}

Create3DObjectModel::~Create3DObjectModel() {
	// TODO Auto-generated destructor stub
	try
	{
		ClearObjectModel3d(mModel);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::~Create3DObjectModel()\n", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createCylinderModel(HTuple& Pose, HTuple& Radius, HTuple& MinExtend, HTuple& MaxExtend){

	try
	{

		GenCylinderObjectModel3d(Pose,Radius,MinExtend, MaxExtend, &mModel);

	}
	catch (HException& except)
    {
        pcl::console::print_error("%s in Create3DObjectModel::createCylinderModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createBoxModel(HTuple& Pose, HTuple LengthX, HTuple LengthY, HTuple LengthZ){

	try
	{

		GenBoxObjectModel3d(Pose,LengthX,LengthY,LengthZ,&mModel);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::createBoxModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createPlaneModel(HTuple& Pose, HTuple& XExtend, HTuple& YExtend){

	try
	{

		GenPlaneObjectModel3d(Pose,XExtend,YExtend,&mModel);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::createPlaneModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createSphereModel(HTuple& Pose, HTuple& Radius){

	try
	{

		GenSphereObjectModel3d(Pose,Radius,&mModel);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::createSphereModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createSphereCenterModel(HTuple& Radius, HTuple& x, HTuple& y ,HTuple& z){

	try
	{

		GenSphereObjectModel3dCenter(x,y,z,Radius,&mModel);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::createSphereCenterModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createModel(const HObject &X, const HObject &Y, const HObject &Z){

	try
	{

		XyzToObjectModel3d(X,Y,Z,&mModel);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::createModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createModelFromPoints(const HTuple& X, const HTuple& Y, const HTuple& Z){

	try
	{

		GenObjectModel3dFromPoints(X,Y,Z,&mModel);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::createModelFromPoints()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createModel2(const HObject& depth_image){

	HObject X;
	HObject Y;
	HObject Z;
	try
	{
		Decompose3(depth_image,&X,&Y,&Z);
		XyzToObjectModel3d(X,Y,Z,&mModel);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::createModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::createModelFromDisparity(HObject& X, HObject& Y, HObject& Z){

	try
	{
		throw "not implemented yet";
			//DisparityImageToXyz() ,&mModel);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::createModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::triangulateModel(HTuple& TriangulationMethod, ObjectModel3DTriangulationParams& Params, HTuple& TriangulatedObjectModel3D_out, HTuple& Information_out){

	HTuple ParamName;
	HTuple ParamValue;
	if(TriangulationMethod == "greedy"){
        pcl::console::print_info("Object model is triangulated using greedy method!");

		ParamName.Append("greedy_kNN");
		ParamName.Append("greedy_radius_type");
		ParamName.Append("greedy_radius_value");
		ParamName.Append("greedy_neigh_orient_tol");
		ParamName.Append("greedy_neigh_orient_consistent");
		ParamName.Append("greedy_neigh_latitude_tol");
		ParamName.Append("greedy_neigh_vertical_tol");
		ParamName.Append("greedy_hole_filling");
		ParamName.Append("greedy_fix_flips");
		ParamName.Append("greedy_prefetch_neighbors");
		ParamName.Append("greedy_mesh_erosion");
		ParamName.Append("greedy_mesh_dilation");
		ParamName.Append("greedy_remove_small_surfaces");
		ParamName.Append("greedy_timeout");
		ParamName.Append("greedy_suppress_timeout_error");


		ParamValue.Append(Params.greedy_kNN);
		ParamValue.Append(Params.greedy_radius_type);
		ParamValue.Append(Params.greedy_radius_value);
		ParamValue.Append(Params.greedy_neigh_orient_tol);
		ParamValue.Append(Params.greedy_neigh_orient_consistent);
		ParamValue.Append(Params.greedy_neigh_latitude_tol);
		ParamValue.Append(Params.greedy_neigh_vertical_tol);
		ParamValue.Append(Params.greedy_hole_filling);
		ParamValue.Append(Params.greedy_fix_flips);
		ParamValue.Append(Params.greedy_prefetch_neighbors);
		ParamValue.Append(Params.greedy_mesh_erosion);
		ParamValue.Append(Params.greedy_mesh_dilation);
		ParamValue.Append(Params.greedy_remove_small_surfaces);
		ParamValue.Append(Params.greedy_timeout);
		ParamValue.Append(Params.greedy_suppress_timeout_error);



    }else if (TriangulationMethod == "implicit"){
        pcl::console::print_info("Object model is triangulated using implicit method!");

		ParamName.Append("implicit_octree_depth");
		ParamName.Append("implicit_solver_depth");
		ParamName.Append("implicit_min_num_samples");

		ParamValue.Append(Params.implicit_octree_depth);
		ParamValue.Append(Params.implicit_solver_depth);
		ParamValue.Append(Params.implicit_min_num_samples);

	}else if (TriangulationMethod == "polygon_triangulation"){
        pcl::console::print_error("Object model is triangulated using polygon_triangulation method!");
		//All parameters are using default values
	}


	try
	{
		TriangulateObjectModel3d(mModel,TriangulationMethod,ParamName,ParamValue,&TriangulatedObjectModel3D_out,&Information_out);

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::triangulateModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::showObjectModel3D(HTuple window_width, HTuple window_height ){

	try
	{

		HTuple GenName;
		HTuple GenParams;
		GenName.Append("colored");
		GenName.Append("color");
		GenParams.Append(3);
		GenParams.Append("red");
		//HTuple Pose;
		//Pose.Append(0.01); Pose.Append(0); Pose.Append(0); Pose.Append(0); Pose.Append(0); Pose.Append(0); Pose.Append(2);

		HWindow w(0, 0, window_width, window_height);

		DispObjectModel3d(w,mModel,HTuple(),HTuple(),GenName,GenParams);
		w.Click();
		w.Clear();

	//	GetObjectModel3dParams(mModel,)

	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::showObjectModel3D()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::readModel(std::string path){

	HTuple status;
	HTuple GenParamsName;
	HTuple GenParamsValue;

	//GenParamsName.Append("file_type");
	//GenParamsValue.Append("dxf");

	try
		{

			ReadObjectModel3d(HTuple(path.c_str()), HTuple("m"),GenParamsName,GenParamsValue,&mModel,&status);

			std::cout << "ReadObjectModel3d status" << std::endl;
			for(int i = 0; i<status.Length()-1; i++){
				std::cout << " " << (std::string)status[i];
			}
				std::cout << std::endl;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::readModel()", except.ErrorText().Text());
		}
}

void Create3DObjectModel::writeModel(std::string path, std::string FileType){

	HTuple status;
	try
		{
			WriteObjectModel3d(mModel,HTuple(FileType.c_str()),HTuple(path.c_str()),HTuple(),HTuple());

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::readModel()", except.ErrorText().Text());
		}
}

void Create3DObjectModel::writeModel(std::string path, std::string FileType, HTuple& model){

	HTuple status;
	try
		{
			WriteObjectModel3d(model,HTuple(FileType.c_str()),HTuple(path.c_str()),HTuple(),HTuple());

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::readModel()", except.ErrorText().Text());
		}
}

HTuple Create3DObjectModel::get3DObjectModelAttribute(HTuple ParamName) const{

	HTuple ParamValue;
	try
		{
			GetObjectModel3dParams(mModel,ParamName,&ParamValue);

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::readModel()", except.ErrorText().Text());
		}
		return ParamValue;
}

void Create3DObjectModel::computeSmallestBoundingBox(HTuple Type, HTuple& Pose, HTuple& Length1, HTuple& Length2, HTuple& Length3){

	try
		{
			SmallestBoundingBoxObjectModel3d(mModel, Type, &Pose, &Length1, &Length2, &Length3);

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::computeSmallestBoundingBox()", except.ErrorText().Text());
		}
}

void Create3DObjectModel::computeSmallestBoundingSphere(HTuple& CenterPoint_out, HTuple& Radius_out){

	try
		{
			SmallestSphereObjectModel3d(mModel, &CenterPoint_out, &Radius_out);

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::computeSmallestBoundingSphere()", except.ErrorText().Text());
		}
}

HTuple Create3DObjectModel::computeMaxDiameter(){

	HTuple Diameter_out = 0;
	try
		{
			MaxDiameterObjectModel3d(mModel, &Diameter_out);


		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::computeMaxDiameter()", except.ErrorText().Text());
		}

		return Diameter_out;
}

void Create3DObjectModel::computeArea(HTuple& Area_out){

	try
		{
			AreaObjectModel3d(mModel, &Area_out);

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::computeArea()", except.ErrorText().Text());
		}
}

HTuple Create3DObjectModel::computePrincipalAxis(){

	HTuple PrincipalAxis = 0;
	try
		{
			MomentsObjectModel3d(mModel,HTuple("principal_axis"), &PrincipalAxis);


		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::computePrincipalAxis()", except.ErrorText().Text());
		}
		return PrincipalAxis;
}

HTuple Create3DObjectModel::computeMeanPoint(){

	HTuple MeanPoint = 0;
	try
		{
			MomentsObjectModel3d(mModel,HTuple("mean_points"), &MeanPoint);


		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::computeMeanPoint()", except.ErrorText().Text());
		}
		return MeanPoint;
}

HTuple Create3DObjectModel::compute2ndCentralMoment(){

	HTuple Moment = 0;
	try
		{
			MomentsObjectModel3d(mModel,HTuple("central_moment_2_points"), &Moment);


		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::compute2ndCentralMoment()", except.ErrorText().Text());
		}
		return Moment;
}

void Create3DObjectModel::computeConvexHullOfObjectModel(HTuple& ConvexHullObjectModel_out){

	try
		{
			ConvexHullObjectModel3d(mModel, &ConvexHullObjectModel_out);
		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::computeConvexHullOfObjectModel()", except.ErrorText().Text());
		}
}


void Create3DObjectModel::computeSurfaceNormals(){

	HTuple GenParamName;
	HTuple GenParamValue;
	try
		{
			SurfaceNormalsObjectModel3d(mModel,HTuple("mls"), GenParamName, GenParamValue, &mModel);
		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::computeSurfaceNormals()", except.ErrorText().Text());
		}
}

void Create3DObjectModel::sampleObjectModel(HTuple SampleDistance){

	HTuple GenParamName;
	HTuple GenParamValue;
	try
	{
		SampleObjectModel3d(mModel,HTuple("fast"),SampleDistance, GenParamName, GenParamValue, &mModel);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::sampleObjectModel()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::affineTransformation(HTuple& TransformationPose, HTuple& TransformedModel_out){

	HTuple HomMat3D;

	try
	{
		PoseToHomMat3d(TransformationPose, &HomMat3D);
		AffineTransObjectModel3d(mModel,HomMat3D, &TransformedModel_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::affineTransformation()", except.ErrorText().Text());
	}
}

void Create3DObjectModel::rigidTransformation(HTuple& TransformationPose, HTuple& TransformedModel_out){



	try
	{
		RigidTransObjectModel3d(mModel,TransformationPose, &TransformedModel_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in Create3DObjectModel::rigidTransformation()", except.ErrorText().Text());
	}
}


bool Create3DObjectModel::hasPoints(){

	HTuple haspoints = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_points"), &haspoints);

		std::string value = (std::string)haspoints.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasPoints()", except.ErrorText().Text());
		}
		return res;
}

bool Create3DObjectModel::hasPointNormals(){

	HTuple hasPointNormals = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_point_normals"), &hasPointNormals);

		std::string value = (std::string)hasPointNormals.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasPointNormals()", except.ErrorText().Text());
		}
		return res;
}

bool Create3DObjectModel::hasTriangles(){

	HTuple hasTriangles = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_triangles"), &hasTriangles);

		std::string value = (std::string)hasTriangles.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasTriangles()", except.ErrorText().Text());
		}
		return res;
}

bool Create3DObjectModel::hasFaces(){

	HTuple hasFaces = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_faces"), &hasFaces);

		std::string value = (std::string)hasFaces.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasFaces()", except.ErrorText().Text());
		}
		return res;
}

bool Create3DObjectModel::hasLines(){

	HTuple hasLines = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_lines"), &hasLines);

		std::string value = (std::string)hasLines.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasLines()", except.ErrorText().Text());
		}
		return res;
}

bool Create3DObjectModel::hasXYZMapping(){

	HTuple hasXYZMapping = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_xyz_mapping"), &hasXYZMapping);

		std::string value = (std::string)hasXYZMapping.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasXYZMapping()", except.ErrorText().Text());
		}
		return res;
}

bool Create3DObjectModel::hasSurfaceBasedMatchingData(){

	HTuple hasSurfaceBasedMatchingData = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_surface_based_matching_data"), &hasSurfaceBasedMatchingData);

		std::string value = (std::string)hasSurfaceBasedMatchingData.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasSurfaceBasedMatchingData()", except.ErrorText().Text());
		}
		return res;
}

bool Create3DObjectModel::hasSegmentationData(){

	HTuple hasSegmentationData = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_segmentation_data"), &hasSegmentationData);

		std::string value = (std::string)hasSegmentationData.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasSegmentationData()", except.ErrorText().Text());
		}
		return res;
}

bool Create3DObjectModel::hasPrimitiveData(){

	HTuple hasPrimitiveData = 0;
	bool res = false;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("has_segmentation_data"), &hasPrimitiveData);

		std::string value = (std::string)hasPrimitiveData.ToString();
		if(value.compare("true") == 0){
			res = true;
		}else
			res = false;

		}
		catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::hasPrimitiveData()", except.ErrorText().Text());
		}
		return res;
}

int Create3DObjectModel::getNumberOfPoints(){

	HTuple NumberOfPoints = 0;
	int res = 0;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("num_points"), &NumberOfPoints);

		res = (int)NumberOfPoints[0];

		}catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::getNumberOfPoints()", except.ErrorText().Text());
		}
		return res;
}

int Create3DObjectModel::getNumberOfTriangles(){

	HTuple NumberOfTriangles = 0;
	int res = 0;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("num_triangles"), &NumberOfTriangles);

		res = (int)NumberOfTriangles[0];

		}catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::getNumberOfTriangles()", except.ErrorText().Text());
		}
		return res;
}

int Create3DObjectModel::getNumberOfFaces(){

	HTuple NumberOfFaces = 0;
	int res = 0;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("num_faces"), &NumberOfFaces);

		res = (int)NumberOfFaces[0];

		}catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::getNumberOfFaces()", except.ErrorText().Text());
		}
		return res;
}

int Create3DObjectModel::getNumberOfLines(){

	HTuple NumberOfLines = 0;
	int res = 0;
	try
		{
		GetObjectModel3dParams(mModel,HTuple("num_lines"), &NumberOfLines);

		res = (int)NumberOfLines[0];

		}catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::getNumberOfLines()", except.ErrorText().Text());
		}
		return res;
}

void Create3DObjectModel::getBoundingBox(double& min_x, double& min_y, double& min_z, double& max_x, double& max_y, double& max_z){

	HTuple data = 0;

	try
		{
		GetObjectModel3dParams(mModel,HTuple("bounding_box1"), &data);

		min_x = (double)data[0];
		min_y = (double)data[1];
		min_z = (double)data[2];
		max_x = (double)data[3];
		max_y = (double)data[4];
		max_z = (double)data[5];

		}catch (HException& except)
		{
            pcl::console::print_error("%s in Create3DObjectModel::getBoundingBox()", except.ErrorText().Text());
		}
}


} /* namespace perception */
