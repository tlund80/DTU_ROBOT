/********************************************************************************************************************
 *
 * @file		ObjectModelAttribute.hpp
 * @author		Thomas Sølund(thso@teknologisk.dk)
 * @date		2013-06-04
 * @version		1.0
 * @brief		Attributes of Halcon object models.
 *
******************************************************************************************************************/
#ifndef OBJECTMODELATTRIBUTE_HPP_
#define OBJECTMODELATTRIBUTE_HPP_

#include <halconcpp/HalconCpp.h>

using namespace HalconCpp;

struct ObjectModelAttributes
{
	ObjectModelAttributes() :
				point_coord_x(HTuple("point_coord_x")),
				point_coord_y(HTuple("point_coord_y")),
				point_coord_z(HTuple("point_coord_z")),
				point_normal_x(HTuple("point_normal_x")),
				point_normal_y(HTuple("point_normal_y")),
				point_normal_z(HTuple("point_normal_z")),
				mapping_row(HTuple("mapping_row")),
				mapping_col(HTuple("mapping_col")),
				triangles(HTuple("triangles")),
				faces(HTuple("point_coord_x")),
				lines(HTuple("point_coord_y")),
				diameter_axis_aligned_bounding_box(HTuple("diameter_axis_aligned_bounding_box")),
				center(HTuple("center")),
				primitive_type(HTuple("primitive_type")),
				primitive_rms(HTuple("primitive_rms")),
				primitive_parameter(HTuple("primitive_parameter")),
				primitive_parameter_pose(HTuple("primitive_parameter_pose")),
				primitive_pose(HTuple("primitive_pose")),
				primitive_parameter_extension(HTuple("primitive_parameter_extension")),
				reference_point(HTuple("reference_point")),
				bounding_box1(HTuple("bounding_box1")),
				num_points(HTuple("num_points")),
				num_triangles(HTuple("num_triangles")),
				num_faces(HTuple("num_faces")),
				num_lines(HTuple("num_lines")),
				num_primitive_parameter_extension(HTuple("num_primitive_parameter_extension")),
				has_points(HTuple("has_points")),
				has_point_normals(HTuple("has_point_normals")),
				has_triangles(HTuple("has_triangles")),
				has_faces(HTuple("has_faces")),
				has_lines(HTuple("has_lines")),
				has_xyz_mapping(HTuple("has_xyz_mapping")),
				has_shape_based_matching_3d_data(HTuple("has_shape_based_matching_3d_data")),
				has_surface_based_matching_data(HTuple("has_surface_based_matching_data")),
				has_segmentation_data(HTuple("has_segmentation_data")),
				has_primitive_data(HTuple("has_primitive_data")),
				has_primitive_rms(HTuple("has_primitive_rms")),
				extended_attribute_names(HTuple("extended_attribute_names")),
				has_extended_attribute(HTuple("has_extended_attribute")),
				num_extended_attribute(HTuple("num_extended_attribute")),
				has_distance_computation_data(HTuple("has_distance_computation_data")){}
	HTuple point_coord_x;
	HTuple point_coord_y;
	HTuple point_coord_z;
	HTuple point_normal_x;
	HTuple point_normal_y;
	HTuple point_normal_z;
	HTuple mapping_row;
	HTuple mapping_col;
	HTuple triangles;
	HTuple faces;
	HTuple lines;
	HTuple diameter_axis_aligned_bounding_box;
	HTuple center;
	HTuple primitive_type;
	HTuple primitive_rms;
	HTuple primitive_parameter;
	HTuple primitive_parameter_pose;
	HTuple primitive_pose;
	HTuple primitive_parameter_extension;
	HTuple reference_point;
	HTuple bounding_box1;
	HTuple num_points;
	HTuple num_triangles;
	HTuple num_faces;
	HTuple num_lines;
	HTuple num_primitive_parameter_extension;
	HTuple has_points;
	HTuple has_point_normals;
	HTuple has_triangles;
	HTuple has_faces;
	HTuple has_lines;
	HTuple has_xyz_mapping;
	HTuple has_shape_based_matching_3d_data;
	HTuple has_surface_based_matching_data;
	HTuple has_segmentation_data;
	HTuple has_primitive_data;
	HTuple has_primitive_rms;
	HTuple extended_attribute_names;
	HTuple has_extended_attribute;
	HTuple num_extended_attribute;
	HTuple has_distance_computation_data;
};

struct ObjectModelSegmentationParams
{
	ObjectModelSegmentationParams() :
						max_orientation_diff(HTuple(0.15)),
						max_curvature_diff(HTuple(0.05)),
						min_area(HTuple(100)),
						fitting(HTuple("false")),
						output_xyz_mapping(HTuple("true")){};

	HTuple max_orientation_diff;
	HTuple max_curvature_diff;
	HTuple min_area;
	HTuple fitting;
	HTuple output_xyz_mapping;
};

struct Primitive3DTypes
{
	Primitive3DTypes() :
			cylinder(HTuple("cylinder")),
			sphere(HTuple("sphere")),
			plane(HTuple("plane")),
			all(HTuple("all"))
			{};
	HTuple cylinder;
	HTuple sphere;
	HTuple plane;
	HTuple all;

};

struct Primitive3DFittingAlgorithm
{
	Primitive3DFittingAlgorithm() :
			least_squares(HTuple("least_squares")),
			least_squares_huber(HTuple("least_squares_huber")),
			least_squares_tukey(HTuple("least_squares_tukey"))
			{};
	HTuple least_squares;
	HTuple least_squares_huber;
	HTuple least_squares_tukey;

};

struct Primitive3DFittingParams
{
	Primitive3DFittingParams():
		min_radius(HTuple(0.02)),
		max_radius(HTuple(0.04)),
		output_point_coord(HTuple("true")),
		output_xyz_mapping(HTuple("false"))
		{};
	HTuple min_radius;
	HTuple max_radius;
	HTuple output_point_coord;
	HTuple output_xyz_mapping;
};

struct ObjectModel3DTriangulationMethod
{
	ObjectModel3DTriangulationMethod():
			greedy(HTuple("greedy")),
			implicit(HTuple("implicit")),
			polygon_triangulation(HTuple("polygon_triangulation"))
			{};
	HTuple greedy;
	HTuple implicit;
	HTuple polygon_triangulation;

};

struct ObjectModel3DTriangulationParams
{
	ObjectModel3DTriangulationParams():
			greedy_kNN(HTuple(40)),
			greedy_radius_type(HTuple("auto")),
			greedy_radius_value(HTuple(1.0)),
			greedy_neigh_orient_tol(HTuple(30)),
			greedy_neigh_orient_consistent(HTuple("false")),
			greedy_neigh_latitude_tol(HTuple(30)),
			greedy_neigh_vertical_tol(HTuple(0.1)),
			greedy_hole_filling(HTuple(40)),
			greedy_fix_flips(HTuple("true")),
			greedy_prefetch_neighbors(HTuple("true")),
			greedy_mesh_erosion(HTuple(0)),
			greedy_mesh_dilation(HTuple(0)),
			greedy_remove_small_surfaces(HTuple("false")),
			greedy_timeout(HTuple("false")),
			greedy_suppress_timeout_error(HTuple("false")),
			implicit_octree_depth(HTuple(6)),
			implicit_solver_depth(HTuple(6)),
			implicit_min_num_samples(HTuple(1))


			{};
	HTuple greedy_kNN;
	HTuple greedy_radius_type;
	HTuple greedy_radius_value;
	HTuple greedy_neigh_orient_tol;
	HTuple greedy_neigh_orient_consistent;
	HTuple greedy_neigh_latitude_tol;
	HTuple greedy_neigh_vertical_tol;
	HTuple greedy_hole_filling;
	HTuple greedy_fix_flips;
	HTuple greedy_prefetch_neighbors;
	HTuple greedy_mesh_erosion;
	HTuple greedy_mesh_dilation;
	HTuple greedy_remove_small_surfaces;
	HTuple greedy_timeout;
	HTuple greedy_suppress_timeout_error;

	HTuple implicit_octree_depth;
	HTuple implicit_solver_depth;
	HTuple implicit_min_num_samples;


};

struct ObjectModelFileFormat
{
	ObjectModelFileFormat():
		om3("om3"),
		dxf("dxf"),
		off("off"),
		ply("ply"),
		obj("obj"),
		stl("stl"),
		stl_binary("stl_binary"),
		stl_ascii("stl_ascii")
		{};

	std::string om3;
	std::string dxf;
	std::string off;
	std::string ply;
	std::string obj;
	std::string stl;
	std::string stl_binary;
	std::string stl_ascii;

};



#endif /* OBJECTMODELATTRIBUTE_HPP_ */
