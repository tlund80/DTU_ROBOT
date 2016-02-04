/********************************************************************************************************************
 *
 * @file		SurfaceModelDetector.cpp
 * @author		Thomas Sølund (thso@teknologisk.dk)
 * @date		2013-04-23
 * @version		1.0
 * @brief		Class for detection of halcon surface models
 *
*********************************************************************************************************************/

#include "SurfaceModelDetector.h"
#include <pcl/console/print.h>
namespace perception {

SurfaceModelDetector::SurfaceModelDetector() {
	// TODO Auto-generated constructor stub
	mVerbose = true;

}

SurfaceModelDetector::~SurfaceModelDetector() {
	// TODO Auto-generated destructor stub
	try
	{
		ClearSurfaceModel(mModelID);
		ClearSurfaceMatchingResult(mSurfaceMatchingResultID);
	}
	catch (HException& except)
    {
        pcl::console::print_error("%s in SurfaceModelDetector::~SurfaceModelDetector()\n", except.ErrorText().Text());
	}
}

void SurfaceModelDetector::loadModel( const std::string& loadPath )
{
	try
	{
		ReadSurfaceModel(HTuple(loadPath.c_str()),&mModelID);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::loadModel()\n", except.ErrorText().Text());
	}
}

int SurfaceModelDetector::detectModel(const HTuple& search_data)
{
	HTuple GenParamName;
	GenParamName.Append("num_matches");
	GenParamName.Append("max_overlap_dist_rel");
	GenParamName.Append("sparse_pose_refinement");
	GenParamName.Append("score_type");
	GenParamName.Append("pose_ref_use_scene_normals");
	GenParamName.Append("dense_pose_refinement");
	GenParamName.Append("pose_ref_num_steps");
	GenParamName.Append("pose_ref_sub_sampling");
	GenParamName.Append("pose_ref_dist_threshold_rel");
	GenParamName.Append("pose_ref_scoring_dist_rel");

	HTuple GenParamValue;
	GenParamValue.Append(mParam.num_matches);
	GenParamValue.Append(mParam.max_overlap_dist_rel);
	GenParamValue.Append(mParam.sparse_pose_refinement);
	GenParamValue.Append(mParam.score_type);
	GenParamValue.Append(mParam.pose_ref_use_scene_normals);
	GenParamValue.Append(mParam.dense_pose_refinement);
	GenParamValue.Append(mParam.pose_ref_num_steps);
	GenParamValue.Append(mParam.pose_ref_sub_sampling);
	GenParamValue.Append(mParam.pose_ref_dist_threshold_rel);
	GenParamValue.Append(mParam.pose_ref_scoring_dist_rel);


 //   std::cout << "modelID: " << mModelID << std::endl;
	try
	{
		//mModelID = the model of the workpiece
		//search_data = the 3D point cloud of the scene
		FindSurfaceModel(mModelID,search_data,mParam.RelSamplingDistance, mParam.KeyPointFraction, mParam.MinScore,
						 mParam.ReturnResultHandle, GenParamName, GenParamValue,&mPose,&mScore,&mSurfaceMatchingResultID);

	/*	std::cout << "mScore: "<< (double)mScore.Length() << std::endl;
		for(int i = 0; i<= mScore.Length()-1; i++)
		//	std::cout << "mRow[" << i << "]: " << (double)mPose << std::endl;
			std::cout << "mScore[" << i << "]: " << (double)mScore << std::endl;
	*/
		return (int)mScore.Length();
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::detectModel()\n", except.ErrorText().Text());
	}
}

void SurfaceModelDetector::getKeyPoints(int index, HTuple& keypoints) const{

	try
	{
		GetSurfaceMatchingResult(mSurfaceMatchingResultID,HTuple("key_points"),HTuple(index),&keypoints);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getKeyPoints()\n", except.ErrorText().Text());
	}

}
/* getSampledScene()
 * If SurfaceMatchingResultID was created by FindSurfaceModel, a 3D object model handle is returned
 * that contains the sampled scene points that were used in the approximate matching step.
 * This is helpful for tuning the sampling distance for the matching (see parameter RelSamplingDistance of operator
 * FindSurfaceModel). The parameter ResultIndex is ignored. The returned ObjectModel3D handle must be freed
 * with ClearObjectModel3d. If SurfaceMatchingResultID was created by RefineSurfaceModelPose, an empty 3D object model
 * is returned
 */
void SurfaceModelDetector::getSampledScene(HTuple& sampledSceneObjectModel3D_out) const{

	try
	{
		GetSurfaceMatchingResult(mSurfaceMatchingResultID,HTuple("sampled_scene"),HTuple(),&sampledSceneObjectModel3D_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getSampledScene()\n", except.ErrorText().Text());
	}

}

void SurfaceModelDetector::getScoreBeforeRefining(int index, HTuple& score_out) const{

	try
	{
		GetSurfaceMatchingResult(mSurfaceMatchingResultID,HTuple("score_unrefined"),HTuple(index),&score_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getScoreBeforeRefining()\n", except.ErrorText().Text());
	}

}

void SurfaceModelDetector::getScoreAfterRefining(int index, HTuple& score_out) const{

	try
	{
		GetSurfaceMatchingResult(mSurfaceMatchingResultID,HTuple("score_refined"),HTuple(index),&score_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getScoreAfterRefining()\n", except.ErrorText().Text());
	}

}

void SurfaceModelDetector::getPose(int index, HTuple& pose_out, HTuple& score_out) const{

	try
	{
		GetSurfaceMatchingResult(mSurfaceMatchingResultID,HTuple("pose"),HTuple(index),&pose_out);
		GetSurfaceMatchingResult(mSurfaceMatchingResultID,HTuple("score_refined"),HTuple(index),&score_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getPose()\n", except.ErrorText().Text());
	}

}

void SurfaceModelDetector::getCenterOfSurfaceModel(HTuple& center_out) const{

	try
	{
		GetSurfaceModelParam(mModelID,"center",&center_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getCenterOfSurfaceModel()\n", except.ErrorText().Text());
	}

}

void SurfaceModelDetector::getBounding_boxOfSurfaceModel(HTuple& Bounding_box_out) const{

	try
	{
		GetSurfaceModelParam(mModelID,"bounding_box1",&Bounding_box_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getBounding_boxOfSurfaceModel()\n", except.ErrorText().Text());
	}

}

void SurfaceModelDetector::getDiameterSurfaceModel(HTuple& Diameter_out) const{

	try
	{
		GetSurfaceModelParam(mModelID,"diameter",&Diameter_out);
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getDiameterSurfaceModel()\n", except.ErrorText().Text());
	}

}

int SurfaceModelDetector::getBestMatch( HTuple& pose, HTuple& score) const
{
	int maxPos = -1;
	double maxVal = -1;
	try
	{
		if( (int) mScore.TupleLength() != 0 )
		{
			pose = mPose[0];
			score = mScore[0];

			for (HTuple I = 0; I < mScore.Length(); I += 1) {
					if((double)mScore[I] > maxVal) {
						maxPos = (int)I[0];
						maxVal = mScore[I];
					}
				}

			pose[0] = mPose[maxPos * 7];
			pose[1] = mPose[maxPos * 7+1];
			pose[2] = mPose[maxPos * 7+2];
			pose[3] = mPose[maxPos * 7+3];
			pose[4] = mPose[maxPos * 7+4];
			pose[5] = mPose[maxPos * 7+5];
			pose[6] = mPose[maxPos * 7+6];

			if(mVerbose)
			{
                pcl::console::print_info("/***** 3D Surface Based Model Detector ******/\n");
                pcl::console::print_info("Best match found at:\n"
											"\tx: %f,"
											"\ty: %f,"
											"\tz: %f,"
											"\tR: %f,"
											"\tP: %f,"
											"\tY: %f,"
											"\tType: %f,"
                                            "\tScore: %f\n",
						(double) pose[0], (double) pose[1], (double) pose[2],
						(double) pose[3], (double) pose[4], (double) pose[5],
						(double) pose[6], (double) maxVal);

			}

			return 1;
		}
	}
	catch (HException& except)
	{
        pcl::console::print_error("%s in SurfaceModelDetector::getBestMatch()\n", except.ErrorText().Text());
	}
	return 0;
}



void SurfaceModelDetector::setParameters(const SurfaceModelDetectionParameters& param)
{
	mParam = param;
}

} /* namespace perception */
