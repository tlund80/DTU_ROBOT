/*
 * SurfaceModelDetector.h
 *
 *  Created on: Apr 23, 2013
 *      Author: thomas
 */

#ifndef SURFACEMODELDETECTOR_H_
#define SURFACEMODELDETECTOR_H_


#include <halconcpp/HalconCpp.h>
#include "ModelDetectors.hpp"
#include "ModelDetectionParameters.hpp"

using namespace HalconCpp;

namespace perception {

class SurfaceModelDetector : public ModelDetectors {
public:
	SurfaceModelDetector();
	virtual ~SurfaceModelDetector();

	void loadModel( const std::string& loadPath );
    int detectModel(const HTuple &search_data);

    /**
     * Set query point set
     * @param query query point set
     */
	void setParameters(const SurfaceModelDetectionParameters& param);

    /**
     * Set query point set
     * @param query query point set
     */
    inline void setSurfaceModel(const HTuple& surfaceModel)
	{
		mModelID.Clear();
		mModelID = surfaceModel;
	}

    /**
     * Set query point set
     * @param query query point set
     */
    inline void setVerbose(const bool verbose)
    {
        mVerbose = verbose;
    }

	/*Successors */
    /**
     * Set query point set
     * @param query query point set
     */
    void getKeyPoints(int index, HTuple& keypoints) const;

    /**
     * Set query point set
     * @param query query point set
     */
    void getSampledScene(HTuple& sampledSceneObjectModel3D_out) const;

    /**
     * Set query point set
     * @param query query point set
     */
    void getScoreBeforeRefining(int index, HTuple& score_out) const;

    /**
     * Set query point set
     * @param query query point set
     */
    void getScoreAfterRefining(int index, HTuple& score_out) const;

    /**
     * Set query point set
     * @param query query point set
     */
    void getPose(int index, HTuple& pose_out, HTuple& score_out) const;

    /**
     * Set query point set
     * @param query query point set
     */
    void getCenterOfSurfaceModel(HTuple& center_out) const;

    /**
     * Set query point set
     * @param query query point set
     */
    void getBounding_boxOfSurfaceModel(HTuple& Bounding_box_out) const;

    /**
     * Set query point set
     * @param query query point set
     */
    void getDiameterSurfaceModel(HTuple& Diameter_out) const;

    /**
     * Set query point set
     * @param query query point set
     */
    int getBestMatch(HTuple& pose, HTuple& score) const;

    /**
     * Set query point set
     * @param query query point set
     */
    HTuple getObjectModel(void) const
    {
        return mModelID;
    }

private:
	SurfaceModelDetectionParameters mParam;
	HTuple mPose, mScore, mSurfaceMatchingResultID;
	bool mVerbose;
};

} /* namespace perception */
#endif /* SURFACEMODELDETECTOR_H_ */
