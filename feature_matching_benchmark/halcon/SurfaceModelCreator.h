/*
 * SurfaceModelCreator.h
 *
 *  Created on: Apr 22, 2013
 *      Author: thomas
 */

#ifndef SURFACEMODELCREATOR_H_
#define SURFACEMODELCREATOR_H_

#include <halconcpp/HalconCpp.h>

#include "ModelCreationParameters.hpp"
#include "ModelCreators.hpp"

using namespace HalconCpp;

namespace perception {

class SurfaceModelCreator  : public ModelCreators{
public:
	SurfaceModelCreator();
	virtual ~SurfaceModelCreator();

	virtual void createModel();
	virtual void saveModel(const std::string& savePath);

    /**
     * Set query point set
     * @param query query point set
     */
	void setParameters(const SurfaceModelCreationParameters& param);

    /**
     * Set query point set
     * @param query query point set
     */
    void showModels();

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

    void getSurfaceModel(HTuple& surfacemodel)const
    {
        surfacemodel = mModelID;
        //return mModelID;
    }
    
     HTuple getSurfaceModel(void)const
    {
       
        return mModelID;
    }

protected:
	/// Save model parameters in database
	virtual void saveModelDB() { };

private:
	HTuple mModelID;				/**< Model ID */
	SurfaceModelCreationParameters mParam;	/**< Model paramters */
};

} /* namespace perception */
#endif /* SURFACEMODELCREATOR_H_ */
