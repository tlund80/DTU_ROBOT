/********************************************************************************************************************
 *
 * @file		HalconDetectors.hpp
 * @author		Kent Hansen (kenh@teknologisk.dk)
 * @date		2013-01-29
 * @version		1.0
 * @brief		Abstract super class for detection of Halcon object models
 *
*********************************************************************************************************************/

#ifndef MODELDETECTORS_HPP_
#define MODELDETECTORS_HPP_

using namespace HalconCpp;

class ModelDetectors
{
	public:
		/// Constructor
		ModelDetectors() {};

		// Destructor
		virtual ~ModelDetectors() {};

		/** Load detection model
		  * @param loadPath Model path
		  */
		virtual void loadModel( const std::string& loadPath ) = 0;

		/** Detect model in image
		  *	@param img Input image
		  */
        virtual int detectModel(const HTuple& search_data) = 0;

	protected:
		HTuple mModelID;
		HTuple mModelName;
};


#endif /* MODELDETECTORS_HPP_ */
