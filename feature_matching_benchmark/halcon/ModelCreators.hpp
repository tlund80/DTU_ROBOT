/********************************************************************************************************************
 *
 * @file		HalconCreators.hpp
 * @author		Kent Hansen (kenh@teknologisk.dk)
 * @date		2013-01-24
 * @version		1.0
 * @brief		Abstract super class for creation of Halcon object models
 *
*********************************************************************************************************************/

#ifndef MODELCREATORS_HPP_
#define MODELCREATORS_HPP_

/** @class ModelCreators
  * Super class for creation of Halcon object models
  */
class ModelCreators
{
	public:
		/// Constructor
		ModelCreators() {};

		/// Destructor
		virtual ~ModelCreators() {};

		/// Create a new object model
		virtual void createModel() = 0;

		/** Save generated object model
		  * @param savePath Save path
		  */
		virtual void saveModel( const std::string& savePath ) = 0;

	protected:
		/// Save model parameters in database
		virtual void saveModelDB() = 0;
};

#endif /* MODELCREATORS_HPP_ */
