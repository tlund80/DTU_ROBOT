#ifndef HALCONESTIMATION_H
#define HALCONESTIMATION_H

#include "detect_base.h"

#include <pcl/point_cloud.h>
#include <pcl/console/print.h>

#include <halconcpp/HalconCpp.h>

#include "../halcon/ModelCreationParameters.hpp"
#include "../halcon/SurfaceModelCreator.h"
#include "../halcon/SurfaceModelDetector.h"
#include "../halcon/ModelCreators.hpp"
#include "../halcon/ModelDetectors.hpp"
#include "../halcon/Create3DObjectModel.h"

using namespace HalconCpp;

namespace covis {
    namespace detect {

    /**
     * @ingroup detect
     * @class HalconEstimation
     * @brief Estimate the pose of an object(PointSource) in a scene(PointTarget)
     *
     *
     *
     * @tparam PointT point type for source and target
     * @author Thomas Sølund
     */
    template <typename PointSource, typename PointTarget>
    class HalconEstimation : public DetectBase {

        public:
            typedef typename pcl::PointCloud<PointSource> PointCloudSource;
            typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
            typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

            typedef typename pcl::PointCloud<PointTarget> PointCloudTarget;
            typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
            typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;


            HalconEstimation(){

                //Check if halcon dongle is avalible
                HTuple info;
                try{
                    GetSystem("hostids", &info);
                }catch (HOperatorException &e)
                {
                   pcl::console::print_error("%s\n", e.ErrorText().Text());

                }
            };

            virtual ~HalconEstimation(){}


            /** \brief Provide a pointer to the input source
             * (e.g., the point cloud that we want to align to the target)
             *
             * \param[in] cloud the input point cloud source
             */
           inline void
           setInputSource (const PointCloudSourceConstPtr &cloud)
           {

            object = cloud;

           }

            /** \brief Provide a pointer to the input target
             * (e.g., the point cloud that we want to align to the target)
             *
             * \param[in] cloud the input point cloud target
             */
             inline void
             setInputTarget (const PointCloudTargetConstPtr &cloud)
             {

               scene = cloud;
             }

            /** \brief Sets the maximum number of matches that are returned (default: 10)
             *
             * \param[in] Maximum number of objects to detect
             */
             inline void
             setNumberOfMatches(const int& num_match)
             {
               _detectParams.num_matches = HTuple(num_match);
             }

             /** \brief Set fraction of sampled scene points used as key points. (default:0.5)"
              *
              * \param[in] Fraction of keypoints
              */
             inline void
             setKeypointFraction (const float& keypoint_fraction)
             {
               _detectParams.KeyPointFraction = keypoint_fraction;
             }

             /** \brief Set Minimum score of the returned poses.(default: 0.1)"
              *
              * \param[in] Minimum score for a pose hypothesis to be accepted
              */
             inline void
             setMinScore (const float& minscore)
             {
               _detectParams.MinScore = minscore;
             }

             /** \brief Set wheather the orientation of the surface normals of the model should be inverted in the construction of the surface model (default: false)"
              *
              * \param[in] Minimum score for a pose hypothesis to be accepted
              */
             inline void
             setInvertNormals (bool invertnormals)
             {
               _smcParams.model_invert_normals = invertnormals;
             }

            /** \brief Set the sampling distance for the pose refinement relative to the object's diameter. (default:0.01)
            *
            * \param[in] Sample Distance
            */
             inline void
             setSampleDistance (const float& sample_dist)
             {
                 _smcParams.RelSamplingDistance = sample_dist;
                 _smcParams.feat_step_size_rel = sample_dist;
             }


             inline void generateSurfaceModel()
             {



             }

             inline bool align(Eigen::Matrix4f &final_transformation)
             {

                 //Create the object model from a point cloud
                 perception::Create3DObjectModel cadModelCreator;
                 HTuple x,y,z;

                 for (size_t i = 0; i < object->points.size (); ++i){
                     x.Append(HTuple(object->points[i].x));
                     y.Append(HTuple(object->points[i].y));
                     z.Append(HTuple(object->points[i].z));
                 }

                 cadModelCreator.createModelFromPoints(x,y,z);
                 cadModelCreator.computeSurfaceNormals();
                HTuple _objectModel3D = cadModelCreator.get3DObjectModel();

                HTuple x1,y1,z1;
                for (size_t i = 0; i < scene->points.size (); ++i){
                    x1.Append(HTuple(scene->points[i].x));
                    y1.Append(HTuple(scene->points[i].y));
                    z1.Append(HTuple(scene->points[i].z));
                }

                //Creating Model of the scene!!
                perception::Create3DObjectModel SearchModelCreator;
                SearchModelCreator.createModelFromPoints(x1,y1,z1);
                SearchModelCreator.computeSurfaceNormals();
                //pcl::console::print_info("Number of points in scene: %d\n", SearchModelCreator.getNumberOfPoints());
                HTuple _scene_data = SearchModelCreator.get3DObjectModel();


               //  if(_surfaceModel.Length() < 1)
                 //generateSurfaceModel();

                 //Create the surface model using the objectModel
                 _smcParams.ObjectModel3D = _objectModel3D;
                 perception::SurfaceModelCreator smc;
                 smc.setParameters(_smcParams);
                 smc.createModel();
                 HTuple _surfaceModel;
                 smc.getSurfaceModel(_surfaceModel);
                 //_surfaceModel = smc.getSurfaceModel();

                 HTuple pose, score;
                 perception::SurfaceModelDetector detector;
                 detector.setParameters(_detectParams);
                 detector.setSurfaceModel(_surfaceModel);
                 detector.setVerbose(false);

                 pcl::console::print_highlight("Detecting 3D model!\n");
                 int instances = detector.detectModel(_scene_data);
                 if(instances > 0){
                     pcl::console::print_highlight("Found ");
                     pcl::console::print_value("%d", instances);
                     pcl::console::print_info(" instances of the model in the scene!\n");

                 if(detector.getBestMatch(pose, score) == 1){
                  HTuple HomMat;
                  PoseToHomMat3d(pose,&HomMat);

                  Eigen::Matrix4f t;
                  t(0,0) = (double)HomMat[0]; t(0,1) = (double)HomMat[1]; t(0,2) = (double)HomMat[2]; t(0,3) = (double)HomMat[3];
                  t(1,0) = (double)HomMat[4]; t(1,1) = (double)HomMat[5]; t(1,2) = (double)HomMat[6]; t(1,3) = (double)HomMat[7];
                  t(2,0) = (double)HomMat[8]; t(2,1) = (double)HomMat[9]; t(2,2) = (double)HomMat[10]; t(2,3) = (double)HomMat[11];
                  t(3,0) = 0; t(3,1) = 0; t(3,2) = 0; t(3,3) = 1;

                  final_transformation = t;

                  }
                  return true;
                 }else{
                  return false;
                 }


             }

        protected:


         //   HTuple _objectModel3D;
         //   HTuple _scene_data;
         //   HTuple _surfaceModel;

            PointCloudSourceConstPtr object;
            PointCloudTargetConstPtr scene;
            SurfaceModelCreationParameters _smcParams;
            SurfaceModelDetectionParameters _detectParams;


    };
}
}

#endif // HALCONESTIMATION_H
