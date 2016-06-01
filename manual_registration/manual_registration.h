/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2012-, Open Perception, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ui_manual_registration.h>

// Qt
#include <QMainWindow>
#include <QMutex>
#include <QTimer>

#include <QVTKWidget.h>

// Boost
#include <boost/thread/thread.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

// Useful macros
#define FPS_CALC(_WHAT_) \
        do {\
            static unsigned count = 0;\
            static double last = pcl::getTime ();\
            double now = pcl::getTime (); \
            ++count; \
            if (now - last >= 1.0) { \
                std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
                count = 0; \
                last = now; \
            } \
        } while(false)

namespace Ui {
    class MainWindow;
}

class ManualRegistration : public QMainWindow {
    Q_OBJECT
    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> Cloud;
        typedef Cloud::Ptr CloudPtr;
        typedef Cloud::ConstPtr CloudConstPtr;
		typedef pcl::PolygonMesh Mesh;
		typedef pcl::PolygonMesh::Ptr MeshPtr;


        ManualRegistration ();

        ~ManualRegistration () {}

        inline void setSrcCloud (CloudPtr cloud_src) {
            cloud_src_ = cloud_src;
            cloud_src_present_ = true;
        }

        inline void setResolution (double res) {
            res_ = res;
        }

        inline void setDstCloud (CloudPtr cloud_dst) {
            cloud_dst_ = cloud_dst;
            cloud_dst_present_ = true;
        }

        void SourcePointPickCallback (const pcl::visualization::PointPickingEvent& event, void*);
        
        void DstPointPickCallback (const pcl::visualization::PointPickingEvent& event, void*);
	
	double ComputeCloudResolution(const ManualRegistration::CloudConstPtr &cloud);

    protected:
       
        float variance(std::vector<float> &v);
	float stdDev(std::vector<float> &v);
	float median(std::vector<float> &v);
	float mean(std::vector<float> &v);
	
	float computeRMSError(const ManualRegistration::CloudConstPtr &cloud_source, 
			     const ManualRegistration::CloudConstPtr &cloud_target,
		             std::string correspondence_type);

	double estimateClutter(const ManualRegistration::CloudConstPtr &model, 
			       const ManualRegistration::CloudConstPtr &scene);
	
	double estimateOcclusion(const ManualRegistration::CloudConstPtr &model,
				  const ManualRegistration::CloudConstPtr &scene);
        
	void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void);
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_src_;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_dst_;

        CloudPtr cloud_src_;
        CloudPtr cloud_dst_;
	CloudPtr cloud_src_sampled_;
        CloudPtr cloud_dst_sampled_;
	CloudPtr cloud_aligned_;
	MeshPtr mesh_src_;
	MeshPtr mesh_dst_;
        double res_;
	double res_scene_;

        QMutex mtx_;
        QMutex vis_mtx_;
        Ui::MainWindow *ui_;
        QTimer *vis_timer_;

        bool                              cloud_src_present_;
        bool                              cloud_src_modified_;
        bool                              cloud_dst_present_;
        bool                              cloud_dst_modified_;
		bool							  color_toggle_;

        bool                              src_point_selected_;
        bool                              dst_point_selected_;
		bool			 				  trfm_computed_;

        PointT                            src_point_;
        PointT                            dst_point_;

        CloudPtr                          src_pc_;
        CloudPtr                          dst_pc_;

        Eigen::Matrix4f                   transform_;

	QString							  model_name_;
	QString							  scene_name_;
	QString							  base_path_;
	
	boost::shared_ptr<QVTKWidget>			_src_widget;
	boost::shared_ptr<QVTKWidget>			_dst_widget;

    public slots:
        void calculatePressed();
        void clearPressed();
   	void applyTrfmPressed(); 
	void on_btnModel_clicked();
	void on_btnScene_clicked();

    private slots:
        void timeoutSlot();


    private:
		
	Eigen::Matrix4f readMatrix(QString filename);


};


