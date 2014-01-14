/*
Copyright (C) 2012 Steven Hickson

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

*/
#pragma once
#include <pcl/pcl_config.h>

#ifndef __PCL_IO_MICROSOFT_GRABBER__
#define __PCL_IO_MICROSOFT_GRABBER__

#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <string>
#include <deque>
#include <pcl/common/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/exceptions.h>
#include <iostream>

#include <assert.h>
#include <windows.h>
#include <vector>
#include <algorithm>
#include <objbase.h>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSkeleton.h>
#include <NuiSensor.h>

#include <opencv2/opencv.hpp>

namespace pcl
{
	struct PointXYZ;
	struct PointXYZRGB;
	struct PointXYZRGBA;
	struct PointXYZI;
	template <typename T> class PointCloud;
	class MatDepth : public cv::Mat { }; //I have to use this to get around the assuming code in registerCallback in grabber.h
	class KinectData {
	public:
		KinectData() { };
		KinectData(cv::Mat &image_, MatDepth &depth_, PointCloud<PointXYZRGBA> &cloud_) : image(image_), depth(depth_), cloud(cloud_) { };
		
		pcl::PointCloud<pcl::PointXYZRGBA> cloud;
		cv::Mat image;
		MatDepth depth;
	};

	/** \brief Grabber for OpenNI devices (i.e., Primesense PSDK, Microsoft Kinect, Asus XTion Pro/Live)
	* \author Nico Blodow <blodow@cs.tum.edu>, Suat Gedikli <gedikli@willowgarage.com>
	* \ingroup io
	*/
	class PCL_EXPORTS MicrosoftGrabber : public Grabber
	{
	public:
		typedef boost::shared_ptr<MicrosoftGrabber> Ptr;
		typedef boost::shared_ptr<const MicrosoftGrabber> ConstPtr;

		typedef enum
		{
			Microsoft_Default_Mode = 0, // VGA@30Hz
			Microsoft_SXGA_15Hz = 1    // Need to fill the rest of this up
		} Mode;

		//define callback signature typedefs
		typedef void (sig_cb_microsoft_image) (const boost::shared_ptr<const cv::Mat> &);
		typedef void (sig_cb_microsoft_depth_image) (const boost::shared_ptr<const MatDepth> &);
		typedef void (sig_cb_microsoft_point_cloud_rgba) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);
		typedef void (sig_cb_microsoft_all_data) (const boost::shared_ptr<const KinectData> &);
		/*typedef void (sig_cb_microsoft_ir_image) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
		typedef void (sig_cb_microsoft_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
		typedef void (sig_cb_microsoft_point_cloud_rgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);
		typedef void (sig_cb_microsoft_point_cloud_i) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);*/

		MicrosoftGrabber (const int instance = 0);
		//const Mode& depth_mode = OpenNI_Default_Mode,
		//const Mode& image_mode = OpenNI_Default_Mode);

		/** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
		virtual ~MicrosoftGrabber () throw ();

		/** \brief Start the data acquisition. */
		virtual void
			start ();

		/** \brief Stop the data acquisition. */
		virtual void
			stop ();

		/** \brief Check if the data acquisition is still running. */
		virtual bool
			isRunning () const;

		virtual std::string
			getName () const;

		/** \brief Obtain the number of frames per second (FPS). */
		virtual float 
			getFramesPerSecond () const;

		/** \brief Get a boost shared pointer to the \ref OpenNIDevice object. */
		/*inline boost::shared_ptr<MicrosoftGrabber>
		getDevice () const;*/


		//Kinect Camera Settings
		INuiColorCameraSettings *CameraSettings;
		INuiCoordinateMapper *mapper;
		bool CameraSettingsSupported;

		void GetPointCloudFromData(const boost::shared_ptr<cv::Mat> &img, const boost::shared_ptr<MatDepth> &depth, boost::shared_ptr<PointCloud<PointXYZRGBA>> &cloud, bool useZeros, bool alignToColor, bool preregistered) const;

		//These should not be called except within the thread by the KinectCapture class process manager
		void ProcessThreadInternal();

	protected:
		boost::signals2::signal<sig_cb_microsoft_image>* image_signal_;
		boost::signals2::signal<sig_cb_microsoft_depth_image>* depth_image_signal_;
		boost::signals2::signal<sig_cb_microsoft_point_cloud_rgba>* point_cloud_rgba_signal_;
		boost::signals2::signal<sig_cb_microsoft_all_data>* all_data_signal_;
		/*boost::signals2::signal<sig_cb_microsoft_ir_image>* ir_image_signal_;
		boost::signals2::signal<sig_cb_microsoft_point_cloud>* point_cloud_signal_;
		boost::signals2::signal<sig_cb_microsoft_point_cloud_i>* point_cloud_i_signal_;
		boost::signals2::signal<sig_cb_microsoft_point_cloud_rgb>* point_cloud_rgb_signal_;
		*/
		Synchronizer<boost::shared_ptr<cv::Mat>, boost::shared_ptr<MatDepth> > rgb_sync_;

		HANDLE hColorStream, hDepthStream, hInfraredStream;
		HANDLE hDepthFrameEvent, hColorFrameEvent, hInfraredFrameEvent, hSkeletonEvent, hStopEvent, hKinectThread;
		bool m_depthStarted, m_videoStarted, m_audioStarted, m_infraredStarted, m_person, m_preregistered;
		BOOL m_nearMode, m_seatedMode;
		INuiSensor *kinectInstance;
		int depthWidth, depthHeight, imgWidth, imgHeight;
		NUI_IMAGE_RESOLUTION colorRes, depthRes;
		bool m_depthUpdated, m_colorUpdated, m_infraredUpdated, m_skeletonUpdated;
		NUI_SKELETON_FRAME m_skeleton;
		//boost::mutex m_depthMutex, m_colorMutex, m_infraredMutex;
		LONGLONG m_rgbTime, m_depthTime, m_infraredTime;

		void Release();
		void GotColor();
		void GotDepth();
		//void GotInfrared();
		//void GotSkeleton();
		void StartVideoCapture();
		void StartDepthCapture();
		//void StartAudioCapture();
		void StartInfraredCapture();
		bool GetCameraSettings();
		
		void imageDepthImageCallback(const boost::shared_ptr<cv::Mat> &image, const boost::shared_ptr<MatDepth> &depth_image);
		boost::shared_ptr<cv::Mat> convertToRGBMat(const NUI_IMAGE_FRAME &color) const;
		boost::shared_ptr<MatDepth> convertTo32SMat(INuiFrameTexture *texture) const;
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > convertToXYZRGBAPointCloud (const boost::shared_ptr<cv::Mat> &image,
                               const boost::shared_ptr<MatDepth> &depth_image) const;
		/** \brief Convert a Depth + RGB image pair to a pcl::PointCloud<PointT>
		* \param[in] image the RGB image to convert
		* \param[in] depth_image the depth image to convert
		*/
		/*template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
			convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
			const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;*/
	};

}

#endif //__PCL_IO_MICROSOFT_GRABBER__