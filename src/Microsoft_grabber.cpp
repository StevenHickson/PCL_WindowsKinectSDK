
/*
Copyright (C) 2014 Steven Hickson

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
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "Microsoft_grabber.h"

using namespace std;
using namespace cv;

//DEFINES constants for kinect cameras
#define KINECT_CX_C 3.5094272028759258e+02
#define KINECT_CY_C 2.4251931828128443e+02
#define KINECT_FX_C 5.2921508098293293e+02
#define KINECT_FY_C 5.2556393630057437e+02


//I preinvert all the depth focal points so they can be multiplied instead of divided. This is because I'm smart
#define KINECT_CX_D 3.2330780975300314e+02
#define KINECT_CY_D 2.4273913761751615e+02
#define KINECT_FX_D 1.6828944189289601e-03
#define KINECT_FY_D 1.6919313269589566e-03

DWORD ProcessThread(LPVOID pParam) {
	pcl::MicrosoftGrabber *p = (pcl::MicrosoftGrabber*) pParam;
	p->ProcessThreadInternal();
	return 0;
}

template <typename T> inline T Clamp(T a, T minn, T maxx)
{ return (a < minn) ? minn : ( (a > maxx) ? maxx : a ); }

namespace pcl {
	MicrosoftGrabber::MicrosoftGrabber(const int instance) {
		HRESULT hr;
		int num = 0;
		hColorStream = NULL;
		hDepthStream = NULL;
		hInfraredStream = NULL;
		hDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
		hColorFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
		hInfraredFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
		hSkeletonEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
		hStopEvent = NULL;
		hKinectThread = NULL;
		m_nearMode = m_person = false;
		hr = NuiGetSensorCount(&num);
		if (FAILED(hr))
		{
			throw exception("Failed to find Kinect sensor");
		}
		int options = 0;
		options |= NUI_INITIALIZE_FLAG_USES_COLOR;
		options |= NUI_INITIALIZE_FLAG_USES_DEPTH;
		imgWidth = 640;
		imgHeight = 480;
		depthWidth = 640;
		depthHeight = 480;
		colorRes = depthRes = NUI_IMAGE_RESOLUTION_640x480;

		if (num > instance) {
			hr = NuiCreateSensorByIndex(instance, &kinectInstance);
			if ( FAILED( hr ) ) {
				throw exception("Failed to connect to Kinect sensor");
			}
			hr = kinectInstance->NuiStatus();
			if ( FAILED( hr ) ) {
				throw exception("Kinect Sensor has improper status");
			}
			m_depthStarted = m_videoStarted = m_audioStarted = m_infraredStarted = false;
			hr = kinectInstance->NuiInitialize(options);
			if ( FAILED( hr ) )
				throw exception("Failed to initialize Kinect sensor");
			hr = kinectInstance->NuiGetCoordinateMapper(&mapper);
			if ( FAILED( hr ) )
				throw exception("Failed to initialize mapper");
		} else
			throw exception("Failed to Find a Kinect sensor");

		// create callback signals
		image_signal_             = createSignal<sig_cb_microsoft_image> ();
		depth_image_signal_    = createSignal<sig_cb_microsoft_depth_image> ();
		point_cloud_rgba_signal_  = createSignal<sig_cb_microsoft_point_cloud_rgba> ();
		all_data_signal_  = createSignal<sig_cb_microsoft_all_data> ();
		/*ir_image_signal_       = createSignal<sig_cb_microsoft_ir_image> ();
		point_cloud_signal_    = createSignal<sig_cb_microsoft_point_cloud> ();
		point_cloud_i_signal_  = createSignal<sig_cb_microsoft_point_cloud_i> ();
		point_cloud_rgb_signal_   = createSignal<sig_cb_microsoft_point_cloud_rgb> ();
		*/ 
		rgb_sync_.addCallback (boost::bind (&MicrosoftGrabber::imageDepthImageCallback, this, _1, _2));
	}

	void MicrosoftGrabber::start() {
		block_signals();
		StartDepthCapture();
		StartVideoCapture();
		GetCameraSettings();
		hStopEvent = CreateEvent( NULL, FALSE, FALSE, NULL );
		hKinectThread = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)&ProcessThread, this, 0, NULL );
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		unblock_signals();
	}

	void MicrosoftGrabber::stop() {
		//stop the ProcessThread
		if(hStopEvent != NULL) {
			//signal the process to stop
			SetEvent(hStopEvent);
			if(hKinectThread != NULL) {
				WaitForSingleObject(hKinectThread,INFINITE);
				CloseHandle(hKinectThread);
				hKinectThread = NULL;
			}
			CloseHandle(hStopEvent);
			hStopEvent = NULL;
		}
	}

	bool MicrosoftGrabber::isRunning () const {
		return (!(hKinectThread == NULL));
	}

	MicrosoftGrabber::~MicrosoftGrabber() {
		Release();
	}

	bool MicrosoftGrabber::GetCameraSettings() {
		CameraSettings = NULL;
		if(S_OK == kinectInstance->NuiGetColorCameraSettings(&CameraSettings))
			CameraSettingsSupported = true;
		else
			CameraSettingsSupported = false;
		return CameraSettingsSupported;
	}

	void MicrosoftGrabber::StartVideoCapture() {
		if(!m_videoStarted) {
			HRESULT hr = kinectInstance->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,colorRes,0,2,hColorFrameEvent,&hColorStream);
			if (FAILED( hr )) {
				throw exception("Failed to Open Kinect Image Stream");
			} else
				m_videoStarted = true;
		}
	}

	void MicrosoftGrabber::StartDepthCapture() {
		if(!m_depthStarted) {
			HRESULT hr;
			if (!m_person)
				hr = kinectInstance->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH,depthRes,0,2,hDepthFrameEvent,&hDepthStream);
			else
				hr = kinectInstance->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,depthRes,0,2,hDepthFrameEvent,&hDepthStream);
			if ( FAILED( hr ) ) {
				throw exception("Failed to Open Kinect Depth Stream");
			} else
				m_depthStarted = true;
		}
	}

	void MicrosoftGrabber::StartInfraredCapture() {
		if(!m_infraredStarted) {
			HRESULT hr = kinectInstance->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR_INFRARED,colorRes,0,2,hInfraredFrameEvent,&hInfraredStream);
			if (FAILED( hr )) {
				throw exception("Failed to Open Kinect Image Stream");
			} else
				m_infraredStarted = true;
		}
	}

	void MicrosoftGrabber::ProcessThreadInternal() {
		const int numEvents = 5;
		HANDLE hEvents[numEvents] = { hStopEvent, hDepthFrameEvent, hColorFrameEvent, hInfraredFrameEvent, hSkeletonEvent };
		int nEventIdx;
		bool quit = false;
		while(!quit) {
			// Wait for any of the events to be signalled
			nEventIdx = WaitForMultipleObjects( numEvents, hEvents, FALSE, 100 );

			// Process signal events
			switch ( nEventIdx )
			{
			case WAIT_TIMEOUT:
				continue;
				// If the stop event, stop looping and exit
			case WAIT_OBJECT_0:
				quit = true;
				continue;
			case WAIT_OBJECT_0 + 1:
				GotDepth();
				break;
			case WAIT_OBJECT_0 + 2:
				GotColor();
				break;
			case WAIT_OBJECT_0 + 3:
				//GotInfrared();
				break;
				/*case WAIT_OBJECT_0 + 4:
				GotSkeleton();
				break;*/
			}
		}
	}

	void MicrosoftGrabber::Release() {
		try {
			//clean up stuff here
			stop();
			if(kinectInstance) {
				//Shutdown NUI and Close handles
				kinectInstance->NuiShutdown();
				if (hColorFrameEvent != NULL && hColorFrameEvent != INVALID_HANDLE_VALUE) {
					CloseHandle(hColorFrameEvent);
					hColorStream = NULL;
				}
				if (hDepthFrameEvent != NULL && hDepthFrameEvent != INVALID_HANDLE_VALUE) {
					CloseHandle(hDepthFrameEvent);
					hDepthStream = NULL;
				}
				if (hInfraredFrameEvent != NULL && hInfraredFrameEvent != INVALID_HANDLE_VALUE) {
					CloseHandle(hInfraredFrameEvent);
					hInfraredStream = NULL;
				}
				if(hSkeletonEvent != NULL && hSkeletonEvent != INVALID_HANDLE_VALUE) {
					CloseHandle(hSkeletonEvent);
					hSkeletonEvent = NULL;
				}
				kinectInstance->Release();
				kinectInstance = NULL;
			}
		} catch(...) {
			//destructor never throws
		}
	}

	string MicrosoftGrabber::getName () const {
		return std::string ("MicrosoftGrabber");
	}

	float MicrosoftGrabber::getFramesPerSecond () const {
		return 30.0f;
	}


#pragma endregion

	//Camera Functions
#pragma region Camera
	void MicrosoftGrabber::GotColor() {
		NUI_IMAGE_FRAME pImageFrame;
		HRESULT hr = kinectInstance->NuiImageStreamGetNextFrame(hColorStream,0,&pImageFrame);
		if ( FAILED( hr ) ) {
			throw exception("Could not get next color frame from kinect");
		}
		//convert here
		boost::shared_ptr<Mat> img = convertToRGBMat(pImageFrame);
		m_rgbTime = pImageFrame.liTimeStamp.QuadPart;
		if (image_signal_->num_slots () > 0) {
			//cout << "img signal num slot!" << endl;
			image_signal_->operator()(img);
		}
		if (num_slots<sig_cb_microsoft_point_cloud_rgba>() > 0 || all_data_signal_->num_slots() > 0)
			rgb_sync_.add0 (img, m_rgbTime);
		kinectInstance->NuiImageStreamReleaseFrame( hColorStream, &pImageFrame );
	}

	boost::shared_ptr<cv::Mat>
		MicrosoftGrabber::convertToRGBMat (const NUI_IMAGE_FRAME &pImageFrame) const {
			INuiFrameTexture * pTexture = pImageFrame.pFrameTexture;
			NUI_LOCKED_RECT LockedRect;
			pTexture->LockRect( 0, &LockedRect, NULL, 0 );

			boost::shared_ptr<Mat> img (new Mat(imgHeight,imgWidth,CV_8UC3));
			if ( LockedRect.Pitch != 0 ) //should be in this if statement otherwise erronous info
			{
				//mapper->MapColorFrameToDepthFrame(NUI_IMAGE_TYPE_COLOR, colorRes, depthRes, 
				//Get Image info and put it into new image (weird arithmetic is to flip the image on the fly)
				BYTE * pBuffer = (BYTE*) LockedRect.pBits;
				int safeWidth = imgWidth - 1, count = safeWidth, loop = 0, multiplier = (imgWidth << 1) - 1;
				Mat_<Vec3b>::iterator pOut = img->begin<Vec3b>();
				pOut += count;
				while(loop < imgHeight) {
					(*pOut)[0] = *pBuffer++;
					(*pOut)[1] = *pBuffer++;
					(*pOut)[2] = *pBuffer++;
					pBuffer++;
					if (count <= 0) {
						loop++;
						if(loop < imgHeight) {
							pOut += multiplier;
							count = safeWidth;
						}
					} else {
						--pOut;
						--count;
					}
				}
			}
			// We're done with the texture so unlock it
			pTexture->UnlockRect(0);
			return (img);
	}

#pragma endregion

	//Depth Functions
#pragma region Depth

	void MicrosoftGrabber::GotDepth() {
		NUI_IMAGE_FRAME pImageFrame;
		INuiFrameTexture *pTexture = NULL;
		HRESULT hr = kinectInstance->NuiImageStreamGetNextFrame(hDepthStream,0,&pImageFrame );
		if ( FAILED( hr ) ) {
			throw exception("Could not get next depth frame from kinect");
		}
		hr = kinectInstance->NuiImageFrameGetDepthImagePixelFrameTexture(hDepthStream, &pImageFrame, &m_nearMode, &pTexture);
		if ( FAILED( hr ) ) {
			throw exception("Could not get extended depth data from kinect");
		}
		boost::shared_ptr<MatDepth> depth_img = convertTo32SMat(pTexture);
		m_depthTime = pImageFrame.liTimeStamp.QuadPart;
		if (depth_image_signal_->num_slots () > 0) {
			//cout << "img signal num slot!" << endl;
			depth_image_signal_->operator()(depth_img);
		}
		if (num_slots<sig_cb_microsoft_point_cloud_rgba>() > 0 || all_data_signal_->num_slots() > 0)
			rgb_sync_.add1 (depth_img, m_depthTime);
		kinectInstance->NuiImageStreamReleaseFrame(hDepthStream, &pImageFrame );
	}

	boost::shared_ptr<MatDepth>
		MicrosoftGrabber::convertTo32SMat (INuiFrameTexture *pTexture) const {
			NUI_LOCKED_RECT LockedRect;
			pTexture->LockRect( 0, &LockedRect, NULL, 0 );
			boost::shared_ptr<MatDepth> img ((MatDepth*)(new Mat(depthHeight,depthWidth,CV_32S)));
			if ( LockedRect.Pitch != 0 )
			{
				Mat_<int>::iterator pOut = img->begin<int>();
				pOut += depthWidth - 1;
				NUI_DEPTH_IMAGE_PIXEL *pBuffer =  (NUI_DEPTH_IMAGE_PIXEL *) LockedRect.pBits;
				for( int y = 0 ; y < depthHeight ; y++ )
				{
					for( int x = 0 ; x < depthWidth ; x++ )
					{
						*pOut-- = (int)pBuffer->depth;
						pBuffer++;
					}
					pOut += (depthWidth << 1);
				}
			}
			// We're done with the texture so unlock it
			pTexture->UnlockRect(0);
			img->addref(); //I have to do this or OpenCV will try to release the Mat too early. I'm not sure why
			return (img);
	}
#pragma endregion

#pragma region Cloud
	void MicrosoftGrabber::imageDepthImageCallback (const boost::shared_ptr<Mat> &image,
		const boost::shared_ptr<MatDepth> &depth_image)
	{
		boost::shared_ptr<PointCloud<PointXYZRGBA>> cloud;
		// check if we have color point cloud slots
		if(point_cloud_rgba_signal_->num_slots() > 0 || all_data_signal_->num_slots() > 0)
			cloud = convertToXYZRGBAPointCloud(image, depth_image);
		if (point_cloud_rgba_signal_->num_slots () > 0)
			point_cloud_rgba_signal_->operator()(cloud);
		if(all_data_signal_->num_slots() > 0) {
			boost::shared_ptr<KinectData> data (new KinectData(*image,*depth_image,*cloud));
			all_data_signal_->operator()(data);
		}
	}

	void MicrosoftGrabber::GetPointCloudFromData(const boost::shared_ptr<Mat> &img, const boost::shared_ptr<MatDepth> &depth, boost::shared_ptr<PointCloud<PointXYZRGBA>> &cloud, bool useZeros, bool alignToColor, bool preregistered) const
	{
		assert(!img->empty() && !depth->empty());

		PointCloud<PointXYZRGBA>::iterator pCloud = cloud->begin();
		Mat_<int>::const_iterator pDepth = depth->begin<int>();
		int safeWidth = img->cols - 1, safeHeight = img->rows - 1, safeDepthWidth = depthWidth - 1, safeDepthHeight = depthHeight - 1;
		float cx_d = KINECT_CX_D, cy_d = KINECT_CY_D;
		bool res_320 = (depthWidth == 320);
		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		for(int j = 0; j < depthHeight; j++) {
			for(int i = 0; i < depthWidth; i++) {
				PointXYZRGBA loc;
				Vec3b color;
				LONG x = res_320 ? i: i >> 1, y = res_320 ? j : j >> 1;
				if(!preregistered) {
					kinectInstance->NuiImageGetColorPixelCoordinatesFromDepthPixel(colorRes,NULL,LONG(320-x),LONG(y),(*pDepth)<<3,&x,&y);
					x = Clamp<int>(safeWidth-x,0,safeWidth);
					y = Clamp<int>(y,0,safeHeight);
					color = img->at<Vec3b>(y,x);
				} else
					color = img->at<Vec3b>(j,i);
				loc.b = color[0];
				loc.g = color[1];
				loc.r = color[2];
				loc.a = 255;
				if(*pDepth == 0) {
					/*loc.x = float(((float)i - cx_d) * KINECT_FX_D);
					if(preregistered)
						loc.y = float(((float)j - cy_d) * KINECT_FY_D);
					else
						loc.y = float(((float)(safeDepthHeight - j) - cy_d) * KINECT_FY_D);*/
					loc.x = loc.y = loc.z = bad_point;
				} else {
					const double newDepth = (*pDepth * 0.001f); //convert from millimeters to meters
					loc.x = float(((float)i - cx_d) * newDepth * KINECT_FX_D);
					if(preregistered)
						loc.y = float(((float)j - cy_d) * newDepth * KINECT_FY_D);
					else
						loc.y = float(((float)(safeDepthHeight - j) - cy_d) * newDepth * KINECT_FY_D);
					loc.z = float(newDepth);
				}
				//cout << "Iter: " << i << ", " << j << endl;
				if(!preregistered && alignToColor) {
					(*cloud)(x,y) = loc;
					//for weird resolution differences
					if(imgWidth == depthWidth << 1) {
						//TOBE DONE LATER
						/*float x_add = loc.z * KINECT_FX_D, y_add = loc.z * KINECT_FY_D;
						if(y + 1 < imgHeight)
						cloud->Set(Point3D<Bgr>(loc.x,loc.y + y_add,loc.z,img(x,y+1),*pDepth!=0),x + (y + 1) * img.Width());
						if(x + 1 < imgWidth)
						cloud->Set(Point3D<Bgr>(loc.x + x_add,loc.y,loc.z,img(x+1,y),*pDepth!=0),x + 1 + y * img.Width());
						if(y + 1 < imgHeight && x + 1 < imgWidth)
						cloud->Set(Point3D<Bgr>(loc.x + x_add,loc.y + y_add,loc.z,img(x+1,y+1),*pDepth!=0),x + 1 + (y + 1) * img.Width());*/
					}
				} else {
					*pCloud = loc;
				}
				++pDepth; ++pCloud;
			}
		}
		if(!preregistered && alignToColor) {
			pCloud = cloud->begin();
			Mat_<Vec3b>::const_iterator pColor = img->begin<Vec3b>();
			while(pCloud != cloud->end()) {
				if(pCloud->z == 0) {
					pCloud->b = (*pColor)[0];
					pCloud->g = (*pColor)[1];
					pCloud->r = (*pColor)[2];
					pCloud->a = 255;
				}
				++pCloud; ++pColor;
			}
		}		
	}

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> MicrosoftGrabber::convertToXYZRGBAPointCloud (const boost::shared_ptr<cv::Mat> &image,
		const boost::shared_ptr<MatDepth> &depth_image) const {
			boost::shared_ptr<PointCloud<PointXYZRGBA> > cloud (new PointCloud<PointXYZRGBA>);

			cloud->header.frame_id =  "/microsoft_rgb_optical_frame";
			cloud->height = std::max (imgHeight, depthHeight);
			cloud->width = std::max (imgWidth, depthWidth);
			cloud->is_dense = false;
			cloud->points.resize (cloud->height * cloud->width);
			GetPointCloudFromData(image,depth_image,cloud,true,false,false);
			cloud->sensor_origin_.setZero ();
			cloud->sensor_orientation_.w () = 1.0;
			cloud->sensor_orientation_.x () = 0.0;
			cloud->sensor_orientation_.y () = 0.0;
			cloud->sensor_orientation_.z () = 0.0;
			return (cloud);
	}

#pragma endregion
};