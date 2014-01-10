
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

DWORD ProcessThread(LPVOID pParam) {
	pcl::MicrosoftGrabber *p = (pcl::MicrosoftGrabber*) pParam;
	p->ProcessThreadInternal();
	return 0;
}

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
		/*ir_image_signal_       = createSignal<sig_cb_microsoft_ir_image> ();
		point_cloud_signal_    = createSignal<sig_cb_microsoft_point_cloud> ();
		point_cloud_i_signal_  = createSignal<sig_cb_microsoft_point_cloud_i> ();
		point_cloud_rgb_signal_   = createSignal<sig_cb_microsoft_point_cloud_rgb> ();
		point_cloud_rgba_signal_  = createSignal<sig_cb_microsoft_point_cloud_rgba> ();*/
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
		if (image_signal_->num_slots () > 0) {
			//cout << "img signal num slot!" << endl;
			image_signal_->operator()(convertToRGBMat(pImageFrame));
		}
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
				//m_rgbTime = pImageFrame.liTimeStamp.QuadPart;
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

		if (depth_image_signal_->num_slots () > 0) {
			//cout << "img signal num slot!" << endl;
			depth_image_signal_->operator()(convertTo32SMat(pTexture));
		}
		kinectInstance->NuiImageStreamReleaseFrame(hDepthStream, &pImageFrame );
	}

	boost::shared_ptr<cv::Mat>
		MicrosoftGrabber::convertTo32SMat (INuiFrameTexture *pTexture) const {
			NUI_LOCKED_RECT LockedRect;
			pTexture->LockRect( 0, &LockedRect, NULL, 0 );
			boost::shared_ptr<Mat> img (new Mat(depthHeight,depthWidth,CV_32S));
			if ( LockedRect.Pitch != 0 )
			{
				//m_depthTime = pImageFrame.liTimeStamp.QuadPart;
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
			return (img);
	}
#pragma endregion
};