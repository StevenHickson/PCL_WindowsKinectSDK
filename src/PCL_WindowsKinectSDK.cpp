#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>

#include "Microsoft_grabber.h"
#include <pcl/visualization/cloud_viewer.h>
/*#include <FaceTrackLib.h>
#include <KinectInteraction.h>
#include <NuiKinectFusionApi.h>
#include <NuiKinectFusionDepthProcessor.h>
#include <NuiKinectFusionVolume.h>*/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace pcl;
using namespace cv;

class SimpleMicrosoftViewer
{
public:
	//SimpleMicrosoftViewer () : viewer ("PCL Microsoft Viewer") {}

	void img_cb_ (const boost::shared_ptr<const cv::Mat> &img)
	{
		/*if (!viewer.wasStopped())
			viewer.showCloud (cloud);*/
		imshow("image", *img);
		waitKey(1);
	}

	void depth_cb_ (const boost::shared_ptr<const MatDepth> &img) 
	{
		/*if (!viewer.wasStopped())
			viewer.showCloud (cloud);*/
		imshow("depth", *img);
		waitKey(1);
	}

	/*void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		static unsigned count = 0;
		static double last = pcl::getTime ();
		if (++count == 30)
		{
			double now = pcl::getTime ();
			std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
			count = 0;
			last = now;
		}
	}*/

	void run ()
	{
		// create a new grabber for OpenNI devices
		pcl::Grabber* my_interface = new pcl::MicrosoftGrabber();

		// make callback function from member function
		boost::function<void (const boost::shared_ptr<const MatDepth>&)> f2 =
			boost::bind (&SimpleMicrosoftViewer::depth_cb_, this, _1);
		boost::function<void (const boost::shared_ptr<const Mat>&)> f =
			boost::bind (&SimpleMicrosoftViewer::img_cb_, this, _1);

		my_interface->registerCallback (f);
		my_interface->registerCallback (f2);

		my_interface->start ();
		Sleep(30);
		while (true)
		{
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}

		my_interface->stop ();
	}

	//pcl::visualization::CloudViewer viewer;
};

int
	main (int argc, char** argv)
{
	PointCloud<PointXYZ> depth;
	PointCloud<PointXYZRGB> color;
	PointCloud<PointXYZRGB> cloud;
	try {
		SimpleMicrosoftViewer v;
		v.run ();
	} catch (pcl::PCLException e) {
		cout << e.detailedMessage() << endl;
	} catch (std::exception &e) {
		cout << e.what() << endl;
	}
	cin.get();
	return (0);
}