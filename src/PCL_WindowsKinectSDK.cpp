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


using namespace std;
using namespace pcl;

class SimpleMicrosoftViewer
{
public:
	SimpleMicrosoftViewer () : viewer ("PCL Microsoft Viewer") {}

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		//cout << "Frame!!" << endl;
		if (!viewer.wasStopped())
			viewer.showCloud (cloud);
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
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
			boost::bind (&SimpleMicrosoftViewer::cloud_cb_, this, _1);

		my_interface->registerCallback (f);

		my_interface->start ();
		Sleep(30);
		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}

		my_interface->stop ();
	}

	pcl::visualization::CloudViewer viewer;
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