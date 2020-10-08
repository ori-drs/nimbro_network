// Throttle /tf bandwidth
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <tf/transform_listener.h>

#include <tf2_msgs/TFMessage.h>

#include <boost/foreach.hpp>

boost::scoped_ptr<tf::TransformListener> g_tf;
ros::Publisher pub;

void sendTransforms(std::string valid_namespace)
{
	std::vector<std::string> frames;
	g_tf->getFrameStrings(frames);

	tf2_msgs::TFMessage msg;
	msg.transforms.reserve(frames.size());

	ros::Time now = ros::Time::now();

	BOOST_FOREACH(std::string frame, frames)
	{
		std::string parentFrame;
		if(!g_tf->getParent(frame, ros::Time(0), parentFrame))
			continue;

		// If there is a namespace specified from which we will send frames, and
		// neither the parent nor the child frame contains it, then skip this
		// frame.
		if (!valid_namespace.empty() && (frame.find(valid_namespace) == std::string::npos || parentFrame.find(valid_namespace) == std::string::npos)) {
			ROS_DEBUG("Skipping frame %s with parent %s because neither frame is in the valid namespace %s.", frame.c_str(), parentFrame.c_str(), valid_namespace.c_str());
			continue;
		}

		tf::StampedTransform transform;
		try
		{
			g_tf->lookupTransform(
				parentFrame,
				frame,
				ros::Time(0),
				transform
			);
		}
		catch(tf::TransformException&)
		{
			continue;
		}

		geometry_msgs::TransformStamped m;
		tf::transformStampedTFToMsg(transform, m);

		msg.transforms.push_back(m);
	}

	pub.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_throttle");

	ros::NodeHandle nh("~");

	g_tf.reset(new tf::TransformListener(nh, ros::Duration(10.0)));

	double rate;
	nh.param<double>("rate", rate, 4.0);

	std::string valid_namespace;
	// If this parameter is set, this node will only republish frames for which
	// either parent or child contain this string
	nh.param<std::string>("valid_namespace", valid_namespace, "");
	if (!valid_namespace.empty()) {
		ROS_INFO("Will only republish frames which contain %s", valid_namespace.c_str());
	} else {
		ROS_INFO("Will republish all frames.");
	}
	
	ros::Timer timer = nh.createTimer(
		ros::Duration(1.0 / rate),
		boost::bind(&sendTransforms, valid_namespace)
	);
	pub = nh.advertise<tf2_msgs::TFMessage>("tf", 1);

	ros::spin();

	return 0;
}
