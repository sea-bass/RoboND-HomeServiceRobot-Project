/* Marker Spawning Node
 * Source Provided by ROS tutorials: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
 * Modified by Sebastian Castro, 2019
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Define pickup position threshold (in meters)
#define PICKUP_THRESHOLD 0.2f

// Create a pose listener class to handle the pose
class PoseListener
{
public:
  float pickupPos[2];
  float dropoffPos[2];
  bool pickupReached;
  visualization_msgs::Marker marker;
  ros::Publisher marker_pub;
  void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
};

// Pose listener callback function
void PoseListener::callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
{
    // Unpack the X and Y positions from the AMCL pose message
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    // ROS_INFO("I heard: [%f %f]", x, y);
  
    // Delete the marker once pickup location is reached
    if ((!this->pickupReached) && (sqrt( pow(x-pickupPos[0],2) + pow(y-pickupPos[1],2) ) < PICKUP_THRESHOLD)) {
      // Delete the marker
      ROS_INFO("Pickup location reached. Deleting pickup marker");
      this->marker.action = visualization_msgs::Marker::DELETE;
      this->marker_pub.publish(this->marker);
      this->pickupReached = true;
      
      // Then, wait 5 seconds and publish elsewhere
      ros::Duration(5.0).sleep();
      ROS_INFO("Publishing marker at dropoff location");
      this->marker.action = visualization_msgs::Marker::ADD;
      this->marker.pose.position.x = this->dropoffPos[0];
      this->marker.pose.position.y = this->dropoffPos[1];
      this->marker.color.r = 1.0f;
      this->marker.color.g = 0.0f;
      this->marker.color.b = 0.0f;
      this->marker.color.a = 1.0;
      this->marker_pub.publish(this->marker);
    }

}

int main( int argc, char** argv )
{
  // Create listener class with pickup and dropoff positions
  PoseListener listener;
  listener.pickupReached = false;
  listener.pickupPos[0] = 6.0; // X
  listener.pickupPos[1] = 4.0; // Y
  listener.dropoffPos[0] = 0.0; // X
  listener.dropoffPos[1] = 4.0; // Y
    
  // Initialize the ROS node
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  
  // Create marker publisher
  listener.marker_pub = n.advertise<visualization_msgs::Marker>("nav_marker", 1);
 
  // Create AMCL pose subscriber
  ros::Subscriber pose_sub = n.subscribe("amcl_pose", 1000, &PoseListener::callback, &listener);
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  if (ros::ok())
  {
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    listener.marker.header.frame_id = "/map";
    listener.marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    listener.marker.ns = "nav_marker";
    listener.marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    listener.marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    listener.marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    listener.marker.pose.position.x = listener.pickupPos[0];
    listener.marker.pose.position.y = listener.pickupPos[1];
    listener.marker.pose.position.z = 0.0;
    listener.marker.pose.orientation.x = 0.0;
    listener.marker.pose.orientation.y = 0.0;
    listener.marker.pose.orientation.z = 0.0;
    listener.marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    listener.marker.scale.x = 0.25;
    listener.marker.scale.y = 0.25;
    listener.marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    listener.marker.color.r = 0.0f;
    listener.marker.color.g = 0.0f;
    listener.marker.color.b = 1.0f;
    listener.marker.color.a = 1.0;

    listener.marker.lifetime = ros::Duration();

    // Publish the marker the first time
    // NOTE: We first wait until a subscriber is created (i.e. added in RViz)
    while (listener.marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO("Publishing marker at pickup location");
    listener.marker_pub.publish(listener.marker);
    
    // Once the first marker is published, we can now execute the rest of the node using the pose subscriber callbacks
    ros::spin();
    
  }
}