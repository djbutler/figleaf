#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

const int N_ARROWS = 3;
float arrow_data[N_ARROWS][7] =  {
//  p_x, p_y, p_z, r_x, r_y, r_z, r_w
    {0.0, 0.0, 2.0, 0.0, 1.0, 0.0, 1.0},
    {0.0, 1.0, 2.0, 0.0, 1.0, 0.0, 1.0},
    {1.0, 0.0, 2.0, 0.0, 1.0, 0.0, 1.0}
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "rviz_arrows");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Set our initial shape type to be an arrow
  uint32_t shape = visualization_msgs::Marker::ARROW;

  while (ros::ok())
  {
    for (int i = 0; i < N_ARROWS; i++) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/odom_combined";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "rviz_arrows";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = arrow_data[i][0];
    marker.pose.position.y = arrow_data[i][1];
    marker.pose.position.z = arrow_data[i][2];
    marker.pose.orientation.x = arrow_data[i][3];
    marker.pose.orientation.y = arrow_data[i][4];
    marker.pose.orientation.z = arrow_data[i][5];
    marker.pose.orientation.w = arrow_data[i][6];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
    };
    r.sleep();
  }
}

