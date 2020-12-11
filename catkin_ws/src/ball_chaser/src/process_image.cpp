#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
bool moving_state = false;
float cur_lin_x=0, cur_ang_z=0;


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  //ROS_INFO_STREAM( "Driving robot towards white ball - linear_x:" << lin_x << ", angular_z:" << ang_z);

  // Request updated movement
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Call the safe_move service and pass the requested join angles
  if ( !client.call(srv) )
    ROS_ERROR( "Failed to call service command_robot" );
}

bool find_white( const sensor_msgs::Image img, int start, int end )
{
    int white_pixel = 255;
    int i, j;
    for ( i = start, j = 0; j < img.height; ) {
        if ( img.data[i++] == white_pixel && img.data[i++] == white_pixel && img.data[i++] == white_pixel ) {
            return true;
        }
        if ( i % img.step >= end-1 ) {
            i = ++j * img.step + start;
        }
    }
    return false;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    bool found_white = false;
    float new_lin_x=0, new_ang_z=0;
    int max_left = img.step*0.25, max_center = img.step*0.75;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // First, let's check left
    if ( found_white = find_white( img, 0, max_left ) ) {
        new_lin_x = 0.1;
        new_ang_z = 0.1;
    }

    // Next, let's check right
    if ( !found_white && (found_white = find_white( img, max_center, img.step )) ) {
        new_lin_x = 0.1;
        new_ang_z = -0.1;
    }

    // Finally, let's check center
    if ( !found_white && (found_white = find_white( img, max_left, max_center )) ) {
        new_lin_x = 0.1;
        new_ang_z = 0;
    }

    if ( found_white ) {
        // Only update robot if movement has changed
        if ( new_lin_x != cur_lin_x || new_ang_z != cur_ang_z ) {
            drive_robot( new_lin_x, new_ang_z );
            cur_lin_x = new_lin_x;
            cur_ang_z = new_ang_z;
            moving_state = true;
        }
    }
    else if ( moving_state ) {
        // No white ball found.  Since robot is moving, stop it
        drive_robot( 0, 0 );
        cur_lin_x = cur_ang_z = 0.0;
        moving_state = false;
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
