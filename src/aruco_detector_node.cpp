#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

class ArucoDetect
{
public:
    ArucoDetect(ros::NodeHandle &nh) : it(nh)
    {
        sub = it.subscribe("/camera_front/image_raw", 1, &ArucoDetect::imageCallback, this);
        cam_info = nh.subscribe("/camera_front/camera_info", 1, &ArucoDetect::cameraInfoCallback, this);
        pos_pub = nh.advertise<geometry_msgs::Pose>("/landpose", 1);
        pub = it.advertise("/aruco_detect/image", 1);

        // Advertise output video (optional)

        // Dictionary
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        detectorParams = cv::aruco::DetectorParameters::create();
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoPtr &msg)
    {
        camMatrix = (cv::Mat_<double>(3, 3) << msg->K[0], msg->K[1], msg->K[2],
                     msg->K[3], msg->K[4], msg->K[5],
                     msg->K[6], msg->K[7], msg->K[8]);

        distCoeffs = cv::Mat(msg->D).clone();
        got_camera_info = true;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;

        // You can read camera parameters from tutorial_camera_params.yml

        cv::Mat objPoints(4, 1, CV_32FC3);
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams);

        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            ROS_INFO_STREAM("Detected " << ids.size() << " markers");
        }

        size_t nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if (!ids.empty())
        {
            // Calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++)
            {
                solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
            cv::Vec3d t = tvecs[0];
            cv::Vec3d r = rvecs[0];
            ROS_INFO("location %f %f %f", t[0], t[1], t[2]);
            ROS_INFO("location %f %f %f ", r[0], r[1], r[2]);

            cam_pose.position.x = t[0];
            cam_pose.position.y = t[1];
            cam_pose.position.z = t[2];

            cv::Mat R;
            cv::Rodrigues(rvecs[0], R);

            tf::Matrix3x3 tf_R(
                R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

            tf::Quaternion q;
            tf_R.getRotation(q);

            geometry_msgs::Quaternion q_msg;
            tf::quaternionTFToMsg(q, q_msg);

            double roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
            double pitch = asin(-R.at<double>(2, 0));
            double yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));

            ROS_INFO("yaw : %f",yaw) ;

            q.setRPY(roll, pitch, yaw);
            cam_pose.orientation.w = q.w() ;
            cam_pose.orientation.x = q.x() ;
            cam_pose.orientation.y = q.y() ;
            cam_pose.orientation.z = q.z() ;

            pos_pub.publish(cam_pose);
        }

        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            for (unsigned int i = 0; i < ids.size(); i++)
                cv::drawFrameAxes(frame, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
        }

        // Publish annotated image
        pub.publish(cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg());
    }

private:
    cv::Mat camMatrix, distCoeffs;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    tf::Quaternion q;
    ros::Subscriber cam_info;
    ros::Publisher pos_pub;
    geometry_msgs::Pose cam_pose;
    float markerLength = 0.5;
    bool got_camera_info;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_detector_node");
    ros::NodeHandle nh;
    ArucoDetect detet(nh);
    ros::spin();
    return 0;
}