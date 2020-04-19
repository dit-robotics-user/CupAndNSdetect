#include <ros/ros.h>
#include <iostream>
#include <aruco_pose/MarkerArray.h>
#include <aruco_pose/Marker.h>
#include "std_msgs/String.h"
#include "math.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <aruco_pose/cup.h>
#include <aruco_pose/ns.h>

//using namespace std;

#define anggleMargin 20

class Camera
{

public:
    ros::NodeHandle n;

    ros::ServiceServer service1;

    ros::ServiceServer service2;

    int MarkerCplt;

    int CupCplt;

    ros::Subscriber aruco;

    ros::Subscriber cup;

    ros::Publisher pub;

    int count1;

    int count2;

    int count3;

    float sum;

    float last_data;

    float angle;

    float angle2;

    float angle3;

    float last_data2;

    float last_data3;

    std::stringstream strBufferMarker;

    std::stringstream strBufferCup;

    std_msgs::String str;

    std_msgs::String CupTempBuffer;

    std_msgs::String CupDataPast;

    std_msgs::String N_S_Result;

    int CupDataStable;

    int N_S_DataStable;

    float markerX;

    float markerY;

    float markerZ;

    uint32_t cupArray[5];

    uint32_t cupResult[5];

    int CupCheckTime;

    int N_S_CheckTime;

    Camera()
    {
        MarkerCplt = 0;
        CupCplt = 0;
        markerX = 0;
        markerY = 0;
        markerZ = 0;
        pub = n.advertise<std_msgs::String>("Cup_NS", 1);
        aruco = n.subscribe("aruco_detect/markers", 1000, &Camera::markersCallback, this);
        cup = n.subscribe("cup", 1000, &Camera::CupCallback, this);
        service1 = n.advertiseService("cup", &Camera::CupService,this);
        service2 = n.advertiseService("ns", &Camera::N_S_Service,this);
        count1 = 0;
        count2 = 0;
        count3 = 0;
        sum = 0;
        last_data = 0;
        angle = 0;
        angle2 = 0;
        angle3 = 0;
        last_data2 = 0;
        last_data3 = 0;
        CupCheckTime = 0;
        N_S_DataStable = 0;
        CupDataStable = 0;
        N_S_DataStable = 0;
        for (int count = 0; count < 5; count++)
        {
            cupArray[count] = 0;
            cupResult[count] = 0;
        }
    }

    void markersCallback(const aruco_pose::MarkerArray::ConstPtr &markers)
    {

        if (!markers->markers.empty())
        {
            // if(markers->markers[0].id != 17) {
            if (MarkerCplt != 1)
            {
                last_data = angle;
                last_data2 = angle2;
                last_data2 = angle3;
                // ROS_INFO("id:%ld",markers->markers[0].id );
                angle = atan2(
                    2 * (markers->markers[0].pose.orientation.w * markers->markers[0].pose.orientation.z +
                         markers->markers[0].pose.orientation.x * markers->markers[0].pose.orientation.y),
                    1 - 2 * (markers->markers[0].pose.orientation.z * markers->markers[0].pose.orientation.z +
                             markers->markers[0].pose.orientation.y * markers->markers[0].pose.orientation.y));
                angle *= -1;
                angle /= (3.1415926);
                angle *= 180;
                if (angle < 0)
                    angle += 360;

                angle2 = asin(
                    2 * (markers->markers[0].pose.orientation.w * markers->markers[0].pose.orientation.y -
                         markers->markers[0].pose.orientation.x * markers->markers[0].pose.orientation.z));
                angle2 *= -1;
                angle2 /= (3.1415926);
                angle2 *= 180;
                if (angle2 < 0)
                    angle2 += 360;

                angle3 = atan2(
                    2 * (markers->markers[0].pose.orientation.y * markers->markers[0].pose.orientation.z +
                         markers->markers[0].pose.orientation.x * markers->markers[0].pose.orientation.w),
                    1 - 2 * (markers->markers[0].pose.orientation.z * markers->markers[0].pose.orientation.z +
                             markers->markers[0].pose.orientation.w * markers->markers[0].pose.orientation.w));
                angle3 *= -1;
                angle3 /= (3.1415926);
                angle3 *= 180;
                if (angle3 < 0)
                    angle3 += 360;

                if ((angle > last_data - 10 || angle < last_data + 10))
                {
                    count1++;
                    //ROS_INFO("angle = %f", angle);
                    if (count1 == 5)
                    {
                        if (angle >= 360 - anggleMargin || angle <= 0 + anggleMargin)
                        {
                            strBufferMarker << 1;
                            N_S_Result.data = strBufferMarker.str();
                            ROS_INFO("%d",N_S_Result.data[0]);
                             // ROS_INFO("Heading to North");
                        }

                        else
                        {
                            strBufferMarker << 0;
                            N_S_Result.data = strBufferMarker.str();
                            ROS_INFO("%d", N_S_Result.data[0]);
                            // ROS_INFO("Heading to South");
                        }
                        count1 = 0;
                        MarkerCplt = 1;
                        // strBufferMarker.data = strBuffer.str();
                        // pub.publish(strBufferMarker);
                        // ROS_INFO("angle = %f", angle);
                    }
                }
                if ((angle2 > last_data2 - 10 || angle2 < last_data2 + 10))
                    count2++;
                //ROS_INFO("angle = %f", angle);
                if (count2 == 5)
                {
                    count2 = 0;
                    // ROS_INFO("angle2 = %f", angle2);
                }

                if ((angle3 > last_data3 - 10 || angle3 < last_data3 + 10))
                    count3++;
                //ROS_INFO("angle = %f", angle);
                if (count3 == 5)
                {
                    count3 = 0;

                    // ROS_INFO("angle3 = %f", angle3);
                }
            }
            // ROS_INFO("x:%f",markers->markers[0].pose.position.x );
            // ROS_INFO("y:%f",markers->markers[0].pose.position.y );
            // ROS_INFO("z:%f",markers->markers[0].pose.position.z );
            // markerX = markers->markers[0].pose.position.x;
            // markerY = markers->markers[0].pose.position.y;
            // markerZ = markers->markers[0].pose.position.z;
            //ROS_INFO("w:%f",markers->markers[0].pose.orientation.w);
            //ROS_INFO("angle = %f", angle);
        }
        else
        {
            // ROS_INFO("None");
        }
        // }
    }

    void CupCallback(const std_msgs::String::ConstPtr &msg)
    {
        // ROS_INFO("I heard: [%s]", msg->data.c_str());
        // ROS_INFO("%s", msg->data.c_str());
        if (CupCplt != 1)
        {
            strBufferCup << msg->data.c_str();
            CupTempBuffer.data = strBufferCup.str();
            for (int QAQ = 0; QAQ < 5; QAQ++)
            {
                // ROS_INFO("%d", CupTempBuffer.data[QAQ]);
                if (CupTempBuffer.data[QAQ] != CupDataPast.data[QAQ])
                {
                    CupCheckTime = 0;
                    for (int OUO = 0; OUO < 5; OUO++)
                    {
                        CupDataPast.data[OUO] = CupTempBuffer.data[OUO];
                    }
                    break;
                }
                else
                {
                    if (QAQ == 4)
                        CupCheckTime++;
                }
            }
            if (CupCheckTime == 5)
                CupDataStable = 1;
            // ROS_INFO("%d", CupTempBuffer.data[0]);
            if (CupDataStable == 1)
            {
                CupCplt = 1;
                CupDataPast.data = CupTempBuffer.data;
                // ROS_INFO("%s",CupDataPast.data);
                pub.publish(CupDataPast);
            }
        }
    }

    bool CupService(aruco_pose::cup::Request &req, aruco_pose::cup::Response &res)
    {
        for (int count = 0; count < 5; count++)
        {
            ROS_INFO("%d",CupDataPast.data[count]);
            res.CupResult.push_back(CupDataPast.data[count]);
        }
        return true;
    }
    bool N_S_Service(aruco_pose::ns::Request &req, aruco_pose::ns::Response &res)
    {
        ROS_INFO("%s",N_S_Result);
        res.ns = N_S_Result.data[0];
        return true;
    }

    void Publish()
    {
        // if (CupCplt == 1 && MarkerCplt == 1)
        //     // if (MarkerCplt == 1)
        //     {
        // ROS_INFO("FUCK Brian");
        std::stringstream strForAssemble;
        strForAssemble << strBufferCup.str() << strBufferMarker.str();
        // strForAssemble << strBufferMarker.str();
        str.data = strForAssemble.str();
        // pub.publish(str);
        // CupCplt = 0;
        MarkerCplt = 0;
        strBufferCup.str("");
        strBufferCup.clear();
        strBufferMarker.str("");
        strBufferMarker.clear();
        // }
    }

    void transformPoint(const tf::TransformListener &listener)
    {
        geometry_msgs::PointStamped Camera_point;
        Camera_point.header.frame_id = "base_Camera";
        //we'll just use the most recent transform available for our simple example
        Camera_point.header.stamp = ros::Time();
        //just an arbitrary point in space
        Camera_point.point.x = markerX;
        Camera_point.point.y = markerY;
        Camera_point.point.z = markerZ;
        try
        {
            geometry_msgs::PointStamped base_point;
            listener.transformPoint("base_link", Camera_point, base_point);
            // ROS_INFO("base_Camera: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
            //         Camera_point.point.x, Camera_point.point.y, Camera_point.point.z,
            //         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        }

        catch (tf::TransformException &ex)
        {

            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "CameraResult");
    tf::TransformListener listener(ros::Duration(10));
    //ros::Subscriber sub2 = n.subscribe("pub", 1, chatterCallback);
    Camera camera;

    // ros::Timer timer = camera.n.createTimer(ros::Duration(1.0), boost::bind(Camera::transformPoint, boost::ref(listener)));
    while (ros::ok())
    {
        camera.Publish();
        //camera.transformPoint(listener);
        // ros::Duration(2).sleep();
        ros::spinOnce();
    }
    return 0;
}