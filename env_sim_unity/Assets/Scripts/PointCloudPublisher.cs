using UnityEngine;
using UnityEngine.Serialization;
using System.Collections.Generic;

using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;


public class PointCloudPublisher : MonoBehaviour
{
    

    public GameObject laser_sensor_link;
    public string point_cloud_topic = "/point_cloud";
    public string pose_topic = "/laser_scan_pose";

    public float RangeMetersMin = 0;
    public float RangeMetersMax = 200;

    public float fov_horizontal = 360;
    public float fov_vertical_start_angle = -25;
    public float fov_vertical_end_angle = 15;
    public float angularResolution_horizontal = 0.4f;
    public int channels = 32;

    public bool isVLP32 = true;    

    ROSConnection ros;
    LaserSensor3D laser_sensor_3d;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(point_cloud_topic);
        ros.RegisterPublisher<PoseStampedMsg>(pose_topic);

        laser_sensor_3d = new LaserSensor3D(laser_sensor_link, RangeMetersMin, RangeMetersMax, fov_horizontal, fov_vertical_start_angle, fov_vertical_end_angle, angularResolution_horizontal, channels, isVLP32);
    }

    void Update()
    {

        byte[] raw_data = laser_sensor_3d.getScanData();

        uint raw_data_len = (uint)raw_data.Length;
        uint numPoints = raw_data_len/16;

        var timestamp = new TimeStamp(Clock.time);
        
        PointCloud2Msg point_cloud_msg = new PointCloud2Msg
        {
            header = new HeaderMsg
            {
                frame_id = laser_sensor_link.name,
                stamp = new TimeMsg
                {
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                }
            },
            height = 1,
            width = numPoints,
            fields = new PointFieldMsg[]
            {
                new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("i", 12, PointFieldMsg.FLOAT32, 1)
            },
            is_bigendian = false,
            point_step = 16,
            row_step = raw_data_len,
            data = raw_data,
            is_dense = false,
        };  

        PoseStampedMsg pose_msg = new PoseStampedMsg
        {
            header = point_cloud_msg.header,
            pose = new PoseMsg
            {
                position = laser_sensor_link.transform.position.To<FLU>(),
                orientation = laser_sensor_link.transform.rotation.To<FLU>(),
            }
        };

        ros.Publish(point_cloud_topic, point_cloud_msg);
        ros.Publish(pose_topic, pose_msg);
    }
}
