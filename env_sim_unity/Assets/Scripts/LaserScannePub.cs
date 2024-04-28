using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;

public class LaserScanSensor : MonoBehaviour
{
    public string topic = "/point_cloud";
    [FormerlySerializedAs("TimeBetweenScansSeconds")]
    public double PublishPeriodSeconds = 0.1;
    public float RangeMetersMin = 0;
    public float RangeMetersMax = 1000;
    

    public float fov_horizontal = 360;
    public float fov_vertical = 45;

    public float angularResolution_vertical = 1;
    public float angularResolution_horizontal = 1;

    // Change the scan start and end by this amount after every publish
    public float ScanOffsetAfterPublish = 0f;
    public float TimeBetweenMeasurementsSeconds = 0.01f;
    public string LayerMaskName = "DiffRobot";
    public string FrameId = "hokuyo_link";

    float m_CurrentScanAngleStart_h;
    float m_CurrentScanAngleEnd_h;
    float m_CurrentScanAngleStart_v;
    float m_CurrentScanAngleEnd_v;
    ROSConnection m_Ros;
    double m_TimeNextScanSeconds = -1;
    int m_NumMeasurementsTaken_h;

    uint numPoints;
    int raw_data_indx;
    int NumMeasurementsPerScan_h;
    int NumMeasurementsPerScan_v;
    uint raw_data_len;
    byte[] raw_data;

    bool isScanning = false;
    double m_TimeLastScanBeganSeconds = -1;

    protected virtual void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();

        m_Ros.RegisterPublisher<PointCloud2Msg>(topic);

        m_CurrentScanAngleStart_h = -fov_horizontal/2;
        m_CurrentScanAngleEnd_h = fov_horizontal/2;
        m_CurrentScanAngleStart_v = -fov_vertical/2;
        m_CurrentScanAngleEnd_v = fov_vertical/2;

        m_TimeNextScanSeconds = Clock.Now + PublishPeriodSeconds;

        NumMeasurementsPerScan_h = Mathf.FloorToInt(fov_horizontal/angularResolution_horizontal);
        NumMeasurementsPerScan_v = Mathf.FloorToInt(fov_vertical/angularResolution_vertical);

        numPoints = (uint)(NumMeasurementsPerScan_h*NumMeasurementsPerScan_v);

        raw_data_len = 16*numPoints;
        raw_data = new byte[raw_data_len];
        raw_data_indx = 0;
    }

    void BeginScan()
    {
        isScanning = true;
        m_TimeLastScanBeganSeconds = Clock.Now;
        m_TimeNextScanSeconds = m_TimeLastScanBeganSeconds + PublishPeriodSeconds;
        m_NumMeasurementsTaken_h = 0;
        raw_data_indx = 0;
    }

    public void EndScan()
    {
        // if (ranges.Count == 0)
        // {
        //     Debug.LogWarning($"Took {m_NumMeasurementsTaken} measurements but found no valid ranges");
        // }
        // else if (ranges.Count != m_NumMeasurementsTaken || ranges.Count != NumMeasurementsPerScan_h)
        // {
        //     Debug.LogWarning($"Expected {NumMeasurementsPerScan_h} measurements. Actually took {m_NumMeasurementsTaken}" +
        //                      $"and recorded {ranges.Count} ranges.");
        // }

        var timestamp = new TimeStamp(Clock.time);
        
        var msg = new PointCloud2Msg
        {
            header = new HeaderMsg
            {
                frame_id = FrameId,
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
        
        m_Ros.Publish(topic, msg);

        m_NumMeasurementsTaken_h = 0;
        raw_data_indx = 0;
        isScanning = false;
        var now = (float)Clock.time;
        if (now > m_TimeNextScanSeconds)
        {
            Debug.LogWarning($"Failed to complete scan started at {m_TimeLastScanBeganSeconds:F} before next scan was " +
                             $"scheduled to start: {m_TimeNextScanSeconds:F}, rescheduling to now ({now:F})");
            m_TimeNextScanSeconds = now;
        }

        // m_CurrentScanAngleStart_h += ScanOffsetAfterPublish;
        // m_CurrentScanAngleEnd_h += ScanOffsetAfterPublish;
        // if (m_CurrentScanAngleStart_h > 360f || m_CurrentScanAngleEnd_h > 360f)
        // {
        //     m_CurrentScanAngleStart_h -= 360f;
        //     m_CurrentScanAngleEnd_h -= 360f;
        // }
    }

    public void Update()
    {
        if (!isScanning)
        {
            if (Clock.NowTimeInSeconds < m_TimeNextScanSeconds)
            {
                return;
            }

            BeginScan();
        }


        var measurementsSoFar_h = TimeBetweenMeasurementsSeconds == 0 ? NumMeasurementsPerScan_h :
            1 + Mathf.FloorToInt((float)(Clock.time - m_TimeLastScanBeganSeconds) / TimeBetweenMeasurementsSeconds);
        if (measurementsSoFar_h > NumMeasurementsPerScan_h)
            measurementsSoFar_h = NumMeasurementsPerScan_h;

        var yawBaseDegrees = transform.rotation.eulerAngles.y; // yaw of object this script is attached to i.e. laser_scanner
        var pitchBaseDegrees = transform.rotation.eulerAngles.x; // pitch of object this script is attached to i.e. laser_scanner
        var rollBaseDegrees = transform.rotation.eulerAngles.z; // roll of object this script is attached to i.e. laser_scanner

        while (m_NumMeasurementsTaken_h < measurementsSoFar_h)
        {
            var t = m_NumMeasurementsTaken_h / (float)NumMeasurementsPerScan_h;

            var yawSensorDegrees = Mathf.Lerp(m_CurrentScanAngleStart_h, m_CurrentScanAngleEnd_h, t);
            var yawDegrees = yawBaseDegrees + yawSensorDegrees;

            int m_NumMeasurementsTaken_v = 0;

            while (m_NumMeasurementsTaken_v<NumMeasurementsPerScan_v)
            {

                var t2 = m_NumMeasurementsTaken_v / (float)NumMeasurementsPerScan_v;
                var pitchSensorDegrees = Mathf.Lerp(m_CurrentScanAngleStart_v, m_CurrentScanAngleEnd_v, t2);
                var pitchDegrees = pitchBaseDegrees + pitchSensorDegrees;

                var directionVector = Quaternion.Euler(pitchDegrees, yawDegrees, 0f) * Vector3.forward;


                var measurementStart = RangeMetersMin * directionVector + transform.position;
                var measurementRay = new Ray(measurementStart, directionVector);
                var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax);
                // Only record measurement if it's within the sensor's operating range
                // x=z, y=-x, z=y
                if (foundValidMeasurement)
                {
                    BitConverter.GetBytes(hit.point.z).CopyTo(raw_data, raw_data_indx * 16);
                    BitConverter.GetBytes(-hit.point.x).CopyTo(raw_data, raw_data_indx * 16+4);
                    BitConverter.GetBytes(hit.point.y).CopyTo(raw_data, raw_data_indx * 16+8);
                    BitConverter.GetBytes(0.0f).CopyTo(raw_data, raw_data_indx * 16 + 12);
                }
                else
                {
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16);
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16+4);
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16+8);
                    BitConverter.GetBytes(0.0f).CopyTo(raw_data, raw_data_indx * 16 + 12);

                }
                // Even if Raycast didn't find a valid hit, we still count it as a measurement
                ++m_NumMeasurementsTaken_v;
                ++raw_data_indx;
            }
            ++m_NumMeasurementsTaken_h;
        }
        
        if (m_NumMeasurementsTaken_h >= NumMeasurementsPerScan_h)
        {
            if (m_NumMeasurementsTaken_h > NumMeasurementsPerScan_h)
            {
                Debug.LogError($"LaserScan has {m_NumMeasurementsTaken_h} measurements but we expected {NumMeasurementsPerScan_h}");
            }
            EndScan();
        }

    }
}