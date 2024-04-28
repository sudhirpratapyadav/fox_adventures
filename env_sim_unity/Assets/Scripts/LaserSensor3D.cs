using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;

public class LaserSensor3D
{
    float RangeMetersMin;
    float RangeMetersMax;

    int NumMeasurementsPerScan_h;
    int NumMeasurementsPerScan_v;

    float[] scanAngleArray_h;
    float[] scanAngleArray_v;
    
    byte[] raw_data;

    GameObject laser_sensor_link;

    // float avg_time;
    // int total_counts;


    public LaserSensor3D(GameObject _laser_sensor_link, float _RangeMetersMin, float _RangeMetersMax, float _fov_horizontal, float _fov_vertical_start_angle, float _fov_vertical_end_angle, float _angularResolution_horizontal, int channels, bool isVLP32)
    {

        laser_sensor_link = _laser_sensor_link;

        RangeMetersMin = _RangeMetersMin;
        RangeMetersMax = _RangeMetersMax;
        
        float fov_horizontal = _fov_horizontal<=360?_fov_horizontal:360;

        if (_fov_vertical_start_angle < -45)
        {
            _fov_vertical_start_angle = -45;
        }
        else if (_fov_vertical_start_angle > -10)
        {
            _fov_vertical_start_angle = -10;
        }
        if (_fov_vertical_end_angle < 10)
        {
            _fov_vertical_end_angle = 10;
        }
        else if (_fov_vertical_end_angle > 45)
        {
            _fov_vertical_end_angle = 45;
        }

        float ScanAngleStart_h = -fov_horizontal/2;
        float ScanAngleEnd_h = fov_horizontal/2;
        float ScanAngleStart_v = _fov_vertical_start_angle;
        float ScanAngleEnd_v = _fov_vertical_end_angle;

        float angularResolution_horizontal = _angularResolution_horizontal;


        NumMeasurementsPerScan_h = Mathf.FloorToInt((ScanAngleEnd_h - ScanAngleStart_h) / angularResolution_horizontal) + 1;
        if (fov_horizontal==360)
        {
            NumMeasurementsPerScan_h = NumMeasurementsPerScan_h -1;
        }
        scanAngleArray_h = new float[NumMeasurementsPerScan_h];
        for (int i = 0; i < NumMeasurementsPerScan_h; i++)
        {
            scanAngleArray_h[i] = ScanAngleStart_h + i * angularResolution_horizontal;
        }

        if (isVLP32==true)
        {
            NumMeasurementsPerScan_v = 32;
            scanAngleArray_v = new float[] {-25f, -15.639f, -11.31f, -8.843f, -7.254f, -6.148f, -5.333f, -4.667f, -4f, -3.667f, -3.333f, -3f, -2.667f, -2.333f, -2f, -1.667f, -1.333f, -1f, -0.667f, -0.333f, 0f, 0.333f, 0.667f, 1f, 1.333f, 1.667f, 2.333f, 3.333f, 4.667f, 7f, 10.333f, 15f};
        }
        else{

            NumMeasurementsPerScan_v = channels;
            float angularResolution_vertical = (_fov_vertical_end_angle - _fov_vertical_start_angle)/channels;
            scanAngleArray_v = new float[NumMeasurementsPerScan_v];
            for (int i = 0; i < NumMeasurementsPerScan_v; i++)
            {
                scanAngleArray_v[i] = ScanAngleStart_v + i * angularResolution_vertical;
            }
        }

        int numPoints = NumMeasurementsPerScan_h*NumMeasurementsPerScan_v;

        raw_data = new byte[16*numPoints];

        // avg_time = 0;
        // total_counts = 0;
    }

    public byte[] getScanData()
    {

        float startTime = Time.realtimeSinceStartup;

        Transform sensor_transform = laser_sensor_link.transform;

        int raw_data_indx = 0;
        for (int i = 0; i < NumMeasurementsPerScan_h; i++)
        {
            for (int j = 0; j < NumMeasurementsPerScan_v; j++)
            {
                var theta = Mathf.Deg2Rad*scanAngleArray_h[i];
                var psi = Mathf.Deg2Rad*scanAngleArray_v[j];
                var local_dir_vec = new Vector3(Mathf.Cos(psi)*Mathf.Sin(theta), -Mathf.Sin(psi), Mathf.Cos(psi)*Mathf.Cos(theta));
                var directionVector = sensor_transform.rotation * local_dir_vec;

                var measurementStart = RangeMetersMin * directionVector + sensor_transform.position;
                var measurementRay = new Ray(measurementStart, directionVector);
                var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax);

                // Only record measurement if it's within the sensor's operating range
                if (foundValidMeasurement)
                {
                    BitConverter.GetBytes(hit.point.z-sensor_transform.position.z).CopyTo(raw_data, raw_data_indx * 16);
                    BitConverter.GetBytes(-(hit.point.x-sensor_transform.position.x)).CopyTo(raw_data, raw_data_indx * 16+4);
                    BitConverter.GetBytes(hit.point.y-sensor_transform.position.y).CopyTo(raw_data, raw_data_indx * 16+8);
                    BitConverter.GetBytes(0.0f).CopyTo(raw_data, raw_data_indx * 16 + 12);
                }
                else
                {
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16);
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16+4);
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16+8);
                    BitConverter.GetBytes(0.0f).CopyTo(raw_data, raw_data_indx * 16 + 12);

                }
                ++raw_data_indx;
            }
        }
        

        

        // float endTime = Time.realtimeSinceStartup;
        // float elapsedTime = 1000*(endTime - startTime);

        // float total_time = (total_counts*avg_time + elapsedTime);
        // total_counts = total_counts + 1;
        // avg_time = total_time/total_counts;

        // Debug.Log("getScanMsg() Execution Time (curr, avg): (" + elapsedTime.ToString("F4") +", "+ avg_time.ToString("F4")+ ") ms");

        return raw_data;
    }
}
