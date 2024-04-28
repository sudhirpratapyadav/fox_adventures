using UnityEngine;

public class Example : MonoBehaviour
{

    public float RangeMetersMin = 0;
    public float RangeMetersMax = 1000;

    void Start()
    {

        Transform sensor_transform = transform;

        Debug.Log("Sensor Transform: " + sensor_transform.position + " " + sensor_transform.rotation);

        var scan_angle_h = -180; // Azimuthal Angle
        var scan_angle_v = -45;
        var psi = Mathf.Deg2Rad*scan_angle_v;
        var theta = Mathf.Deg2Rad*scan_angle_h;

        var yawBaseDegrees = sensor_transform.rotation.eulerAngles.y; // yaw of object this script is attached to i.e. laser_scanner
        var pitchBaseDegrees = sensor_transform.rotation.eulerAngles.x; // pitch of object this script is attached to i.e. laser_scanner
        var rollBaseDegrees = sensor_transform.rotation.eulerAngles.z; // roll of object this script is attached to i.e. laser_scanner

        var yawDegrees = scan_angle_h + yawBaseDegrees;
        var pitchDegrees =  pitchBaseDegrees + scan_angle_v;
        var directionVector_1 = Quaternion.Euler(pitchDegrees, yawDegrees, 0f) * Vector3.forward;

        var local_dir_vec = new Vector3(Mathf.Cos(psi)*Mathf.Sin(theta), -Mathf.Sin(psi), Mathf.Cos(psi)*Mathf.Cos(theta));
        var directionVector = sensor_transform.rotation * local_dir_vec;


        Debug.Log("Scan Angles (H, V): (" + scan_angle_h+", "+scan_angle_v+")");
        Debug.Log("sensor_transform (roll, pitch, yaw): ("+rollBaseDegrees+", "+pitchBaseDegrees+", "+yawBaseDegrees+")"); 
        Debug.Log("sensor_transform (x, y, z): ("+pitchBaseDegrees+", "+yawBaseDegrees+", "+rollBaseDegrees+")");
        Debug.Log("sensor_transform (quaternion): "+sensor_transform.rotation);
        Debug.Log("Vector3.forward: "+Vector3.forward);
        Debug.Log("directionVector_1: "+directionVector_1);
        Debug.Log("---");
        Debug.Log("local_dir_vec: "+local_dir_vec);
        Debug.Log("directionVector: "+directionVector);
        Debug.Log("-----------\n");
        

        // var measurementStart = RangeMetersMin * directionVector + sensor_transform.position;

        // var measurementRay = new Ray(measurementStart, directionVector);
        // var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax);
        // // Only record measurement if it's within the sensor's operating range
        // if (foundValidMeasurement)
        // {
        //     BitConverter.GetBytes(hit.point.z-sensor_transform.position.z).CopyTo(raw_data, raw_data_indx * 16);
        //     BitConverter.GetBytes(-(hit.point.x-sensor_transform.position.x)).CopyTo(raw_data, raw_data_indx * 16+4);
        //     BitConverter.GetBytes(hit.point.y-sensor_transform.position.y).CopyTo(raw_data, raw_data_indx * 16+8);
        //     BitConverter.GetBytes(0.0f).CopyTo(raw_data, raw_data_indx * 16 + 12);
        // }
        // else
        // {
        //     BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16);
        //     BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16+4);
        //     BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16+8);
        //     BitConverter.GetBytes(0.0f).CopyTo(raw_data, raw_data_indx * 16 + 12);

        // }
        // Even if Raycast didn't find a valid hit, we still count it as a measurement

    }
}