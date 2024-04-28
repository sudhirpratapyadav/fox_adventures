using UnityEngine;
using System.Collections.Generic;

public class StraightTraj : MonoBehaviour
{

    public GameObject robot;

    private List<Pose> trajectory;
    private int currentPoseIndex = 0;

    void Start()
    {
        Debug.Log(robot.transform.position);

        // Generate a straight trajectory in the Z direction
        trajectory = generateCircularTrajectory();
    }

    void Update()
    {
        // Check if there are poses in the trajectory
        if (currentPoseIndex < trajectory.Count)
        {
            // Get the current pose
            Pose currentPose = trajectory[currentPoseIndex];

            // Set the robot's position directly
            robot.transform.position = currentPose.translation;

            // Set the robot's rotation directly
            robot.transform.rotation = currentPose.rotation;

            // Move to the next pose in the trajectory
            currentPoseIndex++;
        }
    }

        // Function to generate a circular trajectory
    List<Pose> generateCircularTrajectory()
    {
        Vector3 center = new Vector3(100f, 5f, 0f);
        float radius = 100f;
        List<Pose> trajectory = new List<Pose>();
        int numPoints = 360; // Change this value to set the number of points in the circle

        // Generate poses along the circular path
        for (int i = 0; i < numPoints; i++)
        {
            float angle = i * (360f / numPoints);
            float x = center.x + radius * Mathf.Cos(Mathf.Deg2Rad * angle);
            float z = center.z + radius * Mathf.Sin(Mathf.Deg2Rad * angle);

            Vector3 translation = new Vector3(x, center.y, z);

            // Calculate the rotation to make the z-axis point towards the center
            Vector3 lookAtCenter = center - translation;
            Quaternion rotation = Quaternion.LookRotation(lookAtCenter.normalized, Vector3.up);

            Pose pose = new Pose(translation, rotation);
            trajectory.Add(pose);
        }

        return trajectory;
    }

        // Function to generate a circular trajectory
    // List<Pose> generateCircularTrajectory()
    // {
    //     float radius = 100f;
    //     List<Pose> trajectory = new List<Pose>();
    //     int numPoints = 360; // Change this value to set the number of points in the circle

    //     // Generate poses along the circular path
    //     for (int i = 0; i < numPoints; i++)
    //     {
    //         float angle = i * (360f / numPoints);
    //         float x = radius * Mathf.Cos(Mathf.Deg2Rad * angle);
    //         float z = radius * Mathf.Sin(Mathf.Deg2Rad * angle);

    //         Vector3 translation = new Vector3(x, 0f, z);
    //         Quaternion rotation = Quaternion.Euler(0f, angle, 0f);

    //         Pose pose = new Pose(translation, rotation);
    //         trajectory.Add(pose);
    //     }

    //     return trajectory;
    // }

    // Function to generate a straight trajectory in the Z direction
    List<Pose> generateTrajectory()
    {
        List<Pose> trajectory = new List<Pose>();
        float distance = 10f; // Change this value to set the distance of the trajectory

        // Generate poses along the Z-axis
        for (float z = 0f; z <= distance; z += 1f)
        {
            Vector3 translation = new Vector3(0f, 0f, z);
            Quaternion rotation = Quaternion.Euler(0f, 0f, 0f);

            Pose pose = new Pose(translation, rotation);
            trajectory.Add(pose);
        }

        return trajectory;
    }

    // Structure to represent a pose containing translation and rotation
    struct Pose
    {
        public Vector3 translation;
        public Quaternion rotation;

        public Pose(Vector3 translation, Quaternion rotation)
        {
            this.translation = translation;
            this.rotation = rotation;
        }
    }
}
