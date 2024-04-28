using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class getData : MonoBehaviour
{
    public Camera sensorCamera;

    public GameObject robot;


    private List<Pose> trajectory;
    private int currentPoseIndex = 0;


    int resWidth = 256;
    int resHeight = 256;
    string basePath;
    int imgCount = 0;
    string datasetPath;

    void Awake()
    {
        // trajectory = generateCircularTrajectory();
        trajectory = generateStraightTrajectory();

        if (sensorCamera.targetTexture == null)
        {
            sensorCamera.targetTexture = new RenderTexture(resWidth, resHeight, 24);
        }
        else
        {
            resWidth = sensorCamera.targetTexture.width;
            resHeight = sensorCamera.targetTexture.height;
        }

        // Create subfolder based on the number of existing folders
        string basePath = "/home/cvlab/sudhir/nerf/datasets/simulated/";
        int num_dir = GetNumberOfFolders(basePath);
        datasetPath = $"{basePath}desert{num_dir}/";
        Directory.CreateDirectory(datasetPath);
        Directory.CreateDirectory($"{datasetPath}images/");
    }

    List<Pose> generateStraightTrajectory()
    {
        Vector3 initial_pos = new Vector3(0f, 1f, 0f);
        Vector3 final_pos = new Vector3(0f, 2f, 20f); // Change this to the desired final position
        List<Pose> trajectory = new List<Pose>();
        int numPoints = 50;

        // Generate poses along the straight path
        for (int i = 0; i < numPoints; i++)
        {
            // Interpolate position
            float t = i / (float)(numPoints - 1);
            Vector3 translation = Vector3.Lerp(initial_pos, final_pos, t);

            // Calculate the rotation to make the z-axis point towards the final position
            Vector3 lookAtFinal = final_pos - translation;
            Quaternion rotation = Quaternion.LookRotation(lookAtFinal.normalized, Vector3.up);

            Pose pose = new Pose(translation, rotation);
            trajectory.Add(pose);
        }

        return trajectory;
    }

    List<Pose> generateCircularTrajectory()
    {
        Vector3 center = new Vector3(75f, 5f, 30f);
        
        List<Pose> trajectory = new List<Pose>();
        int numPoints = 50; // Change this value to set the number of points in the circle

        float y_r = 15f;
        float radius = 50f;

        // // Generate poses along the circular path
        // for (int i = 0; i < numPoints; i++)
        // {
        //     float angle = i * (360f / numPoints);
        //     float x = center.x + radius * Mathf.Cos(Mathf.Deg2Rad * angle);
        //     float z = center.z + radius * Mathf.Sin(Mathf.Deg2Rad * angle);

        //     Vector3 translation = new Vector3(x, y_r, z);

        //     // Calculate the rotation to make the z-axis point towards the center
        //     Vector3 lookAtCenter = center - translation;
        //     Quaternion rotation = Quaternion.LookRotation(lookAtCenter.normalized, Vector3.up);

        //     Pose pose = new Pose(translation, rotation);
        //     trajectory.Add(pose);
        // }    

        y_r = 12f;
        radius = 70f;

        // Generate poses along the circular path
        for (int i = 0; i < numPoints; i++)
        {
            float angle = i * (360f / numPoints);
            float x = center.x + radius * Mathf.Cos(Mathf.Deg2Rad * angle);
            float z = center.z + radius * Mathf.Sin(Mathf.Deg2Rad * angle);

            Vector3 translation = new Vector3(x, y_r, z);

            // Calculate the rotation to make the z-axis point towards the center
            Vector3 lookAtCenter = center - translation;
            Quaternion rotation = Quaternion.LookRotation(lookAtCenter.normalized, Vector3.up);

            Pose pose = new Pose(translation, rotation);
            trajectory.Add(pose);
        }


        // y_r = 18f;
        // radius = 20f;

        // // Generate poses along the circular path
        // for (int i = 0; i < numPoints; i++)
        // {
        //     float angle = i * (360f / numPoints);
        //     float x = center.x + radius * Mathf.Cos(Mathf.Deg2Rad * angle);
        //     float z = center.z + radius * Mathf.Sin(Mathf.Deg2Rad * angle);

        //     Vector3 translation = new Vector3(x, y_r, z);

        //     // Calculate the rotation to make the z-axis point towards the center
        //     Vector3 lookAtCenter = center - translation;
        //     Quaternion rotation = Quaternion.LookRotation(lookAtCenter.normalized, Vector3.up);

        //     Pose pose = new Pose(translation, rotation);
        //     trajectory.Add(pose);
        // }

        return trajectory;
    }

    int GetNumberOfFolders(string path)
    {
        // Get the number of existing folders in the specified path
        string[] folders = Directory.GetDirectories(path);
        return folders.Length;
    }

    void Update()
    {
        // Check if there are poses in the trajectory
        if (currentPoseIndex < trajectory.Count)
        {
            moveRobot();
            captureImage();
            currentPoseIndex++;
        }
    }

    void moveRobot()
    {
        Pose currentPose = trajectory[currentPoseIndex];
        robot.transform.position = currentPose.translation;
        robot.transform.rotation = currentPose.rotation;
    }

    void captureImage()
    {
        Texture2D texture = new Texture2D(resWidth, resHeight, TextureFormat.RGB24, false);
        sensorCamera.Render();
        RenderTexture.active = sensorCamera.targetTexture;
        texture.ReadPixels(new Rect(0, 0, resWidth, resHeight), 0, 0);

        byte[] bytes = texture.EncodeToPNG();

        string img_path = $"{datasetPath}images/{imgCount}.png";

        // Debug.Log($"Took screenshot and saved transformation matrix {img_path}");

        System.IO.File.WriteAllBytes(img_path, bytes);

        // Save transformation matrix to a text file
        SaveTransformationMatrix($"{datasetPath}transforms.txt");
        

        // Increment the counter
        imgCount++;



        //  // Flip the texture vertically
        // Texture2D flippedTexture = FlipTexture(texture, false, true);

        // byte[] bytes = flippedTexture.EncodeToPNG();

        // string img_path = $"{datasetPath}images/{imgCount}.png";

        // System.IO.File.WriteAllBytes(img_path, bytes);

        // // Save transformation matrix to a text file
        // SaveTransformationMatrix($"{datasetPath}transforms.txt");

        // // Increment the counter
        // imgCount++;
    }

    Texture2D FlipTexture(Texture2D original, bool flipX, bool flipY)
    {
        int width = original.width;
        int height = original.height;
        Texture2D flipped = new Texture2D(width, height);

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                int newX = flipX ? width - 1 - x : x;
                int newY = flipY ? height - 1 - y : y;

                flipped.SetPixel(newX, newY, original.GetPixel(x, y));
            }
        }

        flipped.Apply();

        return flipped;
    }

    void SaveTransformationMatrix(string filePath)
    {
        // Get the transformation matrix
        Matrix4x4 matrix = sensorCamera.transform.localToWorldMatrix;

        // Flatten the matrix and convert to a string with up to 3 decimal points
        string matrixString = "";
        for (int i = 0; i < 16; i++)
        {
            matrixString += matrix[i].ToString("F3");
            if (i < 15) matrixString += ",";
        }

        // Append the matrix string to the text file
        File.AppendAllText(filePath, matrixString + "\n");
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
