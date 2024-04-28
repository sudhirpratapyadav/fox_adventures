using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class Camera_Pub : MonoBehaviour
{
    public Camera sensorCamera;

    int resWidth = 256;
    int resHeight = 256;
    string basePath;
    int imgCount = 0;
    int frameRate = 3; // Desired frame rate

    float timer = 0f;
    float timeBetweenCaptures;

    string datasetPath;

    void Awake()
    {
        if (sensorCamera.targetTexture == null)
        {
            sensorCamera.targetTexture = new RenderTexture(resWidth, resHeight, 24);
        }
        else
        {
            resWidth = sensorCamera.targetTexture.width;
            resHeight = sensorCamera.targetTexture.height;
        }

        
        timeBetweenCaptures = 1f / frameRate; // Calculate time between captures


        // Create subfolder based on the number of existing folders
        string basePath = "/home/cvlab/sudhir/nerf/datasets/simulated/";
        int num_dir = GetNumberOfFolders(basePath);
        datasetPath = $"{basePath}desert{num_dir}/";
        Directory.CreateDirectory(datasetPath);
        Directory.CreateDirectory($"{datasetPath}images/");
    }

    int GetNumberOfFolders(string path)
    {
        // Get the number of existing folders in the specified path
        string[] folders = Directory.GetDirectories(path);
        return folders.Length;
    }

    void Update()
    {
        // Update the timer
        timer += Time.deltaTime;

        // Check if it's time to capture a new image based on the frame rate
        if (timer >= timeBetweenCaptures)
        {
            captureImage();
            timer = 0f; // Reset the timer
        }
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
}
