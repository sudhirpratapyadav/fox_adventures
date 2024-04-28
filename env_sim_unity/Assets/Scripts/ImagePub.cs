    using UnityEngine;
    using UnityEngine.Serialization;
    using System.Collections.Generic;

    using RosMessageTypes.Geometry;
    using RosMessageTypes.Sensor;
    using RosMessageTypes.Std;
    using RosMessageTypes.BuiltinInterfaces;

    using Unity.Robotics.Core;
    using Unity.Robotics.ROSTCPConnector;

    using System.IO;



    public class ImagePublisher : MonoBehaviour
    {
        

        public string image_topic = "/image";
        // public string camera_info_topic = "/camera_info";
        public Camera sensorCamera;

        ROSConnection ros;

        private RenderTexture renderTexture;
        uint width;
        uint height;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<ImageMsg>(image_topic);

            Debug.Log(sensorCamera);
            Debug.Log(sensorCamera.pixelWidth);

            // Destroy existing renderTexture if it exists
            if (renderTexture != null)
            {
                renderTexture.Release();
                Destroy(renderTexture);
            }

            // Create a new RenderTexture
            renderTexture = new RenderTexture(sensorCamera.pixelWidth, sensorCamera.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
            renderTexture.Create();

            width = (uint)renderTexture.width;
            height = (uint)renderTexture.height;

        }

        void Update()
        {

            sensorCamera.targetTexture = renderTexture;
            RenderTexture currentRT = RenderTexture.active;
            RenderTexture.active = renderTexture;
            sensorCamera.Render();
            Texture2D mainCameraTexture = new Texture2D(renderTexture.width, renderTexture.height);
            mainCameraTexture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
            mainCameraTexture.Apply();
            RenderTexture.active = currentRT;
            // Get the raw byte info from the screenshot
            byte[] imageBytes = mainCameraTexture.GetRawTextureData();
            sensorCamera.targetTexture = null;

            // Debug.Log(Camera.main);


            // var oldRT = RenderTexture.active;
            // RenderTexture.active = sensorCamera.targetTexture;
            // sensorCamera.Render();

            // Debug.Log(sensorCamera);
            // Debug.Log(sensorCamera.targetTexture.width);
            
            // // Copy the pixels from the GPU into a texture so we can work with them
            // // For more efficiency you should reuse this texture, instead of creating a new one every time
            // Texture2D camText = new Texture2D(sensorCamera.targetTexture.width, sensorCamera.targetTexture.height);
            // camText.ReadPixels(new Rect(0, 0, sensorCamera.targetTexture.width, sensorCamera.targetTexture.height), 0, 0);
            // camText.Apply();
            // RenderTexture.active = oldRT;
            
            // Encode the texture as a PNG, and send to ROS
            // byte[] imageBytes = camText.EncodeToPNG();

            int bytesPerPixel = 4; // Each pixel is represented by 4 bytes (R, G, B, A)
            uint step = width * (uint)bytesPerPixel;

            int numChannels = 4;  // RGBA format
            int bitDepth = 8;     // 8 bits per channel

            // Calculate the estimated step size in bytes
            uint estimatedStep = (uint)(width * numChannels * bitDepth / 8);

            // Log the calculated and estimated step size
            Debug.Log("Calculated Step: " + step);
            Debug.Log("Estimated Step: " + estimatedStep);


            // Calculate the size using step * height
            uint calculatedSize = step * height;

            // Debug.Log("width: " + width);

            // Debug.Log("height: " + height);

            // // Log the calculated size
            // Debug.Log("Calculated Size: " + calculatedSize);

            // // Log the directly measured size
            // Debug.Log("Measured Size: " + imageBytes.Length);

            

             // Create a Texture2D and load the raw byte data
            Texture2D texture = new Texture2D(renderTexture.width, renderTexture.height);
            texture.LoadRawTextureData(imageBytes);
            texture.Apply();

            // Encode the texture as PNG
            byte[] pngBytes = texture.EncodeToPNG();

            Debug.Log("width: " + width);
            Debug.Log("height: " + height);
            Debug.Log("Calculated Size: " + calculatedSize);
            Debug.Log("Measured img: " + imageBytes.Length);
            Debug.Log("Measured Size: " + pngBytes.Length);

            SaveAsPNG(pngBytes, "/home/cvlab/saved_data/output.png");

            var message = new ImageMsg(new HeaderMsg(), height, width, "rgba8", 0, step, pngBytes);
            ros.Publish(image_topic, message);
        }

        void SaveAsPNG(byte[] bytes, string filePath)
        {
            // Create a file stream and write the bytes to the file
            using (FileStream fs = new FileStream(filePath, FileMode.Create))
            {
                // Create a BinaryWriter and write the bytes to the file
                using (BinaryWriter bw = new BinaryWriter(fs))
                {
                    bw.Write(bytes);
                }
            }

            Debug.Log("Image saved as: " + filePath);
        }
    }

