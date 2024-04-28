using UnityEngine;
using UnityEngine.Serialization;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class EnvStateSub : MonoBehaviour
{

    ROSConnection ros;

    public GameObject env;

    private GameObject[] food;
    private GameObject[] obstacles;
    private GameObject[] fires;
    private GameObject[] lakes;
    private GameObject[] lions;
    private GameObject fox;
    // Message to store agent position
    // PoseStampedMsg agentPoseMsg;

    // Message to store obstacle positions
    // Float32MultiArrayMsg obstaclePositionsMsg;

    // private init position
    private Vector3 init_pos;


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<Float32MultiArrayMsg>("/obstacle_positions", ObstaclePositionsCallback);
        ros.Subscribe<Float32MultiArrayMsg>("/food_positions", FoodPositionsCallback);
        ros.Subscribe<Float32MultiArrayMsg>("/agent_positions", AgentPositionsCallback);
        ros.Subscribe<Float32MultiArrayMsg>("/lake_positions", LakePositionsCallback);
        ros.Subscribe<Float32MultiArrayMsg>("/fire_positions", FirePositionsCallback);

        // Initialize the position to 0,0,0
        init_pos = new Vector3(0, -20, 0);

        // Initialize obstacle array message
        InitializeEnv();

        // Start publishing at desired rates
        // InvokeRepeating("PublishAgentPosition", 0f, 0.1f); // 10Hz
        // InvokeRepeating("PublishObstaclePositions", 0f, 1f); // 1Hz
    }

    void InitializeEnv()
    {
        // Iterate through each child of the environment object
        foreach (Transform child in env.transform)
        {
            // Process each child based on its name
            switch (child.name)
            {
                case "Obstacles":
                    obstacles = PopulateArray(child);
                    // randomize the sequence of obstacles
                    for (int i = 0; i < obstacles.Length; i++)
                    {
                        int randomIndex = Random.Range(i, obstacles.Length);
                        GameObject temp = obstacles[i];
                        obstacles[i] = obstacles[randomIndex];
                        obstacles[randomIndex] = temp;
                    }
                    break;
                case "Food":
                    food = PopulateArray(child);
                    break;
                case "Fires":
                    fires = PopulateArray(child);
                    break;
                case "Lakes":
                    lakes = PopulateArray(child);
                    break;
                case "Lions":
                    lions = PopulateArray(child);
                    break;
                case "Fox":
                    fox = child.gameObject;
                    fox.transform.position = init_pos;
                    break;
                default:
                    // Handle other objects if needed
                    break;
            }
        }
    }

    // Populate an array with child game objects and set their initial positions
    GameObject[] PopulateArray(Transform parent)
    {
        GameObject[] array = new GameObject[parent.childCount];
        for (int i = 0; i < parent.childCount; i++)
        {
            
            GameObject childObject = parent.GetChild(i).gameObject;
            array[i] = childObject;
            childObject.transform.position = init_pos;
        }
        return array;
    }

    // Callback function to receive obstacle positions
    void ObstaclePositionsCallback(Float32MultiArrayMsg msg)
    {
        int j = 0;
        for (int i = 0; i < msg.data.Length; i += 3)
        {
            obstacles[j].transform.position = new Vector3(msg.data[i], msg.data[i + 1], msg.data[i + 2]);
            j++;
        }
    }

    // Callback function to receive food positions
    void FoodPositionsCallback(Float32MultiArrayMsg msg)
    {
        int j = 0;
        for (int i = 0; i < msg.data.Length; i += 3)
        {
            food[j].transform.position = new Vector3(msg.data[i], msg.data[i + 1], msg.data[i + 2]);
            j++;
        }

        //set remaining food positions to init_pos
        for (int k = j; k < food.Length; k++)
        {
            food[k].transform.position = init_pos;
        }
    }

    // Callback function to receive agent positions
    void AgentPositionsCallback(Float32MultiArrayMsg msg)
    {

        // Assign first 3 values to fox position
        fox.transform.position = new Vector3(msg.data[0], msg.data[1], msg.data[2]);

        // Assign remaining values to lion positions
        int j = 0;
        for (int i = 3; i < msg.data.Length; i += 3)
        {
            lions[j].transform.position = new Vector3(msg.data[i], msg.data[i + 1], msg.data[i + 2]);
            j++;
        }

        //set remaining lion positions to init_pos
        for (int k = j; k < lions.Length; k++)
        {
            lions[k].transform.position = init_pos;
        }
    }

    // Callback function to receive lake positions
    void LakePositionsCallback(Float32MultiArrayMsg msg)
    {
        int j = 0;
        for (int i = 0; i < msg.data.Length; i += 3)
        {
            lakes[j].transform.position = new Vector3(msg.data[i], msg.data[i + 1], msg.data[i + 2]);
            j++;
        }
    }

    void FirePositionsCallback(Float32MultiArrayMsg msg)
    {
        int j = 0;
        for (int i = 0; i < msg.data.Length; i += 3)
        {
            fires[j].transform.position = new Vector3(msg.data[i], msg.data[i + 1], msg.data[i + 2]);
            j++;
        }

        //set remaining fire positions to init_pos
        for (int k = j; k < fires.Length; k++)
        {
            fires[k].transform.position = init_pos;
        }
    }
}
