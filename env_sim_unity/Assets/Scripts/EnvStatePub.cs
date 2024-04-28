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

public class EnvStatePub : MonoBehaviour
{
    public string agent_position_topic = "/agent_position";
    public string obstacle_positions_topic = "/obstacle_positions";
    public string food_positions_topic = "/food_positions"; // New topic for food positions


    ROSConnection ros;

    public GameObject agent;
    public GameObject env;

    public GameObject[] food;

    // Message to store agent position
    PoseStampedMsg agentPoseMsg;

    // Message to store obstacle positions
    Float32MultiArrayMsg obstaclePositionsMsg;


void Start()
{
    ros = ROSConnection.GetOrCreateInstance();
    ros.RegisterPublisher<PoseStampedMsg>(agent_position_topic);
    ros.RegisterPublisher<Float32MultiArrayMsg>(obstacle_positions_topic);

    ros.Subscribe<Float32MultiArrayMsg>(food_positions_topic, FoodPositionsCallback);

    // Initialize obstacle array message
    InitializeEnvPositions();

    // Start publishing at desired rates
    InvokeRepeating("PublishAgentPosition", 0f, 0.1f); // 10Hz
    InvokeRepeating("PublishObstaclePositions", 0f, 1f); // 1Hz
}

    // Method to initialize obstacle array message
    void InitializeEnvPositions()
    {

        obstaclePositionsMsg = new Float32MultiArrayMsg();
        // Update obstacle positions message
        List<float> positions = new List<float>();
        Transform[] envChildren = env.GetComponentsInChildren<Transform>();
        foreach (Transform child in envChildren)
        {
            if (child.name == "Obstacles")
            {
                Transform[] obstacleChildren = child.GetComponentsInChildren<Transform>();
                foreach (Transform obstacleChild in obstacleChildren)
                {
                    if (obstacleChild == child)
                        continue;
                    Vector3 obstaclePosition = obstacleChild.position;
                    positions.Add(obstaclePosition.x);
                    positions.Add(obstaclePosition.y);
                    positions.Add(obstaclePosition.z);
                }
            }
            if (child.name == "Food")
            {
                // Get all chidren of the food object and add them to the food array
                food = new GameObject[child.childCount];
                for (int i = 0; i < child.childCount; i++)
                {
                    food[i] = child.GetChild(i).gameObject;

                    // Set postion of each food object to 0,0,0
                    food[i].transform.position = new Vector3(0, 5, 0);
                }
            }
        }

        // Store positions in the message
        obstaclePositionsMsg.data = positions.ToArray();
    }

    // Method to publish agent position
    void PublishAgentPosition()
    {
        var timestamp = new TimeStamp(Clock.time);

        agentPoseMsg = new PoseStampedMsg();

        // Update agent position message
        agentPoseMsg.header = new HeaderMsg
        {
            frame_id = agent.name,
            stamp = new TimeMsg
            {
                sec = timestamp.Seconds,
                nanosec = timestamp.NanoSeconds,
            }
        };
        agentPoseMsg.pose = new PoseMsg
        {
            position = agent.transform.position.To<FLU>(),
            orientation = agent.transform.rotation.To<FLU>(),
        };

        ros.Publish(agent_position_topic, agentPoseMsg);
    }

    // Method to publish obstacle positions
    void PublishObstaclePositions()
    {
        ros.Publish(obstacle_positions_topic, obstaclePositionsMsg);
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
    }
}
