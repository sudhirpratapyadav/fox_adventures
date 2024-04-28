using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FoodSim : MonoBehaviour
{
    public GameObject prefabToSpawn;
    public Transform spawnPoint;

    void Start()
    {
        // Call a function to spawn objects, or you can trigger this from an event or user input
        SpawnObject();
    }

    void SpawnObject()
    {
        // Instantiate the prefab at the specified spawn point
        GameObject newObject = Instantiate(prefabToSpawn, spawnPoint.position, spawnPoint.rotation);

        // Optionally, you can do further customization of the new object here
    }
}

