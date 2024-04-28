using UnityEngine;

public class MoveRobot : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float turnSpeed = 200f;
    public GameObject game_object;

    void Start()
    {
        if (game_object == null)
        {
            Debug.LogError("Game object not assigned to MoveRobot script!");
            return;
        }

        Debug.Log(game_object.transform.position);
    }

    void Update()
    {
        // Get input from arrow keys
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");

        // Calculate movement along each axis
        Vector3 movement = new Vector3(horizontalInput, 0, verticalInput);
        movement = Vector3.ClampMagnitude(movement, 1f); // Limit diagonal movement speed

        // Move the robot forward/backward and up/down
        game_object.transform.Translate(Vector3.forward * movement.z * moveSpeed * Time.deltaTime);
        game_object.transform.Translate(Vector3.up * movement.y * moveSpeed * Time.deltaTime);

        // Rotate the robot left/right
        if (horizontalInput != 0f)
        {
            game_object.transform.Rotate(Vector3.up, horizontalInput * turnSpeed * Time.deltaTime);
        }
    }
}
