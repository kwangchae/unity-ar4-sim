using UnityEngine;
using UnityEngine.InputSystem;

public class SimpleJointJogger : MonoBehaviour
{
    public ArticulationBody joint;      // link_X의 ArticulationBody 할당
    public float stepDegrees = 5f;      // 한번에 움직일 각도
    public float speedDegPerSec = 90f;  // 목표 각도로 가는 속도

    void Update()
    {
        if (joint == null) return;
        var kb = Keyboard.current;
        if (kb == null) return;

        var drive = joint.xDrive;
        float target = drive.target;

        if (kb.leftArrowKey.wasPressedThisFrame)  target -= stepDegrees;
        if (kb.rightArrowKey.wasPressedThisFrame) target += stepDegrees;

        target = Mathf.Clamp(target, drive.lowerLimit, drive.upperLimit);
        drive.target = Mathf.MoveTowards(drive.target, target, speedDegPerSec * Time.deltaTime);
        joint.xDrive = drive;
    }
}