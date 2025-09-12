using System;
using UnityEngine;
using UnityEngine.InputSystem; // New Input System

public class MultiJointJogger : MonoBehaviour
{
    [Serializable]
    public class JointEntry
    {
        public string label = "joint";
        public ArticulationBody joint;

        [Header("Jog Settings")]
        public float stepDegrees = 5f;       // ← 좌/우 화살표를 한 번 눌렀을 때 변화량
        public float fineStepDegrees = 1f;   // ← Shift 키 누른 상태에서의 미세 조그
        public float speedDegPerSec = 120f;  // ← 목표각도로 수렴하는 속도(부드럽게 가기 위함)
        public bool clampToLimits = true;    // ← xDrive 리밋에 맞춰 Clamp

        [HideInInspector] public float desiredTarget; // 내부 상태: 목표각(도)
    }

    [Header("Joints (drag ArticulationBodies here)")]
    public JointEntry[] joints;

    [Header("UI / Overlay")]
    public bool showOverlay = true;     // OnGUI 슬라이더 오버레이 표시
    public bool showDegrees = true;     // 각도 숫자 표시
    public int selectedIndex = 0;       // 키보드 조그의 대상 관절

    [Header("Hotkeys")]
    public Key selectPrevKey = Key.Comma;   // '<' , 키
    public Key selectNextKey = Key.Period;  // '>' . 키
    public Key jogMinusKey = Key.LeftArrow;
    public Key jogPlusKey = Key.RightArrow;
    public Key resetAllKey = Key.R;         // 모든 관절 target=0으로
    public Key zeroSelectedKey = Key.Z;     // 선택 관절만 0으로

    void Start()
    {
        // 초기 desiredTarget 동기화
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint == null) continue;
            joints[i].desiredTarget = joints[i].joint.xDrive.target;
        }
        selectedIndex = Mathf.Clamp(selectedIndex, 0, Mathf.Max(0, joints.Length - 1));
    }

    void Update()
    {
        var kb = Keyboard.current;
        if (kb == null || joints == null || joints.Length == 0) return;

        // 숫자키 1..9로 빠른 선택
        if (kb.digit1Key.wasPressedThisFrame) selectedIndex = 0;
        if (kb.digit2Key.wasPressedThisFrame) selectedIndex = 1;
        if (kb.digit3Key.wasPressedThisFrame) selectedIndex = 2;
        if (kb.digit4Key.wasPressedThisFrame) selectedIndex = 3;
        if (kb.digit5Key.wasPressedThisFrame) selectedIndex = 4;
        if (kb.digit6Key.wasPressedThisFrame) selectedIndex = 5;
        if (kb.digit7Key.wasPressedThisFrame) selectedIndex = 6;
        if (kb.digit8Key.wasPressedThisFrame) selectedIndex = 7;
        if (kb.digit9Key.wasPressedThisFrame) selectedIndex = 8;

        // 좌우 선택 순환
        if (kb[selectPrevKey].wasPressedThisFrame) selectedIndex = (selectedIndex - 1 + joints.Length) % joints.Length;
        if (kb[selectNextKey].wasPressedThisFrame) selectedIndex = (selectedIndex + 1) % joints.Length;
        selectedIndex = Mathf.Clamp(selectedIndex, 0, joints.Length - 1);

        // 조그 (선택된 관절만)
        if (IsValidIndex(selectedIndex))
        {
            var entry = joints[selectedIndex];
            if (entry.joint != null)
            {
                float step = kb.shiftKey.isPressed ? entry.fineStepDegrees : entry.stepDegrees;
                if (kb[jogMinusKey].wasPressedThisFrame) entry.desiredTarget -= step;
                if (kb[jogPlusKey].wasPressedThisFrame)  entry.desiredTarget += step;
                ApplyDesiredTarget(selectedIndex, entry.desiredTarget, smooth:false); // 즉시 반영(아래에서 부드럽게 수렴)
            }
        }

        // 리셋 핫키
        if (kb[resetAllKey].wasPressedThisFrame)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                if (joints[i].joint == null) continue;
                joints[i].desiredTarget = 0f;
                ApplyDesiredTarget(i, joints[i].desiredTarget, smooth:false);
            }
        }
        if (kb[zeroSelectedKey].wasPressedThisFrame && IsValidIndex(selectedIndex) && joints[selectedIndex].joint != null)
        {
            joints[selectedIndex].desiredTarget = 0f;
            ApplyDesiredTarget(selectedIndex, 0f, smooth:false);
        }

        // 부드러운 수렴(프레임마다 target을 desired로 끌어감)
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint == null) continue;
            var ab = joints[i].joint;
            var d = ab.xDrive;
            float tgt = Mathf.MoveTowards(d.target, joints[i].desiredTarget, joints[i].speedDegPerSec * Time.deltaTime);
            if (joints[i].clampToLimits) tgt = Mathf.Clamp(tgt, d.lowerLimit, d.upperLimit);
            d.target = tgt;
            ab.xDrive = d;
        }
    }

    void ApplyDesiredTarget(int idx, float newTargetDeg, bool smooth)
    {
        if (!IsValidIndex(idx)) return;
        var entry = joints[idx];
        if (entry.joint == null) return;

        var d = entry.joint.xDrive;
        float tgt = newTargetDeg;
        if (entry.clampToLimits) tgt = Mathf.Clamp(tgt, d.lowerLimit, d.upperLimit);
        entry.desiredTarget = tgt;

        if (!smooth)
        {
            d.target = tgt;
            entry.joint.xDrive = d;
        }
    }

    bool IsValidIndex(int i) => i >= 0 && joints != null && i < joints.Length;

    // 간단 오버레이(UI 없이 바로 써먹는 IMGUI)
    void OnGUI()
    {
        if (!showOverlay || joints == null) return;

        const int pad = 8;
        int w = 420;
        int rowH = 26;
        int h = Mathf.Max(40, 30 + joints.Length * rowH + 30);

        var rect = new Rect(pad, pad, w, h);
        GUI.Box(rect, "AR4 Multi-Jogger");

        GUILayout.BeginArea(new Rect(pad + 10, pad + 28, w - 20, h - 40));
        GUILayout.Label("←/→ : 선택된 관절 조그    Shift : 미세조그    1~9 : 관절 선택    R : 전체 0도    Z : 선택 0도");
        for (int i = 0; i < joints.Length; i++)
        {
            var e = joints[i];
            if (e.joint == null)
            {
                GUILayout.Label($"[{i+1}] (빈 슬롯)");
                continue;
            }
            var d = e.joint.xDrive;
            GUILayout.BeginHorizontal();

            // 선택 표시
            string name = string.IsNullOrEmpty(e.label) ? e.joint.name : e.label;
            string tag = (i == selectedIndex) ? "►" : "  ";
            GUILayout.Label($"{tag} [{i+1}] {name}", GUILayout.Width(170));

            // 슬라이더로 직접 제어 (limits 기준)
            float min = d.lowerLimit;
            float max = d.upperLimit;
            float val = d.target;

            float newVal = GUILayout.HorizontalSlider(val, min, max, GUILayout.Width(160));

            if (showDegrees) GUILayout.Label($"{newVal,7:0.0}°", GUILayout.Width(60));

            if (Mathf.Abs(newVal - val) > 0.01f)
            {
                e.desiredTarget = newVal;
                d.target = newVal;
                e.joint.xDrive = d;
            }

            GUILayout.EndHorizontal();
        }
        GUILayout.EndArea();
    }
}