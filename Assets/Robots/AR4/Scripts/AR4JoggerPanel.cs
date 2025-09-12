using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class AR4JoggerPanel : MonoBehaviour
{
    [Serializable]
    public class JointUI
    {
        public string label;
        public ArticulationBody joint;
        [NonSerialized] public Slider slider; // UI Toolkit Slider
        [NonSerialized] public Label valueLabel;
        [NonSerialized] public bool isDragging; // ← 드래그 중이면 true (슬라이더 값 동기화 방지)
    }

    [Header("Joints (link_1 ~ link_6 ArticulationBody)")]
    public JointUI[] joints;

    [Header("Motion")]
    [Tooltip("target가 desired로 이동하는 속도 (deg/sec)")]
    public float speedDegPerSec = 180f;
    public bool clampToLimits = true;

    [Header("Home Pose (deg)")]
    [Tooltip("비워두면 0도 포즈가 홈 포즈로 사용됩니다. 관절 수와 동일 길이 권장")]
    public float[] homePoseDegrees;

    // UI refs
    UIDocument _doc;
    VisualElement _root;
    ScrollView _list;
    TextField _poseNameField;
    DropdownField _poseDropdown;
    Button _btnSave, _btnLoad, _btnHome, _btnSetHomeFromCurrent, _btnRefresh;
    Toggle _clampToggle;

    // runtime state
    float[] _desired; // per-joint desired angle (deg)
    
    // Public accessors for ROS2 integration
    public float[] DesiredAngles => _desired;
    public void SetDesiredAngle(int jointIndex, float angleDeg)
    {
        if (jointIndex >= 0 && jointIndex < _desired.Length)
        {
            _desired[jointIndex] = angleDeg;
        }
    }

    // pose db
    [Serializable] class Pose { public string name; public float[] degrees; }
    [Serializable] class PoseDB { public List<Pose> poses = new List<Pose>(); }
    PoseDB _db = new PoseDB();
    string DbPath => Path.Combine(Application.persistentDataPath, "ar4_poses.json");

    void Awake()
    {
        _doc = GetComponent<UIDocument>();
        if (_doc == null) _doc = gameObject.AddComponent<UIDocument>();
        _root = _doc.rootVisualElement;

        // 패널을 화면 구석의 작은 창으로: 절대 위치 + 고정 폭
        _root.style.position = Position.Absolute;
        _root.style.left = 16;
        _root.style.top = 16;
        _root.style.width = 360;
        _root.style.maxHeight = 600; // 필요시 조절
        _root.style.overflow = Overflow.Visible;

        _list = _root.Q<ScrollView>("joint-list");

        // 헤더를 드래그 핸들로 만들어 창을 끌어서 이동
        var header = _root.Q<VisualElement>(className: "header");
        if (header != null)
        {
            Vector2 dragOffset = Vector2.zero;
            header.RegisterCallback<PointerDownEvent>(evt =>
            {
                dragOffset = evt.localPosition;
                header.CapturePointer(evt.pointerId);
            });
            header.RegisterCallback<PointerMoveEvent>(evt =>
            {
                if (header.HasPointerCapture(evt.pointerId))
                {
                    Vector2 p = (Vector2)evt.position - dragOffset; // 패널 좌표계 기준
                    _root.style.left = p.x;
                    _root.style.top = p.y;
                }
            });
            header.RegisterCallback<PointerUpEvent>(evt => header.ReleasePointer(evt.pointerId));
        }
        _poseNameField = _root.Q<TextField>("pose-name");
        _poseDropdown = _root.Q<DropdownField>("pose-dropdown");
        _btnSave = _root.Q<Button>("btn-save");
        _btnLoad = _root.Q<Button>("btn-load");
        _btnHome = _root.Q<Button>("btn-home");
        _btnSetHomeFromCurrent = _root.Q<Button>("btn-home-from-current");
        _btnRefresh = _root.Q<Button>("btn-refresh");
        _clampToggle = _root.Q<Toggle>("toggle-clamp");

        if (_clampToggle != null)
        {
            _clampToggle.value = clampToLimits;
            _clampToggle.RegisterValueChangedCallback(evt => clampToLimits = evt.newValue);
        }

        _desired = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            var ab = joints[i].joint;
            _desired[i] = ab != null ? ab.xDrive.target : 0f;
        }

        LoadDB();
        BuildJointRows();
        WireButtons();
        RefreshDropdown();
    }

    void Update()
    {
        // Smoothly move joint.xDrive.target → desired
        for (int i = 0; i < joints.Length; i++)
        {
            var ab = joints[i].joint;
            if (ab == null) continue;
            var d = ab.xDrive;
            float tgt = Mathf.MoveTowards(d.target, _desired[i], speedDegPerSec * Time.deltaTime);
            if (clampToLimits) tgt = Mathf.Clamp(tgt, d.lowerLimit, d.upperLimit);
            d.target = tgt;
            ab.xDrive = d;
            if (joints[i].valueLabel != null)
                joints[i].valueLabel.text = $"{d.target:0.0}°";
            if (joints[i].slider != null && !joints[i].isDragging)
                joints[i].slider.SetValueWithoutNotify(d.target);
        }
    }

    void BuildJointRows()
    {
        if (_list == null) return;
        _list.Clear();
        for (int i = 0; i < joints.Length; i++)
        {
            var row = new VisualElement();
            row.AddToClassList("row");

            string label = string.IsNullOrEmpty(joints[i].label) ? (joints[i].joint ? joints[i].joint.name : $"joint{i+1}") : joints[i].label;
            var nameLabel = new Label($"[{i+1}] {label}");
            nameLabel.AddToClassList("name");

            var slider = new Slider(0, 1) { name = $"slider-{i}", showInputField = false }; // min/max는 나중에 설정
            slider.AddToClassList("slider");

            var valueLabel = new Label("0.0°");
            valueLabel.AddToClassList("val");

            row.Add(nameLabel);
            row.Add(slider);
            row.Add(valueLabel);
            _list.Add(row);

            joints[i].slider = slider;
            joints[i].valueLabel = valueLabel;

            var ab = joints[i].joint;
            float min = -180, max = 180, cur = 0;
            if (ab != null)
            {
                var d = ab.xDrive;
                min = d.lowerLimit;
                max = d.upperLimit;
                cur = d.target;
            }
            slider.lowValue = min;
            slider.highValue = max;
            slider.SetValueWithoutNotify(cur);
            valueLabel.text = $"{cur:0.0}°";

            int idx = i; // capture
            slider.RegisterValueChangedCallback(evt =>
            {
                _desired[idx] = evt.newValue;
            });
            // 드래그/포커스 추적: 사용자가 조작 중일 때는 코드가 값 반영을 덮어쓰지 않도록 함
            slider.RegisterCallback<PointerDownEvent>(e => joints[idx].isDragging = true);
            slider.RegisterCallback<PointerUpEvent>(e => joints[idx].isDragging = false);
            slider.RegisterCallback<MouseLeaveEvent>(e => joints[idx].isDragging = false);
            slider.RegisterCallback<BlurEvent>(e => joints[idx].isDragging = false);
        }
    }

    void WireButtons()
    {
        if (_btnSave != null) _btnSave.clicked += () => SaveCurrentAsPose(_poseNameField?.value);
        if (_btnLoad != null) _btnLoad.clicked += () =>
        {
            if (_poseDropdown == null) return;
            ApplyPoseByName(_poseDropdown.value);
        };
        if (_btnHome != null) _btnHome.clicked += ApplyHomePose;
        if (_btnSetHomeFromCurrent != null) _btnSetHomeFromCurrent.clicked += CaptureCurrentAsHome;
        if (_btnRefresh != null) _btnRefresh.clicked += RefreshDropdown;
    }

    // ---- Pose DB ----
    void LoadDB()
    {
        try
        {
            if (File.Exists(DbPath))
            {
                var json = File.ReadAllText(DbPath);
                _db = JsonUtility.FromJson<PoseDB>(json) ?? new PoseDB();
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"Pose DB load failed: {e.Message}");
            _db = new PoseDB();
        }
    }

    void SaveDB()
    {
        try
        {
            var json = JsonUtility.ToJson(_db, true);
            File.WriteAllText(DbPath, json);
#if UNITY_EDITOR
            Debug.Log($"Saved poses → {DbPath}");
#endif
        }
        catch (Exception e)
        {
            Debug.LogWarning($"Pose DB save failed: {e.Message}");
        }
    }

    void RefreshDropdown()
    {
        if (_poseDropdown == null) return;
        var names = _db.poses.Select(p => p.name).Distinct().OrderBy(n => n).ToList();
        if (names.Count == 0) names.Add("(없음)");
        _poseDropdown.choices = names;
        _poseDropdown.value = names[0];
    }

    float[] CaptureCurrent()
    {
        var arr = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            var ab = joints[i].joint;
            arr[i] = (ab != null) ? ab.xDrive.target : 0f;
        }
        return arr;
    }

    void SaveCurrentAsPose(string name)
    {
        name = string.IsNullOrWhiteSpace(name) ? $"pose_{DateTime.Now:HHmmss}" : name.Trim();
        var cur = CaptureCurrent();
        var existing = _db.poses.FirstOrDefault(p => p.name == name);
        if (existing != null) existing.degrees = cur; else _db.poses.Add(new Pose{ name = name, degrees = cur });
        SaveDB();
        RefreshDropdown();
    }

    void ApplyPoseByName(string name)
    {
        if (string.IsNullOrEmpty(name) || name == "(없음)") return;
        var p = _db.poses.FirstOrDefault(x => x.name == name);
        if (p == null) return;
        ApplyPose(p.degrees);
    }

    void ApplyPose(float[] deg)
    {
        if (deg == null) return;
        int n = Mathf.Min(deg.Length, joints.Length);
        for (int i = 0; i < n; i++)
        {
            var ab = joints[i].joint;
            if (ab == null) continue;
            var d = ab.xDrive;
            float tgt = deg[i];
            if (clampToLimits) tgt = Mathf.Clamp(tgt, d.lowerLimit, d.upperLimit);
            _desired[i] = tgt;
            d.target = tgt; // 즉시 반영
            ab.xDrive = d;
        }
    }

    void ApplyHomePose()
    {
        if (homePoseDegrees == null || homePoseDegrees.Length == 0)
        {
            // all zero
            ApplyPose(new float[joints.Length]);
        }
        else
        {
            ApplyPose(homePoseDegrees);
        }
    }

    void CaptureCurrentAsHome()
    {
        homePoseDegrees = CaptureCurrent();
#if UNITY_EDITOR
        Debug.Log("Home pose updated from current.");
#endif
    }
}