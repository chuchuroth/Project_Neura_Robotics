# MiPA-X Minimum Viable Software Architecture

**Version**: 1.0 (MVP)
**Date**: 2026-01-14
**Purpose**: Define the minimum software structure for a safe, functional service robot

---

## Executive Summary

This document defines the **Minimum Viable Software Architecture (MVSA)** for MiPA-X - a service robot that can **see, walk, touch, feel, hear, talk, navigate, think, and understand** its environment. The architecture centralizes all perception on the **NVIDIA Jetson Orin** modules while maintaining safety as the top priority.

**Design Philosophy**:
- **Safety First**: Robot must never harm humans or property
- **Graceful Degradation**: Partial failures should not cause dangerous behavior
- **Lightweight AI**: Functional but imperfect - designed for future improvements
- **Modular**: Easy to upgrade individual components without system-wide changes

---

## 1. System Overview

### 1.1 Capability Matrix

| Capability | Minimum Requirement | Implementation |
|------------|---------------------|----------------|
| **SEE** | Detect obstacles, people, objects | RGBD cameras + LiDAR on Orin |
| **WALK** | Navigate safely in indoor spaces | Nav2 + SLAM on IPC |
| **TOUCH** | Pick and place simple objects | Dual-arm with basic grasping |
| **FEEL** | Detect collisions, monitor forces | Ultrasonic + torque feedback |
| **HEAR** | Recognize wake words, basic commands | Microphone array on Orin |
| **TALK** | Text-to-speech responses | TTS engine on Orin |
| **NAVIGATE** | A→B autonomous navigation | SLAM + path planning |
| **THINK** | Simple decision making | State machine + lightweight LLM |
| **UNDERSTAND** | Basic scene interpretation | Vision-Language Model (VLM) |

### 1.2 Hardware-Software Mapping

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        NVIDIA JETSON ORIN (×2)                          │
│                    "Centralized Perception Brain"                        │
│                                                                          │
│  ┌────────────────────────────────────────────────────────────────────┐│
│  │                     ORIN #1: PERCEPTION                            ││
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ││
│  │  │   Vision    │ │   Audio     │ │  Sensor     │ │   Scene     │ ││
│  │  │  Pipeline   │ │  Pipeline   │ │  Fusion     │ │Understanding│ ││
│  │  │ - Cameras   │ │ - Mic Array │ │ - LiDAR     │ │ - VLM       │ ││
│  │  │ - Detection │ │ - ASR       │ │ - IMU       │ │ - Objects   │ ││
│  │  │ - Tracking  │ │ - Keywords  │ │ - Odometry  │ │ - Humans    │ ││
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ ││
│  └────────────────────────────────────────────────────────────────────┘│
│                                                                          │
│  ┌────────────────────────────────────────────────────────────────────┐│
│  │                     ORIN #2: COGNITION                             ││
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ││
│  │  │ Task        │ │ Lightweight │ │   Speech    │ │  Behavior   │ ││
│  │  │ Planning    │ │    LLM      │ │   Output    │ │   States    │ ││
│  │  │ - Goals     │ │ - Reasoning │ │ - TTS       │ │ - FSM       │ ││
│  │  │ - Actions   │ │ - Dialog    │ │ - Responses │ │ - Safety    │ ││
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ ││
│  └────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                            1Gbps Ethernet
                                    │
                                    v
┌─────────────────────────────────────────────────────────────────────────┐
│                         CHASSIS IPC (ROS 2)                              │
│                       "Navigation & Safety"                              │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐       │
│  │    Nav2     │ │    SLAM     │ │   Safety    │ │   Motion    │       │
│  │ Navigation  │ │   Mapping   │ │  Monitor    │ │  Control    │       │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘       │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                              CAN Bus 1Mbps
                                    │
                                    v
┌─────────────────────────────────────────────────────────────────────────┐
│                    VERDIN iMX8M+ & STM32H7                               │
│                     "Real-Time Control"                                  │
│  ┌─────────────────────────┐     ┌─────────────────────────┐           │
│  │   Arm Control (14 DOF)  │     │   Base Motors + E-Stop  │           │
│  └─────────────────────────┘     └─────────────────────────┘           │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Software Layer Architecture

### 2.1 Layer Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│ Layer 5: APPLICATION       │ Service Tasks, Human Interaction  │
├─────────────────────────────────────────────────────────────────┤
│ Layer 4: COGNITION         │ LLM, Planning, Decision Making    │
├─────────────────────────────────────────────────────────────────┤
│ Layer 3: PERCEPTION        │ Vision, Audio, Sensor Fusion      │
├─────────────────────────────────────────────────────────────────┤
│ Layer 2: NAVIGATION        │ SLAM, Path Planning, Localization │
├─────────────────────────────────────────────────────────────────┤
│ Layer 1: CONTROL           │ Motor Control, Safety, E-Stop     │
├─────────────────────────────────────────────────────────────────┤
│ Layer 0: HARDWARE          │ Sensors, Actuators, Power         │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Minimum Package Structure

```
mipax/
├── mipax_perception/           # Layer 3: All perception on Orin
│   ├── vision_pipeline.py      # Camera processing, object detection
│   ├── audio_pipeline.py       # Microphone, ASR, wake word
│   ├── sensor_fusion.py        # Multi-sensor integration
│   └── scene_understanding.py  # VLM-based scene interpretation
│
├── mipax_cognition/            # Layer 4: AI/Planning on Orin
│   ├── llm_interface.py        # Lightweight LLM wrapper
│   ├── task_planner.py         # High-level task planning
│   ├── dialog_manager.py       # Conversation handling
│   └── behavior_fsm.py         # State machine for behaviors
│
├── mipax_navigation/           # Layer 2: Navigation on IPC
│   ├── slam_manager.py         # SLAM coordination
│   ├── path_planner.py         # Nav2 interface
│   └── localization.py         # Position estimation
│
├── mipax_control/              # Layer 1: Control
│   ├── safety_monitor.py       # Central safety system
│   ├── base_controller.py      # Mobile base control
│   └── emergency_stop.py       # E-stop handling
│
├── mipax_arms/                 # Layer 1: Arm control (existing)
│   ├── tmotor_driver.py        # T-Motor CAN driver
│   └── dual_arm_controller.py  # Dual-arm coordination
│
├── mipax_speech/               # Layer 3/4: Speech I/O
│   ├── speech_recognition.py   # ASR interface
│   ├── text_to_speech.py       # TTS interface
│   └── wake_word_detector.py   # Keyword spotting
│
├── mipax_interface/            # Layer 5: User interface
│   ├── touchscreen_ui.py       # 13.3" touchscreen
│   ├── led_feedback.py         # Status indicators
│   └── remote_api.py           # External API access
│
└── mipax_bringup/              # System launch
    ├── launch/
    │   ├── minimal_robot.launch.py     # Minimum safe operation
    │   ├── full_system.launch.py       # Complete system
    │   └── perception_only.launch.py   # Testing mode
    └── config/
        ├── safety_params.yaml          # Safety thresholds
        ├── perception_params.yaml      # AI model configs
        └── navigation_params.yaml      # Nav2 configs
```

---

## 3. Safety Architecture (CRITICAL)

### 3.1 Safety Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│                    SAFETY PRIORITY LEVELS                        │
├─────────────────────────────────────────────────────────────────┤
│ P0: HARDWARE E-STOP        │ Physical button → immediate stop   │
│                            │ Bypasses all software              │
├─────────────────────────────────────────────────────────────────┤
│ P1: COLLISION DETECTION    │ Ultrasonic + LiDAR → stop motion   │
│                            │ < 100ms response time              │
├─────────────────────────────────────────────────────────────────┤
│ P2: SOFT LIMITS            │ Joint/velocity limits → clamp      │
│                            │ Software-enforced boundaries       │
├─────────────────────────────────────────────────────────────────┤
│ P3: BEHAVIOR LIMITS        │ No-go zones, speed limits          │
│                            │ Context-aware restrictions         │
├─────────────────────────────────────────────────────────────────┤
│ P4: WATCHDOG TIMEOUTS      │ Communication loss → safe stop     │
│                            │ 500ms default timeout              │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 Safety Monitor Node (REQUIRED)

```python
# mipax_control/safety_monitor.py - ALWAYS RUNNING

class SafetyMonitor:
    """
    Central safety coordinator - MUST be running for robot operation.
    Runs at 100Hz on dedicated thread with highest priority.
    """

    SAFETY_STATES = [
        'SAFE',           # Normal operation
        'CAUTION',        # Reduced speed, heightened awareness
        'DANGER',         # Stop motion, maintain position
        'EMERGENCY',      # Immediate stop, disable motors
    ]

    def __init__(self):
        # Collision thresholds (meters)
        self.collision_stop_distance = 0.15      # Immediate stop
        self.collision_slow_distance = 0.50      # Reduce speed
        self.collision_warn_distance = 1.00      # Increase awareness

        # Velocity limits (m/s, rad/s)
        self.max_linear_velocity = 0.5           # Slow for safety
        self.max_angular_velocity = 0.5
        self.max_arm_velocity = 1.0              # rad/s per joint

        # Watchdog timeouts (seconds)
        self.perception_timeout = 1.0            # Camera/LiDAR data
        self.control_timeout = 0.5               # Motor feedback
        self.heartbeat_timeout = 2.0             # System heartbeat

    def check_safety(self) -> SafetyState:
        """Called at 100Hz - determines current safety state."""

        # P1: Check collision sensors
        min_distance = self.get_minimum_obstacle_distance()
        if min_distance < self.collision_stop_distance:
            return 'EMERGENCY'
        elif min_distance < self.collision_slow_distance:
            return 'DANGER'
        elif min_distance < self.collision_warn_distance:
            return 'CAUTION'

        # P4: Check watchdogs
        if not self.check_all_watchdogs():
            return 'DANGER'

        return 'SAFE'
```

### 3.3 Minimum Safety Requirements

| Component | Requirement | Verification |
|-----------|-------------|--------------|
| **E-Stop** | Hardware interrupt, <10ms response | Test before each use |
| **Collision** | 8 ultrasonic sensors active | Continuous monitoring |
| **LiDAR** | 360° coverage operational | Startup check |
| **Watchdog** | All nodes report heartbeat | 500ms timeout |
| **Velocity** | Hard limits in firmware | Cannot be overridden by software |
| **Arm Torque** | <8Nm per joint | Hardware current limiting |

---

## 4. Perception Pipeline (Orin #1)

### 4.1 Vision Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    VISION PIPELINE (30 FPS)                      │
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │ Camera Input │───>│  Detection   │───>│   Tracking   │      │
│  │ - Wide 190°  │    │ - YOLO-Nano  │    │ - DeepSORT   │      │
│  │ - Fish-eye   │    │ - People     │    │ - Object IDs │      │
│  │ - Secondary  │    │ - Objects    │    │              │      │
│  └──────────────┘    └──────────────┘    └──────────────┘      │
│         │                   │                   │               │
│         v                   v                   v               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │    Depth     │───>│   3D Scene   │───>│  Perception  │      │
│  │  Processing  │    │   Graph      │    │   Output     │      │
│  │ - Point cloud│    │ - Positions  │    │ - /objects   │      │
│  │ - Obstacles  │    │ - Relations  │    │ - /people    │      │
│  └──────────────┘    └──────────────┘    └──────────────┘      │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 Minimum Detection Requirements

| Category | Detection | Model | Accuracy Target |
|----------|-----------|-------|-----------------|
| **People** | Presence, position, distance | YOLO-Nano | 85%+ |
| **Obstacles** | Any blocking object | Depth + LiDAR | 95%+ |
| **Hands** | Gesture recognition (basic) | MediaPipe | 70%+ |
| **Faces** | Presence (not identity) | Lightweight CNN | 80%+ |
| **Objects** | Common items (cup, chair, etc.) | YOLO-Nano | 70%+ |

### 4.3 Audio Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    AUDIO PIPELINE                                │
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │  Mic Array   │───>│  Wake Word   │───>│     ASR      │      │
│  │ - 4+ mics    │    │  Detection   │    │  (Whisper)   │      │
│  │ - Echo cancel│    │ "Hey MiPA"   │    │  tiny/base   │      │
│  └──────────────┘    └──────────────┘    └──────────────┘      │
│         │                   │                   │               │
│         v                   v                   v               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │   Beamform   │    │   Intent     │    │    Text      │      │
│  │  Direction   │    │   Classify   │    │   Output     │      │
│  └──────────────┘    └──────────────┘    └──────────────┘      │
└─────────────────────────────────────────────────────────────────┘
```

### 4.4 Sensor Fusion

```python
# mipax_perception/sensor_fusion.py

class SensorFusion:
    """
    Combines all sensor data into unified world model.
    Runs at 50Hz on Orin #1.
    """

    def fuse(self) -> WorldState:
        """
        Outputs:
        - obstacles: List of obstacle positions/sizes
        - people: List of detected people with positions
        - objects: List of recognized objects
        - free_space: Navigable area map
        - robot_pose: Current estimated position
        """

        # Combine LiDAR + depth cameras for obstacle map
        obstacle_map = self.merge_depth_lidar()

        # Overlay people detections
        people = self.track_people(self.vision_detections)

        # Build scene graph
        scene = SceneGraph(
            obstacles=obstacle_map,
            people=people,
            objects=self.detected_objects
        )

        return WorldState(scene, self.robot_pose)
```

---

## 5. Cognition Pipeline (Orin #2)

### 5.1 Lightweight LLM Integration

**Model Selection for MVP**:
- **Primary**: Phi-3-mini (3.8B) or Llama-3.2-3B
- **Fallback**: Rule-based responses for critical functions
- **Future**: Larger models as hardware allows

```python
# mipax_cognition/llm_interface.py

class LightweightLLM:
    """
    Wrapper for on-device LLM inference.
    Designed for simple reasoning and dialog.
    """

    def __init__(self):
        # Load quantized model (INT4/INT8 for Orin)
        self.model = load_model("phi-3-mini-4bit")

        # System prompt defines robot personality/capabilities
        self.system_prompt = """
        You are MiPA-X, a helpful service robot. You can:
        - Navigate to locations in the building
        - Pick up and deliver small objects
        - Answer simple questions
        - Call for human assistance when needed

        Always prioritize safety. If unsure, ask for clarification.
        Keep responses brief and clear.
        """

    def process(self, user_input: str, context: WorldState) -> Response:
        """
        Generate response based on user input and world state.

        Returns:
        - text: Spoken response
        - action: Optional action to execute
        - confidence: Model confidence (0-1)
        """

        # Build context-aware prompt
        prompt = self.build_prompt(user_input, context)

        # Generate with timeout (max 2 seconds)
        response = self.model.generate(
            prompt,
            max_tokens=100,
            timeout=2.0
        )

        # Parse for actions
        action = self.extract_action(response)

        return Response(
            text=response.text,
            action=action,
            confidence=response.confidence
        )
```

### 5.2 Behavior State Machine

```
┌─────────────────────────────────────────────────────────────────┐
│                    BEHAVIOR STATE MACHINE                        │
│                                                                  │
│                         ┌──────────┐                            │
│                         │   IDLE   │◄────────────────┐          │
│                         └────┬─────┘                 │          │
│                              │ wake_word             │          │
│                              v                       │ timeout  │
│                         ┌──────────┐                 │          │
│                    ┌───>│ LISTENING│────────────────>│          │
│                    │    └────┬─────┘                 │          │
│                    │         │ command               │          │
│                    │         v                       │          │
│                    │    ┌──────────┐                 │          │
│                    │    │ THINKING │                 │          │
│                    │    └────┬─────┘                 │          │
│                    │         │ action                │          │
│                    │         v                       │          │
│       ┌────────────┼────┬────┴────┬────────────┐    │          │
│       v            │    v         v            v    │          │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐│          │
│  │NAVIGATING│  │SPEAKING │  │GRASPING │  │ ASKING  ││          │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘│          │
│       │            │            │            │      │          │
│       └────────────┴────────────┴────────────┴──────┘          │
│                              │                                   │
│                              v                                   │
│                    ┌──────────────────┐                         │
│                    │     EMERGENCY    │◄── Any P0/P1 trigger    │
│                    │   (Safe Stop)    │                         │
│                    └──────────────────┘                         │
└─────────────────────────────────────────────────────────────────┘
```

### 5.3 Task Planning

```python
# mipax_cognition/task_planner.py

class TaskPlanner:
    """
    Decomposes high-level goals into executable actions.
    MVP: Simple rule-based + LLM assistance.
    """

    # Predefined task templates
    TASK_TEMPLATES = {
        'fetch_object': [
            ('navigate', 'object_location'),
            ('detect', 'object'),
            ('grasp', 'object'),
            ('navigate', 'delivery_location'),
            ('release', 'object'),
        ],
        'guide_person': [
            ('navigate', 'person_location'),
            ('speak', 'follow_me'),
            ('navigate', 'destination'),
            ('speak', 'arrived'),
        ],
        'deliver_message': [
            ('navigate', 'recipient_location'),
            ('detect', 'person'),
            ('speak', 'message'),
            ('wait', 'acknowledgment'),
        ],
    }

    def plan(self, goal: str, context: WorldState) -> ActionSequence:
        """
        Create action sequence for goal.
        Falls back to simple behaviors if planning fails.
        """

        # Try template matching first (fast, reliable)
        template = self.match_template(goal)
        if template:
            return self.instantiate_template(template, context)

        # Use LLM for novel goals (slower, less reliable)
        llm_plan = self.llm.plan(goal, context)
        if llm_plan.confidence > 0.7:
            return llm_plan.actions

        # Fallback: Ask for clarification
        return ActionSequence([
            ('speak', "I'm not sure how to do that. Could you be more specific?")
        ])
```

---

## 6. Navigation System (IPC)

### 6.1 Nav2 Configuration

```yaml
# config/navigation_params.yaml

# Conservative settings for service robot safety
controller_server:
  ros__parameters:
    controller_frequency: 20.0

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.3          # Slow for safety (m/s)
      max_vel_theta: 0.5      # Limited rotation (rad/s)
      min_vel_x: 0.0
      acc_lim_x: 1.0
      acc_lim_theta: 2.0

      # Critics for safe behavior
      critics: ["ObstacleFootprint", "GoalAlign", "PathAlign", "GoalDist"]
      ObstacleFootprint.scale: 100.0  # High weight = very obstacle-averse

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.3
      use_astar: true
      allow_unknown: false    # Only navigate through known space

costmap:
  ros__parameters:
    robot_radius: 0.30        # 30cm safety margin
    inflation_radius: 0.55    # Keep distance from obstacles
    cost_scaling_factor: 5.0

    # Obstacle detection
    obstacle_layer:
      observation_sources: lidar ultrasonic
      lidar:
        topic: /scan
        marking: true
        clearing: true
        obstacle_range: 5.0
        raytrace_range: 6.0
      ultrasonic:
        topic: /ultrasonic_array
        marking: true
        clearing: false
        obstacle_range: 2.0
```

### 6.2 SLAM Configuration

```yaml
# config/slam_params.yaml

slam_toolbox:
  ros__parameters:
    # Conservative SLAM settings
    mode: mapping            # or localization

    resolution: 0.05         # 5cm map resolution
    max_laser_range: 10.0

    # Loop closure settings
    do_loop_closing: true
    loop_match_minimum_chain_size: 10

    # Update rates
    map_update_interval: 2.0  # seconds

    # Transform timeout
    transform_timeout: 0.5
```

---

## 7. Speech System

### 7.1 Speech Recognition (ASR)

```python
# mipax_speech/speech_recognition.py

class SpeechRecognition:
    """
    Lightweight ASR using Whisper tiny/base on Orin.
    """

    def __init__(self):
        # Whisper tiny for speed, base for accuracy
        self.model = whisper.load_model("tiny", device="cuda")

        # Wake word detector (runs continuously)
        self.wake_detector = WakeWordDetector(
            keywords=["hey mipa", "hello mipa", "mipa"]
        )

    def listen(self) -> Optional[str]:
        """
        Listen for wake word, then transcribe command.
        Returns None if no valid input detected.
        """

        # Wait for wake word (low CPU usage)
        if not self.wake_detector.detected():
            return None

        # Record audio after wake word (max 10 seconds)
        audio = self.record(max_duration=10.0)

        # Transcribe
        result = self.model.transcribe(
            audio,
            language="en",
            fp16=True  # Use half precision on Orin
        )

        return result["text"] if result["confidence"] > 0.5 else None
```

### 7.2 Text-to-Speech (TTS)

```python
# mipax_speech/text_to_speech.py

class TextToSpeech:
    """
    On-device TTS for robot responses.
    Uses Piper or Coqui for lightweight synthesis.
    """

    def __init__(self):
        # Load lightweight TTS model
        self.tts = PiperVoice.load("en_US-amy-medium")

        # Pre-cache common responses
        self.cache = {
            "hello": self.synthesize("Hello! How can I help you?"),
            "follow_me": self.synthesize("Please follow me."),
            "arrived": self.synthesize("We have arrived."),
            "error": self.synthesize("I'm sorry, I couldn't do that."),
            "help": self.synthesize("I'm calling for assistance."),
        }

    def speak(self, text: str):
        """
        Convert text to speech and play through speakers.
        """

        # Check cache first
        if text in self.cache:
            audio = self.cache[text]
        else:
            audio = self.synthesize(text)

        # Play audio
        self.play(audio)
```

---

## 8. Arm Control Integration

### 8.1 Basic Grasping

```python
# mipax_arms/basic_grasping.py

class BasicGrasping:
    """
    MVP grasping - simple approach and grip.
    No complex manipulation, just pick and place.
    """

    # Predefined grasp poses
    GRASP_PRESETS = {
        'cup': {'width': 0.08, 'force': 3.0},
        'bottle': {'width': 0.06, 'force': 4.0},
        'box': {'width': 0.15, 'force': 5.0},
        'default': {'width': 0.10, 'force': 3.5},
    }

    def grasp_object(self, object_pose, object_type='default'):
        """
        Simple grasp sequence:
        1. Move to pre-grasp position (10cm above)
        2. Open gripper
        3. Move down to grasp position
        4. Close gripper with appropriate force
        5. Lift object
        6. Verify grasp success
        """

        preset = self.GRASP_PRESETS.get(object_type, self.GRASP_PRESETS['default'])

        # Pre-grasp approach
        pre_grasp = object_pose.copy()
        pre_grasp.z += 0.10
        self.move_to(pre_grasp)

        # Open gripper
        self.gripper.open(preset['width'] + 0.02)

        # Approach
        self.move_to(object_pose, speed=0.1)

        # Grasp
        self.gripper.close(force=preset['force'])

        # Verify
        if self.gripper.holding():
            self.move_to(pre_grasp)  # Lift
            return True
        else:
            self.gripper.open()
            return False
```

### 8.2 Arm Safety Limits

```yaml
# config/arm_safety.yaml

arm_safety:
  # Never exceed these limits
  max_joint_velocity: 1.0      # rad/s
  max_joint_acceleration: 2.0  # rad/s^2
  max_gripper_force: 10.0      # N

  # Workspace limits (relative to robot base)
  workspace:
    x_min: 0.1
    x_max: 0.8
    y_min: -0.5
    y_max: 0.5
    z_min: 0.2
    z_max: 1.2

  # Force limits for collision detection
  collision_force_threshold: 15.0  # N
  collision_torque_threshold: 5.0  # Nm
```

---

## 9. User Interface

### 9.1 Touchscreen UI

```
┌─────────────────────────────────────────────────────────────────┐
│                    13.3" TOUCHSCREEN UI                          │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    STATUS BAR                                ││
│  │  [Battery: 85%]  [WiFi: OK]  [Status: Ready]  [E-Stop: OK]  ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                  │
│  ┌────────────────────────┐  ┌────────────────────────────────┐│
│  │                        │  │         QUICK ACTIONS           ││
│  │                        │  │  ┌──────────┐ ┌──────────┐    ││
│  │    CAMERA FEED         │  │  │  FOLLOW  │ │   STOP   │    ││
│  │    (Optional)          │  │  │    ME    │ │          │    ││
│  │                        │  │  └──────────┘ └──────────┘    ││
│  │                        │  │  ┌──────────┐ ┌──────────┐    ││
│  │                        │  │  │   HOME   │ │   HELP   │    ││
│  │                        │  │  │          │ │          │    ││
│  └────────────────────────┘  │  └──────────┘ └──────────┘    ││
│                              └────────────────────────────────┘│
│                                                                  │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    CONVERSATION                              ││
│  │  User: "Take this cup to the kitchen"                       ││
│  │  MiPA: "I'll take the cup to the kitchen. Please hand it..." ││
│  │                                                              ││
│  │  [🎤 Listening...]                  [Type a command...]     ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

### 9.2 LED Status Indicators

| LED State | Meaning | Priority |
|-----------|---------|----------|
| **Solid Green** | Ready, idle | Normal |
| **Breathing Blue** | Listening, processing | Normal |
| **Pulsing Yellow** | Moving, working | Normal |
| **Solid Yellow** | Caution, slow mode | Warning |
| **Flashing Red** | Error, needs attention | Error |
| **Solid Red** | Emergency stop active | Critical |

---

## 10. System Integration

### 10.1 Launch Hierarchy

```python
# mipax_bringup/launch/minimal_robot.launch.py

"""
Minimum viable launch - safe operation only.
Use this for initial testing and commissioning.
"""

def generate_launch_description():
    return LaunchDescription([
        # LAYER 0: Hardware drivers (REQUIRED)
        IncludeLaunchDescription('drivers.launch.py'),

        # LAYER 1: Safety & Control (REQUIRED)
        Node(package='mipax_control', executable='safety_monitor'),
        Node(package='mipax_control', executable='emergency_stop'),
        Node(package='mipax_control', executable='base_controller'),

        # LAYER 2: Navigation (REQUIRED for mobility)
        IncludeLaunchDescription('navigation.launch.py'),

        # LAYER 3: Perception (REQUIRED for safety)
        Node(package='mipax_perception', executable='sensor_fusion'),
        Node(package='mipax_perception', executable='obstacle_detector'),

        # LAYER 4-5: Optional for minimal operation
        # - Cognition (LLM) - can be disabled
        # - Speech - can be disabled
        # - Arms - can be disabled
    ])
```

### 10.2 ROS 2 Topic Map

```
┌─────────────────────────────────────────────────────────────────┐
│                    MINIMUM VIABLE TOPICS                         │
│                                                                  │
│  SAFETY (REQUIRED - 100Hz)                                      │
│  ├── /safety/state          SafetyState    Current safety level │
│  ├── /safety/e_stop         Bool           E-stop status        │
│  └── /safety/heartbeat      Heartbeat      System health        │
│                                                                  │
│  PERCEPTION (REQUIRED - 30Hz)                                   │
│  ├── /perception/obstacles  ObstacleArray  Detected obstacles   │
│  ├── /perception/people     PersonArray    Detected people      │
│  └── /perception/scene      SceneGraph     World model          │
│                                                                  │
│  NAVIGATION (REQUIRED - 10Hz)                                   │
│  ├── /odom                  Odometry       Robot pose           │
│  ├── /scan                  LaserScan      LiDAR data           │
│  ├── /map                   OccupancyGrid  Environment map      │
│  └── /cmd_vel               Twist          Velocity commands    │
│                                                                  │
│  SPEECH (OPTIONAL)                                               │
│  ├── /speech/text_in        String         Recognized text      │
│  └── /speech/text_out       String         Text to speak        │
│                                                                  │
│  COGNITION (OPTIONAL)                                            │
│  ├── /task/goal             TaskGoal       High-level goal      │
│  ├── /task/status           TaskStatus     Execution status     │
│  └── /behavior/state        BehaviorState  Current behavior     │
│                                                                  │
│  ARMS (OPTIONAL)                                                 │
│  ├── /arms/joint_states     JointState     Arm feedback         │
│  └── /arms/joint_commands   JointState     Arm commands         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 11. Minimum Viable AI Stack

### 11.1 Model Requirements

| Function | Model | Size | Platform | Latency Target |
|----------|-------|------|----------|----------------|
| **Object Detection** | YOLOv8-nano | 3MB | Orin (TensorRT) | <50ms |
| **Person Detection** | YOLOv8-nano | 3MB | Orin (TensorRT) | <50ms |
| **Depth Estimation** | MiDaS-small | 25MB | Orin (TensorRT) | <100ms |
| **ASR** | Whisper-tiny | 39MB | Orin (CUDA) | <2s |
| **TTS** | Piper (medium) | 60MB | Orin (CPU) | <500ms |
| **LLM** | Phi-3-mini (4-bit) | 2GB | Orin (CUDA) | <3s |
| **VLM** | LLaVA-Phi-3 (4-bit) | 3GB | Orin (CUDA) | <5s |

### 11.2 Fallback Strategy

```
┌─────────────────────────────────────────────────────────────────┐
│                    AI FALLBACK HIERARCHY                         │
│                                                                  │
│  Primary: Full AI Stack                                          │
│  ├── LLM + VLM + ASR + TTS                                      │
│  ├── Capability: Natural conversation, scene understanding       │
│  └── Requirement: Both Orins operational                        │
│                                                                  │
│  Fallback 1: Reduced AI                                          │
│  ├── Rule-based + ASR + TTS (no LLM)                            │
│  ├── Capability: Fixed commands, basic responses                │
│  └── Requirement: One Orin operational                          │
│                                                                  │
│  Fallback 2: Safety Only                                         │
│  ├── Perception + Navigation (no AI)                            │
│  ├── Capability: Obstacle avoidance, return to home             │
│  └── Requirement: IPC operational                               │
│                                                                  │
│  Fallback 3: Emergency                                           │
│  ├── E-stop active, motors disabled                             │
│  ├── Capability: None (safe stop)                               │
│  └── Requirement: Power only                                    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 12. Development Roadmap

### Phase 1: Minimum Viable Robot (MVP) - 8 weeks

**Week 1-2: Safety Foundation**
- [ ] Implement SafetyMonitor node
- [ ] Configure E-stop hardware integration
- [ ] Test collision detection (ultrasonic + LiDAR)
- [ ] Verify velocity limits in firmware

**Week 3-4: Navigation**
- [ ] Configure Nav2 with conservative parameters
- [ ] Implement SLAM with slam_toolbox
- [ ] Test basic autonomous navigation
- [ ] Create initial building map

**Week 5-6: Perception**
- [ ] Deploy YOLO-nano for object/person detection
- [ ] Implement sensor fusion node
- [ ] Test obstacle detection accuracy
- [ ] Integrate depth processing

**Week 7-8: Basic Interaction**
- [ ] Implement wake word detection
- [ ] Deploy Whisper-tiny for ASR
- [ ] Configure TTS with Piper
- [ ] Test basic voice commands

### Phase 2: Service Capability - 8 weeks

**Week 9-10: LLM Integration**
- [ ] Deploy Phi-3-mini on Orin
- [ ] Implement dialog manager
- [ ] Create task templates
- [ ] Test natural language commands

**Week 11-12: Arm Control**
- [ ] Integrate mipax_arms package
- [ ] Implement basic grasping
- [ ] Test pick and place operations
- [ ] Add force feedback monitoring

**Week 13-14: Full Integration**
- [ ] Connect all subsystems
- [ ] Implement behavior state machine
- [ ] Test end-to-end scenarios
- [ ] Conduct safety review

**Week 15-16: Testing & Refinement**
- [ ] User acceptance testing
- [ ] Performance optimization
- [ ] Documentation completion
- [ ] Prepare for deployment

### Phase 3: Future Improvements (Post-MVP)

- **Enhanced AI**: Larger LLMs, better VLMs
- **Advanced Manipulation**: Complex grasping, tool use
- **Multi-Robot**: Fleet coordination
- **Learning**: Behavior adaptation, user preferences
- **Cloud Integration**: Remote monitoring, updates

---

## 13. Key Design Decisions

### 13.1 Why Centralized Perception on Orin?

| Benefit | Explanation |
|---------|-------------|
| **Unified World Model** | All sensors feed into single perception pipeline |
| **GPU Acceleration** | AI models run efficiently on Orin's GPU |
| **Reduced Latency** | No network hops for sensor data |
| **Simpler Debugging** | One place to check perception issues |
| **Future Scalability** | Easy to upgrade AI models |

### 13.2 Why Separate Navigation on IPC?

| Benefit | Explanation |
|---------|-------------|
| **Real-Time Guarantees** | Navigation needs deterministic timing |
| **Isolation** | AI crashes don't affect safe motion |
| **Proven Stack** | Nav2 is battle-tested |
| **Resource Separation** | Orin GPUs free for AI |

### 13.3 Why Lightweight AI First?

| Benefit | Explanation |
|---------|-------------|
| **Fast Iteration** | Quick to deploy, test, improve |
| **Graceful Degradation** | Works even with failures |
| **Power Efficiency** | Longer battery life |
| **Safety** | Simpler = more predictable |
| **Upgrade Path** | Architecture supports larger models |

---

## 14. Acceptance Criteria

### Minimum Viable Robot must:

**Safety (MUST PASS)**
- [ ] E-stop halts all motion within 100ms
- [ ] Robot stops before collision (0.15m minimum)
- [ ] Never exceed velocity limits under any condition
- [ ] Return to safe state on any system failure

**Navigation (MUST PASS)**
- [ ] Navigate from A to B without collision
- [ ] Avoid dynamic obstacles (people)
- [ ] Return to charging station autonomously
- [ ] Operate in environments with mapped layout

**Perception (MUST PASS)**
- [ ] Detect people within 5m at 85%+ accuracy
- [ ] Detect obstacles at 95%+ accuracy
- [ ] Process sensor data at 30Hz minimum

**Interaction (SHOULD HAVE)**
- [ ] Respond to wake word within 2 seconds
- [ ] Understand basic commands (go to, follow, stop)
- [ ] Provide audible feedback for actions
- [ ] Display status on touchscreen

**Manipulation (SHOULD HAVE)**
- [ ] Pick up objects from table height
- [ ] Carry objects while navigating
- [ ] Place objects at designated locations

---

## Summary

This **Minimum Viable Software Architecture** provides:

1. **Safety-First Design**: Multiple layers of protection, hardware E-stop priority
2. **Centralized Perception**: All sensing on NVIDIA Orin for unified AI
3. **Modular Structure**: Easy to upgrade individual components
4. **Graceful Degradation**: Partial failures don't cause dangerous behavior
5. **Lightweight AI**: Functional but imperfect, designed for improvement
6. **Clear Upgrade Path**: Architecture supports future enhancements

The MVP focuses on being a **safe, minimally capable service robot** that can:
- Navigate safely in indoor spaces
- Understand basic voice commands
- Perform simple fetch and delivery tasks
- Interact naturally with humans

This foundation is designed to be incrementally improved as better AI models and hardware become available.

---

*Document Version: 1.0 (MVP)*
*Created: 2026-01-14*
*Next Review: After Phase 1 completion*

---
---
---

# MiPA-X Hardware Architecture

**Version**: 3.1
**Last Updated**: 2026-01-12
**Project**: MiPA-X Robotic Platform

---

## Overview

The MiPA-X architecture is centered around high-compute modules for vision and AI, interconnected via a high-speed 1Gbps Ethernet network. The system utilizes a dual-processor approach for high-level tasks with hierarchical control layers for real-time operation.

---

## 1. Core Compute & Vision Processing

### Primary Processing Units

#### Dual NVIDIA Jetson Orin Modules
- **Function**: Primary "brains" for the robot
- **Location**: Upper Body
- **Responsibilities**:
  - Vision processing from RGBD cameras
  - AI inference and decision making
  - Video streaming at 720p (1280x720) @ 30FPS
  - Audio processing with microphone arrays and echo cancellation

#### Verdin iMX8M Plus
- **Type**: ARM-based System-on-Module (SoM)
- **Location**: Upper Body
- **Responsibilities**:
  - Dual 7-DOF robotic arm control interface
  - Local control bridge between Jetson and arm actuators
  - Real-time motion planning and trajectory execution
  - Dual-CAN bus management (can0 for left arm, can1 for right arm)

> **📘 Dual-Arm Control**: For detailed information about the T-Motor based dual-arm system, see [DUAL_ARM_INTEGRATION.md](DUAL_ARM_INTEGRATION.md) and [mipax_arms/README.md](mipax/mipax_arms/README.md).

#### Industrial PC (IPC)
- **Platform**: Running ROS 2 (Robot Operating System)
- **Location**: Chassis/Base
- **Responsibilities**:
  - Central node for chassis navigation
  - IMU and ultrasonic sensor data coordination
  - Base mobility control
  - SLAM and mapping

#### STM32H7 Microcontroller
- **Interface**: CAN bus @ 1 Mbps
- **Location**: Chassis - MCU + Power Board
- **Responsibilities**:
  - Real-time control loop (1ms / 1000Hz)
  - Low-level hardware interfacing
  - Power distribution management
  - Emergency stop monitoring

---

## 2. Structural Segmentation

### Upper Body (上半身)

**Components**:
- Dual NVIDIA Jetson Orin modules
- Verdin iMX8M Plus SoM
- Power distribution board
- Robotic arm interface
- 13.3-inch touchscreen (flush-mounted)
- Audio subsystem (microphone arrays, echo cancellation)

**Pending Improvements (v3.71)**:
- Optimization of module placement
- Improved cable management with cable trays
- Touchscreen flush integration with housing

### Chassis/Base (底盘)

**Components**:
- Industrial PC (IPC) running ROS 2
- 48V Battery system with display
- Emergency Stop Button (急停按钮)
- Physical safety switches
- Indicator lights
- Motor controllers

**Critical Issues**:
- Chassis exceeds original design dimensions (requires size optimization)
- Battery accessibility improvement needed for user-swappable design
- Motor failure incident (front-right drive motor) indicates need for easier maintenance access

**Pending Improvements (v3.71)**:
- Structural optimization for easier maintenance
- Battery position redesign for autonomous swapping
- Chassis size reduction

---

## 3. Sensors and Connectivity

### Vision Sensors

#### Wide-Angle RGBD Camera
- **FOV**: 190° Horizontal / 114° Vertical
- **Resolution**: 1280x720 (720p)
- **Frame Rate**: 30 Hz
- **Location**: Upper Body
- **Purpose**: Primary spatial awareness and environment understanding

#### Secondary RGBD Camera
- **FOV**: 86° Horizontal / 58° Vertical
- **Resolution**: 640x480
- **Frame Rate**: 30 Hz
- **Location**: Upper Body
- **Purpose**: Localized manipulation and robotic arm feedback

#### Fish-eye Cameras (2x)
- **FOV**: 120° Horizontal
- **Resolution**: 1920x1080
- **Frame Rate**: 30 Hz
- **Location**: Front & Rear of upper body
- **Purpose**: Wide-area surround view

#### 335LG Precision Sensor
- **FOV**: 18° Vertical
- **Location**: Height-specific mounting (moved downward in v3.1 due to space constraints)
- **Purpose**: Precision depth/ranging for specific tasks

### Navigation Sensors

#### 3D LiDAR
- **FOV**: 360° Horizontal coverage
- **Vertical Layers**: 16-32 layers
- **Vertical FOV**: ±15°
- **Range**: 0.3m to 100m
- **Update Rate**: 10 Hz
- **Location**: Center-mounted on base
- **Purpose**: SLAM and environmental mapping

#### IMU (Inertial Measurement Unit)
- **Update Rate**: 50-100 Hz
- **Location**: Chassis
- **Data**: Acceleration, gyroscope, orientation
- **Integration**: EKF fusion with wheel odometry @ 50Hz

#### Ultrasonic Sensor Array
- **Count**: 8 sensors
- **Sensing Angle**: 75° per sensor
- **Positions**: Distributed around base (Front-Left, Front-Right, Right-Front, Right-Rear, Rear-Right, Rear-Left, Left-Rear, Left-Front)
- **Range**: 0.02m to 4.0m
- **Update Rate**: 30 Hz
- **Purpose**: Close-range collision avoidance with 360° proximity coverage

### Network Infrastructure

#### Primary Backbone
- **Bandwidth**: 1Gbps (1000 Mbps) Ethernet
- **Important Note**: Bandwidth specified in **Mb/s (Megabits per second)**, NOT MB/s (Megabytes per second)
- **Topology**: Hierarchical - connecting upper body compute modules to chassis IPC

#### Wireless Connectivity
- **Wi-Fi**: Dual-band (2.4GHz + 5GHz)
- **Bluetooth**: 5.0
- **Purpose**: External communication and remote control

---

## 4. Physical Specifications

### Robotic Arm System

**Linear Elevation**:
- **Lift Range**: 600mm vertical travel
- **Mounting**: Linear axis on upper body

**Workspace Optimization** (Under Evaluation):
- Potential increase in joint angles (J2, J4, J6) for improved elbow and wrist workspace
- Dependent on final housing design
- Payload target: ~7.5kg (not achievable with current hardware)
- Arm length target: 900mm (not achievable with current hardware)

### Dual 7-DOF Arm System (T-Motor Based)

**Configuration**:
- **Left Arm**: 7 DOF (Degrees of Freedom)
- **Right Arm**: 7 DOF (Degrees of Freedom)
- **Total**: 14 actuated joints

**Motor Specifications** (T-Motor AK80-6 V2):
- **Torque**: Continuous 6 Nm, Peak 9 Nm
- **Speed**: Max 41 rad/s (~391 RPM)
- **Control**: Position, Velocity, Torque, Hybrid modes
- **Communication**: CAN 2.0B
- **Encoder**: 16-bit absolute positioning

**CAN Communication**:
- **Left Arm CAN Bus**: can0 @ 1 Mbps
  - Motor IDs: 0x01 through 0x07
- **Right Arm CAN Bus**: can1 @ 1 Mbps
  - Motor IDs: 0x01 through 0x07
- **STM32H7 Interface**: Dual CAN transceivers
- **Control Rate**: 300 Hz (per arm)
- **Bandwidth Utilization**: 67% (33% safety margin)

**Control Architecture**:
- **Low-Level**: STM32H7 MCU (1ms real-time loop)
- **Mid-Level**: Verdin iMX8M Plus (ROS 2 driver node)
- **High-Level**: Pyro-based trajectory planning
- **Modes**: 8 control modes per joint
  - disable, enable, position, velocity, torque, damped_torque, vel_torque, full

**Kinematic Parameters** (Example - Per Arm):
- **Link Lengths**: [0.3, 0.3, 0.25, 0.25, 0.15, 0.12, 0.10] meters
- **Link Masses**: [2.0, 1.8, 1.5, 1.2, 0.8, 0.5, 0.3] kg
- **Total Reach**: ~1.57 meters
- **Payload**: Up to 3 kg per arm

**Software Stack**:
- **ROS 2 Package**: mipax_arms
- **Motor Driver**: mini-cheetah-tmotor-python-can
- **Trajectory Planning**: Pyro robotics library
- **Control Algorithms**: Computed torque, PD control, inverse kinematics
- **Safety Features**: Joint limits, collision avoidance, watchdog timers

**Integration**:
- Based on: [tmotor_ros2](https://github.com/SherbyRobotics/tmotor_ros2.git)
- See: [DUAL_ARM_INTEGRATION.md](DUAL_ARM_INTEGRATION.md) for complete details
- Package: [mipax_arms](mipax/mipax_arms/)

### Base Dimensions
- **Robot Radius**: 0.22m (220mm)
- **Current Status**: Chassis exceeds design specifications
- **Planned**: Size optimization in v3.71

### Power System
- **Battery**: 48V system
- **Display**: Internal battery data display on body
- **Issue**: Current battery position difficult to access (redesign planned)

### User Interface
- **Touchscreen**: 13.3-inch
- **Mounting**: Being updated to flush-mount design

---

## 5. Communication Architecture

### High-Speed Data Layer
```
┌─────────────────────────────────────────────────────┐
│           1Gbps Ethernet Backbone                    │
│              (1000 Mb/s)                             │
│                                                       │
│  ┌──────────────────┐         ┌──────────────────┐  │
│  │  Upper Body      │         │  Chassis/Base    │  │
│  │  - Jetson Orin × 2│◄──────►│  - IPC (ROS 2)  │  │
│  │  - Verdin iMX8M+ │         │  - Navigation    │  │
│  │  - Vision         │         │  - Sensors       │  │
│  └──────────────────┘         └──────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### Real-Time Control Layer
```
┌─────────────────────────────────────────────────────┐
│           CAN Bus (1 Mbps)                           │
│                                                       │
│  ┌──────────────────┐         ┌──────────────────┐  │
│  │  IPC / Upper     │         │  STM32H7 MCU     │  │
│  │  Body Control    │◄──────►│  - 1ms loop      │  │
│  │                  │         │  - Motor control │  │
│  │                  │         │  - Power mgmt    │  │
│  └──────────────────┘         └──────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### ROS 2 Topic Layer

**Primary Data Streams**:
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odom` | nav_msgs/Odometry | 50 Hz | EKF-fused odometry |
| `/scan` | sensor_msgs/LaserScan | 10 Hz | 3D LiDAR scan |
| `/imu/data` | sensor_msgs/Imu | 50-100 Hz | IMU data |
| `/camera/image` | sensor_msgs/Image | 30 Hz | Wide-angle camera |
| `/secondary_camera/image` | sensor_msgs/Image | 30 Hz | Secondary camera |
| `/fisheye_front/image` | sensor_msgs/Image | 30 Hz | Front fish-eye |
| `/fisheye_rear/image` | sensor_msgs/Image | 30 Hz | Rear fish-eye |
| `/cmd_vel` | geometry_msgs/Twist | varies | Velocity commands |

---

## 6. Development Status & Improvements

### Version 3.1 Status

**Completed**:
- ✅ Dual Jetson Orin integration
- ✅ 1Gbps Ethernet backbone
- ✅ 8 ultrasonic sensor array
- ✅ 3D LiDAR integration
- ✅ Multiple camera system (wide-angle, secondary, fish-eye)
- ✅ Emergency stop button integration

**In Progress (v3.71)**:
- 🔄 Battery accessibility redesign
- 🔄 Chassis size optimization
- 🔄 Module relocation evaluation
- 🔄 Touchscreen flush-mount integration
- 🔄 335LG sensor repositioning
- 🔄 Structural optimization for maintenance access

**Issues Identified**:
- ❌ Motor failure (front-right drive) - maintenance difficulty
- ❌ Battery position prevents user swapping
- ❌ Chassis exceeds original dimensions
- ❌ Internal space constraints in upper body

### Production Standardization

**Planned Improvements**:
1. **Wiring Standards**:
   - Cable trays for organization
   - Wire labeling system
   - Power/signal cable separation

2. **Environmental Protection**:
   - Conformal coating on exposed PCBs
   - Dust and moisture protection

3. **Interface Stability**:
   - Structural adhesive at cable interfaces
   - Strain relief for critical connections

---

## 7. Regulatory & Deployment

### Certification Status
- **CE Certification**: Not yet obtained
- **Current Shipping**: Separate parts to clear customs
- **Target**: Complete unit shipping after certification

---

## 8. Technical Notes

### Bandwidth Calculation
**Critical**: All bandwidth measurements use **Mb/s (Megabits per second)**
- 1 Gbps = 1000 Mb/s = 125 MB/s
- Configuration errors possible if using MB/s incorrectly

### Discovery & Network
- **ROS 2 Domain**: 0 (default)
- **Discovery Protocol**: SIMPLE (automatic participant discovery)
- **DDS Implementation**: FastDDS (rmw_fastrtps_cpp)
- **Discovery Range**: Configurable (LOCALHOST or SUBNET)

---

## Sensor Configuration Summary Table

| Sensor Type | FOV | Resolution | Rate | Location | Purpose |
|-------------|-----|------------|------|----------|---------|
| **Wide-Angle RGBD** | 190°H / 114°V | 1280x720 | 30Hz | Upper Body | Primary vision |
| **Secondary RGBD** | 86°H / 58°V | 640x480 | 30Hz | Upper Body | Manipulation |
| **Fish-eye (Front)** | 120°H | 1920x1080 | 30Hz | Front | Surround view |
| **Fish-eye (Rear)** | 120°H | 1920x1080 | 30Hz | Rear | Surround view |
| **335LG Depth** | 18°V | 640x480 | 30Hz | Height-specific | Precision ranging |
| **3D LiDAR** | 360°H / ±15°V | Point cloud | 10Hz | Base center | SLAM/Mapping |
| **Ultrasonic (×8)** | 75° each | Range | 30Hz | Base perimeter | Collision avoidance |
| **IMU** | N/A | 6-DOF | 50-100Hz | Chassis | Motion tracking |

---

*Last Updated: 2026-01-12*
*Architecture Version: 3.1*
*Next Revision: v3.71 (Structural optimization)*
