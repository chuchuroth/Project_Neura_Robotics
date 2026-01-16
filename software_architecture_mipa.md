







---
---


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

# CURRENT STATUS

# MiPA X Refactoring Plan

## Overview
Refactoring mipax codebase into MiPA X - a modular, high-performance mobile robot platform with real-time control and distributed autonomy.

## Package Renaming Strategy

### Core Packages Mapping

| Original Package | New Package | Description |
|-----------------|-------------|-------------|
| `mipax` | `mipax` | Meta-package for MiPA X |
| `mipax_base` | `mipax_base` | Base configurations, EKF filter, ros2_control HAL |
| `mipax_bringup` | `mipax_bringup` | Launch files for system startup |
| `mipax_description` | `mipax_description` | URDF/xacro robot models |
| `mipax_gazebo` | `mipax_gazebo` | Gazebo simulation integration |
| `mipax_navigation` | `mipax_navigation` | Nav2 integration and path planning |
| `mipax_neuraverse` | `mipax_neuraverse` | Neuraverse Skills integration |

### New Architecture Components

Additional packages to emphasize MiPA X architecture:

1. **mipax_control** (enhanced from mipax_base)
   - 1ms real-time chassis control loop
   - ros2_control hardware interface
   - CAN communication with STM32H7
   - Motor control abstractions

2. **mipax_perception** (new)
   - Sensor fusion for Jetson AGX Orin
   - Camera/LIDAR integration
   - Perception pipeline

3. **mipax_skills** (enhanced from mipax_neuraverse)
   - Neuraverse Skills integration
   - High-level behavior trees
   - Multi-robot coordination

## Directory Structure

```
mipax/
├── mipax/                          # Meta-package
├── mipax_base/                     # Base configurations
│   ├── config/
│   │   ├── ekf.yaml
│   │   └── control.yaml
│   └── include/mipax_base/
├── mipax_control/                  # Real-time control (1ms loop)
│   ├── include/mipax_control/
│   │   ├── hardware_interface.hpp
│   │   └── can_interface.hpp
│   └── src/
├── mipax_bringup/                  # Launch files
│   ├── launch/
│   └── config/
├── mipax_description/              # Robot models
│   ├── urdf/
│   ├── meshes/
│   └── launch/
├── mipax_gazebo/                   # Simulation
│   ├── worlds/
│   └── launch/
├── mipax_navigation/               # Nav2 integration
│   ├── config/
│   ├── maps/
│   └── launch/
├── mipax_perception/               # Perception (new)
│   ├── launch/
│   └── config/
└── mipax_skills/                   # Neuraverse Skills
    ├── launch/
    ├── config/
    └── skills/
```

## Key Changes

### 1. Hardware Architecture Updates
- Emphasize Jetson AGX Orin as main compute platform
- Highlight STM32H7 real-time control loop (1ms)
- CAN bus communication layer
- Modular hardware abstraction layer (HAL)

### 2. Control System Updates
- Update ros2_control configuration for 1ms loop
- CAN interface for STM32H7 communication
- Motor controller abstraction
- Real-time guarantees

### 3. Neuraverse Integration
- Replace upper-level logic with Neuraverse Skills
- Integration with NeuraSync dashboard
- Multi-robot coordination primitives

### 4. Documentation Updates
- New README emphasizing MiPA X architecture
- JetPack SDK integration notes
- Nav2 distributed architecture
- Multi-robot capabilities

## Refactoring Steps

### Phase 1: Package Renaming (Automated)
1. Rename all package directories
2. Update package.xml files
3. Update CMakeLists.txt files
4. Clean build/install directories

### Phase 2: Content Updates (Automated)
1. Update all source file includes
2. Update launch file references
3. Update URDF/xacro namespace references
4. Update configuration files

### Phase 3: Architecture Enhancement (Manual)
1. Add mipax_control package with STM32H7 interface
2. Add mipax_perception package
3. Enhance mipax_skills with Neuraverse integration
4. Update hardware_interface for 1ms control loop

### Phase 4: Documentation (Automated)
1. Create comprehensive MiPA X README
2. Update architecture documentation
3. Create integration guides
4. Update NeuraSync dashboard integration

### Phase 5: Testing & Validation
1. Rebuild workspace
2. Test launch files
3. Verify simulation
4. Test NeuraSync integration

## File Pattern Replacements

### Global Search & Replace Patterns

| Pattern | Replacement | Scope |
|---------|------------|-------|
| `mipax` | `mipax` | All files (case-sensitive) |
| `mipax` | `MIPAX` | Environment variables, macros |
| `mipax` | `MipaX` | Documentation, titles |

### Preserve These Patterns
- Git history references to mipax
- External dependency references
- Original author attributions

## Rollback Strategy

Before refactoring:
1. Create git branch: `git checkout -b mipax-refactor`
2. Tag current state: `git tag pre-mipax-refactor`
3. Backup build/install: `tar -czf mipax-backup.tar.gz build install`

## Success Criteria

- [ ] All packages renamed successfully
- [ ] Workspace builds without errors
- [ ] All launch files functional
- [ ] Simulation runs correctly
- [ ] NeuraSync dashboard connects
- [ ] Documentation updated
- [ ] No broken references in code

## Timeline

- Phase 1: 30 minutes (automated)
- Phase 2: 1 hour (automated)
- Phase 3: 2-4 hours (manual enhancement)
- Phase 4: 1 hour (documentation)
- Phase 5: 1 hour (testing)

**Total: ~5-7 hours**

## Notes

- Preserve all ros2_control and hardware_interface foundations
- Maintain backward compatibility where possible
- Document breaking changes
- Update NeuraSync integration scripts
- Consider migration path for existing users

---

**Status**: Ready to execute
**Created**: 2026-01-12
**Target Platform**: Jetson AGX Orin + STM32H7

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


---
---

The architecture is centered around high-compute modules for vision and AI, interconnected via a high-speed network.

---

## 1. Core Compute & Vision Processing

The system utilizes a dual-processor approach for high-level tasks:

* 
**Dual NVIDIA Jetson Orin Modules:** These serve as the primary "brains" for the robot. They handle intensive tasks such as:


* 
**Vision:** Processing data from RGBD cameras with specific fields of view (e.g.,  Horizontal FOV).


* 
**Streaming:** Handling video feeds at 720p resolution (approx. 1280x720) at 30FPS.




* 
**Audio Processing:** Includes a dedicated subsystem for audio streaming, microphone arrays, and echo cancellation.



## 2. Structural Segmentation

The diagram explicitly divides the hardware into two main zones:

* 
**Upper Body (上半身):** Contains the Jetson Orin modules, the **Verdin iMX8M Plus** (an ARM-based System-on-Module), and a power distribution board. It also interfaces with a **Robotic Arm**.


* 
**Chassis/Base (底盘):** Focuses on mobility, safety, and power. It houses:


* An **Industrial PC (IPC) running ROS** (Robot Operating System).


* A **48V Battery** system.


* Safety components like an **Emergency Stop Button** (急停按钮) and physical switches.





## 3. Sensors and Connectivity

* 
**Navigation Sensors:** The chassis includes an **IMU** (Inertial Measurement Unit), **Ultrasonic Sensors** positioned on all four sides (front, rear, left, right) for obstacle detection, and **Indicator Lights**.


* 
**Networking:** The system uses a **1Gbps (1000 Mbps) Ethernet** backbone. There is a specific technical note clarifying that bandwidth units are in **Mb/s (megabits)** rather than MB/s (megabytes) to avoid configuration errors.


* 
**Wireless:** Support for Dual-band Wi-Fi and Bluetooth 5.0 is integrated for external communication.



## 4. Observations and Design Notes

* 
**Development Maturity:** The document includes "Closed Comments" and "Future Improvement" sections, suggesting this is an evolving design (Version 3.1).


* 
**Integration Suggestion:** One specific comment (source 34) suggests that placing a certain component "directly in the chassis might be more appropriate," indicating ongoing optimization of weight distribution or cable management.



---
---
Specific details regarding the identification (ID) and Field of View (FOV) of the robotic platform's sensory systems.

## 1. Sensory Field of View (FOV) Specifications

The document details various FOV parameters for the robot's high-resolution cameras:

* **Horizontal FOV (HFOV):** The platform features a wide-angle sensing capability with a **190°** horizontal field of view.
* **Vertical FOV (VFOV):** The vertical coverage is specified at **114°**.
* **Secondary Sensor FOV:** A separate RGBD camera configuration is noted with an HFOV of **86°** and a VFOV of **58°**.

## 2. Project and Component ID Details

The documentation lists specific identifiers for the hardware and data streams:

* **Processing Units:** The architecture confirms the use of **NVIDIA Jetson Orin** modules as the primary compute hubs.
* **Data Stream ID:** A specific stream is identified as "720P" with a resolution of **1280x720**.
* **Control Boards:** The system integrates a **Verdin iMX8M Plus** and a dedicated **MCU + Power Board** for low-level control.

## 3. Communication and Network ID

* **Bandwidth Note:** There is a clear distinction made for the network ID, specifying that the data rate is measured in **Mb/s**, not MB/s, to ensure technical accuracy during system configuration.
* **Interface ID:** The primary network backbone is identified as a **1Gbps / 1000 Mbps** Ethernet interface.

---
---
here is the mapping of the sensory systems and their specific FOV (Field of View) parameters within the robot's physical architecture.

## Sensory Mapping & Specifications

| Sensor Category | Component / ID | Placement | Field of View (FOV) |
| --- | --- | --- | --- |
| **Wide-Angle Vision** | RGBD Camera 

 | Upper Body 

 | <br>**190° Horizontal** / **114° Vertical** 

 |
| **Secondary Vision** | RGBD Camera 

 | Upper Body 

 | <br>**86° Horizontal** / **58° Vertical** 

 |
| **Obstacle Detection** | Ultrasonic Sensors 

 | Chassis (Front, Rear, Left, Right) 

 | Not specified (Proximity based) 

 |
| **Motion Tracking** | IMU 

 | Chassis 

 | N/A (Internal orientation) 

 |

---

## Detailed Component Analysis

### Upper Body Vision Hub

* 
**Primary Processing:** The **NVIDIA Jetson Orin** modules process high-resolution video streams (720P / 1280x720).


* 
**Camera Integration:** The wide-angle cameras (190° HFOV) are connected via an **Adapter Board** to the Orin modules to facilitate complex spatial awareness.


* 
**Manipulator Feedback:** A **Robotic Arm** is integrated into this section, likely utilizing the 86° FOV cameras for localized tasks.



### Chassis Navigation & Safety

* 
**Spatial Awareness:** Four **Ultrasonic Sensors** provide 360-degree proximity coverage around the base.


* 
**System Control:** An **Industrial PC (IPC)** running **ROS** serves as the central node for the chassis, coordinating the IMU and sensor data.


* 
**Safety Interlocks:** The **Emergency Stop (急停按钮)** and **48V Battery** management are physically housed here to ensure low-level hardware safety.



### Network Infrastructure

* 
**Bandwidth Capacity:** The system utilizes a **1Gbps (1000 Mbps)** Ethernet backbone.


* 
**Data Accuracy Note:** Technical documentation emphasizes that bandwidth measurements are calculated in **Mb/s** (Megabits per second) to prevent confusion with Megabytes (MB/s).


---
---
here is the mapping of the platform's connectivity and sensory systems.

### 1. Data & Compute Connectivity

The system uses a hierarchical communication structure to link the high-level "brains" in the upper body with the low-level controllers in the chassis.

* 
**High-Speed Backbone:** A **1Gbps (1000 Mbps)** Ethernet network serves as the primary data highway between the upper body compute modules and the chassis.


* 
**Upper Body Hub:** * Dual **NVIDIA Jetson Orin** modules handle heavy vision processing (720P streams).


* A **Verdin iMX8M Plus** System-on-Module (SoM) acts as a bridge, likely managing local control and interfacing with the **Robotic Arm**.




* 
**Chassis Integration:** * An **Industrial PC (IPC)** running **ROS** (Robot Operating System) manages base navigation.


* A dedicated **MCU (Microcontroller)** handles power distribution and low-level hardware like the **IMU** and **Indicator Lights**.


* 
**Wireless:** The platform includes **Dual-band Wi-Fi** and **Bluetooth 5.0** for external connectivity.





---

### 2. Sensory FOV & Placement Mapping

The "X Project" utilizes a multi-layered sensor suite for 360° awareness and precise manipulation.

| Sensor Type | Field of View (FOV) | Primary Function | Placement |
| --- | --- | --- | --- |
| **Fish-eye Camera** | <br>**120° HFOV** 

 | Wide-area surround view | Front & Rear 

 |
| **Depth Camera (Base)** | <br>**86° HFOV / 58° VFOV** 

 | Obstacle & ground detection | Base/Chassis 

 |
| **335LG Sensor** | <br>**18° VFOV** 

 | Precision depth/ranging | Height-specific mounting 

 |
| **3D LiDAR** | <br>**360° Horizontal** 

 | SLAM and mapping | Center-mounted on base 

 |
| **Ultrasonic** | <br>**75° Sensing Angle** 

 | Close-range collision avoidance | 8 sensors around the base 

 |

### 3. Physical Layout & Safety

* 
**Linear Elevation:** The **Robotic Arm** is mounted on a linear axis with a **600mm** lift range.


* 
**Safety Interlocks:** A physical **Emergency Stop Button** is integrated into the chassis, noted as being moved downward in the V3.1 revision due to internal space constraints.


* 
**User Interface:** The robot features a **13.3-inch touchscreen** for direct human interaction.


* 
**Power:** The entire platform is powered by a **48V Battery** system , with a dedicated display on the body to check internal battery data.

---
---

# MiPA-X Sensor Configuration Guide

**Version**: 3.1
**Last Updated**: 2026-01-12
**Related**: [HARDWARE_ARCHITECTURE.md](HARDWARE_ARCHITECTURE.md), [mipax_sensory_system_mapping.md](mipax_sensory_system_mapping.md)

---

## Overview

This document provides a comprehensive mapping of all sensors in the MiPA-X robotic platform, including their Field of View (FOV) specifications, physical placement, ROS 2 topic mappings, and configuration file locations.

---

## Sensor Inventory

### Vision Sensors Summary

| Sensor ID | Type | FOV | Resolution | Rate | Location | URDF File |
|-----------|------|-----|------------|------|----------|-----------|
| `camera` | Wide-Angle RGBD | 190°H / 114°V | 1280x720 | 30Hz | Upper Body | [depth_sensor.urdf.xacro](mipax/mipax_description/urdf/sensors/depth_sensor.urdf.xacro) |
| `secondary_camera` | Secondary RGBD | 86°H / 58°V | 640x480 | 30Hz | Upper Body | [secondary_depth_sensor.urdf.xacro](mipax/mipax_description/urdf/sensors/secondary_depth_sensor.urdf.xacro) |
| `fisheye_front` | Fish-eye Camera | 120°H | 1920x1080 | 30Hz | Front | [fisheye_camera.urdf.xacro](mipax/mipax_description/urdf/sensors/fisheye_camera.urdf.xacro) |
| `fisheye_rear` | Fish-eye Camera | 120°H | 1920x1080 | 30Hz | Rear | [fisheye_camera.urdf.xacro](mipax/mipax_description/urdf/sensors/fisheye_camera.urdf.xacro) |
| `sensor_335lg` | Precision Depth | 18°V / 30°H | 640x480 | 30Hz | Height-specific | [sensor_335lg.urdf.xacro](mipax/mipax_description/urdf/sensors/sensor_335lg.urdf.xacro) |

### Navigation Sensors Summary

| Sensor ID | Type | FOV/Range | Update Rate | Location | URDF File |
|-----------|------|-----------|-------------|----------|-----------|
| `lidar_3d` | 3D LiDAR | 360°H / ±15°V | 10Hz | Base Center | [lidar_3d.urdf.xacro](mipax/mipax_description/urdf/sensors/lidar_3d.urdf.xacro) |
| `laser` | 2D LiDAR | 360° / 0.21-5.5m | 10Hz | Base | [laser.urdf.xacro](mipax/mipax_description/urdf/sensors/laser.urdf.xacro) |
| `imu` | IMU (6-DOF) | N/A | 50-100Hz | Chassis | [imu.urdf.xacro](mipax/mipax_description/urdf/sensors/imu.urdf.xacro) |
| `ultrasonic_array` | 8× Ultrasonic | 75° each / 0.02-4.0m | 30Hz | Base Perimeter | [ultrasonic_array.urdf.xacro](mipax/mipax_description/urdf/sensors/ultrasonic_array.urdf.xacro) |

---

## Detailed Sensor Specifications

### 1. Wide-Angle RGBD Camera

**Sensor ID**: `camera`
**Type**: RGBD (Color + Depth)
**Location**: Upper Body - Primary Vision

#### Field of View
- **Horizontal FOV**: 190° (3.316 radians)
- **Vertical FOV**: 114° (1.989 radians)
- **Purpose**: Primary spatial awareness and environment understanding

#### Technical Specifications
- **Resolution**: 1280 × 720 (720p)
- **Frame Rate**: 30 Hz
- **Depth Range**: 0.3m to 100m
- **Format**: R8G8B8

#### ROS 2 Topics
- `/camera/image`: RGB image
- `/camera/depth/image_raw`: Raw depth image
- `/camera/depth/image_rect_raw`: Rectified depth image
- `/camera/camera_info`: Camera calibration info

#### Configuration Files
- **URDF**: [mipax/mipax_description/urdf/sensors/depth_sensor.urdf.xacro](mipax/mipax_description/urdf/sensors/depth_sensor.urdf.xacro)
- **Launch**: [mipax/mipax_bringup/launch/depth.launch.py](mipax/mipax_bringup/launch/depth.launch.py)

---

### 2. Secondary RGBD Camera

**Sensor ID**: `secondary_camera`
**Type**: RGBD (Color + Depth)
**Location**: Upper Body - Manipulation Zone

#### Field of View
- **Horizontal FOV**: 86° (1.501 radians)
- **Vertical FOV**: 58° (1.012 radians)
- **Purpose**: Localized manipulation and robotic arm feedback

#### Technical Specifications
- **Resolution**: 640 × 480 (VGA)
- **Frame Rate**: 30 Hz
- **Depth Range**: 0.3m to 100m
- **Format**: R8G8B8
- **Mass**: 0.120 kg

#### ROS 2 Topics
- `/secondary_camera/image`: RGB image
- `/secondary_camera/depth/image_raw`: Raw depth image
- `/secondary_camera/camera_info`: Camera calibration info

#### Configuration Files
- **URDF**: [mipax/mipax_description/urdf/sensors/secondary_depth_sensor.urdf.xacro](mipax/mipax_description/urdf/sensors/secondary_depth_sensor.urdf.xacro)

---

### 3. Fish-eye Cameras (Front & Rear)

**Sensor IDs**: `fisheye_front`, `fisheye_rear`
**Type**: Wide-angle Fish-eye Camera
**Location**: Front & Rear of Upper Body

#### Field of View
- **Horizontal FOV**: 120° (2.094 radians)
- **Purpose**: Wide-area surround view for situational awareness

#### Technical Specifications
- **Resolution**: 1920 × 1080 (Full HD)
- **Frame Rate**: 30 Hz
- **Range**: 0.1m to 50m
- **Format**: R8G8B8
- **Distortion**: Fish-eye lens model (k1=-0.15, k2=0.05)
- **Mass**: 0.050 kg per camera

#### ROS 2 Topics
- `/fisheye_front/image`: Front camera RGB image
- `/fisheye_rear/image`: Rear camera RGB image
- `/fisheye_front/camera_info`: Front camera info
- `/fisheye_rear/camera_info`: Rear camera info

#### Configuration Files
- **URDF**: [mipax/mipax_description/urdf/sensors/fisheye_camera.urdf.xacro](mipax/mipax_description/urdf/sensors/fisheye_camera.urdf.xacro)

---

### 4. 335LG Precision Depth Sensor

**Sensor ID**: `sensor_335lg`
**Type**: Precision Depth Sensor
**Location**: Height-specific mounting (moved downward in v3.1)

#### Field of View
- **Vertical FOV**: 18° (0.314 radians)
- **Horizontal FOV**: 30° (0.523 radians)
- **Purpose**: Precision depth/ranging for specific tasks

#### Technical Specifications
- **Resolution**: 640 × 480
- **Frame Rate**: 30 Hz
- **Range**: 0.2m to 10.0m
- **Format**: R8G8B8
- **Mass**: 0.040 kg

#### ROS 2 Topics
- `/sensor_335lg/depth`: Depth image
- `/sensor_335lg/camera_info`: Sensor info

#### Configuration Files
- **URDF**: [mipax/mipax_description/urdf/sensors/sensor_335lg.urdf.xacro](mipax/mipax_description/urdf/sensors/sensor_335lg.urdf.xacro)

---

### 5. 3D LiDAR

**Sensor ID**: `lidar_3d`
**Type**: 3D Scanning LiDAR
**Location**: Center-mounted on Base

#### Field of View
- **Horizontal FOV**: 360° (full rotation)
- **Vertical FOV**: ±15° (16-32 layers)
- **Purpose**: SLAM and environmental mapping

#### Technical Specifications
- **Horizontal Samples**: 1800 points per rotation
- **Vertical Layers**: 16 (configurable to 32)
- **Update Rate**: 10 Hz
- **Range**: 0.3m to 100m
- **Resolution**: 0.01m
- **Mass**: 0.830 kg

#### ROS 2 Topics
- `/lidar_3d/points`: Point cloud data (sensor_msgs/PointCloud2)
- `/lidar_3d/scan`: Converted 2D scan if needed

#### Configuration Files
- **URDF**: [mipax/mipax_description/urdf/sensors/lidar_3d.urdf.xacro](mipax/mipax_description/urdf/sensors/lidar_3d.urdf.xacro)

---

### 6. 2D LiDAR (Legacy/Simulation)

**Sensor ID**: `laser`
**Type**: 2D Laser Scanner
**Location**: Base

#### Field of View
- **Horizontal FOV**: 360° (-π to +π)
- **Purpose**: 2D obstacle detection and navigation

#### Technical Specifications
- **Ray Count**: 360
- **Update Rate**: 10 Hz
- **Range**: 0.21m to 5.5m (base) / 0.08m to 12.0m (generic)
- **Angular Resolution**: 1° per sample

#### ROS 2 Topics
- `/scan`: Laser scan data (sensor_msgs/LaserScan)
- `/base/scan`: Base-specific scan

#### Configuration Files
- **URDF**: [mipax/mipax_description/urdf/sensors/laser.urdf.xacro](mipax/mipax_description/urdf/sensors/laser.urdf.xacro)
- **Launch**: [mipax/mipax_bringup/launch/lasers.launch.py](mipax/mipax_bringup/launch/lasers.launch.py)

#### Supported Hardware
- YDLIDAR, LD06, LD19, STL27L
- RPLiDAR (A1, A2, A3, C1, S1, S2, S3)
- XV11

---

### 7. IMU (Inertial Measurement Unit)

**Sensor ID**: `imu`
**Type**: 6-DOF IMU
**Location**: Chassis

#### Specifications
- **Data**: Acceleration (m/s²), Angular Velocity (rad/s), Orientation (quaternion)
- **Update Rate**: 50-100 Hz
- **Purpose**: Motion tracking and sensor fusion

#### ROS 2 Topics
- `/imu/data`: Filtered IMU data (sensor_msgs/Imu)
- `/imu/data_raw`: Raw IMU data (sensor_msgs/Imu)
- `/imu/mag`: Magnetometer data (sensor_msgs/MagneticField)

#### Configuration Files
- **URDF**: [mipax/mipax_description/urdf/sensors/imu.urdf.xacro](mipax/mipax_description/urdf/sensors/imu.urdf.xacro)
- **EKF Config**: [mipax/mipax_base/config/ekf.yaml](mipax/mipax_base/config/ekf.yaml)

#### Integration
- **EKF Fusion**: 50 Hz filter frequency
- **Input**: IMU yaw angular velocity + wheel odometry
- **Output**: `/odom` (fused odometry)

---

### 8. Ultrasonic Sensor Array

**Sensor Array ID**: `ultrasonic_array`
**Type**: 8× Ultrasonic Proximity Sensors
**Location**: Base Perimeter (360° coverage)

#### Placement
1. `ultrasonic_front_left` - Front-Left
2. `ultrasonic_front_right` - Front-Right
3. `ultrasonic_right_front` - Right-Front
4. `ultrasonic_right_rear` - Right-Rear
5. `ultrasonic_rear_right` - Rear-Right
6. `ultrasonic_rear_left` - Rear-Left
7. `ultrasonic_left_rear` - Left-Rear
8. `ultrasonic_left_front` - Left-Front

#### Specifications
- **Sensing Angle**: 75° per sensor
- **Range**: 0.02m to 4.0m
- **Update Rate**: 30 Hz per sensor
- **Purpose**: Close-range collision avoidance
- **Mass**: 0.010 kg per sensor

#### ROS 2 Topics
Each sensor publishes to its own topic:
- `/ultrasonic_front_left`: Range data
- `/ultrasonic_front_right`: Range data
- `/ultrasonic_right_front`: Range data
- ... (8 topics total)

#### Configuration Files
- **URDF**: [mipax/mipax_description/urdf/sensors/ultrasonic_array.urdf.xacro](mipax/mipax_description/urdf/sensors/ultrasonic_array.urdf.xacro)

---

## Sensor Integration Architecture

### Upper Body Vision Hub
```
┌─────────────────────────────────────────────────┐
│          Dual NVIDIA Jetson Orin                │
│                                                  │
│  ┌──────────────┐  ┌──────────────┐            │
│  │ Wide-Angle   │  │ Secondary    │            │
│  │ RGBD Camera  │  │ RGBD Camera  │            │
│  │ 190°H/114°V  │  │ 86°H/58°V    │            │
│  └──────────────┘  └──────────────┘            │
│                                                  │
│  ┌──────────────┐  ┌──────────────┐            │
│  │ Fish-eye     │  │ Fish-eye     │            │
│  │ Front 120°   │  │ Rear 120°    │            │
│  └──────────────┘  └──────────────┘            │
│                                                  │
│  ┌──────────────┐                               │
│  │ 335LG Sensor │                               │
│  │ 18°V Precision│                              │
│  └──────────────┘                               │
└─────────────────────────────────────────────────┘
```

### Chassis Navigation & Safety
```
┌─────────────────────────────────────────────────┐
│         Industrial PC (ROS 2)                    │
│                                                  │
│  ┌──────────────┐  ┌──────────────┐            │
│  │ 3D LiDAR     │  │ IMU          │            │
│  │ 360°H/±15°V  │  │ 6-DOF        │            │
│  └──────────────┘  └──────────────┘            │
│                                                  │
│  ┌─────────────────────────────────────────┐   │
│  │   Ultrasonic Array (8 sensors)          │   │
│  │   75° each, 360° coverage               │   │
│  └─────────────────────────────────────────┘   │
└─────────────────────────────────────────────────┘
```

---

## Supported Camera Drivers

### Depth Camera Support
Configured in [mipax/mipax_bringup/launch/depth.launch.py](mipax/mipax_bringup/launch/depth.launch.py):

- **Intel RealSense**: D435, D435i
- **Stereolabs ZED**: ZED, ZED2, ZED2i, ZED Mini
- **OAK-D**: OAK-D, OAK-D-LITE, OAK-D-PRO

### ZED Camera Configuration
File: [mipax/mipax_bringup/config/zed_common.yaml](mipax/mipax_bringup/config/zed_common.yaml)

- **Resolution**: 2 (HD720 - 1280x720)
- **Grab Frame Rate**: 30 Hz
- **Publish Frame Rate**: 15 Hz
- **Depth Quality**: PERFORMANCE mode
- **Confidence Threshold**: 50
- **Point Cloud Rate**: 10 Hz

---

## Sensor Frame Hierarchy

```
base_link
├── camera_link (Wide-angle RGBD)
│   └── camera_depth_link
├── secondary_camera_link (Secondary RGBD)
│   └── secondary_camera_depth_link
├── fisheye_front_link
│   └── fisheye_front_optical_link
├── fisheye_rear_link
│   └── fisheye_rear_optical_link
├── sensor_335lg_link
│   └── sensor_335lg_optical_link
├── lidar_3d_link
├── laser_link (2D LiDAR)
├── imu_link
├── ultrasonic_front_left_link
├── ultrasonic_front_right_link
├── ultrasonic_right_front_link
├── ultrasonic_right_rear_link
├── ultrasonic_rear_right_link
├── ultrasonic_rear_left_link
├── ultrasonic_left_rear_link
└── ultrasonic_left_front_link
```

---

## Performance Specifications

### Expected Data Rates

| Sensor | Update Rate | Bandwidth (Est.) |
|--------|-------------|------------------|
| Wide-Angle RGBD | 30 Hz | ~55 MB/s |
| Secondary RGBD | 30 Hz | ~18 MB/s |
| Fish-eye Front | 30 Hz | ~60 MB/s |
| Fish-eye Rear | 30 Hz | ~60 MB/s |
| 335LG | 30 Hz | ~18 MB/s |
| 3D LiDAR | 10 Hz | ~5 MB/s |
| 2D LiDAR | 10 Hz | ~50 KB/s |
| IMU | 100 Hz | ~15 KB/s |
| Ultrasonic (×8) | 30 Hz | ~2 KB/s |
| **Total** | - | **~216 MB/s** |

### Network Capacity
- **Ethernet Backbone**: 1 Gbps (125 MB/s)
- **Utilization**: ~173% (requires data compression or selective streaming)

---

## Configuration References

### Key Configuration Files

| Component | File | Purpose |
|-----------|------|---------|
| **Sensor URDF** | [mipax/mipax_description/urdf/sensors/](mipax/mipax_description/urdf/sensors/) | Robot sensor models |
| **Depth Launch** | [mipax/mipax_bringup/launch/depth.launch.py](mipax/mipax_bringup/launch/depth.launch.py) | Depth camera launch |
| **Laser Launch** | [mipax/mipax_bringup/launch/lasers.launch.py](mipax/mipax_bringup/launch/lasers.launch.py) | LiDAR launch |
| **ZED Config** | [mipax/mipax_bringup/config/zed_common.yaml](mipax/mipax_bringup/config/zed_common.yaml) | ZED camera params |
| **EKF Config** | [mipax/mipax_base/config/ekf.yaml](mipax/mipax_base/config/ekf.yaml) | IMU fusion config |

---

## Related Documentation

- [HARDWARE_ARCHITECTURE.md](HARDWARE_ARCHITECTURE.md) - Complete hardware architecture
- [mipax_architecture_diagram.md](mipax_architecture_diagram.md) - Architecture diagram notes
- [mipax_sensory_system_mapping.md](mipax_sensory_system_mapping.md) - Original sensor mapping spec
- [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) - ROS 2 + Dashboard integration

---

*Last Updated: 2026-01-12*
*Configuration Version: 3.1*
*Sensor Count: 15 sensors (5 vision, 1 3D LiDAR, 1 2D LiDAR, 1 IMU, 8 ultrasonic)*

---
---
Based on the "Product Improvement List," the "X Project" robotic platform is undergoing significant updates to its physical structure, production processes, and core hardware integration to improve serviceability and performance.

The following is a summary of the key improvement areas and current issues:

### 1. Structural & Design Improvements

A major focus is on optimizing the robot's physical layout for better usability and maintenance:

* 
**Battery Accessibility**: The current battery position is difficult to access, preventing users from swapping it autonomously; a design optimization is planned for version v3.71.


* **Chassis Size Optimization**: The current chassis exceeds the original design dimensions and needs to be reduced.
* **Module Relocation**: There is a high-priority request to move primary compute modules like the **NVIDIA Jetson Orin** into the chassis, though this may conflict with chassis size reduction goals.
* 
**Aesthetic & Safety Refinement**: The 13.3-inch touchscreen needs to be made flush with the housing , and the **Emergency Stop Button** and certain sensors (335LG) are being moved downward due to internal space constraints.



### 2. Production & Electrical Standardization

To ensure consistent quality across units, several process improvements are being implemented:

* 
**Wiring Standards**: Plans are in place to use cable trays, label all wires, and separate power from signal cables to prevent interference.


* 
**Environmental Protection**: Conformal coating will be applied to exposed PCBs to protect against dust and moisture.


* 
**Interface Stability**: Structural adhesive or silicone will be used at cable interfaces to prevent damage from stress or bending.



### 3. Robotic Arm & Payload

The platform's dual robotic arms are being evaluated for workspace and performance gains:

* 
**Workspace Optimization**: Engineers are looking into increasing the joint angles (specifically joints J2, J4, and J6) to improve the elbow and wrist workspace, though this depends on the final housing design.


* 
**Payload Limitations**: While there was a request to increase the payload to ~7.5kg and the arm length to 900mm, current assessments indicate these specific targets are not achievable with the current hardware.



### 4. Critical Hardware Issues

Recent field reports have identified urgent hardware failures:

* **Motor Failure**: A front-right drive motor recently burnt out, leading to a temporary loss of base mobility.
* **Maintenance Difficulty**: The current base structure makes it extremely difficult to perform maintenance when such failures occur, necessitating a structural optimization in version v3.71.

### 5. Regulatory & Shipping

* **Certification**: The robot currently lacks **CE certification**, which requires it to be shipped in separate parts rather than as a complete unit to clear customs.
