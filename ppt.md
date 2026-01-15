周一的presentation基于此文：



 导航
 灵巧手
 视觉AI
 语言AI
 传感器
 执行器
 
 
按摩机器人：灵巧手另外单卖，稍贵

基础功能：机器人助手，语音聊天，端盘子，图像识别物体，导航，搬东西

高级功能：手，触感，做几款精确度不一样的灵巧手，根据实际需要做选择。


 make it work
 make it right
 make it fast



---
list all the options , trade offs.
我不做决定，我只提供选择和建议，并把各个决定的优缺点一一列出。

和neurasnyc dashboard一起测试，显示数据流量

if it's difficult to make decision, we make many prototype, it's easier to make choices with prototype. in real life, you go shopping, you thought you want something instead you bought something else. also market will tell the truth.


软件架构，把简单的功能做到极致，AI与传感器融合，互相抵消对方的弱点，如果”思考能力”不足，可以用hardcode代替某些智能效果，比如预设30个常用动作，50个标准程序，让机器人看上去很聪明，宇树也是提前预设的动作，这些动作不需要穷尽所有scenario，当然也无法穷尽生活中所有scenario，一些边缘case，cover不到也没关系，很少发生这种情况，只需要守住安全底线，动作不那么精确也无妨。

AI可以有，但不应该主要依赖于AI，嵌入式的发展已经很成熟了。什么时候AI介入，用户可以自行解除AI模式（比如当效率大大受限时）。。


* 如果完全按照嵌入式的思路做机器人，怎样的模式
* 如果完全按照端到端技术做机器人，又该怎样的diagram
* 如果两者融合，怎样的才合理

工业设计，参照汽车工业，设计一款大众喜爱的产品，先入为主，以后的家用服务型机器人就该长这样，不大不小。另外根据各个年龄各个群体的喜好设计不同风格造型，内核不需要改变，换壳即为新品。

家用机器人，外形必须可爱，看起来憨憨的，胖乎乎的，有信任感，看上去安全可靠，让人想要亲近。为什么人们喜欢狗，心理原因。

用户自定义，有参与感，robot personality: 80% embedded + 20% AI= 理工男. 50 + 50 = 普通人，有做事能力，也会犯错，20 + 80 = 创造力强，但会犯错 


尽快launch，尽快get market feedback，迭代一旦开始，就可以商业运作起来。

---

## MiPA X

**MiPA X** is a refactored and extended evolution of the **linorobot2** codebase, designed as a modular, high-performance mobile robot platform with real-time control and distributed autonomy.

The system is built around a **Jetson AGX Orin** running the JetPack SDK, responsible for perception, multi-robot navigation, and global path planning using **ROS 2 Nav2** with a fully integrated navigation stack deployed in a distributed architecture.

Low-level motion control is handled by a **layered MCU architecture** based on **ros2_control**, featuring a **1 ms real-time chassis control loop**. The control loop communicates over **CAN** with an **STM32H7**, ensuring deterministic and high-frequency motor control.

MiPA X adopts a **modular hardware abstraction layer (HAL)**, enabling flexible hardware integration and long-term maintainability. The platform integrates with **NeuraSync** and **Neuraverse**, and replaces the original linorobot2 upper-level logic with **Neuraverse Skills**, while preserving the proven `ros2_control` and `hardware_interface` foundations.

---

MiPA-X Minimum Viable Software Architecture
Core Design Principles
Centralized Perception on NVIDIA Orin - All sensor data (cameras, LiDAR, audio) flows to the dual Jetson Orin modules for unified AI processing

Safety-First Hierarchy:

P0: Hardware E-stop (bypasses all software, <10ms)
P1: Collision detection (ultrasonic + LiDAR, <100ms)
P2: Software limits (velocity, torque, workspace)
P3: Behavior limits (no-go zones, speed restrictions)
P4: Watchdog timeouts (500ms default)
Lightweight AI for MVP - Functional but imperfect, designed for improvement

Capability Mapping
Capability	Hardware	Software	AI Model
SEE	RGBD cameras, LiDAR	Vision pipeline	YOLOv8-nano
WALK	Base motors	Nav2 + SLAM	-
TOUCH	Dual 7-DOF arms	mipax_arms	-
FEEL	Ultrasonic, torque sensors	Safety monitor	-
HEAR	Microphone array	Audio pipeline	Whisper-tiny
TALK	Speakers	TTS	Piper
NAVIGATE	LiDAR, IMU	slam_toolbox	-
THINK	Orin GPU	State machine	Phi-3-mini (3.8B)
UNDERSTAND	Cameras	Scene graph	LLaVA-Phi-3
Compute Distribution

ORIN #1 (Perception)          ORIN #2 (Cognition)
├── Vision pipeline           ├── Lightweight LLM
├── Audio pipeline            ├── Task planning
├── Sensor fusion             ├── Dialog manager
└── Scene understanding       └── Behavior FSM

         │                            │
         └──────── 1Gbps ─────────────┘
                    │
            CHASSIS IPC (Navigation)
            ├── Nav2 + SLAM
            ├── Safety monitor
            └── Motion control
Minimum Package Structure

mipax/
├── mipax_perception/     # Vision, audio, sensor fusion (Orin #1)
├── mipax_cognition/      # LLM, planning, behaviors (Orin #2)
├── mipax_navigation/     # SLAM, Nav2 (IPC)
├── mipax_control/        # Safety, base control (IPC)
├── mipax_arms/           # Dual-arm control (existing)
├── mipax_speech/         # ASR, TTS, wake word
├── mipax_interface/      # Touchscreen UI, LEDs
└── mipax_bringup/        # System launch
AI Model Stack (Lightweight for MVP)
Function	Model	Size	Latency
Object Detection	YOLOv8-nano	3MB	<50ms
Person Detection	YOLOv8-nano	3MB	<50ms
ASR	Whisper-tiny	39MB	<2s
TTS	Piper-medium	60MB	<500ms
LLM	Phi-3-mini (4-bit)	2GB	<3s
VLM	LLaVA-Phi-3 (4-bit)	3GB	<5s
Development Roadmap
Phase 1 (8 weeks): MVP

Safety foundation
Basic navigation
Perception pipeline
Voice interaction
Phase 2 (8 weeks): Service Capability

LLM integration
Arm control for grasping
Full system integration
Phase 3: Future Improvements

Larger AI models
Advanced manipulation
Learning/adaptation
Key Takeaway
This architecture prioritizes safety over capability - the robot will be conservative and may seem "dumb" initially, but it will never harm anyone. As confidence builds and AI models improve, capabilities can be incrementally enhanced without redesigning the core architecture.

The document is now in MiPA-X_architecture.md and pushed to GitLab.
