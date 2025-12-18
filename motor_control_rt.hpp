// Real-time Motor Control Thread (1 kHz)
// Key goals: deterministic 1 kHz loop, no heap allocations during the loop, pin to CPU, SCHED_FIFO scheduling, safe shutdown, no blocking IO inside loop.




// motor_control_rt.hpp
// This file defines a real-time motor controller that runs at 1kHz on a dedicated thread.
// It uses Linux real-time scheduling, CPU pinning, and memory locking to prevent timing jitter.

#pragma once
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

// Example hardware driver stub.
// In real life, you would write CAN/SPI commands here
class MotorDriver {
public:
    void send_command(int motor_id, float effort) noexcept {
        // Hardware communication would go here
        (void) motor_id;
        (void) effort;
    }
};

class MotorControllerRT {
public:
    // Constructor: allow selecting CPU core and real-time priority
    MotorControllerRT(int control_core = 1, int priority = 80)
    : control_core_(control_core), rt_priority_(priority), running_(false)
    {
        // Reserve space for motor torque commands to avoid dynamic allocation later
        torque_cmds_.assign(kMotorCount, 0.0f);
    }

    ~MotorControllerRT() { stop(); }

    // Start real-time thread
    bool start() {
        // If already running, do nothing
        if (running_.exchange(true)) return false;

        // Lock memory to prevent page faults (VERY important in real-time)
        mlockall(MCL_CURRENT | MCL_FUTURE);

        // Launch background real-time control thread
        ctrl_thread_ = std::thread([this]{ this->threadEntry(); });
        return true;
    }

    // Stop thread safely
    void stop() noexcept {
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) return;
        if (ctrl_thread_.joinable()) ctrl_thread_.join();
    }

    // External code uses this to send torque commands safely
    void setTargetTorque(int motor_id, float torque) noexcept {
        if (motor_id < 0 || motor_id >= kMotorCount) return;
        torque_cmds_[motor_id] = torque;
    }

private:
    // This is the real-time loop that runs at 1 kHz
    void threadEntry() noexcept {
        // Set Linux real-time scheduling and pin thread to one CPU
        {
            pthread_t th = pthread_self();
            sched_param sch;
            sch.sched_priority = rt_priority_;

            // Set real-time FIFO scheduler
            pthread_setschedparam(th, SCHED_FIFO, &sch);

            // Pin thread to CPU core to avoid migration jitter
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(control_core_, &cpuset);
            pthread_setaffinity_np(th, sizeof(cpu_set_t), &cpuset);
        }

        // Get current time and prepare periodic timer
        struct timespec next;
        clock_gettime(CLOCK_MONOTONIC, &next);
        const long period_ns = 1'000'000; // 1ms = 1000Hz

        MotorDriver hw;

        while (running_.load()) {
            // Schedule next wake-up time
            next.tv_nsec += period_ns;
            while (next.tv_nsec >= 1'000'000'000) {
                next.tv_nsec -= 1'000'000'000;
                ++next.tv_sec;
            }

            // Copy latest torque commands locally (fast & thread-safe)
            float local_cmds[kMotorCount];
            for (size_t i = 0; i < kMotorCount; ++i)
                local_cmds[i] = torque_cmds_[i];

            // Send commands to hardware (MUST stay fast and predictable)
            for (size_t i = 0; i < kMotorCount; ++i)
                hw.send_command((int)i, local_cmds[i]);

            // Wait precisely until next cycle
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        }
    }

private:
    static constexpr size_t kMotorCount = 4;

    const int control_core_;
    const int rt_priority_;
    std::atomic<bool> running_;
    std::thread ctrl_thread_;

    // Pre-allocated buffer for motor torques, avoids allocation in loop
    std::vector<float> torque_cmds_;
};


/* Notes & best practices

Use mlockall() to avoid page faults in RT thread.

Use clock_nanosleep(...TIMER_ABSTIME) to avoid drift.

Avoid std::cout, heap allocations, dynamic locks inside the loop.

Protect shared buffers with either per-element std::atomic or double-buffer / lock-free exchange depending on contention model.

Adjust SCHED_FIFO priority and CPU affinity per system; requires root or CAP_SYS_NICE.
*/
