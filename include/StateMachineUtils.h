#ifndef STATE_MACHINE_UTILS_H
#define STATE_MACHINE_UTILS_H

#include "mbed.h"
#include <chrono>

/**
 * Erkennt positive (0->1) und negative (1->0) Flanken.
 * Ideal fuer mechanische Taster (PullUp -> negative Flanke).
 */
class EdgeDetector {
private:
    bool previous_state;
    bool current_state;

public:
    EdgeDetector(bool initial_state = false) 
        : previous_state(initial_state), current_state(initial_state) {}

    void update(bool new_state) {
        previous_state = current_state;
        current_state  = new_state;
    }

    bool isRisingEdge() const { return (current_state && !previous_state); }
    bool isFallingEdge() const { return (!current_state && previous_state); }
    bool getState() const { return current_state; }
};

/**
 * Nicht-blockierender Timer fuer State-Übergänge.
 */
class StateTimer {
private:
    Timer timer;
    uint32_t target_delay_ms;
    bool running;

public:
    StateTimer() : target_delay_ms(0), running(false) {}

    void start(uint32_t delay_ms) {
        target_delay_ms = delay_ms;
        timer.reset();
        timer.start();
        running = true;
    }

    void stop() {
        timer.stop();
        running = false;
    }

    bool isExpired() {
        if (!running) return false;
        uint32_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count();
        if (elapsed_ms >= target_delay_ms) {
            timer.stop();
            running = false;
            return true;
        }
        return false;
    }

    bool isRunning() const { return running; }
};

#endif // STATE_MACHINE_UTILS_H