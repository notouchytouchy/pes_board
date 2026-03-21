#ifndef LINEFOLLOWER_CONFIG_H
#define LINEFOLLOWER_CONFIG_H

struct LineFollowerConfig{

    const float d_wheel = 0.039f; // wheel diameter in meters
    const float b_wheel = 0.157f;  // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.120f; // distance from wheel axis to leds on sensor bar / array in meters

    const float Kp = 1.2f * 2.0f;
    const float Kp_nl = 1.2f * 17.0f;

};

#endif