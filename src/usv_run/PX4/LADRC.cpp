#include <ros/ros.h>
#include <cmath>
#include <algorithm>

typedef struct {
    float wo;           // Observer bandwidth
    float wc;           // Controller bandwidth
    float b0;           // Approximate model gain
    float output_limit_l;
    float output_limit_r;
} LADRC_t;

class LADRC {
public:
    explicit LADRC() {
        init();
    }

    float calculate(float current, float target, float dt) {
        if (std::isnan(current) || std::isnan(target) || std::isnan(dt) || dt <= 0) {
            return last_output;
        }

        // Update ESO (Extended State Observer)
        float error = z1 - current;
        z1 = z1 + dt * (z2 - 3 * ladrc_t.wo * error + ladrc_t.b0 * last_output);
        z2 = z2 + dt * (z3 - 3 * ladrc_t.wo * ladrc_t.wo * error);
        z3 = z3 - dt * ladrc_t.wo * ladrc_t.wo * ladrc_t.wo * error;

        // Calculate control law
        float error_tracking = target - z1;
        float u0 = ladrc_t.wc * error_tracking;
        float output = (u0 - z3) / ladrc_t.b0;

        // Limit output
        output = std::clamp(output, ladrc_t.output_limit_l, ladrc_t.output_limit_r);
        last_output = output;

        return output;
    }

    void init() {
        ladrc_t.wo = 10.0f;         // Default observer bandwidth
        ladrc_t.wc = 5.0f;          // Default controller bandwidth
        ladrc_t.b0 = 1.0f;          // Default approximate model gain
        ladrc_t.output_limit_l = -1.0f;
        ladrc_t.output_limit_r = 1.0f;
        
        // Initialize observer states
        z1 = 0.0f;
        z2 = 0.0f;
        z3 = 0.0f;
        last_output = 0.0f;
    }

    void update(LADRC_t ladrc_t_) {
        ladrc_t.wo = ladrc_t_.wo;
        ladrc_t.wc = ladrc_t_.wc;
        ladrc_t.b0 = ladrc_t_.b0;
        ladrc_t.output_limit_l = ladrc_t_.output_limit_l;
        ladrc_t.output_limit_r = ladrc_t_.output_limit_r;
    }

private:
    LADRC_t ladrc_t;
    float z1;           // Estimated state
    float z2;           // Estimated velocity
    float z3;           // Estimated total disturbance
    float last_output;
};