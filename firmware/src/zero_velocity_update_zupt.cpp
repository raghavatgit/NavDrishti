#include <stdbool.h>

bool detect_stance_phase(float accel_norm, float gyro_norm) {
    return (accel_norm > 9.6f && accel_norm < 10.0f && gyro_norm < 0.05f);
}
