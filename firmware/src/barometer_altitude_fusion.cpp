#include <math.h>

float pressure_to_altitude(float pressure_hpa) {
    return 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.190295f));
}
