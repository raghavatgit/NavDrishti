#ifndef IMU_COORDINATE_TRANSFORMS_H
#define IMU_COORDINATE_TRANSFORMS_H

struct Vector3D { float x, y, z; };
struct Quaternion { float w, x, y, z; };

Vector3D rotate_body_to_ned(Vector3D v, Quaternion q);

#endif
