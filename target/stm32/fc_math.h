/*
 * Math primitives for flight controller
 * Port of common/math.py to C
 */
#ifndef FC_MATH_H
#define FC_MATH_H

#include <stdint.h>
#include <math.h>

/* Constants */
#define GRAVITY 9.8f
#define PI 3.14159265358979323846f

/* ===== Vector3D ===== */
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

/* Vector operations */
static inline Vector3D vec3_new(float x, float y, float z) {
    Vector3D v = {x, y, z};
    return v;
}

static inline Vector3D vec3_add(Vector3D a, Vector3D b) {
    return vec3_new(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline Vector3D vec3_sub(Vector3D a, Vector3D b) {
    return vec3_new(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline Vector3D vec3_scale(Vector3D v, float scalar) {
    return vec3_new(v.x * scalar, v.y * scalar, v.z * scalar);
}

static inline float vec3_dot(Vector3D a, Vector3D b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline Vector3D vec3_cross(Vector3D a, Vector3D b) {
    return vec3_new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

static inline float vec3_magnitude(Vector3D v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static inline Vector3D vec3_normalize(Vector3D v) {
    float mag = vec3_magnitude(v);
    if (mag < 1e-6f) {
        return vec3_new(0, 0, 0);
    }
    return vec3_scale(v, 1.0f / mag);
}

/* ===== Quaternion ===== */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

/* Quaternion operations */
static inline Quaternion quat_new(float w, float x, float y, float z) {
    Quaternion q = {w, x, y, z};
    return q;
}

static inline Quaternion quat_identity(void) {
    return quat_new(1.0f, 0.0f, 0.0f, 0.0f);
}

/* Quaternion multiplication: q1 * q2 */
static inline Quaternion quat_mul(Quaternion q1, Quaternion q2) {
    float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    float y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
    float z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;
    return quat_new(w, x, y, z);
}

/* Scalar multiplication */
static inline Quaternion quat_scale(Quaternion q, float scalar) {
    return quat_new(q.w * scalar, q.x * scalar, q.y * scalar, q.z * scalar);
}

/* Quaternion addition */
static inline Quaternion quat_add(Quaternion q1, Quaternion q2) {
    return quat_new(q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z);
}

/* Conjugate */
static inline Quaternion quat_conjugate(Quaternion q) {
    return quat_new(q.w, -q.x, -q.y, -q.z);
}

/* Normalize */
static inline void quat_normalize(Quaternion *q) {
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 1e-6f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

/* Rotate a vector by a quaternion */
static inline Vector3D quat_rotate(Quaternion q, Vector3D v) {
    /* v' = q * [0, v] * q* */
    /* Using rotation matrix is faster */
    float w = q.w, x = q.x, y = q.y, z = q.z;
    
    /* Rotation matrix */
    float r00 = 1 - 2*y*y - 2*z*z;
    float r01 = 2*x*y - 2*z*w;
    float r02 = 2*x*z + 2*y*w;
    
    float r10 = 2*x*y + 2*z*w;
    float r11 = 1 - 2*x*x - 2*z*z;
    float r12 = 2*y*z - 2*x*w;
    
    float r20 = 2*x*z - 2*y*w;
    float r21 = 2*y*z + 2*x*w;
    float r22 = 1 - 2*x*x - 2*y*y;
    
    return vec3_new(
        r00 * v.x + r01 * v.y + r02 * v.z,
        r10 * v.x + r11 * v.y + r12 * v.z,
        r20 * v.x + r21 * v.y + r22 * v.z
    );
}

/* From euler angles (roll, pitch, yaw in radians) */
static inline Quaternion quat_from_euler(float roll, float pitch, float yaw) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    
    float w = cy * cp * cr + sy * sp * sr;
    float x = cy * cp * sr - sy * sp * cr;
    float y = sy * cp * sr + cy * sp * cr;
    float z = sy * cp * cr - cy * sp * sr;
    
    return quat_new(w, x, y, z);
}

/* To euler angles (roll, pitch, yaw in radians) */
static inline void quat_to_euler(Quaternion q, float *roll, float *pitch, float *yaw) {
    float w = q.w, x = q.x, y = q.y, z = q.z;
    
    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(PI / 2.0f, sinp);
    } else {
        *pitch = atan2f(sinp, sqrtf(1.0f - sinp * sinp));
    }
    
    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

/* To rotation vector (axis-angle) */
static inline Vector3D quat_to_rotation_vector(Quaternion q) {
    float w = q.w, x = q.x, y = q.y, z = q.z;
    
    /* Clamp for numerical safety */
    if (w > 1.0f) w = 1.0f;
    if (w < -1.0f) w = -1.0f;
    
    /* Ensure shortest rotation */
    if (w < 0.0f) {
        w = -w;
        x = -x;
        y = -y;
        z = -z;
    }
    
    float v_norm = sqrtf(x*x + y*y + z*z);
    if (v_norm < 1e-6f) {
        return vec3_new(0, 0, 0);
    }
    
    float angle = 2.0f * acosf(w);
    float scale = angle / v_norm;
    
    return vec3_new(x * scale, y * scale, z * scale);
}

/* Wrap angle to [-pi, pi] */
static inline float wrap_angle(float x) {
    /* Normalize to [-pi, pi) */
    return fmodf(x + PI, 2.0f * PI) - PI;
}

/* Clamp value */
static inline float clamp(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

#endif /* FC_MATH_H */

