
#ifndef JOLT_UTIL_H
#define JOLT_UTIL_H


#include <Jolt/Math/Vec3.h>
#include "core/math/vector3.h"

// Convert Godot Vector3 -> JPH::Vec3
inline JPH::Vec3 ToJolt(const Vector3 &vec) {
    return JPH::Vec3(vec.x, vec.y, vec.z);
}

// Convert Godot Vector3 -> JPH::Vec3
inline JPH::Quat ToJolt(const Quaternion &q) {
    return JPH::Quat(q.x, q.y, q.z, q.w);
}

// Convert JPH::Vec3 -> Godot Vector3
inline Vector3 to_godot(JPH::Vec3Arg vec) {
    return Vector3(vec[0], vec[1], vec[2]);
}

// Convert JPH::Quat -> Godot Quaternion
inline Quaternion to_godot(JPH::QuatArg quat) {
    return Quaternion(quat.GetX(), quat.GetY(), quat.GetZ(), quat.GetW());
}

#endif // JOLT_UTIL_H
