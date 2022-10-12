
#ifndef JOLT_UTIL_H
#define JOLT_UTIL_H


#include <Jolt/Math/Vec3.h>
#include "core/math/vector3.h"

// Convert Godot Vector3 -> JPH::Vec3
inline JPH::Vec3 ToJolt(const Vector3 &vec) {
    return JPH::Vec3(vec.x, vec.y, vec.z);
}

// Convert JPH::Vec3 -> Godot Vector3
inline Vector3 to_godot(JPH::Vec3Arg vec) {
    return Vector3(vec[0], vec[1], vec[2]);
}

#endif // JOLT_UTIL_H
