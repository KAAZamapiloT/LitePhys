#pragma once
#include "../math/Vector3.h"
#include "../math/Quaternion.h"
#include <string>
#include <map>

namespace Lite {

struct Socket3D {
    std::string name;
    Vector3 localPosition;
    Quaternion localRotation;

    Vector3 worldPosition;
    Quaternion worldRotation;
};

struct Bone3D {
    std::string name;
    int parentIndex = -1; // -1 for root
    
    // Relative to parent
    Vector3 localPosition;
    Quaternion localRotation;

    // Computed via Forward Kinematics
    Vector3 worldPosition;
    Quaternion worldRotation;
    
    // Original resting length (useful for IK)
    real length = 0.0f;

    // Attachments
    std::map<std::string, Socket3D> sockets;

    Bone3D(const std::string& n = "") : name(n) {}

    // Helpers
    void addSocket(const std::string& socketName, const Vector3& posOffset = Vector3(), const Quaternion& rotOffset = Quaternion()) {
        Socket3D socket;
        socket.name = socketName;
        socket.localPosition = posOffset;
        socket.localRotation = rotOffset;
        sockets[socketName] = socket;
    }
};

} // namespace Lite
