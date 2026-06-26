#pragma once
#include "Bone3D.h"
#include <vector>

namespace Lite {

class Skeleton3D {
public:
    std::vector<Bone3D> bones;

    int addBone(const Bone3D& bone) {
        bones.push_back(bone);
        return (int)bones.size() - 1;
    }

    // Forward Kinematics: cascades local transforms to world transforms.
    // NOTE: This assumes the bones array is sorted topologically (parents before children).
    void computeFK(const Vector3& rootWorldPos = Vector3(0,0,0), const Quaternion& rootWorldRot = Quaternion()) {
        for (size_t i = 0; i < bones.size(); ++i) {
            Bone3D& bone = bones[i];

            if (bone.parentIndex == -1) {
                // Root bone
                bone.worldRotation = rootWorldRot * bone.localRotation;
                bone.worldPosition = rootWorldPos + rootWorldRot.rotate(bone.localPosition);
            } else {
                // Child bone
                Bone3D& parent = bones[bone.parentIndex];
                bone.worldRotation = parent.worldRotation * bone.localRotation;
                bone.worldPosition = parent.worldPosition + parent.worldRotation.rotate(bone.localPosition);
            }

            // Update sockets attached to this bone
            for (auto& pair : bone.sockets) {
                Socket3D& socket = pair.second;
                socket.worldRotation = bone.worldRotation * socket.localRotation;
                socket.worldPosition = bone.worldPosition + bone.worldRotation.rotate(socket.localPosition);
            }
        }
    }
};

} // namespace Lite
