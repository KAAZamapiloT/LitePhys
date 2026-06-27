#pragma once
#include "Skeleton3D.h"
#include <vector>

namespace Lite {

class IK3D {
public:
    // Solves FABRIK for a specific kinematic chain (end effector up to some root)
    // chainIndices should be ordered from Root to EndEffector.
    // Ensure Skeleton3D::computeFK() was called before this!
    // Assumes bones are locally aligned such that the child offset defines their "length vector".
    static void solveFABRIK(Skeleton3D& skeleton, const std::vector<int>& chainIndices, const Vector3& target, int maxIterations = 10, real tolerance = 0.01f) {
        if (chainIndices.size() < 2) return;

        int numBones = chainIndices.size();
        std::vector<Vector3> positions(numBones);
        std::vector<real> lengths(numBones - 1);

        real totalLength = 0.0f;

        // Extract current world positions and lengths
        for (int i = 0; i < numBones; ++i) {
            positions[i] = skeleton.bones[chainIndices[i]].worldPosition;
            if (i > 0) {
                // Length from parent to this bone
                lengths[i - 1] = skeleton.bones[chainIndices[i]].localPosition.magnitude();
                totalLength += lengths[i - 1];
            }
        }

        Vector3 rootPos = positions[0];

        // 1. Check if target is reachable
        if ((target - rootPos).magnitude() >= totalLength) {
            // Unreachable: straighten out towards the target
            Vector3 dir = (target - rootPos).normalized();
            for (int i = 1; i < numBones; ++i) {
                positions[i] = positions[i - 1] + dir * lengths[i - 1];
            }
        } else {
            // Reachable: iterate FABRIK
            for (int iter = 0; iter < maxIterations; ++iter) {
                // Backward Pass (End effector to Root)
                positions[numBones - 1] = target;
                for (int i = numBones - 2; i >= 0; --i) {
                    Vector3 dir = (positions[i] - positions[i + 1]).normalized();
                    positions[i] = positions[i + 1] + dir * lengths[i];
                }

                // Forward Pass (Root to End effector)
                positions[0] = rootPos;
                for (int i = 1; i < numBones; ++i) {
                    Vector3 dir = (positions[i] - positions[i - 1]).normalized();
                    positions[i] = positions[i - 1] + dir * lengths[i - 1];
                }

                // Check tolerance
                if ((positions[numBones - 1] - target).magnitude() < tolerance) {
                    break;
                }
            }
        }

        // 2. Apply new positions back to the skeleton by computing necessary local rotations
        // For a bone, its world direction should point towards its child.
        // We will assume the default "rest" local direction of the bone is the normalized localPosition of the child.
        
        // Root world rotation (we don't change this, we just change the bone local rotations)
        Quaternion parentWorldRot;
        if (skeleton.bones[chainIndices[0]].parentIndex != -1) {
            parentWorldRot = skeleton.bones[skeleton.bones[chainIndices[0]].parentIndex].worldRotation;
        }

        for (int i = 0; i < numBones - 1; ++i) {
            Bone3D& bone = skeleton.bones[chainIndices[i]];
            Bone3D& child = skeleton.bones[chainIndices[i + 1]];

            Vector3 worldDir = (positions[i + 1] - positions[i]).normalized();
            Vector3 localRestDir = child.localPosition.normalized();

            // We need a quaternion that rotates localRestDir (in world space) to worldDir.
            // But first, what is localRestDir in current world space BEFORE this bone's own rotation?
            // It is parentWorldRot * localRestDir.
            Vector3 currentWorldRestDir = parentWorldRot.rotate(localRestDir);

            // Compute rotation from currentWorldRestDir to worldDir
            Vector3 cross = currentWorldRestDir.cross(worldDir);
            real dot = currentWorldRestDir.dot(worldDir);
            
            Quaternion rotationDiff;
            if (dot < -0.9999f) {
                // 180 degree rotation around any orthogonal axis
                Vector3 ortho = (std::abs(currentWorldRestDir.x) > 0.1f) ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
                Vector3 axis = currentWorldRestDir.cross(ortho).normalized();
                rotationDiff = Quaternion::fromAxisAngle(axis, 3.14159265f);
            } else {
                rotationDiff = Quaternion(1.0f + dot, cross.x, cross.y, cross.z);
                rotationDiff.normalize();
            }

            // Apply to bone's world rotation
            bone.worldRotation = rotationDiff * (parentWorldRot * bone.localRotation);
            bone.worldRotation.normalize();

            // Update parentWorldRot for next iteration
            parentWorldRot = bone.worldRotation;

            // Finally, we extract the new local rotation: localRot = (parentWorldRot_inverse) * bone.worldRotation
            // Wait, we just did `bone.worldRotation = diff * (parentWorldRot * oldLocalRot)`.
            // So newLocalRot is `parentWorldRot_inverse * bone.worldRotation`.
            
            Quaternion parentInv(parentWorldRot.r, -parentWorldRot.i, -parentWorldRot.j, -parentWorldRot.k);
            if (bone.parentIndex != -1) {
                Quaternion actualParentWorldRot = skeleton.bones[bone.parentIndex].worldRotation;
                Quaternion actualParentInv(actualParentWorldRot.r, -actualParentWorldRot.i, -actualParentWorldRot.j, -actualParentWorldRot.k);
                bone.localRotation = actualParentInv * bone.worldRotation;
            } else {
                bone.localRotation = bone.worldRotation; // Assuming root has identity parent
            }
            bone.localRotation.normalize();
        }

        // We should trigger a full FK cascade starting from the root to ensure everything is perfectly aligned
        // (especially the end effector and sockets)
        // skeleton.computeFK(); // Removed so the caller can specify root position/rotation
    }
};

} // namespace Lite
