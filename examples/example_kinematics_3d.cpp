#include <iostream>
#include <vector>
#include <litephys/kinematics3d/Skeleton3D.h>
#include <litephys/kinematics3d/IK3D.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - Kinematics (FK/IK) Test\n" << std::endl;

    Skeleton3D skeleton;

    // 1. Create a 3-Bone Arm
    // Bone 0: Shoulder
    Bone3D shoulder("Shoulder");
    shoulder.localPosition = Vector3(0, 5, 0); // Shoulder is 5 units off the ground
    int shoulderIdx = skeleton.addBone(shoulder);

    // Bone 1: Elbow
    Bone3D elbow("Elbow");
    elbow.parentIndex = shoulderIdx;
    elbow.localPosition = Vector3(4, 0, 0); // Upper arm is 4 units long along local X
    int elbowIdx = skeleton.addBone(elbow);

    // Bone 2: Hand
    Bone3D hand("Hand");
    hand.parentIndex = elbowIdx;
    hand.localPosition = Vector3(3, 0, 0); // Lower arm is 3 units long along local X
    
    // Attach a Sword socket to the Hand!
    hand.addSocket("SwordSlot", Vector3(1, 0, 0), Quaternion()); // 1 unit further down the hand
    int handIdx = skeleton.addBone(hand);

    // 2. Forward Kinematics (Rest Pose)
    skeleton.computeFK();

    std::cout << "--- Rest Pose (FK) ---" << std::endl;
    std::cout << "Shoulder: X=" << skeleton.bones[shoulderIdx].worldPosition.x << " Y=" << skeleton.bones[shoulderIdx].worldPosition.y << std::endl;
    std::cout << "Elbow:    X=" << skeleton.bones[elbowIdx].worldPosition.x << " Y=" << skeleton.bones[elbowIdx].worldPosition.y << std::endl;
    std::cout << "Hand:     X=" << skeleton.bones[handIdx].worldPosition.x << " Y=" << skeleton.bones[handIdx].worldPosition.y << std::endl;
    std::cout << "Sword:    X=" << skeleton.bones[handIdx].sockets["SwordSlot"].worldPosition.x << " Y=" << skeleton.bones[handIdx].sockets["SwordSlot"].worldPosition.y << std::endl;

    // 3. Inverse Kinematics (Reaching for a target)
    // The arm is fully extended at X=7. Let's make it reach for X=2, Y=8 (Above the shoulder, bent elbow)
    Vector3 target(2, 8, 0);
    std::vector<int> ikChain = { shoulderIdx, elbowIdx, handIdx };

    std::cout << "\n[EVENT] FABRIK Solving to reach Target: X=" << target.x << " Y=" << target.y << std::endl;
    IK3D::solveFABRIK(skeleton, ikChain, target);

    std::cout << "\n--- IK Pose (FABRIK) ---" << std::endl;
    std::cout << "Shoulder: X=" << skeleton.bones[shoulderIdx].worldPosition.x << " Y=" << skeleton.bones[shoulderIdx].worldPosition.y << std::endl;
    std::cout << "Elbow:    X=" << skeleton.bones[elbowIdx].worldPosition.x << " Y=" << skeleton.bones[elbowIdx].worldPosition.y << std::endl;
    std::cout << "Hand:     X=" << skeleton.bones[handIdx].worldPosition.x << " Y=" << skeleton.bones[handIdx].worldPosition.y << " (Should match target!)" << std::endl;
    std::cout << "Sword:    X=" << skeleton.bones[handIdx].sockets["SwordSlot"].worldPosition.x << " Y=" << skeleton.bones[handIdx].sockets["SwordSlot"].worldPosition.y << std::endl;

    return 0;
}
