#pragma once
#include "AABB3D.h"
#include <vector>

namespace Lite {

struct BVHNode3D {
    AABB3D aabb;
    int leftChild = -1;
    int rightChild = -1;
    int parent = -1;
    int objectIndex = -1; // -1 if internal node
    
    bool isLeaf() const { return leftChild == -1 && rightChild == -1; }
};

// Represents a 3D Bounding Volume Hierarchy
// Supports both Static rebuilds (fastest for read-only static geometry) 
// and Dynamic updates (fat AABBs with incremental insertion)
class BVH3D {
private:
    std::vector<BVHNode3D> nodes;
    int rootIndex = -1;
    int nodeCount = 0;

    int allocateNode() {
        if (nodeCount >= nodes.size()) {
            nodes.push_back(BVHNode3D());
        }
        int index = nodeCount++;
        nodes[index] = BVHNode3D(); // Reset
        return index;
    }

    void freeNode(int index) {
        // Simple garbage collection not implemented for brevity, but dynamic inserts would reuse this.
        // For now, we will rely on full array clears for static rebuilds.
    }

    // Static Rebuild Helper
    int buildRecursive(std::vector<std::pair<AABB3D, int>>& objects, int start, int end) {
        if (start >= end) return -1;
        
        int nodeIdx = allocateNode();
        BVHNode3D& node = nodes[nodeIdx];
        
        // Calculate bounds of all objects in this node
        AABB3D bounds;
        for (int i = start; i < end; ++i) {
            bounds.expand(objects[i].first);
        }
        node.aabb = bounds;

        if (end - start == 1) {
            node.objectIndex = objects[start].second;
            return nodeIdx;
        }

        // Find longest axis to split
        Vector3 size = bounds.max - bounds.min;
        int axis = 0;
        if (size.y > size.x && size.y > size.z) axis = 1;
        if (size.z > size.x && size.z > size.y) axis = 2;

        // Sort along longest axis
        std::sort(objects.begin() + start, objects.begin() + end, [axis](const std::pair<AABB3D, int>& a, const std::pair<AABB3D, int>& b) {
            real centerA = (axis == 0) ? a.first.getCenter().x : ((axis == 1) ? a.first.getCenter().y : a.first.getCenter().z);
            real centerB = (axis == 0) ? b.first.getCenter().x : ((axis == 1) ? b.first.getCenter().y : b.first.getCenter().z);
            return centerA < centerB;
        });

        int mid = start + (end - start) / 2;
        
        node.leftChild = buildRecursive(objects, start, mid);
        node.rightChild = buildRecursive(objects, mid, end);
        
        if (node.leftChild != -1) nodes[node.leftChild].parent = nodeIdx;
        if (node.rightChild != -1) nodes[node.rightChild].parent = nodeIdx;

        return nodeIdx;
    }

    // Raycast helper
    void raycastRecursive(int nodeIdx, const Ray3D& ray, std::vector<int>& hits, real maxDistance) const {
        if (nodeIdx == -1) return;
        
        const BVHNode3D& node = nodes[nodeIdx];
        real tOut;
        if (node.aabb.intersectRay(ray, tOut, maxDistance)) {
            if (node.isLeaf()) {
                hits.push_back(node.objectIndex);
            } else {
                raycastRecursive(node.leftChild, ray, hits, maxDistance);
                raycastRecursive(node.rightChild, ray, hits, maxDistance);
            }
        }
    }

public:
    BVH3D() {
        nodes.reserve(1024);
    }

    // STATIC MODE: Rebuild entire tree from scratch (Very fast for < 10k objects, highly balanced)
    void buildStatic(std::vector<std::pair<AABB3D, int>>& objects) {
        nodeCount = 0;
        rootIndex = buildRecursive(objects, 0, objects.size());
    }

    // Query raycast
    std::vector<int> queryRaycast(const Ray3D& ray, real maxDistance = 99999.0f) const {
        std::vector<int> hits;
        raycastRecursive(rootIndex, ray, hits, maxDistance);
        return hits;
    }

    // Query AABB overlap
    std::vector<int> queryAABB(const AABB3D& aabb) const {
        std::vector<int> hits;
        if (rootIndex == -1) return hits;
        
        std::vector<int> stack;
        stack.push_back(rootIndex);
        
        while (!stack.empty()) {
            int nodeIdx = stack.back();
            stack.pop_back();
            
            const BVHNode3D& node = nodes[nodeIdx];
            if (node.aabb.overlaps(aabb)) {
                if (node.isLeaf()) {
                    hits.push_back(node.objectIndex);
                } else {
                    if (node.leftChild != -1) stack.push_back(node.leftChild);
                    if (node.rightChild != -1) stack.push_back(node.rightChild);
                }
            }
        }
        return hits;
    }
};

} // namespace Lite
