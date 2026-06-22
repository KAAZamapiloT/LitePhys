#pragma once
#include "AABB4D.h"
#include <vector>
#include <algorithm>

namespace Lite {

struct BVHNode4D {
    AABB4D aabb;
    int leftChild = -1;
    int rightChild = -1;
    int parent = -1;
    int objectIndex = -1;
    
    bool isLeaf() const { return leftChild == -1 && rightChild == -1; }
};

class BVH4D {
private:
    std::vector<BVHNode4D> nodes;
    int rootIndex = -1;
    int nodeCount = 0;

    int allocateNode() {
        if (nodeCount >= nodes.size()) {
            nodes.push_back(BVHNode4D());
        }
        int index = nodeCount++;
        nodes[index] = BVHNode4D(); 
        return index;
    }

    int buildRecursive(std::vector<std::pair<AABB4D, int>>& objects, int start, int end) {
        if (start >= end) return -1;
        
        int nodeIdx = allocateNode();
        BVHNode4D& node = nodes[nodeIdx];
        
        AABB4D bounds;
        for (int i = start; i < end; ++i) {
            bounds.expand(objects[i].first);
        }
        node.aabb = bounds;

        if (end - start == 1) {
            node.objectIndex = objects[start].second;
            return nodeIdx;
        }

        Vector4 size = bounds.max - bounds.min;
        int axis = 0;
        if (size.y > size.x && size.y > size.z && size.y > size.w) axis = 1;
        if (size.z > size.x && size.z > size.y && size.z > size.w) axis = 2;
        if (size.w > size.x && size.w > size.y && size.w > size.z) axis = 3;

        std::sort(objects.begin() + start, objects.begin() + end, [axis](const std::pair<AABB4D, int>& a, const std::pair<AABB4D, int>& b) {
            real centerA = (axis == 0) ? a.first.getCenter().x : ((axis == 1) ? a.first.getCenter().y : ((axis == 2) ? a.first.getCenter().z : a.first.getCenter().w));
            real centerB = (axis == 0) ? b.first.getCenter().x : ((axis == 1) ? b.first.getCenter().y : ((axis == 2) ? b.first.getCenter().z : b.first.getCenter().w));
            return centerA < centerB;
        });

        int mid = start + (end - start) / 2;
        
        node.leftChild = buildRecursive(objects, start, mid);
        node.rightChild = buildRecursive(objects, mid, end);
        
        if (node.leftChild != -1) nodes[node.leftChild].parent = nodeIdx;
        if (node.rightChild != -1) nodes[node.rightChild].parent = nodeIdx;

        return nodeIdx;
    }

    void raycastRecursive(int nodeIdx, const Ray4D& ray, std::vector<int>& hits, real maxDistance) const {
        if (nodeIdx == -1) return;
        
        const BVHNode4D& node = nodes[nodeIdx];
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
    BVH4D() {
        nodes.reserve(1024);
    }

    void buildStatic(std::vector<std::pair<AABB4D, int>>& objects) {
        nodeCount = 0;
        rootIndex = buildRecursive(objects, 0, objects.size());
    }

    std::vector<int> queryRaycast(const Ray4D& ray, real maxDistance = 99999.0f) const {
        std::vector<int> hits;
        raycastRecursive(rootIndex, ray, hits, maxDistance);
        return hits;
    }

    std::vector<int> queryAABB(const AABB4D& aabb) const {
        std::vector<int> hits;
        if (rootIndex == -1) return hits;
        
        std::vector<int> stack;
        stack.push_back(rootIndex);
        
        while (!stack.empty()) {
            int nodeIdx = stack.back();
            stack.pop_back();
            
            const BVHNode4D& node = nodes[nodeIdx];
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
