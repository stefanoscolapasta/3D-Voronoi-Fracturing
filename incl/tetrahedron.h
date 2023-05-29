#pragma once
#include <bullet/LinearMath/btVector3.h>
#include <set>

struct btVector3Comparator {
    bool operator()(const btVector3& v1, const btVector3& v2) const {
        if (v1.getX() < v2.getX())
            return true;
        if (v1.getX() > v2.getX())
            return false;

        if (v1.getY() < v2.getY())
            return true;
        if (v1.getY() > v2.getY())
            return false;

        if (v1.getZ() < v2.getZ())
            return true;
        if (v1.getZ() > v2.getZ())
            return false;

        return false; // The vectors are equal
    }
};

struct TriangleFacet;
struct Tetrahedron;

struct TriangleFacet {
    std::vector<btVector3> vertices;

    btVector3 getCenter() const {
        btVector3 center(0, 0, 0);
        int numVertices = vertices.size();

        // Calculate the sum of all vertex coordinates
        for (const auto& vertex : vertices) {
            center += vertex;
        }

        // Divide the sum by the number of vertices to get the average (center)
        center /= numVertices;

        return center;
    }

};

struct Tetrahedron {
    unsigned int VAO;
    std::set<btVector3, btVector3Comparator> allSingularVertices;
    std::vector<TriangleFacet> facets;
    std::vector<float> verticesAsSingleArr;
    glm::vec3 color;
};


struct TetrahedronComparator {
    bool operator()(const Tetrahedron& t1, const Tetrahedron& t2) const {
        if (t1.VAO < t2.VAO)
            return true;
        return false;
    }
};

struct TriangleFacetComparator {
    bool operator()(const TriangleFacet& f1, const TriangleFacet& f2) const {
        
        std::set<btVector3, btVector3Comparator> v1;
        for (btVector3 x : f1.vertices) {
            v1.insert(x);
        }

        std::set<btVector3, btVector3Comparator> v2;
        for (btVector3 x : f2.vertices) {
            v2.insert(x);
        }

        v1.insert(v2.begin(), v2.end());
        return v1.size() == 3;
    }
};

