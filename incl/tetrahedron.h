#pragma once
#include <bullet/LinearMath/btVector3.h>
#include <set>


struct TriangleFacet {
    std::vector<btVector3> vertices;
};

struct Tetrahedron {
    unsigned int VAO;
    std::set<btVector3> allSingularVertices;
    std::vector<TriangleFacet> facets;
    std::vector<float> verticesAsSingleArr;
    glm::vec3 color;
};

struct TriangleFacetComparator {
    bool operator()(const TriangleFacet& f1, const TriangleFacet& f2) const {
        int count = 0;
        for (auto& vertix1 : f1.vertices) {
            for (auto& vertix2 : f2.vertices) {
                if (vertix1.getX() == vertix2.getX() &&
                    vertix1.getY() == vertix2.getY() &&
                    vertix1.getZ() == vertix2.getZ()) 
                {
                    count += 1;
                }
            }
        }
        return count == 3;
    }
};


struct TetrahedronComparator {
    bool operator()(const Tetrahedron& t1, const Tetrahedron& t2) const {
        if (t1.VAO < t2.VAO)
            return true;
        return false;
    }
};
