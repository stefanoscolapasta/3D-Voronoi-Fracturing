#pragma once
#include <vector>
#include <bullet/LinearMath/btVector3.h>
#include <set>

struct VoronoiEdge {
	btVector3 v1;
	btVector3 v2;
};

struct VoronoiFacet {
	std::vector<VoronoiEdge> edges;
	std::vector<btVector3> vertices;
};


struct VoronoiMesh {
	std::set<btVector3> allSingularVertices;
	std::vector<VoronoiFacet> facets;
};

struct VoronoiEdgeComparator {
	bool operator()(const VoronoiEdge& e1, const VoronoiEdge& e2) const {
		if (e1.v1.getX() + e1.v1.getY() + e1.v1.getZ() < e1.v2.getX() + e1.v2.getY() + e1.v2.getZ())
			return true;
		return false;
	}
};