#pragma once
#include <vector>
#include <bullet/LinearMath/btVector3.h>
#include <set>

struct VoronoiFacet {
	std::vector<VoronoiEdge> edges;
	std::vector<btVector3> vertices;
};
struct VoronoiEdge {
	btVector3 v1;
	btVector3 v2;
};

struct VoronoiMesh {
	std::set<btVector3> allSingularVertices;
	std::vector<VoronoiFacet> facets;
};