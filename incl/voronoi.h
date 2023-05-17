#pragma once
#ifndef VORONOI_H
#define VORONOI_H
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
	unsigned int VAO;
	std::set<btVector3> allSingularVertices;
	std::vector<VoronoiFacet> facets;
	std::vector<float> verticesAsSingleArr;
};

struct DelEdge {
	btVector3 v1;
	btVector3 v2;
};

struct PairComparator {
	bool operator()(const std::pair<btVector3, btVector3>& p1, const std::pair<btVector3, btVector3>& p2) const {
		if (p1.first != p2.first)
			return p1.first < p2.first;
		return p1.second < p2.second;
	}
};

struct DelEdgeComparator {
	bool operator()(const DelEdge& e1, const DelEdge& e2) const {
		float sum1 = e1.v1.getX() + e1.v1.getY() + e1.v1.getZ();
		float sum2 = e2.v1.getX() + e2.v1.getY() + e2.v1.getZ();

		if (sum1 < sum2)
			return true;
		if (sum1 > sum2)
			return false;

		// If the sums are equal, compare the second vertices
		float sum3 = e1.v2.getX() + e1.v2.getY() + e1.v2.getZ();
		float sum4 = e2.v2.getX() + e2.v2.getY() + e2.v2.getZ();

		if (sum3 < sum4)
			return true;

		return false;
	}
};


struct VoronoiEdgeComparator {
	bool operator()(const VoronoiEdge& e1, const VoronoiEdge& e2) const {
		float sum1 = e1.v1.getX() + e1.v1.getY() + e1.v1.getZ();
		float sum2 = e2.v1.getX() + e2.v1.getY() + e2.v1.getZ();

		if (sum1 < sum2)
			return true;
		if (sum1 > sum2)
			return false;

		// If the sums are equal, compare the second vertices
		float sum3 = e1.v2.getX() + e1.v2.getY() + e1.v2.getZ();
		float sum4 = e2.v2.getX() + e2.v2.getY() + e2.v2.getZ();

		if (sum3 < sum4)
			return true;

		return false;
	}
};

btRigidBody* addVoronoiRigidBody(PhysicsEngineAbstraction pe, VoronoiMesh voronoi) {
	btRigidBody* voronoiRigidBody = pe.generateVoronoiRigidbody(
		cubePositions[0], // Use cube position as starting position
		voronoi.allSingularVertices,
		btVector3(1.0f, 1.0f, 1.0f)
	);

	pe.dynamicsWorld->addRigidBody(voronoiRigidBody, 1, 1);
	return voronoiRigidBody;
}

int getTotalNumberOfVertices(VoronoiMesh voronoi) {
	std::set<btVector3> vertices;

	for (auto facet : voronoi.facets) {
		for (auto vertex: facet.vertices) {
			vertices.insert(vertex);
		}
	}

	return vertices.size();
}



unsigned int createVoronoiVAO(VoronoiMesh voronoi) {
	unsigned int voronoiVBO, voronoiVAO;
	glGenVertexArrays(1, &voronoiVAO);
	glGenBuffers(1, &voronoiVBO);

	glBindVertexArray(voronoiVAO);

	glBindBuffer(GL_ARRAY_BUFFER, voronoiVBO);
	const int numVertices = getTotalNumberOfVertices(voronoi);
	//need to pass this to OpenGL as its simpler to handle strides and stuff
	float* vertices = new float[numVertices];
	vectorToFloatArray(voronoi.verticesAsSingleArr, vertices);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);


	unsigned int offset = 0;
	for (auto facet : voronoi.facets) {
		unsigned int facetSize = facet.vertices.size();

		// Specify the vertex attribute pointer for the current facet
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)offset);
		glEnableVertexAttribArray(0);

		// Increase the offset based on the facet size
		offset += facetSize * sizeof(float) * 3;
	}
	// Unbind the VAO
	glBindVertexArray(0);

	return voronoiVAO;
}


std::vector<DelEdge> findIncidentEdges( std::vector<Tetrahedron> tetras, btVector3 vertex) {
	std::vector<DelEdge> edges;
	for (auto tetra : tetras) {
		for (auto facet : tetra.facets) {
			auto vertices = facet.vertices;
			if (vertices[0] == vertex) {
				DelEdge edge = { vertices[1], vertices[2] };
				edges.push_back(edge);
			}
			else if (vertices[1] == vertex) {
				DelEdge edge = { vertices[0], vertices[2] };
				edges.push_back(edge);
			}
			else if (vertices[2] == vertex) {
				DelEdge edge = { vertices[0], vertices[1] };
				edges.push_back(edge);
			}
		}
	}
	return edges;
}
#endif