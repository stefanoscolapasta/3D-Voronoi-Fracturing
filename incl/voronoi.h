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
	std::set<btVector3, btVector3Comparator> allUniqueVertices;
	std::vector<VoronoiFacet> facets;
	std::vector<float> verticesAsSingleArr;
	std::vector<unsigned int> indices;
};

struct DelEdge {
	btVector3 v1;
	btVector3 v2;
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


std::vector<DelEdge> findIncidentEdges(std::vector<Tetrahedron> tetras, btVector3 vertex) {
	std::set<DelEdge, DelEdgeComparator> incidentEdges;

	for (Tetrahedron tetra : tetras) {
		for (TriangleFacet facet : tetra.facets) {
			std::vector<btVector3>::iterator vertexIterator = std::find(facet.vertices.begin(), facet.vertices.end(), vertex);
			if (vertexIterator != facet.vertices.end()) {
				int i = vertexIterator - facet.vertices.begin();
					DelEdge e1 = { facet.vertices[i], facet.vertices[(i+1)%3] };
					DelEdge e1_reversed = { facet.vertices[(i + 1) % 3], facet.vertices[i] };
					if(incidentEdges.find(e1) == incidentEdges.end() && incidentEdges.find(e1_reversed) == incidentEdges.end())
						incidentEdges.insert(e1);
					DelEdge e2 = { facet.vertices[i], facet.vertices[(i + 2) % 3] };
					DelEdge e2_reversed = { facet.vertices[(i + 2) % 3], facet.vertices[i] };
					if (incidentEdges.find(e2) == incidentEdges.end() && incidentEdges.find(e2_reversed) == incidentEdges.end())
						incidentEdges.insert(e2);

			}
		}
	}

	// Convert the set to a vector and return
	return std::vector<DelEdge>(incidentEdges.begin(), incidentEdges.end());
}

std::vector<Tetrahedron> getTetrasIncidentToVertex(std::vector<Tetrahedron> tetras, btVector3 vertex){
	std::set<Tetrahedron, TetrahedronComparator> incidentTetras;

	for (Tetrahedron tetra : tetras) {
		for (TriangleFacet facet : tetra.facets) {
			std::vector<btVector3>::iterator vertexIterator = std::find(facet.vertices.begin(), facet.vertices.end(), vertex);
			if (vertexIterator != facet.vertices.end()) {
				incidentTetras.insert(tetra);
			}
		}
	}
	
	std::vector<Tetrahedron> incidentTetras_vector;
	for(auto tetra: incidentTetras){
		incidentTetras_vector.push_back(tetra);
	}
	return incidentTetras_vector;
}


btRigidBody* addVoronoiRigidBody(PhysicsEngineAbstraction pe, VoronoiMesh voronoi, btVector3 startingPosition) {
		btRigidBody* voronoiRigidBody = pe.generateVoronoiRigidbody(
		startingPosition, // Use cube position as starting position
		std::set<btVector3> (voronoi.allUniqueVertices.begin(), voronoi.allUniqueVertices.end()),
		btVector3(1.0f, 1.0f, 1.0f)
	);

	pe.dynamicsWorld->addRigidBody(voronoiRigidBody,1,1);
	return voronoiRigidBody;
}


std::vector<DelEdge> convertEdgeSetToVector(std::set<DelEdge, DelEdgeComparator> vorEdgeSet) {
	std::vector<DelEdge> vorEdgeVec;
	for (auto edge : vorEdgeSet)
		vorEdgeVec.push_back(edge);
	
	return vorEdgeVec;
}



std::vector<DelEdge> findOutgoingEdges(std::vector<Tetrahedron> tetras, btVector3 vertex)
{
	std::set<DelEdge, DelEdgeComparator> outgoingEdges;
	std::vector<Tetrahedron> tetrasIncidentToVertex = getTetrasIncidentToVertex(tetras, vertex );
	for (Tetrahedron tetra : tetrasIncidentToVertex) {
		TriangleFacet sharedFacet = findSharedFacet(tetra, tetra);
		for (btVector3 v : sharedFacet.vertices) {
			if (v != vertex) {
				DelEdge edge = DelEdge({ vertex, v });
				DelEdge reverseEdge = DelEdge({ v, vertex });
				if (outgoingEdges.find(edge) == outgoingEdges.end() &&
					outgoingEdges.find(reverseEdge) == outgoingEdges.end()) {
					outgoingEdges.insert(edge);
				}
			}
		}
	}

	return convertEdgeSetToVector(outgoingEdges);

}



unsigned int createVoronoiVAO(VoronoiMesh & voronoi) {
	unsigned int voronoiVBO, voronoiVAO, voronoiEBO;

	const int numVertices = voronoi.verticesAsSingleArr.size();
	//need to pass this to OpenGL as its simpler to handle strides and stuff
	float* vertices = new float[numVertices];

	std::vector<unsigned int> indices;
	for (auto& facet : voronoi.facets) {
		// Add facet indices to the index array
		unsigned int vertexCount = facet.vertices.size();
		unsigned int startIndex = indices.size();
		for (unsigned int i = 0; i < vertexCount - 2; ++i) {
			indices.push_back(startIndex);
			indices.push_back(startIndex + i + 1);
			indices.push_back(startIndex + i + 2);
		}
	}

	voronoi.indices = indices;

	// Create vertex buffer object (VBO)
	glGenBuffers(1, &voronoiVBO);
	glBindBuffer(GL_ARRAY_BUFFER, voronoiVBO);
	glBufferData(GL_ARRAY_BUFFER, voronoi.verticesAsSingleArr.size() * sizeof(float), voronoi.verticesAsSingleArr.data(), GL_STATIC_DRAW);

	// Create element buffer object (EBO)
	glGenBuffers(1, &voronoiEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, voronoiEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

	// Create vertex array object (VAO)
	glGenVertexArrays(1, &voronoiVAO);
	glBindVertexArray(voronoiVAO);
	glBindBuffer(GL_ARRAY_BUFFER, voronoiVBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, voronoiEBO);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	return voronoiVAO;
}



#endif