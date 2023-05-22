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
	glm::vec3 color;
};

struct DelauneyEdge {
	btVector3 v1;
	btVector3 v2;
};


struct DelauneyEdgeComparator {
	bool operator()(const DelauneyEdge& e1, const DelauneyEdge& e2) const {
		if (e1.v1.getX() < e2.v1.getX())
			return true;
		if (e1.v1.getX() > e2.v1.getX())
			return false;

		if (e1.v1.getY() < e2.v1.getY())
			return true;
		if (e1.v1.getY() > e2.v1.getY())
			return false;

		if (e1.v1.getZ() < e2.v1.getZ())
			return true;
		if (e1.v1.getZ() > e2.v1.getZ())
			return false;

		if (e1.v2.getX() < e2.v2.getX())
			return true;
		if (e1.v2.getX() > e2.v2.getX())
			return false;

		if (e1.v2.getY() < e2.v2.getY())
			return true;
		if (e1.v2.getY() > e2.v2.getY())
			return false;

		if (e1.v2.getZ() < e2.v2.getZ())
			return true;
		if (e1.v2.getZ() > e2.v2.getZ())
			return false;

		return false; // The edges are equal
	}
};

struct VoronoiEdgeComparator {
	bool operator()(const VoronoiEdge& e1, const VoronoiEdge& e2) const {
		if (e1.v1.getX() < e2.v1.getX())
			return true;
		if (e1.v1.getX() > e2.v1.getX())
			return false;

		if (e1.v1.getY() < e2.v1.getY())
			return true;
		if (e1.v1.getY() > e2.v1.getY())
			return false;

		if (e1.v1.getZ() < e2.v1.getZ())
			return true;
		if (e1.v1.getZ() > e2.v1.getZ())
			return false;

		if (e1.v2.getX() < e2.v2.getX())
			return true;
		if (e1.v2.getX() > e2.v2.getX())
			return false;

		if (e1.v2.getY() < e2.v2.getY())
			return true;
		if (e1.v2.getY() > e2.v2.getY())
			return false;

		if (e1.v2.getZ() < e2.v2.getZ())
			return true;
		if (e1.v2.getZ() > e2.v2.getZ())
			return false;

		return false; // The edges are equal
	}
};


btVector3 getVoronoiMeshCenter(VoronoiMesh mesh) {
	btVector3 center(0.0f, 0.0f, 0.0f);
	float vertexCount = 0.0;

	for (auto vertex : mesh.allUniqueVertices) {
		center += vertex;
		vertexCount++;
	}

	if (vertexCount > 0) {
		center = center/vertexCount;
	}

	return center;
}

std::vector<DelauneyEdge> findIncidentEdges(std::vector<Tetrahedron> tetras, btVector3 vertex) {
	std::set<DelauneyEdge, DelauneyEdgeComparator> incidentEdges;

	for (Tetrahedron tetra : tetras) {
		for (TriangleFacet facet : tetra.facets) {
			std::vector<btVector3>::iterator vertexIterator = std::find(facet.vertices.begin(), facet.vertices.end(), vertex);
			if (vertexIterator != facet.vertices.end()) {
				int i = vertexIterator - facet.vertices.begin();
					DelauneyEdge e1 = { facet.vertices[i], facet.vertices[(i+1)%3] };
					DelauneyEdge e1_reversed = { facet.vertices[(i + 1) % 3], facet.vertices[i] };
					if(incidentEdges.find(e1) == incidentEdges.end() && incidentEdges.find(e1_reversed) == incidentEdges.end())
						incidentEdges.insert(e1);
					DelauneyEdge e2 = { facet.vertices[i], facet.vertices[(i + 2) % 3] };
					DelauneyEdge e2_reversed = { facet.vertices[(i + 2) % 3], facet.vertices[i] };
					if (incidentEdges.find(e2) == incidentEdges.end() && incidentEdges.find(e2_reversed) == incidentEdges.end())
						incidentEdges.insert(e2);

			}
		}
	}

	// Convert the set to a vector and return
	return std::vector<DelauneyEdge>(incidentEdges.begin(), incidentEdges.end());
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
		btRigidBody* voronoiRigidBody = pe.generateMeshRigidbody(
		startingPosition, // Use cube position as starting position
		std::set<btVector3, btVector3Comparator> (voronoi.allUniqueVertices.begin(), voronoi.allUniqueVertices.end()),
		btVector3(1.0f, 1.0f, 1.0f)
	);

	pe.dynamicsWorld->addRigidBody(voronoiRigidBody,1,1);
	return voronoiRigidBody;
}


std::vector<DelauneyEdge> convertEdgeSetToVector(std::set<DelauneyEdge, DelauneyEdgeComparator> vorEdgeSet) {
	std::vector<DelauneyEdge> vorEdgeVec;
	for (auto edge : vorEdgeSet)
		vorEdgeVec.push_back(edge);
	
	return vorEdgeVec;
}



std::vector<DelauneyEdge> findOutgoingEdges(std::vector<Tetrahedron> tetras, btVector3 vertex)
{
	std::set<DelauneyEdge, DelauneyEdgeComparator> outgoingEdges;
	std::vector<Tetrahedron> tetrasIncidentToVertex = getTetrasIncidentToVertex(tetras, vertex );
	for (Tetrahedron tetra : tetrasIncidentToVertex) {
		TriangleFacet sharedFacet = findSharedFacet(tetra, tetra);
		for (btVector3 v : sharedFacet.vertices) {
			if (v != vertex) {
				DelauneyEdge edge = DelauneyEdge({ vertex, v });
				DelauneyEdge reverseEdge = DelauneyEdge({ v, vertex });
				if (outgoingEdges.find(edge) == outgoingEdges.end() &&
					outgoingEdges.find(reverseEdge) == outgoingEdges.end()) {
					outgoingEdges.insert(edge);
				}
			}
		}
	}

	return convertEdgeSetToVector(outgoingEdges);

}



unsigned int createVoronoiVAO(VoronoiMesh& voronoi) {
	unsigned int voronoiVBO, voronoiVAO, voronoiEBO;


	/* ---> NOT CONSIDERING COLOR FOR NOW <----*/
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