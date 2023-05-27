#pragma once
#ifndef VORONOI_H
#define VORONOI_H
#include <vector>
#include <bullet/LinearMath/btVector3.h>
#include <set>



struct VoronoiMesh {
	unsigned int VAO, VBO, EBO;
	std::set<btVector3, btVector3Comparator> allUniqueVertices;
	qh_vertex_t* vertices;
	qh_vec3_t* normals;
	unsigned int* indices;
	unsigned int* normalindices;
	unsigned int nindices;
	unsigned int nvertices;
	unsigned int nnormals;
	glm::vec3 color;
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
	for(auto &tetra: incidentTetras){
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

void createVoronoiVAO(VoronoiMesh& voronoi) {

	unsigned int voronoiVBO, voronoiVAO, voronoiEBO;


	/* ---> NOT CONSIDERING COLOR FOR NOW <----*/

	//need to pass this to OpenGL as its simpler to handle strides and stuff
	// Create vertex buffer object (VBO)
	glGenBuffers(1, &voronoiVBO);
	glBindBuffer(GL_ARRAY_BUFFER, voronoiVBO);
	glBufferData(GL_ARRAY_BUFFER, voronoi.nvertices * sizeof(float), voronoi.vertices, GL_STATIC_DRAW);

	// Create element buffer object (EBO)
	glGenBuffers(1, &voronoiEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, voronoiEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, voronoi.nindices * sizeof(unsigned int), voronoi.indices, GL_STATIC_DRAW);

	// Create vertex array object (VAO)
	glGenVertexArrays(1, &voronoiVAO);
	glBindVertexArray(voronoiVAO);
	glBindBuffer(GL_ARRAY_BUFFER, voronoiVBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, voronoiEBO);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	voronoi.VAO = voronoiVAO;
	voronoi.VBO = voronoiVBO;
	voronoi.EBO = voronoiEBO;


}




#endif