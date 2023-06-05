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
	unsigned int* indices;
	unsigned int nindices;
	unsigned int nvertices;
};


double calculateBoundingBoxVolume(VoronoiMesh voronoiMesh) {
	// Calculate the volume of the bounding box of a VoronoiMesh

	// Get the initial minimum and maximum bounds
	btVector3 minBound = *voronoiMesh.allUniqueVertices.begin();
	btVector3 maxBound = *voronoiMesh.allUniqueVertices.begin();

	// Update the bounds by iterating over all the vertices
	for (const btVector3& vertex : voronoiMesh.allUniqueVertices) {
		// Update the minimum and maximum bounds if necessary
		minBound.setX(std::min(minBound.x(), vertex.x()));
		minBound.setY(std::min(minBound.y(), vertex.y()));
		minBound.setZ(std::min(minBound.z(), vertex.z()));

		maxBound.setX(std::max(maxBound.x(), vertex.x()));
		maxBound.setY(std::max(maxBound.y(), vertex.y()));
		maxBound.setZ(std::max(maxBound.z(), vertex.z()));
	}

	// Calculate the dimensions of the bounding box
	double length = maxBound.x() - minBound.x();
	double width = maxBound.y() - minBound.y();
	double height = maxBound.z() - minBound.z();
	double volume = length * width * height;
	return volume;
}


btVector3 getVoronoiMeshCenter(VoronoiMesh mesh) {
	btVector3 center(0.0f, 0.0f, 0.0f);
	float numVertices = (float)mesh.allUniqueVertices.size(); 

	for (auto vertex : mesh.allUniqueVertices) {
		center += vertex;
	}

	center = center/ numVertices;

	return center;
}


btRigidBody* addVoronoiRigidBody(PhysicsEngineAbstraction pe, VoronoiMesh voronoi, btVector3 startingPosition) {
		btRigidBody* voronoiRigidBody = pe.generateMeshRigidbody(
		startingPosition, 
		std::set<btVector3, btVector3Comparator> (voronoi.allUniqueVertices.begin(), voronoi.allUniqueVertices.end()),
		btVector3(1.0f, 1.0f, 1.0f)
	);

	pe.dynamicsWorld->addRigidBody(voronoiRigidBody);
	return voronoiRigidBody;
}



void createVoronoiVAO(VoronoiMesh& voronoi) {
	unsigned int voronoiVBO, voronoiVAO, voronoiEBO;


	/* ---> NOT CONSIDERING COLOR FOR NOW <----*/

	//need to pass this to OpenGL as its simpler to handle strides and stuff
	// Create vertex buffer object (VBO)
	glGenBuffers(1, &voronoiVBO);
	glBindBuffer(GL_ARRAY_BUFFER, voronoiVBO);
	glBufferData(GL_ARRAY_BUFFER, voronoi.nvertices * sizeof(qh_vertex_t), voronoi.vertices, GL_STATIC_DRAW);

	// Create element buffer object (EBO)
	glGenBuffers(1, &voronoiEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, voronoiEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, voronoi.nindices * sizeof(unsigned int), voronoi.indices, GL_STATIC_DRAW);

	// Create vertex array object (VAO)
	glGenVertexArrays(1, &voronoiVAO);
	glBindVertexArray(voronoiVAO);
	glBindBuffer(GL_ARRAY_BUFFER, voronoiVBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, voronoiEBO);


	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(qh_vertex_t), (void*)0);
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(qh_vertex_t), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(3);




	glBindVertexArray(0);


	voronoi.VAO = voronoiVAO;
	voronoi.VBO = voronoiVBO;
	voronoi.EBO = voronoiEBO;


}




#endif