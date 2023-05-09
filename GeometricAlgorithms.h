#pragma once
#include<glm/glm.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <vector>

struct triangleFacet {
    btVector3 vertices[3];
};

struct tetrahedron {
    triangleFacet facets[4];
};

class VoronoiFracturing {

public:

    std::vector<Model*> tetrahedronverticesVector; //Its a vector of arrays of btVector3

    VoronoiFracturing(Model* tetrahedronvertices) { //Will have to generalize to other shapes
        tetrahedronverticesVector.push_back(tetrahedronvertices);
    };

    std::vector<tetrahedron> insertOnePoint(btVector3 t) { //For now no need to implement the walk algorithm, as we try to just insert the point in the main/first tetrahedron
        std::vector<tetrahedron> newTetrahedrons = flip14(t, tetrahedronverticesVector[0]->meshes[0]);

    }

    std::vector<tetrahedron> flip14(btVector3 t, Mesh tetrahedronvertices) {
        
        std::vector<tetrahedron> newTetrahedrons;

        tetrahedron newTetrahedron;

        for (int facet = 0; facet < tetrahedronvertices.indices.size() / 3; facet++) { //I consider the faces to be triangular //this for should be from [0 to 3]

            triangleFacet tetrahedronFacet;
            for (int faceVertexIndex = 0; faceVertexIndex < 3; faceVertexIndex++) {
                tetrahedronFacet.vertices[faceVertexIndex] = fromVertexToBtVector3(tetrahedronvertices.vertices[tetrahedronvertices.indices[facet + faceVertexIndex]]);
            }
            newTetrahedron.facets[facet] = tetrahedronFacet;
        }

        //Now that I have the data organized in a simpler way with my data structs, I can work on the FLIP14 algorithm
        for (auto facet : newTetrahedron.facets) {
            //Each facet will now become part of a separate tetrahedron
            //I need to generate 3 new facets, and together with the initial one it will generate a new tetrahedron
            triangleFacet facet1 = {
                {facet.vertices[0], facet.vertices[1], t}
            };
            triangleFacet facet2 = {
                {facet.vertices[1], facet.vertices[2], t}
            };
            triangleFacet facet3 = {
                {facet.vertices[2], facet.vertices[3], t}
            };

            tetrahedron newInsertedTetrahedron = {
                { facet, facet1, facet2, facet3 }
            };

            newTetrahedrons.push_back(newInsertedTetrahedron);
        }

        return newTetrahedrons;

    }

private:

    btVector3 fromVertexToBtVector3(Vertex v) {
        return btVector3(v.Position.x, v.Position.y, v.Position.z);
    }

    bool verticeAlreadyExistForTetrahedron(int maxIndexReached, btVector3 tetrahedronvertices[], btVector3 newPoint) {
        for (int k = 0; k < maxIndexReached; k++) {
            if (tetrahedronvertices[k] != NULL && newPoint == tetrahedronvertices[k]) {
                return true;
            }
        }
        return false;
    }

    //btVector3* fromModelToVerticesVector(Model* tetrahedronModel) {

    //    //btRigidBody* tetrahedronRigidBodies[numTetrahedrons];
    //    btVector3 tetrahedronVertices[5]; //Because a tetrahedron has 5 vertices yay
    //    int currentlyGeneratedTetrahedrons = 0;
    //    const int currentVerticesSize = tetrahedronModel->meshes[0].vertices.size();

    //    for (int j = 0; j < currentVerticesSize; j++) {
    //        btVector3 newPoint = btVector3(tetrahedronModel->meshes[0].vertices[j].Position.x,
    //            tetrahedronModel->meshes[0].vertices[j].Position.y,
    //            tetrahedronModel->meshes[0].vertices[j].Position.z);

    //        if (!verticeAlreadyExistForTetrahedron(j, tetrahedronVertices, newPoint)) {
    //            tetrahedronVertices[currentlyGeneratedTetrahedrons] = newPoint;
    //        }
    //    }
    //    return tetrahedronVertices;
    //}

};




