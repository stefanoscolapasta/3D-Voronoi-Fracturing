#pragma once
#include<glm/glm.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <vector>
#include <algorithm>
#include <set>

struct TriangleFacet {
    btVector3 vertices[3]; 
};

struct Tetrahedron {
    unsigned int VAO;
    btVector3 allSingularVertices[4];
    TriangleFacet facets[4];
    float verticesAsSingleArr[36]; //4 vertices * 3 Components-per-vertex
};

class VoronoiFracturing {

public:

    std::vector<Model*> tetrahedronsModelsVector; //Its a vector of arrays of btVector3
    std::vector<btRigidBody*> tetraRigidbodies;
    std::vector<Tetrahedron> tetrahedrons;
    std::map<btRigidBody*, unsigned int> tetraToVAO;

    VoronoiFracturing(Model* tetrahedronModel, PhysicsEngineAbstraction& pe) : pe(pe){ //Will have to generalize to other shapes
        tetrahedronsModelsVector.push_back(tetrahedronModel);
    };

    void insertOnePoint(btVector3 t) { //For now no need to implement the walk algorithm, as we try to just insert the point in the main/first tetrahedron
        std::vector<Tetrahedron> newTetrahedrons = flip14(t, tetrahedronsModelsVector[0]->meshes[0]);

        for (auto& newTetrahedron : newTetrahedrons) {
            newTetrahedron.VAO = createTetrahedronVAO(newTetrahedron);
            
            tetrahedrons.push_back(newTetrahedron);

            btRigidBody* tetraRigidbody = pe.generateTetrahedronRigidbody(
                                            cubePositions[0], // Use cube position as starting position
                                            newTetrahedron.allSingularVertices,
                                            btVector3(1.0f, 1.0f, 1.0f)
                                        );

            tetraToVAO[tetraRigidbody] = newTetrahedron.VAO;
            tetraRigidbodies.push_back(tetraRigidbody);
            pe.dynamicsWorld->addRigidBody(tetraRigidbody);
        }
    }

    std::vector<Tetrahedron> flip14(btVector3 t, Mesh tetrahedronvertices) {
        std::vector<Tetrahedron> newTetrahedrons;

        Tetrahedron newTetrahedron;

        for (int facet = 0; facet < tetrahedronvertices.indices.size() / 3; facet++) { //I consider the faces to be triangular //this for should be from [0 to 3]

            TriangleFacet tetrahedronFacet;
            //WTF did I write here :0
            for (int faceVertexIndex = 0; faceVertexIndex < 3; faceVertexIndex++) {
                tetrahedronFacet.vertices[faceVertexIndex] = fromVertexToBtVector3(tetrahedronvertices.vertices[tetrahedronvertices.indices[faceVertexIndex + (facet*3)]]);
            }
            newTetrahedron.facets[facet] = tetrahedronFacet;
        }
        //For some reason the third face is degenerate
        //Now that I have the data organized in a simpler way with my data structs, I can work on the FLIP14 algorithm
        for (auto& facet : newTetrahedron.facets) {
            //Each facet will now become part of a separate tetrahedron
            //I need to generate 3 new facets, and together with the initial one it will generate a new tetrahedron
            TriangleFacet facet1 = {
                { facet.vertices[0], facet.vertices[1], t }
            };
            TriangleFacet facet2 = {
                { facet.vertices[1], facet.vertices[2], t }
            };
            TriangleFacet facet3 = {
                { facet.vertices[2], facet.vertices[0], t }
            };
            TriangleFacet facet4 = {
                { facet.vertices[0], facet.vertices[1], facet.vertices[2]}
            };

            std::vector<TriangleFacet> facets = { facet1, facet2, facet3, facet4 };

            //Checked the individual positions of each facet and they are correct (visualized them in blender)
            std::vector<btVector3> uniqueVertices;
            for (auto& facet : facets) {
                for (int i = 0; i < 3; i++) {
                    if (std::find(uniqueVertices.begin(), uniqueVertices.end(), facet.vertices[i]) == uniqueVertices.end()) { //If not found add it
                        uniqueVertices.push_back(facet.vertices[i]);
                    }
                }
            }



            Tetrahedron newInsertedTetrahedron = {
                {NULL},
                {uniqueVertices[0], uniqueVertices[1], uniqueVertices[2], uniqueVertices[3]},
                { facet1, facet2, facet3, facet4 },
                {}
            };

            //From now onwards I am doing all these operations and for loops to flatten

            std::vector<Vertex> toFill;
            for (int k = 0; k < 4; k++) {
                for (int j = 0; j < 3; j++) {
                    toFill.push_back(btVectorToVertex(newInsertedTetrahedron.facets[k].vertices[j]));
                }
            }

            std::vector<float> flattenedValues = converVertexVectorToFlatFloatArr(toFill);

            for (int k = 0; k < 36; k++) {
                newInsertedTetrahedron.verticesAsSingleArr[k] = flattenedValues[k];
            }

            newTetrahedrons.push_back(newInsertedTetrahedron);
        }

        return newTetrahedrons;
    }

private:

    PhysicsEngineAbstraction pe;

    std::vector<float> converVertexVectorToFlatFloatArr(std::vector<Vertex> allVertices) {
        std::vector<float> allVerticesAsFloatArr;
        for (auto& vertice : allVertices) {
            std::vector<float> vectorComponents = generateVerticesArrayFromVertex(vertice);
            allVerticesAsFloatArr.insert(allVerticesAsFloatArr.end(), vectorComponents.begin(), vectorComponents.end());
        }
        return allVerticesAsFloatArr;
    }

    Vertex btVectorToVertex(btVector3 v) {
        return { { (float)v.getX(), (float)v.getY(), (float)v.getZ() }};
    }

    std::vector<float> generateVerticesArrayFromVertex(Vertex v) {
        return { (float)v.Position.x, (float)v.Position.y, (float)v.Position.z };
    }

    std::vector<float> generateVerticesArrayFromBtVector3(btVector3 v) {
        return { (float)v.getX(), (float)v.getY(), (float)v.getZ() };
    }

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

    unsigned int createTetrahedronVAO(Tetrahedron tetra) {
        unsigned int tetraVBO, tetraVAO;
        glGenVertexArrays(1, &tetraVAO);
        glGenBuffers(1, &tetraVBO);

        glBindVertexArray(tetraVAO);

        glBindBuffer(GL_ARRAY_BUFFER, tetraVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(tetra.verticesAsSingleArr), tetra.verticesAsSingleArr, GL_STATIC_DRAW);
        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        return tetraVAO;
    }

};