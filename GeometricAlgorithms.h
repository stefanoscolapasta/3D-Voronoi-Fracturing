#pragma once
#include<glm/glm.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <vector>
#include <algorithm>
#include <set>

#define GL_VERTICES_PER_TETRA 36
#define FACETS_PER_TETRA 4
#define VERTICES_PER_TETRA_FACET 3
#define UNIQUE_VERTICES_PER_TETRA 4

struct TriangleFacet {
    std::vector<btVector3> vertices;
};

struct Tetrahedron {
    unsigned int VAO;
    std::set<btVector3> allSingularVertices;
    std::vector<TriangleFacet> facets;
    std::vector<float> verticesAsSingleArr;
};

class VoronoiFracturing {

public:

    std::vector<Model*> tetrahedronsModelsVector; //Its a vector of arrays of btVector3
    std::set<btRigidBody*> tetraRigidbodies;
    std::vector<Tetrahedron> tetrahedrons;
    std::map<btRigidBody*, unsigned int> tetraToVAO;
    std::map<btRigidBody*, Tetrahedron> rigidbodyToTetra;

    VoronoiFracturing(Model* tetrahedronModel, PhysicsEngineAbstraction& pe) : pe(pe){ //Will have to generalize to other shapes
        tetrahedronsModelsVector.push_back(tetrahedronModel);

        std::set<btVector3> uniqueVertices;
        uniqueVerticesFromModel(uniqueVertices, tetrahedronModel);

        btRigidBody* tetraRigidbody = pe.generateTetrahedronRigidbody(
            cubePositions[0], // Use cube position as starting position
            uniqueVertices,
            btVector3(1.0f, 1.0f, 1.0f)
        );

        Tetrahedron tetra;
        generateTetraedronFacetsFromMesh(tetra, tetrahedronModel->meshes[0]);
        tetra.allSingularVertices = uniqueVertices;
        tetra.verticesAsSingleArr = convertVertexVectorToFlatFloatArr(tetrahedronModel->meshes[0].vertices);
        tetra.VAO = createTetrahedronVAO(tetra);
        tetraToVAO[tetraRigidbody] = tetra.VAO;
        rigidbodyToTetra[tetraRigidbody] = tetra;
        tetraRigidbodies.insert(tetraRigidbody);
        pe.dynamicsWorld->addRigidBody(tetraRigidbody,1,1);
    };

    void insertOnePoint(btVector3 t, btRigidBody* toFlip) { //For now no need to implement the walk algorithm, as we try to just insert the point in the main/first tetrahedron
        std::vector<Tetrahedron> newTetrahedrons = flip14(t, rigidbodyToTetra[toFlip]);
        //I now remove the original container rigidbody from the structs, as I will add the tetrahedrons in which it is divided
        //the idea is correct but popping the last element makes no sense; I should use a gerarchical data struct (nested map), so that I have 
        tetraRigidbodies.erase(toFlip);
        pe.dynamicsWorld->removeRigidBody(toFlip); //And remember to remove it from the physics world

        for (auto& newTetrahedron : newTetrahedrons) {
            newTetrahedron.VAO = createTetrahedronVAO(newTetrahedron);
            
            tetrahedrons.push_back(newTetrahedron);

            btRigidBody* tetraRigidbody = pe.generateTetrahedronRigidbody(
                                            cubePositions[0], // Use cube position as starting position
                                            newTetrahedron.allSingularVertices,
                                            btVector3(1.0f, 1.0f, 1.0f)
                                        );

            tetraToVAO[tetraRigidbody] = newTetrahedron.VAO;
            rigidbodyToTetra[tetraRigidbody] = newTetrahedron;
            tetraRigidbodies.insert(tetraRigidbody);
            pe.dynamicsWorld->addRigidBody(tetraRigidbody,1,1);
        }
    }

    std::vector<Tetrahedron> flip14(btVector3 t, Tetrahedron tetrahedron) {
        std::vector<Tetrahedron> newTetrahedrons;

        //For some reason the third face is degenerate
        //Now that I have the data organized in a simpler way with my data structs, I can work on the FLIP14 algorithm
        for (auto& facet : tetrahedron.facets) {
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
                for (int i = 0; i < VERTICES_PER_TETRA_FACET; i++) {
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

            //From now onwards I am doing all these operations and for loops to flatten the vector and have each component of each position as element

            std::vector<Vertex> toFill;
            for (int k = 0; k < FACETS_PER_TETRA; k++) {
                for (int j = 0; j < VERTICES_PER_TETRA_FACET; j++) {
                    toFill.push_back(btVectorToVertex(newInsertedTetrahedron.facets[k].vertices[j]));
                }
            }

            std::vector<float> flattenedValues = convertVertexVectorToFlatFloatArr(toFill);

            for (int k = 0; k < GL_VERTICES_PER_TETRA; k++) {
                newInsertedTetrahedron.verticesAsSingleArr.push_back(flattenedValues[k]);
            }
            
            newTetrahedrons.push_back(newInsertedTetrahedron);
        }

        return newTetrahedrons;
    }

    std::vector<float> convertVertexVectorToFlatFloatArr(std::vector<Vertex> allVertices) {
        std::vector<float> allVerticesAsFloatArr;
        for (auto& vertice : allVertices) {
            std::vector<float> vectorComponents = generateVerticesArrayFromVertex(vertice);
            allVerticesAsFloatArr.insert(allVerticesAsFloatArr.end(), vectorComponents.begin(), vectorComponents.end());
        }
        return allVerticesAsFloatArr;
    }

    void vectorToFloatArray(const std::vector<float>& vec, float arr[]) {
        for (size_t i = 0; i < vec.size(); i++) {
            arr[i] = vec[i];
        }
    }

    std::vector<float> generateVerticesArrayFromVertex(Vertex v) {
        return { (float)v.Position.x, (float)v.Position.y, (float)v.Position.z };
    }

private:

    PhysicsEngineAbstraction pe;

    

    void uniqueVerticesFromModel(std::set<btVector3>& uniqueVertices, Model* model) {
        for (auto& vertex : model->meshes[0].vertices) {
            btVector3 v = fromVertexToBtVector3(vertex);
            if (uniqueVertices.count(v) != 1) { //If not found add it
                uniqueVertices.insert(v);
            }
        }
    }

    void generateTetraedronFacetsFromMesh(Tetrahedron& tetra, Mesh tetraModel) {
        for (int facet = 0; facet < tetraModel.indices.size() / 3; facet++) { //I consider the faces to be triangular //this for should be from [0 to 3]

            TriangleFacet tetrahedronFacet;

            for (int faceVertexIndex = 0; faceVertexIndex < VERTICES_PER_TETRA_FACET; faceVertexIndex++) {
                tetrahedronFacet.vertices.push_back(
                    fromVertexToBtVector3(tetraModel.vertices[tetraModel.indices[faceVertexIndex + (facet * VERTICES_PER_TETRA_FACET)]])
                );
            }
            tetra.facets.push_back(tetrahedronFacet);
        }
    }

    Vertex btVectorToVertex(btVector3 v) {
        return { { (float)v.getX(), (float)v.getY(), (float)v.getZ() }};
    }

    std::vector<float> generateVerticesArrayFromBtVector3(btVector3 v) {
        return { (float)v.getX(), (float)v.getY(), (float)v.getZ() };
    }

    btVector3 fromVertexToBtVector3(Vertex v) {
        return btVector3(v.Position.x, v.Position.y, v.Position.z);
    }

    

    unsigned int createTetrahedronVAO(Tetrahedron tetra) {
        unsigned int tetraVBO, tetraVAO;
        glGenVertexArrays(1, &tetraVAO);
        glGenBuffers(1, &tetraVBO);

        glBindVertexArray(tetraVAO);

        glBindBuffer(GL_ARRAY_BUFFER, tetraVBO);
        //need to pass this to OpenGL as its simpler to handle strides and stuff
        float vertices[GL_VERTICES_PER_TETRA];
        vectorToFloatArray(tetra.verticesAsSingleArr, vertices);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        // position attribute
        glVertexAttribPointer(0, VERTICES_PER_TETRA_FACET, GL_FLOAT, GL_FALSE, VERTICES_PER_TETRA_FACET * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        return tetraVAO;
    }

};