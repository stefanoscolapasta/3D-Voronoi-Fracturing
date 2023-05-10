#pragma once
#include<glm/glm.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <vector>
#include <set>
#include<algorithm>
#include "utils.h"
#include "model.h"
#include "physicsEngine.h"
#include "Cube.h"

struct TriangleFacet {
    Tetrahedron *father;
    btVector3 vertices[3];
};

struct Tetrahedron {
    unsigned int VAO;
    btVector3 allSingularVertices[4];
    TriangleFacet facets[4];
};

class VoronoiFracturing {

public:

    std::vector<Model*> tetrahedronsModelsVector; //Its a vector of arrays of btVector3
    std::vector<btRigidBody*> tetraRigidbodies;
    std::vector<Tetrahedron> tetrahedrons;
    std::map<btRigidBody*, unsigned int> tetraToVAO;
    TriangleFacet visibility_walk(std::vector<Tetrahedron> tetras, btVector3 p);
    std::vector<Tetrahedron> getNeighbours(std::vector<Tetrahedron> allTetras, Tetrahedron t);

    VoronoiFracturing(Model* tetrahedronModel, PhysicsEngineAbstraction& pe) : pe(pe) { //Will have to generalize to other shapes
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
            for (int faceVertexIndex = 0; faceVertexIndex < 3; faceVertexIndex++) {
                tetrahedronFacet.vertices[faceVertexIndex] = fromVertexToBtVector3(tetrahedronvertices.vertices[tetrahedronvertices.indices[facet + faceVertexIndex]]);
            }
            newTetrahedron.facets[facet] = tetrahedronFacet;
        }

        //Now that I have the data organized in a simpler way with my data structs, I can work on the FLIP14 algorithm
        for (auto facet : newTetrahedron.facets) {
            //Each facet will now become part of a separate tetrahedron
            //I need to generate 3 new facets, and together with the initial one it will generate a new tetrahedron
            TriangleFacet facet1 = {
                &newTetrahedron,
                { facet.vertices[0], facet.vertices[1], t }
            };
            TriangleFacet facet2 = {
                &newTetrahedron,
                { facet.vertices[1], facet.vertices[2], t }
            };
            TriangleFacet facet3 = {
                &newTetrahedron,
                { facet.vertices[2], facet.vertices[0], t }
            };
            TriangleFacet facet4 = {
                &newTetrahedron,
                { facet.vertices[0], facet.vertices[1], facet.vertices[2]}
            };


            btVector3 allVertices[4];
            int added = 0;
            for (auto& vertex : tetrahedronvertices.vertices) {
                bool alreadyAdded = false;
                btVector3 vertexConverted = fromVertexToBtVector3(vertex);
                for (int i = 0; i < added; i++) {
                    if (allVertices[i].x() == vertexConverted.x() &&
                        allVertices[i].y() == vertexConverted.y() &&
                        allVertices[i].z() == vertexConverted.z()) { //Not sure if btVector3 has an isequal, so I check component by component
                        alreadyAdded = true;
                        break;
                    }
                }
                if (!alreadyAdded) {
                    allVertices[added] = vertexConverted;
                    added += 1;
                }
            }

            if (added != 4) {
                throw std::invalid_argument("You have less than 4 vertices in your tetrahedron");
            }


            Tetrahedron newInsertedTetrahedron = {
                {NULL},
                {allVertices[0], allVertices[1], allVertices[2], allVertices[3]},
                { facet1, facet2, facet3, facet4 }
            };

            newTetrahedrons.push_back(newInsertedTetrahedron);
        }

        return newTetrahedrons;
    }




    //triangles -> tetrahedra
    //edges -> facets
    TriangleFacet visibility_walk(std::vector<Tetrahedron> tetras, btVector3 p) {
        int randomIndex_t = std::rand() % tetras.size(); //random index between 0 and size of tetras
        Tetrahedron t = tetras.at(randomIndex_t);
        Tetrahedron previous = t;
        bool end = false;
        TriangleFacet f;
        while (!end) {
            int randomIndex_f = std::rand() %3; //random index from 0 to 2 - every tetra has 4 fixed facets
            f = t.facets[randomIndex_f];
            std::vector<Tetrahedron> t_neighbours = getNeighbours(tetras, t);
            //check if p is inside one of the neighbours
            bool isPointInNeighbour = false;
            for (auto neighbour : t_neighbours)
                if (isPointInsideTetrahedron(neighbour,p))
                    isPointInNeighbour = true;

            //point is not neighbour of "previous" through facet f
            if (!(isPointInNeighbour && isPointInsideFacet(f, p))) {
                //where is the center positioned in space respect to the facet we are considering
                int centerOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], getTetrahedronCenter(t));
                //same thing for point
                int pointOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], p);
                //if point is not in the same side of center respect to the facet, the two orientations
                //will have different signs -> negative product
                if (centerOrientation * pointOrientation < 0) {
                    previous = t;
                    t = *f.father;
                }
            }
            //point is neighbour of "previous" through facet f
            else {
                f = t.facets[(randomIndex_f + 1)%3];
                //same process as before
                if (!(isPointInNeighbour && isPointInsideFacet(f, p))) {
                    int centerOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], getTetrahedronCenter(t));
                    int pointOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], p);
                    if (centerOrientation * pointOrientation < 0) {
                        previous = t;
                        t = *f.father;
                    }
                }
                else {
                    f = t.facets[(randomIndex_f + 2) % 3];
                    if (!(isPointInNeighbour && isPointInsideFacet(f, p))) {
                        int centerOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], getTetrahedronCenter(t));
                        int pointOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], p);
                        if (centerOrientation * pointOrientation < 0) {
                            previous = t;
                            t = *f.father;
                        }
                    }
                }
            }
        }
    }

    std::vector<Tetrahedron> getNeighbours(std::vector<Tetrahedron> allTetras, Tetrahedron t) {
        std::vector<Tetrahedron> neighbors;

        // Iterate through all tetrahedrons
        for (auto tetra : allTetras) {
            // Skip the input tetrahedron
            if (areTetrasEqual(tetra, t)) {
                continue;
            }

            // Check if the tetrahedron shares a facet with the input tetrahedron
            for (auto f_to_compare : tetra.facets) {
                bool isSameFacet = true;
                for (auto f : t.facets) {
                    bool sameVertices = false;
                    for (int i = 0; i < 3; i++) {
                        bool foundMatchingVertex = false;
                        for (auto v : f_to_compare.vertices) {
                            if (v == f.vertices[i]) {
                                foundMatchingVertex = true;
                                break;
                            }
                        }


                        if (!foundMatchingVertex) {
                            sameVertices = false;
                            break;
                        }
                        else {
                            sameVertices = true;
                        }
                    }
                    if (sameVertices) {
                        isSameFacet = true;
                        break;
                    }
                }
            }
        }

        return neighbors;
    }

 
private:

    PhysicsEngineAbstraction pe;

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
        glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(btVector3), tetra.allSingularVertices, GL_STATIC_DRAW);
        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(btScalar), (void*)0);
        glEnableVertexAttribArray(0);
        return tetraVAO;
    }

};