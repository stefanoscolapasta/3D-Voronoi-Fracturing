#pragma once
#include <glm/glm.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <vector>
#include <algorithm>
#include <set>
#include <algorithm>
#include <map>
#include "utils.h"
#include "tetrahedron.h"
#include "mesh.h"
#include "voronoi.h"
#include "model.h"
#include "physicsEngine.h"
#include "Cube.h"

#define GL_VERTICES_PER_TETRA 36
#define FACETS_PER_TETRA 4
#define VERTICES_PER_TETRA_FACET 3
#define UNIQUE_VERTICES_PER_TETRA 4


class VoronoiFracturing {

public:

    std::vector<Model*> tetrahedronsModelsVector; //Its a vector of arrays of btVector3
    std::set<btRigidBody*> tetraRigidbodies;
    std::vector<Tetrahedron> tetrahedrons;
    std::map<btRigidBody*, unsigned int> tetraToVAO;
    std::map<btRigidBody*, Tetrahedron> rigidbodyToTetra;



std::vector<btRigidBody*> vorRigidBodies;
std::map<btRigidBody*, unsigned int> vorToVAO;
std::map<btRigidBody*, int> vorToNumVertices;
std::map<btRigidBody*, std::vector<unsigned int>> vorToIndices;

    VoronoiFracturing(Model* tetrahedronModel, PhysicsEngineAbstraction& pe) : pe(pe) { //Will have to generalize to other shapes
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
        pe.dynamicsWorld->addRigidBody(tetraRigidbody, 1, 1);
    };

    void insertOnePoint(btVector3 t, btRigidBody* toFlip, btVector3 startPos) { //For now no need to implement the walk algorithm, as we try to just insert the point in the main/first tetrahedron
        std::vector<Tetrahedron> newTetrahedrons = flip14(t, rigidbodyToTetra[toFlip]);
        flip23(rigidbodyToTetra[toFlip], rigidbodyToTetra[toFlip]);
        //I now remove the original container rigidbody from the structs, as I will add the tetrahedrons in which it is divided
        //the idea is correct but popping the last element makes no sense; I should use a gerarchical data struct (nested map), so that I have 
        glm::vec3 parentColor = rigidbodyToTetra[toFlip].color;
        tetraRigidbodies.erase(toFlip);
        pe.dynamicsWorld->removeRigidBody(toFlip); //And remember to remove it from the physics world

        for (auto& newTetrahedron : newTetrahedrons) {
            newTetrahedron.color = parentColor;
            newTetrahedron.VAO = createTetrahedronVAO(newTetrahedron);

            tetrahedrons.push_back(newTetrahedron);

            btRigidBody* tetraRigidbody = pe.generateTetrahedronRigidbody(
                startPos, // Use cube position as starting position
                newTetrahedron.allSingularVertices,
                btVector3(1.0f, 1.0f, 1.0f)
            );

            tetraToVAO[tetraRigidbody] = newTetrahedron.VAO;
            rigidbodyToTetra[tetraRigidbody] = newTetrahedron;
            tetraRigidbodies.insert(tetraRigidbody);
            pe.dynamicsWorld->addRigidBody(tetraRigidbody, 1, 1);
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

    std::vector<Tetrahedron> flip23(Tetrahedron tetrahedron1, Tetrahedron tetrahedron2) {
        //I assume the given tetrahedrons are correct and neighbours
        //I now need to find the facet they share
        std::map<TriangleFacet, int, TriangleFacetComparator> facets;
        for (int i = 0; i < tetrahedron1.facets.size() && i < tetrahedron2.facets.size(); i++) {
            facets[tetrahedron1.facets[i]] += 1;
            facets[tetrahedron2.facets[i]] += 1;
        }

        bool haveOneSameFacet = false;
        TriangleFacet sameFacet;

        for (std::map<TriangleFacet, int, TriangleFacetComparator>::iterator it = facets.begin(); it != facets.end(); ++it) {
            if (it->second > 1) {
                haveOneSameFacet = true;
                sameFacet = it->first;
            }
        }

        if (haveOneSameFacet) {
            btVector3 v1 = getOppositeVerticeToFacet(tetrahedron1, sameFacet);
            btVector3 v2 = getOppositeVerticeToFacet(tetrahedron2, sameFacet);

            //I now need to build an edge between these two vertices, will use this edge as a divideder to create 3 new tetrahedrons

        }

        return { tetrahedron1 ,tetrahedron2 };
    }

    btVector3 getOppositeVerticeToFacet(Tetrahedron tetra, TriangleFacet facet){
        for (auto& vertex : tetra.allSingularVertices) {
            if (std::find(facet.vertices.begin(), facet.vertices.end(), vertex) != facet.vertices.end()) {
                return vertex;
            }
        }
        throw std::invalid_argument("Strange, didn't find the opposite vertice to this facet in this tetrahedron");
    }

    //triangles -> tetrahedra
    //edges -> facets
    Tetrahedron stochasticWalk(std::vector<Tetrahedron> tetras, btVector3 p) {
        int randomIndex_t = std::rand() % tetras.size(); //random index between 0 and size of tetras
        Tetrahedron t = tetras.at(randomIndex_t);
        Tetrahedron previous = t;
        bool end = false;
        TriangleFacet f;
        while (!end) {
            verifyNeighbours(tetras, f, &previous, &t, p);
        }

        return t;
    }

    std::vector<Tetrahedron> getNeighbours(std::vector<Tetrahedron> allTetras, Tetrahedron t) {
        std::vector<Tetrahedron> neighbours;

        for (auto& tetra : allTetras) {

            for (auto& facet : tetra.facets) {
                for (auto& facetToCompare : t.facets) {
                    if (areTriangleFacetsEqual(facet, facetToCompare)) {
                        neighbours.push_back(tetra);
                        break;  // Found a shared facet, move to the next tetrahedron
                    }
                }
            }
        }

        return neighbours;


    }

    void verifyNeighbours(std::vector<Tetrahedron> tetras, TriangleFacet f, Tetrahedron* previous, Tetrahedron* t, btVector3 p) {
        int randomIndex_f = std::rand() % 3; //random index from 0 to 2 - every tetra has 4 fixed facets
        f = t->facets[randomIndex_f];
        std::vector<Tetrahedron> t_neighbours = getNeighbours(tetras, *t);
        //check if p is inside one of the neighbours
        bool isPointInNeighbour = false;
        Tetrahedron p_tetra;
        Tetrahedron neighbour_through_f;
        for (auto& t : t_neighbours) {
            if (isPointInsideTetrahedron(t, p)) {
                isPointInNeighbour = true;
                p_tetra = t;
            }
            if (isFacetInTetrahedron(p_tetra, f))
                neighbour_through_f = t;
        }

        //point p is not neighbour of tetrahedron 
        if ((!isPointInNeighbour) || (isPointInNeighbour &&
            //point p is neighbour of tetrahedron, but not through facet f -> f is not in p's tetrahedron
            !isFacetInTetrahedron(p_tetra, f))) {
            //where is the center positioned in space respect to the facet we are considering
            int centerOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], getTetrahedronCenter(*t));
            //same thing for point
            int pointOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], p);
            //if point is not in the same side of center respect to the facet, the two orientations
            //will have different signs -> negative product
            if (centerOrientation * pointOrientation < 0) {
                previous = t;
                *t = neighbour_through_f;
            }
        }
        //point is neighbour of "previous" through facet f
        else {
            f = t->facets[(randomIndex_f + 1) % 3];
            //same process as before
            for (auto& t : t_neighbours) {
                if (isPointInsideTetrahedron(t, p)) {
                    isPointInNeighbour = true;
                    p_tetra = t;
                }
                if (isFacetInTetrahedron(p_tetra, f))
                    neighbour_through_f = t;
            }
            if ((!isPointInNeighbour) || (isPointInNeighbour &&
                //point p is neighbour of tetrahedron, but not through facet f
                !isFacetInTetrahedron(p_tetra, f))) {
                int centerOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], getTetrahedronCenter(*t));
                int pointOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], p);
                if (centerOrientation * pointOrientation < 0) {
                    previous = t;
                    *t = neighbour_through_f;
                }
            }
            else {
                f = t->facets[(randomIndex_f + 2) % 3];
                for (auto& t : t_neighbours) {
                    if (isPointInsideTetrahedron(t, p)) {
                        isPointInNeighbour = true;
                        p_tetra = t;
                    }
                    if (isFacetInTetrahedron(p_tetra, f))
                        neighbour_through_f = t;
                }
                if ((!isPointInNeighbour) || (isPointInNeighbour &&
                    //point p is neighbour of tetrahedron, but not through facet f
                    !isFacetInTetrahedron(p_tetra, f))) {
                    int centerOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], getTetrahedronCenter(*t));
                    int pointOrientation = orient(f.vertices[0], f.vertices[1], f.vertices[2], p);
                    if (centerOrientation * pointOrientation < 0) {
                        previous = t;
                        *t = neighbour_through_f;
                    }
                }
            }
        }
    }


    std::vector<VoronoiMesh> convertToVoronoi(std::vector<Tetrahedron> tetras) {
        std::vector<VoronoiMesh> voronoiMeshes;
        std::map <Tetrahedron, btVector3, TetrahedronComparator> tetraToCircumcenter;
        std::map<btVector3, std::vector<DelEdge>, btVector3Comparator> outGoingEdgesFromVertex;
        //find equivalent Voronoi vertex for each Delauney tetrahedron
        for (auto& t : tetras) {
            btVector3 circumcenter = getSphereCenter(t.allSingularVertices);
            tetraToCircumcenter.insert({ t, circumcenter });

            for (auto& vertex : t.allSingularVertices) {
                std::vector<DelEdge> outgoingEdges = findOutgoingEdges(tetras, vertex);

                if (outGoingEdgesFromVertex.find(vertex) != outGoingEdgesFromVertex.end())
                    for (auto outgoingEdge : outgoingEdges)
                        outGoingEdgesFromVertex.at(vertex).push_back(outgoingEdge);
                else
                    outGoingEdgesFromVertex.insert({ vertex, outgoingEdges });
            }

        }

        std::set<VoronoiEdge, VoronoiEdgeComparator> uniqueVoronoiEdges;
        for (auto& t : tetras) {
            for (auto& neighbour : getNeighbours(tetras, t)) {
                //facet shared by the adjacent tetrahedra
                TriangleFacet sharedFacet = findSharedFacet(t, neighbour);
                btVector3 v_t = tetraToCircumcenter.at(t);
                btVector3 v_neighbour = tetraToCircumcenter.at(neighbour);
                if (v_t != v_neighbour) {
                    //each edge corresponds to a shared facet
                    //the vertices that form the edge are the Voronoi vertices mapped to the two neighbours
                    VoronoiEdge edge = { v_t, v_neighbour };
                    VoronoiEdge reversedEdge = { v_neighbour, v_t };
                    if (uniqueVoronoiEdges.find(edge) == uniqueVoronoiEdges.end() && uniqueVoronoiEdges.find(reversedEdge) == uniqueVoronoiEdges.end())
                        uniqueVoronoiEdges.insert(edge);
                }
            }
        }

        //PROCESS : vertex -> incident edges -> dual face
        //first: map dual VoronoiFace to every edge in tetras

        std::set<DelEdge, DelEdgeComparator> visitedEdges;
        std::map<DelEdge, VoronoiFacet, DelEdgeComparator> delEdgeToVorFacet;
        for (auto& t : tetras) {
            for (auto& v1 : t.allSingularVertices) {
                for (auto& v2 : t.allSingularVertices) {
                    if (v1 != v2) {
                        DelEdge edge = { v1, v2 };
                        DelEdge reversedEdge = { v2,v1 };
                        if (visitedEdges.find(edge) == visitedEdges.end() && visitedEdges.find(reversedEdge) == visitedEdges.end()) {
                            visitedEdges.insert(edge);
                            //find all tetrahedra that contain the edge (incident tetrahedra
                            //EDGECASE: one edge is only shared by two tetrahedra -> how to handle this? for now just don't add the facet to the mesh in that case
                            std::vector<Tetrahedron> incidentTetras = getTetrasIncidentToEdge(edge.v1, edge.v2, tetras);
                            //every equivalent of a DelTetrahedron becomes a vertex of the facet
                            std::vector<btVector3> vorFacetVertices;
                            for (Tetrahedron incident_tetra : incidentTetras) {
                                vorFacetVertices.push_back(tetraToCircumcenter.at(incident_tetra));
                            }
                            VoronoiFacet facet = makeFacet(vorFacetVertices, uniqueVoronoiEdges);
                            delEdgeToVorFacet.insert({ edge, facet });
                        }
                    }
                }
            }
        }

        //build voronoi mesh
        for (auto& t : tetras) {
            for (auto& v : t.allSingularVertices) {
                std::vector<VoronoiFacet> facets;
                std::set<btVector3, btVector3Comparator> uniqueVertices;
                std::vector<DelEdge> incidentEdges = outGoingEdgesFromVertex.at(v);
                std::set<btVector3, btVector3Comparator> verticesAsSet;
                for (auto&& incidentEdge : incidentEdges) {
                    DelEdge reversedIncidentEdge = { incidentEdge.v2, incidentEdge.v1 };
                    VoronoiFacet vorFacet;
                    if (delEdgeToVorFacet.find(incidentEdge) != delEdgeToVorFacet.end())
                        vorFacet = delEdgeToVorFacet.at(incidentEdge);
                    else if (delEdgeToVorFacet.find(reversedIncidentEdge) != delEdgeToVorFacet.end())
                        vorFacet = delEdgeToVorFacet.at(reversedIncidentEdge);

                    if (vorFacet.vertices.size() > 2) {
                        facets.push_back(vorFacet);
                        for (auto& vertex : vorFacet.vertices)
                            verticesAsSet.insert(vertex);
                    }
                }
                uniqueVertices.insert(verticesAsSet.begin(), verticesAsSet.end());
                //edge case, mesh with 1/2 facets?
                if (uniqueVertices.size() > 3) {
                    VoronoiMesh mesh = {
                        NULL,
                        uniqueVertices,
                        facets,
                        {},
                        {}
                    };

                    std::vector<Vertex> toFill;


                    for (auto& vorFacet : mesh.facets) {
                        for (auto& vorVertex : vorFacet.vertices) {
                            toFill.push_back(btVectorToVertex(vorVertex));
                        }
                    }


                    mesh.verticesAsSingleArr = convertVertexVectorToFlatFloatArr(toFill);
                    voronoiMeshes.push_back(mesh);
                }

            }
        }

        return voronoiMeshes;


    }

    VoronoiFacet makeFacet(std::vector<btVector3> vorFacetVertices, std::set<VoronoiEdge, VoronoiEdgeComparator> voronoiEdges) {
        //for every vertex that forms the facet, find the edges that belong to it
        std::set<VoronoiEdge, VoronoiEdgeComparator> facetEdges;
        for (auto& vertex : vorFacetVertices) {
            for (auto& edge : voronoiEdges) {
                if ((edge.v1 == vertex && std::find(vorFacetVertices.begin(), vorFacetVertices.end(), edge.v2) != vorFacetVertices.end())
                    || (edge.v2 == vertex && std::find(vorFacetVertices.begin(), vorFacetVertices.end(), edge.v1) != vorFacetVertices.end())
                    ) {
                    facetEdges.insert(edge);
                }
            }
        }

        std::vector<VoronoiEdge> facetEdgesVector;
        for (auto& edge : facetEdges)
            facetEdgesVector.push_back(edge);
        VoronoiFacet facet = { facetEdgesVector,  vorFacetVertices };
        return facet;

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

    void fillVertexData(std::vector<float> verticesAsSingleArr, glm::vec3 color, float vertices[]) {
        std::vector<float> colorAsFloatVec = { color.r, color.g, color.b };
        std::vector<float> verticeAndColorssAsSingleArr;
        for (int i = 0; i < verticesAsSingleArr.size(); i += 3) {
            verticeAndColorssAsSingleArr.insert(verticeAndColorssAsSingleArr.end(), verticesAsSingleArr.begin() + i, verticesAsSingleArr.begin() + (i + 3));
            verticeAndColorssAsSingleArr.insert(verticeAndColorssAsSingleArr.end(), colorAsFloatVec.begin(), colorAsFloatVec.end());
        }
        vectorToFloatArray(verticeAndColorssAsSingleArr, vertices);
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
        float vertices[GL_VERTICES_PER_TETRA * 2];

        fillVertexData(tetra.verticesAsSingleArr, tetra.color, vertices);

        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, VERTICES_PER_TETRA_FACET, GL_FLOAT, GL_FALSE, (VERTICES_PER_TETRA_FACET * 2) * sizeof(float), (void*)0); // (VERTICES_PER_TETRA_FACET*2) to account for vertex color
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(3, VERTICES_PER_TETRA_FACET, GL_FLOAT, GL_FALSE, (VERTICES_PER_TETRA_FACET * 2) * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(3);
        return tetraVAO;
    }

};