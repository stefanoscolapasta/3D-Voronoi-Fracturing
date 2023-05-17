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

    void insertOnePoint(btVector3 t, btRigidBody* toFlip) { //For now no need to implement the walk algorithm, as we try to just insert the point in the main/first tetrahedron
        std::vector<Tetrahedron> newTetrahedrons = flip14(t, rigidbodyToTetra[toFlip]);
        flip23(rigidbodyToTetra[toFlip], rigidbodyToTetra[toFlip]);
        //flip23(rigidbodyToTetra[toFlip], rigidbodyToTetra[toFlip]);
        //I now remove the original container rigidbody from the structs, as I will add the tetrahedrons in which it is divided
        //the idea is correct but popping the last element makes no sense; I should use a gerarchical data struct (nested map), so that I have 
        tetraRigidbodies.erase(toFlip);
        pe.dynamicsWorld->removeRigidBody(toFlip); //And remember to remove it from the physics world

        for (auto& newTetrahedron : newTetrahedrons) {
            newTetrahedron.VAO = createTetrahedronVAO(newTetrahedron);

            tetrahedrons.push_back(newTetrahedron);

            //btRigidBody* tetraRigidbody = pe.generateTetrahedronRigidbody(
            //    cubePositions[0], // Use cube position as starting position
            //    newTetrahedron.allSingularVertices,
            //    btVector3(1.0f, 1.0f, 1.0f)
            //);

            //tetraToVAO[tetraRigidbody] = newTetrahedron.VAO;
            //rigidbodyToTetra[tetraRigidbody] = newTetrahedron;
            //tetraRigidbodies.insert(tetraRigidbody);
            //pe.dynamicsWorld->addRigidBody(tetraRigidbody,1,1);
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

        for (auto tetra : allTetras) {
            if (tetra.VAO == t.VAO)
                continue;  // Skip the same tetrahedron

            for (auto facet : tetra.facets) {
                for (auto facetToCompare : t.facets) {
                    if (areTriangleFacetsEqual(facet,facetToCompare)) {
                        neighbours.push_back(tetra);
                        break;  // Found a shared facet, move to the next tetrahedron
                    }
                }
            }
        }

        return neighbours;
        

    }

    btVector3 getOppositeVerticeToFacet(Tetrahedron tetra, TriangleFacet facet){
        for (auto& vertex : tetra.allSingularVertices) {
            if (std::find(facet.vertices.begin(), facet.vertices.end(), vertex) != facet.vertices.end()) {
                return vertex;
            }
        }
        throw std::invalid_argument("Strange, didn't find the opposite vertice to this facet in this tetrahedron");
    }

    void verifyNeighbours(std::vector<Tetrahedron> tetras, TriangleFacet f, Tetrahedron* previous, Tetrahedron* t, btVector3 p) {
        int randomIndex_f = std::rand() % 3; //random index from 0 to 2 - every tetra has 4 fixed facets
        f = t->facets[randomIndex_f];
        std::vector<Tetrahedron> t_neighbours = getNeighbours(tetras, *t);
        //check if p is inside one of the neighbours
        bool isPointInNeighbour = false;
        Tetrahedron p_tetra;
        Tetrahedron neighbour_through_f;
        for (auto t : t_neighbours) {
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
            for (auto t : t_neighbours) {
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
                for (auto t : t_neighbours) {
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
        std::vector<VoronoiMesh> VoronoiMeshes;
        //Vor vertex - Del tetra equivalence
        std::vector<VoronoiEdge> edges;
        //mapping each Delauney tetrahedron with its corresponding Voronoi vertex 
        std::map<Tetrahedron, btVector3, TetrahedronComparator> tetraToVertex;
        //mapping each Voronoi edge with its corresponding Delaunay facet (easier to perform algorithm with this structure)
        std::map<VoronoiEdge, TriangleFacet, VoronoiEdgeComparator> vorEdgeToDelFacet;
        //Vor edges going out from Vor vertex
        std::map<btVector3, std::vector<VoronoiEdge>, btVector3Comparator> vorEdgesFromVorVertex;
        btVector3 voronoiVertex;
        for (auto t : tetras) {
            std::set<btVector3> points = t.allSingularVertices;
            voronoiVertex = getSphereCenter(points);
            tetraToVertex.insert({ t, voronoiVertex });
        }

        //Vor edge - Del face equivalence
        for (Tetrahedron t : tetras) {
            std::vector<Tetrahedron> t_neighbours = getNeighbours(tetras, t);
            for (Tetrahedron neighbour : t_neighbours) {
                //facet shared by the ajacent tetrahedra
                TriangleFacet sharedFacet = findSharedFacet(t, neighbour);
                btVector3 v_t = tetraToVertex.at(t);
                btVector3 v_neighbour = tetraToVertex.at(neighbour);
                //each edge corresponds to a shared facet
                //the vertices that form the edge are the Voronoi vertices mapped to the two neighbours
                VoronoiEdge edge = { v_t, v_neighbour };
                vorEdgeToDelFacet.insert({ edge,sharedFacet });


                //CREATE FUNCTION FOR THIS
                //add outgoing edge 
                if (vorEdgesFromVorVertex.find(v_t) == vorEdgesFromVorVertex.end()) {
                    std::vector<VoronoiEdge> incident_edges;
                    incident_edges.push_back(edge);
                    vorEdgesFromVorVertex.insert({ v_t, incident_edges });
                }
                else {
                    vorEdgesFromVorVertex.at(v_t).push_back(edge);
                }
                if (vorEdgesFromVorVertex.find(v_neighbour) == vorEdgesFromVorVertex.end()) {
                    std::vector<VoronoiEdge> incident_edges;
                    incident_edges.push_back(edge);
                    vorEdgesFromVorVertex.insert({ v_neighbour, incident_edges });
                }
                else {
                    vorEdgesFromVorVertex.at(v_neighbour).push_back(edge);
                }
               
            }
        }

        //Del edge - Vor face equivalence
        std::set<DelEdge, DelEdgeComparator> visitedEdges;
        std::map< DelEdge, VoronoiFacet, DelEdgeComparator> delEdgeToVorFacet;
        for (Tetrahedron t : tetras) {
            for (TriangleFacet facet : t.facets) {
                //iterate through Del edges
                for (int i = 0; i < facet.vertices.size() - 1; ++i) {
                    for (int j = i + 1; j < facet.vertices.size(); ++j) {
                        btVector3 v1 = facet.vertices[i];

                        //for each del edge, find the incident tetras -> get the equivalent vertices -> 

                        btVector3 v2 = facet.vertices[j];
                        DelEdge delEdge = { v1,v2 };

                        if (visitedEdges.find(delEdge) == visitedEdges.end()) {
                            visitedEdges.insert(delEdge);
                           
                            std::vector<Tetrahedron> incidentTetras = getTetrasIncidentToEdge(delEdge.v1, delEdge.v2, tetras);
                            std::vector<btVector3> vorFacetVertices;
                            for (Tetrahedron incident_tetra : incidentTetras) {
                                vorFacetVertices.push_back(tetraToVertex.at(incident_tetra));
                            }

                            //connect the vertices that form the voronoi facet 
                            std::vector<VoronoiEdge> vorFacetEdges;
                            for (btVector3 vorVertex : vorFacetVertices) {
                                for (VoronoiEdge e : vorEdgesFromVorVertex.at(vorVertex)) {
                                    //one of the two vertices of the edge is vorVertex (one of the vertices of the face)
                                    //the other vertex in the edge is also contained in the facet -> the edge belongs to this facet
                                    if ((e.v1 == vorVertex && std::find(std::begin(vorFacetVertices), std::end(vorFacetVertices), e.v2) != std::end(vorFacetVertices))
                                        || (e.v2 == vorVertex && std::find(std::begin(vorFacetVertices), std::end(vorFacetVertices), e.v1) != std::end(vorFacetVertices)))
                                        vorFacetEdges.push_back(e);
                                }
                            }

                            //a Voronoi face, which is dual to a Delaunay edge
                            //e, is formed by all the vertices that are dual to
                            //the Delaunay tetrahedra incident to e.
                            VoronoiFacet facet = {
                                vorFacetEdges,
                                vorFacetVertices
                            };
                            //delauney edge to voronoi facet equivalence - needed to build the voronoi mesh
                            delEdgeToVorFacet.insert({ delEdge, facet });
                        }
                        
                    }
                }
            }
        }


        //Del vertex - Vor mesh equivalence
        //first identifying all the edges incident to p, and then extracting the dual face of each edge.
        for (Tetrahedron t : tetras) {
            std::set<btVector3> vertices = t.allSingularVertices;
            for (btVector3 vertex : vertices) {
                std::vector<VoronoiFacet> meshFacets;
                std::vector<btVector3> meshVertices;
                std::vector<DelEdge> incidentEdges = findIncidentEdges(tetras, vertex);
                for (auto edge : incidentEdges) {
                    VoronoiFacet equivalentFacet = delEdgeToVorFacet.at(edge);
                    meshFacets.push_back(equivalentFacet);
                    meshVertices.insert(meshVertices.end(), equivalentFacet.vertices.begin(), equivalentFacet.vertices.end());
                }
                VoronoiMesh mesh = {
                    0,
                    convertToSet(meshVertices),
                    meshFacets,
                    {}
                };
                mesh.VAO = createVoronoiVAO(mesh);
                VoronoiMeshes.push_back(mesh);

            }
        }

        return VoronoiMeshes;
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