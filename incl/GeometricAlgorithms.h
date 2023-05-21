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

#define GL_TOTAL_VERTICES_FLOAT_VALUES_PER_TETRA 36
#define FACETS_PER_TETRA 4
#define VERTICES_PER_TETRA_FACET 3
#define UNIQUE_VERTICES_PER_TETRA 4
#define MESH_FACE_VERTICES 3


class VoronoiFracturing {

public:

    std::vector<Model*> tetrahedronsModelsVector; //Its a vector of arrays of btVector3
    std::set<btRigidBody*> tetraRigidbodies;
    std::vector<Tetrahedron> tetrahedrons;
    std::map<btRigidBody*, unsigned int> rigidbodyToVAO;
    std::map<btRigidBody*, Tetrahedron> rigidbodyToTetra;
    std::map<Tetrahedron, btRigidBody*,TetrahedronComparator> tetraToRigidbody;

    std::vector<btRigidBody*> vorRigidBodies;
    std::map<btRigidBody*, unsigned int> vorToVAO;
    std::map<btRigidBody*, int> vorToNumVertices;
    std::map<btRigidBody*, std::vector<unsigned int>> vorToIndices;
    std::map<Mesh, std::set<btVector3, btVector3Comparator>, MeshComparator> meshToVertices;

    //The model passed has to be made up of tetrahedrons already
    VoronoiFracturing(Model* tetrahedronModel, PhysicsEngineAbstraction& pe, btVector3 startingPosition) : pe(pe) { //Will have to generalize to other shapes
        tetrahedronsModelsVector.push_back(tetrahedronModel);
        for (Mesh mesh : tetrahedronModel->meshes) {
            std::set<btVector3, btVector3Comparator> meshVertices;
            uniqueVerticesFromMesh(meshVertices, mesh);
            meshToVertices[mesh] = meshVertices;
        }

        for (auto& meshEntry : meshToVertices) {
            Tetrahedron tetra;
            generateTetraedronFacetsFromMesh(tetra, meshEntry.first);
            tetra.allSingularVertices = meshEntry.second;

            std::set<Vertex> toFill;
            for (auto& el : meshEntry.second) {
                toFill.insert(btVectorToVertex(el));
            }
            std::vector<Vertex> v(toFill.begin(), toFill.end());

            tetra.color = glm::vec3(1.0f, 1.0f, 1.0f);
            tetra.verticesAsSingleArr = convertVertexVectorToFlatFloatArr(v);
            tetra.VAO = createTetrahedronVAO(tetra);

            btRigidBody* meshRigidbody = pe.generateMeshRigidbody(
                startingPosition, // Use cube position as starting position
                tetra.allSingularVertices,
                btVector3(1.0f, 1.0f, 1.0f)
            );
            rigidbodyToVAO[meshRigidbody] = tetra.VAO;
            rigidbodyToTetra[meshRigidbody] = tetra;
            tetraToRigidbody[tetra] = meshRigidbody;
            tetraRigidbodies.insert(meshRigidbody);
            pe.dynamicsWorld->addRigidBody(meshRigidbody, 1, 1);
        }

    };

    void insertOnePoint(btVector3 t, btVector3 startPos) { //For now no need to implement the walk algorithm, as we try to just insert the point in the main/first tetrahedron
        std::vector<Tetrahedron> tetras;
        for (auto mi = tetraToRigidbody.begin(); mi != tetraToRigidbody.end(); ++mi)
            tetras.push_back(mi->first);

        Tetrahedron tetraFromWalk = stochasticWalk(tetras, t);

        std::vector<Tetrahedron> newTetrahedrons = flip14(t, tetraFromWalk);
        
        //flip23(rigidbodyToTetra[toFlip], rigidbodyToTetra[toFlip]);
        //I now remove the original container rigidbody from the structs, as I will add the tetrahedrons in which it is divided
        //the idea is correct but popping the last element makes no sense; I should use a gerarchical data struct (nested map), so that I have 
        tetraRigidbodies.erase(tetraToRigidbody[tetraFromWalk]);
        pe.dynamicsWorld->removeRigidBody(tetraToRigidbody[tetraFromWalk]); //And remember to remove it from the physics world
        rigidbodyToTetra.erase(tetraToRigidbody[tetraFromWalk]);
        tetraToRigidbody.erase(tetraFromWalk);
        

        for (auto& newTetrahedron : newTetrahedrons) {
            newTetrahedron.color = tetraFromWalk.color;;
            newTetrahedron.VAO = createTetrahedronVAO(newTetrahedron);

            tetrahedrons.push_back(newTetrahedron);

            btRigidBody* tetraRigidbody = pe.generateMeshRigidbody(
                startPos, // Use cube position as starting position
                newTetrahedron.allSingularVertices,
                btVector3(1.0f, 1.0f, 1.0f)
            );

            rigidbodyToVAO[tetraRigidbody] = newTetrahedron.VAO;
            rigidbodyToTetra[tetraRigidbody] = newTetrahedron;
            tetraToRigidbody[newTetrahedron] = tetraRigidbody;
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

            for (int k = 0; k < GL_TOTAL_VERTICES_FLOAT_VALUES_PER_TETRA; k++) {
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
            std::vector<Tetrahedron> flippedTetrahedrons;
            //Here I create the 3 new tetras
            for (int i = 0; i < 3; i++) {
                TriangleFacet facet1 = {
                { v1, v2,  sameFacet.vertices[i % 3]}
                };
                TriangleFacet facet2 = {
                { v1, v2,  sameFacet.vertices[i + 1 % 3]}
                };
                TriangleFacet facet3 = {
                { v1, sameFacet.vertices[i % 3], sameFacet.vertices[i + 1 % 3]}
                };
                TriangleFacet facet4 = {
                { v2, sameFacet.vertices[i % 3], sameFacet.vertices[i + 1 % 3]}
                };

                std::vector<TriangleFacet> facets = { facet1, facet2, facet3, facet4 };

                Tetrahedron newTetra;
                generateTetrahedronFromFacets(newTetra, facets);
                flippedTetrahedrons.push_back(newTetra);
            }

            return flippedTetrahedrons;
        }

        throw std::invalid_argument("Something went wrong: the passed tetrahedron do not share a facet");
    }



    std::vector<Tetrahedron> flip32(std::vector<Tetrahedron> tetrasToFlip) {
        //I assume the given tetrahedrons are correct and neighbours
        //I now need to find the facet they share
        //I can leverage the fact that the vertices along the shared edge are the only ones shared by 3 tetrahedrons to find them
        std::vector<Tetrahedron> newTetras;
        std::map<btVector3, int, btVector3Comparator> vertices;
        for (auto& tetra : tetrasToFlip) {
            for (auto& vertex : tetra.allSingularVertices) {
                vertices[vertex] += 1;
            }
        }

        bool haveOne2CommonVertices = false;
        int found = 0;
        std::vector<btVector3> commonToAll;

        for (std::map<btVector3, int, TriangleFacetComparator>::iterator it = vertices.begin(); it != vertices.end(); ++it) {
            if (it->second == 3) {
                found += 1;
                commonToAll.push_back(it->first);
            }
        }
        haveOne2CommonVertices = found == 2;

        if (haveOne2CommonVertices) {
            //I get all the singular vertices
            std::set<btVector3> allSingularVerticesAcross3Tetras;
            for (auto& tetra : tetrasToFlip) {
                allSingularVerticesAcross3Tetras.insert(tetra.allSingularVertices.begin(), tetra.allSingularVertices.end());
            }

            allSingularVerticesAcross3Tetras.erase(commonToAll[0]);
            allSingularVerticesAcross3Tetras.erase(commonToAll[1]);

            //They should be 3 now
            std::vector<btVector3> allSingularVerticesAcross3TetrasVec(allSingularVerticesAcross3Tetras.begin(), allSingularVerticesAcross3Tetras.end());

            for (auto& vertex : commonToAll) {
                TriangleFacet facet1 = {
                { vertex, allSingularVerticesAcross3TetrasVec[0],  allSingularVerticesAcross3TetrasVec[1]}
                };
                TriangleFacet facet2 = {
                { vertex, allSingularVerticesAcross3TetrasVec[1],  allSingularVerticesAcross3TetrasVec[2]}
                };
                TriangleFacet facet3 = {
                { vertex, allSingularVerticesAcross3TetrasVec[2], allSingularVerticesAcross3TetrasVec[0]}
                };
                TriangleFacet facet4 = {
                { allSingularVerticesAcross3TetrasVec[0], allSingularVerticesAcross3TetrasVec[1], allSingularVerticesAcross3TetrasVec[2]}
                };

                std::vector<TriangleFacet> facets = { facet1, facet2, facet3, facet4 };

                Tetrahedron newTetra;
                generateTetrahedronFromFacets(newTetra, facets);
                newTetras.push_back(newTetra);
            }

            return newTetras;
        }

        throw std::invalid_argument("Something went wrong: the passed tetrahedron do not share a facet");
    }

    void generateTetrahedronFromFacets(Tetrahedron& tetraToGenerate, std::vector<TriangleFacet> facets) {
        //Checked the individual positions of each facet and they are correct (visualized them in blender)
        std::vector<btVector3> uniqueVertices;

        for (auto& facet : facets) {
            for (int i = 0; i < VERTICES_PER_TETRA_FACET; i++) {
                if (std::find(uniqueVertices.begin(), uniqueVertices.end(), facet.vertices[i]) == uniqueVertices.end()) { //If not found add it
                    uniqueVertices.push_back(facet.vertices[i]);
                }
            }
        }

        tetraToGenerate = {
            {NULL},
            {uniqueVertices[0], uniqueVertices[1], uniqueVertices[2], uniqueVertices[3]},
            { facets[0], facets[1], facets[2], facets[3] },
            {}
        };

        std::vector<Vertex> toFill;
        for (int k = 0; k < FACETS_PER_TETRA; k++) {
            for (int j = 0; j < VERTICES_PER_TETRA_FACET; j++) {
                toFill.push_back(btVectorToVertex(tetraToGenerate.facets[k].vertices[j]));
            }
        }

        std::vector<float> flattenedValues = convertVertexVectorToFlatFloatArr(toFill);

        for (int k = 0; k < GL_TOTAL_VERTICES_FLOAT_VALUES_PER_TETRA; k++) {
            tetraToGenerate.verticesAsSingleArr.push_back(flattenedValues[k]);
        }
        tetraToGenerate.VAO = createTetrahedronVAO(tetraToGenerate);
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

        verifyNeighbours(tetras, f, &previous, &t, p);

        return t;
    }

    std::vector<Tetrahedron> getNeighbours(std::vector<Tetrahedron> allTetras, Tetrahedron t) {
        std::set<Tetrahedron, TetrahedronComparator> neighbours;

        for (auto& tetra : allTetras) {

            for (auto& facet : tetra.facets) {
                for (auto& facetToCompare : t.facets) {
                    if (areTriangleFacetsEqual(facet, facetToCompare)) {
                        neighbours.insert(tetra);
                        break;  // Found a shared facet, move to the next tetrahedron
                    }
                }
            }
        }

        std::vector<Tetrahedron> neighbours_vector;
        for (auto &tetra : neighbours) {
            neighbours_vector.push_back(tetra);
        }

        return neighbours_vector;


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
       
        std::map<btVector3, std::vector<DelauneyEdge>, btVector3Comparator> outgoingEdgesFromVertex;
        std::map <Tetrahedron, btVector3, TetrahedronComparator> tetraToCircumcenter;
        //find equivalent Voronoi vertex for each Delauney tetrahedron

        setupOutgoingEdgesAndCircumcenters(outgoingEdgesFromVertex, tetraToCircumcenter, tetras);

        std::set<VoronoiEdge, VoronoiEdgeComparator> uniqueVoronoiEdges = findUniqueVoronoiEdges(tetraToCircumcenter, tetras);

        //PROCESS : vertex -> incident edges -> dual face
        //first: map dual VoronoiFace to every edge in tetras

        std::map<DelauneyEdge, VoronoiFacet, DelauneyEdgeComparator> delEdgeToVorFacet = 
       delauneyEdgeToVoronoiFacetEquivalence(tetraToCircumcenter, uniqueVoronoiEdges, tetras);

        //build voronoi mesh
        std::vector<VoronoiMesh>voronoiMeshes = buildVoronoiMeshes(outgoingEdgesFromVertex, delEdgeToVorFacet, tetras);

        return voronoiMeshes;
    }

    VoronoiFacet makeFacet(std::set<btVector3, btVector3Comparator> vorFacetVertices, std::set<VoronoiEdge, VoronoiEdgeComparator> voronoiEdges) {
        //for every vertex that forms the facet, find the edges that belong to it
        std::set<VoronoiEdge, VoronoiEdgeComparator> facetEdges;
        for (auto& vertex : vorFacetVertices) {
            for (auto& edge : voronoiEdges) {

                if ((edge.v1 == vertex && vorFacetVertices.find(edge.v2) != vorFacetVertices.end())
                    || (edge.v2 == vertex && vorFacetVertices.find(edge.v1) != vorFacetVertices.end())
                    ) {
                    VoronoiEdge reversedEdge = { edge.v2, edge.v1 };
                    if(facetEdges.find(edge)== facetEdges.end() && facetEdges.find(reversedEdge)==facetEdges.end())
                        facetEdges.insert(edge);
                }
            }
        }

        std::vector<VoronoiEdge> facetEdgesVector;
        for (auto& edge : facetEdges)
            facetEdgesVector.push_back(edge);

        std::vector<btVector3> facetVerticesVector;
        for (btVector3 vertex : vorFacetVertices)
            facetVerticesVector.push_back(vertex);
        VoronoiFacet facet = { facetEdgesVector,  facetVerticesVector };
        return facet;

    }

    void createTetrahedronFromCube(std::vector<btVector3> cubeModelVertices) {
        Tetrahedron initialTetrahedron = CreateTetrahedronAroundCube(cubeModelVertices, glm::vec3(0.5f, 0.5f, 0.5f));

        //override for test
        tetraRigidbodies.erase(tetraRigidbodies.begin(), tetraRigidbodies.end());
        tetrahedrons.erase(tetrahedrons.begin(), tetrahedrons.end());
        rigidbodyToVAO.erase(rigidbodyToVAO.begin(), rigidbodyToVAO.end());
        rigidbodyToTetra.erase(rigidbodyToTetra.begin(), rigidbodyToTetra.end());
        tetraToRigidbody.erase(tetraToRigidbody.begin(), tetraToRigidbody.end());

        btRigidBody* tetraRigidbody = pe.generateMeshRigidbody(
            cubePositions[0], // Use cube position as starting position
            initialTetrahedron.allSingularVertices,
            btVector3(1.0f, 1.0f, 1.0f)
        );

        initialTetrahedron.VAO = createTetrahedronVAO(initialTetrahedron);
        rigidbodyToVAO[tetraRigidbody] = initialTetrahedron.VAO;
        rigidbodyToTetra[tetraRigidbody] = initialTetrahedron;
        tetraToRigidbody[initialTetrahedron] = tetraRigidbody;
        tetraRigidbodies.insert(tetraRigidbody);
        pe.dynamicsWorld->addRigidBody(tetraRigidbody, 1, 1);
   }

 
private:

    PhysicsEngineAbstraction pe;

    void uniqueVerticesFromModel(std::set<btVector3, btVector3Comparator>& uniqueVertices, Model* model) {
        
        for (Mesh mesh : model->meshes) {
            std::set<btVector3, btVector3Comparator> meshVertices;
            uniqueVerticesFromMesh(meshVertices, mesh);

            uniqueVertices.insert(meshVertices.begin(), meshVertices.end());

            meshToVertices[mesh] = meshVertices;
        }
    }

    void uniqueVerticesFromMesh(std::set<btVector3, btVector3Comparator>& uniqueVertices, Mesh mesh) {

        for (auto& vertex : mesh.vertices) {
            btVector3 v = fromVertexToBtVector3(vertex);
            if (uniqueVertices.count(v) != 1) { //If not found add it
                uniqueVertices.insert(v);
            }
        }
    }

    void generateTetraedronFacetsFromMesh(Tetrahedron& tetra, Mesh tetraModel) {
        for (int facet = 0; facet < tetraModel.indices.size() / MESH_FACE_VERTICES; facet++) { //I consider the faces to be triangular //this for should be from [0 to 3]

            TriangleFacet tetrahedronFacet;

            for (int faceVertexIndex = 0; faceVertexIndex < VERTICES_PER_TETRA_FACET; faceVertexIndex++) {
                tetrahedronFacet.vertices.push_back(
                    fromVertexToBtVector3(tetraModel.vertices[tetraModel.indices[faceVertexIndex + (facet * VERTICES_PER_TETRA_FACET)]])
                );
            }
            tetra.facets.push_back(tetrahedronFacet);
        }
    }

    void fillVertexData(std::vector<TriangleFacet> facets, glm::vec3 color, float vertices[]) {
        std::vector<float> verticeAndColorssAsSingleArr;

        for (auto& facet : facets) {
            for (auto& vertex : facet.vertices) {
                std::vector<float> vecComponents = generateVerticesArrayFromBtVector3(vertex);
                verticeAndColorssAsSingleArr.insert(verticeAndColorssAsSingleArr.end(), vecComponents.begin(), vecComponents.end());
            }
        }

        std::vector<float> colorAsFloatVec = { color.r, color.g, color.b };

        for (int i = 0; i < verticeAndColorssAsSingleArr.size();) {
            verticeAndColorssAsSingleArr.insert(verticeAndColorssAsSingleArr.begin() + i+3, colorAsFloatVec.begin(), colorAsFloatVec.end());
            i += 6;
        }
        vectorToFloatArray(verticeAndColorssAsSingleArr, vertices);
    }

    TriangleFacet getOppositeFacetToVertice(Tetrahedron tetra, btVector3 vert) {
        for (auto& facet : tetra.facets) {
            if (std::find(facet.vertices.begin(), facet.vertices.end(), vert) == facet.vertices.end()) {
                return facet;
            }
        }
        throw std::invalid_argument("Strange, didn't find the opposite vertice to this facet in this tetrahedron");
    }

    unsigned int createTetrahedronVAO(Tetrahedron tetra) {
        unsigned int tetraVBO, tetraVAO;
        glGenVertexArrays(1, &tetraVAO);
        glGenBuffers(1, &tetraVBO);

        glBindVertexArray(tetraVAO);

        glBindBuffer(GL_ARRAY_BUFFER, tetraVBO);
        //need to pass this to OpenGL as its simpler to handle strides and stuff
        float vertices[GL_TOTAL_VERTICES_FLOAT_VALUES_PER_TETRA * 2];

        fillVertexData(tetra.facets, tetra.color, vertices);

        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        // position attribute
        glVertexAttribPointer(0, VERTICES_PER_TETRA_FACET, GL_FLOAT, GL_FALSE, (VERTICES_PER_TETRA_FACET * 2) * sizeof(float), (void*)0); // (VERTICES_PER_TETRA_FACET*2) to account for vertex color
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, VERTICES_PER_TETRA_FACET, GL_FLOAT, GL_FALSE, (VERTICES_PER_TETRA_FACET * 2) * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        return tetraVAO;
    }

    //SUPPORTING METHODS FOR VORONOI CONVERSION
    void setupOutgoingEdgesAndCircumcenters(std::map<btVector3, std::vector<DelauneyEdge>, btVector3Comparator>& outgoingEdgesFromVertex, std::map <Tetrahedron, btVector3, TetrahedronComparator>& tetraToCircumcenter, std::vector<Tetrahedron> tetras) {
        for (auto& t : tetras) {
            
            btVector3 circumcenter = getSphereCenter(t.allSingularVertices);
            tetraToCircumcenter.insert({ t, circumcenter });

            for (auto& vertex : t.allSingularVertices) {
                std::vector<DelauneyEdge> outgoingEdges = findOutgoingEdges(tetras, vertex);

                if (outgoingEdgesFromVertex.find(vertex) != outgoingEdgesFromVertex.end())
                    for (auto &outgoingEdge : outgoingEdges)
                        outgoingEdgesFromVertex.at(vertex).push_back(outgoingEdge);
                else
                    outgoingEdgesFromVertex.insert({ vertex, outgoingEdges });
            }

        }
    }

    std::set<VoronoiEdge, VoronoiEdgeComparator> findUniqueVoronoiEdges(std::map <Tetrahedron, btVector3, TetrahedronComparator> tetraToCircumcenter, std::vector<Tetrahedron> tetras) {
        std::set<VoronoiEdge, VoronoiEdgeComparator> uniqueVoronoiEdges;
        for (auto& t : tetras) {
            std::vector<Tetrahedron> neighbours = getNeighbours(tetras, t);
            for (auto& neighbour : neighbours) {
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

        return uniqueVoronoiEdges;
    }

    std::map<DelauneyEdge, VoronoiFacet, DelauneyEdgeComparator> delauneyEdgeToVoronoiFacetEquivalence(
                                                                std::map <Tetrahedron, btVector3, TetrahedronComparator> tetraToCircumcenter,
                                                                std::set<VoronoiEdge, VoronoiEdgeComparator> uniqueVoronoiEdges,
                                                                std::vector<Tetrahedron> tetras) {
        std::map<DelauneyEdge, VoronoiFacet, DelauneyEdgeComparator> delEdgeToVorFacet;
        std::set<DelauneyEdge, DelauneyEdgeComparator> visitedEdges;
        for (auto& t : tetras) {
            for (auto& v1 : t.allSingularVertices) {
                for (auto& v2 : t.allSingularVertices) {
                    if (v1 != v2) {
                        DelauneyEdge edge = { v1, v2 };
                        DelauneyEdge reversedEdge = { v2,v1 };
                        if (visitedEdges.find(edge) == visitedEdges.end() && visitedEdges.find(reversedEdge) == visitedEdges.end()) {
                            visitedEdges.insert(edge);
                            //find all tetrahedra that contain the edge (incident tetrahedra
                            //EDGECASE: one edge is only shared by two tetrahedra -> how to handle this? for now just don't add the facet to the mesh in that case
                            std::vector<Tetrahedron> incidentTetras = getTetrasIncidentToEdge(edge.v1, edge.v2, tetras);
                            //every equivalent of a DelTetrahedron becomes a vertex of the facet
                            std::set<btVector3, btVector3Comparator> vorFacetVertices;
                            for (Tetrahedron incident_tetra : incidentTetras) {
                                vorFacetVertices.insert(tetraToCircumcenter.at(incident_tetra));
                            }
                            VoronoiFacet facet = makeFacet(vorFacetVertices, uniqueVoronoiEdges);
                            delEdgeToVorFacet.insert({ edge, facet });
                        }
                    }
                }
            }
        }
        return delEdgeToVorFacet;
    }

    std::vector<VoronoiMesh> buildVoronoiMeshes(
        std::map<btVector3, std::vector<DelauneyEdge>, btVector3Comparator> outgoingEdgesFromVertex,
        std::map<DelauneyEdge, VoronoiFacet, DelauneyEdgeComparator> delEdgeToVorFacet,
        std::vector<Tetrahedron> tetras) {

        std::vector<VoronoiMesh>voronoiMeshes;
        /* ---> PROVVISORIO <---*/
        glm::vec3 color = tetras[0].color;
        /* ---------------------*/
        std::set<btVector3, btVector3Comparator> alreadyConvertedVertices;
        for (auto& t : tetras) {
            for (auto& v : t.allSingularVertices) {
                if (alreadyConvertedVertices.find(v) == alreadyConvertedVertices.end()) {
                    alreadyConvertedVertices.insert(v);
                    std::vector<VoronoiFacet> facets;
                    std::set<btVector3, btVector3Comparator> uniqueVertices;
                    std::vector<DelauneyEdge> incidentEdges = outgoingEdgesFromVertex.at(v);
                    std::set<btVector3, btVector3Comparator> verticesAsSet;
                    for (auto&& incidentEdge : incidentEdges) {
                        DelauneyEdge reversedIncidentEdge = { incidentEdge.v2, incidentEdge.v1 };
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
                            {},
                            color
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
        }
        return voronoiMeshes;
    }
};