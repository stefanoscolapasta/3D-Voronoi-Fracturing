#pragma once
#include <glm/glm.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include "utils.h"
#include "tetrahedron.h"
#include "mesh.h"
#include "voronoi.h"
#include "model.h"
#include "physicsEngine.h"
#include "defines.h"



class VoronoiFracturing {

public:

    std::vector<Model*> tetrahedronsModelsVector; //Its a vector of arrays of btVector3
    std::set<btRigidBody*> tetraRigidbodies;
    std::vector<Tetrahedron> tetrahedrons;
    std::map<btRigidBody*, unsigned int> rigidbodyToVAO;
    std::map<btRigidBody*, Tetrahedron> rigidbodyToTetra;
    std::map<Tetrahedron, btRigidBody*,TetrahedronComparator> tetraToRigidbody;
    PhysicsEngineAbstraction pe;

    std::vector<btRigidBody*> vorRigidBodies;
    std::map<btRigidBody*, unsigned int> vorToVAO;
    std::map<btRigidBody*, int> vorToNumVertices;
    std::map<btRigidBody*, std::vector<unsigned int>> vorToIndices;
    std::map<Mesh, std::set<btVector3, btVector3Comparator>, MeshComparator> meshToVertices; //TODO: this field is honestly useless here

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

    VoronoiFracturing(Tetrahedron tetrahedronModel, PhysicsEngineAbstraction& pe, btVector3 startingPosition) : pe(pe) { //Will have to generalize to other shapes

            btRigidBody* meshRigidbody = pe.generateMeshRigidbody(
                startingPosition, // Use cube position as starting position
                tetrahedronModel.allSingularVertices,
                btVector3(1.0f, 1.0f, 1.0f)
            );

            rigidbodyToVAO[meshRigidbody] = tetrahedronModel.VAO;
            rigidbodyToTetra[meshRigidbody] = tetrahedronModel;
            tetraToRigidbody[tetrahedronModel] = meshRigidbody;
            tetraRigidbodies.insert(meshRigidbody);
            pe.dynamicsWorld->addRigidBody(meshRigidbody, 1, 1);
    };

    void insertOnePoint(btVector3 t, btVector3 startPos) { //For now no need to implement the walk algorithm, as we try to just insert the point in the main/first tetrahedron
        std::vector<Tetrahedron> tetras;

        for (auto tetraRb = tetraToRigidbody.begin(); tetraRb != tetraToRigidbody.end(); ++tetraRb)
            tetras.push_back(tetraRb->first);

        Tetrahedron tetraFromWalk = stochasticWalk(tetras, t);
              
        std::vector<Tetrahedron> newTetrahedrons = flip14(getTetrahedronCenter(tetraFromWalk), tetraFromWalk, startPos);
        
    }


    //-----------------------------------------FLIPS------------------------------------------------------
    //----------------------------------------------------------------------------------------------------

    std::vector<Tetrahedron> flip14(btVector3 t, Tetrahedron tetrahedron, btVector3 startPos) {
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
            generateAndCompleteTetrahedron(newInsertedTetrahedron, startPos);
            newTetrahedrons.push_back(newInsertedTetrahedron);
        }

        tetraRigidbodies.erase(tetraToRigidbody[tetrahedron]);
        pe.dynamicsWorld->removeRigidBody(tetraToRigidbody[tetrahedron]); //And remember to remove it from the physics world
        rigidbodyToTetra.erase(tetraToRigidbody[tetrahedron]);
        tetraToRigidbody.erase(tetrahedron);

        return newTetrahedrons;
    }



    std::vector<Tetrahedron> flip23(std::vector<Tetrahedron> tetrahedrons, btVector3 startPos) {
        bool haveOneSameFacet = false;
        TriangleFacet sameFacet;

        for (auto& facet1 : tetrahedrons[0].facets) {
            std::set<btVector3, btVector3Comparator> s1(facet1.vertices.begin(), facet1.vertices.end());
            for (auto& facet2 : tetrahedrons[1].facets) {
                std::set<btVector3, btVector3Comparator> s2(facet2.vertices.begin(), facet2.vertices.end());
                s2.insert(s1.begin(), s1.end());
                if (s2.size() == 3) {
                    haveOneSameFacet = true;
                    sameFacet = facet1;
                    break;
                }
            }
            if (haveOneSameFacet) {
                break;
            }
        }

        if (haveOneSameFacet) {
            btVector3 v1 = getOppositeVerticeToFacet(tetrahedrons[0], sameFacet);
            btVector3 v2 = getOppositeVerticeToFacet(tetrahedrons[1], sameFacet);
            std::vector<Tetrahedron> flippedTetrahedrons;
            //Here I create the 3 new tetras
            for (int i = 0; i < 3; i++) {
                TriangleFacet facet1 = {
                { v1, v2,  sameFacet.vertices[i % 3]}
                };
                TriangleFacet facet2 = {
                { v1, v2,  sameFacet.vertices[(i + 1) % 3]}
                };
                TriangleFacet facet3 = {
                { v1, sameFacet.vertices[i % 3], sameFacet.vertices[(i + 1) % 3]}
                };
                TriangleFacet facet4 = {
                { v2, sameFacet.vertices[i % 3], sameFacet.vertices[(i + 1) % 3]}
                };

                std::vector<TriangleFacet> facets = { facet1, facet2, facet3, facet4 };

                Tetrahedron newTetra;
                generateTetrahedronFromFacets(newTetra, facets);
                generateAndCompleteTetrahedron(newTetra, startPos);

                flippedTetrahedrons.push_back(newTetra);
            }

            for (auto& t : tetrahedrons) {
                tetraRigidbodies.erase(tetraToRigidbody[t]);
                pe.dynamicsWorld->removeRigidBody(tetraToRigidbody[t]); //And remember to remove it from the physics world
                rigidbodyToTetra.erase(tetraToRigidbody[t]);
                tetraToRigidbody.erase(t);
            }

            return flippedTetrahedrons;
        }

        throw std::invalid_argument("Something went wrong: the passed tetrahedron do not share a facet");
    }



    std::vector<Tetrahedron> flip32(std::vector<Tetrahedron> tetrasToFlip, btVector3 startPos) {
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

        for (std::map<btVector3, int, btVector3Comparator>::iterator it = vertices.begin(); it != vertices.end(); ++it) {
            if (it->second == 3) {
                found += 1;
                commonToAll.push_back(it->first);
            }
        }

        haveOne2CommonVertices = found == 2;

        if (haveOne2CommonVertices) {
            //I get all the singular vertices
            std::set<btVector3, btVector3Comparator> allSingularVerticesAcross3Tetras;
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
                generateAndCompleteTetrahedron(newTetra, startPos);
                newTetras.push_back(newTetra);
            }

            for (auto& t : tetrasToFlip) {
                tetraRigidbodies.erase(tetraToRigidbody[t]);
                pe.dynamicsWorld->removeRigidBody(tetraToRigidbody[t]); //And remember to remove it from the physics world
                rigidbodyToTetra.erase(tetraToRigidbody[t]);
                tetraToRigidbody.erase(t);
            }

            return newTetras;
        }

        throw std::invalid_argument("Something went wrong: the passed tetrahedron do not share a facet");
    }

    //-----------------------------------------END FLIPS--------------------------------------------------
    //----------------------------------------------------------------------------------------------------




    void generateAndCompleteTetrahedron(Tetrahedron& tetra, btVector3 startPos) {
        tetra.color = glm::vec3(1, 1, 1);
        tetra.VAO = createTetrahedronVAO(tetra);

        tetrahedrons.push_back(tetra);

        btRigidBody* tetraRigidbody = pe.generateMeshRigidbody(
            startPos, // Use cube position as starting position
            tetra.allSingularVertices,
            btVector3(1.0f, 1.0f, 1.0f)
        );

        rigidbodyToVAO[tetraRigidbody] = tetra.VAO;
        rigidbodyToTetra[tetraRigidbody] = tetra;
        tetraToRigidbody[tetra] = tetraRigidbody;
        tetraRigidbodies.insert(tetraRigidbody);
        pe.dynamicsWorld->addRigidBody(tetraRigidbody, 1, 1);
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
            if (std::find(facet.vertices.begin(), facet.vertices.end(), vertex) == facet.vertices.end()) {
                return vertex;
            }
        }
        throw std::invalid_argument("Strange, didn't find the opposite vertice to this facet in this tetrahedron");
    }


    std::vector<Tetrahedron> getNeighbours(std::vector<Tetrahedron> allTetras, Tetrahedron t) {
        std::set<Tetrahedron, TetrahedronComparator> neighbours;

        for (auto& tetra : allTetras) {

            for (auto& facet : tetra.facets) {
                for (auto& facetToCompare : t.facets) {
                    if (areTriangleFacetsEqual(facet, facetToCompare)) {
                        if (tetra.VAO != t.VAO) {
                            neighbours.insert(tetra);
                            break;  // Found a shared facet, move to the next tetrahedron
                        }
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

    Tetrahedron stochasticWalk(std::vector<Tetrahedron> tetras, btVector3 p) {
        if (isPointVertex(tetras, p))
            return findVertexFatherWithLeastNeighbours(tetras, p);
        if (isPointOnAnEdge(tetras, p))
            return findEdgeFatherWithLeastNeighbours(tetras, p);
        if (isPointOnAFace(tetras, p))
            return findFaceFatherWithLeastNeighbours(tetras, p);
        int randomIndex_t = std::rand() % tetras.size(); //random index between 0 and size of tetras
        // check indices 0(VAO=3) and 4 (VAO=9)
        Tetrahedron t = tetras.at(0);
        Tetrahedron previous = t;
        bool end = false;
        TriangleFacet f;
        while (!end) {
            verifyNeighbours(tetras, f, &previous, &t, p, end);
        }

        return t;
    }

    std::vector<VoronoiMesh> convertToVoronoi(std::vector<Tetrahedron> tetras) {
       
        std::map<btVector3, std::vector<DelauneyEdge>, btVector3Comparator> outgoingEdgesFromVertex;
        std::map <Tetrahedron, btVector3, TetrahedronComparator> tetraToVoronoiVertex;
        //find equivalent Voronoi vertex for each Delauney tetrahedron

        for (auto& t : tetras){
            btVector3 circumcenter = getSphereCenter(t.allSingularVertices);
            tetraToVoronoiVertex.insert({ t, circumcenter });
        }

        std::set<VoronoiEdge, VoronoiEdgeComparator> uniqueVoronoiEdges = findUniqueVoronoiEdges(tetraToVoronoiVertex, tetras);

        std::set<btVector3, btVector3Comparator> convertedVertices;
        for (auto& tetra : tetras) {
            for (auto& vertex : tetra.allSingularVertices) {
                if (convertedVertices.find(vertex) == convertedVertices.end()) {
                    convertedVertices.insert(vertex);
                }
            }
       }

        std::map<DelauneyEdge, VoronoiFacet, DelauneyEdgeComparator> delEdgeToVorFacet =
            delauneyEdgeToVoronoiFacetEquivalence(tetraToVoronoiVertex, uniqueVoronoiEdges, tetras);

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

        //TODO: Temporary retransform to vec, awaiting for voronoi.h fix push. Remove later.
        std::vector<btVector3> vorFacetVerticesVec(vorFacetVertices.begin(), vorFacetVertices.end());
        
        VoronoiFacet facet = { facetEdgesVector,  vorFacetVerticesVec };
        return facet;

    }

    void createTetrahedronFromCube(std::vector<btVector3> cubeModelVertices) {
        Tetrahedron initialTetrahedron = CreateTetrahedronAroundShape(cubeModelVertices, glm::vec3(0.5f, 0.5f, 0.5f));

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



    TriangleFacet getOppositeFacetToVertice(Tetrahedron tetra, btVector3 vert) {
        for (auto& facet : tetra.facets) {
            if (std::find(facet.vertices.begin(), facet.vertices.end(), vert) == facet.vertices.end()) {
                return facet;
            }
        }
        throw std::invalid_argument("Strange, didn't find the opposite vertice to this facet in this tetrahedron");
    }

    




    Tetrahedron findVertexFatherWithLeastNeighbours(std::vector<Tetrahedron> tetras, btVector3 p) {
        Tetrahedron fatherWithLeastNeighbours;
        int minNumNeighbours = INT_MAX;
        for (auto& tetra : tetras) {
            std::vector<Tetrahedron> neighbours = getNeighbours(tetras, tetra);
            int numNeighbours = neighbours.size();
            if ((numNeighbours < minNumNeighbours) &&
                std::find(tetra.allSingularVertices.begin(), tetra.allSingularVertices.end(), p) != tetra.allSingularVertices.end()) {
                minNumNeighbours = numNeighbours;
                fatherWithLeastNeighbours = tetra;
            }
               
        }
        return fatherWithLeastNeighbours;
    }

    Tetrahedron findEdgeFatherWithLeastNeighbours(std::vector<Tetrahedron> tetras, btVector3 p) {
        Tetrahedron fatherWithLeastNeighbours;
        int minNumNeighbours = INT_MAX;
        for (auto& tetra : tetras) {
            std::vector<Tetrahedron> neighbours = getNeighbours(tetras, tetra);
            int numNeighbours = neighbours.size();
            if ((numNeighbours < minNumNeighbours) && isPointOnEdgeOfTetra(tetra, p)){
                minNumNeighbours = numNeighbours;
                fatherWithLeastNeighbours = tetra;
            }

        }
        return fatherWithLeastNeighbours;
    }


    Tetrahedron findFaceFatherWithLeastNeighbours(std::vector<Tetrahedron> tetras, btVector3 p) {
        Tetrahedron fatherWithLeastNeighbours;
        int minNumNeighbours = INT_MAX;
        for (auto& tetra : tetras) {
            std::vector<Tetrahedron> neighbours = getNeighbours(tetras, tetra);
            int numNeighbours = neighbours.size();
            if ((numNeighbours < minNumNeighbours) && isPointOnFaceOfTetra(tetra, p)) {
                minNumNeighbours = numNeighbours;
                fatherWithLeastNeighbours = tetra;
            }

        }
        return fatherWithLeastNeighbours;
    }


    void verifyNeighbours(std::vector<Tetrahedron> tetras, TriangleFacet f, Tetrahedron* previous, Tetrahedron* t, btVector3 p, bool& end) {


        if (isPointInsideTetrahedron(*t, p)) {
            end = true;
            return;
        }

        //edge <-> facet
        // triangle <-> tetrahedron
        int randomIndex_f = std::rand() % 4; //random index from 0 to 2- every tetra has 4 fixed facets

        std::vector<Tetrahedron> t_neighbours = getNeighbours(tetras, *t);
        //check if p is inside one of the neighbours
        if (previous->VAO == t->VAO) {
            for (auto& neighbour : t_neighbours)
                if (isPointInsideTetrahedron(neighbour, p)) {
                    *t = neighbour;
                    end = true;
                    return;
                }
        }

        Tetrahedron neighbour_through_f;
        f = t->facets[randomIndex_f];
        if (verifyNeighbourConditions(f, p, *previous, tetras)) {
            previous = t;
            for (auto& neighbour : t_neighbours)
                if (isFacetInTetrahedron(neighbour, f)) {
                    neighbour_through_f = neighbour;
                    break;
                }
                    
            *t = neighbour_through_f;
        }
        //point is neighbour of "previous" through facet f
        else {
            f = t->facets[(randomIndex_f + 1) % 4];
            if (verifyNeighbourConditions(f, p, *previous, tetras)) {
                previous = t;
                for (auto& neighbour : t_neighbours)
                    if (isFacetInTetrahedron(neighbour, f)) {
                        neighbour_through_f = neighbour;
                        break;
                    }
                *t = neighbour_through_f;
            }
            else {
                f = t->facets[(randomIndex_f + 2) % 4];
                if (verifyNeighbourConditions(f, p, *previous, tetras)) {
                    previous = t;
                    for (auto& neighbour : t_neighbours)
                        if (isFacetInTetrahedron(neighbour, f)) {
                            neighbour_through_f = neighbour;
                            break;
                        }
                    *t = neighbour_through_f;
                }
                else
                {
                    f = t->facets[(randomIndex_f + 3) % 4];
                    if (verifyNeighbourConditions(f, p, *previous, tetras)) {
                        previous = t;
                        for (auto& neighbour : t_neighbours)
                            if (isFacetInTetrahedron(neighbour, f)) {
                                neighbour_through_f = neighbour;
                                break;
                            }
                        *t = neighbour_through_f;
                    }
                    else {
                        
                        end = true;
                    }

                }
            }
        }
    }

    bool verifyNeighbourConditions(TriangleFacet f, btVector3 p, Tetrahedron previous, std::vector<Tetrahedron> tetras) {
        bool isPointNotInNeighgbourThroughF = true;

        //if p is not in neighbour of previous through f
        // and p is on the other side of f respect to this previous's center

        //FIRST CONDITION: is p neighbor of previous through f?
        std::vector<Tetrahedron> previous_neighbours = getNeighbours(tetras, previous);
        for (auto& neighbour : previous_neighbours) {
            if (isFacetInTetrahedron(neighbour, f) && isPointInsideTetrahedron(neighbour, p))
                isPointNotInNeighgbourThroughF = false;
        }

        //SECOND CONDITION: is p on the other side of f respect to previous's center?
        bool onDifferentSide = false;

        std::vector<btVector3> sortedVertices = sortFacetVerticesCounterClockwise(f.vertices);
        //where is the center positioned in space respect to the facet we are considering
        int centerOrientation = orient(sortedVertices[0], sortedVertices[1], sortedVertices[2], getTetrahedronCenter(previous));
        //same thing for point
        int pointOrientation = orient(sortedVertices[0], sortedVertices[1], sortedVertices[2], p);
        //if point is not in the same side of center respect to the facet, the two orientations
        //will have different signs -> negative product
        if (centerOrientation * pointOrientation < 0)
            onDifferentSide = true;

        return (isPointNotInNeighgbourThroughF && onDifferentSide);
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

    std::set<VoronoiEdge, VoronoiEdgeComparator> findUniqueVoronoiEdges(std::map <Tetrahedron, btVector3, TetrahedronComparator> tetraToCircumcenter, std::vector<Tetrahedron> tetras) {
        std::set<VoronoiEdge, VoronoiEdgeComparator> uniqueVoronoiEdges;
        std::vector<TriangleFacet> visitedFacets;
        for (auto& t : tetras) {
            std::vector<Tetrahedron> neighbours = getNeighbours(tetras, t);
            for (auto& neighbour : neighbours) {
                //facet shared by the adjacent tetrahedra
                TriangleFacet sharedFacet = findSharedFacet(t, neighbour);
                if (!wasFacetVisited(visitedFacets, sharedFacet)) {
                    visitedFacets.push_back(sharedFacet);
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
        }

        return uniqueVoronoiEdges;
    }

    bool wasFacetVisited(std::vector<TriangleFacet> visitedFacets, TriangleFacet sharedFacet) {
        for (auto& facet : visitedFacets)
            if (areTriangleFacetsEqual(facet, sharedFacet))
                return true;

        return false;
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
                    //ERROR HERE
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