#pragma once
#define QUICKHULL_IMPLEMENTATION
#include "quickhull.h"
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
    std::map<btRigidBody*, VoronoiMesh> vorToMesh;
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
        for (auto tetraRb = tetraToRigidbody.begin(); tetraRb != tetraToRigidbody.end(); ++tetraRb)
            tetras.push_back(tetraRb->first);

        Tetrahedron tetraFromWalk = stochasticWalk(tetras, t);

        std::vector<Tetrahedron> newTetrahedrons = flip14(t, tetraFromWalk);

        //flip23(rigidbodyToTetra[toFlip], rigidbodyToTetra[toFlip]);
        //I now remove the original container riFgidbody from the structs, as I will add the tetrahedrons in which it is divided
        //the idea is correct but popping the last element makes no sense; I should use a gerarchical data struct (nested map), so that I have 
        tetraRigidbodies.erase(tetraToRigidbody[tetraFromWalk]);
        pe.dynamicsWorld->removeRigidBody(tetraToRigidbody[tetraFromWalk]); //And remember to remove it from the physics world
        rigidbodyToTetra.erase(tetraToRigidbody[tetraFromWalk]);
        tetraToRigidbody.erase(tetraFromWalk);


        for (auto& newTetrahedron : newTetrahedrons) {
            newTetrahedron.color = tetraFromWalk.color;
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

    std::vector<Tetrahedron> flip23(std::vector<Tetrahedron> tetrahedrons) {
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



    std::vector<Tetrahedron> getNeighbours(std::vector<Tetrahedron> allTetras, Tetrahedron t) {
        std::set<Tetrahedron, TetrahedronComparator> neighbours;

        for (auto& tetra : allTetras) {
            if (tetra.VAO != t.VAO) {
                for (auto& facet : tetra.facets) {
                    for (auto& facetToCompare : t.facets) {
                        if (areTriangleFacetsEqual(facet, facetToCompare)) {

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

    std::vector<VoronoiMesh>  convertToVoronoi(std::vector<Tetrahedron> tetras) {
        std::vector<VoronoiMesh> vorMeshes;
        for (int i = 0; i < tetras.size(); i++) {
            if (tetras[i].VAO == 6) {
                tetras.erase(tetras.begin() + i);
                break;
            }
        }
        glm::vec3 color = tetras[0].color;
        std::map <int, btVector3> tetraToVoronoiVertex;

        setupOutgoingEdgesAndCircumcenters( tetraToVoronoiVertex, tetras);

        std::set<btVector3, btVector3Comparator> allVertices;
        for (auto& tetra : tetras)
            allVertices.insert(tetra.allSingularVertices.begin(), tetra.allSingularVertices.end());
        
         for (auto& vertex : allVertices) {
                std::vector<btVector3> meshVertices;
                std::vector<Tetrahedron> incidentTetras = getTetrasIncidentToVertex(tetras, vertex);
                for (auto& incidentTetra : incidentTetras) {
                        btVector3 equivalentVertex = tetraToVoronoiVertex.at(incidentTetra.VAO);
                        meshVertices.push_back(equivalentVertex);

                }
                const int n = meshVertices.size();
                if (n > 3) {
                    qh_vertex_t* vertices = QH_MALLOC(qh_vertex_t, n);

                    for (int i = 0; i < n; i++) {
                        vertices[i].x = meshVertices[i].getX();
                        vertices[i].y = meshVertices[i].getY();
                        vertices[i].z = meshVertices[i].getZ();
                    }


                    qh_mesh_t mesh = qh_quickhull3d(&vertices[0], n);
                    VoronoiMesh vorMesh = {
                        0,0,0,
                        std::set<btVector3, btVector3Comparator>(meshVertices.begin(), meshVertices.end()),
                        mesh.vertices,
                        mesh.normals,
                        mesh.indices,
                        mesh.normalindices,
                        mesh.nindices,
                        mesh.nvertices,
                        mesh.nnormals,
                        color
                    };
                    vorMeshes.push_back(vorMesh);
                }
                
            
         }

        std::cout << "";
        return vorMeshes;
       
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



    void setupOutgoingEdgesAndCircumcenters(std::map <int, btVector3>& tetraToCircumcenter, std::vector<Tetrahedron> tetras) {
        for (auto& t : tetras) {
            btVector3 circumcenter = getTetrahedronCenter(t);
            tetraToCircumcenter.insert({ t.VAO, circumcenter });
        }
    }

    bool wasFacetVisited(std::vector<TriangleFacet> visitedFacets, TriangleFacet sharedFacet) {
        for (auto& facet : visitedFacets)
            if (areTriangleFacetsEqual(facet, sharedFacet))
                return true;

        return false;
    }

};