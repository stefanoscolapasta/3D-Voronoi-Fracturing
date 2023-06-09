#pragma once
#define QUICKHULL_IMPLEMENTATION
#include "quickhull.h"
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


    PhysicsEngineAbstraction pe;

    //Tetrahedrons data-structs
    std::set<Tetrahedron, TetrahedronComparator> tetrahedrons;
    std::vector<Tetrahedron> initialTetras;
    std::map<unsigned int, std::vector<Tetrahedron>> tetraToNeighbours;
    //Voronoi data-structs
    std::vector<btRigidBody*> vorRigidBodies;
    std::map<btRigidBody*, VoronoiMesh> vorRigidBodyToMesh;

    //The model passed has to be made up of tetrahedrons already
    //That is why initialization must be done through the "big" tetrahedron
    VoronoiFracturing(Model* tetrahedronModel, PhysicsEngineAbstraction& pe, btVector3 startingPosition) : pe(pe) { //Will have to generalize to other shapes

        std::map<Mesh, std::set<btVector3, btVector3Comparator>, MeshComparator> meshToVertices;

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
            initialTetras.push_back(tetra);
            tetrahedrons.insert(tetra);
        }

    };

    VoronoiFracturing(Tetrahedron tetrahedronModel, PhysicsEngineAbstraction& pe) : pe(pe) { //Will have to generalize to other shapes
        initialTetras.push_back(tetrahedronModel);
        tetrahedrons.insert(tetrahedronModel);
    };

    //TODO: tetras dont have to be rigidbodies

    void insertOnePoint(btVector3 t, btVector3 startPos) { //For now no need to implement the walk algorithm, as we try to just insert the point in the main/first tetrahedron

        //I now convert the set<Tetrahedrons, TetrahedronComparator> to a vector<Tetrahedron>

        std::vector<Tetrahedron> tetrahedronsAsVectorForWalk(tetrahedrons.begin(), tetrahedrons.end());

        Tetrahedron tetraFromWalk = stochasticWalk(tetrahedronsAsVectorForWalk, t);

        //they first need to be tested for the delaunay conditions
        std::vector<Tetrahedron> newTetrahedronsIncidentToP = flip14(getTetrahedronCenter(tetraFromWalk), tetraFromWalk, startPos);

        //here, every tetra will be adjacent to another tetra that is not part of the star2 of the just insterted point P
        for (auto& tetra : newTetrahedronsIncidentToP) {
            TriangleFacet oppositeFacet = getOppositeFacetToVertice(tetra, t);

            bool foundToBeSharingFacet = false;
            Tetrahedron neighbour;
            getTetraSharingFacet(oppositeFacet, foundToBeSharingFacet, neighbour);

            if (foundToBeSharingFacet && isPointInsideSphere(tetra, t)) {
                flip(tetra, neighbour, oppositeFacet, startPos, t);
            }

        }

    }

    void removeExtraTetrahedrons() {
        for (auto& initialTetra : initialTetras) {
            for (auto& vertex : initialTetra.allSingularVertices) {
                for (auto& tetra : tetrahedrons) {

                    if (tetra.allSingularVertices.find(vertex) != tetra.allSingularVertices.end()) {
                        tetrahedrons.erase(tetra);
                        break;
                    }
                }
            }

        }
    }

    //-----------------------------------------FLIPS------------------------------------------------------
   //----------------------------------------------------------------------------------------------------



    void flip(Tetrahedron& t1IncidentToP, Tetrahedron& neighbouring, TriangleFacet& oppositeFacet, btVector3 startPos, btVector3& p) {
        //Because verifying if from p two faces are visible or not is expensive, it makes sense to leave the caseTwo as last one

        if (caseOne(neighbouring, p)) {
            flip23({ t1IncidentToP, neighbouring }, startPos);
        }
        else {
            Tetrahedron thirdTetraFoundToFlip;
            if (caseTwo(t1IncidentToP, neighbouring, oppositeFacet, p, thirdTetraFoundToFlip)) {
                flip32({ t1IncidentToP, neighbouring, thirdTetraFoundToFlip }, startPos);
            }
            else {
                bool areCoplanar;
                std::pair<Tetrahedron, Tetrahedron> neighbourCouple = caseThree(t1IncidentToP, neighbouring, oppositeFacet, p, areCoplanar);
                if (areCoplanar) {

                    std::vector<Tetrahedron> firstTetraCoupleToFlip = { t1IncidentToP, neighbouring };
                    std::vector<Tetrahedron> secondTetraCoupleToFlip = { neighbourCouple.first , neighbourCouple.second };
                    flip44(firstTetraCoupleToFlip, secondTetraCoupleToFlip, startPos);

                }
                else if (caseFour(t1IncidentToP, neighbouring, oppositeFacet, p)) {
                    flip23({ t1IncidentToP, neighbouring }, startPos);
                }
            }
        }


    }

    std::pair<int, std::vector<TriangleFacet>> neighbourTetraFacetsVisibleFromP(btVector3& p, Tetrahedron& neighbour) {
        int numberOfVisibleFacets = 0;
        std::vector<TriangleFacet> visibleFacets;
        for (auto& facet : neighbour.facets) {
            if (!isVectorPassingThroughFacet(p, facet.getCenter(), facet)) {
                numberOfVisibleFacets += 1;
                visibleFacets.push_back(facet);
            }
        }
        return std::make_pair(numberOfVisibleFacets, visibleFacets);
    }

    void getTetraSharingFacet(TriangleFacet& f, bool& found, Tetrahedron& neighbour) {
        for (auto& tetra : tetrahedrons) {
            int count = 0;
            for (auto& vertex : f.vertices) {
                if (tetra.allSingularVertices.find(vertex) == tetra.allSingularVertices.end()) {
                    break;
                }
                else {
                    count += 1;
                }
            }
            if (count == VERTICES_PER_TETRA_FACET) {
                neighbour = tetra;
                found = true;
            }

        }
        found = false;
    }

    bool caseOne(Tetrahedron& neighbour, btVector3& p) {
        return neighbourTetraFacetsVisibleFromP(p, neighbour).first == 1;
    }

    bool caseTwo(Tetrahedron& incidentToP, Tetrahedron& neighbour, TriangleFacet& sharedFacet, btVector3& p, Tetrahedron& thirdTetraFoundToFlip) {
        std::pair<int, std::vector<TriangleFacet>> visibilityInformation = neighbourTetraFacetsVisibleFromP(p, neighbour);
        if (visibilityInformation.first == 2) {
            for (auto& facet : visibilityInformation.second) {
                std::set<btVector3, btVector3Comparator> verticesOfPossibleTetra({ facet.vertices[0], facet.vertices[1], facet.vertices[2], p });
                for (auto& tetra : tetrahedrons) {
                    if (tetra.allSingularVertices == verticesOfPossibleTetra) {
                        thirdTetraFoundToFlip = tetra;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    //TODO: not really useful to return a pair, can simply return a vector
    std::pair<Tetrahedron, Tetrahedron> caseThree(Tetrahedron& t1Incident, Tetrahedron& t2, TriangleFacet& sharedFacet, btVector3& p, bool& areCoplanar) {
        btVector3 oppositeVertex = getOppositeVerticeToFacet(t2, sharedFacet);

        TriangleFacet f1WithP;
        TriangleFacet f2WithOppositeToP;

        bool isFirstCoplanar = arePointsCoplanar(p, sharedFacet.vertices[0], sharedFacet.vertices[1], oppositeVertex);
        bool isSecondCoplanar = false;
        bool isThirdCoplanar = false;

        if (isFirstCoplanar) {
            f1WithP = { std::vector<btVector3>({p, sharedFacet.vertices[0], sharedFacet.vertices[1]}) };
            f2WithOppositeToP = { std::vector<btVector3>({oppositeVertex, sharedFacet.vertices[0], sharedFacet.vertices[1]}) };
        }
        else {
            isSecondCoplanar = arePointsCoplanar(p, sharedFacet.vertices[1], sharedFacet.vertices[2], oppositeVertex);
            if (isSecondCoplanar) {
                f1WithP = { std::vector<btVector3>({p, sharedFacet.vertices[1], sharedFacet.vertices[2]}) };
                f2WithOppositeToP = { std::vector<btVector3>({oppositeVertex, sharedFacet.vertices[1], sharedFacet.vertices[2]}) };
            }
            else {
                isThirdCoplanar = arePointsCoplanar(p, sharedFacet.vertices[2], sharedFacet.vertices[0], oppositeVertex);
                if (isThirdCoplanar) {
                    f1WithP = { std::vector<btVector3>({p, sharedFacet.vertices[2], sharedFacet.vertices[0]}) };
                    f2WithOppositeToP = { std::vector<btVector3>({oppositeVertex, sharedFacet.vertices[2], sharedFacet.vertices[0]}) };
                }
            }

        }

        bool isThereCoplanarity = isFirstCoplanar || isSecondCoplanar || isThirdCoplanar;

        if (!isThereCoplanarity) {
            areCoplanar = false;
            Tetrahedron empty;
            return std::make_pair(empty, empty);
        }
        else {
            Tetrahedron neighbour1WithP;
            bool found1;
            getTetraSharingFacet(f1WithP, found1, neighbour1WithP);
            if (!found1) {
                throw std::invalid_argument("Something went wrong: the passed facet has no neighbouring facets");
            }

            Tetrahedron neighbour2WithOppositeToP;
            bool found2;
            getTetraSharingFacet(f2WithOppositeToP, found2, neighbour2WithOppositeToP);
            if (!found2) {
                throw std::invalid_argument("Something went wrong: the passed facet has no neighbouring facets");
            }

            areCoplanar = true;
            return std::make_pair(neighbour1WithP, neighbour2WithOppositeToP);

        }

    }

    bool caseFour(Tetrahedron& t1Incident, Tetrahedron& t2, TriangleFacet& sharedFacet, btVector3& p) {
        btVector3 d = getOppositeVerticeToFacet(t2, sharedFacet);
        bool coplanarToAnyLateralFacet =
            arePointsCoplanar(sharedFacet.vertices[0], sharedFacet.vertices[1], d, p) ||
            arePointsCoplanar(sharedFacet.vertices[1], sharedFacet.vertices[2], d, p) ||
            arePointsCoplanar(sharedFacet.vertices[2], sharedFacet.vertices[0], d, p);

        //This way it means that it is on an edge of the shared facet
        return arePointsCoplanar(sharedFacet.vertices[0], sharedFacet.vertices[1], sharedFacet.vertices[2], p) && coplanarToAnyLateralFacet;
    }

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
            completeAndInsertTetrahedron(newInsertedTetrahedron, startPos);
            newTetrahedrons.push_back(newInsertedTetrahedron);

        }

        //I need to remove the original tetrahedron as it is now substituted by the 4 new tetras
        tetrahedrons.erase(tetrahedron);

        return newTetrahedrons;
    }



    std::vector<Tetrahedron> flip23(std::vector<Tetrahedron> tetrasToFlip, btVector3 startPos) {
        bool haveOneSameFacet = false;
        TriangleFacet sameFacet;

        for (auto& facet1 : tetrasToFlip[0].facets) {
            std::set<btVector3, btVector3Comparator> s1(facet1.vertices.begin(), facet1.vertices.end());
            for (auto& facet2 : tetrasToFlip[1].facets) {
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
            btVector3 v1 = getOppositeVerticeToFacet(tetrasToFlip[0], sameFacet);
            btVector3 v2 = getOppositeVerticeToFacet(tetrasToFlip[1], sameFacet);
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
                completeAndInsertTetrahedron(newTetra, startPos);

                flippedTetrahedrons.push_back(newTetra);
            }

            for (auto& t : tetrasToFlip) {
                tetrahedrons.erase(t);
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
                completeAndInsertTetrahedron(newTetra, startPos);
                newTetras.push_back(newTetra);
            }

            for (auto& t : tetrasToFlip) {
                tetrahedrons.erase(t);
            }

            return newTetras;
        }

        throw std::invalid_argument("Something went wrong: the passed tetrahedron do not share a facet");
    }

    std::pair<std::vector<Tetrahedron>, std::vector<Tetrahedron>> flip44(std::vector<Tetrahedron> firstCoupleToFlip, std::vector<Tetrahedron> secondCoupleToFlip, btVector3 startPos) {
        //flip44 is a combination of a flip23 followed by a flip32
        std::vector<Tetrahedron> flippedWithDegenerateTetraFirstCouple = flip23(firstCoupleToFlip, startPos);
        std::vector<Tetrahedron> flippedFirstCouple = flip32(flippedWithDegenerateTetraFirstCouple, startPos);

        std::vector<Tetrahedron> flippedWithDegenerateTetraSecondCouple = flip23(secondCoupleToFlip, startPos);
        std::vector<Tetrahedron> flippedSecondCouple = flip32(flippedWithDegenerateTetraSecondCouple, startPos);

        return std::make_pair(flippedFirstCouple, flippedSecondCouple);
    }

    //-----------------------------------------END FLIPS--------------------------------------------------
    //----------------------------------------------------------------------------------------------------




    void completeAndInsertTetrahedron(Tetrahedron& tetra, btVector3 startPos) {
        tetra.color = glm::vec3(1, 1, 1);
        tetra.VAO = createTetrahedronVAO(tetra);
        tetrahedrons.insert(tetra);
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

    std::vector<VoronoiMesh>  convertToVoronoi( btVector3 meshCenter) {
        std::vector<Tetrahedron> tetras = std::vector<Tetrahedron>(tetrahedrons.begin(), tetrahedrons.end());
        std::vector<VoronoiMesh> vorMeshes;
        std::map <int, btVector3> tetraToVoronoiVertex;
        std::set<btVector3, btVector3Comparator> allVertices;
        setupCircumcentersAndVertices(tetraToVoronoiVertex, tetras, &allVertices);

        std::map < btVector3, std::vector<btVector3>, btVector3Comparator> vertexToMeshVertices;
        for (auto& tetra : tetras) {
            for (auto& vertex : tetra.allSingularVertices) {
                    btVector3 equivalentVertex = tetraToVoronoiVertex.at(tetra.VAO);
                    vertexToMeshVertices[vertex].push_back(equivalentVertex);

            }
        }

        
         for (auto& vertex : allVertices) {
             std::vector<btVector3> meshVertices = vertexToMeshVertices[vertex];
                 
                 const int n = meshVertices.size();
                 if (n >3 ) {
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
                     };
                     double meshVolume = calculateBoundingBoxVolume(vorMesh);
                     if (meshVolume < 20.0) {
                         btRigidBody* vorRigidBody = addVoronoiRigidBody(pe, vorMesh, meshCenter);
                         btTransform centerOfMassTransform;
                         centerOfMassTransform.setIdentity();
                         btVector3 centerOfMassPosition(getVoronoiMeshCenter(vorMesh));
                         centerOfMassTransform.setOrigin(meshCenter);
                         vorRigidBody->setCenterOfMassTransform(centerOfMassTransform);
                         vorRigidBodies.push_back(vorRigidBody);
                         vorRigidBodyToMesh[vorRigidBody] = vorMesh;
                         vorMeshes.push_back(vorMesh);
                     }

                 }
                
         }

        std::cout << "";

        return vorMeshes;
       
    }

    void uniqueVerticesFromModel(std::set<btVector3, btVector3Comparator>& uniqueVertices, Model model) {

        for (Mesh mesh : model.meshes) {
            std::set<btVector3, btVector3Comparator> meshVertices;
            uniqueVerticesFromMesh(meshVertices, mesh);

            uniqueVertices.insert(meshVertices.begin(), meshVertices.end());
        }
    }
private:

    

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

    Tetrahedron stochasticWalk(std::vector<Tetrahedron> tetras, btVector3 p) {
        Tetrahedron father;
        
        int randomIndex_t = std::rand() % tetras.size(); //random index between 0 and size of tetras

        Tetrahedron t = tetras.at(randomIndex_t);
        Tetrahedron previous = t;
        bool end = false;
        std::set<int> tetraPath = {};
        while (!end) {
            verifyNeighbours(tetras, &previous, &t, p, end, &tetraPath);
        }
        if (!isPointInsideTetrahedron(t, p))
            std::cout << "Wrong result!" << endl;
        return t;
    }

    void verifyNeighbours(std::vector<Tetrahedron> tetras, Tetrahedron* previous, Tetrahedron* t, btVector3 p, bool& end, std::set<int> *tetraPath) {
        if (isPointInsideTetrahedron(*t, p)) {
            end = true;
            return;
        }
        std::vector<Tetrahedron> t_neighbours;
        if(tetraToNeighbours.find(t-> VAO )!= tetraToNeighbours.end())
            t_neighbours = tetraToNeighbours[t->VAO];
        else {
            t_neighbours = getNeighbours(tetras, *t);
            tetraToNeighbours[t->VAO] = t_neighbours;
        }
            
        Tetrahedron neighbour_through_f;
        TriangleFacet f;
        //edge <-> facet
        // triangle <-> tetrahedron

        int randomIndex_f = std::rand() % 4; //random index from 0 to 2- every tetra has 4 fixed facets
        int facetIndex = randomIndex_f;

        for (int facetCount = 0; facetCount < 4; facetCount++) {
            f = t->facets[(randomIndex_f + facetCount)%4];
            if (verifyFacetOrientationConditions(t, f, p, tetras)) {
                updatePath(t_neighbours, f, neighbour_through_f, previous, t, tetraPath);
                return;
            }
        }
        end = true;
    }

    bool verifyFacetOrientationConditions(Tetrahedron *t, TriangleFacet f, btVector3 p,  std::vector<Tetrahedron> tetras) {


        //CONDITION: is p on the other side of f respect to the tetra's center?
        bool onDifferentSide = false;

        std::vector<btVector3> sortedVertices = sortFacetVerticesCounterClockwise(f.vertices);


            float a1 = sortedVertices[1].getX() - sortedVertices[0].getX();
            float b1 = sortedVertices[1].getY() - sortedVertices[0].getY();
            float c1 = sortedVertices[1].getZ() - sortedVertices[0].getZ();
            float a2 = sortedVertices[2].getX() - sortedVertices[0].getX();
            float b2 = sortedVertices[2].getY() - sortedVertices[0].getY();
            float c2 = sortedVertices[2].getZ() - sortedVertices[0].getZ();
            float a = b1 * c2 - b2 * c1;
            float b = a2 * c1 - a1 * c2;
            float c = a1 * b2 - b1 * a2;
            float d = (-a * sortedVertices[0].getX() - b * sortedVertices[0].getY() - c * sortedVertices[0].getZ());


        btVector3 tetraCenter = getTetrahedronCenter(*t);

        // Put coordinates in plane equation
        float value_1 = a * p.getX() + b * p.getY() + c * p.getZ() + d;
        float value_2 = a * tetraCenter.getX() + b * tetraCenter.getY() + c * tetraCenter.getZ() + d;


        // If both values have different sign
        if (value_1*value_2 < 0.0)
            onDifferentSide = true;

        //we don't need to check the "point in neighbour through f" condition, because in that case the function would have returned.
        return onDifferentSide;

    }

    void updatePath(std::vector<Tetrahedron>& t_neighbours, TriangleFacet& f, Tetrahedron& neighbour_through_f, Tetrahedron* previous, Tetrahedron* t, std::set<int>* tetraPath)
    {
        for (auto& neighbour : t_neighbours) {
            if (isFacetInTetrahedron(neighbour, f)) {
                neighbour_through_f = neighbour;
                break;
            }
        }

        if (tetraPath->find(neighbour_through_f.VAO) != tetraPath->end()) {
            int randomIndex;
            do {
                randomIndex = std::rand() % t_neighbours.size();
            } while (t_neighbours.at(randomIndex).VAO == neighbour_through_f.VAO);
          
            neighbour_through_f = t_neighbours.at(randomIndex); //edgecase of circular walking due to point being on plane of shared face.
        }
        *previous = *t;
        *t = neighbour_through_f;
        tetraPath->insert(t->VAO);
    }

    



    void setupCircumcentersAndVertices( std::map <int, btVector3>& tetraToCircumcenter, std::vector<Tetrahedron> tetras,
        std::set<btVector3, btVector3Comparator> *allVertices) {
        for (auto& tetra : tetras) {
                allVertices->insert(tetra.allSingularVertices.begin(), tetra.allSingularVertices.end());
                btVector3 circumcenter = getTetrahedronCenter(tetra);
                tetraToCircumcenter.insert({ tetra.VAO, circumcenter });
        }
    }

    bool wasFacetVisited(std::vector<TriangleFacet> visitedFacets, TriangleFacet sharedFacet) {
        for (auto& facet : visitedFacets)
            if (areTriangleFacetsEqual(facet, sharedFacet))
                return true;

        return false;
    }

};