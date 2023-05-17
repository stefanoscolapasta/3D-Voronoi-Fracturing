#pragma once
#include <glm/glm.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>

class MyContactResultCallback : public btCollisionWorld::ContactResultCallback {
public:
    virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0, int partId0, int index0,
        const btCollisionObjectWrapper* colObj1, int partId1, int index1) {

        return 0; // not used in this example
    }
};