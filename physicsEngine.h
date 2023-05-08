#pragma once
#include <glm/glm.hpp>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>

class PhysicsEngineAbstraction {
	//TODO: HAVE A .HPP for declaration and a .CPP for the methods' IMPLEMENTATION
	private:
		btBroadphaseInterface* broadphase;
		btDefaultCollisionConfiguration* collisionConfiguration;
		btCollisionDispatcher* dispatcher;
		btSequentialImpulseConstraintSolver* solver;
		
		inline void fillMat4(glm::mat4& mat1, btScalar* arr) {
			mat1 = glm::mat4(
				arr[0], arr[1], arr[2], arr[3],
				arr[4], arr[5], arr[6], arr[7],
				arr[8], arr[9], arr[10], arr[11],
				arr[12], arr[13], arr[14], arr[15]
			);
		}

	public:

		btDiscreteDynamicsWorld* dynamicsWorld;

		PhysicsEngineAbstraction() : broadphase(new btDbvtBroadphase()),
			collisionConfiguration(new btDefaultCollisionConfiguration()), 
			dispatcher(new btCollisionDispatcher(collisionConfiguration)),
			solver(new btSequentialImpulseConstraintSolver)
		{
			this->dynamicsWorld = new btDiscreteDynamicsWorld(this->dispatcher, this->broadphase, this->solver, this->collisionConfiguration);
			this->dynamicsWorld->setGravity(btVector3(0, -9.81, 0)); // set gravity to -9.81 in the y direction
		}

		btRigidBody* generateCubeRigidbody(btVector3 startingPosition, btVector3 dimensions, btVector3 scaleFactor) {
			btTransform startTransform;
			startTransform.setIdentity();
			startTransform.setOrigin(startingPosition);
			btScalar mass(1.0f);
			btVector3 localInertia(0, 0, 0);
			btCollisionShape* shape = new btBoxShape(dimensions);
			shape->setLocalScaling(scaleFactor);
			btDefaultMotionState* cubeMotionState = new btDefaultMotionState(startTransform);
			shape->calculateLocalInertia(mass, localInertia);
			btRigidBody::btRigidBodyConstructionInfo rigidBodyCIC1(mass, cubeMotionState, shape, localInertia);
			btRigidBody* cubeRigidBodyC1 = new btRigidBody(rigidBodyCIC1);
			cubeRigidBodyC1->setWorldTransform(startTransform);
			return cubeRigidBodyC1;
		}

		btRigidBody* generateTetrahedronRigidbody(btVector3 startingPosition, btVector3 tetrahedronVertices[],  btVector3 scaleFactor) {
			btTransform startTransform;
			startTransform.setIdentity();
			startTransform.setOrigin(startingPosition);
			btScalar mass(1.0f);
			btVector3 localInertia(0, 0, 0);
			btConvexHullShape* shape = new btConvexHullShape();
			for (int i = 0; i < 5; i++) {
				shape->addPoint(tetrahedronVertices[i]);
			}
			shape->setLocalScaling(scaleFactor);
			btDefaultMotionState* tetrahedronMotionState = new btDefaultMotionState(startTransform);
			shape->calculateLocalInertia(mass, localInertia);
			btRigidBody::btRigidBodyConstructionInfo rigidBody(mass, tetrahedronMotionState, shape, localInertia);
			btRigidBody* tetrahedronRigidBody = new btRigidBody(rigidBody);
			tetrahedronRigidBody->setWorldTransform(startTransform);
			return tetrahedronRigidBody;
		}


		btRigidBody* generateGroundRigidbody(btVector3 startingPosition) {
			//Remember that it's an infinite plane
			btTransform startGroundTransform;
			startGroundTransform.setIdentity();
			startGroundTransform.setOrigin(startingPosition);
			btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
			btDefaultMotionState* groundMotionState = new btDefaultMotionState(startGroundTransform);
			btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
			return new btRigidBody(groundRigidBodyCI);
		}
		
		glm::mat4 getUpdatedGLModelMatrix(btRigidBody* rb) {
			// get the transform of the rigid body representing the cube
			btTransform cubeTrans;
			rb->getMotionState()->getWorldTransform(cubeTrans);

			// update the position of the cube 
			glm::mat4 model;
			btScalar matr[16];
			cubeTrans.getOpenGLMatrix(matr);
			fillMat4(model, matr);
			return model;
		}
};