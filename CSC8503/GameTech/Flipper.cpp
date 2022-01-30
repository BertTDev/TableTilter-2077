#include "Flipper.h"
#include "../CSC8503Common/GameObject.h"
#include "../CSC8503Common/CollisionDetection.h"
#include "../CSC8503Common/OBBVolume.h"
using namespace NCL;
using namespace CSC8503;

NCL::CSC8503::Flipper::Flipper(string name , Vector3 size, Vector3 pos, GameObject* lock)
{
	this->name = name;

	lockPos = lock;
	Vector3 lockSize = Vector3(1, 1, 1);
	lockPos->GetTransform().SetPosition(pos);
	lockPos->SetPhysicsObject(new PhysicsObject(&this->GetTransform(), this->GetBoundingVolume()));
	lockPos->GetPhysicsObject()->SetInverseMass(0);
	lockPos->GetPhysicsObject()->setFriction(0.8f);
	lockPos->GetPhysicsObject()->setElasticity(0.7f);
	lockPos->GetPhysicsObject()->InitCubeInertia();

	OBBVolume* volume = new OBBVolume(size);
	this->SetBoundingVolume((CollisionVolume*)volume);
	this->GetTransform()
		.SetScale(size * 2)
		.SetPosition(pos);
	this->SetPhysicsObject(new PhysicsObject(&this->GetTransform(), this->GetBoundingVolume()));
	this->GetPhysicsObject()->SetInverseMass(0.1f);
	this->GetPhysicsObject()->setFriction(1.0f);
	this->GetPhysicsObject()->setElasticity(1.0f);
	this->GetPhysicsObject()->InitCubeInertia();

	localOrientation = this->GetTransform().GetOrientation();
	localAngVel = Vector3(0, 5, 0);
}

NCL::CSC8503::Flipper::~Flipper()
{

}

void NCL::CSC8503::Flipper::Update(float dt)
{
	Quaternion lockRotation = lockPos->GetTransform().GetOrientation();
	Vector3 angVel = this->GetPhysicsObject()->GetAngularVelocity();
	localOrientation = localOrientation * Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), angVel.y);
	this->GetTransform().SetOrientation(lockRotation * localOrientation);
	rotationAxis = Matrix3(lockRotation) * Vector3(0, 1, 0);
	this->GetPhysicsObject()->SetAngularVelocity(rotationAxis * localAngVel);
	this->GetTransform().SetPosition(lockPos->GetTransform().GetPosition());
}
