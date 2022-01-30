#include "TiltTable.h"

NCL::CSC8503::TiltTable::TiltTable()
{
}

NCL::CSC8503::TiltTable::~TiltTable()
{
}

void NCL::CSC8503::TiltTable::Update(float dt)
{
	this->GameObject::Update(dt);
	angles = this->GetTransform().GetOrientation().ToEuler();
	if (abs(angles.x) > maxRotation) {
		Vector3 rotateTo = angles;
		if (angles.x > 0) rotateTo.x = maxRotation - 0.01f;
		else rotateTo.x = -maxRotation + 0.01f;
		Quaternion rTo = Quaternion::EulerAnglesToQuaternion(rotateTo.x,rotateTo.y,rotateTo.z);
		this->GetTransform().SetOrientation(rTo);
		this->GetPhysicsObject()->ClearMovement();
	}
	if (abs(angles.z) > maxRotation) {
		Vector3 rotateTo = angles;
		if (angles.z > 0) rotateTo.z = maxRotation - 0.01f;
		else rotateTo.z = -maxRotation + 0.01f;
		Quaternion rTo = Quaternion::EulerAnglesToQuaternion(rotateTo.x, rotateTo.y, rotateTo.z);
		this->GetTransform().SetOrientation(rTo);
		this->GetPhysicsObject()->ClearMovement();
	}
}

void NCL::CSC8503::TiltTable::leanLeft(float dt)
{
	if (angles.x > -maxRotation) this->GetPhysicsObject()->AddTorque(-Vector3(1, 0, 0) * speed * dt);
}

void NCL::CSC8503::TiltTable::leanRight(float dt)
{
	if (angles.x < maxRotation) this->GetPhysicsObject()->AddTorque(Vector3(1, 0, 0) * speed * dt);
}

void NCL::CSC8503::TiltTable::leanForward(float dt)
{
	if (angles.z > -maxRotation) this->GetPhysicsObject()->AddTorque(-Vector3(0, 0, 1) * speed * dt);
}

void NCL::CSC8503::TiltTable::leanBack(float dt)
{
	if (angles.z < maxRotation) this->GetPhysicsObject()->AddTorque(Vector3(0, 0, 1) * speed * dt);
}
