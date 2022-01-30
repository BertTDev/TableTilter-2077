#include "GameObject.h"
#include "CollisionDetection.h"

using namespace NCL::CSC8503;

GameObject::GameObject(string objectName)	{
	name			= objectName;
	worldID			= -1;
	isActive		= true;
	shouldUpdate	= true;
	boundingVolume	= nullptr;
	physicsObject	= nullptr;
	renderObject	= nullptr;
	collisionLayer = CollisionLayer::Ball;
	objColour = Vector4(1, 1, 1, 1);
	parent = nullptr;
	localPos = Vector3(0, 0, 0);
}

GameObject::~GameObject()	{
	delete boundingVolume;
	delete physicsObject;
	delete renderObject;
	if (children.size() > 0) for (auto* c : children) c->parent = nullptr;
}

void GameObject::AddChild(GameObject* g)
{
	children.push_back(g);
	g->SetLocalPosition(g->GetTransform().GetPosition() - this->GetTransform().GetPosition());
	g->parent = this;
}

void NCL::CSC8503::GameObject::OnCollisionBegin(GameObject* otherObject)
{
	if (name == "bonus") {
		shouldDelete = true;
	}
	else {
		if (otherObject->GetName() == "bonus") {
			points += 500;
		}
		if (this->GetName() == "Ball" && otherObject->GetName() == "endZone") hasWon = true;
		if (this->GetName() == "Player" && otherObject->GetName() == "Enemy")	hasWon = true;
	}

}

void NCL::CSC8503::GameObject::OnCollisionEnd(GameObject* otherObject)
{
}

void NCL::CSC8503::GameObject::Update(float dt)
{
	if (parent) {
		Matrix3 parentOrientation = Matrix3(parent->GetTransform().GetOrientation());
		Vector3 parentPosition = parent->GetTransform().GetPosition();
		Vector3 position = parentPosition + parentOrientation* localPos;
		transform.SetPosition(position);
		transform.SetOrientation(parentOrientation);
	}
}

bool GameObject::GetBroadphaseAABB(Vector3&outSize) const {
	if (!boundingVolume) return false;
	outSize = broadphaseAABB;
	return true;
}

void GameObject::UpdateBroadphaseAABB() {
	if (!boundingVolume) return;
	if (boundingVolume->type == VolumeType::AABB) broadphaseAABB = ((AABBVolume&)*boundingVolume).GetHalfDimensions();
	else if (boundingVolume->type == VolumeType::Sphere) {
		float r = ((SphereVolume&)*boundingVolume).GetRadius();
		broadphaseAABB = Vector3(r, r, r);
	}
	else if (boundingVolume->type == VolumeType::OBB) {
		Matrix3 mat = Matrix3(transform.GetOrientation());
		mat = mat.Absolute();
		Vector3 halfSizes = ((OBBVolume&)*boundingVolume).GetHalfDimensions();
		broadphaseAABB = mat * halfSizes;
	}
}