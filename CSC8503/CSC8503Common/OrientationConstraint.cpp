#include "OrientationConstraint.h"
#include "GameObject.h"

void NCL::CSC8503::OrientationConstraint::UpdateConstraint(float dt)
{
	Quaternion relativeOrientation = objectA->GetTransform().GetOrientation() -
		objectB->GetTransform().GetOrientation();

	Vector3 currentDistance = relativeOrientation.ToEuler();

	Vector3 offset = distance - currentDistance;

	if (abs(offset.x) > 0.0f || abs(offset.y) > 0.0f || abs(offset.z) > 0.0f) {
		Vector3 offsetDir = currentDistance.Normalised();
		PhysicsObject* physA = objectA->GetPhysicsObject();
		PhysicsObject* physB = objectB->GetPhysicsObject();

		Vector3 relativeTorque = physA->GetTorque() - physB->GetTorque();

		float constraintMass = physA->GetInverseMass() + physB->GetInverseMass();

		if (constraintMass > 0.0f) {
			float velocityDot = Vector3::Dot(relativeTorque, offsetDir);
			float biasFactor = 0.01f;
			float bias = (-biasFactor / dt) * offset.Length();

			float lambda = -(velocityDot) / constraintMass;

			Vector3 aImpulse = offsetDir * lambda;
			Vector3 bImpulse = -offsetDir * lambda;

			physA->AddTorque(aImpulse);
			physB->AddTorque(bImpulse);
		}
	}
}
