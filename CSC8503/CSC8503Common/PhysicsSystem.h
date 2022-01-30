#pragma once
#include "../CSC8503Common/GameWorld.h"
#include <set>

namespace NCL {
	namespace CSC8503 {
		class PhysicsSystem	{
		public:
			PhysicsSystem(GameWorld& g);
			~PhysicsSystem();

			void Clear();

			void Update(float dt);

			void UseGravity(bool state) {
				applyGravity = state;
			}

			void SetGlobalDamping(float d) {
				globalDamping = d;
			}

			void SetLinearDamping(float d) {
				linearDamping = d;
			}

			float GetLinearDamping() {
				return linearDamping;
			}
			void SetAngularDamping(float d) {
				angularDamping = d;
			}
			float getAngularDamping() {
				return angularDamping;
			}

			void drawColliders() {
				shouldDrawColliders = !shouldDrawColliders;
			}

			void drawQuadtree() {
				shouldDrawQuadtree = !shouldDrawQuadtree;
			}

			const bool ShouldDrawColliders() {
				return shouldDrawColliders;
			}
			void SetGravity(const Vector3& g);

		protected:
			void BasicCollisionDetection();
			void BroadPhase();
			void NarrowPhase();

			void ClearForces();

			void IntegrateAccel(float dt);
			void IntegrateVelocity(float dt);

			void UpdateConstraints(float dt);

			void UpdateCollisionList();
			void UpdateObjectAABBs();

			void ImpulseResolveCollision(GameObject& a , GameObject&b, CollisionDetection::ContactPoint& p) const;

			GameWorld& gameWorld;

			bool	applyGravity;
			Vector3 gravity;
			float	dTOffset;
			float	globalDamping;
			float	linearDamping;
			float	angularDamping;
			std::set<CollisionDetection::CollisionInfo> allCollisions;
			std::set<CollisionDetection::CollisionInfo> broadphaseCollisions;

			bool useBroadPhase		= true;
			bool shouldDrawColliders = false;
			bool shouldDrawQuadtree = false;
			int numCollisionFrames	= 5;
		};
	}
}

