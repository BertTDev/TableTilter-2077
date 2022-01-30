#pragma once
#include "Transform.h"
#include "CollisionVolume.h"

#include "PhysicsObject.h"
#include "RenderObject.h"

#include <vector>

using std::vector;

namespace NCL {
	enum class CollisionLayer {
		Container = 1,
		Ball = 2,
		Damager = 4,
		Mover = 8,
		Ghost = 16,
		Water = 32,
		Invalid = 256
	};
	namespace CSC8503 {

		class GameObject	{
		public:
			GameObject(string name = "");
			~GameObject();

			void SetBoundingVolume(CollisionVolume* vol) {
				boundingVolume = vol;
			}

			const CollisionVolume* GetBoundingVolume() const {
				return boundingVolume;
			}

			void SetCollisionLayer(CollisionLayer cl) {
				collisionLayer = cl;
			}

			const CollisionLayer GetCollisionLayer() const {
				return collisionLayer;
			}

			bool IsActive() const {
				return isActive;
			}

			Transform& GetTransform() {
				return transform;
			}

			RenderObject* GetRenderObject() const {
				return renderObject;
			}

			PhysicsObject* GetPhysicsObject() const {
				return physicsObject;
			}

			void SetRenderObject(RenderObject* newObject) {
				renderObject = newObject;
			}

			void SetPhysicsObject(PhysicsObject* newObject) {
				physicsObject = newObject;
			}

			void SetName(string n) {
				name = n;
			}
			const string& GetName() const {
				return name;
			}

			void SetLocalPosition(Vector3 pos){
				localPos = pos;
			}

			Vector3 GetLocalPosition() {
				return localPos;
			}
			virtual void OnCollisionBegin(GameObject* otherObject);

			virtual void OnCollisionEnd(GameObject* otherObject);

			virtual void Update(float dt);

			bool GetBroadphaseAABB(Vector3&outsize) const;

			void UpdateBroadphaseAABB();

			void SetWorldID(int newID) {
				worldID = newID;
			}

			int		GetWorldID() const {
				return worldID;
			}

			Vector4 GetObjColour() {
				return objColour;
			}

			void SetObjColour(Vector4 col) {
				this->objColour = col;
			}

			void AddChild(GameObject* g);

			int GetPoints() {
				return points;
			}

			bool ShouldDelete() {
				return shouldDelete;
			}

			bool HasWon() {
				return hasWon;
			}
			std::vector<GameObject*>::const_iterator GetChildIteratorStart() { return children.begin(); }
			std::vector<GameObject*>::const_iterator GetChildIteratorEnd() { return children.end(); }

		protected:
			Transform			transform;

			CollisionVolume*		boundingVolume;
			CollisionLayer			collisionLayer;
			vector<CollisionLayer>	collidesWith;
			PhysicsObject*			physicsObject;
			RenderObject*			renderObject;

			GameObject* parent;
			Vector3 localPos;
			vector<GameObject*>children;

			bool	isActive;
			bool	shouldUpdate;
			int		worldID;
			string	name;

			int points = 0;
			bool shouldDelete = false;
			bool hasWon = false;
			Vector4 objColour = Vector4(1, 1, 1, 1);

			Vector3 broadphaseAABB;
		};
	}
}

