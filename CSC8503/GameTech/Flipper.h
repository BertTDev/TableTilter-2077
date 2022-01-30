#pragma once
#include "../CSC8503Common/GameObject.h"
namespace NCL {
	namespace CSC8503 {
		class Flipper : public GameObject
		{
		public:
			Flipper(string name, Vector3 size, Vector3 pos, GameObject* lock);
			~Flipper();
			virtual void Update(float dt);

			GameObject* getLock() {
				return lockPos;
			}
			float getSpeed() {
				return spinSpeed;
			}
			void setSpeed(float speed) {
				spinSpeed = speed;
			}
			void flipDirection() {
				spinDir *= -1;
			}

			Vector3 rotationAxis = Vector3();
		protected:
			GameObject* lockPos;
			float spinSpeed = 10;
			float spinDir = 1;
			Quaternion localOrientation;
			Vector3 localAngVel = Vector3();
		};
	}
}


