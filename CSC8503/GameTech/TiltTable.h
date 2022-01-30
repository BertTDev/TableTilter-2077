#pragma once
#include "../CSC8503Common/GameObject.h"
namespace NCL {
	namespace CSC8503 {
		class TiltTable : public GameObject
		{
		public:
			TiltTable();
			~TiltTable();
			virtual void Update(float dt);
			void leanLeft(float dt);
			void leanRight(float dt);
			void leanForward(float dt);
			void leanBack(float dt);
		protected:
			Vector3 angles = Vector3();
			float speed = 500000;
			float maxRotation = 15;
		};
	}
}


