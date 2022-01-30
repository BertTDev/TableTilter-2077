#pragma once
#include <functional>

namespace NCL {
	namespace CSC8503 {
		typedef std::function<void(float)> StateUpdateFunction;
		class State		{
		public:
			State() {}
			State(StateUpdateFunction someFunc) {
				func = someFunc;
			}
			virtual ~State() {}
			virtual void Update(float dt) {
				if (func != nullptr) func(dt);
			};
		protected:
			StateUpdateFunction func;
		};
	}
}