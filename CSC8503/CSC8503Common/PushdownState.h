#pragma once
#include "State.h"

namespace NCL {
	enum class StateType {
		Menu = 1,
		Level1 = 2,
		Level2 = 4,
		Invalid = 256
	};
	namespace CSC8503 {
		class PushdownState
		{
		public:
			enum PushdownResult {
				Push, Pop, NoChange
			};
			PushdownState() {};
			~PushdownState() {};

			virtual PushdownResult OnUpdate(float dt, PushdownState** pushfunc) = 0;

			virtual void OnAwake() {}
			virtual void OnSleep() {}
			StateType type = StateType::Invalid;
			bool triggered = false;
		};
	}
}

