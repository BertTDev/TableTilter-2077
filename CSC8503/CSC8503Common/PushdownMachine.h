#pragma once
#include <stack>
#include "../CSC8503Common/PushdownState.h"
namespace NCL {
	namespace CSC8503 {

		class PushdownMachine
		{
		public:
			PushdownMachine(PushdownState* initialState) {
				this->initialState = initialState;
			};
			~PushdownMachine();

			bool Update(float dt);

			void Reset() {
				if (initialState) {
					activeState = initialState;
					activeState->OnAwake();
				}
			}
			PushdownState* GetActiveState() {
				return activeState;
			}
		protected:
			PushdownState * activeState;
			PushdownState* initialState;

			std::stack<PushdownState*> stateStack;
		};
	}
}

