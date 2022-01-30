#pragma once
#include "GameTechRenderer.h"
#include "../CSC8503Common/PhysicsSystem.h"
#include "Flipper.h"
#include  "../CSC8503Common/StateGameObject.h"
#include "../CSC8503Common/PushdownMachine.h"
#include "../CSC8503Common/PushdownState.h"
namespace NCL {
	namespace CSC8503 {
		class TiltTable;
		class NavigationGrid;
		class StateMachine;
		class TutorialGame		{
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);

			//Public Facing methods to handle Enemy AI State
			void setPathfindtarget(GameObject* targ) {
				pathfindTarget = targ;
			}
			GameObject* getPlayer() {
				return playChar;
			}
			vector<GameObject*>* getBonuses() {
				return &bonuses;
			}

			GameObject* getPathfindTarget() {
				return pathfindTarget;
			}

			GameObject* playChar;
			GameObject* enemyChar;

			// Game Management Flags
			bool L1ButtonClicked = false;
			bool L2ButtonClicked = false;
			bool ExitButtonClicked = false;
			bool menuTrigger = false;
			bool hasChangedStates = false;

		protected:
			void InitialiseAssets();

			void InitCamera();
			void SetCamera(float Pitch, float Yaw, Vector3 Position);
			void InitWorld();

			bool SelectObject();
			void UpdateKeys(float dt);
			void UpdateDebugKeys(float dt);

			void levelHandlerUpdate(float dt);
			void levelUpdate();
			void InitMainMenu();

			void InitLevel1();
			TiltTable* table;
			bool shouldBuildBridge = true;
			GameObject* ball = nullptr;
			vector<Flipper*> flippers;

			void InitLevel2();
			vector<GameObject*> L2Walls;
			bool is2Player = false;

			void DrawGUI(float dt);

			void DrawColliders();
			void DrawCollider(GameObject* g);
			void DrawAABBCollider(Vector3 pos,AABBVolume* vol);
			void DrawSphereCollider(Quaternion Orientation, Vector3 pos, SphereVolume* vol);
			void DrawOBBCollider(Quaternion Orientation, Vector3 pos, OBBVolume* vol);
			void DrawCapsuleCollider(Vector3 pos, CapsuleVolume* vol);


			
			TiltTable*	AddTableToWorld();
			void		AddLevel2WallsToWorld();
			GameObject* AddSlopeToWorld(const Vector3& position, const Vector3& size, float angle, float inverseMass = 0.0f);
			GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f);
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			GameObject* AddOrientedCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			GameObject* AddFlipperToWorld(const Vector3& position, const Vector3& size);
			GameObject* AddButtonToWorld(const Vector2& position, const Vector2& size, string name, StateType type);

			void UpdateTiltTable(float dt);
			void UpdatePlayer(float dt);
			void UpdateEnemy(float dt);

			GameTechRenderer*	renderer;
			PhysicsSystem*		physics;
			GameWorld*			world;

			bool useGravity;
			bool inSelectionMode;
			bool shouldDrawColliders;

			float		forceMagnitude;

			GameObject* selectionObject = nullptr;

			OGLMesh*	capsuleMesh = nullptr;
			OGLMesh*	cubeMesh	= nullptr;
			OGLMesh*	sphereMesh	= nullptr;
			OGLTexture* basicTex	= nullptr;
			OGLTexture* doge = nullptr;
			OGLTexture* ballTex = nullptr;
			OGLTexture* cubeTex = nullptr;
			OGLShader*	basicShader = nullptr;

			//Coursework Meshes
			OGLMesh*	charMeshA	= nullptr;
			OGLMesh*	charMeshB	= nullptr;
			OGLMesh*	enemyMesh	= nullptr;
			OGLMesh*	bonusMesh	= nullptr;

			PushdownMachine* levelHandler;
			GameObject* L1Button;
			GameObject* L2Button;
			GameObject* ExitButton;
			StateType currentState = StateType::Invalid;

			bool shouldClose = false;
			string title = "TABLE TILTER 2077";
			Vector4 titleColour = Vector4(0,0,0,0);
			float titleSize = 45.0f;

			NavigationGrid* grid;
			Vector3 startPos;
			Vector3 endPos;
			Vector3 gridOffset;
			Vector3 offsetPos;
			vector<Vector3> testNodes;
			void DisplayPathfinding();
			StateMachine* enemyStates;
			GameObject* pathfindTarget;
			void makeEnemyStateMachine();

			float bonusSpawnTimer = 0.0f;
			float bonusSpawnMax = 3.0f;
			void spawnBonus(float dt);
			vector<GameObject*> bonuses;
			void outputPoints();

			bool hasWon = false;
			float gameTimer = 0;
			void showEndScreen();
		};

		class ButtonGameObject : public GameObject {
		public:
			void SetButtonType(StateType t) {
				type = t;
			}
			StateType GetButtonType() {
				return type;
			}
		protected:
			StateType type = StateType::Invalid;
		};

		class Level1 : public PushdownState {
		public:
			Level1(TutorialGame* game) {
				this->game = game;
				type = StateType::Level1;
			}
			PushdownResult OnUpdate(float dt, PushdownState** newState) override {
				if (triggered) triggered = false;
				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::M)||
					game->menuTrigger) {
					return PushdownResult::Pop;
				}
				return PushdownResult::NoChange;
			}
			void OnAwake() override {
				triggered = true;
				std::cout << "Entering Level 1\n";
			}
			bool getTriggered() {
				return triggered;
			}
		protected:
			TutorialGame* game;
		};

		class Level2 : public PushdownState {
		public:
			Level2(TutorialGame* game) {
				this->game = game;
				type = StateType::Level2;
			}
			PushdownResult OnUpdate(float dt, PushdownState** newState) override {
				if (triggered) triggered = false;
				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::M) ||
					game->menuTrigger) {
					return PushdownResult::Pop;
				}
				return PushdownResult::NoChange;
			}
			void OnAwake() override {
				triggered = true;
				std::cout << "Entering Level 2\n";
			}
			bool getTriggered() {
				return triggered;
				
			}
		protected:
			TutorialGame* game;
		};

		class MainMenu : public PushdownState {
		public:
			MainMenu(TutorialGame* game) {
				this->game = game;
				type = StateType::Menu;
			}
			PushdownResult OnUpdate(float dt, PushdownState** newState) override {
				if (triggered) triggered = false;
				if (game->L1ButtonClicked) {
					*newState = new Level1(game);
					return PushdownResult::Push;
				}
				if (game->L2ButtonClicked) {
					*newState = new Level2(game);
					return PushdownResult::Push;
				}
				return PushdownResult::NoChange;
			}
			void OnAwake() override {
				triggered = true;
				std::cout << "Entering Menu\n";
			}
			bool getTriggered() {
				return triggered;
			}
		protected:
			TutorialGame* game;
		};
	}
}

