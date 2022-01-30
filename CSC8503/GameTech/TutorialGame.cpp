#include "TutorialGame.h"
#include "../CSC8503Common/GameWorld.h"
#include "../CSC8503Common/PositionConstraint.h"
#include "../CSC8503Common/OrientationConstraint.h"
#include "../../Plugins/OpenGLRendering/OGLMesh.h"
#include "../../Plugins/OpenGLRendering/OGLShader.h"
#include "../../Plugins/OpenGLRendering/OGLTexture.h"
#include "../../Common/TextureLoader.h"
#include "../CSC8503Common/PushdownMachine.h"
#include "../CSC8503Common/PushdownState.h"
#include "Flipper.h"
#include "TiltTable.h"
#include "../CSC8503Common/NavigationGrid.h"
#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/State.h"
#include "../CSC8503Common/StateTransition.h"
#include <cmath>
using namespace NCL;
using namespace CSC8503;



TutorialGame::TutorialGame()	{
	world		= new GameWorld();
	renderer	= new GameTechRenderer(*world);
	physics		= new PhysicsSystem(*world);
	physics->SetGravity(Vector3(0,-9.8f * 5.0f,0));

	PushdownState* entry = new MainMenu(this);
	levelHandler = new PushdownMachine(entry);

	forceMagnitude	= 10.0f;
	useGravity		= true;
	inSelectionMode = false;
	shouldDrawColliders = false;

	Debug::SetRenderer(renderer);

	grid = new NavigationGrid("Level2Grid.txt");
	offsetPos = Vector3(60, 0, 60);
	gridOffset = Vector3(5,5,5);
	startPos = Vector3(10, 0, 10) + gridOffset;
	endPos = Vector3(100, 0, 100) + gridOffset;
	pathfindTarget = new GameObject();
	
	InitialiseAssets();
}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	auto loadFunc = [](const string& name, OGLMesh** into) {
		*into = new OGLMesh(name);
		(*into)->SetPrimitiveType(GeometryPrimitive::Triangles);
		(*into)->UploadToGPU();
	};

	loadFunc("cube.msh"		 , &cubeMesh);
	loadFunc("sphere.msh"	 , &sphereMesh);
	loadFunc("Male1.msh"	 , &charMeshA);
	loadFunc("courier.msh"	 , &charMeshB);
	loadFunc("security.msh"	 , &enemyMesh);
	loadFunc("coin.msh"		 , &bonusMesh);
	loadFunc("capsule.msh"	 , &capsuleMesh);

	basicTex	= (OGLTexture*)TextureLoader::LoadAPITexture("checkerboard.png");
	doge = (OGLTexture*)TextureLoader::LoadAPITexture("doge.png");
	ballTex = (OGLTexture*)TextureLoader::LoadAPITexture("BallTex.png");
	cubeTex = (OGLTexture*)TextureLoader::LoadAPITexture("wallTex.png");

	basicShader = new OGLShader("GameTechVert.glsl", "GameTechFrag.glsl");

	InitCamera();
}

TutorialGame::~TutorialGame()	{
	delete cubeMesh;
	delete sphereMesh;
	delete charMeshA;
	delete charMeshB;
	delete enemyMesh;
	delete bonusMesh;

	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

void TutorialGame::UpdateGame(float dt) {
	world->GetMainCamera()->UpdateCamera(dt);
	UpdateKeys(dt);

	levelHandlerUpdate(dt);
	if(levelHandler->GetActiveState()->type == StateType::Menu) SelectObject();
	if(!hasWon)physics->Update(dt);
	world->UpdateWorld(dt);
	levelUpdate();

	DrawGUI(dt);
	if (physics->ShouldDrawColliders()) DrawColliders();
	showEndScreen();
	renderer->Update(dt);
	Debug::FlushRenderables(dt);
	renderer->Render();
}

void TutorialGame::UpdateKeys(float dt) {

	switch (currentState) {
	case StateType::Menu: if(Window::GetKeyboard()->KeyPressed(KeyboardKeys::V)) is2Player = !is2Player; break;
	case StateType::Level1: UpdateTiltTable(dt); break;
	case StateType::Level2: UpdatePlayer(dt); UpdateEnemy(dt); break;
	}
	UpdateDebugKeys(dt);
}

void NCL::CSC8503::TutorialGame::UpdateDebugKeys(float dt)
{
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1) || Window::GetKeyboard()->KeyPressed(KeyboardKeys::R)) {
		InitWorld();
		selectionObject = nullptr;
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F2))	InitCamera();

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::X))		physics->drawColliders();
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F))		physics->drawQuadtree();
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9))	world->ShuffleConstraints(true);
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10))	world->ShuffleConstraints(false);
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F7))	world->ShuffleObjects(true);
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F8))	world->ShuffleObjects(false);
}

void NCL::CSC8503::TutorialGame::levelHandlerUpdate(float dt)
{
	levelHandler->Update(dt);
	StateType stateType = levelHandler->GetActiveState()->type;
	if (levelHandler->GetActiveState()->triggered) {
		InitWorld();
	}
	if (stateType == StateType::Menu) {
		renderer->DrawString(title, Vector2(8, 15), titleColour, titleSize);
		renderer->DrawString("LEVEL 1", Vector2(8, 31), Vector4(1, 1, 1, 1), 35);
		if (!is2Player)renderer->DrawString("LEVEL 2", Vector2(8, 57.5f), Vector4(1, 1, 1, 1), 35);
		else renderer->DrawString("LEVEL 2 : TWO PLAYER", Vector2(8, 57.5f), Vector4(1, 1, 1, 1), 35);
		renderer->DrawString("EXIT GAME", Vector2(8, 84), Vector4(1, 1, 1, 1), 35);
	}

	if (currentState != levelHandler->GetActiveState()->type) {
		selectionObject = nullptr;
		currentState = levelHandler->GetActiveState()->type;
	}
}

void NCL::CSC8503::TutorialGame::levelUpdate()
{
	
	if (currentState == StateType::Level1) {
		if (ball->HasWon()) hasWon = ball->HasWon();
	}
	else if (currentState == StateType::Level2) {
		if (playChar->HasWon()) hasWon = playChar->HasWon();
		for (GameObject* b : bonuses) {
			if (b->ShouldDelete()) bonuses.erase(std::remove(bonuses.begin(), bonuses.end(), b), bonuses.end());
		}
	}
}

void NCL::CSC8503::TutorialGame::DrawColliders()
{
	if (currentState == StateType::Level2)DisplayPathfinding();
	world->OperateOnContents(
		[this](GameObject* g) {
			this->DrawCollider(g);
		});
}

void NCL::CSC8503::TutorialGame::DrawCollider(GameObject* g)
{
	Vector3 objPos = g->GetTransform().GetPosition();
	const CollisionVolume* colVol = g->GetBoundingVolume();
	if (!colVol) {
		return;
	}

	switch (colVol->type) {
	case VolumeType::AABB:		DrawAABBCollider(objPos,(AABBVolume*) colVol); break;
	case VolumeType::OBB:		DrawOBBCollider(g->GetTransform().GetOrientation(),objPos, (OBBVolume*)colVol); break;
	case VolumeType::Sphere:	DrawSphereCollider(g->GetTransform().GetOrientation(),objPos, (SphereVolume*)colVol); break;
	case VolumeType::Capsule:	DrawCapsuleCollider(objPos, (CapsuleVolume*)colVol); break;
	}
}

void NCL::CSC8503::TutorialGame::DrawAABBCollider(Vector3 pos,AABBVolume* vol)
{
	Vector3 halfSize = vol->GetHalfDimensions() + Vector3(0.05f,0.05f,0.05f);
	Vector3 corners[8] = {
		Vector3(-halfSize.x,-halfSize.y,-halfSize.z),Vector3(halfSize.x,-halfSize.y,-halfSize.z),
		Vector3(halfSize.x,-halfSize.y,halfSize.z),Vector3(-halfSize.x,-halfSize.y,halfSize.z),
		Vector3(-halfSize.x,halfSize.y,-halfSize.z),Vector3(halfSize.x,halfSize.y,-halfSize.z),
		Vector3(halfSize.x,halfSize.y,halfSize.z),Vector3(-halfSize.x,halfSize.y,halfSize.z)
	};

	for (int i = 0; i < 8; ++i) corners[i] += pos;

	Vector3 startPos;
	Vector3 endPos;
	//Bottom Square
	startPos = corners[0];
	endPos = corners[1];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[1];
	endPos = corners[2];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[2];
	endPos = corners[3];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[3];
	endPos = corners[0];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	//Top Square
	startPos = corners[4];
	endPos = corners[5];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[5];
	endPos = corners[6];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[6];
	endPos = corners[7];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[7];
	endPos = corners[4];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	//Sides
	startPos = corners[0];
	endPos = corners[4];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[1];
	endPos = corners[5];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[2];
	endPos = corners[6];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[3];
	endPos = corners[7];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));

}

void NCL::CSC8503::TutorialGame::DrawSphereCollider(Quaternion Orientation, Vector3 pos, SphereVolume* vol)
{
	float angleChange = 30.0f;
	float PI = 3.1f;
	angleChange = (2.0f * PI) / angleChange;
	float radius = vol->GetRadius() + 0.05f;
	Matrix3 transform = Matrix3(Orientation);
	for (float angle = 0.0f; angle <= 2.0f*PI; angle += angleChange) {
		float cosineStart = cos(angle) * radius;
		float sineStart = sin(angle) * radius;
		float cosineEnd = cos(angle+angleChange) * radius;
		float sineEnd = sin(angle+angleChange) * radius;

		Vector3 startPos = (transform * Vector3(cosineStart, sineStart,0.0f)) + pos;
		Vector3 endPos = (transform * Vector3(cosineEnd, sineEnd, 0)) + pos;
		Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
		startPos = (transform * Vector3(cosineStart, 0, sineStart)) + pos;
		endPos = (transform * Vector3(cosineEnd,0, sineEnd)) + pos;
		Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
		startPos = (transform * Vector3(0, sineStart, cosineStart)) + pos;
		endPos = (transform * Vector3(0, sineEnd, cosineEnd)) + pos;
		Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	}
}

void NCL::CSC8503::TutorialGame::DrawOBBCollider(Quaternion Orientation, Vector3 pos, OBBVolume* vol)
{
	Vector3 halfSize = vol->GetHalfDimensions() + Vector3(0.05f, 0.05f, 0.05f);
	Vector3 corners[8] = {
		Vector3(-halfSize.x,-halfSize.y,-halfSize.z),Vector3(halfSize.x,-halfSize.y,-halfSize.z),
		Vector3(halfSize.x,-halfSize.y,halfSize.z),Vector3(-halfSize.x,-halfSize.y,halfSize.z),
		Vector3(-halfSize.x,halfSize.y,-halfSize.z),Vector3(halfSize.x,halfSize.y,-halfSize.z),
		Vector3(halfSize.x,halfSize.y,halfSize.z),Vector3(-halfSize.x,halfSize.y,halfSize.z)
	};

	Matrix3 transform = Matrix3(Orientation);
	for (int i = 0; i < 8; ++i) {
		corners[i] = transform * corners[i];
		corners[i] += pos;
	}

	Vector3 startPos;
	Vector3 endPos;
	//Bottom Square
	startPos = corners[0];
	endPos = corners[1];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[1];
	endPos = corners[2];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[2];
	endPos = corners[3];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[3];
	endPos = corners[0];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	//Top Square
	startPos = corners[4];
	endPos = corners[5];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[5];
	endPos = corners[6];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[6];
	endPos = corners[7];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[7];
	endPos = corners[4];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	//Sides
	startPos = corners[0];
	endPos = corners[4];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[1];
	endPos = corners[5];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[2];
	endPos = corners[6];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
	startPos = corners[3];
	endPos = corners[7];
	Debug::DrawLine(startPos, endPos, Vector4(0, 1, 0, 1));
}

void NCL::CSC8503::TutorialGame::DrawCapsuleCollider(Vector3 pos, CapsuleVolume* vol)
{
}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(0.1f);
	world->GetMainCamera()->SetFarPlane(1000.0f);
	world->GetMainCamera()->SetPitch(-15.0f);
	world->GetMainCamera()->SetYaw(315.0f);
	world->GetMainCamera()->SetPosition(Vector3(-60, 40, 60));
	Window::GetWindow()->ShowOSPointer(false);
	Window::GetWindow()->LockMouseToWindow(true);
	inSelectionMode = false;
}

void TutorialGame::SetCamera(float Pitch, float Yaw, Vector3 Position) {
	world->GetMainCamera()->SetPitch(Pitch);
	world->GetMainCamera()->SetYaw(Yaw);
	world->GetMainCamera()->SetPosition(Position);
}

void TutorialGame::InitWorld() {
	hasWon = false;
	gameTimer = 0;
	world->Clear();
	physics->Clear();
	bonuses.clear();
	InitCamera();

	if (levelHandler->GetActiveState()) {
		switch (levelHandler->GetActiveState()->type) {
		case StateType::Menu: InitMainMenu(); break;
		case StateType::Level1: InitLevel1(); break;
		case StateType::Level2: InitLevel2(); break;
		}
	}
	else InitMainMenu();
}

void NCL::CSC8503::TutorialGame::InitMainMenu()
{
	L1ButtonClicked = false;
	L2ButtonClicked = false;
	ExitButtonClicked = false;
	menuTrigger = false;
	inSelectionMode = true;
	Vector2 screenSize = Window::GetWindow()->GetScreenSize();
	Vector2 buttonSize = Vector2(screenSize.x, screenSize.y / 15);
	this->SetCamera(0, 180, Vector3(0, screenSize.y/2 + 50, 10));

	Window::GetWindow()->ShowOSPointer(true);
	Window::GetWindow()->LockMouseToWindow(false);

	L1Button	= AddButtonToWorld(Vector2(screenSize.x / 2, 4 * screenSize.y / 5 - 50), buttonSize, "L1 Button", StateType::Level1);
	L2Button	= AddButtonToWorld(Vector2(screenSize.x / 2, 3 * screenSize.y / 5 - 50), buttonSize, "L2 Button", StateType::Level2);
	ExitButton	= AddButtonToWorld(Vector2(screenSize.x/2,2 * screenSize.y/5 -50),buttonSize, "Exit Button", StateType::Invalid);
}

void NCL::CSC8503::TutorialGame::InitLevel1()
{
	this->SetCamera(-70, -90, Vector3(-150, 450, 0));

	ball = AddSphereToWorld(Vector3(-100,100,100),10,0.5);
	ball->SetName("Ball");
	ball->GetPhysicsObject()->setElasticity(1.0f);
	table = AddTableToWorld();
}

void NCL::CSC8503::TutorialGame::InitLevel2()
{
	this->SetCamera(-90, 0, Vector3(60, 200, 60));
	makeEnemyStateMachine();

	playChar = AddSphereToWorld(startPos,4,1);
	playChar->GetRenderObject()->SetColour(Vector4(0,1,1,1));
	playChar->SetName("Player");
	pathfindTarget = playChar;

	enemyChar = AddSphereToWorld(Vector3(105,5,105), 4, 1);
	enemyChar->GetRenderObject()->SetColour(Vector4(1, 0, 0, 1));
	enemyChar->SetName("Enemy");

	
	AddLevel2WallsToWorld();
}

void NCL::CSC8503::TutorialGame::DrawGUI(float dt)
{
	if (currentState == StateType::Level2) {
		outputPoints();
		spawnBonus(dt);
	}
	if (currentState != StateType::Menu && !hasWon) {
		gameTimer += dt;
		string timer = "TIME: " + std::to_string((int)gameTimer);
		renderer->DrawString(timer, Vector2(45, 10), Vector4(1, 1, 1, 1));
	}
}

GameObject* NCL::CSC8503::TutorialGame::AddSlopeToWorld(const Vector3& position, const Vector3& size, float angle, float inverseMass)
{
	GameObject* floor = new GameObject();

	OBBVolume* volume = new OBBVolume(size);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(size * 2)
		.SetPosition(position)
		.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1,0,0), angle));

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, cubeTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(inverseMass);
	floor->GetPhysicsObject()->setFriction(0.8f);
	floor->GetPhysicsObject()->setElasticity(0.7f);
	floor->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(floor);

	return floor;
}

TiltTable* NCL::CSC8503::TutorialGame::AddTableToWorld()
{
	//ADD THE BASE FLOOR
	TiltTable* floor = new TiltTable();
	floor->SetName("Table");
	Vector3 position = Vector3(0, 0, 0);
	Vector3 size = Vector3(150, 2, 150);

	Vector3 centerSize = Vector3(150.0f, 1, 150.0f);
	OBBVolume* volume = new OBBVolume(centerSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(centerSize * 2)
		.SetPosition(position)
		.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1, 0, 0), 0));

	floor->SetRenderObject(nullptr);
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(1);
	floor->GetPhysicsObject()->setFriction(0.5f);
	floor->GetPhysicsObject()->setElasticity(0.7f);
	floor->GetPhysicsObject()->InitCubeInertia();

	//ADD THE CENTRAL LOCKING POINT
	Vector3 lockPosition = Vector3(0, 0, 0);
	Vector3 lockSize = Vector3(0.5f, 0.5f, 0.5f);
	GameObject* tableLock = AddOrientedCubeToWorld(lockPosition, lockSize,0);
	tableLock->SetName("TableLock");
	PositionConstraint* constraint = new PositionConstraint(tableLock, (GameObject*)floor, 0);

	world->AddGameObject(floor);
	world->AddConstraint(constraint);
	
	//ADD THE TABLE FLOORS
	Vector3 offsetPos = Vector3(0, 50, 0);

	GameObject* BottomFloor = AddOrientedCubeToWorld(offsetPos + Vector3(-98,0,0),Vector3(size.x/3,size.y,size.z),0);
	BottomFloor->GetPhysicsObject()->setFriction(0.0f);
	BottomFloor->SetName("Floor1");
	BottomFloor->SetObjColour(Vector4(0,0.9f,1,1));
	BottomFloor->GetRenderObject()->SetColour(BottomFloor->GetObjColour());
	floor->AddChild(BottomFloor);

	GameObject* TopFloor = AddOrientedCubeToWorld(offsetPos + Vector3(98, 0, 0), Vector3(size.x / 3, size.y, size.z), 0);
	TopFloor->SetName("Floor2");
	floor->AddChild(TopFloor);

	//ADDS THE BRIDGE 
	if (shouldBuildBridge) {
		Vector3 cubeSize = Vector3(size.x / 3, 2, 25);

		float invCubeMass = 5;
		int numLinks = 5;
		float maxDistance = 48;
		float cubeDistance = 10;
		Vector3 startPos = offsetPos + Vector3(0, 0, -145);

		GameObject* start = AddOrientedCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize, 0);
		start->SetName("bridgeStart");
		GameObject* end = AddOrientedCubeToWorld(startPos + Vector3(0, 0, 290), cubeSize, 0);
		start->SetName("bridgeEnd");
		floor->AddChild(start);
		floor->AddChild(end);
		GameObject* previous = start;
		for (int i = 0; i < numLinks; ++i) {
			GameObject* block = AddOrientedCubeToWorld(startPos + Vector3(0, 0, (i + 1) * cubeDistance), cubeSize + Vector3(-5, 0, 0), invCubeMass);
			PositionConstraint* bridgeconstraint = new PositionConstraint(previous, block, maxDistance);
			world->AddConstraint(bridgeconstraint);
			previous = block;
		}
		PositionConstraint* bridgeconstraint = new PositionConstraint(previous, end, maxDistance);
		world->AddConstraint(bridgeconstraint);
	}
	else {
		GameObject* MiddleFloor = AddOrientedCubeToWorld(offsetPos + Vector3(0, 0, 0), Vector3(size.x / 3, size.y, size.z), 0);
		MiddleFloor->SetName("Floor3");
		floor->AddChild(MiddleFloor);
	}
	
	//ADD THE WALLS
	float wallHeight = 20;
	float wallWidth = 5;
	//Left Wall
	GameObject* rightWall = AddOrientedCubeToWorld(offsetPos + Vector3(0,wallHeight/2, size.z + wallWidth),Vector3(size.x,wallHeight,wallWidth),0);
	floor->AddChild(rightWall);
	//Right Wall
	GameObject* leftWall = AddOrientedCubeToWorld(offsetPos + Vector3(0, wallHeight / 2, -size.z - wallWidth), Vector3(size.x, wallHeight, wallWidth), 0);
	floor->AddChild(leftWall);
	//Bottom Wall
	GameObject* bottomWall = AddOrientedCubeToWorld(offsetPos + Vector3(size.x + wallWidth, wallHeight / 2, 0), Vector3(wallWidth, wallHeight, size.z), 0);
	floor->AddChild(bottomWall);
	//Top Wall
	GameObject* TopWall = AddOrientedCubeToWorld(offsetPos + Vector3(-size.x - wallWidth, wallHeight / 2, 0), Vector3(wallWidth, wallHeight, size.z), 0);
	floor->AddChild(TopWall);

	//Section Spltting Walls
	GameObject* MidWall1 = AddOrientedCubeToWorld(offsetPos + Vector3(-50, wallHeight / 2, 40), Vector3(wallWidth, wallHeight, size.z-40), 0);
	floor->AddChild(MidWall1);
	GameObject* MidWall2 = AddOrientedCubeToWorld(offsetPos + Vector3(50, wallHeight / 2, -40), Vector3(wallWidth, wallHeight, size.z-40), 0);
	floor->AddChild(MidWall2);
	GameObject* StartWall1 = AddOrientedCubeToWorld(offsetPos + Vector3(-50 - (size.x/5), wallHeight / 2, 40), Vector3(size.x/5, wallHeight, wallWidth), 0);
	floor->AddChild(StartWall1);
	GameObject* StartWall2 = AddOrientedCubeToWorld(offsetPos + Vector3(-100 - (size.x / 5), wallHeight / 2, -20), Vector3(size.x / 5, wallHeight, wallWidth), 0);
	floor->AddChild(StartWall2);

	//ADD FLIPPERS
	float flipperHeight = 18;
	float flipperWidth = 3;
	Flipper* flipper1 = (Flipper*)AddFlipperToWorld(offsetPos+ Vector3(125,flipperHeight/2 + 1,20),Vector3(size.x/6,flipperHeight,flipperWidth));
	flippers.emplace_back(flipper1);
	floor->AddChild(flipper1->getLock());
	flipper1 = (Flipper*)AddFlipperToWorld(offsetPos + Vector3(85, flipperHeight / 2 + 1, -60), Vector3(size.x / 6, flipperHeight, flipperWidth));
	flippers.emplace_back(flipper1);
	floor->AddChild(flipper1->getLock());

	//ADD AN ENDZONE
	GameObject* endZone = AddOrientedCubeToWorld(offsetPos + Vector3(98,13,-140),Vector3(size.x/3 -5,15,5), 0);
	endZone->SetName("endZone");
	endZone->GetRenderObject()->SetDefaultTexture(basicTex);
	endZone->GetRenderObject()->SetColour(Vector4(0,1,0,1));
	floor->AddChild(endZone);

	return floor;
}

void NCL::CSC8503::TutorialGame::AddLevel2WallsToWorld()
{
	AddSlopeToWorld(offsetPos, Vector3(60, 2, 60), 0, 0);

	GameObject* wall = AddOrientedCubeToWorld(offsetPos + Vector3(0, 11, 60), Vector3(60, 20, 5), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
	wall = AddOrientedCubeToWorld(offsetPos + Vector3(0, 11, -60), Vector3(60, 20, 5), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
	wall = AddOrientedCubeToWorld(offsetPos + Vector3(60, 11, 0), Vector3(5, 20, 60), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
	wall = AddOrientedCubeToWorld(offsetPos + Vector3(-60, 11, 0), Vector3(5, 20, 60), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);

	wall = AddOrientedCubeToWorld(offsetPos + Vector3(-30, 11, 35), Vector3(10, 20, 5), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);

	wall = AddOrientedCubeToWorld(offsetPos + Vector3(20, 11, 35), Vector3(20, 20, 5), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);

	wall = AddOrientedCubeToWorld(offsetPos + Vector3(35, 11, 25), Vector3(5, 20, 15), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
	wall = AddOrientedCubeToWorld(offsetPos + Vector3(-15, 11, 15), Vector3(25, 20, 5), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
	wall = AddOrientedCubeToWorld(offsetPos + Vector3(-35, 11, -5), Vector3(5, 20, 15), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
	wall = AddOrientedCubeToWorld(offsetPos + Vector3(-15, 11, -5), Vector3(5, 20, 5), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
	wall = AddOrientedCubeToWorld(offsetPos + Vector3(20, 11, -20), Vector3(10, 20, 10), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
	wall = AddOrientedCubeToWorld(offsetPos + Vector3(-10, 11, -45), Vector3(10, 20, 5), 0);
	wall->GetRenderObject()->SetColour(Vector4(0, 0, 0, 1));
	L2Walls.emplace_back(wall);
}

GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject("sphere");

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, ballTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->setFriction(0.8f);
	sphere->GetPhysicsObject()->setElasticity(0.7f);
	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(sphere);

	return sphere;
}


GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new ButtonGameObject();

	AABBVolume* volume = new AABBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* NCL::CSC8503::TutorialGame::AddOrientedCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass)
{
	GameObject* cube = new GameObject();

	OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, cubeTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* NCL::CSC8503::TutorialGame::AddFlipperToWorld(const Vector3& position, const Vector3& size)
{
	GameObject* lock = new GameObject("lock");
	Flipper* flipper = new Flipper("flip1",size,position,lock);
	flipper->setSpeed(500.0f);
	((GameObject*)flipper)->SetRenderObject(new RenderObject(&((GameObject*)flipper)->GetTransform(), cubeMesh, cubeTex, basicShader));
	if(((GameObject*)flipper)->GetRenderObject())((GameObject*)flipper)->GetRenderObject()->SetColour(Vector4(1,0,0,1));
	PositionConstraint* constraint = new PositionConstraint(flipper->getLock(), (GameObject*)flipper, 0);
	world->AddGameObject((GameObject*) flipper);
	world->AddGameObject(lock);
	world->AddConstraint(constraint);
	return (GameObject*)flipper;
}

GameObject* NCL::CSC8503::TutorialGame::AddButtonToWorld(const Vector2& position, const Vector2& size, string name, StateType type)
{
	float zPos = 5;
	float zSize = 5;
	ButtonGameObject* button = (ButtonGameObject*)AddCubeToWorld(Vector3(zPos,position.y, position.x),Vector3(size.x,size.y, zSize),0);
	button->SetName("Button");
	button->SetRenderObject(nullptr);
	if(button->GetRenderObject())button->GetRenderObject()->SetColour(Vector4(0,0,0,1));
	button->SetObjColour(Vector4(0, 0, 0, 1));
	button->SetButtonType(type);
	return (GameObject*)button;
}

void NCL::CSC8503::TutorialGame::UpdateTiltTable(float dt)
{
	float multi = 10;
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
		table->leanForward(dt);
		ball->GetPhysicsObject()->AddForce(Vector3(1 * multi, 0, 0));
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
		table->leanBack(dt);
		ball->GetPhysicsObject()->AddForce(Vector3(-1 * multi, 0, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
		table->leanLeft(dt);
		ball->GetPhysicsObject()->AddForce(Vector3(0, 0, -1 * multi));
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
		table->leanRight(dt);
		ball->GetPhysicsObject()->AddForce(Vector3(0, 0, 1 * multi));
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::N)) {
		shouldBuildBridge = !shouldBuildBridge;
	}
}

void NCL::CSC8503::TutorialGame::UpdatePlayer(float dt)
{
	float multi = 36;
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
		playChar->GetPhysicsObject()->AddForce(Vector3(1 * multi, 0, 0));
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
		playChar->GetPhysicsObject()->AddForce(Vector3(-1 * multi, 0, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
		playChar->GetPhysicsObject()->AddForce(Vector3(0, 0, -1 * multi));
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
		playChar->GetPhysicsObject()->AddForce(Vector3(0, 0, 1 * multi));
	}
}

void NCL::CSC8503::TutorialGame::UpdateEnemy(float dt)
{
	if (!is2Player) {
		enemyStates->Update(dt);

		float multi = 30;
		NavigationPath outPath;
		Vector3 enemyPos = enemyChar->GetTransform().GetPosition();
		Vector3 playerPos = pathfindTarget->GetTransform().GetPosition();
		bool found = grid->FindPath(enemyPos, playerPos, outPath);
		Vector3 pos;
		testNodes.clear();

		if (outPath.PopWaypoint(pos)) testNodes.push_back(pos);
		if (outPath.PopWaypoint(pos)) testNodes.push_back(pos);
		Vector3 dir = (pos - enemyPos).Normalised();
		while (outPath.PopWaypoint(pos)) {
			testNodes.push_back(pos);
		}
		enemyChar->GetPhysicsObject()->AddForce(Vector3(dir.x, 0, dir.z) * multi);
	}
	else {
		float multi = 36;
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::D)) {
			enemyChar->GetPhysicsObject()->AddForce(Vector3(1 * multi, 0, 0));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::A)) {
			enemyChar->GetPhysicsObject()->AddForce(Vector3(-1 * multi, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::W)) {
			enemyChar->GetPhysicsObject()->AddForce(Vector3(0, 0, -1 * multi));
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::S)) {
			enemyChar->GetPhysicsObject()->AddForce(Vector3(0, 0, 1 * multi));
		}
	}
}

void NCL::CSC8503::TutorialGame::DisplayPathfinding()
{
	for (int i = 1; i < testNodes.size(); ++i) {
		Vector3 a = Vector3(testNodes[i - 1]);
		Vector3 b = Vector3(testNodes[i]);
		a.y = 5;
		b.y = 5;
		Debug::DrawLine(a, b, Vector4(1, 0, 0, 1));
	}
}

void NCL::CSC8503::TutorialGame::spawnBonus(float dt)
{
	bonusSpawnTimer += dt;
	if (bonusSpawnTimer > bonusSpawnMax) {
		bonusSpawnTimer -= bonusSpawnMax;

		int randX = std::rand() % (10 - 1) + 1;
		int randZ = std::rand() % (10 - 1) + 1;
		Vector3 bonusPos((randX * 10) + 5,5,(randZ * 10) + 5);

		GameObject* bonus = AddSphereToWorld(bonusPos,2,0);
		bonus->GetRenderObject()->SetDefaultTexture(basicTex);
		bonus->GetRenderObject()->SetColour(Vector4(1,0.94f,0.32f,1));
		bonus->GetPhysicsObject()->setElasticity(0);
		bonus->SetName("bonus");

		bonuses.emplace_back(bonus);
	}
}

void NCL::CSC8503::TutorialGame::outputPoints()
{
	string playerPoints = "PLAYER: " + std::to_string(playChar->GetPoints());
	string enemyPoints;
	if (is2Player) enemyPoints = "PLAYER 2: " + std::to_string(enemyChar->GetPoints());
	else enemyPoints = "ENEMY: " + std::to_string(enemyChar->GetPoints());

	renderer->DrawString(playerPoints,Vector2(5,5),Vector4(0,1,1,1));
	renderer->DrawString(enemyPoints, Vector2(75, 5), Vector4(1, 0, 0, 1));
}

void NCL::CSC8503::TutorialGame::showEndScreen()
{
	if (hasWon) {
		if (currentState == StateType::Level1) {
			if (ball->HasWon()) {
				renderer->DrawString("YOU WON!", Vector2(35, 30), Vector4(0, 1, 0, 1), 40);
				renderer->DrawString("YOU WON!", Vector2(35.5, 30.5), Vector4(0, 0, 0, 1), 40);
			}
			else {
				renderer->DrawString("YOU LOST!", Vector2(35, 30), Vector4(1, 0, 0, 1), 40);
				renderer->DrawString("YOU LOST!", Vector2(35.5, 30.5), Vector4(0, 0, 0, 1), 40);
			}
			renderer->DrawString("IT TOOK YOU: " + std::to_string((int)gameTimer) +"s", Vector2(32, 40), Vector4(1, 0, 0, 1), 30);
			renderer->DrawString("IT TOOK YOU: " + std::to_string((int)gameTimer) + "s", Vector2(32.5, 40.5), Vector4(0, 0, 0, 1), 30);

			
		}
		if (currentState == StateType::Level2) {

			if (playChar->GetPoints() > enemyChar->GetPoints()) {
				if (is2Player) {
					renderer->DrawString("PLAYER 1 WON!", Vector2(35, 30), Vector4(0, 1, 0, 1), 40);
					renderer->DrawString("PLAYER 1 WON!", Vector2(35.5, 30.5), Vector4(0, 0, 0, 1), 40);
				}
				else {
					renderer->DrawString("YOU WON!", Vector2(35, 30), Vector4(0, 1, 0, 1), 40);
					renderer->DrawString("YOU WON!", Vector2(35.5, 30.5), Vector4(0, 0, 0, 1), 40);
				}
			
			}
			else if(playChar->GetPoints() < enemyChar->GetPoints()){
				if (is2Player) {
					renderer->DrawString("PLAYER 2 WON!", Vector2(35, 30), Vector4(0, 1, 0, 1), 40);
					renderer->DrawString("PLAYER 2 WON!", Vector2(35.5, 30.5), Vector4(0, 0, 0, 1), 40);
				}
				else {
					renderer->DrawString("YOU LOSE!", Vector2(35, 30), Vector4(1, 0, 0, 1), 40);
					renderer->DrawString("YOU LOSE!", Vector2(35.5, 30.5), Vector4(0, 0, 0, 1), 40);
				}
			}
			else {
				renderer->DrawString("YOU DRAW!", Vector2(35, 30), Vector4(0, 1, 1, 1), 40);
				renderer->DrawString("YOU DRAW!", Vector2(35.5, 30.5), Vector4(0, 0, 0, 1), 40);
			}
		}
		renderer->DrawString("PRESS M TO RETURN TO MENU", Vector2(30, 50), Vector4(1, 0, 0, 1), 20);
		renderer->DrawString("PRESS M TO RETURN TO MENU", Vector2(30.25, 50.25), Vector4(0, 0, 0, 1), 20);
		renderer->DrawString("PRESS R TO RESTART", Vector2(30, 55), Vector4(1, 0, 0, 1), 20);
		renderer->DrawString("PRESS R TO RESTART", Vector2(30.25, 55.25), Vector4(0, 0, 0, 1), 20);
	}
}

void NCL::CSC8503::TutorialGame::makeEnemyStateMachine()
{
	enemyStates = new StateMachine();
	State* goToPlayer = new State([&](float dt)-> void {
		if (this->hasChangedStates) {
			this->hasChangedStates = false;
			this->setPathfindtarget(this->getPlayer());
		}
		});
	State* goToBonus = new State([&](float dt)->void {
		if (this->hasChangedStates) {
			this->hasChangedStates = false;
			if (this->getBonuses()->size() > 0)this->setPathfindtarget((*this->getBonuses())[0]);
		}
		if (this->getPathfindTarget()->ShouldDelete()) {
				if(this->getBonuses()->size() > 0)this->setPathfindtarget((*this->getBonuses())[0]);
		}
		});
	enemyStates->AddState(goToBonus);
	enemyStates->AddState(goToPlayer);

	enemyStates->AddTransition(new StateTransition(goToPlayer, goToBonus, 
		[&]()->bool {
			bool val = this->enemyChar->GetPoints() <= this->getPlayer()->GetPoints() && this->getBonuses()->size() > 0;
			if (val) this->hasChangedStates = true;
			return val;
		}));
	enemyStates->AddTransition(new StateTransition(goToBonus, goToPlayer,
		[&]()->bool {

			bool val = this->getBonuses()->size() < 1 || this->enemyChar->GetPoints() > this->getPlayer()->GetPoints();

			if (val) this->hasChangedStates = true;
			return val;
		}));
}

bool TutorialGame::SelectObject() {
	if (inSelectionMode) {
		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::LEFT)) {
			if (selectionObject) {
				selectionObject = nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {
				selectionObject = (GameObject*)closestCollision.node;				
				if ((ButtonGameObject*)selectionObject) {
					ButtonGameObject* selObj = (ButtonGameObject*) selectionObject;
					if (selObj->GetButtonType() == ((ButtonGameObject*)L1Button)->GetButtonType()) L1ButtonClicked = true;
					else if (selObj->GetButtonType() == ((ButtonGameObject*)L2Button)->GetButtonType())L2ButtonClicked = true;
					else if (selObj->GetButtonType() == ((ButtonGameObject*)ExitButton)->GetButtonType()) ExitButtonClicked = true;
				}
				return true;
			}
			else return false;
		}
	}
	return false;
}