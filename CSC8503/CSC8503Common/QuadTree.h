#pragma once
#include "../../Common/Vector2.h"
#include "../CSC8503Common/CollisionDetection.h"
#include "Debug.h"
#include <list>
#include <functional>

namespace NCL {
	using namespace NCL::Maths;
	namespace CSC8503 {
		template<class T>
		class QuadTree;

		template<class T>
		struct QuadTreeEntry {
			Vector3 pos;
			Vector3 size;
			T object;

			QuadTreeEntry(T obj, Vector3 pos, Vector3 size) {
				object		= obj;
				this->pos	= pos;
				this->size	= size;
			}
		};

		template<class T>
		class QuadTreeNode	{
		public:
			typedef std::function<void(std::list<QuadTreeEntry<T>>&)> QuadTreeFunc;
		protected:
			friend class QuadTree<T>;

			QuadTreeNode() {}

			QuadTreeNode(Vector2 pos, Vector2 size) {
				children		= nullptr;
				this->position	= pos;
				this->size		= size;
			}

			~QuadTreeNode() {
				delete[] children;
			}

			void Insert(T& object, const Vector3& objectPos, const Vector3& objectSize, int depthLeft, int maxSize) {
				if (!CollisionDetection::AABBTest(objectPos, Vector3(position.x, -100, position.y), objectSize, Vector3(size.x, 1000.0f, size.y))) {
					return;
				}
				if (children) for (int i = 0; i < 4; ++i) children[i].Insert(object, objectPos, objectSize, depthLeft - 1, maxSize);
				else {
					contents.push_back(QuadTreeEntry<T>(object, objectPos, objectSize));
					if ((int)contents.size() > maxSize && depthLeft > 0) {
						if (!children) {
							Split();
							for (const auto& i : contents) {
								for (int j = 0; j < 4; ++j) {
									auto entry = i;
									children[j].Insert(entry.object, entry.pos, entry.size, depthLeft - 1, maxSize);
								}
							}
							contents.clear();
						}
					}
				}
			}

			void Split() {
				Vector2 halfSize = size / 2.0f;
				children = new QuadTreeNode<T>[4];
				children[0] = QuadTreeNode<T>(position + Vector2(-halfSize.x, halfSize.y), halfSize);
				children[1] = QuadTreeNode<T>(position + Vector2(halfSize.x, halfSize.y), halfSize);
				children[2] = QuadTreeNode<T>(position + Vector2(-halfSize.x, -halfSize.y), halfSize);
				children[3] = QuadTreeNode<T>(position + Vector2(halfSize.x, -halfSize.y), halfSize);
			}

			void DebugDraw() {
				if (children) for (int i = 0; i < 4; ++i) children[i].DebugDraw();
				else {
					Vector3 halfSize = Vector3(size.x, 100.0f, size.y);
					Vector3 corners[8] = {
						Vector3(-halfSize.x,-halfSize.y,-halfSize.z),Vector3(halfSize.x,-halfSize.y,-halfSize.z),
						Vector3(halfSize.x,-halfSize.y,halfSize.z),Vector3(-halfSize.x,-halfSize.y,halfSize.z),
						Vector3(-halfSize.x,halfSize.y,-halfSize.z),Vector3(halfSize.x,halfSize.y,-halfSize.z),
						Vector3(halfSize.x,halfSize.y,halfSize.z),Vector3(-halfSize.x,halfSize.y,halfSize.z)
					};

					for (int i = 0; i < 8; ++i) corners[i] += Vector3(position.x,-50,position.y);
					Vector3 startPos;
					Vector3 endPos;
					startPos = corners[4];
					endPos = corners[5];
					Debug::DrawLine(startPos, endPos, Vector4(0, 0, 1, 1));
					startPos = corners[5];
					endPos = corners[6];
					Debug::DrawLine(startPos, endPos, Vector4(0, 0, 1, 1));
					startPos = corners[6];
					endPos = corners[7];
					Debug::DrawLine(startPos, endPos, Vector4(0, 0, 1, 1));
					startPos = corners[7];
					endPos = corners[4];
					Debug::DrawLine(startPos, endPos, Vector4(0, 0, 1, 1));
				}
			}

			void OperateOnContents(QuadTreeFunc& func) {
				if (children) for (int i = 0; i < 4; ++i) children[i].OperateOnContents(func);
				else if (!contents.empty()) func(contents);
			}

		protected:
			std::list< QuadTreeEntry<T> >	contents;

			Vector2 position;
			Vector2 size;

			QuadTreeNode<T>* children;
		};
	}
}


namespace NCL {
	using namespace NCL::Maths;
	namespace CSC8503 {
		template<class T>
		class QuadTree
		{
		public:
			QuadTree(Vector2 size, int maxDepth = 6, int maxSize = 5){
				Vector2 pos = Vector2(0,0);
				root = QuadTreeNode<T>(pos, size);
				this->maxDepth	= maxDepth;
				this->maxSize	= maxSize;
			}
			~QuadTree() {
			}

			void Insert(T object, const Vector3& pos, const Vector3& size) {
				root.Insert(object, pos, size, maxDepth, maxSize);
			}

			void DebugDraw() {
				root.DebugDraw();
			}

			void OperateOnContents(typename QuadTreeNode<T>::QuadTreeFunc  func) {
				root.OperateOnContents(func);
			}

		protected:
			QuadTreeNode<T> root;
			int maxDepth;
			int maxSize;
		};
	}
}