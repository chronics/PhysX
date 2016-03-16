#pragma once

#include "BasicActors.h"
#include "MyActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = 
	{ 
		PxVec3(255.f / 255.f,.0f / 255.f,.0f / 255.f),		//red			0
		PxVec3(255.f / 255.f,255.f / 255.f,.0f / 255.f),	//yellow		1
		PxVec3(.0f / 255.f,.0f / 255.f,255.f / 255.f),		//blue			2

		PxVec3(128.f / 255.f,.0f / 255.f,128.f / 255.f),	//purple		3
		PxVec3(.0f / 255.f,255.f / 255.f,.0f / 255.f),		//green			4
		PxVec3(255.f / 255.f,50.f / 255.f,.0f / 255.f),		//orange		5

		PxVec3(46.f / 255.f,9.f / 255.f,39.f / 255.f),		//pink			6
		PxVec3(217.f / 255.f,0.f / 255.f,0.f / 255.f),		//salmon pink	7
		PxVec3(139.f / 255.f,69.f / 255.f,19.f / 255.f),	//brown			8
		PxVec3(255.f / 255.f,140.f / 255.f,54.f / 255.f),	//cream			9
		PxVec3(4.f / 255.f,117.f / 255.f,111.f / 255.f),	//turquoise		10
		PxVec3(147.f / 255.f,112.f / 255.f,219.f / 255.f),	//mediumpurple	11	

		PxVec3(.0f / 255.f,102.f / 255.f,0.f / 255.f),		//Dark Green	12	
		PxVec3(.0f / 255.f,102.f / 255.f,204.f / 255.f)		//skyish blue	13	
	};

	//pyramid vertices
	static PxVec3 pyramid_verts[] = 
	{
		PxVec3(0,1,0),
		PxVec3(1,0,0),
		PxVec3(-1,0,0), 
		PxVec3(0,0,1), 
		PxVec3(0,0,-1)
	};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {
		1, 4, 0, 
		3, 1, 0,
		2, 3, 0,
		4, 2, 0,
		3, 2, 1, 
		2, 4, 1};

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose=PxTransform(PxIdentity), PxReal density=1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2),
			ACTOR3		= (2 << 2)
			//add more if you need
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(0.f,thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			top = new Box(PxTransform(PxVec3(0.f,dimensions.y+thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,-dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,-dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);
		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};
	
	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;
		int collisions = 0; //create a new int for later use
		int level = 1;

		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND " << endl;
						trigger = true;
						collisions++; //incriment this int when a collision is detected 
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST " << endl;
						trigger = false;
						collisions--; // reset when collision is lost
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{

		
	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene() {};


		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
		}

		PxReal gForceStrength = 20;

		Plane* plane;
		Box* box, * box2;
		MySimulationEventCallback* my_callback;

		backWall* _backWall;

		Wall1x1x1* smallWall_1, *LVL3_smallWall1, *LVL3_smallWall2, *LVL3_smallWall3, *LVL3_smallWall4;
		Wall2x1x1* midWall_1, *LVL2_midWall_1, *LVL3_midWall2;
		Wall3x1x1* longWall_1, *LVL1_LongWall_1, *LVL1_LongWall_2, *LVL2_LongWall_1;
		Wall3x2x1* bigWall_1, *LVL2_bigWall_1;
		Goal* goal1;

		PxRigidBody* a, *b;
		PxMaterial* Floor;

		CylinderStatic* cyl1, *cyl2;

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.2f);

			Floor = CreateMaterial(0.f, 2.f, 2.f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			//floor
			plane = new Plane();
			plane->Color(color_palette[12]);
			Add(plane);

			_backWall = new backWall(PxTransform(PxVec3(.0f, .0f, -1.1f)));
			_backWall->Color(color_palette[13]);
			Add(_backWall);

			CustomLevel1();
			CustomActors();
			CustomJoints();			
		}

		virtual void CustomLevel1()
		{

			//level 1 objects
			LVL1_LongWall_1 = new Wall3x1x1(PxTransform(PxVec3(-2.f, .5f, .0f))); 
			LVL1_LongWall_1->Color(color_palette[8]);
			Add(LVL1_LongWall_1);

			LVL1_LongWall_2 = new Wall3x1x1(PxTransform(PxVec3(1.f, .5f, .0f)));
			LVL1_LongWall_2->Color(color_palette[8]);
			Add(LVL1_LongWall_2);

			goal1 = new Goal(PxTransform(PxVec3(-.5f, 1.f, .0f)));
			goal1->Color(color_palette[1]);
			goal1->SetTrigger(my_callback);
			Add(goal1);


			LVL2_LongWall_1 = new Wall3x1x1(PxTransform(PxVec3(-2.f, 20.5f, .0f)));
			LVL2_LongWall_1->Color(color_palette[8]);
			Add(LVL2_LongWall_1);

			//right start block
			LVL2_midWall_1 = new Wall2x1x1(PxTransform(PxVec3(3.5f, 20.5f, .0f)));
			LVL2_midWall_1->Color(color_palette[8]);
			Add(LVL2_midWall_1);

			//left of goal
			LVL2_bigWall_1 = new Wall3x2x1(PxTransform(PxVec3(1.f, 21.f, .0f)));
			LVL2_bigWall_1->Color(color_palette[8]);
			Add(LVL2_bigWall_1);

			////level 3
			//LVL3_smallWall1 = new Wall1x1x1(PxTransform(PxVec3(-3.0f, 20.f, .0f)));
			//LVL3_smallWall1->Color(color_palette[8]);
			//Add(LVL3_smallWall1);

			//LVL3_smallWall2 = new Wall1x1x1(PxTransform(PxVec3(-2.0f, 20.f, .0f)));
			//LVL3_smallWall2->Color(color_palette[8]);
			//Add(LVL3_smallWall2);

			//LVL3_smallWall3 == new Wall1x1x1(PxTransform(PxVec3(2.0f, 20.f, .0f)));
			//LVL3_smallWall3->Color(color_palette[8]);
			//Add(LVL3_smallWall3);

			//LVL3_smallWall4 == new Wall1x1x1(PxTransform(PxVec3(3.0f, 20.f, .0f)));
			//LVL3_smallWall4->Color(color_palette[8]);
			//Add(LVL3_smallWall4);

			//LVL3_midWall2 == new Wall2x1x1(PxTransform(PxVec3(.0f, 20.f, .0f)));
			//LVL3_midWall2->Color(color_palette[8]);
			//Add(LVL3_midWall2);
		}

		virtual void CustomActors()
		{
			box = new Box(PxTransform(PxVec3(-3.f, 3.5f, .0f)));
			box->Color(color_palette[2]);
			box->Get()->setName("player1");
			Add(box);

			box2 = new Box(PxTransform(PxVec3(2.f, 3.5f, .0f)));
			box2->Color(color_palette[3]);
			box2->Name("palyer1.5");
			Add(box2);

			a = (PxRigidBody*)box->Get();
			b = (PxRigidBody*)box2->Get();
		}

		//D6Joint* d6joint;
		virtual void CustomJoints()
		{
			/*cyl1 = new CylinderStatic(PxTransform(PxVec3(.0f, .5f, .0f))); Add(cyl1);
			cyl2 = new CylinderStatic(PxTransform(PxVec3(.0f, 1.f, .0f))); Add(cyl2);*/

			//RevoluteJoint joint1(cyl1, PxTransform(PxVec3(0.f, 5.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), cyl2, PxTransform(PxVec3(0.f, 1.5f, 0.f)));
			//DistanceJoint DJoint(cyl1, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), cyl2, PxTransform(PxVec3(0.f, 5.f, 0.f)));
			/*d6joint= new D6Joint(cyl1, PxTransform(PxVec3(0.f, 5.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), cyl2, PxTransform(PxVec3(0.f, 1.5f, 0.f)));
			d6joint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);*/
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			
			if (my_callback->level == 1 && my_callback->collisions >= 2) 
			{
				my_callback->level++;
			}
			else if (my_callback->level == 2 && my_callback->collisions >= 2) //find out if this variable within this class is 2 if so...
			{
				cerr << "two collisions there for level complete " << endl;
				a->setGlobalPose(PxTransform(PxVec3(-3.f, 3.5f, .0f)));	//move actor a
				b->setGlobalPose(PxTransform(PxVec3(4.f, 3.5f, .0f)));	//move actor b

				//move walls to change the levels
				LVL1_LongWall_1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -4.0f, .0f)));
				LVL1_LongWall_2->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -4.0f, .0f)));
				LVL2_LongWall_1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -20.0f, .0f)));
				LVL2_midWall_1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -20.0f, .0f)));
				LVL2_bigWall_1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -20.0f, .0f)));

				goal1->GetShape()->setLocalPose(PxTransform(PxVec3(2.f, 1.25f, .0f)));	// move the goal to teh new location
			}
			//else if (my_callback->level == 3 && my_callback->collisions >= 2) //load level 3
			//{
			//	LVL2_LongWall_1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -30.0f, .0f)));
			//	LVL2_midWall_1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -30.0f, .0f)));
			//	LVL2_bigWall_1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -30.0f, .0f)));
			//	
			//	LVL3_smallWall1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -17.0f, .0f)));
			//	LVL3_smallWall2->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -14.0f, .0f)));
			//	LVL3_smallWall3->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -14.0f, .0f)));
			//	LVL3_smallWall4->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -17.0f, .0f)));
			//	LVL3_midWall2->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, -18.f, .0f)));

			//	goal1->GetShape()->setLocalPose(PxTransform(PxVec3(0.f, 18.25f, .0f)));
			//}

		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			//cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandlerD()
		{
			cerr << "I am pressed! : D " << endl;
			
				a->addForce(PxVec3(1, 0, 0)*gForceStrength); //add force in direction *20
				b->addForce(PxVec3(-1, 0, 0)*gForceStrength);
			}

		/// An example use of key presse handling
		void ExampleKeyPressHandlerA()
		{
			cerr << "I am pressed! : A " << endl;

			
				a->addForce(PxVec3(-1, 0, 0)*gForceStrength);
				b->addForce(PxVec3(1, 0, 0)*gForceStrength);
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandlerW()
		{
			cerr << "I am pressed! : W " << endl;
			
			
				a->addForce(PxVec3(0, 260, 0));
				b->addForce(PxVec3(0, 260, 0));
		}	
	};
}
