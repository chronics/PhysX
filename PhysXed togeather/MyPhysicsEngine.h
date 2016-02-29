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
		PxVec3(255.f / 255.f,45.f / 255.f,0.f / 255.f),		//red			8
		PxVec3(255.f / 255.f,140.f / 255.f,54.f / 255.f),	//cream			9
		PxVec3(4.f / 255.f,117.f / 255.f,111.f / 255.f),	//turquoise		10
		PxVec3(147.f / 255.f,112.f / 255.f,219.f / 255.f)	//mediumpurple	11	
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
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST " << endl;
						trigger = false;
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
		Plane* plane;
		Box* box, * box2;
		MySimulationEventCallback* my_callback;
		
		//my objects
		CylinderStatic* StaticCyl;
		Cylinder* cyl, *cyl1;


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

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			CustomLevel1();
			CustomActors();
			CustomJoints();
			
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}

		//static object vars
			Wall3x1x1* playerStartL;
			Wall3x2x1* wall3_2;
			Wall2x1x1* PlayerStartR, *wall3_1;
			Goal* goal1;

		// create static objects in the world
		virtual void CustomLevel1()
		{
			
			//floor
			plane = new Plane();
			plane->Color(PxVec3(210.f / 255.f, 210.f / 255.f, 210.f / 255.f));
			Add(plane);

			//left start block
			playerStartL = new Wall3x1x1(PxTransform(PxVec3(-2.f, .5f, .0f)));
			playerStartL->Color(PxVec3(1.f / 255.f, 1.f / 255.f, 1.f / 255.f));
			Add(playerStartL);
			
			//right start block
			PlayerStartR = new Wall2x1x1(PxTransform(PxVec3(3.5f, .5f, .0f)));
			PlayerStartR->Color(PxVec3(1.f / 255.f, 1.f / 255.f, 1.f / 255.f));
			Add(PlayerStartR);

			//left of goal
			wall3_2 = new Wall3x2x1(PxTransform(PxVec3(1.f, 1.f, .0f)));
			wall3_2->GetShape(0)->setLocalPose(PxTransform(PxVec3(.0f, .0f, .0f)/*, PxQuat(PxPi / 2, PxVec3(.0f, .0f, 1.f))*/));	//rotate on the Z axis
			wall3_2->Color(PxVec3(1.f / 255.f, 1.f / 255.f, 1.f / 255.f));
			Add(wall3_2);
			
			//goal 
			goal1 = new Goal(PxTransform(PxVec3(1.5f, 2.25f, .0f)));
			goal1->Color(color_palette[1]);
			goal1->SetTrigger(my_callback);
			Add(goal1);	

		}

		//dynamic object vars
		playerbox* player1;

		virtual void CustomActors()
		{	
			/*player1 = new playerbox(PxTransform(PxVec3(.0f, 1.5f, .0f)));											//set the globel pose
			player1->GetShape(0)->setLocalPose(PxTransform(PxVec3(-3.f, .0f, .0f)));								//set the offset of the 1st object
			player1->GetShape(1)->setLocalPose(PxTransform(PxVec3(4.f, .0f, .0f)));									//set the offset of the 2nd object
			player1->GetShape(0)->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 && FilterGroup::ACTOR3);
			player1->GetShape(1)->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR2 && FilterGroup::ACTOR3);
			player1->Color(color_palette[2]);																		//set colour to blue
			player1->Name("Player");																				//set the name of the object
			Add(player1);*/																							//add the object to the simulation

			box = new Box(PxTransform(PxVec3(-3.f, 3.f, .0f)));
			box->Color(color_palette[2]);
			box->Name("player1");
			//box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1);
			Add(box);

			box2 = new Box(PxTransform(PxVec3(4.f, 3.f, .0f)));
			box2->Color(color_palette[3]);
			box2->Name("palyer1.5");
			//box->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);
			Add(box2);
			
			/*cyl = new Cylinder(PxTransform(PxVec3(2.f, 1.5f, .0f)));
			cyl->Color(color_palette[11]);
			Add(cyl);

			cyl1 = new Cylinder(PxTransform(PxVec3(-2.f, 1.5f, .0f)));
			cyl1->Color(color_palette[10]);
			Add(cyl1);*/
		}

		virtual void CustomJoints()
		{
			//RevoluteJoint joint(box, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), cyl, PxTransform(PxVec3(0.f, 5.f, 0.f)));
			//DistanceJoint DJoint(cyl1, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), cyl, PxTransform(PxVec3(0.f, 5.f, 0.f)));
		}

	};
}
