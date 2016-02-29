#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	class Goal : public StaticActor
	{
	public:
		Goal(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(1.f, .25f, .49f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class Wall1x1x1 : public StaticActor
	{
	public:
		Wall1x1x1(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class Wall2x1x1 : public StaticActor
	{
	public:
		Wall2x1x1(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(1.f, .5f, .5f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class Wall3x1x1 : public StaticActor
	{
	public:
		Wall3x1x1(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(1.5f, .5f, .5f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class Wall3x2x1 : public StaticActor
	{
	public:
		Wall3x2x1(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(1.5f, 1.f, .5f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};


	static PxVec3 cylinder_verts[] =
	{
		PxVec3(0,.5,0),			//0
		PxVec3(0,.5,.5),		//1
		PxVec3(.33,.5,.33),		//2
		PxVec3(.5,.5,0),		//3
		PxVec3(.33,.5,-.33),	//4
		PxVec3(0,.5,-.5),		//5
		PxVec3(-.33,.5,-.33),	//6
		PxVec3(-.5,.5,0),		//7
		PxVec3(-.33,.5,.33),	//8

		PxVec3(0,-.5,0),		//9
		PxVec3(0,-.5,.5),		//10
		PxVec3(.33,-.5,.33),	//11
		PxVec3(.5,-.5,0),		//12
		PxVec3(.33,-.5,-.33),	//13
		PxVec3(0,-.5,-.5),		//14
		PxVec3(-.33,-.5,-.33),	//15
		PxVec3(-.5,-.5,0),		//16
		PxVec3(-.33,-.5,.33),	//17


	};

	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 cylinder_trigs[] = 
	{
		0,1,2,		0,2,3,		0,3,4,		0,4,5,		0,5,6,		0,6,7,		0,7,8,		0,8,1, //top
		9,10,11,	9,11,12,	9,12,13,	9,13,14,	9,14,15,	9,15,16,	9,16,17,	9,17,10, //bottom

		//sides 
		1,10,2,		10,11,2,	 3,2,11,	11,12,3,	4,3,12,		12,13,4,	5,4,13,		13,14,5,
		6,5,14,		14,15,6,	7,6,15,		15,16,7,	8,7,16,		16,17,8,	1,8,17,		17,10,1
	};

	class Cylinder : public ConvexMesh
	{
	public:
		Cylinder(PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.f) :
			ConvexMesh(vector<PxVec3>(begin(cylinder_verts), end(cylinder_verts)), pose, density)
		{
		}
	};

	class CylinderStatic : public TriangleMesh
	{
	public:
		CylinderStatic(PxTransform pose = PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(cylinder_verts), end(cylinder_verts)), vector<PxU32>(begin(cylinder_trigs), end(cylinder_trigs)), pose)
		{
		}
	};

	class playerbox : public DynamicActor
	{
	public:
		playerbox(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};
}