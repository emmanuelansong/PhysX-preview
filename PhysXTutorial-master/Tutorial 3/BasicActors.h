#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>
#include <math.h>   

namespace PhysicsEngine
{
	class Domino : public DynamicActor
	{
	public:
		float mass = 0.0008; //kg
		//Domino Class
		// - pose in 0,0,0
		// - dimensions: 1m (W) x 1m (L) x 1m (H)
		// - denisty: 1kg/m^3
		Domino(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.008f, 0.05f, 0.025f), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			density = mass / (dimensions.x * dimensions.y * dimensions.z);
			CreateShape(PxBoxGeometry(dimensions), density);
			
			
		}


	};


	class Plank : public DynamicActor
	{
	public:
		float mass = 1; //kg
		//Domino Class
		// - pose in 0,0,0
		// - dimensions: 1m (W) x 1m (L) x 1m (H)
		// - denisty: 1kg/m^3
		Plank(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(3.f, 0.1f, 0.3f), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			density = mass / (dimensions.x * dimensions.y * dimensions.z);
			CreateShape(PxBoxGeometry(dimensions), density);


		}


	};
	class InitActor : public DynamicActor
	{
		
	public:

		InitActor(const PxTransform& pose = PxTransform(PxIdentity), PxReal radius = .5f, PxReal density = 1.)
			: DynamicActor(pose)
		{
			float volume = (4 / 3) * PxPi * (radius * radius * radius);
			float mass = 1.f;
			density = mass / volume;
			CreateShape(PxSphereGeometry(radius), density);


		}
		
	};

	class Anvil : public DynamicActor
	{
	public:
		float mass = 50; //kg
		//Domino Class
		// - pose in 0,0,0
		// - dimensions: 1m (W) x 1m (L) x 1m (H)
		// - denisty: 1kg/m^3
		Anvil(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.6, 0.6, 0.6), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			density = mass / (dimensions.x * dimensions.y * dimensions.z);

			CreateShape(PxBoxGeometry(0.6, 0.16, 0.45), density);

			CreateShape(PxBoxGeometry(0.3, 0.29, 0.15), density);
			Actor::GetShape(1)->setLocalPose(PxTransform(PxVec3(0., -0.15, 0.f)));

			CreateShape(PxBoxGeometry(0.35, 0.05, 0.2), density);
			Actor::GetShape(2)->setLocalPose(PxTransform(PxVec3(0., -0.5, 0.f)));

			CreateShape(PxBoxGeometry(0.38, 0.1, 0.35), density);
			Actor::GetShape(3)->setLocalPose(PxTransform(PxVec3(0., -.6, 0.f)));

		}


	};

	//crane class
	class Windmill : public StaticActor
	{
	public:
		//Compound
		
		Windmill(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(1.f, 15.f, 1.f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			
			float volume = dimensions.x * dimensions.y * dimensions.z;
			float mass = 20000.f;
			density = mass / volume;

			//Actor::Get();
			//base
			CreateShape(PxBoxGeometry(dimensions), density);


			//pole vertical
			CreateShape(PxBoxGeometry(.5f, 5.f, .5f), density);
			Actor::GetShape(1)->setLocalPose(PxTransform(PxVec3(0., 5.f, .0f)));
			
			//box at top
			CreateShape(PxBoxGeometry(.75f, .75f, .75f), density);
			Actor::GetShape(2)->setLocalPose(PxTransform(PxVec3(0, 10., 0.f)));
			
			//pivot
			//CreateShape(PxBoxGeometry(.25f, .25f, .25f), density);
			//Actor::GetShape(3)->setLocalPose(PxTransform(PxVec3(0, 10., 1.f)));
			
			//Actor::GetShape(2)->setLocalPose(PxTransform(PxVec3(-1., .5f, .0f)));



		}

	};
	class WindmillBlades : public DynamicActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		WindmillBlades(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.25f, 1.f, .25f), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			//pivot
			CreateShape(PxBoxGeometry(PxVec3(.5, .25, .25)), density);

			//blade 1
			CreateShape(PxBoxGeometry(PxVec3(.15, 9.95, .15)), density);
			Actor::GetShape(1)->setLocalPose(PxTransform(PxVec3(-1., 0., 0.f)));

			//blade 2
			CreateShape(PxBoxGeometry(PxVec3(.15, .15, 9.95)), density);
			Actor::GetShape(2)->setLocalPose(PxTransform(PxVec3(-1., 0., 0.f)));

		}


	};


	

	///Plane class
	class Plane : public StaticActor
	{
	public:
		//A plane with default paramters: XZ plane centred at (0,0,0)
		Plane(PxVec3 normal=PxVec3(0.f, 1.f, 0.f), PxReal distance=0.f) 
			: StaticActor(PxTransformFromPlaneEquation(PxPlane(normal, distance)))
		{
			CreateShape(PxPlaneGeometry());
		}
	};

	
	///Sphere class
	class Sphere : public DynamicActor
	{
	public:
		//a sphere with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m
		// - denisty: 1kg/m^3
		Sphere(const PxTransform& pose=PxTransform(PxIdentity), PxReal radius=.05f, PxReal density=1.f) 
			: DynamicActor(pose)
		{ 
			CreateShape(PxSphereGeometry(radius), density);
		}
	};
	class Stand : public StaticActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		Stand(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(1.f, 1.f, .5f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);

		}
	};
	///Box class
	class Box : public DynamicActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		Box(const PxTransform& pose=PxTransform(PxIdentity), PxVec3 dimensions=PxVec3(.5f,.5f,.5f), PxReal density=1.f) 
			: DynamicActor(pose)
		{ 
			CreateShape(PxBoxGeometry(dimensions), density);
			
		}
	};

	class Capsule : public DynamicActor
	{
	public:
		Capsule(const PxTransform& pose=PxTransform(PxIdentity), PxVec2 dimensions=PxVec2(1.f,1.f), PxReal density=1.f) 
			: DynamicActor(pose)
		{
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
		}
	};

	///The ConvexMesh class
	class ConvexMesh : public DynamicActor
	{
	public:
		//constructor
		ConvexMesh(const std::vector<PxVec3>& verts, const PxTransform& pose=PxTransform(PxIdentity), PxReal density=1.f)
			: DynamicActor(pose)
		{
			PxConvexMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
			mesh_desc.vertexLimit = 256;

			CreateShape(PxConvexMeshGeometry(CookMesh(mesh_desc)), density);
		}

		//mesh cooking (preparation)
		PxConvexMesh* CookMesh(const PxConvexMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if(!GetCooking()->cookConvexMesh(mesh_desc, stream))
				throw new Exception("ConvexMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createConvexMesh(input);
		}
	};

	///The TriangleMesh class
	class TriangleMesh : public StaticActor
	{
	public:
		//constructor
		TriangleMesh(const std::vector<PxVec3>& verts, const std::vector<PxU32>& trigs, const PxTransform& pose=PxTransform(PxIdentity))
			: StaticActor(pose)
		{
			PxTriangleMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.triangles.count = (PxU32)trigs.size()/3;
			mesh_desc.triangles.stride = 3*sizeof(PxU32);
			mesh_desc.triangles.data = &trigs.front();

			CreateShape(PxTriangleMeshGeometry(CookMesh(mesh_desc)));
		}

		//mesh cooking (preparation)
		PxTriangleMesh* CookMesh(const PxTriangleMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if(!GetCooking()->cookTriangleMesh(mesh_desc, stream))
				throw new Exception("TriangleMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createTriangleMesh(input);
		}
	};

	//Distance joint with the springs switched on
	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = (PxJoint*)PxDistanceJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
			((PxDistanceJoint*)joint)->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
			Damping(1.f);
			Stiffness(1.f);
		}

		void Stiffness(PxReal value)
		{
			((PxDistanceJoint*)joint)->setStiffness(value);
		}

		PxReal Stiffness()
		{
			return ((PxDistanceJoint*)joint)->getStiffness();		
		}

		void Damping(PxReal value)
		{
			((PxDistanceJoint*)joint)->setDamping(value);
		}

		PxReal Damping()
		{
			return ((PxDistanceJoint*)joint)->getDamping();
		}
	};

	///Revolute Joint
	class RevoluteJoint : public Joint
	{
	public:
		RevoluteJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = PxRevoluteJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION,true);
		}

		void DriveVelocity(PxReal value)
		{
			//wake up the attached actors
			PxRigidDynamic *actor_0, *actor_1;
			((PxRevoluteJoint*)joint)->getActors((PxRigidActor*&)actor_0, (PxRigidActor*&)actor_1);
			if (actor_0)
			{
				if (actor_0->isSleeping())
					actor_0->wakeUp();
			}
			if (actor_1)
			{
				if (actor_1->isSleeping())
					actor_1->wakeUp();
			}
			((PxRevoluteJoint*)joint)->setDriveVelocity(value);
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
		}

		PxReal DriveVelocity()
		{
			return ((PxRevoluteJoint*)joint)->getDriveVelocity();
		}

		void SetLimits(PxReal lower, PxReal upper)
		{
			((PxRevoluteJoint*)joint)->setLimit(PxJointAngularLimitPair(lower, upper));
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
		}
	};
}