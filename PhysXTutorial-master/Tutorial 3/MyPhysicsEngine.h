#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;


	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	//pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-01,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};


	//pyramid vertices
	static PxVec3 pyramid_verts2[] = { PxVec3(0,0,0), PxVec3(0,1,0), PxVec3(0,0,1), PxVec3(0,1,1), PxVec3(1,0,0), PxVec3(1,0,1)};
	//static PxVec3 pyramid_verts3[] = { PxVec3(0,0,0), PxVec3(0,1,0), PxVec3(0,0,1), PxVec3(0,1,1), PxVec3(1,0,0), PxVec3(1,0,1) };
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static const PxU32 pyramid_trigs2[] = { 2, 1, 0, 3, 1, 2, 3, 1, 5, 0, 4, 1, 1, 4, 3, 4, 5, 3,0,4,3 };
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
			

			//mainJoint = new RevoluteJoint(, PxTransform(PxVec3(0., 10, 1.5), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), pivot, PxTransform(PxVec3(0.f, 0.f, 0.)));
		}

		
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2)
			//add more if you need
		};
	};

	class Wedge : public ConvexMesh
	{

	public:
		Wedge(PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts2), end(pyramid_verts2)), pose, density)
		{
		}
	};

	class Ramp : public StaticActor
	{
		Wedge* wedge;
		PxMaterial* cardboard = CreateMaterial(0.65, 0, 0);
	public:
		Ramp(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			PxVec3 transform = PxVec3(.5, 0., 1.);
			wedge = new Wedge(PxTransform(PxVec3(transform.x,transform.y,transform.z)));
			PxTransform pose2 = ((PxRigidBody*)wedge->Get())->getGlobalPose();
			pose2.q = PxQuat(Deg2Rad(-90), PxVec3(0., 1., 0.));
			((PxRigidBody*)wedge->Get())->setGlobalPose(pose2);

		}

		void AddToScene(Scene* scene)
		{
			wedge->Color(color_palette[0]);
			wedge->Material(cardboard);
			scene->Add(wedge);
			//scene->Add(box);
		}
		float Deg2Rad(float degrees)
		{
			return degrees * (PxPi / 180);
		}
	};
	class Seesaw : public DynamicActor
	{
	public:
		
		RevoluteJoint* mainJoint;

		Pyramid* pivot;
		Box* box;
		Plank* plank;
		RevoluteJoint* joint;
		
		PxMaterial* leather_metal = CreateMaterial(0.4, 0, .5);
		Seesaw(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(2.f, 1.f, 2.f), PxReal density = 1.f)
			: DynamicActor(pose)
		{

			float volume = dimensions.x * dimensions.y * dimensions.z;
			float mass = 100.f;
			density = mass / volume;

			plank = new Plank(PxTransform(PxVec3(-2.5, 1.1, -4.f)), PxVec3(.3f, 0.1f, 3.f));
			pivot = new Pyramid(PxTransform(PxVec3(-2.5, 1, -4.f)));

			pivot->Color(color_palette[4]);
			joint = new RevoluteJoint(pivot, PxTransform(PxVec3(0., 1., 0), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), plank, PxTransform(PxVec3(0.f, 0.f, 0.)));

			PxTransform pose2 = ((PxRigidBody*)pivot->Get())->getGlobalPose();
			pose2.q = PxQuat(Deg2Rad(-10), PxVec3(0., 1., 0.));
			((PxRigidBody*)pivot->Get())->setGlobalPose(pose2);

			

		}
		void AddToScene(Scene* scene)
		{
			plank->Material(leather_metal);
			
			scene->Add(plank);
			scene->Add(pivot);
		

		}

		float Deg2Rad(float degrees)
		{
			return degrees * (PxPi / 180);
		}

	};
	class NewWindmill : public DynamicActor
	{
	public:
		//Compound
		WindmillBlades* pivot;
		Box* null;
		Windmill* windmill;
		WindmillBlades* blades;
		RevoluteJoint* mainJoint, *joint1;
		

		NewWindmill(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(2.f, 1.f, 2.f), PxReal density = 1.f)
			: DynamicActor(pose)
		{

			float volume = dimensions.x * dimensions.y * dimensions.z;
			float mass = 1.5f;
			density = mass / volume;

			windmill = new Windmill(PxTransform(PxVec3(0.f, 0., 0.f)), PxVec3(.75, dimensions.y, 1.));
			pivot = new WindmillBlades(PxTransform(PxVec3(0.f, 10., 0.f)), PxVec3(.5, .25, .25));
			pivot->Color(color_palette[4]);
			mainJoint = new RevoluteJoint(null,PxTransform(PxVec3(0., 10, 1.5), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), pivot, PxTransform(PxVec3(0.f, 0.f, 0.)));
			mainJoint->DriveVelocity(.5);

		}
		void AddToScene(Scene* scene)
		{
			scene->Add(windmill);
			scene->Add(pivot);

		}

	};
	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;
		PxMaterial* leather_oak = CreateMaterial(0.61, 0.52, 0.25);
	public:
		Trampoline(const PxVec3& dimensions=PxVec3(.5f,.5f,.5f), PxReal stiffness=20.f, PxReal damping=0.01f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(.5,thickness,-2.)),PxVec3(dimensions.x,thickness,dimensions.z));
			top = new Box(PxTransform(PxVec3(.5,dimensions.y+thickness,-2.)),PxVec3(dimensions.x,thickness,dimensions.z));
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
			top->Material(leather_oak);
			bottom->Material(leather_oak);
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
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						
						trigger = true;
						
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
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
#if PX_PHYSICS_VERSION >= 0x304000
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
#endif
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
		
		//materials
		PxMaterial* wood = CreateMaterial(0.3, 1., 0.5);

		PxMaterial* cast_iron = CreateMaterial(1.1, 0.15, 0.1);

		PxMaterial* leather_oak = CreateMaterial(0.61, 0.52, 0.25);

		PxMaterial* rubber_cardboard = CreateMaterial(0.65, 1., 0.5);

		MySimulationEventCallback* my_callback;

		Box* box2;
		Plane* plane;
		Domino* domino;
		NewWindmill* windmill;

		Trampoline* trampoline;
		
		InitActor* initActor;

		Seesaw* seesaw;
		Anvil* anvil;

		Stand* stand1, *standTrigger;

		Ramp* ramp;
		
	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		//MyScene() : Scene() {};
		Box* box;
		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f); // to visualise joint local axes;
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 2.0f); // to visualise the limits of joints.

		}


		//Custom scene initialisation
		virtual void CustomInit()
		{

			SetVisualisation();

			
			
			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			plane = new Plane();
			plane->Color(PxVec3(210.f / 255.f, 210.f / 255.f, 210.f / 255.f));
			Add(plane);
			plane->Material(wood);

			//wood
			GetMaterial(0)->setDynamicFriction(0.2);


			//spawn actors
			initActor = new InitActor(PxTransform(PxVec3(0.f, 10.f, -2.f)));
			initActor->Color(color_palette[0]);
			((PxRigidBody*)initActor->Get())->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
			Add(initActor);
			initActor->Material(leather_oak);

			trampoline = new Trampoline();
			trampoline->AddToScene(this);
			windmill = new NewWindmill();
			windmill->AddToScene(this);

			seesaw = new Seesaw();

			seesaw->AddToScene(this);
			
			stand1 = new Stand(PxTransform(PxVec3(-5, 0., -1)));
			Add(stand1);
			stand1->Color(color_palette[2]);

			ChangeStandOrientation(stand1);

			standTrigger = new Stand(PxTransform(PxVec3(-4.98, 0., -1.01)));
			standTrigger->SetTrigger(true);
			Add(standTrigger);
			standTrigger->Color(color_palette[2]);

			ChangeStandOrientation(standTrigger);

			box = new Box(PxTransform(PxVec3(-0, 1., .75)), PxVec3(0.1), .5);
			Add(box);
			box->Color(color_palette[4]);
			box->Material(rubber_cardboard);
			
			GetMaterial(1)->setDynamicFriction(20.f);
			ramp = new Ramp();
			ramp->AddToScene(this);

			//Create Dominos
			CreateDominos();
	
		}
		
		//int value = 0;
		//Custom udpate function
		virtual void CustomUpdate() 
		{
			CheckTrigger_1(box);
		
		}
		float Deg2Rad(float degrees)
		{
			return degrees * (PxPi / 180);
		}
		/// An example use of key release handling

		void CheckTrigger_1(Box* box)
		{
			
			if (my_callback->trigger == true)
			{
				cout << "Wall hit" << endl;
				((PxRigidBody*)box->Get())->addForce(PxVec3(0, 0, 1) * .15);
				stand1->Color(color_palette[4]);
				standTrigger->Color(color_palette[4]);
			}

		}

		void CreateDominos()
		{
			//placed in a diamond formation
			PxMaterial* PTFE = CreateMaterial(0.04, 0.04, 0);
			int count = 28;
			for (int i = count; i < 50 + count; i++)
			{
					domino = new Domino(PxTransform(PxVec3(0, 0., 0.1*i)));

					domino->Material(PTFE);
					
					Add(domino);

					PxTransform pose = ((PxRigidBody*)domino->Get())->getGlobalPose();
					pose.q = PxQuat(Deg2Rad(90), PxVec3(0., 1., 0));
					((PxRigidBody*)domino->Get())->setGlobalPose(pose);


				

			}
			
			int inc = 2;
			count += 50;
			for (int i = count; i < 50 + count; i++)
			{
				
				domino = new Domino(PxTransform(PxVec3(0.015 *inc, 0.12, 0.1 * i)), PxVec3(0.008f, 0.075f, 0.025f), 0.1);
				inc++;

				domino->Material(PTFE);

				Add(domino);

				PxTransform pose = ((PxRigidBody*)domino->Get())->getGlobalPose();
				pose.q = PxQuat(Deg2Rad(-65), PxVec3(0., 1., 0));
				((PxRigidBody*)domino->Get())->setGlobalPose(pose);
				

			}
			inc = 0;
			
			for (int i = count; i < 50 + count; i++)
			{

				domino = new Domino(PxTransform(PxVec3(-0.015 * inc, 0.12, 0.1 * i)), PxVec3(0.008f, 0.075f, 0.025f));
				inc++;

				domino->Material(PTFE);

				Add(domino);

				PxTransform pose = ((PxRigidBody*)domino->Get())->getGlobalPose();
				pose.q = PxQuat(Deg2Rad(75), PxVec3(0., 1., 0));
				((PxRigidBody*)domino->Get())->setGlobalPose(pose);



			}

			inc = 0;
			count += 50;
			for (int i =  50 + count; i >= count; i--)
			{
				domino = new Domino(PxTransform(PxVec3(0.015 * inc, 0.12, 0.1 * i)), PxVec3(0.008f, 0.075f, 0.025f));
				inc++;

				domino->Material(PTFE);

				Add(domino);

				PxTransform pose = ((PxRigidBody*)domino->Get())->getGlobalPose();
				pose.q = PxQuat(Deg2Rad(75), PxVec3(0., 1., 0));
				((PxRigidBody*)domino->Get())->setGlobalPose(pose);



			}
			inc = 0;
			for (int i = 49 + count; i >= count; i--)
			{
				domino = new Domino(PxTransform(PxVec3(-0.015*inc, 0.12, 0.1 * i)), PxVec3(0.008f, 0.075f, 0.025f));
				inc++;

				domino->Material(PTFE);

				Add(domino);

				PxTransform pose = ((PxRigidBody*)domino->Get())->getGlobalPose();
				pose.q = PxQuat(Deg2Rad(75), PxVec3(0., 1., 0));
				((PxRigidBody*)domino->Get())->setGlobalPose(pose);



			}
			count += 51;
			for (int i = count; i < 20 + count; i++)
			{
				domino = new Domino(PxTransform(PxVec3(0, 0., 0.1 * i)));

				domino->Material(PTFE);

				Add(domino);

				PxTransform pose = ((PxRigidBody*)domino->Get())->getGlobalPose();
				pose.q = PxQuat(Deg2Rad(90), PxVec3(0., 1., 0));
				((PxRigidBody*)domino->Get())->setGlobalPose(pose);




			}
			
			

		}

		void CheckEnd()
		{

		}
		void ChangeStandOrientation(Stand* stand)
		{
			PxTransform pose = ((PxRigidBody*)stand->Get())->getGlobalPose();
			pose.q = PxQuat(Deg2Rad(-45), PxVec3(0., 1., 0));
			((PxRigidBody*)stand->Get())->setGlobalPose(pose);
		}
		void SceneRuiner()
		{
			for (int j = -5; j < 10; j++)
			{
				anvil = new Anvil(PxTransform(PxVec3(0, 20, j)));
				anvil->Material(cast_iron);
				anvil->Color(color_palette[3]);
				Add(anvil);

			}
			
		}
		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}
	};
}
