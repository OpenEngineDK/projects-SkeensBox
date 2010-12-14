// main
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

// 
// No time for Havok! - Its time for bullet(s)
// This engine is like unreal! *TEARS THE ENGINE APART* now its open engine
// Oh lolololol >.<

// OpenEngine stuff
#include <Meta/Config.h>
#include <Logging/Logger.h>
#include <Logging/StreamLogger.h>
#include <Core/Engine.h>

#include <Physics/PhysicsFacade.h>
#include <Physics/RigidBody.h>
#include <Physics/DynamicBody.h>

#include <Geometry/TriangleMesh.h>
#include <Geometry/BoundingSphere.h>
#include <Geometry/AABB.h>

#include <Scene/MeshNode.h>
#include <Scene/SceneNode.h>
#include <Scene/RenderStateNode.h>
#include <Scene/RenderNode.h>
#include <Scene/HeightMapNode.h>
#include <Scene/SunNode.h>
#include <Scene/PointLightNode.h>

// Assimp
#include <Resources/AssimpResource.h>

#include <Utils/SimpleSetup.h>
#include <Utils/MeshUtils.h>
#include <Utils/CameraTool.h>
#include <Utils/ToolChain.h>
#include <Utils/MouseSelection.h>
#include <Utils/ActionCallback.h>
#include <Utils/TerrainUtils.h>
#include <Utils/TerrainTexUtils.h>
#include <Utils/MeshCreator.h>
#include <Utils/BetterMoveHandler.h>
#include <Utils/Bag.h>

#include <Display/Camera.h>
#include <Display/SDLEnvironment.h>

#include <Math/RandomGenerator.h>

#include <Resources/ResourceManager.h>
#include <Resources/SDLImage.h>

using namespace OpenEngine::Logging;
using namespace OpenEngine::Core;
using namespace OpenEngine::Utils;
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Physics;
using namespace OpenEngine::Scene;
using namespace OpenEngine::Display;
using namespace OpenEngine::Devices;
using namespace OpenEngine::Math;
using namespace OpenEngine::Resources;
using namespace OpenEngine::Renderers::OpenGL;


// config
Vector<2,float> hmapsize(33,33);

//Get a random vector within min-->max using the randomGen rg.
Vector<3,float> RandomVector(RandomGenerator* rg,
                             Vector<3,float> min,
                             Vector<3,float> max)
{
    return Vector<3,float>(rg->UniformFloat(min[0],max[0]),
                           rg->UniformFloat(min[1],max[1]),
                           rg->UniformFloat(min[2],max[2]));
}



ISceneNode* CreateRes(char* ResourcePath)
{
    //IModelResourcePtr Res = ResourceManager<IModelResource>::Create("duck/duck.dae");
    //IModelResourcePtr Res = ResourceManager<IModelResource>::Create("sphere/sphere.dae");
    //IModelResourcePtr Res = ResourceManager<IModelResource>::Create("dice/dice.dae");
    //IModelResourcePtr Res = ResourceManager<IModelResource>::Create("collada/collada.dae");
    IModelResourcePtr Res = ResourceManager<IModelResource>::Create(ResourcePath);
	Res->Load();
	
	ISceneNode *ResSceneNode = Res->GetSceneNode();

    Res->Unload();
	return ResSceneNode;
}

    typedef struct
    {
        MeshNode *mesh;
        BoundingSphere *sphere;
        RigidBody* rb;
        DynamicBody* db;
    } Dealloc;

class ActionHandler : public IListener<OpenEngine::Core::ProcessEventArg>
{
    ISceneNode* root;
    PhysicsFacade* phy;
    Vector<3,float> start;
    RandomGenerator* rg;
    Timer dropTimer;
    //Needed for dealloc
    Bag<pair<pair<DynamicBody*, TransformationNode*>, Dealloc* > > array;
public:
    ActionHandler(ISceneNode* root,
                  PhysicsFacade* phy,
                  Vector<3,float> p)
        : root(root), phy(phy), start(p), rg(new RandomGenerator())
    {
        rg->SeedWithTime();
		dropTimer.Start();
    }


    void Handle (OpenEngine::Core::ProcessEventArg arg)
	{
        if((dropTimer.GetElapsedTime().AsInt()) >= 10000)
		{
            dropTimer.Reset();
            DropBox();
        }
    }

    void DropBox()
	{
        const float sphereSize = 2.5f;        
        float mass = rg->UniformFloat(100, 500);
        //logger.info << "Box: " << mass << logger.end;

        MeshNode *mesh = new MeshNode();
        mesh->SetMesh(OpenEngine::Utils::MeshCreator::CreateSphere(sphereSize));
        BoundingSphere *sphere = new BoundingSphere(Vector<3,float>(0,0,0), sphereSize*2);
        RigidBody* rb = new RigidBody(sphere);
        DynamicBody* db = new DynamicBody(rb);
        db->SetPosition(start + RandomVector(rg,
                                             Vector<3,float>(-15,30,-15)*2*sphereSize,
                                             Vector<3,float>(15,60,15)*2*sphereSize));
        db->SetMass(mass);
        phy->AddRigidBody(db);
        
        TransformationNode* duckTrans = rb->GetTransformationNode();
        duckTrans->AddNode(mesh);
        root->AddNode(duckTrans);

        Dealloc *dealloc = new Dealloc();
        dealloc->mesh=mesh;
        dealloc->sphere=sphere;
        dealloc->rb=rb;
        dealloc->db=db;

        array.push_back(make_pair(make_pair(db, duckTrans), dealloc));
        for(unsigned int x=0; x<array.size(); x++)
        {
            Vector<3,float> a = (array[x].first.first)->GetPosition();
            //Element should be deleted if y<0
            if(a[1]<0)
            {
                // Unregister
                phy->RemoveRigidBody(array[x].first.first);
                root->RemoveNode(array[x].first.second);
                // Clean up (delete all pointers in dealloc)
                Dealloc *dealloc = array[x].second;
                // Remove
                array.erase(array.begin() + x);
                // Dealloc
                delete dealloc->db;
                delete dealloc->rb;
                delete dealloc->sphere;
                //delete dealloc->mesh;
                delete dealloc; //delete dealloc itself
                x--;
            }
        }
        logger.info << array.size() << logger.end;
    }
};

int main(int argc, char** argv)
{
    ResourceManager<UCharTexture2D>::AddPlugin(new UCharSDLImagePlugin());
    ResourceManager<IModelResource>::AddPlugin(new AssimpPlugin());
	DirectoryManager::AppendPath("projects/SkeensBox/data/");
 	
	// Setup logging facilities.
    // Create simple setup
    IEnvironment* env = new SDLEnvironment(800,600);

    SimpleSetup* setup = new SimpleSetup("Boxiii", env);

    // Print usage info.
    logger.info << "========= Running OpenEngine Test Project =========" << logger.end;

    // root
    RenderStateNode *rsn = new RenderStateNode();
    setup->GetRenderer().SetBackgroundColor(Vector<4,float>(0,0,1,1));

    // rsn->DisableOption(RenderStateNode::TEXTURE);
    rsn->EnableOption(RenderStateNode::COLOR_MATERIAL);
    rsn->EnableOption(RenderStateNode::LIGHTING);
    ISceneNode *root = rsn;
    setup->SetScene(*root);
    
    //float* startVertex = hmapn->GetVertex(0, 0);
    Vector<3,float> startPoint = Vector<3,float>();

    Vector<3,float> endPoint = Vector<3,float>(hmapsize[0]*16, 1000 , hmapsize[1]*16);
    endPoint += startPoint;

    Vector<3,float> middle = endPoint*0.5 + startPoint;

    // Physics engine
    AABB world(middle, Vector<3,float>(1000,1000,1000));
    PhysicsFacade* phy = new PhysicsFacade(world, Vector<3,float>(0,-9.82,0));

	//Ground
	MeshNode *groundMesh = new MeshNode();
	groundMesh->SetMesh(OpenEngine::Utils::MeshCreator::CreatePlane(800));
    RigidBody* ground = new RigidBody(new AABB(Vector<3,float>(),
                                               Vector<3,float>(400,1,400)));
    ground->SetPosition(middle);
	phy->AddRigidBody(ground);

	TransformationNode* tn3 = ground->GetTransformationNode();
    tn3->AddNode(groundMesh);
    root->AddNode(tn3);

	// DEBUG DRAW
    //root->AddNode(phy->getRenderNode(&(setup->GetRenderer())));

	// Camera
    Camera* cam = setup->GetCamera();
    cam->SetPosition(middle + Vector<3,float>(0,500,-500));
    cam->LookAt(middle);

	// Attach Physics engine
    setup->GetEngine().InitializeEvent().Attach(*phy);
    setup->GetEngine().ProcessEvent().Attach(*phy);
    setup->GetEngine().DeinitializeEvent().Attach(*phy);

    // Get autospawning boxes
    ActionHandler *hdl = new ActionHandler(root, phy, middle);
    setup->GetEngine().ProcessEvent().Attach(*hdl);

	// Duck
    ISceneNode *duckNode = CreateRes("duck/duck.dae");
	RigidBody* duck = new RigidBody(new TriangleMesh(duckNode));
	duck->SetPosition(middle);
	phy->AddRigidBody(duck);
	TransformationNode* duckTrans = duck->GetTransformationNode();
	duckTrans->AddNode(duckNode);
	root->AddNode(duckTrans);

	// Light
    PointLightNode* pl = new PointLightNode();
    TransformationNode* plt = new TransformationNode();
    plt->AddNode(pl);
    plt->SetPosition(Vector<3,float>(0,1000,0));
    root->AddNode(plt);
    
	// Camera Control
    BetterMoveHandler *move = new BetterMoveHandler(*cam,
                                                    setup->GetMouse(),
                                                    true);
    setup->GetEngine().InitializeEvent().Attach(*move);
    setup->GetEngine().ProcessEvent().Attach(*move);

	setup->GetKeyboard().KeyEvent().Attach(*move);   
	setup->GetMouse().MouseButtonEvent().Attach(*move);
	setup->GetMouse().MouseMovedEvent().Attach(*move);
	
	// Show FPS
    setup->ShowFPS();

    // Start the engine.
    setup->GetEngine().Start();

    // Return when the engine stops.
    return EXIT_SUCCESS;
}
