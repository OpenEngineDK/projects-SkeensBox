// main
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

// OpenEngine stuff
#include <Meta/Config.h>
#include <Logging/Logger.h>
#include <Logging/StreamLogger.h>
#include <Core/Engine.h>


#include <Physics/PhysicsFacade.h>
#include <Physics/RigidBody.h>
#include <Physics/DynamicBody.h>

#include <Geometry/AABB.h>
#include <Geometry/HeightfieldTerrainShape.h>

#include <Scene/MeshNode.h>
#include <Scene/SceneNode.h>
#include <Scene/RenderStateNode.h>
#include <Scene/RenderNode.h>
#include <Scene/HeightMapNode.h>
#include <Scene/SunNode.h>
#include <Scene/PointLightNode.h>

#include <Utils/SimpleSetup.h>
#include <Utils/PerlinNoise.h>
#include <Utils/MeshUtils.h>
#include <Utils/CameraTool.h>
#include <Utils/ToolChain.h>
#include <Utils/MouseSelection.h>
#include <Utils/ActionCallback.h>
#include <Utils/TerrainUtils.h>
#include <Utils/TerrainTexUtils.h>

#include <Devices/KeyboardActionMapper.h>

#include <Display/Camera.h>
#include <Display/SDLEnvironment.h>

#include <Math/RandomGenerator.h>

#include <Renderers/OpenGL/TerrainRenderingView.h>

#include <Resources/ResourceManager.h>
#include <Resources/SDLImage.h>

// Game factory
//#include "GameFactory.h"

// name spaces that we will be using.
// this combined with the above imports is almost the same as
// fx. import OpenEngine.Logging.*; in Java.
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
//Vector<2,float> hmapsize(64,64);
//Vector<2,float> hmapsize(193,193);

class KeyArg {
public:
    ButtonEvent type;
    KeyArg(ButtonEvent t) : type(t) {}
};

class KeyActionMap : public KeyboardActionMapper<KeyArg> {
    KeyArg toAction(KeyboardEventArg arg) { return KeyArg(arg.type); }
};

Vector<3,float> RandomVector(RandomGenerator* rg,
                             Vector<3,float> min,
                             Vector<3,float> max) {
    return Vector<3,float>(rg->UniformFloat(min[0],max[0]),
                           rg->UniformFloat(min[1],max[1]),
                           rg->UniformFloat(min[2],max[2]));
}

class ActionHandler : public IListener<ProcessEventArg> {
    
    ISceneNode* root;
    PhysicsFacade* phy;
    Vector<3,float> start;
    RandomGenerator* rg;
    Timer dropTimer;
public:
    ActionHandler(ISceneNode* root,
                      PhysicsFacade* phy,
                      Vector<3,float> p)
        : root(root)
        , phy(phy)
        , start(p)
        , rg(new RandomGenerator())
    {
        rg->SeedWithTime();
    }


    void Handle (ProcessEventArg arg) {
        if (dropTimer.GetElapsedIntervals(300000) >= 1) {
            dropTimer.Reset();
            DropBox();
        }
    }

    void ToggleAuto(KeyArg arg) {
        logger.info << "toggle auto" << logger.end;
        if (dropTimer.IsRunning())
            dropTimer.Stop();
        else
            dropTimer.Start();
    }

    void NewBox(KeyArg arg) {
        DropBox();
    }
    void DropBox() {
        const float boxSize = 50;        
        float mass = rg->UniformFloat(1.0, 1000.0);
        logger.info << "Box: " << mass << logger.end;
        RigidBody* body = new RigidBody(new AABB(Vector<3,float>(),
                                                 Vector<3,float>(boxSize)));
        DynamicBody* db = new DynamicBody(body);
        db->SetPosition(start + RandomVector(rg,
                                             Vector<3,float>(-2,20,-2)*boxSize,
                                             Vector<3,float>(2,40,2)*boxSize));
        db->SetMass(mass);
        
        phy->AddRigidBody(db);


        MeshNode *mn = new MeshNode();
        mn->SetMesh(CreateCube(boxSize*2,RandomVector(rg,
                                               Vector<3,float>(),
                                               Vector<3,float>(1)),1));

        TransformationNode* tn = body->GetTransformationNode();
        tn->AddNode(mn);

        root->AddNode(tn);

    }
};

HeightMapNode* SetupTerrain(SimpleSetup*, 
                            PhysicsFacade* );

/**
 * Main method for the first quarter project of CGD.
 * Corresponds to the
 *   public static void main(String args[])
 * method in Java.
 */
int main(int argc, char** argv) {
    ResourceManager<UCharTexture2D>::AddPlugin(new UCharSDLImagePlugin());
    DirectoryManager::AppendPath("projects/Boxes/data/");
 // Setup logging facilities.
    // Create simple setup
    IEnvironment* env = new SDLEnvironment(800,600);

    Viewport* vp = new Viewport(env->GetFrame());
    IRenderingView* rv = new TerrainRenderingView(*vp);

    SimpleSetup* setup = new SimpleSetup("Boxiii",vp,env,rv);

    // Print usage info.
    logger.info << "========= Running OpenEngine Test Project =========" << logger.end;

    KeyActionMap map;
    map.listenTo(setup->GetKeyboard());

    // root
    RenderStateNode *rsn = new RenderStateNode();

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
    AABB world(middle,
               Vector<3,float>(1000,1000,1000));

    PhysicsFacade* phy = new PhysicsFacade(world, Vector<3,float>(0,-9.82,0));
    HeightMapNode* hmapn = SetupTerrain(setup,phy);

    MeshNode *mn = new MeshNode();
    mn->SetMesh(CreateCube(10,Vector<3,float>(1,0,0),1));

    MeshNode *mn2 = new MeshNode();
    mn2->SetMesh(CreateCube(10,Vector<3,float>(1,0,0),1));

    RigidBody* ground = new RigidBody(new AABB(Vector<3,float>(),
                                               Vector<3,float>(100,1,100)));
    ground->SetPosition(middle);

    RigidBody* b1 = new RigidBody(new AABB(Vector<3,float>(),
                                           Vector<3,float>(5)));

    RigidBody* b2 = new RigidBody(new AABB(Vector<3,float>(),
                                           Vector<3,float>(5)));

    

    DynamicBody* db = new DynamicBody(b1);
    DynamicBody* db2 = new DynamicBody(b2);
    db->SetPosition(middle + Vector<3,float>(0,10,0));
    db2->SetPosition(middle + Vector<3,float>(5,40,5));
    phy->AddRigidBody(db);
    phy->AddRigidBody(db2);
    //phy->AddRigidBody(ground);

    TransformationNode* tn = b1->GetTransformationNode();
    TransformationNode* tn2 = b2->GetTransformationNode();

    tn->AddNode(mn);
    tn2->AddNode(mn2);

    setup->GetRenderer().SetBackgroundColor(Vector<4,float>(0,0,1,1));


    root->AddNode(tn);
    root->AddNode(tn2);
    root->AddNode(phy->getRenderNode(&(setup->GetRenderer())));


    Camera* cam = setup->GetCamera();
    cam->SetPosition(middle + Vector<3,float>(0,100,-100));
    cam->LookAt(middle);

    setup->GetEngine().InitializeEvent().Attach(*phy);
    setup->GetEngine().ProcessEvent().Attach(*phy);
    setup->GetEngine().DeinitializeEvent().Attach(*phy);

    // keyh
    ActionHandler *hdl = new ActionHandler(root, phy, middle);
    map.onKeyPress(KEY_n,
                   *(new ActionCallback<ActionHandler,KeyArg>(hdl, &ActionHandler::NewBox)));
    map.onKeyPress(KEY_a,
                   *(new ActionCallback<ActionHandler,KeyArg>(hdl, &ActionHandler::ToggleAuto)));
    setup->GetEngine().ProcessEvent().Attach(*hdl);

    // terrain

    // Light
    PointLightNode* pl = new PointLightNode();
    TransformationNode* plt = new TransformationNode();
    plt->AddNode(pl);
    plt->SetPosition(Vector<3,float>(0,1000,0));
    root->AddNode(plt);
    // mouse tools
    MouseSelection* ms = new MouseSelection(setup->GetFrame(), setup->GetMouse(), NULL);
    CameraTool* ct   = new CameraTool();
    ToolChain* tc    = new ToolChain();
    tc->PushBackTool(ct);

    setup->GetRenderer().PostProcessEvent().Attach(*ms);
    setup->GetMouse().MouseMovedEvent().Attach(*ms);
    setup->GetMouse().MouseButtonEvent().Attach(*ms);
    setup->GetKeyboard().KeyEvent().Attach(*ms);

    ms->BindTool(&(setup->GetRenderer().GetViewport()), tc);


    setup->ShowFPS();



    // Start the engine.
    setup->GetEngine().Start();

    // Return when the engine stops.
    return EXIT_SUCCESS;
}

HeightMapNode* SetupTerrain(SimpleSetup* setup, PhysicsFacade* phy) {
    //FloatTexture2DPtr map = FloatTexture2DPtr(new FloatTexture2D(193, 193, 1));    
    FloatTexture2DPtr map = FloatTexture2DPtr(new FloatTexture2D(hmapsize[0], hmapsize[1], 1));
    // //UCharTexture2DPtr map1 = ResourceManager<UCharTexture2D>::Create("Heightmap32.tga");
    // //FloatTexture2DPtr map = ConvertTex(map1);
    Empty(map);
    // map = CreateSmoothTerrain(map, 50, 40, 200);


    map = CreateSmoothTerrain(map, 1, 160, 300);
    map = CreateSmoothTerrain(map, 25, 20, 60);
    map = CreateSmoothTerrain(map, 125, 5, 40);
    map = CreateSmoothTerrain(map, 250, 3, -6);
    map = CreateSmoothTerrain(map, 625, 2, 3);
    map = MakePlateau(map, 700, 30);


    map = PerlinNoise::Generate(hmapsize[0],
                                hmapsize[1],
                                512, 0.5, 1.0, 10, 1/4, 0);

    PerlinNoise::Normalize(map, 0, 1024);

    float widthScale = 16.0;

    Vector<3, float> origo = Vector<3, float>(map->GetHeight() * widthScale / 2, 
                                              0, 
                                              map->GetWidth() * widthScale / 2);
    Vector<3, float> sunDir = Vector<3, float>(1448, 2048, 1448);

    HeightMapNode* node = new HeightMapNode(map);
    node->SetWidthScale(widthScale);
    //node->SetSun(sun);
    setup->GetRenderer().InitializeEvent().Attach(*node);
    setup->GetEngine().ProcessEvent().Attach(*node);
    
    setup->GetScene()->AddNode(node);
    node->Load();

    logger.info << hmapsize << logger.end;
    logger.info << node->GetVerticeWidth() << logger.end;
    logger.info << node->GetVerticeDepth() << logger.end;
        
    HeightfieldTerrainShape* hground = new HeightfieldTerrainShape(node->GetVertexBuffer(),
                                                                   node->GetVerticeWidth(),
                                                                   node->GetVerticeDepth(),
                                                                   1000,
                                                                   16,
                                                                   1,
                                                                   true,
                                                                   true);
    RigidBody* gnd = new RigidBody(hground);
    gnd->SetRotation(Quaternion<float>(0,-PI/2,0));
     gnd->SetPosition(Vector<3,float>(16*hmapsize[0]/2 - 16/2,
                                      1000/2,
                                      16*hmapsize[1]/2 - 16/2
                                      ));

    phy->AddRigidBody(gnd);

    return node;
}
