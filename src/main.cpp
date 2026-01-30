#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>

#include "render/GUI.hpp"
#include "physics/RigidBody.hpp"
#include "render/Scene.hpp"
#include "render/Window.hpp"
#include <cfenv>
#include <thread>

#include "physics/joints/FixedJoint.hpp"
#include "physics/joints/RevoluteJoint.hpp"
#include "physics/joints/PrismaticJoint.hpp"
#include "physics/joints/UniversalJoint.hpp"
#include "render/controller/BasicController.hpp"
#include "render/light/AmbientLight.hpp"
#include "render/light/PointLight.hpp"

using namespace PhyCRenderer;

void init(World& world) {
    std::lock_guard lock(world.worldMutex);
    world.gravity.z() = approximateGravitationalAcceleration(glm::radians(-35.363262), 584.0);

    /*for (int i = 0; i < 1; i++) {
        auto& body = world.createRigidBody("Cube");
        body.curState.pos = {0, 0, 5.0 + i * 2.0};
        body.curState.angVel = {0.5, 1.0, 0};
        body.surface = Surfaces::cube(1.0);
        body.colliders = {OBB::cube(1.0)};
    }*/

    auto& ground = world.createRigidBody("Ground");
    ground.isStatic = true;
    ground.curState.pos = {0, 0, -2};
    ground.surface = Surfaces::rectPrism({50.0, 50.0, 2.0}, {0.5, 0.5, 0.5});
    ground.colliders = {OBB::simple(vec3(50.0, 50.0, 2.0))};

    auto& bodyA = world.createRigidBody("BodyA");
    bodyA.curState.pos = {0, 0, 5};
    bodyA.surface = Surfaces::cube(1.0);
    bodyA.colliders = {OBB::cube(1.0)};

    auto& bodyB = world.createRigidBody("BodyB");
    bodyB.curState.pos = {0, 0, 6};
    bodyB.surface = Surfaces::cube(1.0);
    bodyB.colliders = {OBB::cube(1.0)};

    /*auto& bodyC = world.createRigidBody("BodyC");
    bodyC.curState.pos = {1, 2, 5};
    bodyC.surface = Surfaces::cube(1.0);
    bodyC.colliders = {OBB::cube(1.0)};*/

    /*auto& fixedJoint = bodyA.createJoint<FixedJoint>(bodyB);
    fixedJoint.parentAnchor = vec3(0, 0, 0.5);
    fixedJoint.childAnchor = vec3(0, 0, -0.5);*/

    auto& spring = world.createSpring(bodyA, bodyB);
    spring.stiffness = 20.0;
    spring.damping = 5.0;
    spring.restLength = 4.0;

    /*auto& revJoint = world.addJoint<RevoluteJoint>();
    revJoint.parent = &bodyA;
    revJoint.child = &bodyB;
    revJoint.parentAnchor = vec3(0.5, 0, 0);
    revJoint.childAnchor = vec3(-0.5, 0, 0);
    revJoint.axis = vec3(0, 0, 1);
    revJoint.enableLimits = true;
    revJoint.minAngle = -glm::pi<double>() / 4;
    revJoint.maxAngle = glm::pi<double>() / 4;

    auto& prismJoint = world.addJoint<PrismaticJoint>();
    prismJoint.parent = &bodyB;
    prismJoint.child = &bodyC;
    prismJoint.parentAnchor = vec3(0, 0.5, 0);
    prismJoint.childAnchor = vec3(0, -0.5, 0);
    prismJoint.axis = vec3(0, 1, 0);
    prismJoint.enableLimits = true;
    prismJoint.minPosition = -1.0;
    prismJoint.maxPosition = 1.0;

    auto& uniJoint = world.addJoint<UniversalJoint>();
    uniJoint.parent = &bodyC;
    uniJoint.child = &bodyA;
    uniJoint.parentAnchor = vec3(-0.5, 0, 0);
    uniJoint.childAnchor = vec3(0.5, 0, 0);
    uniJoint.axis1 = vec3(1, 0, 0);
    uniJoint.axis2 = vec3(0, 1, 0);*/
}

int main() {
    Window window{1280, 720};

    Scene scene{window};
    scene.camera.position = glm::vec3(0.5f, 3.0f, 0.5f);
    scene.camera.lookAt({0, 0, 0});

    auto pointLight = std::make_shared<PointLight>();
    pointLight->position = {5.0f, 5.0f, 5.0f};
    scene.addLight(pointLight);
    auto pointLight2 = std::make_shared<PointLight>();
    pointLight2->position = {-5.0f, -5.0f, 5.0f};
    scene.addLight(pointLight2);

    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

    World world{};

    auto controller = std::make_shared<BasicController>();
    scene.addController(controller);

    PhysicsGUI gui{world, scene, window, init};

    std::atomic isOver = false;

    std::thread physicsThread([&world, &isOver]() {
        while (!isOver) {
            world.update();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while (!window.shouldClose()) {
        //scene.camera.lookAt(toGlm(cube->pos));
        scene.poll();
        gui.render();
        window.swapBuffers();
    }

    isOver = true;

    physicsThread.join();

    return 0;
}
