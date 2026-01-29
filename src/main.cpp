#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>

#include "renderer/GUI.hpp"
#include "physics/RigidBody.hpp"
#include "renderer/Scene.hpp"
#include "renderer/Window.hpp"
#include <cfenv>
#include <thread>

#include "renderer/controller/BasicController.hpp"
#include "renderer/light/AmbientLight.hpp"
#include "renderer/light/PointLight.hpp"

using namespace PhyCRenderer;

void init(World& world) {
    world.gravity.z() = approximateGravitationalAcceleration(glm::radians(-35.363262), 584.0);

    for (int i = 0; i < 5; i++) {
        auto& body = world.createRigidBody("Cube");
        body.curState.pos = {0, 0, 5};
        body.curState.angVel = {0.5, 1.0, 0};
        body.surface = Surfaces::cube(1.0);
        body.colliders = {OBB::cube(1.0)};
    }

    auto& ground = world.createRigidBody("Ground");
    ground.isStatic = true;
    ground.curState.pos = {0, 0, -2};
    ground.surface = Surfaces::rectPrism({50.0, 50.0, 2.0}, {0.5, 0.5, 0.5});
    ground.colliders = {OBB::simple(vec3(50.0, 50.0, 2.0))};
}

int main() {
    Window window{1280, 720};

    Scene scene{window};
    scene.camera.position = glm::vec3(0.5f, 3.0f, 0.5f);
    scene.camera.lookAt({0, 0, 0});

    auto controller = std::make_shared<BasicController>();
    scene.addController(controller);

    auto pointLight = std::make_shared<PointLight>();
    pointLight->position = {5.0f, 5.0f, 5.0f};
    scene.addLight(pointLight);
    auto pointLight2 = std::make_shared<PointLight>();
    pointLight2->position = {-5.0f, -5.0f, 5.0f};
    scene.addLight(pointLight2);

    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

    World world{};

    PhysicsGUI gui{world, scene, window, init};

    std::mutex over;
    bool isOver = false;

    std::thread physicsThread([&world, &over, &isOver]() {
        while (true) {
            {
                std::lock_guard lock(over);
                if (isOver) break;
            }
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

    {
        std::lock_guard lock(over);
        isOver = true;
    }

    physicsThread.join();

    return 0;
}
