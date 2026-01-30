#include "render/GUI.hpp"

#include <filesystem>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

PhysicsGUI::PhysicsGUI(
    PhyC::World& world,
    PhyCRenderer::Scene& scene,
    const PhyCRenderer::Window& window,
    const std::function<void(PhyC::World&)>& initFunc
) : world(world), scene(scene), window(window), initFunc(initFunc) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = &ImGui::GetIO();
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImFontConfig config;
    config.SizePixels = 18.0f;
    constexpr auto fontPath = "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf";
    if (std::filesystem::exists(fontPath)) {
        io->Fonts->AddFontFromFileTTF(fontPath, 18.0f);
    } else {
        std::cerr << "Warning: Font not found, using default." << std::endl;
        io->Fonts->AddFontDefault(&config);
    }

    ImGui_ImplGlfw_InitForOpenGL(window.handle, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    if (initFunc) initFunc(world);
}

void PhysicsGUI::render() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(window.width - panelWidth, 0));
    ImGui::SetNextWindowSize(ImVec2(panelWidth, window.height));
    ImGui::Begin("Control Panel", nullptr,
                 ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoTitleBar);

    ImGui::Text("PhyC Control Panel");
    ImGui::Separator();
    ImGui::Text("FPS: %.1f (%.3f ms)", io->Framerate, 1000.0f / io->Framerate);
    ImGui::Separator();

    if (ImGui::CollapsingHeader("Scene Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool stepping = ImGui::Button("Step");
        ImGui::SameLine();
        ImGui::DragFloat("##StepTime", &stepTime, world.timeStep, world.timeStep, 1.0f, "Step Time: %.3f s");
        if (stepping) {
            world.resetTimer();
            world.step(stepTime);
        }
        if (initFunc && ImGui::Button("Reset Simulation")) {
            world.reset();
            initFunc(world);
        }
        double minVal = 0.0;
        double maxVal = 5.0;
        if (ImGui::Button("    ##Normal Speed")) {
            world.speed = 1.0;
        }
        ImGui::SameLine();
        ImGui::SliderScalar("Speedup", ImGuiDataType_Double, &world.speed, &minVal, &maxVal);

        ImGui::ColorEdit3("Background", &scene.backgroundColor[0]);

        auto gravity = PhyC::toGlm(world.gravity);
        if (ImGui::DragFloat3("Gravity", &gravity[0], 0.1f)) {
            world.gravity = PhyC::toEigen(gravity);
        }
    }

    if (ImGui::CollapsingHeader("Camera")) {
        ImGui::DragFloat3("Position##Camera", glm::value_ptr(scene.camera.position), 0.1f);
        // ImGui::DragFloat3("Front##Camera", glm::value_ptr(scene.camera.front), 0.1f);
        // ImGui::DragFloat3("Up##Camera", glm::value_ptr(scene.camera.up), 0.1f);
    }

    for (int i = 0; i < world.bodies.size(); ++i) {
        auto& body = *world.bodies[i];
        ImGui::PushID(body.id);
        if (ImGui::CollapsingHeader(body.name.c_str())) {
            glm::vec3 posVals(body.curState.pos.x(), body.curState.pos.y(), body.curState.pos.z());
            if (ImGui::DragFloat3("Position", glm::value_ptr(posVals), 0.1f)) {
                body.curState.pos.x() = posVals.x;
                body.curState.pos.y() = posVals.y;
                body.curState.pos.z() = posVals.z;
            }

            glm::vec3 velVals(body.curState.vel.x(), body.curState.vel.y(), body.curState.vel.z());
            if (ImGui::DragFloat3("Lin. Velocity", glm::value_ptr(velVals), 0.1f)) {
                body.curState.vel.x() = velVals.x;
                body.curState.vel.y() = velVals.y;
                body.curState.vel.z() = velVals.z;
            }

            glm::vec3 angVelVals(body.curState.angVel.x(), body.curState.angVel.y(), body.curState.angVel.z());
            if (ImGui::DragFloat3("Ang. Velocity", glm::value_ptr(angVelVals), 0.1f)) {
                body.curState.angVel.x() = angVelVals.x;
                body.curState.angVel.y() = angVelVals.y;
                body.curState.angVel.z() = angVelVals.z;
            }

            if (ImGui::Button("Reset Position")) {
                body.curState.pos = {0, 0, 5};
                body.curState.rot = PhyC::quaternion::Identity();
                body.curState.vel = {0, 0, 0};
                body.curState.angVel = {0.5, 1, 0};
            }
        }
        ImGui::PopID();
    }

    for (int i = 0; i < scene.lights.size(); ++i) {
        auto& light = scene.lights[i];
        auto typeName = PhyCRenderer::LightTypeNames[static_cast<int>(light->type)];
        ImGui::PushID(light->id);
        if (ImGui::CollapsingHeader((typeName + " Light").c_str())) {
            light->gui();
        }
        ImGui::PopID();
    }

    panelWidth = ImGui::GetWindowWidth();
    ImGui::End();

    scene.viewport = {0, 0, window.width - panelWidth, window.height};
    scene.render(world);

    ImGui::Render();
    glViewport(0, 0, window.width, window.height);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

PhysicsGUI::~PhysicsGUI() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}
