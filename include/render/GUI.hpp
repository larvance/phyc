#pragma once
#include <imgui.h>

#include "Scene.hpp"
#include "Window.hpp"

struct PhysicsGUI {
    float panelWidth = 300.0f;
    PhyC::World& world;
    PhyCRenderer::Scene& scene;
    const PhyCRenderer::Window& window;
    ImGuiIO* io;
    std::function<void(PhyC::World&)> initFunc;
    float stepTime = 1.0 / 30.0;

    explicit PhysicsGUI(
        PhyC::World& world,
        PhyCRenderer::Scene& scene,
        const PhyCRenderer::Window& window,
        const std::function<void(PhyC::World&)>& initFunc = nullptr
    );

    void render();

    ~PhysicsGUI();
};
