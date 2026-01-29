#pragma once

namespace PhyCRenderer {
    struct Scene;

    struct Controller {
        virtual ~Controller() = default;

        virtual void onMouseMove(Scene& scene) {
        }

        virtual void onMouseDown(Scene& scene, int button, int mods) {
        }

        virtual void onMouseUp(Scene& scene, int button, int mods) {
        }

        virtual void onMouseClick(Scene& scene, int button) {
        }

        virtual void onScroll(Scene& scene, double dx, double dy) {
        }

        virtual void onResize(Scene& scene) {
        }

        virtual void onFocusIn(Scene& scene) {
        }

        virtual void onFocusOut(Scene& scene) {
        }

        virtual void onKeyDown(Scene& scene, int key, int scancode, int mods) {
        }

        virtual void onKeyPress(Scene& scene, int key, int scancode, int mods) {
        }

        virtual void onKeyUp(Scene& scene, int key, int scancode, int mods) {
        }

        virtual void onUpdate(Scene& scene) {
        }
    };
}
