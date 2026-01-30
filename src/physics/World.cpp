#include "physics/World.hpp"

#include "physics/FluidProperties.hpp"

using namespace PhyC;

double PhyC::approximateGravitationalAcceleration(double lat_rad, double alt_m) {
    double g_ecuador = 9.7803253359;
    double k = 0.00193185265241;
    double e2 = 0.00669437999013; // first eccentricity squared

    double sin_lat_sq = sin(lat_rad);
    sin_lat_sq *= sin_lat_sq;

    double g_phi = g_ecuador * (1 + k * sin_lat_sq) / sqrt(1 - e2 * sin_lat_sq);

    return -(g_phi - (3.086e-6) * alt_m);
}

FluidProperties World::sampleAirProperties(vec3 position) const {
    return FluidProperties{
        .vel = vec3{0.0, 0.0, 0.0},
        .density = 1.225,
        .pressure = 101325.0,
        .viscosity = 1.81e-5
    };
}

RigidBody& World::createRigidBody(const std::string& name) {
    std::lock_guard lock(worldMutex);
    auto body = std::make_unique<RigidBody>();
    body->name = name;
    body->world = this;
    bodies.push_back(std::move(body));
    return *bodies.back();
}

RigidBody& World::addDebugVector(const vec3& pos, const vec3& vec) {
    std::lock_guard lock(worldMutex);
    auto& body = createRigidBody("Vector");
    body.isStatic = true;
    body.curState.pos = pos;
    body.surface = Surfaces::arrow(vec, {1, 0, 0}, 0.05, 0.1, 0.25);
    return body;
}

Spring& World::createSpring(RigidBody& body, const vec3& targetPoint) {
    std::lock_guard lock(worldMutex);

    auto spring = std::make_unique<Spring>();
    spring->bodyA = &body;
    spring->anchorA = vec3(0, 0, 0);

    spring->bodyB = nullptr;
    spring->worldTargetPos = targetPoint;

    double dist = (targetPoint - body.curState.pos).norm();
    spring->restLength = dist;

    springs.push_back(std::move(spring));
    return *springs.back();
}

Spring& World::createSpring(RigidBody& body, RigidBody& targetBody) {
    std::lock_guard lock(worldMutex);

    auto spring = std::make_unique<Spring>();
    spring->bodyA = &body;
    spring->anchorA = vec3(0, 0, 0);

    spring->bodyB = &targetBody;
    spring->anchorB = vec3(0, 0, 0);

    double dist = (targetBody.curState.pos - body.curState.pos).norm();
    spring->restLength = dist;

    springs.push_back(std::move(spring));
    return *springs.back();
}

bool World::hasRigidBody(const RigidBody& body) const {
    std::lock_guard lock(worldMutex);
    for (const auto& b : bodies) {
        if (b->id == body.id) {
            return true;
        }
    }
    return false;
}

void World::removeRigidBody(RigidBody& body) {
    std::lock_guard lock(worldMutex);
    std::erase_if(bodies, [&body](const std::unique_ptr<RigidBody>& b) {
        return b.get() == &body;
    });
}

bool World::raycast(RayHit& hit, double mouseX, double mouseY, double screenW, double screenH,
                    const mat4& view, const mat4& projection) const {
    std::lock_guard lock(worldMutex);

    vec4 viewport(0.0f, 0.0f, screenW, screenH);

    float flippedY = screenH - mouseY;

    vec3 winCoords(mouseX, flippedY, 0.0f);
    vec3 winCoordsFar(mouseX, flippedY, 1.0f);

    vec3 rayStartGLM = unProject(winCoords, view, projection, viewport);
    vec3 rayEndGLM = unProject(winCoordsFar, view, projection, viewport);

    vec3 rayOrigin{rayStartGLM.x(), rayStartGLM.y(), rayStartGLM.z()};
    vec3 rayDirRaw{rayEndGLM.x() - rayStartGLM.x(), rayEndGLM.y() - rayStartGLM.y(), rayEndGLM.z() - rayStartGLM.z()};

    vec3 rayDir = rayDirRaw.normalized();

    RigidBody* closestBody = nullptr;
    double minDistance = std::numeric_limits<double>::max();
    vec3 closestPoint;

    for (auto& body : bodies) {
        double radius = 1.0;

        vec3 oc = body->curState.pos - rayOrigin;
        double t_proj = oc.dot(rayDir);

        if (t_proj < 0) continue;

        double distSq = oc.dot(oc) - (t_proj * t_proj);
        double radiusSq = radius * radius;

        if (distSq <= radiusSq) {
            double distToSurface = t_proj - std::sqrt(radiusSq - distSq);

            if (distToSurface < minDistance && distToSurface > 0) {
                minDistance = distToSurface;
                closestBody = body.get();
                closestPoint = rayOrigin + (rayDir * distToSurface);
            }
        }
    }

    if (closestBody) {
        hit.body = closestBody;
        hit.point = closestPoint;
        hit.distance = minDistance;
        return true;
    }

    return false;
}

void World::update() {
    std::lock_guard lock(worldMutex);
    double nowSeconds = getTime_s();
    if (lastUpdate == 0.0) {
        lastUpdate = nowSeconds;
        return;
    }
    double dt = (nowSeconds - lastUpdate) * speed;
    lastUpdate = nowSeconds;
    step(dt);
}

void World::step(double dt) {
    std::lock_guard lock(worldMutex);
    accumulatedTime += dt;
    while (accumulatedTime >= timeStep) {
        stepFixed();
        accumulatedTime -= timeStep;
    }
}

void World::stepFixed() {
    std::lock_guard lock(worldMutex);
    pGravityLen = gravity.norm();

    for (auto& body : bodies) {
        body->beforeUpdate();
    }

    for (auto& spring : springs) {
        spring->applyForce(timeStep);
    }

    for (auto& body : bodies) {
        body->update(timeStep);
    }

    for (int i = 0; i < jointIterations; ++i) {
        for (auto& joint : joints) {
            joint->solve(timeStep);
        }
    }

    for (auto& body : bodies) {
        body->afterUpdate(timeStep);
    }
}

void World::reset() {
    std::lock_guard lock(worldMutex);
    for (auto& body : bodies) body->world = nullptr;
    bodies.clear();
    joints.clear();
    resetTimer();
}

void World::resetTimer() {
    std::lock_guard lock(worldMutex);
    lastUpdate = 0.0;
    accumulatedTime = 0.0;
}
