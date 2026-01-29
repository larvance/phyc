#include "World.hpp"

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

AirProperties World::sampleAirProperties(vec3 position) const {
    return AirProperties{
        .vel = vec3{0.0, 0.0, 0.0},
        .density = 1.225,
        .pressure = 101325.0,
        .viscosity = 1.81e-5
    };
}
