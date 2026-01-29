#version 330 core
#define MAX_LIGHTS 8/////////////////
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;
in vec3 Color;

struct Light {
// type:
// 0 - Directional
// 1 - Spot
// 2 - Hemisphere
// 3 - Point
    int type;
// struct DirectionalLight {
    vec3 direction;
    vec3 color;
    float intensity;

// struct SpotLight {
    vec3 position;
// vec3 direction;
// vec3 color;
// float intensity;
    float distance;
    float decay;
    float coneCos;// cos(angle)
    float penumbraCos;// cos(angle * (1 - penumbra))

// struct HemisphereLight {
// vec3 direction;
    vec3 skyColor;
    vec3 groundColor;
// float intensity;

// struct PointLight {
// vec3 position;
// vec3 color;
// float intensity;
// float distance;
// float decay;
};

uniform Light lights[MAX_LIGHTS];
uniform int numLights;

uniform vec3 ambientLightColor;
uniform vec3 viewPos;
const float materialShininess = 32.0;

vec3 calcBlinnPhong(vec3 lightDir, vec3 lightColor, float intensity, vec3 normal, vec3 viewDir, vec3 baseColor) {
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor * intensity;

    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), materialShininess);
    vec3 specular = vec3(0.5) * spec * lightColor * intensity;

    return (diffuse + specular) * baseColor;
}

float getDistanceAtten(float lightDistance, float cutoffDistance, float decayExponent) {
    if (cutoffDistance > 0.0 && decayExponent > 0.0) {
        float d = max(lightDistance, 0.001);
        float falloff = 1.0 / pow(d, decayExponent);

        float distSq = d * d;
        float rangeSq = cutoffDistance * cutoffDistance;
        float window = max(0.0, 1.0 - pow(distSq / rangeSq, 2.0));
        window = window * window;

        return falloff * window;
    }
    return 1.0;
}

void main() {
    vec3 norm = normalize(Normal);
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 result = vec3(0.0);

    result += ambientLightColor * Color;


    for (int i = 0; i < numLights; i++) {
        switch (lights[i].type) {
            case 0:// Directional
            {
                vec3 lightDir = normalize(-lights[i].direction);
                result += calcBlinnPhong(lightDir, lights[i].color, lights[i].intensity, norm, viewDir, Color);
                break;
            }
            case 1:// Spot
            {
                vec3 lightDir = normalize(lights[i].position - FragPos);
                float distance = length(lights[i].position - FragPos);

                float theta = dot(lightDir, normalize(-lights[i].direction));

                float epsilon = max(lights[i].penumbraCos - lights[i].coneCos, 0.0001);// prevent division by zero
                float intensity = clamp((theta - lights[i].coneCos) / epsilon, 0.0, 1.0);

                if (intensity > 0.0) {
                    float attenuation = getDistanceAtten(distance, lights[i].distance, lights[i].decay);
                    vec3 effect = calcBlinnPhong(lightDir, lights[i].color, lights[i].intensity, norm, viewDir, Color);
                    result += effect * attenuation * intensity;
                }
                break;
            }
            case 2:// Hemisphere
            {
                float dotNV = dot(norm, normalize(lights[i].direction));
                float mixFactor = 0.5 * dotNV + 0.5;
                vec3 ambientHemi = mix(lights[i].groundColor, lights[i].skyColor, mixFactor);
                result += ambientHemi * lights[i].intensity * Color;
                break;
            }
            case 3:// Point
            {
                vec3 lightDir = normalize(lights[i].position - FragPos);
                float distance = length(lights[i].position - FragPos);

                float attenuation = getDistanceAtten(distance, lights[i].distance, lights[i].decay);

                vec3 effect = calcBlinnPhong(lightDir, lights[i].color, lights[i].intensity, norm, viewDir, Color);

                result += effect * attenuation;
                break;
            }
        }
    }

    FragColor = vec4(result, 1.0);
}