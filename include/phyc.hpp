#pragma once
#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <glm/glm.hpp>

namespace PhyC {
    static constexpr auto PHYC_VERSION = "0.0.2";
    static constexpr auto PHYC_VERSION_MAJOR = 0;
    static constexpr auto PHYC_VERSION_MINOR = 0;
    static constexpr auto PHYC_VERSION_PATCH = 2;

    using vec2 = Eigen::Vector2d;
    using vec3 = Eigen::Vector3d;
    using vec4 = Eigen::Vector4d;
    using mat2 = Eigen::Matrix2d;
    using mat3 = Eigen::Matrix3d;
    using mat4 = Eigen::Matrix4d;
    using quaternion = Eigen::Quaterniond;

    template <typename Char, size_t N>
    struct FixedString {
        Char buf[N]{};
        // ReSharper disable once CppNonExplicitConvertingConstructor
        constexpr FixedString(const Char (&str)[N]) {
            for (size_t i = 0; i < N; ++i) buf[i] = str[i];
        }
    };

    template <typename StreamT, size_t N, typename... Args>
    static constexpr auto& __print_impl(StreamT& Stream, const FixedString<typename StreamT::char_type, N>& Sep,
                                        Args&&... args) {
        size_t i = 0;
        using CharT = StreamT::char_type;
        const CharT empty_str[] = {0};
        return ((
            Stream
            << std::forward<Args>(args)
            << (++i == sizeof...(Args) ? empty_str : static_cast<const StreamT::char_type*>(Sep.buf))
        ), ...);
    }

    template <auto& Stream = std::cout, FixedString Sep = "", typename... Args>
    static constexpr auto& print(Args&&... args) {
        return __print_impl(Stream, Sep, std::forward<Args>(args)...);
    }

    template <auto& Stream = std::cout, FixedString Sep = "", typename... Args>
    static constexpr auto& println(Args&&... args) {
        return print<Stream, Sep>(std::forward<Args>(args)...) << std::endl;
    }

    template <auto& Stream = std::cout, FixedString Sep = "", typename... Args>
    static constexpr void info(Args&&... args) {
        return println<Stream, Sep>(std::forward<Args>(args)...);
    }

    template <auto& Stream = std::cerr, FixedString Sep = "", typename... Args>
    static constexpr void warn(Args&&... args) {
        return println<Stream, Sep>(std::forward<Args>(args)...);
    }

    template <auto& Stream = std::cerr, FixedString Sep = "", typename... Args>
    static constexpr void error(Args&&... args) {
        return println<Stream, Sep>(std::forward<Args>(args)...);
    }

    template <auto& Stream = std::cerr, FixedString Sep = "", typename... Args>
    static constexpr void crash(Args&&... args) {
        println<Stream, Sep>(std::forward<Args>(args)...);
        throw std::runtime_error("PhyC Error");
    }

    static std::string readFile(const std::string& path) {
        std::ifstream file;
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        try {
            file.open(path);
            std::stringstream stream;
            stream << file.rdbuf();
            file.close();
            return stream.str();
        } catch (std::ifstream::failure& e) {
            crash("Failed to read the file '" + path + "': " + e.what());
            return "";
        }
    }

    inline constexpr glm::vec3 toGlm(const vec3& v) {
        return {v.x(), v.y(), v.z()};
    }

    inline constexpr glm::mat3 toGlm(const mat3& m) {
        return {
            m(0, 0), m(1, 0), m(2, 0),
            m(0, 1), m(1, 1), m(2, 1),
            m(0, 2), m(1, 2), m(2, 2)
        };
    }

    inline vec3 toEigen(const glm::vec3& v) {
        return {v.x, v.y, v.z};
    }

    inline vec4 toEigen(const glm::vec4& v) {
        return {v.x, v.y, v.z, v.w};
    }

    inline mat3 toEigen(const glm::mat3& m) {
        return mat3{
            {m[0][0], m[1][0], m[2][0]},
            {m[0][1], m[1][1], m[2][1]},
            {m[0][2], m[1][2], m[2][2]}
        };
    }

    inline mat4 toEigen(const glm::mat4& m) {
        return mat4{
            {m[0][0], m[1][0], m[2][0], m[3][0]},
            {m[0][1], m[1][1], m[2][1], m[3][1]},
            {m[0][2], m[1][2], m[2][2], m[3][2]},
            {m[0][3], m[1][3], m[2][3], m[3][3]}
        };
    }

    inline vec3 unProject(const vec3& winCoords, const mat4& view, const mat4& proj, const vec4& viewport) {
        vec4 tmp;
        tmp[0] = (winCoords[0] - viewport[0]) / viewport[2] * 2.0f - 1.0f;
        tmp[1] = (winCoords[1] - viewport[1]) / viewport[3] * 2.0f - 1.0f;
        tmp[2] = winCoords[2] * 2.0f - 1.0f;
        tmp[3] = 1.0f;
        mat4 invVP = (proj * view).inverse();
        vec4 obj = invVP * tmp;
        obj /= obj[3];
        return obj.head<3>();
    }

    inline mat3 skew(const vec3& v) {
        return mat3{
            {0.0, -v.z(), v.y()},
            {v.z(), 0.0, -v.x()},
            {-v.y(), v.x(), 0.0}
        };
    }

    inline mat3 outerProduct(const vec3& a, const vec3& b) {
        return mat3{
            {a.x() * b.x(), a.x() * b.y(), a.x() * b.z()},
            {a.y() * b.x(), a.y() * b.y(), a.y() * b.z()},
            {a.z() * b.x(), a.z() * b.y(), a.z() * b.z()}
        };
    }
};
