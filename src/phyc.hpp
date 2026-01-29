#pragma once
#include <fstream>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <glm/glm.hpp>

namespace PhyC {
    static constexpr auto PHYC_VERSION = "0.0.1";
    static constexpr auto PHYC_VERSION_MAJOR = 0;
    static constexpr auto PHYC_VERSION_MINOR = 0;
    static constexpr auto PHYC_VERSION_PATCH = 1;

    using vec3 = Eigen::Vector3d;
    using mat3 = Eigen::Matrix3d;
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

    inline mat3 toEigen(const glm::mat3& m) {
        return mat3{
            {m[0][0], m[1][0], m[2][0]},
            {m[0][1], m[1][1], m[2][1]},
            {m[0][2], m[1][2], m[2][2]}
        };
    }
};
