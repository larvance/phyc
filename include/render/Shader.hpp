#pragma once
#include <iostream>
#include <glad/glad.h>

namespace PhyCRenderer {
    static void checkCompileErrors(GLuint shader, const std::string& type) {
        GLint success;
        GLchar infoLog[1024];
        if (type != "PROGRAM") {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success) {
                glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
                std::cerr << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << std::endl;
            }
        } else {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success) {
                glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
                std::cerr << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << std::endl;
            }
        }
    }

    static GLuint compileShader(GLenum type, const char* source) {
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        checkCompileErrors(shader, "SHADER");
        return shader;
    }

    struct ShaderVariable {
        GLint ID;

        void set(int value) const {
            glUniform1i(ID, value);
        }

        void set(float value) const {
            glUniform1f(ID, value);
        }

        void set(const glm::vec2& value) const {
            glUniform2fv(ID, 1, &value[0]);
        }

        void set(const glm::vec3& value) const {
            glUniform3fv(ID, 1, &value[0]);
        }

        void set(const glm::vec4& value) const {
            glUniform4fv(ID, 1, &value[0]);
        }

        void set(const glm::mat2& value) const {
            glUniformMatrix2fv(ID, 1, GL_FALSE, &value[0][0]);
        }

        void set(const glm::mat3& value) const {
            glUniformMatrix3fv(ID, 1, GL_FALSE, &value[0][0]);
        }

        void set(const glm::mat4& value) const {
            glUniformMatrix4fv(ID, 1, GL_FALSE, &value[0][0]);
        }
    };

    struct Shader {
        GLuint ID = -1;
        std::unordered_map<std::string, GLint> uniformLocations{};

        Shader(const Shader&) = delete;
        Shader& operator=(const Shader&) = delete;

        Shader() {
        }

        void load(const char* vertexSource, const char* fragmentSource) {
            uniformLocations.clear();

            if (ID != -1) {
                glDeleteProgram(ID);
            }

            GLuint vShader = compileShader(GL_VERTEX_SHADER, vertexSource);
            GLuint fShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);

            ID = glCreateProgram();
            glAttachShader(ID, vShader);
            glAttachShader(ID, fShader);
            glLinkProgram(ID);
            checkCompileErrors(ID, "PROGRAM");

            glDeleteShader(vShader);
            glDeleteShader(fShader);
        }

        void use() const {
            glUseProgram(ID);
        }

        GLint getUniformLocation(const std::string& name) {
            auto it = uniformLocations.find(name);
            if (it != uniformLocations.end()) return it->second;

            GLint location = glGetUniformLocation(ID, name.c_str());
            uniformLocations[name] = location;
            return location;
        }

        template <typename T>
        void set(const std::string& name, const T& value) {
            ShaderVariable var{getUniformLocation(name)};
            var.set(value);
        }

        ShaderVariable get(const std::string& name) {
            return {getUniformLocation(name.c_str())};
        }

        ~Shader() {
            if (ID != -1) {
                glDeleteProgram(ID);
            }
        }
    };
}
