#pragma once
#include <glad/glad.h>

namespace PhyCRenderer {
    struct ShadowMapResources {
        GLuint fbo{0};
        GLuint textureID{0};

        void bindForWriting() const {
            glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        }

        void bindForReading(GLenum textureUnit) const {
            glActiveTexture(textureUnit);
            glBindTexture(GL_TEXTURE_2D, textureID);
        }
    };
}
