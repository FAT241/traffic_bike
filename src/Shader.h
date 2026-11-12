#pragma once
#include <GL/glext.h>
#include <GLFW/glfw3.h>
#include <stdexcept>
#include <string>
#define SHADER_FUNCTIONS(X)                                                                        \
    X(PFNGLCREATESHADERPROC, glCreateShader)                                                       \
    X(PFNGLSHADERSOURCEPROC, glShaderSource)                                                       \
    X(PFNGLCOMPILESHADERPROC, glCompileShader)                                                     \
    X(PFNGLGETSHADERIVPROC, glGetShaderiv)                                                         \
    X(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog)                                               \
    X(PFNGLCREATEPROGRAMPROC, glCreateProgram)                                                     \
    X(PFNGLATTACHSHADERPROC, glAttachShader)                                                       \
    X(PFNGLLINKPROGRAMPROC, glLinkProgram)                                                         \
    X(PFNGLGETPROGRAMIVPROC, glGetProgramiv)                                                       \
    X(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog)                                             \
    X(PFNGLDELETESHADERPROC, glDeleteShader)                                                       \
    X(PFNGLDELETEPROGRAMPROC, glDeleteProgram)                                                    \
    X(PFNGLUSEPROGRAMPROC, glUseProgram)
#define DECLARE(T, N) inline T N = nullptr;
SHADER_FUNCTIONS(DECLARE)
#undef DECLARE
inline void loadShaders() {
#define LOAD(T, N)                                                                                 \
    N = reinterpret_cast<T>(glfwGetProcAddress(#N));                                               \
    if (!N)                                                                                        \
        throw std::runtime_error("Missing OpenGL function: " #N);
    SHADER_FUNCTIONS(LOAD)
#undef LOAD
}

// Biên dịch một shader và trả thông báo lỗi GLSL nếu mã nguồn không hợp lệ.
inline GLuint compileShader(GLenum kind, const char *source) {
    GLuint shader = glCreateShader(kind);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        char log[2048]{};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        glDeleteShader(shader);
        throw std::runtime_error(std::string("Shader compilation failed: ") + log);
    }
    return shader;
}

// Tạo chương trình tô sáng từng pixel bằng pháp tuyến và vector nửa góc Blinn–Phong.
inline GLuint createLightingProgram() {
    const char *vertexSource = R"glsl(#version 120
varying vec3 surfaceNormal;
varying vec3 eyePosition;

void main() {
    vec4 position = gl_ModelViewMatrix * gl_Vertex;
    eyePosition = position.xyz;
    surfaceNormal = gl_NormalMatrix * gl_Normal;
    gl_FrontColor = gl_Color;
    gl_Position = gl_ProjectionMatrix * position;
}
)glsl";
    const char *fragmentSource = R"glsl(#version 120
varying vec3 surfaceNormal;
varying vec3 eyePosition;

void main() {
    vec3 normal = normalize(surfaceNormal);
    vec3 lightDirection = normalize(gl_LightSource[0].position.xyz);
    vec3 viewDirection = normalize(-eyePosition);
    float diffuse = max(dot(normal, lightDirection), 0.0);
    float specular = diffuse > 0.0
        ? pow(max(dot(normal, normalize(lightDirection + viewDirection)), 0.0),
              max(1.0, gl_FrontMaterial.shininess))
        : 0.0;
    gl_FragColor = vec4(gl_Color.rgb * (0.28 + 0.72 * diffuse)
                       + gl_FrontMaterial.specular.rgb * specular, 1.0);
}
)glsl";
    GLuint vertex = 0, fragment = 0, program = 0;
    try {
        vertex = compileShader(GL_VERTEX_SHADER, vertexSource);
        fragment = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
        program = glCreateProgram();
        glAttachShader(program, vertex);
        glAttachShader(program, fragment);
        glLinkProgram(program);
        GLint linked = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &linked);
        if (!linked) {
            char log[2048]{};
            glGetProgramInfoLog(program, sizeof(log), nullptr, log);
            throw std::runtime_error(std::string("Shader link failed: ") + log);
        }
    } catch (...) {
        if (program)
            glDeleteProgram(program);
        if (fragment)
            glDeleteShader(fragment);
        if (vertex)
            glDeleteShader(vertex);
        throw;
    }
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    return program;
}
