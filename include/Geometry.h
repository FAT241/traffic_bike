#pragma once
#include "Math.h"
#include <GLFW/glfw3.h>
#include <array>
#include <vector>
struct Vertex {
    V p, n;
};
struct Face {
    std::array<size_t, 3> vertexIndices;
};
struct Frame {
    Frame() = default;
    Frame(const Frame &) = delete;
    Frame &operator=(const Frame &) = delete;
    Frame(Frame &&other) noexcept;
    Frame &operator=(Frame &&other) noexcept;
    ~Frame();
    // Lưới đã vẽ phải được hủy khi context OpenGL vẫn còn hoạt động.
    mutable GLuint displayList = 0;
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    void addTriangle(Vertex a, Vertex b, Vertex c);
    void draw(bool normals = false) const;
};
Frame ellipsoid(double x, double y, double z);
void material(V c, double shine = 30);
void box(double x, double y, double z);
void rod(V a, V b, double radius);
