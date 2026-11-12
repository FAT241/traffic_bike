#pragma once
#include "BikeModel.h"
#include "Simulation.h"
struct View {
    double yaw = 40, pitch = 48, distance = 63;
    int mode = 0, selected = 0;
    BikeType showroom = BikeType::Wave;
    double showroomYaw = 24, showroomPitch = 12, showroomDist = 2.9;
    bool cleanPreview = false;
    bool orthographic = false, dimensions = false;
    bool wire = false, normals = false, paths = false, controls = false, paused = false,
         phong = true;
};
class Renderer {
  public:
    Renderer() = default;
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;
    ~Renderer();
    std::array<BikeModel, 2> models, distantModels;
    GLuint program = 0;
    void init();
    void draw(const Simulation &sim, View &view, int width, int height);

  private:
    Mesh signalLens;
    V setupCamera(const Simulation &sim, View &view, size_t selectedIndex, int width, int height);
    void drawShowroom(const View &view);
    void drawIntersection(const Simulation &sim, const View &view);
    void drawVehicles(const Simulation &sim, const View &view, V eye);
    void drawHud(const Simulation &sim, const View &view, size_t selectedIndex, int width, int height);
};
