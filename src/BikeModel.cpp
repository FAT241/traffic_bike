#include "BikeMeshBuilder.h"
#include "Transform.h"
#include <iostream>
#include <stdexcept>
#include <utility>

using namespace std;

// Honda VN product leaflets, Oct/Nov 2024. Body stations are photo estimates.
// See README for the exact editions and dimension conventions.
// Trả thông số kích thước và cỡ lốp của Wave hoặc Vision.
const BikeSpec &bikeSpec(BikeType type) {
    static const BikeSpec wave{"Honda Wave Alpha 110", 1.913,       .689, 1.076, 1.224, .770,
                               {70, 90, 17},           {80, 90, 17}};
    static const BikeSpec vision{
        "Honda Vision Standard", 1.871, .686, 1.101, 1.255, .761, {80, 90, 14}, {90, 90, 14}};
    return type == BikeType::Wave ? wave : vision;
}

namespace {
using bike_geometry::origin;
// Chọn màu vật liệu, kể cả màu đèn hậu khi xe phanh.
V pigment(Finish f, bool wave, bool braking) {
    switch (f) {
    case Finish::Paint:
        return wave ? V{.034, .22, .52} : V{.93, .91, .84};
    case Finish::Graphite:
        return {.068, .078, .091};
    case Finish::Rubber:
        return {.031, .035, .040};
    case Finish::Seat:
        return {.070, .065, .068};
    case Finish::DarkMetal:
        return {.174, .189, .206};
    case Finish::Alloy:
        return {.52, .57, .62};
    case Finish::Bright:
        return {.78, .81, .84};
    case Finish::Lens:
        return {.80, .85, .86};
    case Finish::Reflector:
        return {.92, .95, .95};
    case Finish::Amber:
        return {1, .35, .055};
    case Finish::Tail:
        return braking ? V{1, .065, .028} : V{.57, .028, .022};
    case Finish::Mirror:
        return {.58, .75, .84};
    case Finish::Accent:
        return {.90, .42, .08};
    case Finish::Decal:
        return {.84, .89, .90};
    }
    return {};
}
} // namespace
// Dựng và kiểm tra lưới xe mới rồi thay toàn bộ mô hình cũ.
void BikeModel::build(BikeType kind, bool distantDetail) {
    BikeModel built;
    built.distant = distantDetail;
    built.type = kind;
    bike_geometry::Builder b(built);
    const auto &s = bikeSpec(kind);
    bool wave = kind == BikeType::Wave;
    if (wave)
        bike_geometry::waveBody(b, s);
    else
        bike_geometry::visionBody(b, s);
    bike_geometry::steering(b, s, wave);
    bike_geometry::wheel(b, s, true, wave);
    bike_geometry::wheel(b, s, false, wave);
    built.verify();
    *this = move(built);
}
// Vẽ thân, cụm lái và hai bánh bằng các ma trận chuyển động riêng.
void BikeModel::draw(double distance, double steer, bool braking, bool normals,
                     bool controls) const {
    const auto &s = bikeSpec(type);
    bool wave = type == BikeType::Wave;
    // Inclined steering axis through the front hub; fork and wheel inherit yaw.
    // Negative roll moves the tire contact point backwards in the bike's frame.
    double rake = wave ? 25. : 26.;
    Matrix4x4 steeringRotation = multipleMatrix(
        rotationZMatrix(rake), multipleMatrix(rotationYMatrix(steer), rotationZMatrix(-rake)));
    for (Joint j : {Joint::Frame, Joint::Fork, Joint::FrontWheel, Joint::RearWheel}) {
        transforms.push();
        if (j != Joint::Frame) {
            V o = origin(s, j);
            auto local = translationMatrix(o.x, o.y, o.z);
            if (j != Joint::RearWheel)
                local = multipleMatrix(local, steeringRotation);
            if (j != Joint::Fork) {
                double r = (j == Joint::FrontWheel ? s.front : s.rear).radius();
                local = multipleMatrix(local, rotationZMatrix(-distance / r * 180 / PI));
            }
            transforms.apply(local);
        }
        for (auto it = batches.lower_bound({j, Finish::Paint});
             it != batches.end() && it->first.first == j; ++it) {
            auto f = it->first.second;
            double shine = (f == Finish::Rubber || f == Finish::Seat) ? 9. : (f == Finish::Graphite ? 22. : 78.);
            material(pigment(f, wave, braking), shine);
            float highlight = (f == Finish::Rubber     ? .045f
                               : f == Finish::Seat     ? .085f
                               : f == Finish::Graphite ? .15f
                                                  : .49f);
            const GLfloat specular[] = {highlight, highlight, highlight, 1};
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
            it->second.draw(normals);
        }
        transforms.pop();
    }
    if (controls) {
        glDisable(GL_LIGHTING);
        glColor3f(1, .58, .05);
        glLineWidth(2);
        glPointSize(6);
        for (auto mode : {GL_POINTS, GL_LINE_STRIP}) {
            glBegin(mode);
            for (V p : seatControls)
                glVertex3d(p.x, p.y + .006, p.z);
            glEnd();
        }
        glEnable(GL_LIGHTING);
    }
}
// Kiểm tra đỉnh, chỉ số mặt, kích thước xe và vị trí tiếp đất của lốp.
void BikeModel::verify() const {
    const auto &s = bikeSpec(type);
    V low{10, 10, 10}, high{-10, -10, -10};
    size_t triangles = 0;
    for (const auto &batch : batches) {
        V o = origin(s, batch.first.first);
        for (const auto &vertex : batch.second.vertices) {
            V p = vertex.p + o;
            if (!isfinite(dot(p, p)) || fabs(length(vertex.n) - 1) > .00001)
                throw runtime_error("Invalid bike mesh");
            low = {min(low.x, p.x), min(low.y, p.y), min(low.z, p.z)};
            high = {max(high.x, p.x), max(high.y, p.y), max(high.z, p.z)};
        }
        for (const Face &face : batch.second.faces)
            for (size_t index : face.vertexIndices)
                if (index >= batch.second.vertices.size())
                    throw runtime_error("Bike face references a missing vertex");
        triangles += batch.second.faces.size();
    }
    auto check = [&](bool valid, const char *name) {
        if (!valid)
            throw runtime_error(string(s.name) + " geometry: " + name);
    };
    double contactCenter[2]{};
    for (Joint j : {Joint::FrontWheel, Joint::RearWheel}) {
        const auto &vertices = batches.at({j, Finish::Rubber}).vertices;
        V o = origin(s, j);
        double ground = 10, radius = 0, minX = 10, maxX = -10;
        for (const auto &v : vertices) {
            ground = min(ground, v.p.y + o.y);
            minX = min(minX, v.p.x + o.x);
            maxX = max(maxX, v.p.x + o.x);
            radius = max(radius, hypot(v.p.x, v.p.y));
        }
        contactCenter[j == Joint::FrontWheel ? 1 : 0] = (maxX + minX) / 2;
        check(fabs(ground) < 1e-7 && fabs(radius - o.y) < 1e-7,
              "nominal tire radius / ground contact");
    }
    check(fabs(high.x - low.x - s.length) < .002, "overall length");
    check(fabs(high.z - low.z - s.width) < .002, "overall width");
    check(fabs(contactCenter[1] - contactCenter[0] - s.wheelbase) < 1e-8, "wheelbase");
    double seatX = type == BikeType::Wave ? -.215 : -.26, riderSeat = -1;
    for (const auto &v : batches.at({Joint::Frame, Finish::Seat}).vertices)
        if (fabs(v.p.x - seatX) < 1e-7)
            riderSeat = max(riderSeat, v.p.y);
    check(fabs(riderSeat - s.seatHeight) < 1e-6, "seat surface at rider station");
    cout << s.name << (distant ? " [traffic]" : " [studio]") << ": mesh (mm, mirrors fitted) "
              << lround(1000 * (high.x - low.x)) << " x "
              << lround(1000 * (high.z - low.z)) << " x "
              << lround(1000 * (high.y - low.y)) << ", rider seat "
              << lround(1000 * riderSeat) << ", wheelbase " << lround(1000 * s.wheelbase)
              << ", " << triangles << " triangles\n";
}
