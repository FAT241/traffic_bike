#pragma once
#include "BikeModel.h"
#include <functional>

namespace bike_geometry {
struct Station {
    // Vị trí dọc xe, cao độ dưới/trên, nửa bề rộng; đơn vị mét.
    double x, low, high, halfWidth;
};
using Stations = std::vector<Station>;
using Surface = std::function<V(double, double)>;
// Tính tâm trục bánh trước hoặc sau từ chiều dài cơ sở và bán kính lốp.
inline V axle(const BikeSpec &s, bool front) {
    return {s.wheelbase * (front ? .5 : -.5), (front ? s.front : s.rear).radius(), 0};
}
// Chọn gốc tọa độ cục bộ cho thân, cụm lái hoặc bánh xe.
inline V origin(const BikeSpec &s, Joint j) { return j == Joint::Frame ? V{} : axle(s, j != Joint::RearWheel); }
// Tính lũy thừa giữ dấu để tạo tiết diện bo tròn.
inline double signedPower(double x, double power) {
    return std::copysign(std::pow(std::abs(x), power), x);
}

// Piecewise Hermite interpolation with bounded slopes. Passes through the actual
// authored stations, preserving peaks such as the seat reference height.
// Nội suy Hermite giữa các tiết diện để bề mặt chuyển tiếp liên tục.
inline double interpolate(const Stations &a, int i, double t, double Station::*field) {
    auto slope = [&](int k) { return (a[k + 1].*field - a[k].*field) / (a[k + 1].x - a[k].x); };
    auto tangent = [&](int k) {
        if (k == 0)
            return slope(0);
        if (k == int(a.size()) - 1)
            return slope(k - 1);
        double l = slope(k - 1), r = slope(k);
        return l * r <= 0 ? 0. : 2 * l * r / (l + r);
    };
    double d = a[i + 1].x - a[i].x, q = 1 - t;
    return q * q * (1 + 2 * t) * (a[i].*field) + t * t * (3 - 2 * t) * (a[i + 1].*field) +
           d * t * q * (q * tangent(i) - t * tangent(i + 1));
}
class Builder {
    BikeModel &model;
    const BikeSpec &spec;
    // Lấy lưới ứng với cụm chuyển động và vật liệu đang dựng.
    Frame &frame(Finish f) { return model.batches[{joint, f}]; }
    // Đưa vị trí đỉnh về hệ tọa độ cục bộ của cụm chuyển động.
    Vertex vertex(V p, V n) const { return {p - origin(spec, joint), n}; }

  public:
    // Kiểm tra mức chi tiết thấp dùng khi quan sát xe từ xa.
    bool distant() const { return model.distant; }
    Joint joint = Joint::Frame;
    // Gắn bộ dựng hình với mẫu xe và thông số kích thước của nó.
    Builder(BikeModel &m) : model(m), spec(bikeSpec(m.type)) {}
    // Thêm một mặt tam giác và chỉnh thứ tự đỉnh theo hướng pháp tuyến.
    void tri(Finish f, V a, V b, V c, V facing) {
        V n = cross(b - a, c - a);
        if (length(n) < 1e-12)
            return;
        if (dot(n, facing) < 0) {
            std::swap(b, c);
            n = n * (-1);
        }
        n = unit(n);
        frame(f).addTriangle(vertex(a, n), vertex(b, n), vertex(c, n));
    }
    // Lấy mẫu bề mặt thành lưới đỉnh và mặt tam giác dùng chung chỉ số.
    void grid(Finish f, int nu, int nv, const Surface &surface, bool flip = false) {
        Frame &out = frame(f);
        const size_t first = out.vertices.size();
        for (int i = 0; i <= nu; i++)
            for (int k = 0; k <= nv; k++) {
                double u = double(i) / nu, v = double(k) / nv, e = 1e-5;
                V du = surface(std::min(1., u + e), v) - surface(std::max(0., u - e), v);
                // v is also used for open patches; one-sided differences at their edges.
                V dv = surface(u, std::min(1., v + e)) - surface(u, std::max(0., v - e));
                V n = cross(du, dv);
                double l = length(n);
                out.vertices.push_back(
                    vertex(surface(u, v), l > 1e-20 ? n * ((flip ? -1. : 1.) / l) : V{0, 1, 0}));
            }
        for (int i = 0; i < nu; i++)
            for (int k = 0; k < nv; k++) {
                size_t a = first + i * (nv + 1) + k, b = a + nv + 1, c = b + 1, d = a + 1;
                if (flip) {
                    out.faces.push_back({{a, c, b}});
                    out.faces.push_back({{a, d, c}});
                } else {
                    out.faces.push_back({{a, b, c}});
                    out.faces.push_back({{a, c, d}});
                }
            }
    }
    // Dựng ống hoặc thanh tròn nối hai điểm, dùng cho khung, phuộc và nan hoa.
    void tube(Finish f, V a, V b, double ra, double rb = 0, int slices = 12) {
        if (distant())
            slices = std::min(slices, 8);
        if (rb <= 0)
            rb = ra;
        V axis = unit(b - a),
          e = unit(cross(axis, std::abs(axis.y) < .9 ? V{0, 1, 0} : V{1, 0, 0})),
          g = cross(axis, e);
        auto radial = [&](double v) { return e * cos(2 * PI * v) + g * sin(2 * PI * v); };
        grid(
            f, 1, slices,
            [&](double u, double v) { return a + (b - a) * u + radial(v) * (ra + (rb - ra) * u); },
            true);
        for (int k = 0; k < slices; k++) {
            V p = radial(double(k) / slices), q = radial(double(k + 1) / slices);
            tri(f, a, a + p * ra, a + q * ra, axis * (-1));
            tri(f, b, b + p * rb, b + q * rb, axis);
        }
    }
    // Rounded longitudinal sections for seat cushions, side fairings and casings.
    // Dựng vỏ cong từ danh sách tiết diện dọc theo thân xe.
    void skin(Finish f, const Stations &s, double power = .72, double zCenter = 0,
              bool markSeat = false) {
        int n = int(s.size()) - 1;
        auto sample = [&](double u, double v) {
            double at = u * n;
            int i = std::min(n - 1, int(at));
            double t = at - i;
            double low = interpolate(s, i, t, &Station::low),
                   high = interpolate(s, i, t, &Station::high);
            double width = interpolate(s, i, t, &Station::halfWidth), ang = 2 * PI * v;
            return V{s[i].x + (s[i + 1].x - s[i].x) * t,
                     (high + low) * .5 + (high - low) * .5 * signedPower(cos(ang), power),
                     zCenter + width * signedPower(sin(ang), power)};
        };
        int ringSamples = distant() ? 12 : 32;
        grid(f, n * (distant() ? 2 : 6), ringSamples, sample, true);
        for (int end : {0, 1})
            for (int k = 0; k < ringSamples; k++) {
                V p = sample(end, double(k) / ringSamples),
                  q = sample(end, double(k + 1) / ringSamples);
                auto st = end ? s.back() : s.front();
                tri(f, {st.x, (st.low + st.high) * .5, zCenter}, p, q, {end ? 1. : -1., 0, 0});
            }
        if (markSeat) {
            for (auto st : s)
                model.seatControls.push_back({st.x, st.high, zCenter});
        }
    }
    // A pressed panel described by a perimeter, a bevel and a crowned face.
    // The loop must be star-shaped about its centre. All coordinates are in meters.
    // Dựng tấm đa giác có viền vát, dùng cho các chi tiết dạng tấm.
    void panel(Finish f, const std::vector<V> &outline, V normal, double bevel) {
        V center{};
        for (auto p : outline)
            center = center + p;
        center = center * (1. / outline.size());
        normal = unit(normal);
        std::vector<V> prev = outline;
        for (auto layer : {std::pair<double, double>{.92, .62}, {.70, .92}, {.40, 1.0}}) {
            std::vector<V> next;
            for (V p : outline)
                next.push_back(center + (p - center) * layer.first +
                               normal * (bevel * layer.second));
            for (size_t i = 0; i < prev.size(); i++) {
                size_t k = (i + 1) % prev.size();
                tri(f, prev[i], prev[k], next[i], normal);
                tri(f, prev[k], next[k], next[i], normal);
            }
            prev = next;
        }
        for (size_t i = 0; i < prev.size(); i++)
            tri(f, prev[i], prev[(i + 1) % prev.size()], center + normal * bevel, normal);
    }
    // Optical unit with a shaped black gasket, metal reflector surround and separate lens.
    // Dựng mặt kính, viền và phần phản quang của cụm đèn.
    void optical(const std::vector<V> &perimeter, V normal, Finish lens, double depth) {
        V mid{};
        for (V p : perimeter)
            mid = mid + p;
        mid = mid * (1. / perimeter.size());
        normal = unit(normal);
        auto layer = [&](double scale, double step, Finish f, double crown) {
            std::vector<V> ring;
            for (V p : perimeter)
                ring.push_back(mid + (p - mid) * scale + normal * step);
            panel(f, ring, normal, crown);
        };
        for (size_t i = 0; i < perimeter.size(); ++i) {
            V a = perimeter[i], b = perimeter[(i + 1) % perimeter.size()],
              delta = normal * (depth * 1.7);
            V facing = (a + b) * .5 - mid;
            tri(Finish::Graphite, a, b, b - delta, facing);
            tri(Finish::Graphite, a, b - delta, a - delta, facing);
        }
        layer(1., 0, Finish::Graphite, depth * .28);
        layer(.92, depth * .32, Finish::Bright, depth * .25);
        layer(.82, depth * .63, lens, depth * .32);
        // Prismatic reflector flutes across the clear lens.
        V across = unit(cross({0, 1, 0}, normal));
        V up = unit(cross(normal, across));
        double w = 0, h = 0;
        for (V p : perimeter) {
            w = std::max(w, std::abs(dot(p - mid, across)));
            h = std::max(h, std::abs(dot(p - mid, up)));
        }
        if (!distant() && lens == Finish::Lens)
            for (int i = -3; i <= 3; i++) {
                V base = mid + normal * depth + across * (i * w * .15);
                tube(Finish::Reflector, base - up * h * .32, base + up * h * .35, .0009, 0, 6);
            }
    }
    // Extruded curved alloy spoke (not a solid wheel disk).
    // Dựng một chấu vành đúc với hai mặt bên và bề dày.
    void blade(V center, double angle, double hub, double rimR) {
        std::vector<V> sides;
        for (auto rw : {std::pair<double, double>{hub, -.22},
                        {rimR * .68, -.04},
                        {rimR, .26},
                        {rimR, .38},
                        {rimR * .60, .17},
                        {hub, .35}})
            sides.push_back(center + V{rw.first * cos(angle + rw.second),
                                       rw.first * sin(angle + rw.second), 0});
        auto face = sides;
        for (auto &p : face)
            p.z += .009;
        panel(Finish::Alloy, face, {0, 0, 1}, .004);
        auto back = sides;
        for (auto &p : back)
            p.z -= .009;
        panel(Finish::Alloy, back, {0, 0, -1}, .004);
        for (size_t i = 0; i < sides.size(); ++i) {
            auto k = (i + 1) % sides.size();
            V n = cross(face[k] - face[i], back[i] - face[i]);
            tri(Finish::DarkMetal, face[i], face[k], back[i], n);
            tri(Finish::DarkMetal, face[k], back[k], back[i], n);
        }
    }
};

void waveBody(Builder &b, const BikeSpec &s);
void visionBody(Builder &b, const BikeSpec &s);
void steering(Builder &b, const BikeSpec &s, bool wave);
void wheel(Builder &b, const BikeSpec &s, bool front, bool wave);
} // namespace bike_geometry
