#include "BikeMeshBuilder.h"

using namespace std;

namespace bike_geometry {
namespace {
// Quadratic Bezier corner fillets; retain the outline rather than using an ellipse
// for every lamp and mirror. q(t) = (1-t)^2 A + 2t(1-t) B + t^2 C.
// Bo tròn các góc của viền đèn và gương bằng đường Bézier bậc hai.
vector<V> roundedOutline(const vector<V> &corners, double radius, int samples) {
    vector<V> result;
    for (size_t i = 0; i < corners.size(); ++i) {
        V p = corners[i], a = p + (corners[(i + corners.size() - 1) % corners.size()] - p) * radius,
          c = p + (corners[(i + 1) % corners.size()] - p) * radius;
        for (int k = 0; k <= samples; ++k) {
            double t = double(k) / samples;
            result.push_back(a * ((1 - t) * (1 - t)) + p * (2 * t * (1 - t)) + c * (t * t));
        }
    }
    return result;
}
// A fender is a thin curved shell around the top of the wheel, with rolled edges.
// Dựng vè cong ôm phía trên lốp và hai mép vè.
void fender(Builder &b, V axlePos, double radius, double halfWidth, Finish f, double start, double end) {
    auto top = [&](double u, double v) {
        double a = (start + (end - start) * u) * PI / 180, t = (v - .5) * PI;
        double span = .80 + .20 * sin(PI * u),
               r = radius + .032 + .025 * cos(t) - .014 * (1 - cos(t)) * sin(PI * u);
        return axlePos + V{r * cos(a), r * sin(a), halfWidth * span * sin(t)};
    };
    b.grid(f, 36, 22, top);
    for (int side : {0, 1})
        b.grid(
            f, 36, 3,
            [&](double u, double v) {
                V p = top(u, side);
                return p + V{0, -v * (.004 + .021 * sin(PI * u)), (side ? 1 : -1) * .002 * v};
            },
            side == 1);
}
} // namespace

// Dựng lốp, vành, nan hoa hoặc chấu, trục và chi tiết phanh.
void wheel(Builder &b, const BikeSpec &s, bool front, bool wave) {
    b.joint = front ? Joint::FrontWheel : Joint::RearWheel;
    Tire tire = front ? s.front : s.rear;
    V c = axle(s, front);
    double rim = tire.rimRadius(), outer = tire.radius(), width = tire.widthMm / 1000.;
    // Pneumatic tire: outer radius and widest section follow the nominal marking.
    double minor = (outer - rim) / 2, major = (outer + rim) / 2;
    auto tirePoint = [&](double a, double t) {
        double r = major + minor * cos(t);
        return c + V{r * cos(a), r * sin(a), width * .5 * sin(t)};
    };
    b.grid(Finish::Rubber, b.distant() ? 32 : 64, b.distant() ? 12 : 20,
           [&](double u, double v) { return tirePoint(2 * PI * u, 2 * PI * v); });
    for (int side : {-1, 1})
        for (int i = 0; i < (b.distant() ? 0 : 48); i++) {
            double a = 2 * PI * i / 48;
            b.grid(
                Finish::Graphite, 3, 1,
                [&](double u, double v) {
                    double az = a + .052 * u + .010 * v;
                    return tirePoint(az, side * (.12 + .97 * u)) +
                           V{.0003 * cos(az), .0003 * sin(az), 0};
                },
                side > 0);
        }
    auto ring = [&](Finish f, double r, double radial, double axial, double z) {
        b.grid(f, b.distant() ? 24 : 48, b.distant() ? 6 : 10, [&](double u, double v) {
            double a = 2 * PI * u, t = 2 * PI * v;
            return c + V{(r + radial * cos(t)) * cos(a), (r + radial * cos(t)) * sin(a),
                         z + axial * sin(t)};
        });
    };
    ring(wave ? Finish::Bright : Finish::Alloy, rim - .007, .011, width * .30, 0);
    for (int sign : {-1, 1})
        ring(Finish::Bright, rim - .003, .004, .003, sign * width * .29);
    double hub = wave ? .059 : .038;
    b.tube(Finish::Alloy, c + V{0, 0, -.042}, c + V{0, 0, .042}, hub, 0, 30);
    if (wave) {
        for (int k = 0; k < 36; k++) {
            double a = k * 2 * PI / 36, sgn = k % 2 ? 1 : -1, shift = (k % 4 < 2 ? .45 : -.45);
            b.tube(Finish::Bright, c + V{hub * cos(a + shift), hub * sin(a + shift), sgn * .039},
                   c + V{(rim - .012) * cos(a), (rim - .012) * sin(a), sgn * .009}, .0017, 0, 5);
        }
    } else {
        for (int k = 0; k < 8; k++)
            b.blade(c, 2 * PI * k / 8, hub, rim - .01);
        if (front) {
            // Annular brake rotor with openings between bridges and outer braking track.
            ring(Finish::Bright, .094, .009, .0025, -.041);
            for (int k = 0; k < 8; k++) {
                double a = k * PI / 4;
                b.tube(Finish::DarkMetal, c + V{.032 * cos(a), .032 * sin(a), -.041},
                       c + V{.093 * cos(a + .27), .093 * sin(a + .27), -.041}, .007, 0, 6);
            }
        }
    }
    b.joint = Joint::Fork;
    if (!wave && front)
        b.tube(Finish::DarkMetal, c + V{-.063, .038, -.044}, c + V{-.090, .080, -.044}, .018);
}

// Dựng phuộc trước, tay lái, cụm đèn đầu, gương và đồng hồ.
void steering(Builder &b, const BikeSpec &s, bool wave) {
    b.joint = Joint::Fork;
    V c = axle(s, true);
    double stem = wave ? .317 : .331, head = s.height;
    for (int sign : {-1, 1}) {
        double z = sign * (wave ? .069 : .078);
        V base = c + V{0, 0, z}, knee = c + V{-.054, .207, z}, top = c + V{-.108, .391, z};
        b.tube(wave ? Finish::Graphite : Finish::Alloy, base, knee, .020, .024);
        b.tube(Finish::Bright, knee, top, .0145);
        b.tube(Finish::Rubber, knee, knee + V{-.006, .026, 0}, .022);
        b.tube(Finish::Alloy, base, base + V{0, 0, sign * .012}, .014, 0, 6);
    }
    b.tube(Finish::DarkMetal, c + V{-.12, .383, -.078}, c + V{-.12, .383, .078}, .021);
    b.tube(Finish::DarkMetal, c + V{-.12, .38, 0}, {stem, head - .075, 0}, .029);
    fender(b, c, s.front.radius(), wave ? .057 : .070, Finish::Paint, 37, 157);
    // Lower bar enclosure and lacquered handlebar nacelle surround the headlamp.
    b.skin(Finish::Graphite,
           {{stem - .095, head - .107, head - .056, .105},
            {stem - .049, head - .131, head - .046, .171},
            {stem + .035, head - .137, head - .046, .168},
            {stem + .065, head - .117, head - .058, .144}},
           .59);
    b.skin(Finish::Paint,
           {{stem - .1, head - .069, head - .054, .088},
            {stem - .063, head - .091, head - .009, .152},
            {stem + .023, head - .104, head, .184},
            {stem + .052, head - .106, head - .025, .172},
            {stem + .070, head - .089, head - .039, .145}},
           .62);
    // Lens vertices follow a sloping front face. Vision's central reflector is
    // narrower; Wave integrates the turn lamps into the handlebar housing.
    double face = stem + .155, w = wave ? .125 : .104;
    auto lens = [&](double z, double y) {
        return V{face - .50 * y - .022 * abs(z / w), head - .077 + y, z};
    };
    vector<V> headlightRim =
        roundedOutline({lens(-w * .96, .054), lens(-w * .43, .063), lens(w * .43, .063),
                        lens(w * .96, .054), lens(w * .89, -.019), lens(w * .53, -.061),
                        lens(0, -.067), lens(-w * .53, -.061), lens(-w * .89, -.019)},
                       .24, b.distant() ? 1 : 4);
    V rimCenter{};
    for (V p : headlightRim)
        rimCenter = rimCenter + p;
    rimCenter = rimCenter * (1. / headlightRim.size());
    for (size_t i = 0; i < headlightRim.size(); ++i) {
        V a = headlightRim[i], b2 = headlightRim[(i + 1) % headlightRim.size()];
        auto back = [&](V p) {
            return V{stem + .038, rimCenter.y + (p.y - rimCenter.y) * .80, p.z * 1.12};
        };
        auto trim = (a.y + b2.y) * .5 < rimCenter.y - .013 ? Finish::Graphite : Finish::Paint;
        V outward = (a + b2) * .5 - rimCenter;
        b.tri(trim, a, b2, back(a), outward);
        b.tri(trim, b2, back(b2), back(a), outward);
    }
    b.optical(headlightRim, {1, .50, 0}, Finish::Lens, .009);
    for (int sign : {-1, 1}) {
        if (wave) {
            auto indicator = roundedOutline({lens(sign * .124, .042), lens(sign * .158, .032),
                                             lens(sign * .182, .009), lens(sign * .167, -.027),
                                             lens(sign * .123, -.033)},
                                            .26, b.distant() ? 1 : 3);
            for (size_t k = 0; k < indicator.size(); ++k) {
                V a = indicator[k], c2 = indicator[(k + 1) % indicator.size()];
                V backA{stem + .045, a.y, a.z}, backC{stem + .045, c2.y, c2.z};
                V facing = (a + c2) * .5 - lens(sign * .148, .003);
                b.tri(Finish::Paint, a, c2, backA, facing);
                b.tri(Finish::Paint, c2, backC, backA, facing);
            }
            b.optical(indicator, {1, .28, sign * .42}, Finish::Amber, .006);
        }
        double edge = s.width * .5;
        V grip1{stem - .044, head - .066, sign * .212},
            grip2{stem - .061, head - .072, sign * (edge - .005)};
        b.tube(Finish::DarkMetal, {stem, head - .075, sign * .125}, grip1, .019);
        b.tube(Finish::Graphite, grip1, grip1 + V{0, 0, sign * .026}, .028);
        b.tube(Finish::Rubber, grip1 + V{0, 0, sign * .025}, grip2, .0175);
        for (int k = 0; k < (b.distant() ? 0 : 7); k++) {
            double t = .29 + .092 * k;
            V pos = grip1 + (grip2 - grip1) * t;
            b.tube(Finish::Graphite, pos, pos + V{0, 0, sign * .002}, .0182, 0, 10);
        }
        b.tube(Finish::DarkMetal, {grip2.x, grip2.y, grip2.z}, {grip2.x, grip2.y, sign * edge}, .017);
        if (!wave || sign > 0) {
            V hinge{stem, head - .072, sign * .189}, bend{stem + .010, head - .073, sign * .235};
            b.tube(Finish::Alloy, hinge, bend, .007);
            b.tube(Finish::Alloy, bend, {stem - .018, head - .081, sign * (edge - .005)}, .006, .0035);
        }
        // Back shell, silvered glass and stem are independent parts.
        V base{stem - .035, head - .023, sign * .177}, neck{stem - .050, head + .080, sign * .225};
        V mid{stem - .062, head + .142, sign * .261};
        b.tube(Finish::Graphite, base, base + V{0, .031, sign * .008}, .010);
        b.tube(Finish::DarkMetal, base, neck, .006);
        b.tube(Finish::DarkMetal, neck, mid, .006);
        vector<V> outline;
        for (auto zy : {pair<double, double>{-.061, -.017},
                        {-.043, -.040},
                        {.034, -.025},
                        {.065, -.006},
                        {.065, .020},
                        {.038, .044},
                        {-.025, .041},
                        {-.062, .015}})
            outline.push_back(mid + V{.010 * zy.first / .065, zy.second, sign * zy.first});
        outline = roundedOutline(outline, .37, b.distant() ? 1 : 4);
        b.panel(Finish::Graphite, outline, {1, .15, sign * .16}, .020);
        b.panel(Finish::Graphite, outline, {-1, .1, -sign * .16}, .006);
        for (auto &p : outline)
            p = mid + (p - mid) * .86 + V{-.009, 0, 0};
        b.panel(Finish::Mirror, outline, {-1, .15, -sign * .16}, .003);
    }
    // Recessed analog cluster on the rider-facing sloped surface.
    V mid{stem - .078, head - .027, 0}, across{0, 0, 1}, up{.034, .019, 0};
    vector<V> dial;
    for (int k = 0; k < 32; k++) {
        double a = 2 * PI * k / 32;
        dial.push_back(mid + across * (.086 * cos(a)) + up * sin(a));
    }
    b.panel(Finish::DarkMetal, dial, {-1, .73, 0}, .005);
    for (int k = 0; k < 11; k++) {
        double a = PI * k / 10;
        V p = mid + V{-.006, .005, 0} + across * (.07 * cos(a)) + up * (.79 * sin(a));
        b.tube(Finish::Reflector, p, p - across * (.005 * cos(a)) - up * (.11 * sin(a)), .0014, 0, 5);
    }
    b.tube(Finish::Amber, mid + V{-.01, .007, 0}, mid + V{-.01, .007, -.052}, .0016, 0, 6);
}
} // namespace bike_geometry
