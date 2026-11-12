#include "BikeFrameBuilder.h"
#include <string>

using namespace std;

namespace bike_geometry {
namespace {
// Invert the superellipse section to put a detail ON the authored skin.
// Flat polygons intersect a curved fairing and produce broken, jagged decals.
// Tính điểm trên mặt bên của ốp để tem bám theo độ cong thân xe.
V skinSide(const Stations &stations, double power, double x, double y, int sign,
           double clearance = .002) {
    size_t i = 0;
    while (i + 2 < stations.size() && stations[i + 1].x < x)
        ++i;
    double t = (x - stations[i].x) / (stations[i + 1].x - stations[i].x);
    double lo = interpolate(stations, i, t, &Station::low),
           hi = interpolate(stations, i, t, &Station::high);
    double w = interpolate(stations, i, t, &Station::halfWidth);
    double cosine = pow(min(1., abs((2 * y - hi - lo) / (hi - lo))), 1. / power);
    double z = w * pow(max(0., 1 - cosine * cosine), power / 2);
    return {x, y, sign * (z + clearance)};
}
// Subdivide a decal in the side-view plane BEFORE mapping it onto the fairing.
// Subdivision prevents the interior of each triangle cutting through the body.
// Tạo các mặt tam giác của tem, áp từng điểm lên bề mặt ốp.
void bodyDecal(Builder &b, Finish finish, const Stations &stations, double power, int sign,
               const vector<V> &outline) {
    V centre{};
    for (auto p : outline)
        centre = centre + p;
    centre = centre * (1. / outline.size());
    auto mapped = [&](V p) { return skinSide(stations, power, p.x, p.y, sign); };
    int n = b.distant() ? 5 : 14;
    for (size_t k = 0; k < outline.size(); ++k) {
        V a = outline[k], c = outline[(k + 1) % outline.size()];
        auto point = [&](int i, int j) {
            return mapped(centre + (a - centre) * (double(i) / n) + (c - centre) * (double(j) / n));
        };
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n - i; ++j) {
                b.tri(finish, point(i, j), point(i + 1, j), point(i, j + 1), {0, 0, double(sign)});
                if (i + j + 1 < n)
                    b.tri(finish, point(i + 1, j), point(i + 1, j + 1), point(i, j + 1),
                          {0, 0, double(sign)});
            }
    }
}
// Dựng giảm xóc gồm ty, vỏ và lò xo xoắn.
void shock(Builder &b, V a, V end) {
    b.tube(Finish::Bright, a, end, .014);
    if (b.distant()) {
        b.tube(Finish::DarkMetal, a + (end - a) * .18, a + (end - a) * .88, .022);
        return;
    }
    V dir = unit(end - a), e = unit(cross(dir, {0, 0, 1})), n = cross(dir, e);
    V last{};
    for (int i = 0; i <= 72; i++) {
        double u = i / 72., t = 2 * PI * 8 * u;
        V p = a + (end - a) * (.15 + .73 * u) + (e * cos(t) + n * sin(t)) * .021;
        if (i)
            b.tube(Finish::DarkMetal, last, p, .004, 0, 6);
        last = p;
    }
}

// A section across the leg shield at a given elevation (rear skin, nose and half-width).
// Reuse the station interpolator vertically: x = height, low/high = back/nose.
using Shield = Stations;
// Tính điểm trên tiết diện yếm theo độ cao và góc quanh yếm.
V shieldPoint(const Shield &rows, double u, double theta) {
    double t = u * (rows.size() - 1);
    int i = min(int(rows.size()) - 2, int(t));
    t -= i;
    double back = interpolate(rows, i, t, &Station::low),
           nose = interpolate(rows, i, t, &Station::high);
    double width = interpolate(rows, i, t, &Station::halfWidth);
    return {(back + nose) * .5 + (nose - back) * .5 * signedPower(cos(theta), .80),
            rows[i].x * (1 - t) + rows[i + 1].x * t, width * signedPower(sin(theta), .80)};
}
// Dựng yếm trước, đường viền và đèn xi nhan gắn trên yếm.
void legShield(Builder &b, const Shield &rows, bool indicators) {
    for (int k = 0; k < 4; k++)
        b.grid(k == 0 || k == 3 ? Finish::Paint : Finish::Graphite, (rows.size() - 1) * 5, 16,
               [&](double u, double v) { return shieldPoint(rows, u, (k + v) * PI / 2); });
    for (int end : {0, 1})
        for (int k = 0; k < 32; k++) {
            V a = shieldPoint(rows, end, k * PI / 16),
              c = shieldPoint(rows, end, (k + 1) * PI / 16);
            auto r = end ? rows.back() : rows.front();
            b.tri(Finish::Graphite, {(r.high + r.low) / 2, r.x, 0}, a, c, {0, end ? 1. : -1., 0});
        }
    if (indicators)
        for (int sign : {-1, 1}) {
            // Map every lens vertex onto the same curved apron. An inset scaled in
            // Cartesian space can otherwise disappear inside the convex panel.
            auto strip = [&](Finish finish, double lo, double hi, double width, double offset) {
                b.grid(
                    finish, 22, 8,
                    [&](double u, double v) {
                        double angle = .29 + .66 * u + width * (2 * v - 1) * (.17 + sin(PI * u));
                        V p = shieldPoint(rows, lo + (hi - lo) * u, sign * angle);
                        return p + V{offset, 0, sign * offset * .60};
                    },
                    sign < 0);
            };
            strip(Finish::Graphite, .47, .91, .29, .004);
            strip(Finish::Bright, .485, .895, .25, .008);
            strip(Finish::Lens, .502, .882, .207, .012);
            strip(Finish::Amber, .675, .729, .103, .017);
        }
    else
        for (int sign : {-1, 1}) {
            // Lower side opening of the underbone's leg shield, with three moulded ribs.
            auto vent = [&](double u, double v) {
                double angle = 1.12 + (.08 + .40 * sin(PI * u)) * (v - .5);
                return shieldPoint(rows, .31 + .24 * u, sign * angle) + V{.0025, 0, sign * .003};
            };
            b.grid(Finish::Graphite, b.distant() ? 10 : 24, 8, vent, sign < 0);
            for (int rib = 1; rib <= 3; ++rib) {
                V previous = vent(.20 * rib, .10);
                for (int k = 1; k <= 8; ++k) {
                    V p = vent(.20 * rib, .10 + .8 * k / 8);
                    b.tube(Finish::Paint, previous, p, .0035, 0, 6);
                    previous = p;
                }
            }
            if (!b.distant())
                for (double u : {.41, .78}) {
                    V p = shieldPoint(rows, u, sign * 1.48);
                    b.tube(Finish::DarkMetal, p, p + V{.001, 0, sign * .004}, .0045, 0, 8);
                }
        }
}
// Dựng mặt yên cong, đường may và lưu các điểm điều khiển để quan sát.
void stitchedSeat(Builder &b, const Stations &steps) {
    Stations base = steps;
    for (auto &st : base) {
        double bottom = st.low;
        st.high = bottom + .013;
        st.low = bottom - .075;
        st.halfWidth *= .94;
    }
    b.skin(Finish::Graphite, base, .54);
    b.skin(Finish::Seat, steps, .63, 0, true);
    if (b.distant())
        return;
    for (int sign : {-1, 1}) {
        V previous{};
        int count = (int(steps.size()) - 1) * 6;
        for (int k = 0; k <= count; k++) {
            int i = min(int(steps.size()) - 2, k / 6);
            double t = k / 6. - i;
            double low = interpolate(steps, i, t, &Station::low),
                   hi = interpolate(steps, i, t, &Station::high);
            V p{steps[i].x * (1 - t) + steps[i + 1].x * t, low + .64 * (hi - low),
                sign * interpolate(steps, i, t, &Station::halfWidth) * .98};
            if (k)
                b.tube(Finish::Graphite, previous, p, .0017, 0, 6);
            previous = p;
        }
    }
}
// Dựng đuôi xe, cụm đèn hậu và các chi tiết phía sau.
void tail(Builder &b, const BikeSpec &s, bool wave) {
    // Tire is the foremost point at straight ahead. The lower rear mud flap
    // fixes nominal overall length instead of stretching an entire bike frame.
    double xEnd = axle(s, true).x + s.front.radius() - s.length;
    double x = xEnd + (wave ? .13 : .095), y = wave ? .757 : .768;
    b.skin(Finish::Graphite,
           {{xEnd, .377, .395, .045},
            {xEnd + .035, .383, .485, .07},
            {x + .05, .60, y, .085},
            {x + .13, .65, y + .015, .045}},
           .6);
    b.panel(Finish::Reflector,
            {{xEnd + .009, .421, -.086},
             {xEnd + .009, .421, .086},
             {xEnd + .029, .490, .086},
             {xEnd + .029, .490, -.086}},
            {-1, .28, 0}, .002);
    b.optical({{x - .012, y + .036, -.108},
               {x - .025, y + .032, 0},
               {x - .012, y + .036, .108},
               {x + .018, y - .031, .087},
               {x - .008, y - .042, 0},
               {x + .018, y - .031, -.087}},
              {-1, .16, 0}, Finish::Tail, .010);
    for (int sign : {-1, 1}) {
        b.tube(Finish::Alloy, {x + .27, y + .038, sign * .124}, {x + .089, y + .099, sign * .132}, .011);
        b.tube(Finish::Alloy, {x + .089, y + .099, sign * .132}, {x + .045, y + .11, sign * .08}, .011);
    }
    b.tube(Finish::Alloy, {x + .045, y + .11, -.08}, {x + .045, y + .11, .08}, .011);
}
// Dựng cổ pô, ống xả và tấm chắn nhiệt theo mẫu xe.
void exhaust(Builder &b, bool wave) {
    double z = wave ? .134 : .172;
    vector<V> header = {
        {.09, .273, .045}, {.14, .205, .083}, {.10, .158, z}, {-.11, .168, z}, {-.30, .221, z}};
    for (size_t i = 1; i < header.size(); i++)
        b.tube(Finish::DarkMetal, header[i - 1], header[i], .016);
    Stations pipe = {{-.88, .344, .420, .025},
                     {-.83, .310, .438, .050},
                     {-.52, .237, .360, .063},
                     {-.28, .197, .279, .043},
                     {-.23, .205, .24, .020}};
    b.skin(Finish::DarkMetal, pipe, .65, z);
    b.skin(wave ? Finish::Bright : Finish::Graphite,
           {{-.86, .365, .417, .009},
            {-.78, .337, .427, .019},
            {-.53, .273, .370, .022},
            {-.34, .242, .300, .016},
            {-.32, .253, .269, .005}},
           .55, z + .047);
    for (V p : {V{-.77, .379, z + .066}, V{-.43, .295, z + .067}})
        b.tube(Finish::Alloy, p, p + V{0, 0, .004}, .006, 0, 6);
    b.tube(Finish::Rubber, {-.877, .389, z}, {-.886, .39, z}, .017, 0, 16);
}
// The badge follows the outer skin rather than a flat sign next to the bike.
// Dựng chữ tên mẫu xe trên mặt bên của ốp.
void sideBadge(Builder &b, const Stations &stations, double power, bool wave) {
    if (b.distant())
        return;
    const string word = wave ? "WAVE" : "VISION";
    auto glyph = [](char c) -> vector<vector<V>> {
        switch (c) {
        case 'W':
            return {{{0, 1}, {.12, 0}, {.5, .53}, {.73, 0}, {1, 1}}};
        case 'A':
            return {{{0, 0}, {.46, 1}, {1, 0}}, {{.2, .4}, {.75, .4}}};
        case 'V':
            return {{{0, 1}, {.52, 0}, {1, 1}}};
        case 'E':
            return {{{1, 1}, {0, 1}, {0, 0}, {1, 0}}, {{0, .52}, {.76, .52}}};
        case 'I':
            return {{{.48, 0}, {.48, 1}}};
        case 'S':
            return {{{1, .90},
                     {.75, 1},
                     {.13, 1},
                     {0, .72},
                     {.15, .56},
                     {.84, .44},
                     {1, .28},
                     {.85, 0},
                     {.15, 0},
                     {0, .10}}};
        case 'O':
            return {{{.20, 1},
                     {.80, 1},
                     {1, .78},
                     {1, .20},
                     {.80, 0},
                     {.20, 0},
                     {0, .20},
                     {0, .78},
                     {.20, 1}}};
        default:
            return {{{0, 0}, {0, 1}, {1, 0}, {1, 1}}};
        }
    };
    for (int sign : {-1, 1}) {
        auto point = [&](size_t letter, V g) {
            const double advance = wave ? .039 : .028, baseX = wave ? -.745 : -.735;
            double distance = advance * (letter + .77 * g.x + .14 * g.y);
            double x = baseX + (sign > 0 ? distance : advance * word.size() - distance);
            double y = (wave ? .755 : .724) - .23 * (x - baseX) + .017 * g.y;
            return skinSide(stations, power, x, y, sign, .003);
        };
        for (size_t i = 0; i < word.size(); ++i)
            for (const auto &stroke : glyph(word[i]))
                for (size_t k = 1; k < stroke.size(); ++k)
                    b.tube(wave ? Finish::Decal : Finish::Bright, point(i, stroke[k - 1]),
                           point(i, stroke[k]), .0015, 0, 6);
    }
}
// Dựng mặt tì chân có bề dày và các gờ chống trượt.
void pedalPad(Builder &b, V center, double halfLength, double halfWidth, Finish finish) {
    const vector<V> outline{
        center + V{-halfLength, 0, -halfWidth}, center + V{halfLength, 0, -halfWidth},
        center + V{halfLength, 0, halfWidth}, center + V{-halfLength, 0, halfWidth}};
    b.panel(finish, outline, {0, 1, 0}, .006);
    b.panel(finish, outline, {0, -1, 0}, .004);
    for (int row : {-1, 0, 1}) {
        const V ridge = center + V{row * halfLength * .46, .006, 0};
        b.tube(finish, ridge + V{0, 0, -halfWidth * .68},
               ridge + V{0, 0, halfWidth * .68}, .002, 0, 6);
    }
}

// Dựng cần số hai đầu bên trái và bàn đạp phanh sau bên phải của Wave.
void waveFootControls(Builder &b) {
    const V shiftPivot{-.115, .260, -.124};
    const V shiftHub{-.115, .260, -.181};
    const V toePad{.055, .259, -.205};
    const V heelPad{-.246, .265, -.205};
    b.tube(Finish::DarkMetal, shiftPivot, shiftHub, .013);
    b.tube(Finish::Alloy, shiftHub, toePad, .009);
    b.tube(Finish::Alloy, shiftHub, heelPad, .009);
    pedalPad(b, toePad, .019, .023, Finish::Rubber);
    pedalPad(b, heelPad, .024, .024, Finish::Rubber);

    const V brakePivot{-.218, .198, .091};
    const V brakeElbow{-.193, .171, .145};
    const V brakeRise{.068, .179, .156};
    const V brakePad{.114, .229, .180};
    b.tube(Finish::DarkMetal, {-.125, .230, .070}, brakePivot, .013);
    b.tube(Finish::DarkMetal, brakePivot, brakeElbow, .013);
    b.tube(Finish::Alloy, brakeElbow, brakeRise, .009);
    b.tube(Finish::Alloy, brakeRise, brakePad, .009);
    pedalPad(b, brakePad, .028, .027, Finish::Alloy);
}

// Dựng chống nghiêng và chống đứng ở tư thế gập sát gầm khi xe chạy.
void foldedStands(Builder &b, double pivotX, double pivotY) {
    const V sidePivot{pivotX + .08, pivotY, -.135};
    const V sideBend = sidePivot + V{-.132, -.045, -.024};
    const V sideFoot = sideBend + V{-.095, .005, -.016};
    b.tube(Finish::DarkMetal, sidePivot + V{.025, .108, .069},
           sidePivot + V{0, 0, .048}, .016);
    b.tube(Finish::DarkMetal, sidePivot + V{0, 0, .060}, sidePivot, .015);
    b.tube(Finish::DarkMetal, sidePivot, sideBend, .012);
    b.tube(Finish::DarkMetal, sideBend, sideFoot, .011);
    b.tube(Finish::DarkMetal, sideFoot + V{0, 0, -.018},
           sideFoot + V{0, 0, .018}, .013);
    b.tube(Finish::Alloy, sidePivot + V{-.014, .022, .010},
           sideBend + V{.027, .018, .012}, .005);

    b.tube(Finish::DarkMetal, {pivotX, pivotY, -.118}, {pivotX, pivotY, .118}, .013);
    for (int side : {-1, 1}) {
        const V legPivot{pivotX, pivotY, side * .108};
        const V bend = legPivot + V{-.105, -.047, 0};
        const V foot = bend + V{-.088, .006, 0};
        b.tube(Finish::DarkMetal, legPivot + V{.036, .116, side * -.054},
               legPivot, .016);
        b.tube(Finish::DarkMetal, legPivot, bend, .013);
        b.tube(Finish::DarkMetal, bend, foot, .012);
        b.tube(Finish::DarkMetal, foot + V{0, 0, -.017}, foot + V{0, 0, .017}, .014);
    }
    b.tube(Finish::DarkMetal, {pivotX - .145, pivotY - .044, -.108},
           {pivotX - .145, pivotY - .044, .108}, .011);
    b.tube(Finish::DarkMetal, {pivotX - .14, pivotY - .044, -.108},
           {pivotX - .168, pivotY - .020, -.220}, .010);
}

} // namespace

// Lắp thân Wave từ các tiết diện ốp, yên, yếm và cụm động cơ.
void waveBody(Builder &b, const BikeSpec &s) {
    tail(b, s, true);
    // Black step-through frame, narrow upper side covers and horizontal cylinder.
    const Stations frameSections = {
        {-.89, .715, .776, .052},
        {-.71, .655, .801, .112},
        {-.46, .516, .765, .133},
        {-.27, .445, .712, .124},
        {-.10, .39, .582, .094},
        {.04, .39, .49, .066},
        {.28, .50, .61, .045}
    };
    b.skin(Finish::Graphite, frameSections, .65);
    const Stations sideCover = {
        {-.895, .746, .794, .062},
        {-.78, .706, .812, .126},
        {-.64, .660, .793, .145},
        {-.42, .558, .755, .141},
        {-.26, .530, .706, .126},
        {-.09, .493, .571, .092},
        {-.048, .505, .523, .062}
    };
    b.skin(Finish::Paint, sideCover, .60);
    sideBadge(b, sideCover, .60, true);
    const Stations seatSections = {
        {-.846, .794, .801, .055},
        {-.78, .792, .832, .118},
        {-.63, .770, .821, .146},
        {-.40, .725, .791, .145},
        {-.215, .689, s.seatHeight, .124},
        {-.108, .676, .752, .099},
        {-.078, .682, .713, .051}
    };
    stitchedSeat(b, seatSections);
    const Shield legShieldSections = {
        {.184, .183, .224, .055},
        {.308, .202, .300, .083},
        {.480, .226, .393, .133},
        {.664, .289, .511, .152},
        {.802, .331, .564, .155},
        {.909, .315, .476, .102},
        {.944, .304, .411, .055}
    };
    legShield(b, legShieldSections, false);
    const Stations engineCoverSections = {
        {-.22, .29, .414, .075},
        {-.04, .287, .438, .088},
        {.148, .319, .404, .062},
        {.241, .196, .311, .064}
    };
    b.skin(Finish::Graphite, engineCoverSections, .7);
    // Lốc máy, xi lanh có cánh tản nhiệt và gác chân.
    b.tube(Finish::DarkMetal, {-.044, .299, -.115}, {-.044, .299, .110}, .091, 0, 32);
    b.tube(Finish::Alloy, {-.044, .299, .109}, {-.044, .299, .119}, .075, 0, 32);
    for (int i = 0; i < 6; i++) {
        double a = i * PI / 3;
        b.tube(Finish::Bright, {-.044 + .078 * cos(a), .299 + .078 * sin(a), .107},
               {-.044 + .078 * cos(a), .299 + .078 * sin(a), .117}, .005, 0, 6);
    }
    for (int k = 0; k < 7; k++)
        b.tube(Finish::Alloy, {.055 + k * .014, .300 + k * .002, 0},
               {.060 + k * .014, .301 + k * .002, 0}, .060, 0, 12);
    b.tube(Finish::DarkMetal, {-.10, .218, -.203}, {-.10, .218, .203}, .012);
    for (int sign : {-1, 1}) {
        b.tube(Finish::Rubber, {-.10, .219, sign * .118}, {-.10, .219, sign * .202}, .019);
        b.tube(Finish::Alloy, {-.45, .315, sign * .074}, {-.425, .380, sign * .152}, .012);
        b.tube(Finish::Rubber, {-.425, .38, sign * .15}, {-.425, .38, sign * .222}, .013);
        b.tube(Finish::DarkMetal, {-.625, .29, sign * .059}, {-.14, .281, sign * .060}, .020);
        shock(b, {-.588, .307, sign * .076}, {-.487, .585, sign * .088});
        // Tapered printed graphics conform to the side panel, including their interior.
        bodyDecal(b, Finish::Decal, sideCover, .60, sign,
                  {{-.748, .752}, {-.665, .740}, {-.216, .559}, {-.477, .637}});
        bodyDecal(b, Finish::Accent, sideCover, .60, sign,
                  {{-.700, .778}, {-.641, .757}, {-.300, .611}, {-.554, .703}});
    }
    const Stations chainGuardSections = {
        {-.671, .243, .316, .017},
        {-.609, .235, .336, .027},
        {-.27, .247, .307, .024},
        {-.166, .26, .30, .015}
    };
    b.skin(Finish::DarkMetal, chainGuardSections, .60, -.079);
    // Right crank cover has two cast lobes; boltheads and kick lever sit proud.
    b.tube(Finish::DarkMetal, {-.094, .286, .103}, {-.094, .286, .119}, .047, 0, 24);
    b.tube(Finish::Alloy, {-.043, .298, .117}, {-.043, .298, .123}, .050, 0, 28);
    b.tube(Finish::Bright, {-.109, .286, .125}, {-.158, .396, .138}, .008);
    b.tube(Finish::Rubber, {-.158, .396, .132}, {-.158, .396, .194}, .010);
    waveFootControls(b);
    foldedStands(b, -.282, .175);
    exhaust(b, true);
}
// Lắp thân Vision từ các tiết diện ốp, yên, sàn để chân và hộp truyền động.
void visionBody(Builder &b, const BikeSpec &s) {
    tail(b, s, false);
    const Stations frameSections = {
        {-.89, .660, .770, .057},
        {-.76, .547, .802, .146},
        {-.55, .461, .79, .169},
        {-.34, .341, .717, .167},
        {-.18, .265, .655, .145},
        {-.085, .243, .500, .118},
        {-.047, .251, .310, .107}
    };
    b.skin(Finish::Graphite, frameSections, .57);
    const Stations sideCover = {
        {-.9, .721, .789, .058},
        {-.79, .686, .807, .144},
        {-.59, .594, .788, .169},
        {-.407, .450, .735, .188},
        {-.296, .397, .641, .185},
        {-.240, .395, .540, .181},
        {-.224, .425, .490, .177}
    };
    b.skin(Finish::Paint, sideCover, .56);
    sideBadge(b, sideCover, .56, false);
    const Stations seatSections = {
        {-.827, .786, .804, .060},
        {-.768, .775, .850, .130},
        {-.61, .753, .832, .169},
        {-.44, .719, .799, .165},
        {-.26, .682, s.seatHeight, .139},
        {-.165, .674, .76, .106},
        {-.123, .686, .727, .073}
    };
    stitchedSeat(b, seatSections);
    // Lower body sweeps under a continuous treaded footboard.
    const Stations lowerFairingSections = {
        {-.40, .215, .384, .111},
        {-.27, .177, .321, .157},
        {-.05, .152, .275, .168},
        {.173, .156, .276, .17},
        {.310, .182, .326, .157},
        {.366, .211, .382, .107}
    };
    b.skin(Finish::Paint, lowerFairingSections, .46);
    const Stations footboardSections = {
        {-.29, .282, .306, .11},
        {-.205, .253, .286, .150},
        {-.06, .247, .269, .162},
        {.125, .252, .273, .158},
        {.249, .275, .307, .148},
        {.308, .304, .344, .124}
    };
    b.skin(Finish::Rubber, footboardSections, .40);
    for (int sign : {-1, 1})
        for (int k = 0; k < 4; k++) {
            double z = sign * (.046 + k * .025);
            b.tube(Finish::Graphite, {-.14, .275, z}, {.135, .276, z}, .003, 0, 6);
        }
    const Shield legShieldSections = {
        {.212, .239, .355, .125},
        {.346, .238, .426, .179},
        {.515, .270, .483, .166},
        {.655, .291, .568, .177},
        {.749, .317, .614, .178},
        {.869, .316, .573, .155},
        {.934, .312, .479, .092},
        {.950, .306, .409, .056}
    };
    legShield(b, legShieldSections, true);
    // CVT casing and airbox on the left, silencer on the right; single rear shock.
    const Stations transmissionSections = {
        {-.77, .208, .319, .027},
        {-.713, .184, .365, .055},
        {-.57, .176, .362, .068},
        {-.40, .185, .326, .06},
        {-.235, .191, .35, .059},
        {-.168, .22, .322, .025}
    };
    b.skin(Finish::DarkMetal, transmissionSections, .65, -.083);
    const Stations airboxSections = {
        {-.76, .345, .388, .025},
        {-.66, .339, .448, .049},
        {-.45, .35, .452, .054},
        {-.31, .36, .410, .025}
    };
    b.skin(Finish::Graphite, airboxSections, .48, -.087);
    b.tube(Finish::Alloy, {-.626, .256, -.126}, {-.626, .256, -.151}, .036, 0, 24);
    shock(b, {-.611, .264, -.064}, {-.479, .587, -.098});
    for (int sign : {-1, 1}) {
        b.tube(Finish::Alloy, {-.447, .349, sign * .100}, {-.363, .330, sign * .174}, .013);
        // Small crease below the separate chrome nameplate; not a floating flat plate.
        bodyDecal(b, Finish::Graphite, sideCover, .56, sign,
                  {{-.756, .718}, {-.554, .660}, {-.380, .534}, {-.574, .644}});
    }
    foldedStands(b, -.246, .163);
    exhaust(b, false);
}

} // namespace bike_geometry
