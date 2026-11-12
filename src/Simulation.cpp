// Routes combine straight segments and cubic Bezier turns. Arc-length tables map meters to
// positions.
#include "Simulation.h"
#include "Matrix.h"
#include <limits>
#include <stdexcept>

using namespace std;

namespace {
constexpr double followingGap = 2.7;          // Khoảng cách tâm khi bám xe (m).
constexpr double minimumCenterGap = 2.25;     // Khoảng cách tâm tối thiểu (m).
constexpr double exitClearance = 3.0;        // Đoạn thoát cần đi hết để trả quyền (m).
constexpr double comfortableBraking = 2.5;    // Gia tốc phanh êm (m/s²).
constexpr double reactionTime = 0.45;        // Thời gian phản ứng khi đèn vàng (s).
constexpr double cruiseSpeed = 7.0;          // Vận tốc trên đoạn đường thẳng (m/s).
constexpr double crossingSpeed = 5.0;
constexpr double turningSpeed = 3.2;
constexpr double stoppedSpeed = 0.15;
constexpr double greenDuration = 18.0;
constexpr double yellowDuration = 3.0;
constexpr double clearanceDuration = 2.0;
}

// Tra vị trí 3D của xe từ quãng đường đã đi dọc tuyến.
V Route::position(double s) const {
    if (s <= 0)
        return points.front();
    if (s >= total)
        return points.back();
    auto it = upper_bound(arc.begin(), arc.end(), s);
    size_t i = it - arc.begin();
    double t = (s - arc[i - 1]) / (arc[i] - arc[i - 1]);
    return points[i - 1] * (1 - t) + points[i] * t;
}
// Tính hướng tiếp tuyến để đặt hướng thân xe trên quỹ đạo.
V Route::tangent(double s) const {
    return unit(position(min(total, s + .04)) - position(max(0., s - .04)));
}
// Tạo 12 tuyến đi và bảng xung đột giữa các tuyến qua ngã tư.
Simulation::Simulation() {
    for (int side = 0; side < 4; ++side)
        for (Turn turn : {Turn::Straight, Turn::Right, Turn::Left}) {
            Route r;
            r.side = side;
            r.turn = turn;
            const Matrix4x4 routeRotation = rotationYMatrix(-side * 90.);
            V p0{2, 0, 9}, p3; // right-hand traffic; approach from south
            if (turn == Turn::Straight)
                p3 = {2, 0, -9};
            else if (turn == Turn::Right)
                p3 = {9, 0, 2};
            else
                p3 = {-9, 0, -2};
            V endDir = turn == Turn::Straight ? V{0, 0, -1} : (turn == Turn::Right ? V{1, 0, 0} : V{-1, 0, 0});
            V p1 = p0 + V{0, 0, -1} * (turn == Turn::Left ? 11. : 5.),
              p2 = p3 - endDir * (turn == Turn::Left ? 11. : 5.);
            auto add = [&](V p) {
                p = transformPoint(routeRotation, p);
                if (!r.points.empty())
                    r.total += length(p - r.points.back());
                r.points.push_back(p);
                r.arc.push_back(r.total);
            };
            for (int i = 0; i <= 100; i++) {
                add({2, 0, 44 - 35. * i / 100});
            }
            r.entry = r.total;
            // Stop bar is 13 m from the junction centre, before the crosswalk.
            // The bike's centre stops 1.05 m before it: its front tire stays behind the bar.
            r.stop = 44. - 13. - 1.05;
            for (int i = 1; i <= 200; i++) {
                add(bezier(p0, p1, p2, p3, i / 200.));
            }
            r.leave = r.total;
            for (int i = 1; i <= 100; i++) {
                add(p3 + endDir * (35. * i / 100));
            }
            routes.push_back(r);
        }
    // Precompute which entire crossing paths overlap. Disjoint streams may use the
    // intersection together. A platoon on the same approach is controlled by headway.
    for (int a = 0; a < 12; ++a)
        for (int b = a + 1; b < 12; ++b) {
            if (routes[a].side == routes[b].side)
                continue;
            bool conflict = false;
            for (double x = routes[a].stop; x < routes[a].leave + exitClearance && !conflict; x += .35)
                for (double y = routes[b].stop; y < routes[b].leave + exitClearance; y += .35)
                    if (length(routes[a].position(x) - routes[b].position(y)) < 3.1) {
                        conflict = true;
                        break;
                    }
            conflicts[a][b] = conflicts[b][a] = conflict;
        }
    reset();
}
// Đưa mô phỏng về trạng thái ban đầu với hạt giống ngẫu nhiên cố định.
void Simulation::reset() {
    bikes.clear();
    time = spawnClock = totalWait = maxWait = 0;
    passed = 0;
    nextId = 1;
    phaseIndex = 0;
    phaseElapsed = 0;
    rng.seed(42);
}
// Trả chỉ số pha đèn hiện tại trong chu kỳ sáu pha.
int Simulation::phase() const { return phaseIndex; }
// Tính số giây còn lại của pha đèn hiện tại.
double Simulation::remaining() const {
    const double duration[] = {greenDuration, yellowDuration, clearanceDuration,
                               greenDuration, yellowDuration, clearanceDuration};
    return max(0., duration[phaseIndex] - phaseElapsed);
}
// Kiểm tra hướng đường đang được đèn xanh.
bool Simulation::green(int side) const { return phase() == ((side % 2) ? 3 : 0); }
// Lấy màu đèn cho một hướng đường.
SignalColor Simulation::signal(int side) const {
    if (green(side))
        return SignalColor::Green;
    if (phaseIndex == ((side % 2) ? 4 : 1))
        return SignalColor::Yellow;
    return SignalColor::Red;
}
// Lấy tên pha đèn để hiển thị trên HUD.
const char *Simulation::phaseName() const {
    const char *names[] = {"BAC-NAM XANH",  "BAC-NAM VANG",  "DO TOAN BO / DON NUT",
                           "DONG-TAY XANH", "DONG-TAY VANG", "DO TOAN BO / DON NUT"};
    return names[phaseIndex];
}
// Đếm số xe còn giữ quyền đi qua ngã tư.
int Simulation::activeReservations() const {
    return count_if(bikes.begin(), bikes.end(), [](const Bike &b) { return b.reserved; });
}
// Đếm xe đang dừng chờ ở một hướng đường.
int Simulation::queueLength(int side) const {
    return count_if(bikes.begin(), bikes.end(), [&](const Bike &b) {
        return routes[b.route].side == side && !b.entered && b.v < stoppedSpeed;
    });
}
// Chuyển pha đèn khi hết giờ và chờ các xe cũ thoát nút nếu cần.
void Simulation::updateSignal(double dt) {
    phaseElapsed += dt;
    if (remaining() > 1e-9)
        return;
    bool clearance = phaseIndex == 2 || phaseIndex == 5;
    // Never start a new green across vehicles still finishing the previous phase.
    if (clearance && activeReservations() > 0)
        return;
    phaseIndex = (phaseIndex + 1) % 6;
    phaseElapsed = 0;
}
// Thử tạo một xe ở đầu tuyến khi có đủ khoảng trống.
void Simulation::spawn() {
    int side = rng() % 4;
    double u = double(rng() % 1000) / 1000;
    Turn turn = u < .55 ? Turn::Straight : (u < .8 ? Turn::Right : Turn::Left);
    int ri = side * 3 + static_cast<int>(turn);
    V p = routes[ri].position(0);
    for (auto &b : bikes)
        if (length(routes[b.route].position(b.s) - p) < 4.5)
            return;
    Bike b;
    b.id = nextId++;
    b.route = ri;
    bikes.push_back(b);
}
// Tiến một bước mô phỏng theo thứ tự đèn, quyền đi, chuyển động và xe ra khỏi đường.
void Simulation::step(double dt) {
    if (!isfinite(dt) || dt <= 0 || dt > .1)
        throw invalid_argument("Simulation requires a positive timestep <= 0.1 s");
    time += dt;
    releaseReservations();
    updateSignal(dt);
    spawnVehicles(dt);
    updateWaitingVehicles();
    grantReservations();
    moveVehicles(dt);
    removeFinishedVehicles();
}

// Trả quyền qua ngã tư khi xe đã đi hết đoạn thoát.
void Simulation::releaseReservations() {
    for (auto &b : bikes)
        if (b.s > routes[b.route].leave + exitClearance)
            b.reserved = false;
}

// Sinh xe theo lưu lượng yêu cầu và khoảng trống đầu đường.
void Simulation::spawnVehicles(double dt) {
    spawnClock += dt;
    double interval = density > 0 ? 1 / density : numeric_limits<double>::infinity();
    while (spawnClock >= interval) {
        spawnClock -= interval;
        spawn();
    }
}

// Cập nhật yêu cầu qua nút và quyết định dừng hoặc tiếp tục khi đèn vàng.
void Simulation::updateWaitingVehicles() {
    for (auto &b : bikes) {
        auto &r = routes[b.route];
        if (b.entered)
            continue;
        if (b.requestSince < 0 && b.s >= r.stop - 6)
            b.requestSince = time;
        if (signal(r.side) == SignalColor::Yellow && b.reserved && !b.yellowCommitted) {
            // Yellow dilemma zone: react for 0.45 s, then brake at 2.5 m/s^2.
            double comfortableStop = reactionTime * b.v + b.v * b.v / (2 * comfortableBraking);
            b.yellowCommitted = b.v > .5 && r.stop - b.s <= comfortableStop;
            if (!b.yellowCommitted)
                b.reserved = false;
        }
        if (signal(r.side) == SignalColor::Red) {
            b.reserved = false;
            b.yellowCommitted = false;
        }
    }
}

// Cấp quyền đi theo thứ tự chờ, kiểm tra xung đột và đường thoát.
void Simulation::grantReservations() {
    vector<Bike *> requests;
    for (auto &b : bikes)
        if (!b.entered && !b.reserved && b.requestSince >= 0 && green(routes[b.route].side))
            requests.push_back(&b);
    sort(requests.begin(), requests.end(), [](const Bike *a, const Bike *b) {
        return a->requestSince == b->requestSince ? a->id < b->id
                                                  : a->requestSince < b->requestSince;
    });
    for (auto *b : requests) {
        const auto &r = routes[b->route];
        bool allowed = true;
        for (const auto &q : bikes) {
            if (q.id == b->id)
                continue;
            // Do not pass an unadmitted leader or reserve an intersecting route.
            if ((!q.entered && !q.reserved && routes[q.route].side == r.side && q.s > b->s) ||
                (q.reserved && routesConflict(b->route, q.route)))
                allowed = false;
            // Keep the junction clear if its receiving lane is backed up.
            if (q.entered && q.v < .3 &&
                length(routes[q.route].position(q.s) - r.position(r.leave + 2)) < followingGap)
                allowed = false;
        }
        if (allowed) {
            b->reserved = true;
            b->yellowCommitted = false;
        }
    }
}

// Tính tốc độ và dịch chuyển từng xe, giữ khoảng cách với xe phía trước.
void Simulation::moveVehicles(double dt) {
    sort(bikes.begin(), bikes.end(), [](const Bike &a, const Bike &b) { return a.s > b.s; });
    for (auto &b : bikes) {
        auto &r = routes[b.route];
        V p = r.position(b.s), dir = r.tangent(b.s);
        double free = 1e9;
        string reason = "CRUISE";
        if (!b.entered && !b.reserved) {
            free = max(0., r.stop - b.s);
            reason = green(r.side)                           ? "YIELD / CONFLICT"
                     : signal(r.side) == SignalColor::Yellow ? "YELLOW / BRAKE"
                                                             : "RED / STOP";
        } else if (b.yellowCommitted) {
            reason = "CLEAR ON YELLOW";
        } else if (b.entered && !green(r.side)) {
            reason = "CLEAR JUNCTION";
        }
        for (auto &q : bikes)
            if (q.id != b.id) {
                const auto &qr = routes[q.route];
                bool sameLane =
                    b.route == q.route || (r.side == qr.side && b.s < r.entry && q.s < qr.entry);
                if (sameLane && q.s > b.s && q.s - b.s - followingGap < free) {
                    free = max(0., q.s - b.s - followingGap);
                    reason = "FOLLOW / QUEUE";
                }
                V d = qr.position(q.s) - p;
                double ahead = dot(d, dir), lateral = length(d - dir * ahead);
                if (ahead > 0 && lateral < 1.5 && ahead - followingGap < free &&
                    dot(dir, qr.tangent(q.s)) > .3) {
                    free = max(0., ahead - followingGap);
                    reason = "FOLLOW / QUEUE";
                }
            }
        double target = min(cruiseSpeed, sqrt(2 * comfortableBraking * free));
        if (b.s > r.entry - exitClearance && b.s < r.leave + exitClearance)
            target = min(target, r.turn == Turn::Straight ? crossingSpeed : turningSpeed);
        double old = b.v;
        b.a = clamp((target - b.v) * 2., -4., 2.);
        b.v = max(0., b.v + b.a * dt);
        double ds = min(b.v * dt, free);
        V candidate = r.position(b.s + ds);
        for (auto &q : bikes)
            if (q.id != b.id && length(candidate - routes[q.route].position(q.s)) < minimumCenterGap) {
                ds = 0;
                b.v = 0;
                reason = "SAFETY GAP";
                break;
            }
        b.v = min(b.v, ds / dt);
        b.a = (b.v - old) / dt;
        b.s += ds;
        if (b.s > r.stop + .00001)
            b.entered = true;
        b.reason = b.v < 6.9 ? reason : "CRUISE";
        if (b.v < stoppedSpeed) {
            b.wait += dt;
            maxWait = max(maxWait, b.wait);
        }
    }
}

// Xóa xe đã ra khỏi đường và cộng thống kê thời gian chờ.
void Simulation::removeFinishedVehicles() {
    bikes.erase(remove_if(bikes.begin(), bikes.end(),
                               [&](const Bike &b) {
                                   if (b.s >= routes[b.route].total) {
                                       passed++;
                                       totalWait += b.wait;
                                       return true;
                                   }
                                   return false;
                               }),
                bikes.end());
}
