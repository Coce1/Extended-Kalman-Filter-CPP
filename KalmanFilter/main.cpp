// EKF 2D (modele unicycle) : fusion GPS (mesure lineaire) + distance a une balise (mesure NON lineaire).
// Version corrigee : bruit gaussien, R/Q coherents avec les vrais bruits, odometrie bruitee dans la verite
// terrain, pas de temps fixe, forme de Joseph, metriques RMSE et NIS.
#include <algorithm>
#include <cmath>
#include <deque>
#include <random>

#include <Eigen/Dense>
#include <raylib.h>

#include "ekf.hpp"

namespace cfg {
    constexpr int    SCREEN_W = 800;
    constexpr int    SCREEN_H = 600;
    constexpr double PX_PER_M = 100.0;

    constexpr double DT = 1.0 / 60.0;
    constexpr int    GPS_EVERY = 6;      // 10 Hz
    constexpr int    RANGE_EVERY = 6;    // 10 Hz, decale de 3 pas par rapport au GPS
    constexpr int    RANGE_OFFSET = 3;

    constexpr double CMD_V = 2.0;        // m/s
    constexpr double CMD_W = 1.0;        // rad/s  -> cercle de rayon 2 m

    constexpr double SIGMA_GPS = 0.30;   // m
    constexpr double SIGMA_RANGE = 0.10; // m
    constexpr double SIGMA_V = 0.10;     // m/s (bruit d'odometrie reel ET modelise dans Q)
    constexpr double SIGMA_W = 0.03;     // rad/s

    constexpr size_t TRAIL_MAX = 600;
} // namespace cfg

using Vec2 = Eigen::Vector2d;

struct Truth { double x, y, theta; };

class Simulation {
public:
    Simulation() : ekf_(cfg::SIGMA_V, cfg::SIGMA_W) { reset(); }

    void reset() {
        truth_ = { 4.0, 1.5, 0.0 };   // le cercle de rayon 2 m reste dans la fenetre (8 m x 6 m)

        // Etat initial volontairement decale pour montrer la convergence du filtre
        ekf::ExtendedKalmanFilter::Vec3 x0(4.5, 1.0, 0.3);
        Eigen::Matrix3d P0 = Eigen::Vector3d(1.0, 1.0, 0.5).asDiagonal();
        ekf_.reset(x0, P0);

        lastGps_ = Vec2(truth_.x, truth_.y);
        tick_ = 0;
        sumSqEkf_ = sumSqGps_ = sumNisGps_ = sumNisRange_ = 0.0;
        nPos_ = nGps_ = nRange_ = 0;
        trailTruth_.clear();
        trailEkf_.clear();
        rng_.seed(42);   // reproductible
    }

    void step() {
        const double v = cfg::CMD_V, w = cfg::CMD_W;

        // Verite terrain : l'odometrie reelle est bruitee (le filtre ne le sait que via Q)
        const double vTrue = v + cfg::SIGMA_V * n01_(rng_);
        const double wTrue = w + cfg::SIGMA_W * n01_(rng_);
        truth_.x += vTrue * std::cos(truth_.theta) * cfg::DT;
        truth_.y += vTrue * std::sin(truth_.theta) * cfg::DT;
        truth_.theta = ekf::wrapAngle(truth_.theta + wTrue * cfg::DT);

        // Prediction EKF avec les commandes
        ekf_.predict(v, w, cfg::DT);

        // GPS (mesure lineaire)
        if (tick_ % cfg::GPS_EVERY == 0) {
            lastGps_ = Vec2(truth_.x + cfg::SIGMA_GPS * n01_(rng_),
                truth_.y + cfg::SIGMA_GPS * n01_(rng_));
            sumNisGps_ += ekf_.updatePosition(lastGps_, cfg::SIGMA_GPS);
            sumSqGps_ += (lastGps_ - Vec2(truth_.x, truth_.y)).squaredNorm();
            ++nGps_;
        }

        // Distance a la balise (mesure non lineaire, Jacobienne H_k recalculee)
        if (tick_ % cfg::RANGE_EVERY == cfg::RANGE_OFFSET) {
            const double trueRange = (Vec2(truth_.x, truth_.y) - beacon_).norm();
            lastRange_ = trueRange + cfg::SIGMA_RANGE * n01_(rng_);
            sumNisRange_ += ekf_.updateRange(beacon_, lastRange_, cfg::SIGMA_RANGE);
            ++nRange_;
        }

        sumSqEkf_ += (Vec2(ekf_.x(), ekf_.y()) - Vec2(truth_.x, truth_.y)).squaredNorm();
        ++nPos_;
        ++tick_;

        if (tick_ % 3 == 0) {
            push(trailTruth_, Vec2(truth_.x, truth_.y));
            push(trailEkf_, Vec2(ekf_.x(), ekf_.y()));
        }
    }

    const Truth& truth() const { return truth_; }
    const ekf::ExtendedKalmanFilter& ekf() const { return ekf_; }
    const Vec2& gps() const { return lastGps_; }
    const Vec2& beacon() const { return beacon_; }
    const std::deque<Vec2>& trailTruth() const { return trailTruth_; }
    const std::deque<Vec2>& trailEkf() const { return trailEkf_; }
    double rmseEkf() const { return nPos_ ? std::sqrt(sumSqEkf_ / nPos_) : 0.0; }
    double rmseGps() const { return nGps_ ? std::sqrt(sumSqGps_ / nGps_) : 0.0; }
    double meanNisGps() const { return nGps_ ? sumNisGps_ / nGps_ : 0.0; }      // attendu ~ 2
    double meanNisRange() const { return nRange_ ? sumNisRange_ / nRange_ : 0.0; } // attendu ~ 1

private:
    static void push(std::deque<Vec2>& t, const Vec2& p) {
        t.push_back(p);
        if (t.size() > cfg::TRAIL_MAX) t.pop_front();
    }

    ekf::ExtendedKalmanFilter ekf_;
    Truth truth_{ 0, 0, 0 };
    Vec2 lastGps_ = Vec2::Zero();
    Vec2 beacon_ = Vec2(0.8, 0.8);
    double lastRange_ = 0.0;

    std::mt19937 rng_{ 42 };
    std::normal_distribution<double> n01_{ 0.0, 1.0 };

    std::deque<Vec2> trailTruth_, trailEkf_;
    int tick_ = 0;
    double sumSqEkf_ = 0, sumSqGps_ = 0, sumNisGps_ = 0, sumNisRange_ = 0;
    long nPos_ = 0, nGps_ = 0, nRange_ = 0;
};

static Vector2 toPx(const Vec2& p) {
    return { float(p.x() * cfg::PX_PER_M), float(p.y() * cfg::PX_PER_M) };
}

static void drawTrail(const std::deque<Vec2>& t, Color c) {
    for (size_t i = 1; i < t.size(); ++i) DrawLineV(toPx(t[i - 1]), toPx(t[i]), c);
}

static void render(const Simulation& sim) {
    ClearBackground(RAYWHITE);

    drawTrail(sim.trailTruth(), Fade(DARKGREEN, 0.6f));
    drawTrail(sim.trailEkf(), Fade(BLUE, 0.6f));

    // Balise et ligne de mesure de distance
    const Vector2 b = toPx(sim.beacon());
    const Vector2 e = toPx(Vec2(sim.ekf().x(), sim.ekf().y()));
    DrawLineV(b, e, Fade(PURPLE, 0.35f));
    DrawPoly(b, 4, 10, 45.0f, PURPLE);

    // GPS brut, verite terrain, estimation EKF
    DrawCircleV(toPx(sim.gps()), 4, RED);
    DrawCircleV(toPx(Vec2(sim.truth().x, sim.truth().y)), 6, DARKGREEN);
    DrawCircleV(e, 15, Fade(BLUE, 0.8f));
    DrawLineEx(e, { e.x + 25.0f * float(std::cos(sim.ekf().theta())),
                   e.y + 25.0f * float(std::sin(sim.ekf().theta())) }, 3, DARKBLUE);

    // HUD (ASCII uniquement)
    DrawText("EKF 2D (unicycle) : GPS lineaire + distance balise non lineaire", 10, 10, 20, DARKGRAY);
    DrawText(TextFormat("Commandes: v=%.1f m/s, w=%.1f rad/s", cfg::CMD_V, cfg::CMD_W), 10, 38, 18, DARKGREEN);
    DrawText("Rouge: GPS | Bleu: EKF | Vert: verite | Violet: balise  (R : reset)", 10, 62, 18, DARKGRAY);
    DrawText(TextFormat("RMSE  EKF: %.3f m   GPS brut: %.3f m", sim.rmseEkf(), sim.rmseGps()), 10, 86, 18, DARKBLUE);
    DrawText(TextFormat("NIS moyen  GPS: %.2f (attendu ~2)   Distance: %.2f (attendu ~1)",
        sim.meanNisGps(), sim.meanNisRange()), 10, 110, 18, DARKPURPLE);
}

int main() {
    InitWindow(cfg::SCREEN_W, cfg::SCREEN_H, "Portfolio R&D - EKF 2D (Unicycle)");
    SetTargetFPS(60);

    Simulation sim;
    double accumulator = 0.0;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) sim.reset();

        // Pas de temps fixe, independant de la cadence d'affichage
        accumulator += std::min(double(GetFrameTime()), 0.1);
        while (accumulator >= cfg::DT) {
            sim.step();
            accumulator -= cfg::DT;
        }

        BeginDrawing();
        render(sim);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}