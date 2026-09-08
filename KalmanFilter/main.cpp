#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <raylib.h>

class ExtendedKalmanFilter {
private:
    Eigen::VectorXd x_hat; // État : [X, Y, Theta]
    Eigen::MatrixXd P, Q, R, I, H;

public:
    ExtendedKalmanFilter() {
        x_hat = Eigen::VectorXd::Zero(3);
        P = Eigen::MatrixXd::Identity(3, 3);
        Q = Eigen::MatrixXd::Identity(3, 3) * 0.1; // Bruit du modèle
        R = Eigen::MatrixXd::Identity(2, 2) * 2.0; // Bruit du capteur GPS (X, Y)
        H = Eigen::MatrixXd::Zero(2, 3);
        H << 1, 0, 0,  // Le capteur lit X
             0, 1, 0;  // Le capteur lit Y
        I = Eigen::MatrixXd::Identity(3, 3);
    }

    void setInitialState(double x, double y, double theta) {
        x_hat << x, y, theta;
    }

    // ==========================================
    // PHASE A : PREDICTION NON-LINÉAIRE (EKF)
    // ==========================================
    void predict(double v, double w, double dt) {
        double theta = x_hat(2);

        // 1. Prédiction de l'état (Physique non-linéaire pure)
        x_hat(0) = x_hat(0) + v * cos(theta) * dt;
        x_hat(1) = x_hat(1) + v * sin(theta) * dt;
        x_hat(2) = x_hat(2) + w * dt;

        // 2. Calcul dynamique de la Jacobienne J_F
        Eigen::MatrixXd J_F(3, 3);
        J_F << 1, 0, -v * sin(theta) * dt,
               0, 1,  v * cos(theta) * dt,
               0, 0,  1;

        // 3. Prédiction de la covariance
        P = J_F * P * J_F.transpose() + Q;
    }

    // ==========================================
    // PHASE B : MISE À JOUR 
    // ==========================================
    void update(const Eigen::VectorXd& z) {
        // Comme notre capteur GPS est linéaire (il lit juste X et Y), H reste une matrice normale
        Eigen::VectorXd y = z - H * x_hat;
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();
        
        x_hat = x_hat + K * y;
        P = (I - K * H) * P;
    }

    // Accesseurs
    double getX() { return x_hat(0); }
    double getY() { return x_hat(1); }
    double getTheta() { return x_hat(2); }
};

int main() {
    // 1. Paramétrage
    ExtendedKalmanFilter ekf;
    ekf.setInitialState(4.0, 2.0, 0.0); // Départ au centre (4m, 2m)
    
    double dt = 1.0 / 60.0;
    
    // Commandes motrices (Le robot va tourner en rond)
    double v = 2.0;    // Vitesse linéaire : 2 m/s
    double w = 1.0;    // Vitesse angulaire : 1 rad/s

    // 2. Initialisation Fenêtre 2D
    const int screenWidth = 800;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "Portfolio R&D - EKF 2D (Unicycle)");
    SetTargetFPS(60);

    Eigen::VectorXd z(2); // Mesure du capteur [X, Y]

    // 3. Boucle temps réel
    while (!WindowShouldClose()) {
        // --- SIMULATION DU CAPTEUR BRUITÉ ---
        // On génère une vérité terrain (le modèle parfait) pour créer la mesure du capteur
        static double vrai_x = 4.0, vrai_y = 2.0, vrai_theta = 0.0;
        vrai_x += v * cos(vrai_theta) * dt;
        vrai_y += v * sin(vrai_theta) * dt;
        vrai_theta += w * dt;

        // Le GPS lit la position avec une erreur aléatoire (bruit)
        z(0) = vrai_x + (GetRandomValue(-50, 50) / 100.0);
        z(1) = vrai_y + (GetRandomValue(-50, 50) / 100.0);

        // --- ALGORITHME EKF ---
        ekf.predict(v, w, dt);
        ekf.update(z);

        // --- RENDU GRAPHIQUE ---
        BeginDrawing();
            ClearBackground(RAYWHITE);

            // Conversion Mètres -> Pixels (Échelle : 1m = 100px)
            int gps_px_x = (int)(z(0) * 100);
            int gps_px_y = (int)(z(1) * 100);
            
            int ekf_px_x = (int)(ekf.getX() * 100);
            int ekf_px_y = (int)(ekf.getY() * 100);

            // Dessin du point GPS (rouge clignotant/chaotique)
            DrawCircle(gps_px_x, gps_px_y, 4, RED);

            // Dessin du Robot EKF (cercle bleu avec ligne de direction)
            DrawCircle(ekf_px_x, ekf_px_y, 15, Fade(BLUE, 0.8f));
            DrawLineEx(
                {(float)ekf_px_x, (float)ekf_px_y}, 
                {(float)(ekf_px_x + 25 * cos(ekf.getTheta())), (float)(ekf_px_y + 25 * sin(ekf.getTheta()))}, 
                3, DARKBLUE
            );

            // HUD Technique
            DrawText("Filtre de Kalman Etendu (EKF) - Jacobienne Dynamique", 10, 10, 20, DARKGRAY);
            DrawText(TextFormat("Commandes: v=%.1fm/s, w=%.1frad/s", v, w), 10, 40, 20, DARKGREEN);
            DrawText("Point Rouge : Mesure GPS bruitee", 10, 70, 20, RED);
            DrawText("Cercle Bleu : Estimation EKF lisse", 10, 100, 20, BLUE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}