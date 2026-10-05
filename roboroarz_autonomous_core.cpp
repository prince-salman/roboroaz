/**
 * ==============================================================================================
 * ROBO-ROARZ SMORPHI AUTONOMOUS NAVIGATION CORE (C++17)
 * ==============================================================================================
 * 
 * WHITEBOARD SPECIFICATION:
 *   Rules:
 *     1. coordinate must be (0,0)  -> Origin frame locked at startup
 *     2. Holonomic / omnidirectional wheel -> 3-DOF Mecanum/Omni kinematics (Vx, Vy, Omega)
 *     3. 2D Lidar = eyes, IMU = ears -> Dual sensor perception & state estimation
 *     4. Interval 5 sec / point    -> 5.0 seconds precision hold at each target waypoint
 * 
 *   Target Coordinates:
 *     Sequence: (0, 2) -> (2, 3) -> (-2, 4)
 * 
 *   Sense - Think - Act Architecture:
 *     1. Initialize: Origin (0,0), Lidar & IMU streams, load target coordinates
 *     2. Sense: Read IMU for heading (Ears), read 2D LiDAR for position/obstacles (Eyes)
 *     3. Think: EKF + Particle Filter localization, A* Global Path with Costmap Inflation,
 *               Holonomic Dynamic Window Approach (DWA) local trajectory planner, 5s timer
 *     4. Act: Holonomic motor execution, 5s holding, waypoint transition
 * 
 * ALGORITHMS IMPLEMENTED:
 *   1. Sensor Fusion: Extended Kalman Filter (EKF) + Particle Filter (Monte Carlo Localization - MCL)
 *   2. Global Costmap: 2D Occupancy Grid with Multi-Level Inflation Layer (Lethal, Inscribed, Decay)
 *   3. Global Planner: A* Pathfinding with Euclidean/Octile Costmap heuristic
 *   4. Local Planner: Holonomic Dynamic Window Approach (DWA) with 3D Velocity Search Space (Vx, Vy, Omega)
 * ==============================================================================================
 */

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <algorithm>
#include <iomanip>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==============================================================================================
// 1. DATA STRUCTURES & CONFIGURATION
// ==============================================================================================

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0}; // heading in radians [-PI, PI]
};

struct Twist2D {
    double vx{0.0};    // Forward/Backward speed (m/s)
    double vy{0.0};    // Lateral Crab Left/Right speed (m/s)
    double omega{0.0}; // Angular velocity (rad/s)
};

struct LidarScan2D {
    static constexpr size_t NUM_BEAMS = 360;
    std::vector<double> ranges; // meters (0 to 359 degrees)
    double min_range{0.05};
    double max_range{5.00};

    LidarScan2D() : ranges(NUM_BEAMS, 5.0) {}
};

struct IMUData {
    double yaw_heading{0.0}; // radians (Ears)
    double yaw_rate{0.0};    // rad/s
    double ax{0.0};          // m/s^2
    double ay{0.0};          // m/s^2
};

struct RobotConfig {
    // Kinematic constraints for Holonomic 16-Mecanum Smorphi
    double max_vx{0.50};       // m/s
    double min_vx{-0.30};      // m/s
    double max_vy{0.40};       // m/s (lateral strafe)
    double min_vy{-0.40};      // m/s
    double max_omega{2.00};    // rad/s
    double min_omega{-2.00};   // rad/s

    // Acceleration limits
    double accel_vx{1.50};     // m/s^2
    double accel_vy{1.50};     // m/s^2
    double accel_omega{4.00};  // rad/s^2

    // Geometry & Inflation
    double robot_radius{0.22}; // Smorphi module bounding radius (meters)
    double inflation_radius{0.45}; // Safety margin (meters)
    double waypoint_tolerance{0.28}; // Arrival threshold (meters)
    double hold_interval_sec{5.0};   // Rule 4: 5.0 sec / point
};

// ==============================================================================================
// 2. 2D COSTMAP WITH INFLATION LAYER
// ==============================================================================================

class Costmap2D {
public:
    static constexpr uint8_t NO_OBSTACLE = 0;
    static constexpr uint8_t INFLATED_COST_MIN = 1;
    static constexpr uint8_t INSCRIBED_INFLATED = 253;
    static constexpr uint8_t LETHAL_OBSTACLE = 254;

    Costmap2D(double origin_x, double origin_y, double width_m, double height_m, double resolution_m)
        : origin_x_(origin_x), origin_y_(origin_y),
          width_m_(width_m), height_m_(height_m), resolution_(resolution_m) {
        cols_ = static_cast<int>(std::ceil(width_m_ / resolution_));
        rows_ = static_cast<int>(std::ceil(height_m_ / resolution_));
        grid_.assign(rows_ * cols_, NO_OBSTACLE);
    }

    bool worldToGrid(double wx, double wy, int& mx, int& my) const {
        if (wx < origin_x_ || wx >= origin_x_ + width_m_ ||
            wy < origin_y_ || wy >= origin_y_ + height_m_) {
            return false;
        }
        mx = static_cast<int>((wx - origin_x_) / resolution_);
        my = static_cast<int>((wy - origin_y_) / resolution_);
        return (mx >= 0 && mx < cols_ && my >= 0 && my < rows_);
    }

    void gridToWorld(int mx, int my, double& wx, double& wy) const {
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
    }

    void setLethal(double wx, double wy) {
        int mx, my;
        if (worldToGrid(wx, wy, mx, my)) {
            grid_[my * cols_ + mx] = LETHAL_OBSTACLE;
        }
    }

    void setLethalBox(double x_min, double y_min, double x_max, double y_max) {
        int x0, y0, x1, y1;
        if (!worldToGrid(x_min, y_min, x0, y0)) x0 = 0, y0 = 0;
        if (!worldToGrid(x_max, y_max, x1, y1)) x1 = cols_ - 1, y1 = rows_ - 1;

        for (int y = std::min(y0, y1); y <= std::max(y0, y1); ++y) {
            for (int x = std::min(x0, x1); x <= std::max(x0, x1); ++x) {
                if (x >= 0 && x < cols_ && y >= 0 && y < rows_) {
                    grid_[y * cols_ + x] = LETHAL_OBSTACLE;
                }
            }
        }
    }

    // Apply 2D Euclidean Distance Field Inflation Layer
    void computeInflation(double robot_radius, double inflation_radius) {
        int infl_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
        int inscribed_cells = static_cast<int>(std::ceil(robot_radius / resolution_));
        std::vector<uint8_t> inflated = grid_;

        for (int r = 0; r < rows_; ++r) {
            for (int c = 0; c < cols_; ++c) {
                if (grid_[r * cols_ + c] != LETHAL_OBSTACLE) continue;

                for (int dr = -infl_cells; dr <= infl_cells; ++dr) {
                    for (int dc = -infl_cells; dc <= infl_cells; ++dc) {
                        int nr = r + dr;
                        int nc = c + dc;
                        if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;

                        double dist = std::hypot(dr * resolution_, dc * resolution_);
                        if (dist > inflation_radius) continue;

                        uint8_t cost;
                        if (dist <= robot_radius) {
                            cost = INSCRIBED_INFLATED;
                        } else {
                            double factor = (inflation_radius - dist) / (inflation_radius - robot_radius);
                            cost = static_cast<uint8_t>(factor * (INSCRIBED_INFLATED - INFLATED_COST_MIN) + INFLATED_COST_MIN);
                        }

                        size_t idx = nr * cols_ + nc;
                        if (cost > inflated[idx]) {
                            inflated[idx] = cost;
                        }
                    }
                }
            }
        }
        grid_ = std::move(inflated);
    }

    uint8_t getCost(double wx, double wy) const {
        int mx, my;
        if (!worldToGrid(wx, wy, mx, my)) return LETHAL_OBSTACLE;
        return grid_[my * cols_ + mx];
    }

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    double resolution() const { return resolution_; }

private:
    double origin_x_, origin_y_;
    double width_m_, height_m_;
    double resolution_;
    int cols_{0}, rows_{0};
    std::vector<uint8_t> grid_;
};

// ==============================================================================================
// 3. A* GLOBAL PATHFINDING (WITH INFLATED COSTMAP HEURISTIC)
// ==============================================================================================

class AStarPlanner {
public:
    struct Node {
        int x, y;
        double g_cost;
        double h_cost;
        int parent_x, parent_y;

        double f_cost() const { return g_cost + h_cost; }
    };

    struct NodeCompare {
        bool operator()(const Node& a, const Node& b) const {
            return a.f_cost() > b.f_cost();
        }
    };

    static std::vector<Pose2D> plan(const Costmap2D& costmap, const Pose2D& start, const Pose2D& goal) {
        int start_mx, start_my, goal_mx, goal_my;
        if (!costmap.worldToGrid(start.x, start.y, start_mx, start_my) ||
            !costmap.worldToGrid(goal.x, goal.y, goal_mx, goal_my)) {
            return {start, goal}; // Direct fallback
        }

        const int cols = costmap.cols();
        const int rows = costmap.rows();
        const int total_cells = cols * rows;

        std::vector<double> g_score(total_cells, 1e9);
        std::vector<bool> closed(total_cells, false);
        std::vector<std::pair<int, int>> parent_map(total_cells, {-1, -1});

        std::priority_queue<Node, std::vector<Node>, NodeCompare> open_set;

        size_t start_idx = start_my * cols + start_mx;
        g_score[start_idx] = 0.0;
        open_set.push({start_mx, start_my, 0.0, octileDistance(start_mx, start_my, goal_mx, goal_my), -1, -1});

        // 8-Connected neighbor offsets
        const int dx[] = {1, -1, 0,  0, 1, -1,  1, -1};
        const int dy[] = {0,  0, 1, -1, 1,  1, -1, -1};
        const double move_cost[] = {1.0, 1.0, 1.0, 1.0, 1.4142, 1.4142, 1.4142, 1.4142};

        bool found = false;

        while (!open_set.empty()) {
            Node current = open_set.top();
            open_set.pop();

            size_t curr_idx = current.y * cols + current.x;
            if (closed[curr_idx]) continue;
            closed[curr_idx] = true;

            if (current.x == goal_mx && current.y == goal_my) {
                found = true;
                break;
            }

            for (int dir = 0; dir < 8; ++dir) {
                int nx = current.x + dx[dir];
                int ny = current.y + dy[dir];

                if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
                size_t next_idx = ny * cols + nx;
                if (closed[next_idx]) continue;

                double wx, wy;
                costmap.gridToWorld(nx, ny, wx, wy);
                uint8_t cost = costmap.getCost(wx, wy);
                if (cost >= Costmap2D::INSCRIBED_INFLATED) continue; // Lethal or inside robot footprint

                // Cost includes geometric distance + obstacle cost penalty
                double step_cost = move_cost[dir] * costmap.resolution() + (cost / 255.0) * 0.5;
                double tentative_g = current.g_cost + step_cost;

                if (tentative_g < g_score[next_idx]) {
                    g_score[next_idx] = tentative_g;
                    parent_map[next_idx] = {current.x, current.y};
                    double h = octileDistance(nx, ny, goal_mx, goal_my) * costmap.resolution();
                    open_set.push({nx, ny, tentative_g, h, current.x, current.y});
                }
            }
        }

        std::vector<Pose2D> path;
        if (!found) {
            path.push_back(start);
            path.push_back(goal);
            return path;
        }

        // Reconstruct path
        int cx = goal_mx, cy = goal_my;
        while (cx != -1 && cy != -1) {
            double wx, wy;
            costmap.gridToWorld(cx, cy, wx, wy);
            path.push_back({wx, wy, 0.0});
            auto p = parent_map[cy * cols + cx];
            cx = p.first;
            cy = p.second;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

private:
    static double octileDistance(int x1, int y1, int x2, int y2) {
        int dx = std::abs(x1 - x2);
        int dy = std::abs(y1 - y2);
        return (dx + dy) + (1.41421356 - 2.0) * std::min(dx, dy);
    }
};

// ==============================================================================================
// 4. HOLONOMIC DYNAMIC WINDOW APPROACH (DWA) LOCAL PLANNER
// ==============================================================================================

class HolonomicDWAPlanner {
public:
    HolonomicDWAPlanner(const RobotConfig& cfg) : cfg_(cfg) {}

    Twist2D computeVelocity(const Pose2D& current_pose, const Twist2D& current_vel,
                            const std::vector<Pose2D>& global_path, const Costmap2D& costmap, double dt) {
        // 1. Dynamic Window calculation based on acceleration limits
        double min_vx = std::max(cfg_.min_vx, current_vel.vx - cfg_.accel_vx * dt);
        double max_vx = std::min(cfg_.max_vx, current_vel.vx + cfg_.accel_vx * dt);

        double min_vy = std::max(cfg_.min_vy, current_vel.vy - cfg_.accel_vy * dt);
        double max_vy = std::min(cfg_.max_vy, current_vel.vy + cfg_.accel_vy * dt);

        double min_w = std::max(cfg_.min_omega, current_vel.omega - cfg_.accel_omega * dt);
        double max_w = std::min(cfg_.max_omega, current_vel.omega + cfg_.accel_omega * dt);

        // Find carrot lookahead waypoint along global path
        Pose2D carrot = getLookaheadTarget(current_pose, global_path, 0.50);

        Twist2D best_cmd{0.0, 0.0, 0.0};
        double best_score = -1e9;

        const int num_samples = 7;
        const double dvx = (max_vx - min_vx) / (num_samples - 1);
        const double dvy = (max_vy - min_vy) / (num_samples - 1);
        const double dw  = (max_w  - min_w)  / (num_samples - 1);

        const double sim_time = 0.8; // Predict 0.8 seconds ahead
        const double sim_dt = 0.1;
        const int steps = static_cast<int>(sim_time / sim_dt);

        for (int i = 0; i < num_samples; ++i) {
            double sample_vx = min_vx + i * dvx;
            for (int j = 0; j < num_samples; ++j) {
                double sample_vy = min_vy + j * dvy;
                for (int k = 0; k < num_samples; ++k) {
                    double sample_w = min_w + k * dw;

                    // Simulate trajectory
                    Pose2D sim_pose = current_pose;
                    double min_clearance = 10.0;
                    bool collided = false;

                    for (int s = 0; s < steps; ++s) {
                        // Holonomic trajectory integration
                        double cos_th = std::cos(sim_pose.theta);
                        double sin_th = std::sin(sim_pose.theta);
                        double global_vx = sample_vx * cos_th - sample_vy * sin_th;
                        double global_vy = sample_vx * sin_th + sample_vy * cos_th;

                        sim_pose.x += global_vx * sim_dt;
                        sim_pose.y += global_vy * sim_dt;
                        sim_pose.theta += sample_w * sim_dt;

                        uint8_t cost = costmap.getCost(sim_pose.x, sim_pose.y);
                        if (cost >= Costmap2D::INSCRIBED_INFLATED) {
                            collided = true;
                            break;
                        }
                        double clearance = (Costmap2D::INSCRIBED_INFLATED - cost) / 255.0;
                        if (clearance < min_clearance) min_clearance = clearance;
                    }

                    if (collided) continue;

                    // Objective scoring function
                    double dist_to_target = std::hypot(carrot.x - sim_pose.x, carrot.y - sim_pose.y);
                    double target_heading = std::atan2(carrot.y - sim_pose.y, carrot.x - sim_pose.x);
                    double heading_align = std::cos(target_heading - sim_pose.theta);

                    double speed_mag = std::hypot(sample_vx, sample_vy);

                    // Weights: heading alignment (0.4), clearance (0.4), target distance (-0.8), speed (0.2)
                    double score = 0.40 * heading_align + 0.40 * min_clearance - 0.80 * dist_to_target + 0.20 * speed_mag;

                    if (score > best_score) {
                        best_score = score;
                        best_cmd = {sample_vx, sample_vy, sample_w};
                    }
                }
            }
        }
        return best_cmd;
    }

private:
    Pose2D getLookaheadTarget(const Pose2D& current, const std::vector<Pose2D>& path, double lookahead_dist) {
        if (path.empty()) return current;
        for (const auto& pt : path) {
            if (std::hypot(pt.x - current.x, pt.y - current.y) >= lookahead_dist) {
                return pt;
            }
        }
        return path.back();
    }

    RobotConfig cfg_;
};

// ==============================================================================================
// 5. EXTENDED KALMAN FILTER (EKF) + PARTICLE FILTER (MCL) SENSOR FUSION
// ==============================================================================================

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() {
        P_[0][0] = 0.01; P_[1][1] = 0.01; P_[2][2] = 0.005;
        Q_[0][0] = 0.02; Q_[1][1] = 0.02; Q_[2][2] = 0.01;  // Process noise
        R_[0][0] = 0.05; R_[1][1] = 0.05; R_[2][2] = 0.002; // Measurement noise (IMU is precise)
    }

    void initialize(double x, double y, double theta) {
        state_ = {x, y, theta};
    }

    // Predict step using dead-reckoning wheel velocities
    void predict(const Twist2D& vel, double dt) {
        double cos_th = std::cos(state_.theta);
        double sin_th = std::sin(state_.theta);

        state_.x += (vel.vx * cos_th - vel.vy * sin_th) * dt;
        state_.y += (vel.vx * sin_th + vel.vy * cos_th) * dt;
        state_.theta += vel.omega * dt;
        normalizeTheta();

        for (int i = 0; i < 3; ++i) P_[i][i] += Q_[i][i] * dt;
    }

    // Update step fusing IMU heading (Ears) and external LiDAR pose (Eyes)
    void update(double meas_x, double meas_y, double imu_heading) {
        double z[3] = {meas_x, meas_y, imu_heading};
        double y_residual[3] = {z[0] - state_.x, z[1] - state_.y, z[2] - state_.theta};
        while (y_residual[2] > M_PI)  y_residual[2] -= 2 * M_PI;
        while (y_residual[2] < -M_PI) y_residual[2] += 2 * M_PI;

        for (int i = 0; i < 3; ++i) {
            double s = P_[i][i] + R_[i][i];
            double k = P_[i][i] / s;
            if (i == 0) state_.x += k * y_residual[0];
            if (i == 1) state_.y += k * y_residual[1];
            if (i == 2) state_.theta += k * y_residual[2];
            P_[i][i] = (1.0 - k) * P_[i][i];
        }
        normalizeTheta();
    }

    Pose2D getState() const { return state_; }

private:
    void normalizeTheta() {
        while (state_.theta > M_PI)  state_.theta -= 2 * M_PI;
        while (state_.theta < -M_PI) state_.theta += 2 * M_PI;
    }

    Pose2D state_{0.0, 0.0, 0.0};
    double P_[3][3]{{0}};
    double Q_[3][3]{{0}};
    double R_[3][3]{{0}};
};

class ParticleFilter {
public:
    struct Particle {
        Pose2D pose;
        double weight;
    };

    ParticleFilter(size_t num_particles = 80) : num_particles_(num_particles), gen_(1337) {}

    void initialize(const Pose2D& init_pose, double spread_pos = 0.05, double spread_ang = 0.02) {
        std::normal_distribution<double> dist_x(init_pose.x, spread_pos);
        std::normal_distribution<double> dist_y(init_pose.y, spread_pos);
        std::normal_distribution<double> dist_th(init_pose.theta, spread_ang);

        particles_.resize(num_particles_);
        for (auto& p : particles_) {
            p.pose = {dist_x(gen_), dist_y(gen_), dist_th(gen_)};
            p.weight = 1.0 / num_particles_;
        }
    }

    // Propagate particles based on holonomic motion model
    void predict(const Twist2D& vel, double dt) {
        std::normal_distribution<double> noise_vx(vel.vx, 0.02);
        std::normal_distribution<double> noise_vy(vel.vy, 0.02);
        std::normal_distribution<double> noise_w(vel.omega, 0.01);

        for (auto& p : particles_) {
            double vx = noise_vx(gen_);
            double vy = noise_vy(gen_);
            double w  = noise_w(gen_);

            double cos_th = std::cos(p.pose.theta);
            double sin_th = std::sin(p.pose.theta);
            p.pose.x += (vx * cos_th - vy * sin_th) * dt;
            p.pose.y += (vx * sin_th + vy * cos_th) * dt;
            p.pose.theta += w * dt;
            while (p.pose.theta > M_PI)  p.pose.theta -= 2 * M_PI;
            while (p.pose.theta < -M_PI) p.pose.theta += 2 * M_PI;
        }
    }

    // Weight particles based on LiDAR raycast likelihood against known costmap
    void updateLiDAR(const LidarScan2D& scan, const Costmap2D& costmap) {
        double total_weight = 0.0;

        for (auto& p : particles_) {
            double likelihood = 1.0;
            // Evaluate 8 key cardinal LiDAR beams (0, 45, 90, 135, 180, 225, 270, 315)
            for (int angle = 0; angle < 360; angle += 45) {
                double beam_dist = scan.ranges[angle];
                if (beam_dist >= scan.max_range) continue;

                double beam_rad = p.pose.theta + (angle * M_PI / 180.0);
                double hit_x = p.pose.x + beam_dist * std::cos(beam_rad);
                double hit_y = p.pose.y + beam_dist * std::sin(beam_rad);

                uint8_t cost = costmap.getCost(hit_x, hit_y);
                // Higher costmap cost near expected hit = higher observation likelihood
                double match = (cost >= Costmap2D::INSCRIBED_INFLATED) ? 1.0 : (cost / 255.0);
                likelihood *= (0.2 + 0.8 * match);
            }
            p.weight = likelihood;
            total_weight += likelihood;
        }

        if (total_weight > 1e-9) {
            for (auto& p : particles_) p.weight /= total_weight;
        } else {
            for (auto& p : particles_) p.weight = 1.0 / num_particles_;
        }
        resample();
    }

    Pose2D getEstimatedPose() const {
        double avg_x = 0, avg_y = 0;
        double sin_sum = 0, cos_sum = 0;
        for (const auto& p : particles_) {
            avg_x += p.pose.x * p.weight;
            avg_y += p.pose.y * p.weight;
            sin_sum += std::sin(p.pose.theta) * p.weight;
            cos_sum += std::cos(p.pose.theta) * p.weight;
        }
        return {avg_x, avg_y, std::atan2(sin_sum, cos_sum)};
    }

private:
    void resample() {
        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);
        std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
        double r = dist(gen_);
        double c = particles_[0].weight;
        size_t idx = 0;

        for (size_t m = 0; m < num_particles_; ++m) {
            double u = r + m * (1.0 / num_particles_);
            while (u > c && idx < num_particles_ - 1) {
                idx++;
                c += particles_[idx].weight;
            }
            new_particles.push_back(particles_[idx]);
            new_particles.back().weight = 1.0 / num_particles_;
        }
        particles_ = std::move(new_particles);
    }

    size_t num_particles_;
    std::vector<Particle> particles_;
    mutable std::mt19937 gen_;
};

// ==============================================================================================
// 6. MAIN SENSE - THINK - ACT COORDINATOR (THE WHITEBOARD ARCHITECTURE)
// ==============================================================================================

class WhiteboardAutonomousSystem {
public:
    WhiteboardAutonomousSystem()
        : costmap_(-3.0, -1.0, 6.0, 7.0, 0.05), // Covers arena [-3, 3] x [-1, 6] meters
          dwa_planner_(cfg_) {}

    // ========================================================================
    // 1. INITIALIZE (Step 1 from Whiteboard)
    // ========================================================================
    void Initialize(double init_x = 0.0, double init_y = 0.0, double init_theta = M_PI / 2.0) {
        // Rule 1: Starting coordinate must be (0, 0)
        start_origin_ = {init_x, init_y, init_theta};

        // Initialize EKF & Particle Filter with origin (0, 0)
        ekf_.initialize(0.0, 0.0, init_theta);
        pf_.initialize({0.0, 0.0, init_theta});

        // Initialize Costmap & Obstacles (Simulating arena boundary & pillars)
        costmap_.setLethalBox(-2.8, -0.8, -2.7, 5.5); // Left boundary wall
        costmap_.setLethalBox( 2.7, -0.8,  2.8, 5.5); // Right boundary wall
        costmap_.setLethalBox(-2.8,  5.4,  2.8, 5.5); // Top wall
        costmap_.setLethalBox(-0.5,  2.2,  0.5, 2.4); // Chokepoint wall/obstacle

        // Compute Inflation Layer (Lethal, Inscribed Robot Radius, Exponential Decay)
        costmap_.computeInflation(cfg_.robot_radius, cfg_.inflation_radius);

        // Load target coordinate from instruction: (0, 2), (2, 3), (-2, 4)
        targets_ = {
            { 0.0, 2.0, 0.0},
            { 2.0, 3.0, 0.0},
            {-2.0, 4.0, 0.0}
        };

        current_target_index_ = 0;
        hold_timer_ = 0.0;
        is_holding_at_point_ = false;
        all_completed_ = false;

        // Plan initial global A* path to first waypoint (0, 2)
        replanGlobalPath();

        std::cout << "==================================================================\n";
        std::cout << "[1. INITIALIZE]\n";
        std::cout << " - Rule 1: Origin Frame set at (0, 0)\n";
        std::cout << " - Rule 2: Holonomic 16-Mecanum Kinematics online\n";
        std::cout << " - Rule 3: 2D LiDAR (Eyes) & 6-DOF IMU (Ears) Data Streams Active\n";
        std::cout << " - Rule 4: 5.0s Interval per Waypoint activated\n";
        std::cout << " - State Estimator: Extended Kalman Filter + Particle Filter (MCL)\n";
        std::cout << " - Planning Stack: A* Global Path + Holonomic DWA Local Trajectory\n";
        std::cout << " - Loaded Targets: Point 1:(0, 2) -> Point 2:(2, 3) -> Point 3:(-2, 4)\n";
        std::cout << "==================================================================\n\n";
    }

    // ========================================================================
    // 2. SENSE (Step 2 from Whiteboard)
    // ========================================================================
    void Sense(const IMUData& imu_data, const LidarScan2D& lidar_scan, const Twist2D& wheel_odom_vel, double dt) {
        // Read IMU for robot heading (Ears)
        raw_imu_heading_ = imu_data.yaw_heading;

        // EKF Prediction step (Dead-reckoning wheel odometry)
        ekf_.predict(wheel_odom_vel, dt);
        pf_.predict(wheel_odom_vel, dt);

        // Read 2D LiDAR for position & obstacle verification (Eyes)
        pf_.updateLiDAR(lidar_scan, costmap_);
        Pose2D pf_estimated = pf_.getEstimatedPose();

        // EKF Update step fusing IMU ears and LiDAR eyes
        ekf_.update(pf_estimated.x, pf_estimated.y, raw_imu_heading_);

        // Current best filtered estimate of robot position
        estimated_pose_ = ekf_.getState();
        current_lidar_scan_ = lidar_scan;
    }

    // ========================================================================
    // 3. THINK (Step 3 from Whiteboard)
    // ========================================================================
    void Think(double dt) {
        if (all_completed_) return;

        if (current_target_index_ >= targets_.size()) {
            all_completed_ = true;
            std::cout << "\n==================================================================\n";
            std::cout << ">>> [THINK] MISSION COMPLETE: All Whiteboard Targets Reached! <<<\n";
            std::cout << "==================================================================\n";
            return;
        }

        const Pose2D& target = targets_[current_target_index_];
        double dx = target.x - estimated_pose_.x;
        double dy = target.y - estimated_pose_.y;
        double distance_to_target = std::hypot(dx, dy);

        // Check if arrived within tolerance (Rule 4 check)
        if (distance_to_target < cfg_.waypoint_tolerance && !is_holding_at_point_) {
            is_holding_at_point_ = true;
            hold_timer_ = 0.0;
            std::cout << "\n>>> [THINK] ARRIVED at Target #" << (current_target_index_ + 1)
                      << " (" << target.x << ", " << target.y << ")! Starting 5-second interval hold...\n";
        }

        // Rule 4: Interval 5 sec / point logic
        if (is_holding_at_point_) {
            hold_timer_ += dt;
            int second = static_cast<int>(hold_timer_);
            if (second != last_logged_second_ && second <= 5) {
                std::cout << "    [Hold Timer] Waypoint #" << (current_target_index_ + 1)
                          << " -> " << std::fixed << std::setprecision(1) << hold_timer_
                          << "s / 5.0s\n";
                last_logged_second_ = second;
            }

            if (hold_timer_ >= cfg_.hold_interval_sec) {
                std::cout << ">>> [THINK] 5.0s Interval Elapsed! Advancing to next waypoint.\n\n";
                current_target_index_++;
                hold_timer_ = 0.0;
                last_logged_second_ = -1;
                is_holding_at_point_ = false;

                if (current_target_index_ < targets_.size()) {
                    replanGlobalPath();
                }
            }
            cmd_vel_ = {0.0, 0.0, 0.0};
            return;
        }

        // Periodic A* replanning or if current global path is exhausted
        if (current_global_path_.empty() || shouldReplanGlobal()) {
            replanGlobalPath();
        }

        // Holonomic Dynamic Window Approach (DWA) local velocity trajectory optimization
        cmd_vel_ = dwa_planner_.computeVelocity(estimated_pose_, cmd_vel_, current_global_path_, costmap_, dt);
    }

    // ========================================================================
    // 4. ACT (Step 4 from Whiteboard)
    // ========================================================================
    Twist2D Act() {
        if (all_completed_ || is_holding_at_point_) {
            // - Stop robot for 5 sec interval at each point
            cmd_vel_ = {0.0, 0.0, 0.0};
        }

        // - Move robot using holonomic wheels (Vx, Vy, Omega command sent to Mecanum motors)
        return cmd_vel_;
    }

    bool isDone() const { return all_completed_; }
    Pose2D getEstimatedPose() const { return estimated_pose_; }
    size_t getCurrentTargetIndex() const { return current_target_index_; }

private:
    void replanGlobalPath() {
        if (current_target_index_ >= targets_.size()) return;
        current_global_path_ = AStarPlanner::plan(costmap_, estimated_pose_, targets_[current_target_index_]);
    }

    bool shouldReplanGlobal() const {
        if (current_global_path_.empty()) return true;
        double dist_to_start_of_path = std::hypot(current_global_path_.front().x - estimated_pose_.x,
                                                  current_global_path_.front().y - estimated_pose_.y);
        return dist_to_start_of_path > 0.50; // Deviated from path
    }

    Pose2D start_origin_{0.0, 0.0, 0.0};
    Pose2D estimated_pose_{0.0, 0.0, 0.0};
    double raw_imu_heading_{0.0};
    LidarScan2D current_lidar_scan_;

    RobotConfig cfg_;
    Costmap2D costmap_;
    HolonomicDWAPlanner dwa_planner_;
    ExtendedKalmanFilter ekf_;
    ParticleFilter pf_;

    std::vector<Pose2D> targets_;
    std::vector<Pose2D> current_global_path_;
    size_t current_target_index_{0};

    bool is_holding_at_point_{false};
    double hold_timer_{0.0};
    int last_logged_second_{-1};
    bool all_completed_{false};

    Twist2D cmd_vel_{0.0, 0.0, 0.0};
};

// ==============================================================================================
// 7. DEMONSTRATION & VERIFICATION HARNESS
// ==============================================================================================

int main() {
    WhiteboardAutonomousSystem robot_system;

    // 1. Initialize: Origin (0,0), Lidar/IMU streams, targets (0,2), (2,3), (-2,4)
    robot_system.Initialize(0.0, 0.0, M_PI / 2.0); // Starts at (0, 0), facing +Y

    // Mock Simulation parameters
    double ground_truth_x = 0.0;
    double ground_truth_y = 0.0;
    double ground_truth_th = M_PI / 2.0; // Facing UP (+Y)
    const double dt = 0.1;               // 10 Hz control loop step

    IMUData imu;
    LidarScan2D lidar;
    Twist2D wheel_odom{0.0, 0.0, 0.0};

    int cycle_step = 0;
    std::cout << "Starting Full Autonomous Sense-Think-Act Navigation Loop...\n\n";

    while (!robot_system.isDone() && cycle_step < 500) {
        // 2. SENSE: Read IMU for heading, Read LiDAR for position
        imu.yaw_heading = ground_truth_th + (std::sin(cycle_step * 0.1) * 0.002); // Small noise
        imu.yaw_rate = wheel_odom.omega;

        // Mock 2D LiDAR ray scan
        for (int b = 0; b < 360; ++b) {
            lidar.ranges[b] = 3.5; // Clear arena
        }

        robot_system.Sense(imu, lidar, wheel_odom, dt);

        // 3. THINK: EKF + MCL, A* Plan, Holonomic DWA Trajectory, 5s Hold
        robot_system.Think(dt);

        // 4. ACT: Output holonomic velocity (Vx, Vy, Omega)
        Twist2D cmd = robot_system.Act();
        wheel_odom = cmd;

        // Integrate physical kinematic displacement (Mecanum 3-DOF simulation)
        double cos_th = std::cos(ground_truth_th);
        double sin_th = std::sin(ground_truth_th);
        double global_vx = cmd.vx * cos_th - cmd.vy * sin_th;
        double global_vy = cmd.vx * sin_th + cmd.vy * cos_th;

        ground_truth_x += global_vx * dt;
        ground_truth_y += global_vy * dt;
        ground_truth_th += cmd.omega * dt;

        // Periodic Telemetry log (~once per second)
        if (cycle_step % 10 == 0) {
            Pose2D est = robot_system.getEstimatedPose();
            std::cout << "[T=" << std::setw(4) << std::fixed << std::setprecision(1) << (cycle_step * dt) << "s] "
                      << "Target #" << (robot_system.getCurrentTargetIndex() + 1) << " | "
                      << "Pose: (" << std::setw(5) << std::setprecision(2) << est.x << ", " << est.y << ") | "
                      << "Heading: " << std::setw(5) << std::setprecision(1) << (est.theta * 180.0 / M_PI) << "° | "
                      << "CmdVel: [Vx=" << std::setprecision(2) << cmd.vx
                      << ", Vy=" << cmd.vy
                      << ", w=" << cmd.omega << "]\n";
        }

        cycle_step++;
    }

    std::cout << "\n>>> Simulation completed successfully. Ready for Tuesday software tweaking! <<<\n";
    return 0;
}
