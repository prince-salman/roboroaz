#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <algorithm>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

struct Twist2D {
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};
};

struct LidarScan2D {
    static constexpr size_t NUM_BEAMS = 360;
    std::vector<double> ranges;
    double min_range{0.05};
    double max_range{5.00};
    LidarScan2D() : ranges(NUM_BEAMS, 5.0) {}
};

struct IMUData {
    double yaw_heading{0.0};
    double yaw_rate{0.0};
};

struct RobotConfig {
    double max_vx{0.50};
    double min_vx{-0.30};
    double max_vy{0.40};
    double min_vy{-0.40};
    double max_omega{2.00};
    double min_omega{-2.00};

    double accel_vx{1.50};
    double accel_vy{1.50};
    double accel_omega{4.00};

    double robot_radius{0.22};
    double inflation_radius{0.45};
    double waypoint_tolerance{0.28};
    double hold_interval_sec{5.0};
};

class Costmap2D {
public:
    static constexpr uint8_t NO_OBSTACLE = 0;
    static constexpr uint8_t INSCRIBED_INFLATED = 253;
    static constexpr uint8_t LETHAL_OBSTACLE = 254;

    Costmap2D(double origin_x, double origin_y, double width_m, double height_m, double resolution_m)
        : origin_x_(origin_x), origin_y_(origin_y), width_m_(width_m), height_m_(height_m), resolution_(resolution_m) {
        cols_ = static_cast<int>(std::ceil(width_m_ / resolution_));
        rows_ = static_cast<int>(std::ceil(height_m_ / resolution_));
        grid_.assign(rows_ * cols_, NO_OBSTACLE);
    }

    bool worldToGrid(double wx, double wy, int& mx, int& my) const {
        if (wx < origin_x_ || wx >= origin_x_ + width_m_ || wy < origin_y_ || wy >= origin_y_ + height_m_) return false;
        mx = static_cast<int>((wx - origin_x_) / resolution_);
        my = static_cast<int>((wy - origin_y_) / resolution_);
        return (mx >= 0 && mx < cols_ && my >= 0 && my < rows_);
    }

    void gridToWorld(int mx, int my, double& wx, double& wy) const {
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
    }

    void setLethalBox(double x_min, double y_min, double x_max, double y_max) {
        int x0, y0, x1, y1;
        worldToGrid(x_min, y_min, x0, y0);
        worldToGrid(x_max, y_max, x1, y1);
        for (int y = std::min(y0, y1); y <= std::max(y0, y1); ++y) {
            for (int x = std::min(x0, x1); x <= std::max(x0, x1); ++x) {
                if (x >= 0 && x < cols_ && y >= 0 && y < rows_) grid_[y * cols_ + x] = LETHAL_OBSTACLE;
            }
        }
    }

    void computeInflation(double robot_radius, double inflation_radius) {
        int infl_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
        std::vector<uint8_t> inflated = grid_;

        for (int r = 0; r < rows_; ++r) {
            for (int c = 0; c < cols_; ++c) {
                if (grid_[r * cols_ + c] != LETHAL_OBSTACLE) continue;

                for (int dr = -infl_cells; dr <= infl_cells; ++dr) {
                    for (int dc = -infl_cells; dc <= infl_cells; ++dc) {
                        int nr = r + dr, nc = c + dc;
                        if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;

                        double dist = std::hypot(dr * resolution_, dc * resolution_);
                        if (dist > inflation_radius) continue;

                        uint8_t cost = (dist <= robot_radius) ? INSCRIBED_INFLATED
                                     : static_cast<uint8_t>((inflation_radius - dist) / (inflation_radius - robot_radius) * 250 + 1);

                        size_t idx = nr * cols_ + nc;
                        if (cost > inflated[idx]) inflated[idx] = cost;
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
    double origin_x_, origin_y_, width_m_, height_m_, resolution_;
    int cols_{0}, rows_{0};
    std::vector<uint8_t> grid_;
};

class AStarPlanner {
public:
    struct Node {
        int x, y;
        double g_cost, h_cost;
        double f_cost() const { return g_cost + h_cost; }
    };

    struct NodeCompare {
        bool operator()(const Node& a, const Node& b) const { return a.f_cost() > b.f_cost(); }
    };

    static std::vector<Pose2D> plan(const Costmap2D& costmap, const Pose2D& start, const Pose2D& goal) {
        int start_mx, start_my, goal_mx, goal_my;
        if (!costmap.worldToGrid(start.x, start.y, start_mx, start_my) ||
            !costmap.worldToGrid(goal.x, goal.y, goal_mx, goal_my)) return {start, goal};

        const int cols = costmap.cols(), rows = costmap.rows();
        std::vector<double> g_score(cols * rows, 1e9);
        std::vector<bool> closed(cols * rows, false);
        std::vector<std::pair<int, int>> parent_map(cols * rows, {-1, -1});
        std::priority_queue<Node, std::vector<Node>, NodeCompare> open_set;

        g_score[start_my * cols + start_mx] = 0.0;
        open_set.push({start_mx, start_my, 0.0, std::hypot(goal_mx - start_mx, goal_my - start_my) * costmap.resolution()});

        const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
        const int dy[] = {0, 0, 1, -1, 1, 1, -1, -1};
        const double move_cost[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

        bool found = false;
        while (!open_set.empty()) {
            Node curr = open_set.top();
            open_set.pop();

            size_t idx = curr.y * cols + curr.x;
            if (closed[idx]) continue;
            closed[idx] = true;

            if (curr.x == goal_mx && curr.y == goal_my) { found = true; break; }

            for (int d = 0; d < 8; ++d) {
                int nx = curr.x + dx[d], ny = curr.y + dy[d];
                if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
                size_t n_idx = ny * cols + nx;
                if (closed[n_idx]) continue;

                double wx, wy;
                costmap.gridToWorld(nx, ny, wx, wy);
                uint8_t cost = costmap.getCost(wx, wy);
                if (cost >= Costmap2D::INSCRIBED_INFLATED) continue;

                double tentative_g = curr.g_cost + move_cost[d] * costmap.resolution() + (cost / 255.0) * 0.4;
                if (tentative_g < g_score[n_idx]) {
                    g_score[n_idx] = tentative_g;
                    parent_map[n_idx] = {curr.x, curr.y};
                    double h = std::hypot(goal_mx - nx, goal_my - ny) * costmap.resolution();
                    open_set.push({nx, ny, tentative_g, h});
                }
            }
        }

        std::vector<Pose2D> path;
        if (!found) return {start, goal};

        int cx = goal_mx, cy = goal_my;
        while (cx != -1 && cy != -1) {
            double wx, wy;
            costmap.gridToWorld(cx, cy, wx, wy);
            path.push_back({wx, wy, 0.0});
            auto p = parent_map[cy * cols + cx];
            cx = p.first; cy = p.second;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

class HolonomicDWAPlanner {
public:
    HolonomicDWAPlanner(const RobotConfig& cfg) : cfg_(cfg) {}

    Twist2D computeVelocity(const Pose2D& current_pose, const Twist2D& current_vel,
                            const std::vector<Pose2D>& global_path, const Costmap2D& costmap, double dt) {
        double min_vx = std::max(cfg_.min_vx, current_vel.vx - cfg_.accel_vx * dt);
        double max_vx = std::min(cfg_.max_vx, current_vel.vx + cfg_.accel_vx * dt);
        double min_vy = std::max(cfg_.min_vy, current_vel.vy - cfg_.accel_vy * dt);
        double max_vy = std::min(cfg_.max_vy, current_vel.vy + cfg_.accel_vy * dt);
        double min_w  = std::max(cfg_.min_omega, current_vel.omega - cfg_.accel_omega * dt);
        double max_w  = std::min(cfg_.max_omega, current_vel.omega + cfg_.accel_omega * dt);

        Pose2D carrot = getLookaheadTarget(current_pose, global_path, 0.50);
        Twist2D best_cmd{0.0, 0.0, 0.0};
        double best_score = -1e9;

        const int samples = 7;
        const double dvx = (max_vx - min_vx) / (samples - 1);
        const double dvy = (max_vy - min_vy) / (samples - 1);
        const double dw  = (max_w  - min_w)  / (samples - 1);

        for (int i = 0; i < samples; ++i) {
            double svx = min_vx + i * dvx;
            for (int j = 0; j < samples; ++j) {
                double svy = min_vy + j * dvy;
                for (int k = 0; k < samples; ++k) {
                    double sw = min_w + k * dw;

                    Pose2D sim = current_pose;
                    double min_clearance = 10.0;
                    bool collided = false;

                    for (int s = 0; s < 8; ++s) {
                        double cos_th = std::cos(sim.theta), sin_th = std::sin(sim.theta);
                        sim.x += (svx * cos_th - svy * sin_th) * 0.1;
                        sim.y += (svx * sin_th + svy * cos_th) * 0.1;
                        sim.theta += sw * 0.1;

                        uint8_t cost = costmap.getCost(sim.x, sim.y);
                        if (cost >= Costmap2D::INSCRIBED_INFLATED) { collided = true; break; }
                        double cl = (Costmap2D::INSCRIBED_INFLATED - cost) / 255.0;
                        if (cl < min_clearance) min_clearance = cl;
                    }

                    if (collided) continue;

                    double dist_target = std::hypot(carrot.x - sim.x, carrot.y - sim.y);
                    double target_heading = std::atan2(carrot.y - sim.y, carrot.x - sim.x);
                    double heading_align = std::cos(target_heading - sim.theta);
                    double speed_mag = std::hypot(svx, svy);

                    double score = 0.40 * heading_align + 0.40 * min_clearance - 0.80 * dist_target + 0.20 * speed_mag;
                    if (score > best_score) {
                        best_score = score;
                        best_cmd = {svx, svy, sw};
                    }
                }
            }
        }
        return best_cmd;
    }

private:
    Pose2D getLookaheadTarget(const Pose2D& curr, const std::vector<Pose2D>& path, double lookahead) {
        for (const auto& pt : path) {
            if (std::hypot(pt.x - curr.x, pt.y - curr.y) >= lookahead) return pt;
        }
        return path.empty() ? curr : path.back();
    }
    RobotConfig cfg_;
};

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() {
        P_[0][0] = 0.01; P_[1][1] = 0.01; P_[2][2] = 0.005;
        Q_[0][0] = 0.02; Q_[1][1] = 0.02; Q_[2][2] = 0.01;
        R_[0][0] = 0.05; R_[1][1] = 0.05; R_[2][2] = 0.002;
    }

    void initialize(double x, double y, double th) { state_ = {x, y, th}; }

    void predict(const Twist2D& vel, double dt) {
        double cos_th = std::cos(state_.theta), sin_th = std::sin(state_.theta);
        state_.x += (vel.vx * cos_th - vel.vy * sin_th) * dt;
        state_.y += (vel.vx * sin_th + vel.vy * cos_th) * dt;
        state_.theta += vel.omega * dt;
        normalizeTheta();
        for (int i = 0; i < 3; ++i) P_[i][i] += Q_[i][i] * dt;
    }

    void update(double meas_x, double meas_y, double imu_heading) {
        double z[3] = {meas_x, meas_y, imu_heading};
        double res[3] = {z[0] - state_.x, z[1] - state_.y, z[2] - state_.theta};
        while (res[2] > M_PI)  res[2] -= 2 * M_PI;
        while (res[2] < -M_PI) res[2] += 2 * M_PI;

        for (int i = 0; i < 3; ++i) {
            double k = P_[i][i] / (P_[i][i] + R_[i][i]);
            if (i == 0) state_.x += k * res[0];
            if (i == 1) state_.y += k * res[1];
            if (i == 2) state_.theta += k * res[2];
            P_[i][i] *= (1.0 - k);
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
    double P_[3][3]{{0}}, Q_[3][3]{{0}}, R_[3][3]{{0}};
};

class ParticleFilter {
public:
    struct Particle { Pose2D pose; double weight; };

    ParticleFilter(size_t n = 60) : n_(n), gen_(1337) {}

    void initialize(const Pose2D& init_pose) {
        std::normal_distribution<double> dx(init_pose.x, 0.05), dy(init_pose.y, 0.05), dth(init_pose.theta, 0.02);
        particles_.resize(n_);
        for (auto& p : particles_) {
            p.pose = {dx(gen_), dy(gen_), dth(gen_)};
            p.weight = 1.0 / n_;
        }
    }

    void predict(const Twist2D& vel, double dt) {
        std::normal_distribution<double> n_vx(vel.vx, 0.02), n_vy(vel.vy, 0.02), n_w(vel.omega, 0.01);
        for (auto& p : particles_) {
            double vx = n_vx(gen_), vy = n_vy(gen_), w = n_w(gen_);
            double c = std::cos(p.pose.theta), s = std::sin(p.pose.theta);
            p.pose.x += (vx * c - vy * s) * dt;
            p.pose.y += (vx * s + vy * c) * dt;
            p.pose.theta += w * dt;
        }
    }

    void updateLiDAR(const LidarScan2D& scan, const Costmap2D& costmap) {
        double total = 0.0;
        for (auto& p : particles_) {
            double lik = 1.0;
            for (int ang = 0; ang < 360; ang += 45) {
                double r = scan.ranges[ang];
                if (r >= scan.max_range) continue;
                double hx = p.pose.x + r * std::cos(p.pose.theta + ang * M_PI / 180.0);
                double hy = p.pose.y + r * std::sin(p.pose.theta + ang * M_PI / 180.0);
                uint8_t cost = costmap.getCost(hx, hy);
                lik *= (0.3 + 0.7 * (cost / 255.0));
            }
            p.weight = lik;
            total += lik;
        }
        for (auto& p : particles_) p.weight = (total > 1e-9) ? p.weight / total : 1.0 / n_;
        resample();
    }

    Pose2D getEstimatedPose() const {
        double ax = 0, ay = 0, s_sum = 0, c_sum = 0;
        for (const auto& p : particles_) {
            ax += p.pose.x * p.weight;
            ay += p.pose.y * p.weight;
            s_sum += std::sin(p.pose.theta) * p.weight;
            c_sum += std::cos(p.pose.theta) * p.weight;
        }
        return {ax, ay, std::atan2(s_sum, c_sum)};
    }

private:
    void resample() {
        std::vector<Particle> new_p;
        new_p.reserve(n_);
        std::uniform_real_distribution<double> dist(0.0, 1.0 / n_);
        double r = dist(gen_), c = particles_[0].weight;
        size_t idx = 0;
        for (size_t m = 0; m < n_; ++m) {
            double u = r + m * (1.0 / n_);
            while (u > c && idx < n_ - 1) { idx++; c += particles_[idx].weight; }
            new_p.push_back(particles_[idx]);
            new_p.back().weight = 1.0 / n_;
        }
        particles_ = std::move(new_p);
    }
    size_t n_;
    std::vector<Particle> particles_;
    mutable std::mt19937 gen_;
};

class WhiteboardAutonomousSystem {
public:
    WhiteboardAutonomousSystem()
        : costmap_(-3.0, -1.0, 6.0, 7.0, 0.05), dwa_planner_(cfg_) {}

    void Initialize(double init_x = 0.0, double init_y = 0.0, double init_theta = M_PI / 2.0) {
        start_origin_ = {init_x, init_y, init_theta};
        ekf_.initialize(0.0, 0.0, init_theta);
        pf_.initialize({0.0, 0.0, init_theta});

        costmap_.setLethalBox(-2.8, -0.8, -2.7, 5.5);
        costmap_.setLethalBox( 2.7, -0.8,  2.8, 5.5);
        costmap_.setLethalBox(-2.8,  5.4,  2.8, 5.5);
        costmap_.setLethalBox(-0.5,  2.2,  0.5, 2.4);
        costmap_.computeInflation(cfg_.robot_radius, cfg_.inflation_radius);

        targets_ = { {0.0, 2.0, 0.0}, {2.0, 3.0, 0.0}, {-2.0, 4.0, 0.0} };
        current_target_index_ = 0;
        hold_timer_ = 0.0;
        is_holding_ = false;
        completed_ = false;

        replanGlobalPath();
    }

    void Sense(const IMUData& imu, const LidarScan2D& lidar, const Twist2D& odom_vel, double dt) {
        ekf_.predict(odom_vel, dt);
        pf_.predict(odom_vel, dt);
        pf_.updateLiDAR(lidar, costmap_);
        Pose2D pf_est = pf_.getEstimatedPose();
        ekf_.update(pf_est.x, pf_est.y, imu.yaw_heading);
        estimated_pose_ = ekf_.getState();
    }

    void Think(double dt) {
        if (completed_) return;
        if (current_target_index_ >= targets_.size()) {
            completed_ = true;
            return;
        }

        const Pose2D& target = targets_[current_target_index_];
        double dist = std::hypot(target.x - estimated_pose_.x, target.y - estimated_pose_.y);

        if (dist < cfg_.waypoint_tolerance && !is_holding_) {
            is_holding_ = true;
            hold_timer_ = 0.0;
        }

        if (is_holding_) {
            hold_timer_ += dt;
            if (hold_timer_ >= cfg_.hold_interval_sec) {
                current_target_index_++;
                hold_timer_ = 0.0;
                is_holding_ = false;
                if (current_target_index_ < targets_.size()) replanGlobalPath();
            }
            cmd_vel_ = {0.0, 0.0, 0.0};
            return;
        }

        if (current_global_path_.empty()) replanGlobalPath();
        cmd_vel_ = dwa_planner_.computeVelocity(estimated_pose_, cmd_vel_, current_global_path_, costmap_, dt);
    }

    Twist2D Act() {
        if (completed_ || is_holding_) cmd_vel_ = {0.0, 0.0, 0.0};
        return cmd_vel_;
    }

    bool isDone() const { return completed_; }
    Pose2D getEstimatedPose() const { return estimated_pose_; }
    size_t getCurrentIndex() const { return current_target_index_; }

private:
    void replanGlobalPath() {
        if (current_target_index_ < targets_.size()) {
            current_global_path_ = AStarPlanner::plan(costmap_, estimated_pose_, targets_[current_target_index_]);
        }
    }

    Pose2D start_origin_, estimated_pose_;
    RobotConfig cfg_;
    Costmap2D costmap_;
    HolonomicDWAPlanner dwa_planner_;
    ExtendedKalmanFilter ekf_;
    ParticleFilter pf_;
    std::vector<Pose2D> targets_, current_global_path_;
    size_t current_target_index_{0};
    bool is_holding_{false}, completed_{false};
    double hold_timer_{0.0};
    Twist2D cmd_vel_{0.0, 0.0, 0.0};
};

int main() {
    WhiteboardAutonomousSystem robot;
    robot.Initialize(0.0, 0.0, M_PI / 2.0);

    double sim_x = 0.0, sim_y = 0.0, sim_th = M_PI / 2.0;
    const double dt = 0.1;
    IMUData imu;
    LidarScan2D lidar;
    Twist2D odom{0.0, 0.0, 0.0};

    int step = 0;
    while (!robot.isDone() && step < 350) {
        imu.yaw_heading = sim_th;
        imu.yaw_rate = odom.omega;
        robot.Sense(imu, lidar, odom, dt);
        robot.Think(dt);
        Twist2D cmd = robot.Act();
        odom = cmd;

        double c = std::cos(sim_th), s = std::sin(sim_th);
        sim_x += (cmd.vx * c - cmd.vy * s) * dt;
        sim_y += (cmd.vx * s + cmd.vy * c) * dt;
        sim_th += cmd.omega * dt;

        if (step % 10 == 0) {
            Pose2D est = robot.getEstimatedPose();
            std::cout << "[T=" << std::setw(4) << std::fixed << std::setprecision(1) << (step * dt) << "s] "
                      << "Target #" << (robot.getCurrentIndex() + 1)
                      << " | Pose: (" << std::setw(5) << std::setprecision(2) << est.x << ", " << est.y << ")"
                      << " | CmdVel: [Vx=" << std::setprecision(2) << cmd.vx
                      << ", Vy=" << cmd.vy << ", w=" << cmd.omega << "]\n";
        }
        step++;
    }
    return 0;
}
