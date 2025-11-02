//
// Created by Hanyu Zhang on 10/12/25.
//

#ifndef MCL_H
#define MCL_H

#include <random>
#include <algorithm>
#include <cmath>

#include "data.h"
#include "util.h"

inline constexpr int MAX_DIST_SENSOR_RANGE = 2000;
inline constexpr int MAX_DATA_COUNT = 60 * 10;

struct Particle {
    Pose pose;
    int weight;

    Particle(const Pose& pose = {0, 0, 0}, int weight = 0)
        : pose(pose),
          weight(weight) {
    }
};

inline std::vector<Segment> obstacles;

struct Distance {

public:
    static const int MAX_DIST_SENSOR_RANGE = 2000;

    static int get_distance(const Pose pos) {
        float min_distance = std::numeric_limits<float>::infinity();

        // Ray starting point and direction
        float ray_x = pos.x_;
        float ray_y = pos.y_;
        float ray_dx = std::cos(pos.h_);
        float ray_dy = std::sin(pos.h_);

        for (const auto& obstacle : obstacles) {
            float intersection_distance = raySegmentIntersection(
                ray_x, ray_y, ray_dx, ray_dy,
                obstacle.a.x, obstacle.a.y, obstacle.b.x, obstacle.b.y
            );

            if (intersection_distance > 0 && intersection_distance < min_distance) {
                min_distance = intersection_distance;
            }
        }

        // Convert from field units to millimeters
        // Assuming field coordinates are in inches (common for VEX fields)
        // 1 inch = 25.4 mm
        if (std::isinf(min_distance)) {
            return std::numeric_limits<int>::max();
        }

        int dist = static_cast<int>(min_distance * 25.4f);
        if (dist > MAX_DIST_SENSOR_RANGE) {
            dist = INT_MAX;
        }
        return dist;
    }

private:
    // Ray-segment intersection using parametric line equations
    static float raySegmentIntersection(float ray_x, float ray_y, float ray_dx, float ray_dy,
                                 float seg_x1, float seg_y1, float seg_x2, float seg_y2) {
        float seg_dx = seg_x2 - seg_x1;
        float seg_dy = seg_y2 - seg_y1;

        float denominator = ray_dx * seg_dy - ray_dy * seg_dx;

        // Lines are parallel
        if (std::abs(denominator) < 1e-10) {
            return -1.0f;
        }

        float t = ((seg_x1 - ray_x) * seg_dy - (seg_y1 - ray_y) * seg_dx) / denominator;
        float u = ((seg_x1 - ray_x) * ray_dy - (seg_y1 - ray_y) * ray_dx) / denominator;

        // Check if intersection point is on the ray (t >= 0) and on the segment (0 <= u <= 1)
        if (t >= 0 && u >= 0 && u <= 1) {
            return t; // Distance along the ray
        }

        return -1.0f; // No valid intersection
    }
};

class MCL {
    const float t_ft = 0.5; // trans-from-trans (inch per inch)
    const float t_fr = 0.5; // trans-from-rot (inch per rad)
    const float r_ft = 0.00; // rot-from-trans (rad per inch)
    const float r_fr = 0; // rot-from-rot
    const float ct_t = 0.02; // cross-track-from-trans

    float min_trans = 0.1;

    const int PREDICTION_DISTRIB_COUNT = 8;
    const int N_SPLATTER = 20;

    const int RESAMPLE_PARTICLES = 20;


    std::random_device rd;
    std::mt19937 gen{rd()};
    std::uniform_int_distribution<> distrib{-100, 100};

    float drive_in_ = 0;

    float turn_rad_ = 0;

    std::vector<Particle> particles_;

    int BEST_THRESHOLD = INT_MAX - 150 * 150;

    bool splatter_particles = false;

public:
    std::vector<std::pair<pros::Distance*, Pose>> distance_sensors;
    lemlib::Chassis* chassis = nullptr;
    std::vector<int> dist_sensor_measurements;

    float randR() {
        return static_cast<float>(distrib(gen)) / 100.0f;
    }

    std::vector<Particle> getParticles() {
        return particles_;
    }

    void initializePose(const Pose pose) {
        particles_.clear();
        particles_.emplace_back(pose, INT_MAX);
    }

    /**
     * Prediction step of MCL.
     *
     * @param drive_in inches driven
     * @param turn_rad radians turned
     */
    void predict(const float drive_in, const float turn_rad) {
        split_particles();
        set_delta(drive_in, turn_rad);
        turn_points();
        move_points();
        distrib_trans();
        normalize_particles();
    }

    void split_particles() {
        const std::vector<Particle> reference = particles_;
        // particles_.clear();

        // if (splatter_particles) {
        //     for (int i = 0; i < N_PARTICLES; ++i) {
        //         particles_.push_back({{randR() * 70, randR() * 70, reference.at(0).pose.h_}, 0});
        //     }
        //     std::cout << "splattered\n";
        // }
        splatter_particles = false;

        for (Particle particle : reference) {
            for (int i = 0; i < PREDICTION_DISTRIB_COUNT; ++i) {
                particles_.push_back(particle);
            }
        }

        for (int i = 0; i < N_SPLATTER; ++i) {
            particles_.push_back({{randR() * 70, randR() * 70, reference.at(0).pose.h_}, 0});
        }
    }

    void set_delta(const float drive_in, const float turn_rad) {
        drive_in_ = drive_in;
        turn_rad_ = turn_rad;
    }

    void turn_points() {
        for (auto& [pose, _] : particles_)
            pose.h_ -= turn_rad_;
    }

    void distrib_heading() {
        for (auto& [pose, _] : particles_)
            pose.h_ -= randR() * turn_rad_ * r_fr + randR() * drive_in_ * r_ft;
    }

    void move_points() {
        for (auto& [pose, _] : particles_)
            pose.move(drive_in_);
    }

    void distrib_trans() {
        for (auto& [pose, _] : particles_) {
            const float trans = std::max(randR() * min_trans, randR() * t_ft * drive_in_ + randR() * t_fr * turn_rad_);
            pose.move(trans, randR() * M_PI);
        }
    }

    void normalize_particles() {
        for (auto& [pose, _] : particles_) {
            pose.x_ = std::fmod(pose.x_, 72);
            pose.y_ = std::fmod(pose.y_, 72);
        }
    }

    void measure() {
        measure_real_sensors();
        simulate_dist_sensor();
        resample_weight();
    }

    void measure_real_sensors() {
        dist_sensor_measurements.clear();
        for (auto& [dist_sensor, offset] : distance_sensors) {
            const int dist_sensed = dist_sensor->get_distance();

            dist_sensor_measurements.push_back(dist_sensed);
        }
    }


    void simulate_dist_sensor() {
        int best = INT_MIN;

        for (auto& [pose, weight] : particles_) {
            std::vector<int> curr_measurements;

            for (auto& [_, offset] : distance_sensors) {
                Pose dsp = Pose::rotate(pose.h_, offset);
                dsp += pose;
                dsp.h_ = offset.h_ + pose.h_;

                const int dist_sensed = Distance::get_distance(dsp);

                curr_measurements.push_back(dist_sensed);
            }

            const size_t count = std::min(curr_measurements.size(), dist_sensor_measurements.size());

            weight = INT_MAX;

            for (int i = 0; i < count; ++i) {
                int real = dist_sensor_measurements.at(i);
                int curr = curr_measurements.at(i);

                if (real > 2000) real = MAX_DIST_SENSOR_RANGE * 1.5;
                if (curr == INT_MAX) curr = MAX_DIST_SENSOR_RANGE * 1.5;

                weight -= (real - curr) * (real - curr);
            }

            if (weight > best) best = weight;
        }

        if (best < BEST_THRESHOLD) {
            splatter_particles = true;
        }


        int error = std::sqrt(INT_MAX - best) / 25.4;
        min_trans = (error);
    }

    void resample_random() {
        std::uniform_int_distribution<> resample_rand{0, PREDICTION_DISTRIB_COUNT};

        const std::vector<Particle> reference = particles_;
        particles_.clear();

        bool first = true;
        for (Particle particle : reference) {
            if (resample_rand(gen) == 0 || first) {
                particles_.push_back(particle);
            }
            first = false;
        }
    }

    void resample_weight() {
        if (particles_.empty()) return;

        // Sort particles by weight in descending order (highest weight first)
        std::sort(particles_.begin(), particles_.end(),
                  [](const Particle& a, const Particle& b) {
                      return a.weight > b.weight;
                  });

        // Determine N - keep top particles (you can adjust this value)
        const size_t N = std::min(static_cast<size_t>(RESAMPLE_PARTICLES), particles_.size());
        // Keep top 100 or all if less

        // Keep only the top N particles
        particles_.resize(N);
    }
};

#endif //MCL_H
