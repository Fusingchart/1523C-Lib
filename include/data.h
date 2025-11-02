//
// Created by Hanyu Zhang on 10/12/25.
//

#ifndef DATA_H
#define DATA_H

struct Vec2 {
    float x;
    float y;

    Vec2(const float x, const float y): x(x), y(y) {
    }
};

struct Segment {
    Vec2 a;
    Vec2 b;

    Segment(const Vec2 a, const Vec2 b): a(a), b(b) {
    }
};

struct Pose {
    float x_;
    float y_;
    float h_;

    Pose(const float x, const float y, const float h): x_(x), y_(y), h_(h) {
    }

    Pose& operator+=(const Pose& other) {
        x_ += other.x_;
        y_ += other.y_;
        h_ += other.h_;
        return *this;
    }

    static Pose sum(const Pose& a, const Pose& b) {
        return {
            a.x_ + b.x_, a.y_ + b.y_, a.h_ + b.h_
        };
    }

    static Pose migrate(const Point point) {
        return {static_cast<float>(point.x), static_cast<float>(point.y), 0};
    }

    void move(const float dist, float rad = std::numeric_limits<float>::quiet_NaN()) {
        if (std::isnan(rad)) rad = h_;
        x_ += dist * cos(rad);
        y_ += dist * sin(rad);
    }

    static Pose move(const Pose pose, const float dist, float rad = std::numeric_limits<float>::quiet_NaN()) {
        if (std::isnan(rad)) rad = pose.h_;
        const float x = pose.x_ + dist * cos(rad);
        const float y = pose.y_ + dist * sin(rad);

        return {x, y, pose.h_};
    }

    static Pose rotate(const float rad, const Pose pose, const Pose center = {0, 0, 0}) {
        // Translate pose to origin relative to center
        float dx = pose.x_ - center.x_;
        float dy = pose.y_ - center.y_;

        // Apply rotation matrix
        float cos_rad = std::cos(rad);
        float sin_rad = std::sin(rad);

        float rotated_x = dx * cos_rad - dy * sin_rad;
        float rotated_y = dx * sin_rad + dy * cos_rad;

        // Translate back and update heading
        float new_x = rotated_x + center.x_;
        float new_y = rotated_y + center.y_;
        float new_h = pose.h_ + rad;

        return {new_x, new_y, new_h};
    }
};

#endif //DATA_H
