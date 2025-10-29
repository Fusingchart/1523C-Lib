#pragma once

struct Point {
    double x;
    double y;

    Point(const double x, const double y): x(x), y(y) {
    }

    Point operator+(const Point& other) const {
        return {x + other.x, y + other.y};
    }

    Point operator-(const Point& other) const {
        return {x - other.x, y - other.y};
    }

    Point operator*(const double value) const {
        return {x * value, y * value};
    }

    static double dist(const Point& a, const Point& b) {
        const double x_dif = (a.x - b.x);
        const double y_dif = (a.y - b.y);
        return std::sqrt(x_dif * x_dif + y_dif * y_dif);
    }

    static Point rotate(const Point& c, const Point& p, const double rad) {
        const Point rp = p - c;
        const double c_rad = cos(rad);
        const double s_rad = sin(rad);

        return c + Point(rp.x * c_rad - rp.y * s_rad, rp.x * s_rad + rp.y * c_rad);
    }
};

enum class Dir {
    Front,
    Left,
    Right,
    Back
};

static std::int32_t dir_to_sign(const Dir dir) {
    if (dir == Dir::Front || dir == Dir::Right) return 1;
    return -1;
}

enum class Color {
    Red, Blue, None
};
