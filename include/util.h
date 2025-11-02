//
// Created by Hanyu Zhang on 10/12/25.
//

#ifndef UTIL_H
#define UTIL_H

constexpr float FIELD_DISPLAY_SCALE = 4.7f;

inline int sgn(const double x) {
    return (x > 0) ? 1 : (x < 0) ? -1 : 0;
}
inline int sgn(const float x) {
    return (x > 0) ? 1 : (x < 0) ? -1 : 0;
}
inline int sgn(const int x) {
    return (x > 0) ? 1 : (x < 0) ? -1 : 0;
}

inline float dist(const float x, const float y) {
    return sqrt(x * x + y * y);
}

#endif //UTIL_H
