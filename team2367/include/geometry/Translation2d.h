#pragma once

#include <cmath>
#include <iostream>
#include <iomanip>
#include "geometry/Rotation2d.h"

class Translation2d {
public:

  double x_;
  double y_;

  Translation2d() : x_(0), y_(0) {}

  Translation2d(double x, double y) : x_(x), y_(y) {}

  Translation2d(const Translation2d& other) : x_(other.x_), y_(other.y_) {}

  Translation2d(const Translation2d& start, const Translation2d& end) : x_(end.x_ - start.x_), y_(end.y_ - start.y_) {}

  // The "norm" of a transform is the Euclidean distance in x and y.
  double norm() const {
    return std::hypot(x_, y_);
  }

  double norm2() const {
    return x_ * x_ + y_ * y_;
  }

  double x() {
    return x_;
  }

  double y() {
    return y_;
  }

  // We can compose Translation2d's by adding together the x and y shifts.
  Translation2d translateBy(const Translation2d& other) const {
    return Translation2d(x_ + other.x_, y_ + other.y_);
  }

  Translation2d operator+(const Translation2d& other) const {
    return Translation2d(x_ + other.x_, y_ + other.y_);
  }

  Translation2d operator-(const Translation2d& other) const {
    return Translation2d(x_ - other.x_, y_ - other.y_);
  }

  Translation2d operator-() const {
    return Translation2d(-x_, -y_);
  }

  Translation2d operator*(double scalar) const {
    return Translation2d(x_ * scalar, y_ * scalar);
  }




  // The inverse simply means a Translation2d that "undoes" this object.
  Translation2d inverse() const {
    return Translation2d(-x_, -y_);
  }

  // Interpolates between this translation and another translation.
  Translation2d interpolate(const Translation2d& other, double x) const {
    if (x <= 0) {
      return Translation2d(*this);
    } else if (x >= 1) {
      return Translation2d(other);
    }
    return extrapolate(other, x);
  }

  // Extrapolates between this translation and another translation.
  Translation2d extrapolate(const Translation2d& other, double x) const {
    return Translation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
  }

  Translation2d scale(double s) const {
    return Translation2d(x_ * s, y_ * s);
  }
};
