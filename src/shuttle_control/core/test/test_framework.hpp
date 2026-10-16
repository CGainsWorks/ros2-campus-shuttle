// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Minimal assertion harness.
//
// The core library depends on nothing but the standard library, and its tests
// keep that property so they can be run in any toolchain image without pulling
// in gtest. The ROS 2 side of the package is tested separately with ament.
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace testing_lite
{

struct Registry
{
  int checks = 0;
  int failures = 0;
  std::string current;

  static Registry & instance()
  {
    static Registry r;
    return r;
  }
};

inline void reportFailure(const char * file, int line, const std::string & message)
{
  Registry & r = Registry::instance();
  ++r.failures;
  std::cerr << "  FAIL [" << r.current << "] " << file << ":" << line << "  " << message << "\n";
}

inline void checkTrue(bool cond, const char * expr, const char * file, int line)
{
  ++Registry::instance().checks;
  if (!cond) {
    reportFailure(file, line, std::string("expected true: ") + expr);
  }
}

inline void checkNear(
  double actual, double expected, double tol, const char * expr, const char * file, int line)
{
  ++Registry::instance().checks;
  if (!(std::fabs(actual - expected) <= tol)) {
    reportFailure(
      file, line,
      std::string(expr) + " = " + std::to_string(actual) + ", expected " +
      std::to_string(expected) + " +/- " + std::to_string(tol));
  }
}

/// Compare two values and print both on failure, so a wrong size or count
/// reports what it actually was rather than only that the check failed.
template<typename A, typename B, typename Cmp>
inline void checkCompare(
  const A & a, const B & b, Cmp cmp, const char * expr, const char * file, int line)
{
  ++Registry::instance().checks;
  if (!cmp(a, b)) {
    std::ostringstream msg;
    msg << "expected " << expr << ", got " << a << " and " << b;
    reportFailure(file, line, msg.str());
  }
}

}  // namespace testing_lite

#define CHECK(cond) testing_lite::checkTrue((cond), #cond, __FILE__, __LINE__)
#define CHECK_EQ(a, b) \
  testing_lite::checkCompare((a), (b), std::equal_to<>(), #a " == " #b, __FILE__, __LINE__)
#define CHECK_GT(a, b) \
  testing_lite::checkCompare((a), (b), std::greater<>(), #a " > " #b, __FILE__, __LINE__)
#define CHECK_NEAR(actual, expected, tol) \
  testing_lite::checkNear((actual), (expected), (tol), #actual, __FILE__, __LINE__)
#define TEST_CASE(name) \
  testing_lite::Registry::instance().current = (name); \
  std::cout << "- " << (name) << "\n";
