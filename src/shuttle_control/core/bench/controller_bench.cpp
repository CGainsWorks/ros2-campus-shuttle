// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Offline, headless, deterministic comparison of Pure Pursuit against Stanley.
//
// Usage:
//   controller_bench [--out DIR] [--speeds 0.5,1.0,1.5] [--tune] [--gains FILE]
//                    [--no-timeseries]
//
// Outputs:
//   DIR/metrics.csv                  one row per run
//   DIR/timeseries/<run>.csv         per-control-step log for every run
//   DIR/tuned_gains.yaml             written by --tune
//
// Both controllers are driven through the identical scenario set, at the
// identical speeds, from the identical start poses, against a plant seeded
// identically. The only difference between the two columns of the results table
// is the lateral control law.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "shuttle_control/bench_runner.hpp"
#include "shuttle_control/pure_pursuit.hpp"
#include "shuttle_control/scenarios.hpp"
#include "shuttle_control/stanley.hpp"

namespace
{

using shuttle::PurePursuitParams;
using shuttle::StanleyParams;

struct Options
{
  std::string out_dir{"results/bench"};
  std::string gains_file;
  std::string dump_paths_dir;
  std::vector<double> speeds{0.5, 1.0, 1.5};
  bool tune{false};
  bool timeseries{true};
};

std::vector<std::string> split(const std::string & s, char delim)
{
  std::vector<std::string> parts;
  std::string item;
  std::istringstream is(s);
  while (std::getline(is, item, delim)) {
    if (!item.empty()) {
      parts.push_back(item);
    }
  }
  return parts;
}

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

/// Minimal reader for the flat `key: value` subset of YAML this tool writes.
std::map<std::string, double> readGains(const std::string & path)
{
  std::map<std::string, double> out;
  std::ifstream in(path);
  if (!in) {
    return out;
  }
  std::string line;
  while (std::getline(in, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string val = trim(line.substr(colon + 1));
    if (key.empty() || val.empty()) {
      continue;
    }
    try {
      out[key] = std::stod(val);
    } catch (const std::exception &) {
      // Not a scalar line (a nested mapping header, say); skip it.
    }
  }
  return out;
}

double lookup(const std::map<std::string, double> & m, const std::string & k, double fallback)
{
  const auto it = m.find(k);
  return it == m.end() ? fallback : it->second;
}

std::string speedTag(double v)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(1) << v;
  std::string s = os.str();
  std::replace(s.begin(), s.end(), '.', 'p');
  return s;
}

shuttle::RunConfig makeConfig(const std::string & scenario, double speed, bool noise)
{
  shuttle::RunConfig cfg;
  cfg.noise.enabled = noise;
  cfg.seed = shuttle::seedFor(scenario, speed, noise);
  return cfg;
}

void applyCommon(shuttle::PathTracker & tracker, double speed)
{
  auto & c = tracker.common();
  c.speed.v_max = speed;
  c.speed.a_lon_max = 0.8;
  c.speed.a_lat_max = 0.8;
  c.speed.v_min_moving = 0.15;
  c.goal_tolerance = 0.25;
  c.max_omega = 1.5;
  c.search_window = 120;
}

// Reference scales for the tuning objective. Cross-track RMS and control effort
// differ by two orders of magnitude in raw units (metres against rad/s^2), so
// summing them directly would make the effort term invisible and drive the
// search to the most aggressive gains on the grid. Each term is divided by a
// scale that represents "clearly poor" for that quantity.
constexpr double kCtRef = 0.05;      ///< m, RMS cross-track error
constexpr double kEffortRef = 5.0;   ///< rad/s^2, RMS d(omega)/dt
constexpr double kEffortWeight = 0.5;  ///< accuracy counts twice as much as smoothness
constexpr double kFailurePenalty = 100.0;

/// Tuning objective: normalised RMS cross-track error plus a normalised control
/// effort term, averaged over the tuning scenarios.
///
/// The effort term is not cosmetic. Without it the search minimises tracking
/// error alone, which for Pure Pursuit means the shortest lookahead on the grid
/// and for Stanley the highest cross-track gain, and both of those sit right on
/// the edge of oscillation. Penalising effort keeps the chosen gains in the
/// interior of the grid.
///
/// The tuning set is deliberately a subset of the scenarios. heading_offset,
/// figure_eight and sharp_turns are held out, so the reported comparison
/// includes conditions neither controller was tuned on.
double tuningCost(shuttle::PathTracker & tracker, const std::vector<double> & speeds)
{
  const std::vector<std::string> tuning_set{
    "straight_offset", "circle", "corner", "campus_loop"};
  double total = 0.0;
  int n = 0;
  for (const auto & name : tuning_set) {
    for (double v : speeds) {
      shuttle::Scenario sc;
      if (!shuttle::makeScenario(name, v, sc)) {
        continue;
      }
      applyCommon(tracker, v);
      const auto res = shuttle::runScenario(tracker, sc, makeConfig(name, v, false));
      double cost = res.metrics.cross_track.rms / kCtRef +
        kEffortWeight * res.metrics.control_effort / kEffortRef;
      if (res.metrics.diverged || res.metrics.stalled || !res.metrics.goal_reached) {
        cost += kFailurePenalty;
      }
      total += cost;
      ++n;
    }
  }
  return n > 0 ? total / n : 1e9;
}

// The lookahead lower bound is tuned, not fixed. At the benchmark speeds the
// clamp is what actually binds -- 0.2 * 0.5 + 0.2 is below any sensible floor --
// so holding lookahead_min constant would leave the low-speed behaviour outside
// the search entirely, which is how the first version of this sweep ended up
// selecting a lookahead that limit-cycles on tight corners.
PurePursuitParams tunePurePursuit(const std::vector<double> & speeds)
{
  PurePursuitParams best;
  double best_cost = 1e18;
  bool found = false;
  // The grid deliberately extends below any lookahead one would choose by hand,
  // so that the selected value can be seen to be an interior optimum rather
  // than whatever the lower bound happened to be.
  for (double gain = 0.05; gain <= 1.401; gain += 0.15) {
    for (double base = 0.0; base <= 0.601; base += 0.3) {
      for (double lmin = 0.15; lmin <= 1.351; lmin += 0.3) {
        PurePursuitParams p;
        p.lookahead_gain = gain;
        p.lookahead_base = base;
        p.lookahead_min = lmin;
        p.lookahead_max = 3.0;
        shuttle::PurePursuit tracker(p);
        const double cost = tuningCost(tracker, speeds);
        if (!found || cost < best_cost) {
          best_cost = cost;
          best = p;
          found = true;
        }
      }
    }
  }
  std::cout << "  pure_pursuit  best cost " << best_cost
            << "  (gain " << best.lookahead_gain << ", base " << best.lookahead_base
            << ", L_min " << best.lookahead_min << ")\n";
  return best;
}

StanleyParams tuneStanley(const std::vector<double> & speeds)
{
  StanleyParams best;
  double best_cost = 1e18;
  bool found = false;
  for (double k_e = 0.5; k_e <= 5.01; k_e += 0.5) {
    for (double k_soft = 0.05; k_soft <= 1.201; k_soft += 0.2) {
      for (double l_fa = 0.1; l_fa <= 0.901; l_fa += 0.1) {
        StanleyParams p;
        p.k_e = k_e;
        p.k_soft = k_soft;
        p.front_axle_distance = l_fa;
        p.max_steer = 1.0;
        shuttle::Stanley tracker(p);
        const double cost = tuningCost(tracker, speeds);
        if (!found || cost < best_cost) {
          best_cost = cost;
          best = p;
          found = true;
        }
      }
    }
  }
  std::cout << "  stanley       best cost " << best_cost
            << "  (k_e " << best.k_e << ", k_soft " << best.k_soft
            << ", L_fa " << best.front_axle_distance << ")\n";
  return best;
}

bool writeGains(
  const std::string & path, const PurePursuitParams & pp, const StanleyParams & st)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << std::fixed << std::setprecision(4);
  out << "# Generated by controller_bench --tune. Do not edit by hand.\n"
      << "#\n"
      << "# Objective, averaged over straight_offset, circle, corner and campus_loop\n"
      << "# at every benchmark speed with noise off:\n"
      << "#\n"
      << "#   cost = RMS_cross_track / 0.05 m + 0.5 * control_effort / 5 rad/s^2\n"
      << "#\n"
      << "# plus a penalty of " << kFailurePenalty << " for any run that diverges,\n"
      << "# stalls or fails to reach the goal. heading_offset, figure_eight and\n"
      << "# sharp_turns are held out of tuning and reported on.\n"
      << "pure_pursuit_lookahead_gain: " << pp.lookahead_gain << "\n"
      << "pure_pursuit_lookahead_base: " << pp.lookahead_base << "\n"
      << "pure_pursuit_lookahead_min: " << pp.lookahead_min << "\n"
      << "pure_pursuit_lookahead_max: " << pp.lookahead_max << "\n"
      << "stanley_k_e: " << st.k_e << "\n"
      << "stanley_k_soft: " << st.k_soft << "\n"
      << "stanley_front_axle_distance: " << st.front_axle_distance << "\n"
      << "stanley_max_steer: " << st.max_steer << "\n";
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--out" && i + 1 < argc) {
      opt.out_dir = argv[++i];
    } else if (a == "--gains" && i + 1 < argc) {
      opt.gains_file = argv[++i];
    } else if (a == "--speeds" && i + 1 < argc) {
      opt.speeds.clear();
      for (const auto & s : split(argv[++i], ',')) {
        opt.speeds.push_back(std::stod(s));
      }
    } else if (a == "--dump-paths" && i + 1 < argc) {
      opt.dump_paths_dir = argv[++i];
    } else if (a == "--tune") {
      opt.tune = true;
    } else if (a == "--no-timeseries") {
      opt.timeseries = false;
    } else if (a == "--help" || a == "-h") {
      std::cout << "controller_bench [--out DIR] [--speeds a,b,c] [--tune] "
        "[--gains FILE] [--no-timeseries] [--dump-paths DIR]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  // Dump the reference geometry so the Python path publisher used in Gazebo can
  // be checked against it (tests/test_scenarios_match.py). The two
  // implementations exist for good reasons -- one has to run without ROS, the
  // other without a compiler -- but duplicated geometry drifts unless something
  // compares it.
  if (!opt.dump_paths_dir.empty()) {
    for (const auto & name : shuttle::scenarioNames()) {
      shuttle::Scenario sc;
      if (!shuttle::makeScenario(name, 1.0, sc)) {
        continue;
      }
      const std::string path = opt.dump_paths_dir + "/" + name + ".csv";
      std::ofstream out(path);
      if (!out) {
        std::cerr << "cannot write " << path << "\n";
        return 1;
      }
      out << "x,y\n" << std::fixed << std::setprecision(6);
      for (const auto & p : sc.path.points()) {
        out << p.x << ',' << p.y << "\n";
      }
      std::cout << "Wrote " << path << " (" << sc.path.size() << " points)\n";
    }
    return 0;
  }

  PurePursuitParams pp;
  StanleyParams st;

  if (opt.tune) {
    std::cout << "Tuning gains (grid search, this is the reproducible tuning step)\n";
    pp = tunePurePursuit(opt.speeds);
    st = tuneStanley(opt.speeds);
    const std::string gains_path = opt.out_dir + "/tuned_gains.yaml";
    if (!writeGains(gains_path, pp, st)) {
      std::cerr << "failed to write " << gains_path
                << " (does the output directory exist?)\n";
      return 1;
    }
    std::cout << "Wrote " << gains_path << "\n";
  } else if (!opt.gains_file.empty()) {
    const auto g = readGains(opt.gains_file);
    if (g.empty()) {
      std::cerr << "failed to read gains from " << opt.gains_file << "\n";
      return 1;
    }
    pp.lookahead_gain = lookup(g, "pure_pursuit_lookahead_gain", pp.lookahead_gain);
    pp.lookahead_base = lookup(g, "pure_pursuit_lookahead_base", pp.lookahead_base);
    pp.lookahead_min = lookup(g, "pure_pursuit_lookahead_min", pp.lookahead_min);
    pp.lookahead_max = lookup(g, "pure_pursuit_lookahead_max", pp.lookahead_max);
    st.k_e = lookup(g, "stanley_k_e", st.k_e);
    st.k_soft = lookup(g, "stanley_k_soft", st.k_soft);
    st.front_axle_distance =
      lookup(g, "stanley_front_axle_distance", st.front_axle_distance);
    st.max_steer = lookup(g, "stanley_max_steer", st.max_steer);
    std::cout << "Loaded gains from " << opt.gains_file << "\n";
  }

  std::ofstream metrics(opt.out_dir + "/metrics.csv");
  if (!metrics) {
    std::cerr << "cannot write to " << opt.out_dir
              << " (create the directory first)\n";
    return 1;
  }
  metrics << shuttle::metricsCsvHeader() << "\n";

  int runs = 0;
  int failures = 0;

  for (const auto & scenario_name : shuttle::scenarioNames()) {
    for (double speed : opt.speeds) {
      for (bool noise : {false, true}) {
        for (int which = 0; which < 2; ++which) {
          std::unique_ptr<shuttle::PathTracker> tracker;
          if (which == 0) {
            tracker.reset(new shuttle::PurePursuit(pp));
          } else {
            tracker.reset(new shuttle::Stanley(st));
          }

          shuttle::Scenario sc;
          if (!shuttle::makeScenario(scenario_name, speed, sc)) {
            continue;
          }
          applyCommon(*tracker, speed);

          const auto cfg = makeConfig(scenario_name, speed, noise);
          const auto res = shuttle::runScenario(*tracker, sc, cfg);
          metrics << shuttle::metricsCsvRow(res.metrics) << "\n";
          ++runs;
          if (res.metrics.diverged || res.metrics.stalled || !res.metrics.goal_reached) {
            ++failures;
            const char * why = res.metrics.diverged ? " diverged" :
              (res.metrics.stalled ? " stalled" : " did not reach the goal");
            std::cout << "  [!] " << res.metrics.controller << " " << scenario_name
                      << " @ " << speed << " m/s" << (noise ? " +noise" : "") << why
                      << " (progress " << std::fixed << std::setprecision(1)
                      << 100.0 * res.metrics.progress << "%)\n";
          }

          if (opt.timeseries) {
            const std::string tag = std::string(tracker->name()) + "_" + scenario_name + "_" +
              speedTag(speed) + (noise ? "_noise" : "_clean");
            std::ofstream ts(opt.out_dir + "/timeseries/" + tag + ".csv");
            if (ts) {
              ts << shuttle::sampleCsvHeader() << "\n";
              for (const auto & s : res.samples) {
                ts << shuttle::sampleCsvRow(s) << "\n";
              }
            } else if (runs == 1) {
              std::cerr << "warning: cannot write time series to "
                        << opt.out_dir << "/timeseries/ (directory missing)\n";
            }
          }
        }
      }
    }
  }

  std::cout << "Completed " << runs << " runs, " << failures
            << " did not finish cleanly.\n";
  std::cout << "Metrics: " << opt.out_dir << "/metrics.csv\n";
  return 0;
}
