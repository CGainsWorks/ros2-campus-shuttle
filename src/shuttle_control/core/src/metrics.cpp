// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "shuttle_control/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace shuttle
{

ErrorStats computeStats(const std::vector<double> & values)
{
  ErrorStats st;
  if (values.empty()) {
    return st;
  }
  std::vector<double> abs_vals;
  abs_vals.reserve(values.size());
  double sum_abs = 0.0;
  double sum_sq = 0.0;
  for (double v : values) {
    const double a = std::fabs(v);
    abs_vals.push_back(a);
    sum_abs += a;
    sum_sq += v * v;
  }
  st.count = values.size();
  st.mean_abs = sum_abs / static_cast<double>(values.size());
  st.rms = std::sqrt(sum_sq / static_cast<double>(values.size()));
  st.final_abs = std::fabs(values.back());

  std::sort(abs_vals.begin(), abs_vals.end());
  st.max_abs = abs_vals.back();
  // Nearest-rank 95th percentile.
  const std::size_t rank =
    static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(abs_vals.size())));
  const std::size_t idx = rank == 0 ? 0 : std::min(rank - 1, abs_vals.size() - 1);
  st.p95_abs = abs_vals[idx];
  return st;
}

RunMetrics summarise(
  const std::vector<Sample> & samples,
  const std::string & controller,
  const std::string & scenario,
  double target_speed,
  bool noise,
  double path_length,
  bool goal_reached,
  bool diverged,
  bool stalled)
{
  RunMetrics m;
  m.controller = controller;
  m.scenario = scenario;
  m.target_speed = target_speed;
  m.noise = noise;
  m.path_length = path_length;
  m.goal_reached = goal_reached;
  m.diverged = diverged;
  m.stalled = stalled;

  if (samples.empty()) {
    return m;
  }

  std::vector<double> ct;
  std::vector<double> hd;
  ct.reserve(samples.size());
  hd.reserve(samples.size());

  double sum_v = 0.0;
  double sum_omega_sq = 0.0;
  double sum_domega_sq = 0.0;
  std::size_t domega_count = 0;
  double dist = 0.0;
  double max_s = 0.0;
  double max_lat = 0.0;

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const Sample & s = samples[i];
    ct.push_back(s.cross_track);
    hd.push_back(s.heading);
    sum_v += std::fabs(s.v);
    sum_omega_sq += s.omega * s.omega;
    max_s = std::max(max_s, s.s);
    max_lat = std::max(max_lat, std::fabs(s.v * s.omega));

    if (i > 0) {
      const Sample & p = samples[i - 1];
      dist += std::hypot(s.x - p.x, s.y - p.y);
      const double dt = s.t - p.t;
      if (dt > 1e-9) {
        const double d = (s.omega_cmd - p.omega_cmd) / dt;
        sum_domega_sq += d * d;
        ++domega_count;
      }
    }
  }

  m.cross_track = computeStats(ct);
  m.heading = computeStats(hd);
  m.duration = samples.back().t - samples.front().t;
  m.distance_travelled = dist;
  m.progress = path_length > 1e-9 ? std::min(1.0, max_s / path_length) : 0.0;
  m.mean_speed = sum_v / static_cast<double>(samples.size());
  m.rms_omega = std::sqrt(sum_omega_sq / static_cast<double>(samples.size()));
  m.control_effort =
    domega_count > 0 ? std::sqrt(sum_domega_sq / static_cast<double>(domega_count)) : 0.0;
  m.max_lateral_accel = max_lat;
  return m;
}

std::string sampleCsvHeader()
{
  return "t,x,y,yaw,x_meas,y_meas,yaw_meas,path_x,path_y,s,cross_track,heading,"
         "v_cmd,omega_cmd,v,omega,kappa";
}

std::string sampleCsvRow(const Sample & s)
{
  std::ostringstream os;
  os.precision(6);
  os << std::fixed;
  os << s.t << ',' << s.x << ',' << s.y << ',' << s.yaw << ','
     << s.x_meas << ',' << s.y_meas << ',' << s.yaw_meas << ','
     << s.path_x << ',' << s.path_y << ',' << s.s << ','
     << s.cross_track << ',' << s.heading << ','
     << s.v_cmd << ',' << s.omega_cmd << ',' << s.v << ',' << s.omega << ',' << s.kappa;
  return os.str();
}

std::string metricsCsvHeader()
{
  return "controller,scenario,target_speed,noise,"
         "ct_mean_abs,ct_rms,ct_max_abs,ct_p95_abs,ct_final_abs,"
         "hd_mean_abs,hd_rms,hd_max_abs,hd_p95_abs,hd_final_abs,"
         "duration,path_length,distance_travelled,progress,goal_reached,diverged,stalled,"
         "mean_speed,control_effort,rms_omega,max_lateral_accel,samples";
}

std::string metricsCsvRow(const RunMetrics & m)
{
  std::ostringstream os;
  os.precision(6);
  os << std::fixed;
  os << m.controller << ',' << m.scenario << ',' << m.target_speed << ','
     << (m.noise ? 1 : 0) << ','
     << m.cross_track.mean_abs << ',' << m.cross_track.rms << ','
     << m.cross_track.max_abs << ',' << m.cross_track.p95_abs << ','
     << m.cross_track.final_abs << ','
     << m.heading.mean_abs << ',' << m.heading.rms << ','
     << m.heading.max_abs << ',' << m.heading.p95_abs << ','
     << m.heading.final_abs << ','
     << m.duration << ',' << m.path_length << ',' << m.distance_travelled << ','
     << m.progress << ',' << (m.goal_reached ? 1 : 0) << ',' << (m.diverged ? 1 : 0) << ','
     << (m.stalled ? 1 : 0) << ','
     << m.mean_speed << ',' << m.control_effort << ',' << m.rms_omega << ','
     << m.max_lateral_accel << ',' << m.cross_track.count;
  return os.str();
}

}  // namespace shuttle
