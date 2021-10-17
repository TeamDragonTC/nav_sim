#ifndef _NOISE_HPP_
#define _NOISE_HPP_

#include <geometry_msgs/Twist.h>

#include <cmath>
#include <random>
#include <nav_sim/data_struct.hpp>

class Noise
{
public:
  Noise(double distance_noise_rate, double direction_noise)
  : distance_noise_rate_(distance_noise_rate), direction_noise_(direction_noise)
  {
    // 移動に対して発生する雑音
    distance_until_noise_ = getExponentialDistribution(1.0 / 5.0);
    // 速度に対するバイアス誤差
    bias_rate_ = getGaussDistribution(1.0, 0.1);
    // スタック
    time_until_escape_ = getExponentialDistribution(1.0 / 60.0);
    time_until_stuck_ = getExponentialDistribution(1.0 / 60.0);
    // センサ値に対するバイアス
    distance_noise_std_ = getGaussDistribution(0.0, distance_noise_rate_);
    direction_noise_std_ = getGaussDistribution(0.0, direction_noise_);
  }
  ~Noise() = default;

  double bias(double input)
  {
    return input * bias_rate_;
  }
  double getExponentialDistribution(double parameter)
  {
    std::random_device seed;
    std::default_random_engine engine(seed());
    std::exponential_distribution<> exponential(parameter);
    return exponential(engine);
  }
  double getGaussDistribution(double mean, double std)
  {
    std::random_device seed;
    std::default_random_engine engine(seed());
    std::normal_distribution<> gauss(mean, std);
    return gauss(engine);
  }

  void stuck(double& velocity, double& omega, double time_interval)
  {
    if (is_stuck_) {
      time_until_escape_ -= time_interval;
      if (time_until_escape_ <= 0.0) {
        time_until_escape_ += getExponentialDistribution(1.0 / 60.0);
        is_stuck_ = false;
        return;
      }
      velocity = 0.0;
      omega = 0.0;
    } else {
      time_until_stuck_ -= time_interval;
      if (time_until_stuck_ <= 0.0) {
        time_until_stuck_ += getExponentialDistribution(1.0 / 60.0);
        is_stuck_ = true;
      }
    }
  }
  std::pair<double, double> observationNoise(const std::pair<double, double> position)
  {
    const auto distance = position.first;
    const auto degree = position.second;

    const double gauss_for_distance = getGaussDistribution(distance, distance * distance_noise_rate_);
    const double gauss_for_degree = getGaussDistribution(degree, direction_noise_);

    return std::make_pair(gauss_for_distance, gauss_for_degree);
  }
  std::pair<double, double> observationBias(const std::pair<double, double> position)
  {
    const auto distance = position.first;
    const auto degree = position.second;

    return std::make_pair(distance + distance * distance_noise_std_, degree + direction_noise_std_);
  }
  void noise(State& state, geometry_msgs::Twist twist, double time_interval)
  {
    distance_until_noise_ -=
      ((std::fabs(twist.linear.x) * time_interval + std::fabs(twist.angular.z) * time_interval));
    if (distance_until_noise_ <= 0.0) {
      distance_until_noise_ += getExponentialDistribution(1.0 / 5.0);
      state.yaw_ += getGaussDistribution(0.0, M_PI / 60.0);
    }
}

private:
  double distance_noise_rate_;
  double distance_noise_std_;

  double direction_noise_;
  double direction_noise_std_;

  double distance_until_noise_;

  double bias_rate_;

  double time_until_stuck_;
  double time_until_escape_;
  bool is_stuck_{ false };
};

#endif
