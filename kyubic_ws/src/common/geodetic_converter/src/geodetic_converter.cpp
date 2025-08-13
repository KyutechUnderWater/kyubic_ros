#include "geodetic_converter/geodetic_converter.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace common
{

GeodeticConverter::GeodeticConverter(int systemId) : systemId_(systemId)
{
  setReferencePoint(systemId_);
}

void GeodeticConverter::setReferencePoint(int systemId)
{
  if (systemId < 1 || systemId > 19) {
    throw std::out_of_range("Invalid system ID. Must be between 1 and 19.");
  }
  phi0_rad_ = deg2rad(dms2deg(ORIGIN_POINTS_DMS[systemId - 1][0]));
  lambda0_rad_ = deg2rad(dms2deg(ORIGIN_POINTS_DMS[systemId - 1][1]));
  S_bar_phi0_ = calc_S_bar_phi0(phi0_rad_);
}

double GeodeticConverter::dms2deg(const std::array<double, 3> & dms)
{
  return static_cast<double>(dms[0]) + static_cast<double>(dms[1]) / 60.0 +
         static_cast<double>(dms[2]) / 3600.0;
}

std::array<double, 3> GeodeticConverter::deg2dmg(const double & deg) const
{
  double d = (int)deg;
  double _m = (deg - d) * 60;
  double m = (int)_m;
  double s = (_m - m) * 60;

  return std::array<double, 3>({d, m, s});
}

double GeodeticConverter::deg2rad(double deg) const { return deg * M_PI / 180.0; }

double GeodeticConverter::rad2deg(double rad) const { return rad * 180.0 / M_PI; }

Geodetic GeodeticConverter::getOrigin() { return {phi0_rad_, lambda0_rad_, 0.0, 0.0}; }

double GeodeticConverter::calc_S_bar_phi0(const double & phi0_rad) const
{
  double sigma = 0.0;
  for (int j = 1; j < 6; j++) {
    sigma += A[j] * sin(2 * j * phi0_rad);
  }
  return A_bar * phi0_rad + (m0 * a / (1 + n)) * sigma;
}

// 平面直角座標から緯度経度へ変換
PlaneXY GeodeticConverter::geo2xy(const Geodetic & latlon) const
{
  double phi_rad = deg2rad(latlon.latitude);
  double lambda_rad = deg2rad(latlon.longitude);

  double t =
    sinh(atanh(sin(phi_rad)) - 2 * sqrt(n) * atanh(2 * sqrt(n) * sin(phi_rad) / (1 + n)) / (1 + n));
  double t_bar = sqrt(1 + pow(t, 2));

  double lambda_c = cos(lambda_rad - lambda0_rad_);
  double lambda_s = sin(lambda_rad - lambda0_rad_);
  double xi_prime = atan(t / lambda_c);
  double eta_prime = atanh(lambda_s / t_bar);

  double x = A_bar * xi_prime - S_bar_phi0_;
  double y = A_bar * eta_prime;
  double sigma = 1.0;
  double tau = 0.0;
  for (int j = 1; j < 6; j++) {
    double _x = 2 * j * xi_prime;
    double _y = 2 * j * eta_prime;
    x += A_bar * alpha[j] * sin(_x) * cosh(_y);
    y += A_bar * alpha[j] * cos(_x) * sinh(_y);
    sigma += 2 * j * alpha[j] * cos(_x) * cosh(_y);
    tau += 2 * j * alpha[j] * sin(_x) * sinh(_y);
  }

  double gamma = atan(
    (tau * t_bar * lambda_c + sigma * t * lambda_s) /
    (sigma * t_bar * lambda_c - tau * t * lambda_s));

  double m = (A_bar / a) * sqrt(
                             ((pow(sigma, 2) + pow(tau, 2)) / (pow(t, 2) + pow(lambda_c, 2))) *
                             (1 + pow((1 - n) * tan(phi_rad) / (1 + n), 2)));

  return {x, y, rad2deg(gamma), m};
}

// 平面直角座標から緯度経度へ変換
Geodetic GeodeticConverter::xy2geo(const PlaneXY & planeXY) const
{
  double x = planeXY.x;
  double y = planeXY.y;

  double eta = y / A_bar;

  double xi = (x + S_bar_phi0_) / A_bar;

  double xi_prime = xi;
  double eta_prime = eta;
  double sigma_prime = 1.0;
  double tau_prime = 0.0;
  for (int j = 1; j < 6; j++) {
    double _x = 2 * j * xi;
    double _y = 2 * j * eta;

    xi_prime -= beta[j] * sin(_x) * cosh(_y);
    eta_prime -= beta[j] * cos(_x) * sinh(_y);
    sigma_prime -= 2 * j * beta[j] * cos(_x) * cosh(_y);
    tau_prime += 2 * j * beta[j] * sin(_x) * sinh(_y);
  }

  double chi = asin(sin(xi_prime) / cosh(eta_prime));

  double sigma = 0.0;
  for (int j = 1; j < 6; j++) {
    sigma += delta[j] * sin(2 * j * chi);
  }
  double t_phi = ((1 + n) / (1 - n)) * tan(chi + sigma);

  // Caluculate latitude and longitude
  double phi_rad = atan(((1 + n) / (1 - n)) * t_phi);
  double lambda_rad = lambda0_rad_ + atan(sinh(eta_prime) / cos(xi_prime));

  // Caluculate meridian convergence
  double tmp = tan(xi_prime) * tanh(eta_prime);
  double gamma = atan((tau_prime + sigma_prime * tmp) / (sigma_prime - tau_prime * tmp));

  // Caluculate scale coefficients
  double m = (A_bar / a) * sqrt(
                             (pow(cos(xi_prime), 2) + pow(sinh(eta_prime), 2)) *
                             (1 + pow(t_phi, 2)) / (pow(sigma_prime, 2) + pow(tau_prime, 2)));

  return {rad2deg(phi_rad), rad2deg(lambda_rad), rad2deg(gamma), m};
}

}  // namespace common
