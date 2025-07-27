#include "geodetic_transformations/geodetic_transformations.hpp"
#include <cmath>
#include <vector>

namespace GSI {
namespace {
    // 定数定義
    constexpr double PI = 3.14159265358979323846;
    constexpr double a = 6377397.155;
    constexpr double F_inv = 299.1528128;
    constexpr double m0 = 0.9999;
    constexpr double f = 1.0 / F_inv;
    constexpr double n = f / (2.0 - f);

    // ★★★ 順変換用のα係数を正しく定義 ★★★
    const std::vector<double> alpha_coeffs = {
        0.0,
        1.0/2.0*n - 2.0/3.0*pow(n, 2) + 5.0/16.0*pow(n, 3) + 41.0/180.0*pow(n, 4) - 127.0/288.0*pow(n, 5),
        13.0/48.0*pow(n, 2) - 3.0/5.0*pow(n, 3) + 557.0/1440.0*pow(n, 4) + 281.0/630.0*pow(n, 5),
        61.0/240.0*pow(n, 3) - 103.0/140.0*pow(n, 4) + 15061.0/26880.0*pow(n, 5),
        49561.0/161280.0*pow(n, 4) - 179.0/168.0*pow(n, 5),
        34729.0/80640.0*pow(n, 5)
    };

    // 逆変換用のβ係数
    const std::vector<double> beta_coeffs = {
        0.0,
        1.0/2.0*n - 2.0/3.0*pow(n,2) + 37.0/96.0*pow(n,3) - 1.0/360.0*pow(n,4) - 81.0/512.0*pow(n,5),
        1.0/48.0*pow(n,2) + 1.0/15.0*pow(n,3) - 437.0/1440.0*pow(n,4) + 46.0/105.0*pow(n,5),
        17.0/480.0*pow(n,3) - 37.0/840.0*pow(n,4) - 209.0/4480.0*pow(n,5),
        4397.0/161280.0*pow(n,4) - 11.0/504.0*pow(n,5),
        4583.0/161280.0*pow(n,5)
    };

    // その他の係数...
    const std::vector<double> A_coeffs = {
        1.0 + (1.0/4.0)*pow(n, 2) + (1.0/64.0)*pow(n, 4),
        -3.0/2.0 * (n - (1.0/8.0)*pow(n, 3) - (1.0/64.0)*pow(n, 5)),
        15.0/16.0 * (pow(n, 2) - (1.0/4.0)*pow(n, 4)),
        -35.0/48.0 * (pow(n, 3) - (5.0/16.0)*pow(n, 5)),
        315.0/512.0 * pow(n, 4),
        -693.0/1280.0 * pow(n, 5)
    };
    const std::vector<double> delta_coeffs = {
        0.0,
        2.0*n - 2.0/3.0*pow(n,2) - 2.0*pow(n,3) + 116.0/45.0*pow(n,4) + 26.0/45.0*pow(n,5),
        7.0/3.0*pow(n,2) - 8.0/5.0*pow(n,3) - 227.0/45.0*pow(n,4) + 2704.0/315.0*pow(n,5),
        56.0/15.0*pow(n,3) - 136.0/35.0*pow(n,4) - 1262.0/105.0*pow(n,5),
        4279.0/630.0*pow(n,4) - 332.0/35.0*pow(n,5),
        4174.0/315.0*pow(n,5)
    };

    // ヘルパー関数
    double deg2rad(double deg) { return deg * PI / 180.0; }
    double rad2deg(double rad) { return rad * 180.0 / PI; }
}

XY bl2xy(double lat, double lon, double origin_lat, double origin_lon) {
    const double phi = deg2rad(lat);
    const double lambda = deg2rad(lon);
    const double phi_0 = deg2rad(origin_lat);
    const double lambda_0 = deg2rad(origin_lon);
    const double A_bar = (m0 * a) / (1.0 + n) * A_coeffs[0];

    double S_phi0_term = A_coeffs[0] * phi_0;
    for (size_t j = 1; j < A_coeffs.size(); ++j) {
        S_phi0_term += A_coeffs[j] * sin(2.0 * j * phi_0);
    }
    const double S_bar_phi0 = (m0 * a) / (1.0 + n) * S_phi0_term;

    const double t = sinh(atanh(sin(phi)) - (2.0 * sqrt(n)) / (1.0 + n) * atanh((2.0 * sqrt(n)) / (1.0 + n) * sin(phi)));
    const double t_bar = sqrt(1.0 + t*t);
    const double xi_prime = atan2(t, cos(lambda - lambda_0));
    const double eta_prime = atanh(sin(lambda - lambda_0) / t_bar);

    double x_term = xi_prime;
    double y_term = eta_prime;

    // ★★★ ここで正しい alpha_coeffs を使う ★★★
    for (size_t j = 1; j < alpha_coeffs.size(); ++j) {
        x_term += alpha_coeffs[j] * sin(2.0 * j * xi_prime) * cosh(2.0 * j * eta_prime);
        y_term += alpha_coeffs[j] * cos(2.0 * j * xi_prime) * sinh(2.0 * j * eta_prime);
    }
    return {A_bar * x_term - S_bar_phi0, A_bar * y_term};
}

LatLon xy2bl(double x, double y, double origin_lat, double origin_lon) {
    const double phi_0 = deg2rad(origin_lat);
    const double lambda_0 = deg2rad(origin_lon);
    
    double S_phi0_term = A_coeffs[0] * phi_0;
    for (size_t j = 1; j < A_coeffs.size(); ++j) {
        S_phi0_term += A_coeffs[j] * sin(2.0 * j * phi_0);
    }
    const double S_bar_phi0 = (m0 * a) / (1.0 + n) * S_phi0_term;
    const double A_bar = (m0 * a) / (1.0 + n) * A_coeffs[0];

    const double xi = (x + S_bar_phi0) / A_bar;
    const double eta = y / A_bar;
    
    double xi_prime = xi;
    double eta_prime = eta;
    for (size_t j = 1; j < beta_coeffs.size(); ++j) {
        xi_prime -= beta_coeffs[j] * sin(2.0 * j * xi) * cosh(2.0 * j * eta);
        eta_prime -= beta_coeffs[j] * cos(2.0 * j * xi) * sinh(2.0 * j * eta);
    }

    const double chi = asin(sin(xi_prime) / cosh(eta_prime));
    double phi = chi;
    for(size_t j = 1; j < delta_coeffs.size(); ++j) {
        phi += delta_coeffs[j] * sin(2.0 * j * chi);
    }

    const double lambda = lambda_0 + atan2(sinh(eta_prime), cos(xi_prime));
    
    return {rad2deg(phi), rad2deg(lambda)};
}

} // namespace GSI