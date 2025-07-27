#include "geodetic_transformations/geodetic_transformations.hpp"

#include <iostream>  // For debug output, can be removed in production

// マクロ定数 PIの定義
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 平面直角座標系の基準点データ
// {systemId, 基準緯度(度), 基準経度(度)}
// 環境省 自然環境局 測量計画図 縮尺データに基づく
// https://www.env.go.jp/nature/doing/ekakei/kibo/pdf/houi_keisankata.pdf
// (ただし、実際の計算式は国土地理院のものを参照)
const double REFERENCE_POINTS[19][3] = {
  {1, 33 + 0.0 / 60.0 + 0.0 / 3600.0,
   129 + 30.0 / 60.0 + 0.0 / 3600.0},  // I系: 33°00′00″N, 129°30′00″E
  {2, 33 + 0.0 / 60.0 + 0.0 / 3600.0,
   131 + 0.0 / 60.0 + 0.0 / 3600.0},  // II系: 33°00′00″N, 131°00′00″E
  {3, 36 + 0.0 / 60.0 + 0.0 / 3600.0,
   132 + 10.0 / 60.0 + 30.0 / 3600.0},  // III系: 36°00′00″N, 132°10′30″E (変更される可能性あり)
  {4, 33 + 0.0 / 60.0 + 0.0 / 3600.0,
   133 + 0.0 / 60.0 + 0.0 / 3600.0},  // IV系: 33°00′00″N, 133°00′00″E
  {5, 36 + 0.0 / 60.0 + 0.0 / 3600.0,
   134 + 0.0 / 60.0 + 0.0 / 3600.0},  // V系: 36°00′00″N, 134°00′00″E
  {6, 36 + 0.0 / 60.0 + 0.0 / 3600.0,
   136 + 0.0 / 60.0 + 0.0 / 3600.0},  // VI系: 36°00′00″N, 136°00′00″E
  {7, 36 + 0.0 / 60.0 + 0.0 / 3600.0,
   137 + 0.0 / 60.0 + 0.0 / 3600.0},  // VII系: 36°00′00″N, 137°00′00″E
  {8, 36 + 0.0 / 60.0 + 0.0 / 3600.0,
   138 + 0.0 / 60.0 + 0.0 / 3600.0},  // VIII系: 36°00′00″N, 138°00′00″E
  {9, 36 + 0.0 / 60.0 + 0.0 / 3600.0,
   139 + 0.0 / 60.0 + 0.0 / 3600.0},  // IX系: 36°00′00″N, 139°00′00″E
  {10, 40 + 0.0 / 60.0 + 0.0 / 3600.0,
   140 + 0.0 / 60.0 + 0.0 / 3600.0},  // X系: 40°00′00″N, 140°00′00″E
  {11, 40 + 0.0 / 60.0 + 0.0 / 3600.0,
   142 + 0.0 / 60.0 + 0.0 / 3600.0},  // XI系: 40°00′00″N, 142°00′00″E
  {12, 44 + 0.0 / 60.0 + 0.0 / 3600.0,
   142 + 0.0 / 60.0 + 0.0 / 3600.0},  // XII系: 44°00′00″N, 142°00′00″E
  {13, 44 + 0.0 / 60.0 + 0.0 / 3600.0,
   144 + 0.0 / 60.0 + 0.0 / 3600.0},  // XIII系: 44°00′00″N, 144°00′00″E
  {14, 26 + 0.0 / 60.0 + 0.0 / 3600.0,
   127 + 30.0 / 60.0 + 0.0 / 3600.0},  // XIV系: 26°00′00″N, 127°30′00″E
  {15, 26 + 0.0 / 60.0 + 0.0 / 3600.0,
   124 + 0.0 / 60.0 + 0.0 / 3600.0},  // XV系: 26°00′00″N, 124°00′00″E
  {16, 26 + 0.0 / 60.0 + 0.0 / 3600.0,
   141 + 0.0 / 60.0 + 0.0 / 3600.0},  // XVI系: 26°00′00″N, 141°00′00″E
  {17, 26 + 0.0 / 60.0 + 0.0 / 3600.0,
   157 + 0.0 / 60.0 + 0.0 / 3600.0},  // XVII系: 26°00′00″N, 157°00′00″E
  {18, 20 + 0.0 / 60.0 + 0.0 / 3600.0,
   136 + 0.0 / 60.0 + 0.0 / 3600.0},  // XVIII系: 20°00′00″N, 136°00′00″E
  {19, 26 + 0.0 / 60.0 + 0.0 / 3600.0,
   130 + 0.0 / 60.0 + 0.0 / 3600.0}  // XIX系: 26°00′00″N, 130°00′00″E
};

GeodeticConverter::GeodeticConverter(int systemId) : systemId_(systemId)
{
  setReferencePoint(systemId_);
}

void GeodeticConverter::setReferencePoint(int systemId)
{
  if (systemId < 1 || systemId > 19) {
    throw std::out_of_range("Invalid system ID. Must be between 1 and 19.");
  }
  phi0_rad_ = degToRad(REFERENCE_POINTS[systemId - 1][1]);
  lambda0_rad_ = degToRad(REFERENCE_POINTS[systemId - 1][2]);
}

double GeodeticConverter::degToRad(double deg) const { return deg * M_PI / 180.0; }

double GeodeticConverter::radToDeg(double rad) const { return rad * 180.0 / M_PI; }

// 子午線弧長 M(φ) の計算
double GeodeticConverter::calculateM(double phi_rad) const
{
  double e = std::sqrt(E2);  // 離心率
  double e2 = E2;
  double e4 = e2 * e2;
  double e6 = e4 * e2;

  double M = A * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi_rad -
                  (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * std::sin(2 * phi_rad) +
                  (15 * e4 / 256 + 45 * e6 / 1024) * std::sin(4 * phi_rad) -
                  (35 * e6 / 3072) * std::sin(6 * phi_rad));
  return M;
}

// 補助関数 T(φ)
double GeodeticConverter::calculateT(double phi_rad) const
{
  double tan_phi = std::tan(phi_rad);
  return tan_phi * tan_phi;
}

// 補助関数 D(λ-λ0)
double GeodeticConverter::calculateD(double delta_lambda_rad) const { return delta_lambda_rad; }

// 緯度経度から平面直角座標へ変換
PlaneXY GeodeticConverter::convertLatLonToPlaneXY(const LatLon & latLon) const
{
  double phi_rad = degToRad(latLon.latitude);
  double lambda_rad = degToRad(latLon.longitude);
  double delta_lambda_rad = lambda_rad - lambda0_rad_;

  double N = A / std::sqrt(1.0 - E2 * std::sin(phi_rad) * std::sin(phi_rad));  // 卯酉線曲率半径
  double M0 = calculateM(phi0_rad_);  // 基準緯度の子午線弧長
  double M = calculateM(phi_rad);     // 現在緯度の子午線弧長

  double t = calculateT(phi_rad);                                       // tan^2(phi)
  double n_squared = E_PRIME2 * std::cos(phi_rad) * std::cos(phi_rad);  // (e'^2 * cos^2(phi))

  double sin_phi = std::sin(phi_rad);
  double cos_phi = std::cos(phi_rad);

  // X座標の計算
  double term1_X = N * sin_phi * cos_phi;
  double term2_X = (N / 6.0) * sin_phi * std::pow(cos_phi, 3.0) * (1.0 - t + n_squared);
  double term3_X = (N / 120.0) * sin_phi * std::pow(cos_phi, 5.0) *
                   (5.0 - 18.0 * t + t * t + 14.0 * n_squared - 58.0 * t * n_squared);

  double x = M - M0 + term1_X * (std::pow(delta_lambda_rad, 2.0) / 2.0) +
             term2_X * (std::pow(delta_lambda_rad, 4.0) / 24.0) +
             term3_X * (std::pow(delta_lambda_rad, 6.0) / 720.0);

  // Y座標の計算
  double term1_Y = N * cos_phi;
  double term2_Y = (N / 6.0) * std::pow(cos_phi, 3.0) * (1.0 - t + n_squared);
  double term3_Y = (N / 120.0) * std::pow(cos_phi, 5.0) *
                   (5.0 - 18.0 * t + t * t + 14.0 * n_squared - 58.0 * t * n_squared);

  double y = term1_Y * delta_lambda_rad + term2_Y * (std::pow(delta_lambda_rad, 3.0) / 6.0) +
             term3_Y * (std::pow(delta_lambda_rad, 5.0) / 120.0);

  // 平面直角座標系では、X座標に+100km、Y座標に+0km（中央子午線上に設定）のオフセットが加えられることが多い。
  // 日本の測量ではX座標は原点から北へ向かって正、Y座標は原点から東へ向かって正。
  // ただし、この実装では純粋な計算結果を返す。必要に応じてオフセットを追加する。
  // 例: x += 100000.0; // X座標を100kmシフトする場合

  return {x, y};
}

// 平面直角座標から緯度経度へ変換 (逆計算はより複雑)
LatLon GeodeticConverter::convertPlaneXYToLatLon(const PlaneXY & planeXY) const
{
  // 逆変換は非常に複雑な反復計算が必要
  // ここでは簡略化のため、概略的な計算、または一部のみ実装
  // 実際の逆変換には反復計算アルゴリズムが必要となる

  double x_prime = planeXY.x;  // + 100000.0; // X座標のオフセットを考慮する場合はここで調整
  double y_prime = planeXY.y;

  double m_prime = calculateM(phi0_rad_) + x_prime;  // 基準緯度の子午線弧長 + X座標

  // φ' の初期値 (反復計算の初期近似)
  double phi_prime_rad = m_prime / A;

  double nu_prime;           // 卯酉線曲率半径の近似値
  double rho_prime;          // 子午線曲率半径の近似値
  double eta_prime_squared;  // 第2離心率の2乗 * cos^2(phi_prime_rad)

  // φ' の反復計算 (数回繰り返すことで精度を高める)
  for (int i = 0; i < 5; ++i)  // 5回程度の反復で十分な精度が得られることが多い
  {
    nu_prime = A / std::sqrt(1.0 - E2 * std::sin(phi_prime_rad) * std::sin(phi_prime_rad));
    rho_prime =
      A * (1.0 - E2) / std::pow(1.0 - E2 * std::sin(phi_prime_rad) * std::sin(phi_prime_rad), 1.5);
    eta_prime_squared = E_PRIME2 * std::cos(phi_prime_rad) * std::cos(phi_prime_rad);

    double term_phi_prime = (m_prime - calculateM(phi_prime_rad)) /
                            rho_prime;  // M0が既に考慮されているためm_primeからM(phi)を引く
    phi_prime_rad = phi_prime_rad + term_phi_prime;
  }

  double t_prime = std::tan(phi_prime_rad) * std::tan(phi_prime_rad);
  double d_prime = y_prime / nu_prime;  // Y' / N'

  // 経度の計算
  double term1_lambda = d_prime;
  double term2_lambda = (1.0 + 2.0 * t_prime + eta_prime_squared) * std::pow(d_prime, 3.0) / 6.0;
  double term3_lambda = (5.0 + 28.0 * t_prime + 24.0 * t_prime * t_prime + 6.0 * eta_prime_squared +
                         8.0 * t_prime * eta_prime_squared) *
                        std::pow(d_prime, 5.0) / 120.0;

  double delta_lambda_rad =
    (term1_lambda - term2_lambda + term3_lambda) / std::cos(phi_prime_rad);  // (λ - λ0)
  double lambda_rad = lambda0_rad_ + delta_lambda_rad;                       // 経度

  // 緯度の計算
  double term1_phi = t_prime;
  double term2_phi = (1.0 + t_prime - eta_prime_squared) * std::pow(d_prime, 2.0) / 2.0;
  double term3_phi =
    (5.0 - 2.0 * t_prime + 3.0 * eta_prime_squared + 4.0 * eta_prime_squared * eta_prime_squared -
     9.0 * t_prime * eta_prime_squared) *
    std::pow(d_prime, 4.0) / 24.0;
  double term4_phi =
    (61.0 - 148.0 * t_prime + 16.0 * t_prime * t_prime) * std::pow(d_prime, 6.0) / 720.0;

  double phi_rad =
    phi_prime_rad - (term2_phi - term3_phi + term4_phi) * std::sin(phi_prime_rad);  // 緯度

  return {radToDeg(phi_rad), radToDeg(lambda_rad)};
}
