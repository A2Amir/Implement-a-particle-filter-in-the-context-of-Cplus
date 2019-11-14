#include <iostream>
#include <cmath>
double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs, double mu_x, double mu_y);
int main()
{
  // define inputs
  double sig_x, sig_y, x_obs, y_obs, mu_x, mu_y;
  // define outputs for observations
  double weight1, weight2, weight3;
  // final weight
  double final_weight;

  // OBS1 values
  sig_x = 0.3;
  sig_y = 0.3;
  x_obs = 6;
  y_obs = 3;
  mu_x = 5;
  mu_y = 3;
  // Calculate OBS1 weight
  weight1 = multiv_prob(sig_x, sig_y, x_obs, y_obs, mu_x, mu_y);
  // should be around 0.00683644777551 rounding to 6.84E-3
  std::cout << "Weight1: " << weight1 << std::endl;

  // OBS2 values
  sig_x = 0.3;
  sig_y = 0.3;
  x_obs = 2;
  y_obs = 2;
  mu_x = 2;
  mu_y = 1;
  // Calculate OBS2 weight
  weight2 = multiv_prob(sig_x, sig_y, x_obs, y_obs, mu_x, mu_y);
  // should be around 0.00683644777551 rounding to 6.84E-3
  std::cout << "Weight2: " << weight2 << std::endl;

  // OBS3 values
  sig_x = 0.3;
  sig_y = 0.3;
  x_obs = 0;
  y_obs = 5;
  mu_x = 2;
  mu_y = 1;
  // Calculate OBS3 weight
  weight3 = multiv_prob(sig_x, sig_y, x_obs, y_obs, mu_x, mu_y);
  // should be around 9.83184874151e-49 rounding to 9.83E-49
  std::cout << "Weight3: " << weight3 << std::endl;

  // Output final weight
  final_weight = weight1 * weight2 * weight3;
  // 4.60E-53
  std::cout << "Final weight: " << final_weight << std::endl;

  return 0;
}

double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);

  return weight;
}
