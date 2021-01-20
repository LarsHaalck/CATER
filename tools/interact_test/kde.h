// Kernel density estimation by Tim Nugent (c) 2014
#ifndef KDE_H
#define KDE_H

#include <algorithm>
#include <numeric>
#include <vector>

class KDE
{
public:
    enum class BandwidthType
    {
        Silverman,
        Secant,
        Bisection
    };

    KDE(BandwidthType type);

    template <typename T>
    void fit(T&& x)
    {
        data = x;
        sum_x = std::accumulate(std::begin(data), std::end(data), 0.0);
        sum_x2 = std::inner_product(std::begin(data), std::end(data), std::begin(data), 0.0);
        min = *std::min_element(std::begin(data), std::end(data));
        max = *std::max_element(std::begin(data), std::end(data));
        calc_bandwidth();
    }

    double get_min() { return (min - (3 * get_default_bandwidth())); };
    double get_max() { return (max + (3 * get_default_bandwidth())); };

    double get_real_min() { return min; };
    double get_real_max() { return max; };

    double pdf(double x);
    double cdf(double x);
    std::vector<double> pdf(const std::vector<double>& x);
    std::vector<double> cdf(const std::vector<double>& x);

    double get_bandwidth() { return bandwidth; };

private:
    void calc_bandwidth();
    double get_default_bandwidth();
    double optimal_bandwidth_secant(int maxiters = 25, double eps = 1e-03);
    double optimal_bandwidth_bisection(double eps = 1e-03);
    double gauss_cdf(double x, double m, double s);
    double gauss_pdf(double x, double m, double s);
    double gauss_curvature(double x, double m, double s);
    double optimal_bandwidth_equation(double w, double min, double max);
    double stiffness_integral(double w, double min, double max);
    double curvature(double x, double w);

private:
    BandwidthType bw_type;
    double bandwidth;

    double sum_x, sum_x2, min, max;
    std::vector<double> data;
};

#endif // KDE_H
