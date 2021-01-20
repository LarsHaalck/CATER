// Kernel density estimation by Tim Nugent (c) 2014
// Based on Philipp K. Janert's Perl module:
// http://search.cpan.org/~janert/Statistics-KernelEstimation-0.05
// Multivariate stuff from here:
// http://sfb649.wiwi.hu-berlin.de/fedc_homepage/xplore/ebooks/html/spm/spmhtmlnode18.html

#include <algorithm>
#include <cmath>
#include <kde.h>
#include <limits>
#include <map>
#include <numeric>
#include <vector>

using namespace std;

KDE::KDE(BandwidthType type)
    : bw_type(type)
    , sum_x(0)
    , sum_x2(0)
    , min(numeric_limits<double>::max())
    , max(numeric_limits<double>::min())
    , data()
{
}

double KDE::pdf(double x)
{
    double d = 0.0;
    for (unsigned int i = 0; i < data.size(); i++)
        d += gauss_pdf(x, data[i], bandwidth);
    return d;
}

double KDE::cdf(double x)
{
    double d = 0.0;
    for (unsigned int i = 0; i < data.size(); i++)
        d += gauss_cdf(x, data[i], bandwidth);
    return (d / data.size());
}

std::vector<double> KDE::pdf(const std::vector<double>& x)
{
    std::vector<double> y;
    y.resize(x.size());
    std::transform(std::begin(x), std::end(x), std::begin(y), [&](auto xi) { return pdf(xi); });
    return y;
}

std::vector<double> KDE::cdf(const std::vector<double>& x)
{
    std::vector<double> y;
    y.resize(x.size());
    std::transform(std::begin(x), std::end(x), std::begin(y), [&](auto xi) { return cdf(xi); });
    return y;
}

void KDE::calc_bandwidth()
{
    switch (bw_type)
    {
    case BandwidthType::Silverman:
        bandwidth = get_default_bandwidth();
        break;
    case BandwidthType::Secant:
        bandwidth = optimal_bandwidth_secant();
        break;
    case BandwidthType::Bisection:
        bandwidth = optimal_bandwidth_bisection();
        break;
    default:
        break;
    }
}

double KDE::get_default_bandwidth()
{
    double x = sum_x / data.size();
    double x2 = sum_x2 / data.size();
    double sigma = sqrt(x2 - (x * x));
    double b = sigma * (pow((3.0 * data.size() / 4.0), (-1.0 / 5.0)));
    return b;
}

// Secant method
double KDE::optimal_bandwidth_secant(int maxiters, double eps)
{
    auto default_bandwidth = get_default_bandwidth();
    double x0 = default_bandwidth;
    double y0 = optimal_bandwidth_equation(x0, get_min(), get_max());
    double x = 0.8 * x0;
    double y = optimal_bandwidth_equation(x, get_min(), get_max());
    int i = 0;

    while (i < maxiters)
    {
        x -= y * (x0 - x) / (y0 - y);
        y = optimal_bandwidth_equation(x, get_min(), get_max());
        if (abs(y) < eps * y0)
            break;
        i++;
    }
    return x;
}

double KDE::optimal_bandwidth_equation(double w, double mina, double maxa)
{
    double alpha = 1.0 / (2.0 * sqrt(M_PI));
    double sigma = 1.0;
    double n = data.size();
    double q = stiffness_integral(w, mina, maxa);
    return w - pow(((n * q * pow(sigma, 4)) / alpha), (-1.0 / 5.0));
}

// Calculates the integral over the square of the curvature using
// the trapezoidal rule. decreases step size by half until the relative
// error in the last step is less eps.
double KDE::stiffness_integral(double w, double mn, double mx)
{
    double eps = 1e-4;
    double nn = 1;
    double dx = (mx - mn) / nn;
    double curv_mx = curvature(mx, w);
    double curv_mn = curvature(mn, w);
    double yy = 0.5 * ((curv_mx * curv_mx) + (curv_mn * curv_mn)) * dx;
    double maxn = (mx - mn) / sqrt(eps);

    maxn = maxn > 2048 ? 2048 : maxn;

    for (int n = 2; n <= maxn; n *= 2)
    {
        dx /= 2.0;
        double y = 0.0;
        for (int i = 1; i <= n - 1; i += 2)
        {
            curv_mn = pow(curvature(mn + i * dx, w), 2);
            y += curv_mn;
        }
        yy = 0.5 * yy + y * dx;
        if (n > 8 && abs(y * dx - 0.5 * yy) < eps * yy)
        {
            break;
        }
    }
    return (yy);
}

double KDE::optimal_bandwidth_bisection(double eps)
{
    auto default_bandwidth = get_default_bandwidth();
    double x0 = default_bandwidth / data.size();
    double x1 = 2 * default_bandwidth;
    double y0 = optimal_bandwidth_equation(x0, min, max);

    double x = 0.0, y = 0.0;
    int i = 0;

    while (abs(x0 - x1) > eps * x1)
    {
        i += 1;
        x = (x0 + x1) / 2;
        y = optimal_bandwidth_equation(x, min, max);

        if (abs(y) < eps * y0)
            break;
        if (y * y0 < 0)
            x1 = x;
        else
        {
            x0 = x;
            y0 = y;
        }
    }
    return x;
}

double KDE::curvature(double x, double w)
{

    double y = 0.0;
    for (auto it = data.begin(); it != data.end(); it++)
        y += gauss_curvature(x, *it, w);
    return (y / data.size());
}

/* void KDE::fit(std::vector<double>&& x) */
/* { */
/*     data = x; */
/*     sum_x = std::accumulate(std::begin(data), std::end(data), 0.0); */
/*     sum_x2 = std::inner_product(std::begin(data), std::end(data), std::begin(data), 0.0); */
/*     min = *std::min_element(std::begin(data), std::end(data)); */
/*     max = *std::max_element(std::begin(data), std::end(data)); */
/*     calc_bandwidth(); */
/* } */

/* void KDE::add_data(double x) */
/* { */
/*     if (!data.size()) */
/*     { */
/*         data.push_back(x); */
/*         sum_x = x; */
/*         sum_x2 = x * x; */
/*         min = x; */
/*         max = x; */
/*     } */
/*     else */
/*     { */
/*         data.push_back(x); */
/*         sum_x += x; */
/*         sum_x2 += x * x; */
/*         min = x < min ? x : min; */
/*         max = x > max ? x : max; */
/*     } */
/* } */

double KDE::gauss_cdf(double x, double m, double s)
{
    // Abramowitz Stegun Normal Cumulative Distribution Function
    double z = abs(x - m) / s;
    double t = 1.0 / (1.0 + 0.2316419 * z);
    double y = t
        * (0.319381530
            + t * (-0.356563782 + t * (1.781477937 + t * (-1.821255978 + t * 1.330274429))));
    if (x >= m)
    {
        return 1.0 - gauss_pdf(x, m, s) * y * s;
    }
    else
    {
        return gauss_pdf(x, m, s) * y * s;
    }
}

double KDE::gauss_pdf(double x, double m, double s)
{

    double z = (x - m) / s;
    return exp(-0.5 * z * z) / (s * sqrt(2.0 * M_PI));
}

double KDE::gauss_curvature(double x, double m, double s)
{

    double z = (x - m) / s;
    return ((z * z) - 1.0) * gauss_pdf(x, m, s) / (s * s);
}
