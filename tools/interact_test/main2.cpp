// Kernel density estimation by Tim Nugent (c) 2014

#include <fstream>
#include <iostream>
#include <kde.h>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <memory>

using namespace std;

void usage(const char* prog)
{

    cout << "Read data from a csv file then perform kernel density estimation:\nUsage:\n"
         << prog << " [options] <csv_file>" << endl
         << endl;
    cout << "Options:" << endl;
    cout << "-b <int>   Bandwidth optimisation (Gaussian only):" << endl;
    cout << "           1 = Default" << endl;
    cout << "           2 = AMISE optimal, secant method" << endl;
    cout << "           3 = AMISE optimal, bisection method" << endl;
    cout << "-p <int>   Calculate:" << endl;
    cout << "           1 = PDF (default)" << endl;
    cout << "           2 = CDF" << endl;
    cout << endl;
}

int main(int argc, const char* argv[])
{

    auto kde = std::make_unique<KDE>(KDE::BandwidthType::Silverman);
    string output, line;
    int pdf = 1;

    if (argc < 2)
    {
        usage(argv[0]);
        return (1);
    }
    else
    {
        for (int i = 0; i < argc; i++)
        {
            if (string(argv[i]) == "-b" && i < argc - 1)
            {
                switch(atoi(argv[i+1]))
                {
                case 1:
                    kde = std::make_unique<KDE>(KDE::BandwidthType::Silverman);
                    break;
                case 2:
                    kde = std::make_unique<KDE>(KDE::BandwidthType::Secant);
                    break;
                case 3:
                    kde = std::make_unique<KDE>(KDE::BandwidthType::Bisection);
                    break;
                default:
                    break;
                }
            }
            if (string(argv[i]) == "-o" && i < argc - 1)
            {
                output = argv[i + 1];
            }
            if (string(argv[i]) == "-p" && i < argc - 1)
            {
                pdf = atoi(argv[i + 1]);
            }
        }
    }

    ifstream file(argv[argc - 1]);
    std::vector<double> data;
    while (getline(file, line))
        data.push_back(std::stod(line));
    file.close();

    kde->fit(data);


    double min_x = kde->get_min();
    double max_x = kde->get_max();
    double x_increment = (max_x - min_x) / 1000.0;
    for (double x = min_x; x <= max_x; x += x_increment)
    {
        if (pdf)
            printf("%2.6F\n", kde->pdf(x));
        else
            printf("%2.6F,%2.6F\n", x, kde->cdf(x));
    }

    return (0);
}
