#include "utils.hpp"

namespace utils
{
    void findHistogramCounts(unsigned int nBins, std::vector<double> &vals,
                             Eigen::VectorXd &binCounts, Eigen::VectorXd &edges)
    {
        double maxVal = *std::max_element(&vals[0], &vals[0]+vals.size());
        double minVal = *std::min_element(&vals[0], &vals[0]+vals.size());
        double binSize = (maxVal - minVal)/nBins;
        edges.resize(nBins+1);
        edges = Eigen::VectorXd::LinSpaced(nBins+1, minVal, maxVal);

        for (size_t i = 0; i < vals.size(); i++)
        {
            double valFromLeft = vals[i] - minVal;
            double valFromRight = maxVal - vals[i];
            int ind;
            if (valFromLeft < 1e-3)
            {
                ind = 0;
            } else if (valFromRight < 1e-3)
            {
                ind = nBins - 1;
            } else
            {
                ind = ceil((vals[i] - minVal) / binSize) - 1;
            }
            binCounts(ind) = binCounts(ind) + 1;
        }
    }

    double findMedian(std::vector<double> &a) 
    {
        int n  = a.size();
        std::sort(a.begin(), a.end());

        if (n  % 2 != 0)
        {
            return a[n/2];
        } else
        {
            return (a[(n-1)/2] + a[(n)/2])/2;
        }
    }

    int otsu(Eigen::VectorXd histogramCounts)
    {
        int level = -1; // default value
        double total = histogramCounts.sum();
        int top = histogramCounts.rows();
        double sumB = 0;
        double wB = 0;
        double maximum = 0;
        double sum1 = histogramCounts.dot(Eigen::VectorXd::LinSpaced(histogramCounts.rows(), 0, top-1));

        for (int ii = 0; ii < top; ii++)
        {
            double wF = total - wB;
            if (wB > 0 && wF > 0)
            {
                double mF = (sum1 - sumB) / wF;
                double val = wB * wF * ((sumB / wB) - mF) * ((sumB / wB) - mF);
                if (val >= maximum)
                {
                    level = ii + 1;
                    maximum = val;
                }
            }
            wB = wB + histogramCounts(ii);
            sumB = sumB + (ii) * histogramCounts(ii);
        }
        return level;
    }

}
