#pragma once

#include <vector>

#include "eigen3/Eigen/Dense"

namespace utils
{
    /**
     * @brief Find the median of the elements.
     * 
     * @param a         The set of values to find the median of.
     * @return double   The median of the values in vector "a".
     */
    double findMedian(std::vector<double> &a);

    /**
     * @brief Find the histogram counts for a set of values.
     * 
     * @param nBins     The number of bins to discrteize the values into.
     * @param vals      The values to bin.
     * @param binCounts The result of the binning process.    
     * @param edges     The bin edges.
     */
    void findHistogramCounts(unsigned int nBins, std::vector<double> &vals,
                            Eigen::VectorXd &binCounts, Eigen::VectorXd &edges);

    /**
     * @brief Perform an automatic Otsu thresholding thresholding on the histogram
     *        counts.
     * 
     * @param histogramCounts   The histogram counts to automatically threshold.
     * @return int              The automatic threshold corresponding to the bin
     *                          number.
     */
    int otsu(Eigen::VectorXd histogramCounts);
}
