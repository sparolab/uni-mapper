#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace utils
{
    /**
     * @brief Read strings and convert to numbers to correctly order the input
     *        files. No two files should have the same name.
     * 
     * @param a         The first string to compare.
     * @param b         The second string to compare.
     * @return true     If file a's name is smaller than file b's name.
     * @return false    If file b's name is smaller than file a's name.
     */
    bool compareStrings(std::string a, std::string b);

    /**
     * @brief Construct a homogeneous (4x4) matrix.
     * 
     * @param roll              Roll angle in radians.
     * @param pitch             Pitch angle in radians.
     * @param yaw               Yaw angle in radians.
     * @param x                 X position in metres.
     * @param y                 Y position in metres.
     * @param z                 Z position in metres.
     * @return Eigen::Matrix4d  The homogeneous (4x4) matrix constructed. 
     */
    Eigen::Matrix4d homogeneous(double roll, double pitch, double yaw, double x, double y, double z);

    /**
     * @brief Read pose estimates in the sensor frame. The file must be in the
     *        KITTI format. 
     * 
     * @param fileName                             Name of the pose estimates file.
     * @return std::vector<std::vector<double> >   The pose estimates read from the
     *                                             file stored in a (n by 12) 
     *                                             matrix where there an n sensor
     *                                             poses.
     */
    std::vector<std::vector<double> > readPoseEstimates(const std::string &fileName);

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