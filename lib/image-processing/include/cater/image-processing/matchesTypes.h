#ifndef CT_MATCHES_TYPES_H
#define CT_MATCHES_TYPES_H

#include <opencv2/core.hpp>
#include <cater/util/pairHash.h>

namespace ct
{
using Trafo = cv::Mat;
using Trafos = std::vector<cv::Mat>;
using Match = cv::DMatch;
using Matches = std::vector<cv::DMatch>;

using PairwiseMatches = std::unordered_map<std::pair<std::size_t, std::size_t>, Matches, hash>;
using PairwiseTrafos = std::unordered_map<std::pair<std::size_t, std::size_t>, Trafo, hash>;
} // namespace ct

#endif // CT_MATCHES_TYPES_H
