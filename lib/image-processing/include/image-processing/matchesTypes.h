#ifndef HT_MATCHES_TYPES_H
#define HT_MATCHES_TYPES_H

#include <opencv2/core.hpp>
#include "util/pairHash.h"

namespace ht
{
using Trafo = cv::Mat;
using Trafos = std::vector<cv::Mat>;
using Match = cv::DMatch;
using Matches = std::vector<cv::DMatch>;

using PairwiseMatches = std::unordered_map<std::pair<std::size_t, std::size_t>, Matches, hash>;
using PairwiseTrafos = std::unordered_map<std::pair<std::size_t, std::size_t>, Trafo, hash>;
} // namespace ht

#endif // HT_MATCHES_TYPES_H
