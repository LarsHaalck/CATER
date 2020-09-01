#include "image-processing/superGlue.h"

#include "image-processing/images.h"
#include "progressbar/progressBar.h"
#include "util/stopWatch.h"

#include <spdlog/spdlog.h>

#include <torch/script.h>
#include <torch/torch.h>

#include <opencv2/highgui.hpp>

using namespace torch;
using namespace torch::indexing;
namespace fs = std::filesystem;

using KeyPoints = std::vector<cv::KeyPoint>;

namespace ht::matches
{

namespace detail
{
    std::tuple<KeyPoints, KeyPoints, Matches> glue(const cv::Mat& imgI, const cv::Mat& imgJ,
        jit::script::Module& sp, jit::script::Module& sg, Device& device, float scale,
        int targetWidth, int targetHeight);

    inline cv::Mat prepare(const cv::Mat& img, int targetWidth, int targetHeight)
    {
        cv::Mat res;
        cvtColor(img, res, cv::COLOR_BGR2GRAY);
        res.convertTo(res, CV_32F, 1.0f / 255.0f);
        cv::resize(res, res, {targetWidth, targetHeight});
        return res;
    }

    inline std::tuple<Tensor, Tensor, Tensor> unpackResult(const IValue& result)
    {
        auto dict = result.toGenericDict();
        return {dict.at("keypoints").toTensorVector()[0], //
            dict.at("scores").toTensorVector()[0], //
            dict.at("descriptors").toTensorVector()[0]};
    }

    inline torch::Dict<std::string, Tensor> toTensorDict(const IValue& value)
    {
        return c10::impl::toTypedDict<std::string, Tensor>(value.toGenericDict());
    }

    inline torch::Tensor mat2tensor(cv::Mat image)
    {
        torch::Tensor tensor = torch::from_blob(image.data, {1, 1, image.rows, image.cols},
            torch::TensorOptions().dtype(torch::kFloat32));
        return tensor.clone();
    }

    inline KeyPoints tensor2KeyPoints(Tensor kpTensor, Tensor scoreTensor, float scale)
    {
        kpTensor = kpTensor.to(torch::kCPU);
        scoreTensor = scoreTensor.to(torch::kCPU);

        KeyPoints kps;
        int n = kpTensor.size(0);
        kps.reserve(n);
        for (int i = 0; i < n; i++)
        {
            auto kp = kpTensor[i];
            cv::Point2f pt = {kp[0].item<float>(), kp[1].item<float>()};
            pt = 1 / scale * pt;
            kps.push_back(cv::KeyPoint(pt, 5.0, -1, *scoreTensor[i].data_ptr<float>(), 0, -1));
        }
        return kps;
    }

    inline Matches tensor2Matches(Tensor matchTensor, Tensor confTensor)
    {
        matchTensor = matchTensor.to(torch::kCPU);
        confTensor = confTensor.to(torch::kCPU);

        auto valid = at::nonzero(matchTensor > -1).squeeze();
        int n = valid.size(0);

        std::vector<cv::DMatch> matches;
        matches.reserve(n);
        for (int i = 0; i < n; i++)
        {
            auto idI = *valid[i].data_ptr<long>();
            auto idJ = *matchTensor[valid[i]].data_ptr<long>();
            auto conf = *confTensor[valid[i]].data_ptr<float>();
            matches.push_back(cv::DMatch(idI, idJ, conf));
        }
        return matches;
    }
} // namespace detail

SuperGlue::SuperGlue(const std::filesystem::path& zipFolder, int targetWidth)
    : mFolder(zipFolder)
    , mTargetWidth(targetWidth)
{
    if (!fs::exists(zipFolder / "SuperPoint.zip") || !fs::exists(zipFolder / "SuperGlue.zip"))
    {
        throw fs::filesystem_error("TorchScript module files doe not exist", zipFolder,
            std::make_error_code(std::errc::no_such_file_or_directory));
    }
}

Features SuperGlue::compute(const Images& imgContainer, const std::filesystem::path& ftDir,
    const std::filesystem::path& matchDir, GeometricType geomType, MatchType matchType,
    std::size_t window, double minCoverage, std::unique_ptr<PairRecommender> recommender,
    std::size_t cacheSize, const size_t_vec& ids, std::shared_ptr<BaseProgressBar> cb)
{
    autograd::GradMode::set_enabled(false);

    if (!cb)
        cb = std::make_shared<ProgressBar>();
    if (matchType != MatchType::Manual && (!fs::exists(matchDir) || !fs::is_directory(matchDir)))
        fs::create_directories(matchDir);

    auto put = putative(
        imgContainer, ftDir, matchDir, matchType, window, std::move(recommender), ids, cb);
}

PairwiseMatches SuperGlue::putative(const Images& imgs, const std::filesystem::path& ftDir,
    const std::filesystem::path& matchDir, MatchType matchType, std::size_t window,
    std::unique_ptr<PairRecommender> recommender, const size_t_vec& ids,
    std::shared_ptr<BaseProgressBar> cb)
{
    Device device(kCPU);
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA is available! Training on GPU." << std::endl;
        device = Device(kCUDA);
    }
    jit::script::Module superpoint = jit::load(mFolder / "SuperPoint.zip");
    superpoint.eval();
    superpoint.to(device);
    jit::script::Module superglue = jit::load(mFolder / "SuperGlue.zip");
    superglue.eval();
    superglue.to(device);

    auto pairList
        = detail::getPairList(matchType, imgs.size(), window, std::move(recommender), ids);

    float scale = static_cast<float>(mTargetWidth) / imgs.getImgSize().width;
    std::cout << "scale: " << scale << std::endl;
    int targetHeight = std::lround(scale * imgs.getImgSize().height);

    spdlog::info("Computing putative matches");
    PreciseStopWatch timer;
    cb->status("Computing putative matches");
    cb->setTotal(pairList.size());

    PairwiseMatches matches;
    for (auto [idI, idJ] : pairList)
    {
        auto imgI = imgs.at(idI);
        auto imgJ = imgs.at(idJ);

        auto [ftsI, ftsJ, currMatches] = detail::glue(imgI.clone(), imgJ.clone(), superpoint,
            superglue, device, scale, mTargetWidth, targetHeight);

        if (!currMatches.empty())
            matches[{idI, idJ}] = std::move(currMatches);
        cb->inc();
    }
    auto elapsed_time = timer.elapsed_time<unsigned int, std::chrono::milliseconds>();
    cb->status("Finished");
    cb->done();
    detail::writeMatches(matchDir, matches, GeometricType::Putative);
    spdlog::debug("Found {} matches and wrote to {}", matches.size(), matchDir.string());
    spdlog::info("Computed matches in {} ms", elapsed_time);
    return matches;
}

namespace detail
{
    std::tuple<KeyPoints, KeyPoints, Matches> glue(const cv::Mat& imgI, const cv::Mat& imgJ,
        jit::script::Module& sp, jit::script::Module& sg, Device& device, float scale,
        int targetWidth, int targetHeight)
    {
        cv::Mat grayI = detail::prepare(imgI, targetWidth, targetHeight);
        cv::Mat grayJ = detail::prepare(imgJ, targetWidth, targetHeight);
        auto grayTI = detail::mat2tensor(grayI).to(device);
        auto grayTJ = detail::mat2tensor(grayJ).to(device);

        auto [kps0, scores0, descs0] = detail::unpackResult(sp.forward({grayTI}));
        auto [kps1, scores1, descs1] = detail::unpackResult(sp.forward({grayTJ}));

        torch::Dict<std::string, Tensor> input;
        input.insert("image0", grayTI);
        input.insert("image1", grayTJ);
        input.insert("keypoints0", kps0.unsqueeze(0));
        input.insert("keypoints1", kps1.unsqueeze(0));
        input.insert("scores0", scores0.unsqueeze(0));
        input.insert("scores1", scores1.unsqueeze(0));
        input.insert("descriptors0", descs0.unsqueeze(0));
        input.insert("descriptors1", descs1.unsqueeze(0));

        torch::Dict<std::string, Tensor> pred = detail::toTensorDict(sg.forward({input}));
        auto matches = pred.at("matches0")[0];
        auto confidence = pred.at("matching_scores0")[0];

        auto retKps0 = detail::tensor2KeyPoints(kps0, scores0, scale);
        auto retKps1 = detail::tensor2KeyPoints(kps1, scores1, scale);
        return {retKps0, retKps1, detail::tensor2Matches(matches, confidence)};
    }
} // namespace detail
} // namespace ht
