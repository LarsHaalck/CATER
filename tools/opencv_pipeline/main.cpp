// taken from: https://docs.opencv.org/master/d8/d19/tutorial_stitcher.html

#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;
bool divide_images = false;
Stitcher::Mode mode = Stitcher::PANORAMA;
vector<Mat> imgs;
string result_name = "result.jpg";

void printUsage(char** argv)
{
    cout << "Images stitcher.\n\n"
         << "Usage :\n"
         << argv[0]
         << " [Flags] img1 img2 [...imgN]\n\n"
            "Flags:\n"
            "  --d3\n"
            "      internally creates three chunks of each image to increase stitching success\n"
            "  --mode (panorama|scans)\n"
            "      Determines configuration of stitcher. The default is 'panorama',\n"
            "      mode suitable for creating photo panoramas. Option 'scans' is suitable\n"
            "      for stitching materials under affine transformation, such as scans.\n"
            "  --output <result_img>\n"
            "      The default is 'result.jpg'.\n\n"
            "Example usage :\n"
         << argv[0] << " --d3 --try_use_gpu yes --mode scans img1.jpg img2.jpg\n";
}

int parseCmdArgs(int argc, char** argv)
{
    if (argc == 1)
    {
        printUsage(argv);
        return EXIT_FAILURE;
    }
    for (int i = 1; i < argc; ++i)
    {
        if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
        {
            printUsage(argv);
            return EXIT_FAILURE;
        }
        else if (string(argv[i]) == "--d3")
        {
            divide_images = true;
        }
        else if (string(argv[i]) == "--output")
        {
            result_name = argv[i + 1];
            i++;
        }
        else if (string(argv[i]) == "--mode")
        {
            if (string(argv[i + 1]) == "panorama")
                mode = Stitcher::PANORAMA;
            else if (string(argv[i + 1]) == "scans")
                mode = Stitcher::SCANS;
            else
            {
                cout << "Bad --mode flag value\n";
                return EXIT_FAILURE;
            }
            i++;
        }
        else
        {
            ifstream infile(argv[i]);
            for (std::string line = {}; std::getline(infile, line);)
            {
                Mat img = imread(line);
                if (img.empty())
                {
                    cout << "Can't read image '" << argv[i] << "'\n";
                    return EXIT_FAILURE;
                }
                imgs.push_back(img);
            }
        }
    }
    return EXIT_SUCCESS;
}

int main(int argc, char* argv[])
{
    int retval = parseCmdArgs(argc, argv);
    if (retval)
        return EXIT_FAILURE;
    Mat pano;
    Ptr<Stitcher> stitcher = Stitcher::create(mode);
    stitcher->setFeaturesFinder(ORB::create(2000));
    auto ptr = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
    /* stitcher->setFeaturesMatcher(ptr); */
    Stitcher::Status status = stitcher->stitch(imgs, pano);
    if (status != Stitcher::OK)
    {
        cout << "Can't stitch images, error code = " << int(status) << endl;
        return EXIT_FAILURE;
    }
    imwrite(result_name, pano);
    cout << "stitching completed successfully\n" << result_name << " saved!";
    return EXIT_SUCCESS;
}
