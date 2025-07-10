#include "aggregate_pcds.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " <lio_result.txt> [trim_start_secs=0] [trim_end_secs=0]\n"
                  << "  " << argv[0] << " <PCD_folder> <scan_states.txt> [trim_start_secs=0] [trim_end_secs=0]\n";
        return EXIT_FAILURE;
    }

    double trim_start_secs = 0.0;
    double trim_end_secs = 0.0;
    std::vector<std::pair<std::string, std::string>> pcdPosePairs;
    std::string arg1 = argv[1];

    if (hasSuffix(arg1, ".txt")) {
        // Mode 1: treat argv[1] as a list file with alternating lines:
        //   PCD_folder
        //   pose_file
        if (argc >= 3) {
            trim_start_secs = std::stod(argv[2]);
        }
        if (argc >= 4) {
            trim_end_secs = std::stod(argv[3]);
        }
        std::ifstream ifs(arg1);
        if (!ifs) {
            std::cerr << "Error: cannot open list file “" << arg1 << "”\n";
            return EXIT_FAILURE;
        }
        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty()) continue;
            std::string pcdFolder = line;
            if (!std::getline(ifs, line) || line.empty()) break;
            std::string poseFile = line;
            pcdPosePairs.emplace_back(pcdFolder, poseFile);
        }

    } else {
        // Mode 2: single PCD folder + scan_states.txt
        if (argc < 3) {
            std::cerr << "Error: missing scan_states.txt\n";
            return EXIT_FAILURE;
        }
        std::string pcdFolder = arg1;
        std::string poseFile  = argv[2];
        if (argc >= 4) {
            trim_start_secs = std::stod(argv[3]);
        }
        if (argc >= 5) {
            trim_end_secs = std::stod(argv[4]);
        }
        pcdPosePairs.emplace_back(pcdFolder, poseFile);
    }

    std::cout << "Trimming start " << trim_start_secs << " and end " << trim_end_secs << " seconds from each sequence\n";
    std::cout << "Found " << pcdPosePairs.size() << " PCD/pose-file pairs:\n";
    for (size_t i = 0; i < pcdPosePairs.size(); ++i) {
        std::cout << "  [" << i << "] “" << pcdPosePairs[i].first
                  << "”  ↔  “" << pcdPosePairs[i].second << "”\n";
    }

    // Compute output directory from the first PCD folder
    std::filesystem::path outDir = std::filesystem::path(pcdPosePairs.front().first).parent_path();
    std::cout << "Output directory: " << outDir << "\n\n";

    aggregatePointCloudsWithPose(pcdPosePairs, outDir.string(), trim_start_secs, trim_end_secs);
    return EXIT_SUCCESS;
}
