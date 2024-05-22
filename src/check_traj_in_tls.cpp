/**
 * @file check_traj_in_tls.cpp
 * @brief Check if a trajectory is in the TLS map
 */
#include "dist_checkup.h"

int main(int argc, char **argv) {
    std::string path_to_traj;
    std::string output_file;
    std::vector<std::string> tls_ref_traj_files = 
        {"./src/FAST_LIO_SLAM/FAST_LIO/data/20231109/data1/scan_states.txt", 
         "./src/FAST_LIO_SLAM/FAST_LIO/data/20231109/data2/scan_states.txt"};

    if (argc < 2) {
        std::cerr << "Usage: check_traj_in_tls <path_to_traj> [dist_threshold=8]" << std::endl;
    } else {
        path_to_traj = argv[1];
    }
    std::string bn = path_to_traj.substr(path_to_traj.find_last_of("/\\") + 1);
    std::string bn_noext = bn.substr(0, bn.find_last_of("."));
    std::string dn = path_to_traj.substr(0, path_to_traj.find_last_of("/\\"));

    double dist_threshold = 8; // roads in wuhan univ info faculty are usually less than 10 m wide.
    if (argc >= 3) {
        dist_threshold = std::stod(argv[2]);
    }
    std::string dist_str;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << dist_threshold;
    ss >> dist_str;
    output_file = dn + "/" + bn_noext + "_" + dist_str + ".txt";
    std::cout << "traj file " << path_to_traj << ", dist_threshold " << dist_threshold << std::endl;

    Trajectory ref_traj;
    load_ref_traj(tls_ref_traj_files, ref_traj);

    Trajectory traj;
    size_t numposes = load_states(path_to_traj, traj);
    std::cout << "Loaded " << numposes << " poses from " << path_to_traj << std::endl;

    std::vector<bool> status = check_dist_to_ref_traj(traj, ref_traj, dist_threshold);

    std::ofstream status_file(output_file);
    if (!status_file.is_open()) {
        std::cerr << "Failed to open file: " << output_file << std::endl;
        return 1;
    }
    for (size_t i = 0; i < status.size(); ++i) {
        status_file << std::fixed << std::setprecision(9) << traj[i](0) << " " 
            << std::setprecision(6) << traj[i](1) << " " << traj[i](2) << " " << traj[i](3)
            << " " << status[i] << std::endl;
    }
    status_file.close();
    std::cout << "Saved status to " << output_file << std::endl;
    return 0;
}
