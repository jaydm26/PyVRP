#include "Congestion.h"
#include "includes/csv.hpp"

namespace pyvrp::congestion
{
CongestionProfile::CongestionProfile(const std::filesystem::path &path)
    : path_(path)
{
    csv::CSVReader congestionCsvFile(std::filesystem::absolute(path_).string());
    for (csv::CSVRow row : congestionCsvFile)
    {
        // Congestion profiles are expected to have two columns: time and
        // congestion. Time in seconds and congestion in percentage.
        double t = row["time"].get<double>() * 3600;  // Convert to seconds
        time_.push_back(t);
        double c = row["congestion"].get<double>();  // Convert to fraction
        congestion_.push_back(c);
    }

    double stepSize = (time_.back() - time_.front()) / (time_.size() - 1);
    spline_ = boost::math::interpolators::cardinal_cubic_b_spline<double>(
        congestion_.begin(), congestion_.end(), time_.front(), stepSize);
}
CongestionProfile const getCongestionProfile(
    [[maybe_unused]] CongestionBehaviour const congestionBehaviour)
{
    std::filesystem::path currentFile = __FILE__;
    // We run this in the pyvrp folder (you can see
    // the .so file)
    std::filesystem::path rootDir = currentFile.parent_path().parent_path();
    std::filesystem::path congestionCsv
        = rootDir / "research" / "data" / "congestion" / "congestion.csv";
    return CongestionProfile(congestionCsv);
}
}  // namespace pyvrp::congestion