
#include <basalt/image/image.h>
#include <basalt/utils/sophus_utils.hpp>
#include <vector>

namespace basalt {

struct ApriltagDetectorData;

class ApriltagDetector {
 public:
  ApriltagDetector(int numTags);

  ~ApriltagDetector();

  template <typename PixelType>
  void detectTags(basalt::ManagedImage<PixelType>& img_raw,
                  Eigen::aligned_vector<Eigen::Vector2d>& corners,
                  std::vector<int>& ids, std::vector<double>& radii,
                  Eigen::aligned_vector<Eigen::Vector2d>& corners_rejected,
                  std::vector<int>& ids_rejected,
                  std::vector<double>& radii_rejected);

 private:
  ApriltagDetectorData* data;
};

}  // namespace basalt
