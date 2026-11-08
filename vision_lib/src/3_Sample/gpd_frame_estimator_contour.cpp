// KINOVA CHANGE: copy of third_party/gpd/src/gpd/candidate/frame_estimator.cpp
// (atenpas GPD). GPD's "gpd_frame_estimator" library is built from THIS file
// instead of the original (see set_property(TARGET gpd_frame_estimator ...)
// in kinova_grasp/CMakeLists.txt), so the original stays unchanged.
// Changes from the original are marked with "KINOVA CHANGE": the frames can
// follow the closing directions of the object (3_Sample/contour_frames.h).

#include <gpd/candidate/frame_estimator.h>

#include "3_Sample/contour_frames.h"  // KINOVA CHANGE

// KINOVA CHANGE: closing directions set by setContourDirections (cloud frame).
static std::vector<Eigen::Vector3d> contour_directions;

// KINOVA CHANGE: setContourDirections, declared in 3_Sample/contour_frames.h.
void setContourDirections(const std::vector<Eigen::Vector3d> &directions) {
  contour_directions = directions;
}

namespace gpd {
namespace candidate {

// KINOVA CHANGE: contourFrames: one frame per closing direction, at the
// sample of frame and with its normal. findAverageNormalAxis takes the
// normal as the axis of largest spread of the normals it gets and the
// curvature axis as the axis of smallest spread (local_frame.cpp:14-40).
// So it gets 10 copies of the normal and one vector normal x c: the normal
// stays, the curvature axis becomes +-c, and GPD closes the fingers along
// the curvature axis at the orientation -90 deg (hand_set.cpp:66-69,
// hand_search.cpp:151-155). Directions almost along the normal are skipped.
static void contourFrames(const LocalFrame &frame, std::vector<LocalFrame> &frames_out) {
  const Eigen::Vector3d normal = frame.getNormal();
  size_t added = 0;
  for (const Eigen::Vector3d &direction : contour_directions) {
    Eigen::Vector3d c = direction - direction.dot(normal) * normal;
    if (c.norm() < 0.5) {
      continue;
    }
    c.normalize();
    Eigen::Matrix3Xd normals(3, 11);
    for (int k = 0; k < 10; k++) {
      normals.col(k) = normal;
    }
    normals.col(10) = normal.cross(c);
    LocalFrame contour_frame(frame.getSample());
    contour_frame.findAverageNormalAxis(normals);
    frames_out.push_back(contour_frame);
    added++;
  }
  if (added == 0) {
    frames_out.push_back(frame);
  }
}

std::vector<LocalFrame> FrameEstimator::calculateLocalFrames(
    const util::Cloud &cloud_cam, const std::vector<int> &indices,
    double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree) const {
  double t1 = omp_get_wtime();
  std::vector<std::unique_ptr<LocalFrame>> frames;
  frames.resize(indices.size());

#ifdef _OPENMP  // parallelization using OpenMP
#pragma omp parallel for num_threads(num_threads_)
#endif
  for (int i = 0; i < indices.size(); i++) {
    const pcl::PointXYZRGBA &sample =
        cloud_cam.getCloudProcessed()->points[indices[i]];
    frames[i] =
        calculateFrame(cloud_cam.getNormals(),
                       sample.getVector3fMap().cast<double>(), radius, kdtree);
  }

  std::vector<LocalFrame> frames_out;
  for (int i = 0; i < frames.size(); i++) {
    if (frames[i]) {
      // KINOVA CHANGE: the contour frames instead, if there are directions.
      if (contour_directions.empty()) {
        frames_out.push_back(*frames[i].get());
      } else {
        contourFrames(*frames[i].get(), frames_out);
      }
    }
  }

  double t2 = omp_get_wtime();
  printf("Estimated %zu frames in %3.4fs.\n", frames_out.size(), t2 - t1);

  return frames_out;
}

std::vector<LocalFrame> FrameEstimator::calculateLocalFrames(
    const util::Cloud &cloud_cam, const Eigen::Matrix3Xd &samples,
    double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree) const {
  double t1 = omp_get_wtime();
  std::vector<std::unique_ptr<LocalFrame>> frames;
  frames.resize(samples.cols());

#ifdef _OPENMP  // parallelization using OpenMP
#pragma omp parallel for num_threads(num_threads_)
#endif
  for (int i = 0; i < samples.cols(); i++) {
    frames[i] =
        calculateFrame(cloud_cam.getNormals(), samples.col(i), radius, kdtree);
  }

  // Only keep frames that are not null.
  std::vector<LocalFrame> frames_out;
  for (int i = 0; i < frames.size(); i++) {
    if (frames[i]) {
      // KINOVA CHANGE: the contour frames instead, if there are directions.
      if (contour_directions.empty()) {
        frames_out.push_back(*frames[i].get());
      } else {
        contourFrames(*frames[i].get(), frames_out);
      }
    }
  }

  double t2 = omp_get_wtime();
  printf("Estimated %zu frames in %3.4fs.\n", frames_out.size(), t2 - t1);

  return frames_out;
}

std::unique_ptr<LocalFrame> FrameEstimator::calculateFrame(
    const Eigen::Matrix3Xd &normals, const Eigen::Vector3d &sample,
    double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA> &kdtree) const {
  std::unique_ptr<LocalFrame> frame = nullptr;
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::PointXYZRGBA sample_pcl = eigenVectorToPcl(sample);

  if (kdtree.radiusSearch(sample_pcl, radius, nn_indices, nn_dists) > 0) {
    Eigen::Matrix3Xd nn_normals(3, nn_indices.size());

    for (int i = 0; i < nn_normals.cols(); i++) {
      nn_normals.col(i) = normals.col(nn_indices[i]);
    }

    frame = std::make_unique<LocalFrame>(sample);
    frame->findAverageNormalAxis(nn_normals);
  }

  return frame;
}

pcl::PointXYZRGBA FrameEstimator::eigenVectorToPcl(
    const Eigen::Vector3d &v) const {
  pcl::PointXYZRGBA p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}

}  // namespace candidate
}  // namespace gpd
