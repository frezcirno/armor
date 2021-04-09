/**
 * SORT: A Simple, Online and Realtime Tracker
 *
 * This is a C++ implementation of the 2016 by Alex Bewley proposed tracking
 * algorithm. See https://github.com/abewley/sort and
 * @inproceedings{sort2016,
 * author = {Bewley, Alex and Ge, Zongyuan and Ott, Lionel and Ramos, Fabio and Upcroft, Ben},
 * booktitle = {2016 IEEE International Conference on Image Processing (ICIP)},
 * title = {Simple online and realtime tracking},
 * year = {2016},
 * pages = {3464-3468}
 * }
 * for further information.
 * This was written by Falko Becker, tyler.newnoise@gmail.com, 2020.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SORT_SORT_H
#define SORT_SORT_H
#define NDEBUG
#include <algorithm>
#include <map>
#include <vector>

#include <dlib/optimization/max_cost_assignment.h>
#include <opencv2/video/tracking.hpp>

#if !defined(NDEBUG)
#include <string>
#endif

namespace sort {

typedef cv::Rect_<float> BBox;

struct Track {
  BBox bbox;
  std::size_t id{};
};
/**
 * This class implements a simple multi target tracker. It holds an
 * unordered map with the track ids as the key and a KalmanBoxTracker object as
 * the value.
 */
class SORT {
public:
  /**
   * Constructor.
   *
   * @param iou_threshold Minimum intersection over union threshold.
   * @param max_age Maximum number of missed misses before track is deleted.
   * @param n_init Number of consecutive detections before the track is
   *        confirmed.
   */
  SORT(const double iou_threshold, const unsigned int max_age,
          const unsigned int n_init)
          :
          id_{0},
          frame_cnt_{0},
          max_age_{max_age},
          min_hits_{n_init},
          precision_{10000},
          trackers_{std::unordered_map<std::size_t, std::unique_ptr<KalmanBoxTracker>>()}
  {
    iou_threshold_ = iou_threshold * precision_;
  }

  ~SORT() = default;

  /**
  * Perform measurement update and track management.
  *
  * @param detections A list with detections at the current time step.
  * @return The list with active tracks at the current time step.
  */
  [[nodiscard]] std::vector<struct Track> update(const std::vector<BBox >& detections)
  {
    ++frame_cnt_;
    // Get predicted locations from existing trackers.
    for (auto& track : trackers_)
      track.second->predict();

    if (!detections.empty()) {
      std::vector<struct Match_> matches;
      std::vector<std::size_t> unmatched_dets;

      associate_detections_to_trackers_(detections, matches, unmatched_dets);

      // Create and initialize new trackers for unmatched detections.
      for (std::size_t d : unmatched_dets)
        trackers_.emplace(id_++, std::make_unique<KalmanBoxTracker>(detections[d]));

      // Update trackers with matched detections.
      for (auto& match : matches)
        trackers_.at(match.track_id)->update(detections.at(match.detection_id));
    }

#if !defined(NDEBUG)
    std::cout << "trackers: " << std::endl;
    for (auto& tracker : trackers_)
        std::cout << "tracker " << tracker.first + 1 << " is active with "
                  << tracker.second->hits
                  << " hits and time_since_update "
                  << tracker.second->time_since_update
                  << " with a bbox: " << tracker.second->get_state()
                  << std::endl;
#endif

    // Return active trackers.
    std::vector<struct Track> active_tracks;
    for (const auto& trk : trackers_) {
      // Start tracking at the very first frame.
      if ((trk.second->time_since_update < max_age_) && (trk.second->hits >= min_hits_ || frame_cnt_ <= min_hits_)) {
        struct Track track;
        track.bbox = trk.second->get_state();
        track.id = trk.first + 1; // +1 as MOT benchmark requires positive.
        active_tracks.emplace_back(track);
      }
    }

    // Delete dead trackers.
    for (auto it = trackers_.begin(); it != trackers_.end();) {
      if (it->second->time_since_update > max_age_ || (it->second->time_since_update == 1 && it->second->hits < min_hits_))
        it = trackers_.erase(it);
      else
        it++;
    }

#if !defined(NDEBUG)
    for (const auto& t : active_tracks) {
        std::cout << "returned tracker: " << t.id << " with bbox: " << t.bbox << std::endl;
      }
#endif
    return active_tracks;
  }

private:
  /**
   * This nested class implements a single target track with state
   * [x, y, s, r, x', y', s'], where x,y are the center, s is the scale/area
   * and r the aspect ratio of the bounding box, and x', y', s' their
   * respective velocities.
   */
  class KalmanBoxTracker {
  public:
    /**
     * Constructor.
     *
     * @param bbox A cv::rect_ with the detection in [x, y, w, h] format.
     */
    explicit KalmanBoxTracker(const BBox& bbox)
            :hits{0},
             time_since_update{0}
    {
      kf_.transitionMatrix = (cv::Mat_<float>(7, 7) <<
              1, 0, 0, 0, 1, 0, 0,
              0, 1, 0, 0, 0, 1, 0,
              0, 0, 1, 0, 0, 0, 1,
              0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 0, 1);

      kf_.measurementMatrix = (cv::Mat_<float>(4, 7) <<
              1, 0, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0);

      kf_.measurementNoiseCov = (cv::Mat_<float>(4, 4) <<
              1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 10, 0,
              0, 0, 0, 10);

      kf_.processNoiseCov = (cv::Mat_<float>(7, 7) <<
              1, 0, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 0.01, 0, 0,
              0, 0, 0, 0, 0, 0.01, 0,
              0, 0, 0, 0, 0, 0, 0.0001);

      kf_.errorCovPost = (cv::Mat_<float>(7, 7) <<
              10, 0, 0, 0, 0, 0, 0,
              0, 10, 0, 0, 0, 0, 0,
              0, 0, 10, 0, 0, 0, 0,
              0, 0, 0, 10, 0, 0, 0,
              0, 0, 0, 0, 10000, 0, 0,
              0, 0, 0, 0, 0, 10000, 0,
              0, 0, 0, 0, 0, 0, 10000);

      kf_.statePost.at<float>(0, 0) = bbox.x + bbox.width / 2;
      kf_.statePost.at<float>(1, 0) = bbox.y + bbox.height / 2;
      kf_.statePost.at<float>(2, 0) = bbox.area();
      kf_.statePost.at<float>(3, 0) = bbox.width / bbox.height;
    }

    ~KalmanBoxTracker() = default;

    //! Return the current bounding box estimate.
    [[nodiscard]] BBox get_state() const
    {
      return x_to_bbox(kf_.statePost);
    }

    //! Advances the state vector on the current time step.
    void predict()
    {
      // https://github.com/abewley/sort/issues/43
      if (kf_.statePost.at<float>(6, 0) + kf_.statePost.at<float>(2, 0) <= 0)
        kf_.statePost.at<float>(6, 0) *= 0.0;
      time_since_update++;
      kf_.predict();
    }

    //! Perform measurement update.
    void update(const BBox& bbox)
    {
      hits++;
      time_since_update = 0;
      kf_.correct(bbox_to_z(bbox));
    }

    std::size_t hits;
    std::size_t time_since_update;

  private:
    cv::KalmanFilter kf_ = cv::KalmanFilter(7, 4, 0);

    //! Convert bounding box in the form [x, y, width, height] and return z in
    //! the form [center_x,center_y, scale, ratio].
    static cv::Mat bbox_to_z(const BBox& bbox)
    {
      auto center_x{bbox.x + bbox.width / 2};
      auto center_y{bbox.y + bbox.height / 2};
      auto area{bbox.area()};
      auto ratio{bbox.width / bbox.height};

      cv::Mat z{(cv::Mat_<float>({center_x, center_y, area, ratio}))};
      return z;
    }

    //! Convert bounding box in the center form
    //! [center_x, center_y, scale, ratio] and returns it in the form of
    //! [x, y, width, height].
    static BBox x_to_bbox(const cv::Mat& state)
    {
      auto center_x{state.at<float>(0, 0)};
      auto center_y{state.at<float>(1, 0)};
      auto area{state.at<float>(2, 0)};
      auto ratio{state.at<float>(3, 0)};

      auto width{sqrt(area * ratio)};
      auto height{area / width};
      auto x{(center_x - width / 2)};
      auto y{(center_y - height / 2)};

      // BBox in image space.
      if (x < 0 && center_x > 0)
        x = 0;
      if (y < 0 && center_y > 0)
        y = 0;
      return BBox(x, y, width, height);
    }
  }; // class KalmanBoxTracker

  struct Match_{
    std::size_t detection_id{};
    std::size_t track_id{};
  };

  void associate_detections_to_trackers_(
          const std::vector<BBox>&,
          std::vector<struct Match_>&,
          std::vector<std::size_t>&);

  static float iou_(const BBox&, const BBox&);

  std::size_t id_;
  std::size_t frame_cnt_;
  unsigned int max_age_;
  unsigned int min_hits_;
  const double precision_;
  std::unordered_map<std::size_t, std::unique_ptr<KalmanBoxTracker>> trackers_;
  double iou_threshold_;
};

//! Assign detections to tracked boxes.
void SORT::associate_detections_to_trackers_(
        const std::vector<BBox >& detections,
        std::vector<struct Match_>& matches,
        std::vector<std::size_t>& unmatched_dets
)
{
  if (trackers_.empty()) {
    for (std::size_t i{0}; i < detections.size(); ++i)
      unmatched_dets.push_back(i);
    return;
  }

  // Create iou cost matrix.
  std::size_t rows {detections.size()};
  std::size_t col;
  std::size_t cols {trackers_.size()};
  dlib::matrix<std::size_t> iou_cost(rows, cols);

  for (std::size_t row{0}; row < rows; ++row) {
    col = 0;
    for (auto& trk : trackers_) {
      iou_cost(row, col) =
          std::size_t(precision_ * iou_(detections[row],
              trk.second->get_state()));
      ++col;
    }
  }

  // Create mapping of rows from iou cost matrix to tracker ids.
  col = 0;
  std::vector<std::size_t> idx_to_trkid;
  for (auto& trk : trackers_) {
    idx_to_trkid.push_back(trk.first);
    ++col;
  }

  // Pad iou cost matrix if not square.
  if (iou_cost.nr() > iou_cost.nc())
    iou_cost =
        dlib::join_rows(iou_cost,
            dlib::zeros_matrix<std::size_t>(
                1, iou_cost.nr() - iou_cost.nc()));
  else if (iou_cost.nc() > iou_cost.nr())
    iou_cost =
        dlib::join_cols(iou_cost,
            dlib::zeros_matrix<std::size_t>(
                iou_cost.nc() - iou_cost.nr(), 1));

  // Solve linear assignment problem.
  std::vector<long> lap = dlib::max_cost_assignment(iou_cost);

#if !defined(NDEBUG)
  std::cout << "trackers: " << std::endl;
  for (auto& t: trackers_) {
    std::cout << t.second->get_state() << std::endl;
  }
  std::cout << "\niou_cost matrix: "<< std::endl;
  std::cout << iou_cost << std::endl;
  std::cout << "assignment: "<< std::endl;
  for (auto & l: lap)
    std::cout << l <<' ' ;
  std::cout << std::endl;
#endif

  // Filter out matched with low iou and assign detections to tracks.
  for (std::size_t d{0}; d < lap.size(); ++d) {
    if (iou_cost(d, lap[d]) < iou_threshold_) {
      if (d < detections.size())
        unmatched_dets.push_back(d);
    } else {
      struct Match_ match;
      match.detection_id = d;
      match.track_id = idx_to_trkid[lap[d]];
      matches.emplace_back(match);
    }
  }
}

//! Compute intersection-over-union between two bounding boxes.
float SORT::iou_(const BBox& det, const BBox& trk)
{
  auto xx1{std::max(det.tl().x, trk.tl().x)};
  auto yy1{std::max(det.tl().y, trk.tl().y)};
  auto xx2{std::min(det.br().x, trk.br().x)};
  auto yy2{std::min(det.br().y, trk.br().y)};
  auto width{std::max(0.f, xx2 - xx1)};
  auto height{std::max(0.f, yy2 - yy1)};
  auto intersection{width * height};
  float union_area{det.area() + trk.area() - intersection};
  return intersection / union_area;
}
} // namespace sort
#endif
