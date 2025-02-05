// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber and Anne Elisabeth Reichert,
// German Aerospace Center (DLR)

#include <m3t/region_model.h>
namespace m3t {

RegionModel::RegionModel(const std::string &name,
                         const std::shared_ptr<Body> &body_ptr,
                         const std::filesystem::path &model_path,
                         float sphere_radius, int n_divides, int n_points,
                         float max_radius_depth_offset,
                         float stride_depth_offset, bool use_random_seed,
                         int image_size)
    : Model{name,
            body_ptr,
            model_path,
            sphere_radius,
            n_divides,
            n_points,
            max_radius_depth_offset,
            stride_depth_offset,
            use_random_seed,
            image_size} {}

RegionModel::RegionModel(const std::string &name,
                         const std::filesystem::path &metafile_path,
                         const std::shared_ptr<Body> &body_ptr)
    : Model{name, metafile_path, body_ptr} {}

bool RegionModel::SetUp() {
  set_up_ = false;
  if (!metafile_path_.empty())
    if (!LoadMetaData()) return false;

  // Check if all required objects are set up
  if (!body_ptr_->set_up()) {
    std::cerr << "Body " << body_ptr_->name() << " was not set up" << std::endl;
    return false;
  }

  for (auto &associated_body_ptr : associated_body_ptrs_) {
    if (!associated_body_ptr->set_up()) {
      if (!associated_body_ptr->SetUp()) {
        std::cerr << "Body " << associated_body_ptr->name() << " was not set up"
                  << std::endl;
        return false;
      }
    }
  }

  if (!DepthOffsetVariablesValid()) return false;
  if (use_random_seed_ || !LoadModel()) {
    if (!GenerateModel()) return false;
    if (!SaveModel()) return false;
  }
  set_up_ = true;
  return true;
}

bool RegionModel::AddAssociatedBody(
    const std::shared_ptr<Body> &associated_body_ptr, bool movable,
    bool same_region) {
  if (!AddPtrIfNameNotExists(associated_body_ptr, &associated_body_ptrs_)) {
    std::cerr << "Body " << associated_body_ptr->name() << " already exists"
              << std::endl;
    return false;
  }

  if (movable) {
    if (same_region)
      movable_same_region_body_ptrs_.push_back(associated_body_ptr);
    else
      movable_body_ptrs_.push_back(associated_body_ptr);
  } else {
    if (same_region)
      fixed_same_region_body_ptrs_.push_back(associated_body_ptr);
    else
      fixed_body_ptrs_.push_back(associated_body_ptr);
  }
  return true;
}

bool RegionModel::DeleteAssociatedBody(const std::string &name) {
  if (!DeletePtrIfNameExists(name, &associated_body_ptrs_)) {
    std::cerr << "Body " << name << " not found" << std::endl;
    return false;
  } else {
    DeletePtrIfNameExists(name, &fixed_body_ptrs_);
    DeletePtrIfNameExists(name, &fixed_same_region_body_ptrs_);
    DeletePtrIfNameExists(name, &movable_same_region_body_ptrs_);
    DeletePtrIfNameExists(name, &movable_body_ptrs_);
  }
  return true;
}

void RegionModel::ClearAssociatedBodies() {
  associated_body_ptrs_.clear();
  fixed_body_ptrs_.clear();
  movable_body_ptrs_.clear();
  fixed_same_region_body_ptrs_.clear();
  movable_same_region_body_ptrs_.clear();
}

bool RegionModel::GetClosestView(const Transform3fA &body2camera_pose,
                                 const View **closest_view) const {
  if (!set_up_) {
    std::cerr << "Set up region model " << name_ << " first" << std::endl;
    return false;
  }

  if (body2camera_pose.translation().norm() == 0.0f) {
    *closest_view = &views_[0];
    return true;
  }

  Eigen::Vector3f orientation{
      body2camera_pose.rotation().inverse() *
      body2camera_pose.translation().matrix().normalized()};

  std::cout << "orientation norm " << orientation.norm() << "\n";

  float closest_dot = -1.0f;
  for (auto &view : views_) {
    float dot = orientation.dot(view.orientation);
    if (dot > closest_dot) {
      *closest_view = &view;
      closest_dot = dot;
    }
  }
  std::cout << "closest_dot " << closest_dot << "\n";

  return true;
}

float RegionModel::max_contour_length() const { return max_contour_length_; }

const std::vector<std::shared_ptr<Body>> &RegionModel::associated_body_ptrs()
    const {
  return associated_body_ptrs_;
}

const std::vector<std::shared_ptr<Body>> &RegionModel::fixed_body_ptrs() const {
  return fixed_body_ptrs_;
}

const std::vector<std::shared_ptr<Body>> &RegionModel::movable_body_ptrs()
    const {
  return movable_body_ptrs_;
}

const std::vector<std::shared_ptr<Body>> &
RegionModel::fixed_same_region_body_ptrs() const {
  return fixed_same_region_body_ptrs_;
}

const std::vector<std::shared_ptr<Body>> &
RegionModel::movable_same_region_body_ptrs() const {
  return movable_same_region_body_ptrs_;
}

bool RegionModel::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!ReadRequiredValueFromYaml(fs, "model_path", &model_path_)) {
    std::cerr << "Could not read all required body parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  ReadOptionalValueFromYaml(fs, "sphere_radius", &sphere_radius_);
  ReadOptionalValueFromYaml(fs, "n_divides", &n_divides_);
  ReadOptionalValueFromYaml(fs, "n_points", &n_points_);
  ReadOptionalValueFromYaml(fs, "max_radius_depth_offset",
                            &max_radius_depth_offset_);
  ReadOptionalValueFromYaml(fs, "stride_depth_offset", &stride_depth_offset_);
  ReadOptionalValueFromYaml(fs, "use_random_seed", &use_random_seed_);
  ReadOptionalValueFromYaml(fs, "image_size", &image_size_);
  fs.release();

  // Process parameters
  if (model_path_ == "INFER_FROM_NAME")
    model_path_ = metafile_path_.parent_path() / (name_ + ".bin");
  else if (model_path_.is_relative())
    model_path_ = metafile_path_.parent_path() / model_path_;
  return true;
}

bool RegionModel::GenerateModel() {
  // Generate camera poses
  std::vector<Transform3fA> camera2body_poses;
  GenerateGeodesicPoses(&camera2body_poses);

  // Create RendererGeometries in main thread to comply with GLFW thread safety
  // requirements
  auto main_renderer_geometry_ptrs{
      GenerateSetUpRendererGeometries(omp_get_max_threads())};
  auto associated_renderer_geometry_ptrs{
      GenerateSetUpAssociatedRendererGeometries(omp_get_max_threads())};

  // Generate template views
  std::cout << "Start generating model " << name_ << std::endl;
  views_.resize(camera2body_poses.size());
  max_contour_length_ = 0.0f;
  bool cancel = false;
  std::atomic<int> count = 1;
#pragma omp parallel
  {
    // Set up renderer
    std::shared_ptr<FullSilhouetteRenderer> main_renderer_ptr;
    if (!SetUpRenderer(main_renderer_geometry_ptrs[omp_get_thread_num()],
                       body_ptr_, &main_renderer_ptr) ||
        !AddBodiesToRenderer(main_renderer_ptr, {body_ptr_}, kMainBodyID) ||
        !AddBodiesToRenderer(main_renderer_ptr, fixed_body_ptrs_,
                             kDifferentBodyID))
      cancel = true;

    AssociatedRendererPtrs associated_renderer_ptrs;
    if (!SetUpAssociatedRenderers(associated_renderer_geometry_ptrs,
                                  omp_get_thread_num(),
                                  &associated_renderer_ptrs) ||
        !AddBodiesToAssociatedRenderers(associated_renderer_ptrs))
      cancel = true;

#pragma omp for
    for (int i = 0; i < int(views_.size()); ++i) {
      if (cancel) continue;
      std::stringstream msg;
      msg << "Generate region model " << name_ << ": view " << count++ << " of "
          << views_.size() << std::endl;
      std::cout << msg.str();

      // Render and fetch images
      RenderAndFetchSilhouetteImage(main_renderer_ptr, camera2body_poses[i],
                                    true);
      RenderAndFetchAssociatedImages(associated_renderer_ptrs,
                                     camera2body_poses[i]);

      // Generate data
      views_[i].orientation =
          camera2body_poses[i].matrix().col(2).segment(0, 3);

      // std::cout << "views_[i].orientation " << views_[i].orientation << "\n";
      // std::cout << "translation " << camera2body_poses[i] << "\n";

      views_[i].data_points.resize(n_points_);
      if (!GeneratePointData(*main_renderer_ptr, associated_renderer_ptrs,
                             camera2body_poses[i], &views_[i].data_points,
                             &views_[i].contour_length))
        cancel = true;

#pragma omp critical
      {
        // Set maximal contour length
        if (views_[i].contour_length > max_contour_length_)
          max_contour_length_ = views_[i].contour_length;
      }
    }
  }
  if (cancel) return false;
  std::cout << "Finish generating model " << name_ << std::endl;
  return true;
}

bool RegionModel::LoadModel() {
  std::ifstream ifs{model_path_, std::ios::in | std::ios::binary};
  if (!ifs.is_open() || ifs.fail()) {
    ifs.close();
    std::cout << "Could not open model file " << model_path_ << std::endl;
    return false;
  }

  if (!LoadModelParameters(kVersionID, kModelType, &ifs)) {
    std::cout << "Model file " << model_path_
              << " was generated using different model parameters" << std::endl;
    return false;
  }

  if (!LoadBodyData(body_ptr_, &ifs)) {
    std::cout << "Model file " << model_path_
              << " was generated using different body parameters" << std::endl;
    return false;
  }

  if (!LoadAssociatedBodyData(&ifs)) {
    std::cout << "Model file " << model_path_
              << " was generated using different associated body parameters or "
                 "configurations"
              << std::endl;
    return false;
  }

  // Load view data
  size_t n_views;
  ifs.read((char *)(&n_views), sizeof(n_views));
  views_.clear();
  views_.reserve(n_views);
  for (size_t i = 0; i < n_views; i++) {
    View tv;
    tv.data_points.resize(n_points_);
    ifs.read((char *)(tv.data_points.data()), n_points_ * sizeof(DataPoint));
    ifs.read((char *)(tv.orientation.data()), sizeof(tv.orientation));
    ifs.read((char *)(&(tv.contour_length)), sizeof(tv.contour_length));
    views_.push_back(std::move(tv));
  }
  ifs.close();

  // Calculate max contour length
  max_contour_length_ = 0.0f;
  for (const auto &view : views_)
    max_contour_length_ = std::max(max_contour_length_, view.contour_length);
  return true;
}

bool RegionModel::SaveModel() const {
  std::ofstream ofs{model_path_, std::ios::out | std::ios::binary};
  SaveModelParameters(kVersionID, kModelType, &ofs);
  SaveBodyData(body_ptr_, &ofs);
  SaveAssociatedBodyData(&ofs);

  // Save main data
  size_t n_views = views_.size();
  ofs.write((const char *)(&n_views), sizeof(n_views));
  for (const auto &v : views_) {
    ofs.write((const char *)(v.data_points.data()),
              n_points_ * sizeof(DataPoint));
    ofs.write((const char *)(v.orientation.data()), sizeof(v.orientation));
    ofs.write((const char *)(&v.contour_length), sizeof(v.contour_length));
  }
  ofs.flush();
  ofs.close();
  return true;
}

void RegionModel::SaveAssociatedBodyData(std::ofstream *ofs) const {
  size_t num_associated_bodies = associated_body_ptrs_.size();
  ofs->write((const char *)(&num_associated_bodies),
             sizeof(num_associated_bodies));
  std::vector<std::vector<std::shared_ptr<Body>>> associated_body_ptr_vectors{
      fixed_body_ptrs_, fixed_same_region_body_ptrs_, movable_body_ptrs_,
      movable_same_region_body_ptrs_};

  for (auto &associated_body_ptr_vector : associated_body_ptr_vectors) {
    num_associated_bodies = associated_body_ptr_vector.size();
    ofs->write((const char *)(&num_associated_bodies),
               sizeof(num_associated_bodies));
    for (const auto &associated_body_ptr : associated_body_ptr_vector)
      SaveBodyData(associated_body_ptr, ofs);
  }
}

bool RegionModel::LoadAssociatedBodyData(std::ifstream *ifs) {
  size_t num_associated_bodies;
  ifs->read((char *)(&num_associated_bodies), sizeof(num_associated_bodies));
  if (num_associated_bodies != associated_body_ptrs_.size()) return false;

  std::vector<std::vector<std::shared_ptr<Body>>> associated_body_ptr_vectors{
      fixed_body_ptrs_, fixed_same_region_body_ptrs_, movable_body_ptrs_,
      movable_same_region_body_ptrs_};

  for (auto &associated_body_ptr_vector : associated_body_ptr_vectors) {
    ifs->read((char *)(&num_associated_bodies), sizeof(num_associated_bodies));
    if (num_associated_bodies != associated_body_ptr_vector.size())
      return false;
    for (const auto &associated_body_ptr : associated_body_ptr_vector)
      if (!LoadBodyData(associated_body_ptr, ifs)) return false;
  }
  return true;
}

RegionModel::AssociatedRendererGeometryPtrs
RegionModel::GenerateSetUpAssociatedRendererGeometries(
    int n_renderer_geometries) const {
  AssociatedRendererGeometryPtrs associated_renderer_geometry_ptrs;
  if (!movable_body_ptrs_.empty()) {
    associated_renderer_geometry_ptrs.occlusion =
        GenerateSetUpRendererGeometries(n_renderer_geometries);
  }
  if (!fixed_same_region_body_ptrs_.empty() ||
      !movable_same_region_body_ptrs_.empty()) {
    associated_renderer_geometry_ptrs.same_region =
        GenerateSetUpRendererGeometries(n_renderer_geometries);
  }
  if (!movable_body_ptrs_.empty() || !fixed_same_region_body_ptrs_.empty() ||
      !movable_same_region_body_ptrs_.empty()) {
    associated_renderer_geometry_ptrs.foreground =
        GenerateSetUpRendererGeometries(n_renderer_geometries);
    associated_renderer_geometry_ptrs.background =
        GenerateSetUpRendererGeometries(n_renderer_geometries);
  }
  return associated_renderer_geometry_ptrs;
}

bool RegionModel::SetUpAssociatedRenderers(
    const AssociatedRendererGeometryPtrs &associated_renderer_geometry_ptrs,
    int thread_num, AssociatedRendererPtrs *associated_renderers) {
  if (!movable_body_ptrs_.empty()) {
    if (!SetUpRenderer(associated_renderer_geometry_ptrs.occlusion[thread_num],
                       body_ptr_, &associated_renderers->occlusion))
      return false;
  }
  if (!fixed_same_region_body_ptrs_.empty() ||
      !movable_same_region_body_ptrs_.empty()) {
    if (!SetUpRenderer(
            associated_renderer_geometry_ptrs.same_region[thread_num],
            body_ptr_, &associated_renderers->same_region))
      return false;
  }
  if (!movable_body_ptrs_.empty() || !fixed_same_region_body_ptrs_.empty() ||
      !movable_same_region_body_ptrs_.empty()) {
    if (!SetUpRenderer(associated_renderer_geometry_ptrs.foreground[thread_num],
                       body_ptr_, &associated_renderers->foreground) ||
        !SetUpRenderer(associated_renderer_geometry_ptrs.background[thread_num],
                       body_ptr_, &associated_renderers->background))
      return false;
  }
  return true;
}

bool RegionModel::AddBodiesToAssociatedRenderers(
    const AssociatedRendererPtrs &associated_renderer_ptrs) {
  // Setup movable render
  if (!movable_body_ptrs_.empty()) {
    if (!AddBodiesToRenderer(associated_renderer_ptrs.occlusion, {body_ptr_},
                             kBackgroundID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.occlusion,
                             fixed_body_ptrs_, kBackgroundID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.occlusion,
                             movable_body_ptrs_, kMainBodyID))
      return false;
  }

  // Setup same-region render
  if (!fixed_same_region_body_ptrs_.empty() ||
      !movable_same_region_body_ptrs_.empty()) {
    if (!AddBodiesToRenderer(associated_renderer_ptrs.same_region, {body_ptr_},
                             kBackgroundID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.same_region,
                             fixed_body_ptrs_, kBackgroundID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.same_region,
                             fixed_same_region_body_ptrs_, kMainBodyID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.same_region,
                             movable_same_region_body_ptrs_, kMainBodyID))
      return false;
  }

  // Setup foreground and background render
  if (!movable_body_ptrs_.empty() || !fixed_same_region_body_ptrs_.empty() ||
      !movable_same_region_body_ptrs_.empty()) {
    if (!AddBodiesToRenderer(associated_renderer_ptrs.foreground, {body_ptr_},
                             kMainBodyID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.foreground,
                             fixed_body_ptrs_, kBackgroundID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.foreground,
                             movable_body_ptrs_, kBackgroundID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.foreground,
                             fixed_same_region_body_ptrs_, kMainBodyID))
      return false;
    if (!AddBodiesToRenderer(associated_renderer_ptrs.background, {body_ptr_},
                             kMainBodyID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.background,
                             fixed_body_ptrs_, kBackgroundID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.background,
                             fixed_same_region_body_ptrs_, kMainBodyID) ||
        !AddBodiesToRenderer(associated_renderer_ptrs.background,
                             movable_same_region_body_ptrs_, kMainBodyID))
      return false;
  }
  return true;
}

void RegionModel::RenderAndFetchAssociatedImages(
    const AssociatedRendererPtrs &associated_renderer_ptrs,
    const Transform3fA &camera2world_pose) {
  RenderAndFetchSilhouetteImage(associated_renderer_ptrs.same_region,
                                camera2world_pose);
  RenderAndFetchSilhouetteImage(associated_renderer_ptrs.occlusion,
                                camera2world_pose);
  RenderAndFetchSilhouetteImage(associated_renderer_ptrs.foreground,
                                camera2world_pose);
  RenderAndFetchSilhouetteImage(associated_renderer_ptrs.background,
                                camera2world_pose);
}

bool RegionModel::GeneratePointData(
    const FullSilhouetteRenderer &main_renderer,
    const AssociatedRendererPtrs &associated_renderer_ptrs,
    const Transform3fA &camera2body_pose, std::vector<DataPoint> *data_points,
    float *contour_length) const {
  // Compute silhouette
  const cv::Mat &silhouette_image{main_renderer.silhouette_image()};

  // Generate contours
  int pixel_contour_length;
  std::vector<std::vector<cv::Point2i>> contours;
  if (!GenerateValidContours(silhouette_image, &contours, &pixel_contour_length))
    return false;

  // cv::Mat silhouette_display{};
  // cv::resize(silhouette_image, silhouette_display, cv::Size(), 0.5, 0.5);
  // cv::imshow("silhouette", silhouette_display);
  // cv::waitKey();

  if (pixel_contour_length == 0) {
    *contour_length = 0.0f;
    return true;
  }

  // Validate contours
  std::vector<cv::Point2i> valid_contour_points;
  float pixel_to_meter = sphere_radius_ / main_renderer.intrinsics().fu;
  float max_depth_difference = pixel_to_meter * kMaxSurfaceGradient;
  for (auto &contour : contours) {
    // cv::Mat vis_img;
    // silhouette_image.copyTo(vis_img);

    // for (auto &p : contour) {
    //   cv::circle(vis_img, p, 2, 128, cv::FILLED);
    // }

    // cv::resize(vis_img, vis_img, cv::Size(), 0.5, 0.5);
    // cv::imshow("contour", vis_img);
    // cv::waitKey();

    for (auto &point : contour) {
      if (IsContourPointValid(max_depth_difference, point, main_renderer,
                              associated_renderer_ptrs))
        valid_contour_points.push_back(point);
    }

    //{
    //   cv::Mat vis_img;
    //   silhouette_image.copyTo(vis_img);

    //  for (auto &p : valid_contour_points) {
    //    cv::circle(vis_img, p, 4, 128, cv::FILLED);
    //  }

    //  //cv::resize(vis_img, vis_img, cv::Size(), 0.5, 0.5);
    //  cv::imshow("valid contour", vis_img);
    //  cv::waitKey();
    //}
  }
  *contour_length = float(valid_contour_points.size()) * pixel_to_meter;
  if (*contour_length == 0.0f) return true;

  // Set up generator
  std::mt19937 generator{7};
  if (use_random_seed_)
    generator.seed(unsigned(std::chrono::system_clock::now().time_since_epoch().count()));

  static constexpr std::size_t aligned_lines_num = 0;

  auto num_data_points = data_points->size() - aligned_lines_num;
  auto num_contour_points = valid_contour_points.size();

  float step_f = static_cast<float>(num_contour_points) / num_data_points;
  float contour_point_idx_f = 0;

  // Generate DataPoints from valid contour points
  int n_tries = 0;
  for (auto data_point = data_points->begin(); data_point != data_points->end() - aligned_lines_num;) {
    // Limit the number of tries
    if (n_tries++ > kMaxPointSamplingTries) {
      *contour_length = 0.0f;
      return true;
    }

    // Randomly sample point on contour
    // cv::Point2i center = SampleContourPointCoordinate(valid_contour_points, generator);
    cv::Point2i center = valid_contour_points[static_cast<std::size_t>(contour_point_idx_f)];

    // if (data_point->center_f_body.maxCoeff() >= 0.05) {
    //   continue;
    // }

    // auto center_u = center_f_camera(0) * main_renderer.intrinsics().fu /
    //                     center_f_camera(2) +
    //                 main_renderer.intrinsics().ppu;

    // auto center_v = center_f_camera(1) * main_renderer.intrinsics().fv /
    //                     center_f_camera(2) +
    //                 main_renderer.intrinsics().ppv;

    // auto norm = cv::norm(static_cast<cv::Point2f>(center) -
    // cv::Point2f(center_u, center_v)); assert(norm <= 10);

    // Calculate contour segment and approximate normal vector
    std::vector<cv::Point2i> contour_segment;
    if (!CalculateContourSegment(contours, center, &contour_segment))
      continue;
    Eigen::Vector2f normal = ApproximateNormalVector(contour_segment);
    Eigen::Vector3f normal_f_camera{normal.x(), normal.y(), 0.0f};
    data_point->normal_f_body = camera2body_pose.rotation() * normal_f_camera;

    // Calculate 3D center
    Eigen::Vector3f center_f_camera = main_renderer.PointVector(center, cv::Point2f(normal.x(), normal.y()));
    data_point->center_f_body = camera2body_pose * center_f_camera;

    // Calculate depth offset
    float pixel_to_meter = center_f_camera(2) / main_renderer.intrinsics().fu;
    CalculateDepthOffsets(main_renderer, center, pixel_to_meter,
                          &data_point->depth_offsets);

    // Calculate foreground and background distance
    CalculateLineDistances(main_renderer, associated_renderer_ptrs, contours,
                           center, normal, pixel_to_meter,
                           &data_point->foreground_distance,
                           &data_point->background_distance);

    data_point++;
    n_tries = 0;
    contour_point_idx_f += step_f;
  }

  std::partial_sort(data_points->begin(), data_points->begin() + aligned_lines_num, data_points->end(), [](DataPoint &left, DataPoint &right) {
    return left.center_f_body.y() > right.center_f_body.y();
  });

  for (std::size_t front_idx = 0; front_idx < aligned_lines_num; ++front_idx) {
    std::size_t back_idx = data_points->size() - 1 - front_idx;
    auto &data_point = data_points->at(back_idx); 
    
    data_point = data_points->at(front_idx);
    
    auto center_body = data_point.center_f_body;
    auto normal_body = Eigen::Vector3f(0, 1, 0);

    auto start_cam = camera2body_pose.inverse() * center_body;
    auto end_cam = camera2body_pose.inverse() * (center_body  + normal_body);

    Eigen::Vector2f start_proj = start_cam.head<2>() / start_cam.z();
    Eigen::Vector2f end_proj = end_cam.head<2>() / end_cam.z();

    Eigen::Vector2f normal = end_proj - start_proj;
    Eigen::Vector3f normal_f_camera{normal.x(), normal.y(), 0.0f};
    data_point.normal_f_body = camera2body_pose.rotation() * normal_f_camera;

    // Calculate depth offset
    float pixel_to_meter = start_cam(2) / main_renderer.intrinsics().fu;
    CalculateDepthOffsets(main_renderer, cv::Point2i(start_cam.x(), start_cam.y()), pixel_to_meter,
                          &data_point.depth_offsets);

    // Calculate foreground and background distance
    CalculateLineDistances(main_renderer, associated_renderer_ptrs, contours,
                           cv::Point2i(start_cam.x(), start_cam.y()), normal, pixel_to_meter,
                           &data_point.foreground_distance,
                           &data_point.background_distance);
  }

  return true;
}

bool RegionModel::GenerateValidContours(
    const cv::Mat &silhouette_image,
    std::vector<std::vector<cv::Point2i>> *contours,
    int *pixel_contour_length) const {
  // Set everything except for main body to black
  cv::Mat main_body_silhouette_image;
  cv::Mat lookUpTable(1, 256, CV_8U, cv::Scalar(0));
  lookUpTable.data[kMainBodyID] = kMainBodyID;
  cv::LUT(silhouette_image, lookUpTable, main_body_silhouette_image);

  // Compute contours
  cv::findContours(main_body_silhouette_image, *contours,
                   cv::RetrievalModes::RETR_LIST, cv::CHAIN_APPROX_NONE);

  // Filter contours that are too short
  contours->erase(std::remove_if(begin(*contours), end(*contours),
                                 [](const std::vector<cv::Point2i> &contour) {
                                   return contour.size() < kMinContourLength;
                                 }),
                  end(*contours));

  // Test if contours are closed
  for (auto &contour : *contours) {
    // for (auto &point : contour) {
    //   cv::circle(main_body_silhouette_image, point, 1, 128, cv::FILLED);
    // }
    //
    // cv::Mat resized;
    // cv::resize(main_body_silhouette_image, resized, cv::Size(), 0.5, 0.5);
    // cv::imshow("silhouette_image", resized);
    // cv::waitKey(0);

    if (abs(contour.front().x - contour.back().x) > 1 ||
        abs(contour.front().y - contour.back().y) > 1) {
      std::cerr << "Contours are not closed." << std::endl;
      return false;
    }
  }

  // Calculate total pixel length of contour
  *pixel_contour_length = 0;
  for (auto &contour : *contours) {
    *pixel_contour_length += int(contour.size());
  }

  // Check if pixel length is greater zero
  if (*pixel_contour_length == 0) {
    std::cerr << "No valid contour in image." << std::endl;
  }
  return true;
}

bool RegionModel::IsContourPointValid(
    float max_depth_difference, const cv::Point2i &image_coordinates,
    const FullSilhouetteRenderer &main_renderer,
    const AssociatedRendererPtrs &associated_renderer_ptrs) {
  auto min_width = main_renderer.intrinsics().width * 0.01;
  auto max_width = main_renderer.intrinsics().width * 0.99;
  auto min_height = main_renderer.intrinsics().height * 0.01;
  auto max_height = main_renderer.intrinsics().height * 0.99;

  if (image_coordinates.x < min_width || image_coordinates.x > max_width ||
      image_coordinates.y < min_height || image_coordinates.y > max_height)
    return false;

  std::vector<cv::Point2i> neighboring_points = {
      cv::Point2i(image_coordinates.x, image_coordinates.y + 1),
      cv::Point2i(image_coordinates.x, image_coordinates.y - 1),
      cv::Point2i(image_coordinates.x + 1, image_coordinates.y),
      cv::Point2i(image_coordinates.x - 1, image_coordinates.y),
  };

  // Neighboring same-region bodies
  if (associated_renderer_ptrs.same_region) {
    for (auto &point : neighboring_points) {
      if (associated_renderer_ptrs.same_region->silhouette_image().at<uchar>(
              point) != kBackgroundID)
        return false;
    }
  }

  // Occlusions from movable body
  if (associated_renderer_ptrs.occlusion) {
    if (associated_renderer_ptrs.occlusion->silhouette_image().at<uchar>(
            image_coordinates) != kBackgroundID)
      return false;
  }

  // Fixed body with bigger average depth than center
  float sum_depth_neighboring_fixed_body_pixels = 0;
  int number_neighboring_fixed_body_pixels = 0;
  for (auto &point : neighboring_points) {
    if (main_renderer.silhouette_image().at<uchar>(point) == kDifferentBodyID) {
      sum_depth_neighboring_fixed_body_pixels += main_renderer.Depth(point);
      number_neighboring_fixed_body_pixels++;
    }
  }
  if (number_neighboring_fixed_body_pixels > 0) {
    if (sum_depth_neighboring_fixed_body_pixels /
            float(number_neighboring_fixed_body_pixels) <
        main_renderer.Depth(image_coordinates) - max_depth_difference)
      return false;
  }
  return true;
}

cv::Point2i RegionModel::SampleContourPointCoordinate(
    const std::vector<cv::Point2i> &valid_contour_points,
    std::mt19937 &generator) {
  int idx = int(generator() % valid_contour_points.size());
  return valid_contour_points[idx];
}

bool RegionModel::CalculateContourSegment(
    const std::vector<std::vector<cv::Point2i>> &contours, cv::Point2i &center,
    std::vector<cv::Point2i> *contour_segment) {
  for (auto &contour : contours) {
    for (int idx = 0; idx < contour.size(); ++idx) {
      if (contour.at(idx) == center) {
        int start_idx = idx - kContourNormalApproxRadius;
        int end_idx = idx + kContourNormalApproxRadius;
        if (start_idx < 0) {
          contour_segment->insert(end(*contour_segment),
                                  end(contour) + start_idx, end(contour));
          start_idx = 0;
        }
        if (end_idx >= int(contour.size())) {
          contour_segment->insert(end(*contour_segment),
                                  begin(contour) + start_idx, end(contour));
          start_idx = 0;
          end_idx = end_idx - int(contour.size());
        }
        contour_segment->insert(end(*contour_segment),
                                begin(contour) + start_idx,
                                begin(contour) + end_idx + 1);

        // Check quality of contour segment
        float segment_distance = std::hypotf(
            float(contour_segment->back().x - contour_segment->front().x),
            float(contour_segment->back().y - contour_segment->front().y));
        return segment_distance > float(kContourNormalApproxRadius);
      }
    }
  }
  std::cerr << "Could not find point on contour" << std::endl;
  return false;
}

Eigen::Vector2f RegionModel::ApproximateNormalVector(
    const std::vector<cv::Point2i> &contour_segment) {
  return Eigen::Vector2f{
      -float(contour_segment.back().y - contour_segment.front().y),
      float(contour_segment.back().x - contour_segment.front().x)}
      .normalized();
}

bool is_in_bounds(Intrinsics intrinsics, int x, int y) {
  return x >= 0 && x < intrinsics.width && y >= 0 && y < intrinsics.height;
}

void RegionModel::CalculateLineDistances(
    const FullSilhouetteRenderer &main_renderer,
    const AssociatedRendererPtrs &associated_renderer_ptrs,
    const std::vector<std::vector<cv::Point2i>> &contours,
    const cv::Point2i &center, const Eigen::Vector2f &normal,
    float pixel_to_meter, float *foreground_distance,
    float *background_distance) const {
  // Define which images are used to calculate line distances
  const cv::Mat *foreground_silhouette_image;
  const cv::Mat *background_silhouette_image;
  if (associated_renderer_ptrs.foreground ||
      associated_renderer_ptrs.background) {
    foreground_silhouette_image =
        &associated_renderer_ptrs.foreground->silhouette_image();
    background_silhouette_image =
        &associated_renderer_ptrs.background->silhouette_image();
  } else {
    foreground_silhouette_image = &main_renderer.silhouette_image();
    background_silhouette_image = &main_renderer.silhouette_image();
  }

  // Calculate starting positions and steps for both sides of the line
  float u_out = float(center.x) + 0.5f;
  float v_out = float(center.y) + 0.5f;
  float u_in = float(center.x) + 0.5f;
  float v_in = float(center.y) + 0.5f;
  float u_step, v_step;
  if (std::fabs(normal.y()) < std::fabs(normal.x())) {
    u_step = float(sgn(normal.x()));
    v_step = normal.y() / abs(normal.x());
  } else {
    u_step = normal.x() / abs(normal.y());
    v_step = float(sgn(normal.y()));
  }

  // Search for first inwards intersection with contour
  int u_in_endpoint, v_in_endpoint;
  while (true) {
    u_in -= u_step;
    v_in -= v_step;
    if (!is_in_bounds(main_renderer.intrinsics(), u_in, v_in)) {
      *foreground_distance = std::numeric_limits<float>::max();
      break;
    }

    if (foreground_silhouette_image->at<uchar>(int(v_in), int(u_in)) !=
        kMainBodyID) {
      FindClosestContourPoint(contours, u_in + u_step - 0.5f,
                              v_in + v_step - 0.5f, &u_in_endpoint,
                              &v_in_endpoint);
      *foreground_distance =
          pixel_to_meter * hypotf(float(u_in_endpoint - center.x),
                                  float(v_in_endpoint - center.y));
      break;
    }
  }

  // Search for first outwards intersection with contour
  int u_out_endpoint, v_out_endpoint;
  while (true) {
    u_out += u_step;
    v_out += v_step;
    if (!is_in_bounds(main_renderer.intrinsics(), u_out, v_out)) {
      *background_distance = std::numeric_limits<float>::max();
      break;
    }
    if (background_silhouette_image->at<uchar>(int(v_out), int(u_out)) ==
        kMainBodyID) {
      FindClosestContourPoint(contours, u_out - 0.5f, v_out - 0.5f,
                              &u_out_endpoint, &v_out_endpoint);
      *background_distance =
          pixel_to_meter * hypotf(float(u_out_endpoint - center.x),
                                  float(v_out_endpoint - center.y));
      break;
    }
  }
}

void RegionModel::FindClosestContourPoint(
    const std::vector<std::vector<cv::Point2i>> &contours, float u, float v,
    int *u_contour, int *v_contour) {
  float min_distance = std::numeric_limits<float>::max();
  for (auto &contour : contours) {
    for (auto &point : contour) {
      float distance = hypotf(float(point.x) - u, float(point.y) - v);
      if (distance < min_distance) {
        *u_contour = point.x;
        *v_contour = point.y;
        min_distance = distance;
      }
    }
  }
}

}  // namespace m3t
