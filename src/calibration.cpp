/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <chrono>
#include <iostream>
#include <thread>

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

#include <tbb/concurrent_unordered_map.h>

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/image/typed_image.h>
#include <pangolin/pangolin.h>

#include <visnav/local_parameterization_se3.hpp>

#include <visnav/aprilgrid.h>
#include <visnav/calibration.h>
#include <visnav/reprojection.h>

#include <visnav/serialization.h>

#include <CLI/CLI.hpp>

using namespace visnav;

void drawImageOverlay(pangolin::View& v, size_t cam_id);
void load_data(const std::string& path);
void compute_projections();
void save_calib();
void optimize();

// describes the physical layout of the april grid used
AprilGrid aprilgrid;

// calibration: camera intrinsics and camera-to-camera extrinsics
Calibration calib_cam;

// camera poses (IMU-to-world transformation; for us, IMU frame coincides with
// camera 0)
std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> vec_T_w_i;

// detected 2d corners for every image
tbb::concurrent_unordered_map<FrameCamId, CalibCornerData> calib_corners;

// initial poses (loaded from file) for every image
tbb::concurrent_unordered_map<FrameCamId, CalibInitPoseData> calib_init_poses;

// loaded images
tbb::concurrent_unordered_map<FrameCamId, pangolin::TypedImage> calib_images;

// projected 3d corners for every image;
// They are computed in `compute_projections` based on current estimates of
// camera poses, intrinsics and extrinsics, and are used for visualization.
tbb::concurrent_unordered_map<FrameCamId, CalibCornerData> opt_corners;

pangolin::Var<int> show_frame("ui.show_frame", 0, 0, 1500);
pangolin::Var<bool> show_detected("ui.show_detected", true, true);
pangolin::Var<bool> show_opt("ui.show_opt", true, true);

std::string dataset_path;
std::string cam_model = "ds";

int main(int argc, char** argv) {
  const int UI_WIDTH = 200;
  const int NUM_CAMS = 2;

  bool show_gui = true;

  CLI::App app{"App description"};

  app.add_option("--show-gui", show_gui, "Show GUI");
  app.add_option("--dataset-path", dataset_path, "Dataset path")->required();
  app.add_option(
      "--cam-model", cam_model,
      "Camera model. Possible values: pinhole, ds, eucm, kb4. Default: ds.");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  load_data(dataset_path);
  compute_projections();

  if (show_gui) {
    pangolin::CreateWindowAndBind("Main", 1800, 1000);

    pangolin::View* img_view_display;

    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;

    img_view_display =
        &pangolin::CreateDisplay()
             .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
             .SetLayout(pangolin::LayoutEqual);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    while (img_view.size() < NUM_CAMS) {
      std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

      size_t idx = img_view.size();
      img_view.push_back(iv);

      img_view_display->AddDisplay(*iv);
      iv->extern_draw_function =
          std::bind(&drawImageOverlay, std::placeholders::_1, idx);
    }

    pangolin::Var<std::function<void(void)>> optimize_btn("ui.optimize",
                                                          std::bind(&optimize));
    pangolin::Var<std::function<void(void)>> save_calib_btn(
        "ui.save_calib", std::bind(&save_calib));

    while (!pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (show_frame.GuiChanged()) {
        FrameId frame_id = static_cast<FrameId>(show_frame);

        for (CamId cam_id = 0; cam_id < NUM_CAMS; cam_id++) {
          FrameCamId fcid;
          fcid.frame_id = frame_id;
          fcid.cam_id = cam_id;
          if (calib_images.find(fcid) != calib_images.end()) {
            img_view[cam_id]->SetImage(calib_images[fcid]);
          } else {
            img_view[cam_id]->Clear();
          }
        }
      }

      pangolin::FinishFrame();

      // prevent the GUI from burning too much CPU when just idling
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  } else {
    optimize();
    save_calib();
  }

  return 0;
}

void drawImageOverlay(pangolin::View& v, size_t cam_id) {
  UNUSED(v);

  FrameId frame_id = static_cast<FrameId>(show_frame);

  FrameCamId fcid(frame_id, cam_id);

  if (show_detected) {
    glLineWidth(1.0);
    glColor3f(1.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (calib_corners.find(fcid) != calib_corners.end()) {
      const CalibCornerData& cr = calib_corners.at(fcid);

      for (size_t i = 0; i < cr.corners.size(); i++) {
        Eigen::Vector2d c = cr.corners[i];
        pangolin::glDrawCirclePerimeter(c[0], c[1], 3.0);
      }

      pangolin::GlFont::I()
          .Text("Detected %d corners", cr.corners.size())
          .Draw(5, 50);

    } else {
      glLineWidth(1.0);

      pangolin::GlFont::I().Text("Corners not processed").Draw(5, 50);
    }
  }

  if (show_opt) {
    glLineWidth(1.0);
    glColor3f(1.0, 0.0, 1.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (opt_corners.find(fcid) != opt_corners.end()) {
      const CalibCornerData& cr = opt_corners.at(fcid);

      for (size_t i = 0; i < cr.corners.size(); i++) {
        Eigen::Vector2d c = cr.corners[i];
        pangolin::glDrawCirclePerimeter(c[0], c[1], 3.0);
      }

      pangolin::GlFont::I()
          .Text("Detected %d corners", cr.corners.size())
          .Draw(5, 50);

    } else {
      glLineWidth(1.0);

      pangolin::GlFont::I().Text("Too few corners detected").Draw(5, 100);
    }
  }
}

/// This function loads images as well as some preprocessed data prepared for
/// the exercises: detected corners, initial guesses for camera intrinsic and
/// extrinsic caliubration as well as initial pose guesses. It initializes the
/// global data structures like `calib_init_poses`, `calib_corners`,
/// `calib_cam`, `vec_T_w_i` and `calib_images`.
void load_data(const std::string& dataset_path) {
  std::string poses_path = dataset_path + "/init_poses.json";
  std::string corners_path = dataset_path + "/detected_corners.json";
  std::string calib_path = dataset_path + "/calibration-double-sphere.json";

  {
    std::ifstream os(poses_path, std::ios::binary);

    if (os.is_open()) {
      cereal::JSONInputArchive archive(os);
      archive(calib_init_poses);
      std::cout << "Loaded " << calib_init_poses.size() << " poses "
                << std::endl;

    } else {
      std::cerr << "could not load poses from " << poses_path << std::endl;
    }
  }

  {
    std::ifstream os(corners_path, std::ios::binary);

    if (os.is_open()) {
      cereal::JSONInputArchive archive(os);
      archive(calib_corners);
      std::cout << "Loaded " << calib_corners.size() << " corners "
                << std::endl;

    } else {
      std::cerr << "could not load corners from " << poses_path << std::endl;
    }
  }

  {
    std::ifstream os(calib_path, std::ios::binary);

    LoadCalibration<double, DoubleSphereCamera<double>> loaded_cam_calib;
    loaded_cam_calib.T_i_c.resize(2);
    loaded_cam_calib.intrinsics.resize(2);

    if (os.is_open()) {
      cereal::JSONInputArchive archive(os);
      archive(loaded_cam_calib);
      std::cout << "Loaded camera" << std::endl;

      calib_cam.T_i_c = loaded_cam_calib.T_i_c;

      calib_cam.intrinsics.clear();
      for (size_t i = 0; i < loaded_cam_calib.intrinsics.size(); i++) {
        calib_cam.intrinsics.push_back(AbstractCamera<double>::initialize(
            cam_model, loaded_cam_calib.intrinsics[i].data()));
      }

    } else {
      std::cerr << "could not load camera " << poses_path << std::endl;
    }
  }

  vec_T_w_i.resize(calib_corners.size() / 2);

  for (const auto& kv : calib_corners) {
    std::stringstream ss;
    ss << dataset_path << "/" << kv.first.frame_id << "_" << kv.first.cam_id
       << ".jpg";

    pangolin::TypedImage img = pangolin::LoadImage(ss.str());

    CamId cam_id = kv.first.cam_id;
    if (calib_cam.intrinsics.at(cam_id)->width() == 0) {
      // set the width and height from first image
      calib_cam.intrinsics.at(cam_id)->width() = img.w;
      calib_cam.intrinsics.at(cam_id)->height() = img.h;
    }

    calib_images[kv.first] = std::move(img);

    if (calib_init_poses.find(kv.first) != calib_init_poses.end() && 
        kv.first.cam_id == 0) {
      Sophus::SE3d pose(calib_init_poses[kv.first].T_a_c.matrix());
      vec_T_w_i[kv.first.frame_id] = pose;
    }
  }

  show_frame.Meta().range[1] = calib_images.size() / 2 - 1;
  show_frame.Meta().gui_changed = true;
}

// Loop over all time-cam-ids and compute the projected 2d points based on the
// current estimates for camera-poses, camera-to-cameras extrinsics, and
// camera intrinsics. The result is stored in opt_corners and used for
// visualization.
void compute_projections() {
  opt_corners.clear();

  for (const auto& kv : calib_corners) {
    CalibCornerData ccd;

    // std::cout << "id " << kv.first.cam_id << std::endl;

    for (size_t i = 0; i < aprilgrid.aprilgrid_corner_pos_3d.size(); i++) {
      // Transformation from body (IMU) frame to world frame
      Sophus::SE3d T_w_i = vec_T_w_i[kv.first.frame_id];
      // Transformation from camera to body (IMU) frame
      Sophus::SE3d T_i_c = calib_cam.T_i_c[kv.first.cam_id];
      // 3D coordinates of the aprilgrid corner in the world frame
      Eigen::Vector3d p_3d = aprilgrid.aprilgrid_corner_pos_3d[i];

      // TODO SHEET 2: project point
      Eigen::Vector2d p_2d;

      Eigen::Vector3d p_3d_camera_frame =
          T_i_c.inverse() * T_w_i.inverse() * p_3d;

      p_2d = calib_cam.intrinsics[kv.first.cam_id]->project(p_3d_camera_frame);

      ccd.corners.push_back(p_2d);
    }

    opt_corners[kv.first] = ccd;
  }
}

void optimize() {
  // Build the problem.
  ceres::Problem problem;

  // std::cout << "problem created " << std::endl;

  // TODO SHEET 2: setup optimization problem
  /*
  unknowns:

  intrinsic parameters for camera i
  calib_cam.T_i_c[i] camera pose for each camera i
  calib_cam.intrinsics[i] intrinsic parameters for camera i

  extrinsic parameters
  vec_T_w_i[i] body pose for each frame i ?

  given:
  initial guess?
  3d positions of the keypoints
  2d detections of the keypoints
  camera model

  */

  // Specify local update rule for our parameter
  // similar to test_ceres_se3.cpp file and
  // http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres7Problem19SetParameterizationEPdP21LocalParameterization

  // calib_cam.T_i_c[i] camera pose for each camera i
  for (Sophus::SE3d& Tic_i : calib_cam.T_i_c) {
    problem.AddParameterBlock(Tic_i.data(), Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);
  }

  // std::cout << "Tic_i added " << std::endl;

  // the first cam transformation is fixed
  if (calib_cam.T_i_c.size() > 0) {
    problem.SetParameterBlockConstant(calib_cam.T_i_c[0].data());
  }

  // std::cout << "SetParameterBlockConstant" << std::endl;

  // calib_cam.T_i_c[i] camera pose for each camera i
  for (auto intrinsics_i : calib_cam.intrinsics) {
    problem.AddParameterBlock(intrinsics_i->data(), 8);
  }

  // std::cout << "intrinsics_i added " << std::endl;

  // vec_T_w_i[i] camera pose for each camera i
  for (Sophus::SE3d& T_w_i : vec_T_w_i) {
    problem.AddParameterBlock(T_w_i.data(), Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);
  }

  // std::cout << "T_w_i added " << std::endl;

  // map<FrameCamId, CalibCornerData> calib_corners;
  // for each frame with FrameCamId
  for (const auto& kv : calib_corners) {
    // std::cout << "id " << kv.first.cam_id << std::endl;
    // kv.first >> FrameCamId
    // FrameCamId{frame_id, cam_id}
    // kv.second >> CalibCornerData
    // CalibCornerData{corners, corner_ids}

    // intrinsics:
    // Transformation from body (IMU) frame to world frame
    double* intrinsics_i = calib_cam.intrinsics[kv.first.cam_id]->data();
    // std::cout << "intrinsics_i added " << std::endl;
    // Transformation from camera to body (IMU) frame
    Sophus::SE3d& T_i_c = calib_cam.T_i_c[kv.first.cam_id];
    // std::cout << "T_i_c added " << std::endl;

    // extrinsics:
    // Transformation from body (IMU) frame to world frame (world frame is april
    // board frame?)
    Sophus::SE3d& T_w_i = vec_T_w_i[kv.first.frame_id];
    // std::cout << "extrinsics added " << std::endl;

    // for each 2d detection and corresponding 3d point
    Eigen::Vector2d p_2d;  // detected 2d point
    Eigen::Vector3d p_3d;  // 3d point in the world frame
    int point_id;          // id to take 3d position from the aprilgrid
    for (size_t i = 0; i < kv.second.corners.size(); i++) {
      point_id = kv.second.corner_ids[i];
      // std::cout << "point_id added " << std::endl;
      p_2d = kv.second.corners[i];
      // std::cout << "p_2d added " << std::endl;
      p_3d = aprilgrid.aprilgrid_corner_pos_3d[point_id];
      // std::cout << "p_3d added " << std::endl;
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<visnav::ReprojectionCostFunctor, 2, 7,
                                          7, 8>(
              new visnav::ReprojectionCostFunctor(p_2d, p_3d, cam_model));
      // std::cout << "cost_function added " << std::endl;
      problem.AddResidualBlock(cost_function, nullptr, T_w_i.data(),
                               T_i_c.data(), intrinsics_i);
      // std::cout << "AddResidualBlock added " << std::endl;
    }
  }

  /////////////////////////
  ceres::Solver::Options options;
  options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.num_threads = std::thread::hardware_concurrency();

  // Solve
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  {
    cereal::JSONOutputArchive archive(std::cout);
    archive(calib_cam);
    std::cout << std::endl;
  }

  compute_projections();
}

void save_calib() {
  std::ofstream os("opt_calib.json");

  if (os.is_open()) {
    cereal::JSONOutputArchive archive(os);
    archive(calib_cam);
    std::cout << "Saved camera calibration" << std::endl;
  }
  os.close();
}
