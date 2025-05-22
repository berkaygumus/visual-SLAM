# Sheet 1: Repo build and Theory
* 3D geometry 
* lie group & algebra
* exponential & logarithmic map
* rodriguas' formula
* what is slam

# sheet 2: Camera models & camera calibration
* implement camera models: 
    * 3D point wrt camera to 2D wrt image plane:
    * pinhole
    * ds - Double Sphere, 
    * eucm - Extended Unified Camera Model, 
    * kb4 - Kannala-Brandt 4-parameter model

* learn ceres
    * Ceres as an optimization library. It provides a general
interface for solving non-linear least squares problem

* stereo camera calibration with checker board
    ```cpp
    /*
    unknowns:

    intrinsic parameters for camera i
    calib_cam.intrinsics[i] intrinsic parameters for camera i

    extrinsic parameters
    calib_cam.T_i_c[i] camera pose wrt body(IMU) for each camera i
    vec_T_w_i[i] body(IMU) pose wrt world (apriltag board) for each frame i ?

    given:
    initial guess for unknowns
    3d positions of the keypoints
    2d detections of the keypoints
    camera model

    */
    ```
    * given
        * init_poses.json: camera pose(p,q) and projected points (key points) 
        ```cpp
        // initial poses (loaded from file) for every image
        tbb::concurrent_unordered_map<FrameCamId, CalibInitPoseData> calib_init_poses; 
        ```
        * detected_corners.json: detected points (key points)
        ```cpp
        // detected 2d corners for every image
        tbb::concurrent_unordered_map<FrameCamId, CalibCornerData> calib_corners;
        ```
        * calibration-double-sphere.json:
        ```cpp
        // calibration: camera intrinsics and camera-to-camera extrinsics
        Calibration calib_cam;
        ```
    * non-linear least squares problem:
        * residual: 2d error between projection and detection
        ```cpp
        residuals = p_2d - cam->project(T_i_c.inverse() * T_w_i.inverse() * p_3d);
        ```
    * output
        * opt_calib.json: camera calibration
        ```cpp
        // calibration: camera intrinsics and camera-to-camera extrinsics
        Calibration calib_cam;
        ```
    * TODO:
        * check optimization parameters, linear-system
        * (J^T J + D) dx = -J^T r
        * ceres::SPARSE_NORMAL_CHOLESKY, ceres::DENSE_QR, ceres::SPARSE_SCHUR

# sheet 3: Feature Detectors, Descriptors, Epipolar Geometry, RANSAC
* detect keypoints
    * check the changes in x and y-> H matrix
    *  Shi-Tomasi algorithm (https://docs.opencv.org/4.x/d4/d8c/tutorial_py_shi_tomasi.html) is provided
* generate descriptors
    * compute angles: direction of the intensity gradient
    * compute descriptors (ORB uses BRIEF): binary descriptor with 256 bits
* keypoint matching
    * brute force:  check distance p-q< thr,pi and qi must be unique, distance must be much less than the distance to second best
* epipolar constraint for stereo matches
    * compute essential matrix E = t_skew * R using camera calibration, T between two cameras
    * filter outliers
    ```cpp
    abs(cam1->unproject(p0_2d).transpose() * E * cam2->unproject(p1_2d)) <= epipolar_error_threshold
    ```
    * calls for each stereo image pair
    ```cpp
    for (int i = 0; i < num_images; i++) {
        const FrameCamId fcid1(i, 0), fcid2(i, 1);
        findInliersEssential
        ...
    }
    ```
* Five-point algorithm and RANSAC for non-stereo matches
    * Five-point algorithm: Transformation can be estimated with 5 points (matches). OpenGV provides RANSAC for 5 points and gives T and inliers.
    * 1. Use OpenGV adapter to find inliers
    * 2. use all inliers and nonlinear opt for transformation
    ```cpp
    // from
    // https://laurentkneip.github.io/opengv/page_how_to_use.html#sec_conventions

    // 1) ------>firstly, find inliers with RANSAC

    // create the central relative adapter
    opengv::relative_pose::CentralRelativeAdapter adapter(bearingVectors1,
                                                            bearingVectors2);

    // create a RANSAC object
    opengv::sac::Ransac<
        opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>
        ransac;
    // create a CentralRelativePoseSacProblem
    // (set algorithm to NISTER 5 point algorithm)
    std::shared_ptr<
        opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>
        relposeproblem_ptr(
            new opengv::sac_problems::relative_pose::
                CentralRelativePoseSacProblem(
                    adapter, opengv::sac_problems::relative_pose::
                                CentralRelativePoseSacProblem::NISTER));
    // run ransac
    ransac.sac_model_ = relposeproblem_ptr;
    ransac.threshold_ = ransac_thresh;
    ransac.max_iterations_ = 1000;  // default 1000
    if (!ransac.computeModel()) {
        return;
    }
    // get the result
    opengv::transformation_t best_transformation = ransac.model_coefficients_;
    std::vector<int> inliers_ = ransac.inliers_;

    if (int(inliers_.size()) < ransac_min_inliers) {
        // If the number of inliers is smaller than
        // ransac_min_inliers, leave md.inliers empty
        return;
    }

    // 2) ------>secondly, optimize with all inliers
    // non-linear optimization (using all inlier correspondences)

    opengv::bearingVectors_t inlier_bearingVectors1, inlier_bearingVectors2;

    for (int inlier_index : inliers_) {
        inlier_bearingVectors1.emplace_back(bearingVectors1[inlier_index]);
        inlier_bearingVectors2.emplace_back(bearingVectors2[inlier_index]);

        md.inliers.emplace_back(std::pair<int, int>(
            md.matches[inlier_index].first, md.matches[inlier_index].second));
    }

    opengv::relative_pose::CentralRelativeAdapter nonlinear_adapter(
        inlier_bearingVectors1, inlier_bearingVectors2);
    // best_transformation is Eigen::Matrix<double,3,4
    nonlinear_adapter.sett12(best_transformation.block(0, 3, 3, 1));
    nonlinear_adapter.setR12(best_transformation.block(0, 0, 3, 3));
    opengv::transformation_t nonlinear_transformation =
        opengv::relative_pose::optimize_nonlinear(nonlinear_adapter);

    // md.T_i_j.block(0, 0, 3, 4) = nonlinear_transformation;
    // md.T_i_j.setRotationMatrix(nonlinear_transformation.block(0, 0, 3, 3));
    Eigen::Vector3d t12 = nonlinear_transformation.block(0, 3, 3, 1);
    Eigen::Matrix3d R12 = nonlinear_transformation.block(0, 0, 3, 3);
    md.T_i_j = Sophus::SE3d(R12, t12.normalized());
    ```
    * calls in match_all(): for every non-stereo image pairs
    ```cpp
    std::vector<FrameCamId> keys;

    for (const auto& kv : images) keys.push_back(kv.first);

    std::vector<std::pair<int, int>> ids_to_match;

    for (size_t i = 0; i < keys.size(); i++) {
        for (size_t j = 0; j < i; j++) {
        // Do not add stereo pairs (have same timestamp)
        if (keys[i].frame_id != keys[j].frame_id) ids_to_match.emplace_back(i, j);
        }
    }
    ```
    * calls in match_bow(): get pairs from bow
    ```cpp
    std::vector<FrameCamId> keys;
    std::unordered_map<FrameCamId, int> id_to_key;
    std::vector<std::pair<int, int>> ids_to_match;

    for (const auto& kv : feature_corners) {
        int curr_id = keys.size();
        id_to_key[kv.first] = curr_id;
        keys.push_back(kv.first);

        BowVector v;
        bow_voc->transform(kv.second.corner_descriptors, v);

        BowQueryResult r;
        bow_db->query(v, num_bow_candidates, r);

        for (const auto& res_kv : r) {
        // Do not add stereo pairs (have same timestamp)
        if (kv.first.frame_id != res_kv.first.frame_id) {
            ids_to_match.emplace_back(curr_id, id_to_key.at(res_kv.first));
        }
        }

        bow_db->insert(kv.first, v);
    }
    // pairs -> ids_to_match
    ```
* Bag-of-Words for Place Recognition
    * a place recogition approach that allows us to find candidate pairs using the bag-of-words descriptor
    * Implement the transformFeatureToWord method that propagates a given feature through the vocabulary tree and saves the corresponding word and its weighting. After that, implement the transform method that builds a BoW descriptor from an array of features for a certain frame. You should use L1 normalization for the BoW descriptor.
    * implement insert() and query() methods.

# sheet 4: SfM, Triangulation, PnP, Bundle Adjustment
* get image pairs from match_all() or match_bow() with inlier matches

* initialize map: use the first stereo pairs and triangulate their inlier matches

* add new cameras: use 3d-2d correspondences (existign 3d landmarks and 2d detections), employ the <strong>perspective-n-point (PnP)</strong> algorithm in a RANSAC scheme
```cpp
// https://github.com/laurentkneip/opengv/blob/master/test/test_absolute_pose_sac.cpp#L94
// create the central adapter
opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearingVectors, points);
// create a Ransac object
opengv::sac::Ransac<
    opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
    ransac;
// create an AbsolutePoseSacProblem
// (algorithm is selectable: KNEIP, GAO, or EPNP)
std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
    absposeproblem_ptr(
        new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
            adapter, opengv::sac_problems::absolute_pose::
                        AbsolutePoseSacProblem::KNEIP));
```

* add new landmarks: For each of the new cameras, we check if there are any feature tracks that are not yet part of the map but are observed by at least 2 cameras. Each new landmark is triangulated based on a pair of images, using the estimated camera poses in the map. This uses the same triangulation that is also needed for map initialization
```cpp
opengv::relative_pose::CentralRelativeAdapter adapter(
        p0_3d_vector, p1_3d_vector, translation, rotation);
```
* perform map optimization to refine both the camera poses as well as the landmark positions. This is known as Bundle Adjustment (BA). This optimization is quite similar to the optimization we used in sheet 2 for camera calibration. The differences include: 
    * 1 we now also optimize the 3D positions, 
    * 2 we now don’t explicitly model the stereo camera such that every image gets its own pose, 
    * 3 we typically keep the intrinsics fixed.

```cpp
// p_2d observation
// T_w_c unknown: camera pose wrt world
// p_3d_w unknown: landmark pos wrt world
// camera instrinsic and extrinsic are fixed
// first camera is fixed (assumed as world frame)
residuals = p_2d - cam->project(T_w_c.inverse() * p_3d_w);
```
* check outliers and remove and then optimize again