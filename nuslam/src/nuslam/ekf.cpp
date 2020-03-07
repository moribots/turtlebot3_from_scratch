#include "nuslam/ekf.hpp"

namespace nuslam
{
	// Used to for prediction stage
    using rigid2d::Twist2D;

    // used for model update
    using rigid2d::Pose2D;

    // used for map update
    using rigid2d::Vector2D;

	// CovarianceMatrix
	CovarianceMatrix::CovarianceMatrix()
	{
        robot_state = Pose2D();  // init to 0,0,0
        std::vector<double> robot_state_cov{0.0, 0.0, 0.0}; // init to 0,0,0
        // 3*3
        // construct EigenVector from Vector
        Eigen::VectorXd robot_state_cov_vct = Eigen::VectorXd::Map(robot_state_cov.data(), robot_state_cov.size());
        Eigen::MatrixXd robot_cov_mtx = robot_state_cov_vct.asDiagonal();

        cov_mtx = robot_cov_mtx;
	}

	CovarianceMatrix::CovarianceMatrix(const Pose2D & robot_state_, const std::vector<Vector2D> & map_state_)
	{
		robot_state = robot_state_;  // init to 0,0,0
        std::vector<double> robot_state_cov{0.0, 0.0, 0.0}; // init to 0,0,0
        // 3*3
        // construct EigenVector from Vector
        Eigen::VectorXd robot_state_cov_vct = Eigen::VectorXd::Map(robot_state_cov.data(), robot_state_cov.size());
        Eigen::MatrixXd robot_cov_mtx = robot_state_cov_vct.asDiagonal();

        map_state = map_state_;
        // 2n*2n diag matrix since map containts x1,y1, ... xn,yn
        std::vector<double> map_state_cov(2 * map_state.size(), std::numeric_limits<double>::infinity()); // init to 0,0,0
        // construct EigenVector from Vector
        Eigen::VectorXd map_state_cov_vct = Eigen::VectorXd::Map(map_state_cov.data(), map_state_cov.size());
        Eigen::MatrixXd map_cov_mtx = map_state_cov_vct.asDiagonal();

        // 2n*3
        Eigen::MatrixXd bottom_left = Eigen::MatrixXd::Zero(2 * map_state.size(), 3);
        // 3*2n
        Eigen::MatrixXd top_right = Eigen::MatrixXd::Zero(3, 2 * map_state.size());

        // Construct left half of matrix
        Eigen::MatrixXd left_mtx(robot_cov_mtx.rows() + bottom_left.rows(), robot_cov_mtx.cols());
        left_mtx << robot_cov_mtx, bottom_left;
        // Construct right half of matrix
        Eigen::MatrixXd right_mtx(top_right.rows() + map_cov_mtx.rows(), top_right.cols());
        right_mtx << robot_cov_mtx, bottom_left;

        // Merge two matrixes to construct Covariance Matrix
        Eigen::MatrixXd mtx(left_mtx.rows(), left_mtx.cols() + right_mtx.cols());
        mtx << left_mtx, right_mtx;

        cov_mtx = mtx;
	}

	CovarianceMatrix::CovarianceMatrix(const Pose2D & robot_state_, const std::vector<Vector2D> & map_state_, \
                         			   const std::vector<double> & robot_state_cov_,\
			                           const std::vector<double> & map_state_cov_)
	{
		robot_state = robot_state_;  // init to 0,0,0
        robot_state_cov = robot_state_cov_;
        // construct EigenVector from Vector
        Eigen::VectorXd robot_state_cov_vct = Eigen::VectorXd::Map(robot_state_cov.data(), robot_state_cov.size());
        Eigen::MatrixXd robot_cov_mtx = robot_state_cov_vct.asDiagonal();

        map_state = map_state_;
        map_state_cov = map_state_cov_;
        // construct EigenVector from Vector
        Eigen::VectorXd map_state_cov_vct = Eigen::VectorXd::Map(map_state_cov.data(), map_state_cov.size());
        Eigen::MatrixXd map_cov_mtx = map_state_cov_vct.asDiagonal();

        // 2n*3
        Eigen::MatrixXd bottom_left = Eigen::MatrixXd::Zero(2 * map_state.size(), 3);
        // 3*2n
        Eigen::MatrixXd top_right = Eigen::MatrixXd::Zero(3, 2 * map_state.size());

        // Construct left half of matrix
        Eigen::MatrixXd left_mtx(robot_cov_mtx.rows() + bottom_left.rows(), robot_cov_mtx.cols());
        left_mtx << robot_cov_mtx, bottom_left;
        // Construct right half of matrix
        Eigen::MatrixXd right_mtx(top_right.rows() + map_cov_mtx.rows(), top_right.cols());
        right_mtx << robot_cov_mtx, bottom_left;

        // Merge two matrixes to construct Covariance Matrix
        Eigen::MatrixXd mtx(left_mtx.rows(), left_mtx.cols() + right_mtx.cols());
        mtx << left_mtx, right_mtx;

        cov_mtx = mtx;
	}

}