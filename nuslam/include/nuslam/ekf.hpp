#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
/// \file
/// \brief Library EKF EKF detection and classification.
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <nuslam/landmarks.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <numeric>
#include <functional>
#include <ros/ros.h>
#include <limits>  // set variable to max (inf)

namespace nuslam
{
    // Used to for prediction stage
    using rigid2d::Twist2D;

    // used for model update
    using rigid2d::Pose2D;

    // used for map update
    using rigid2d::Vector2D;

    // Struct to store Covariance Matrix for EKF
    struct CovarianceMatrix
    {
        // Holds Robot x,y,theta
        Pose2D robot_state;

        // Populates top left of covariance matrix for robot
        std::vector<double> robot_state_cov;

        // holds Landmarks x,y
        std::vector<Vector2D> map_state;

        // Populates bottom right of covariance matrix for landmarks
        std::vector<double> map_state_cov;

        // Covariance Matrix
        Eigen::MatrixXd cov_mtx;

        /// \brief constructor for covariance matrix with robot_state_cov init to zero
        /// and map_state_cov init to infinity but with no recorded landmarks
        CovarianceMatrix();

        /// \brief constructor for covariance matrix with robot_state_cov init to zero
        /// and map_state_cov init to infinity
        CovarianceMatrix(const Pose2D & robot_state_, const std::vector<Vector2D> & map_state_);


        /// \brief constructor for covariance matrix with robot_state_cov
        /// and map_state_cov init to user-specified input
        CovarianceMatrix(const Pose2D & robot_state_, const std::vector<Vector2D> & map_state_, \
                         const std::vector<double> & robot_state_cov_,\
                         const std::vector<double> & map_state_cov_);

    };

    // Struct to store Process Noise for ERK
    struct ProcessNoise
    {
    };

    // Struct to store Measurement Noise for ERK
    struct MeasurementNoise
    {
    };

    /// \brief handles model propagation for EKF SLAM
    class EKF
    {
    public:
        /// \brief the default constructor creates a EKF with 0 radius and 0,0 pose and default threshold
        EKF();

        /// \brief start with guess of robot state (0,0,0) with zero covariance for robot state, indicating
        /// full confidence in initial state, and infinite covariance for ladmarks state, indicating we
        /// know nothing about them.
        /// \param
        /// \returns
        void initialize();

        /// \brief forward-propagate the nonlinear motion model to get an estimate (prediction, and, using
        /// Taylor-Series expantion, get a linearized state transition model, which is used to propagate uncertainty.
        /// \param
        /// \returns
        void predict();

        /// \brief incorporate sequential landmark measurements to perform a correction of our predicted estimate, 
        /// then, update the EKF parameters for the next ieration. Also initializes new landmarks
        /// \param
        /// \returns
        void update();

    private:
    };
}

#endif
