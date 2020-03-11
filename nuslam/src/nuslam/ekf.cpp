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
        std::vector<double> robot_state_cov{0.0, 0.0, 0.0}; // init to 0,0,0
        // 3*3
        // construct EigenVector from Vector
        Eigen::VectorXd robot_state_cov_vct = Eigen::VectorXd::Map(robot_state_cov.data(), robot_state_cov.size());
        Eigen::MatrixXd robot_cov_mtx = robot_state_cov_vct.asDiagonal();

        cov_mtx = robot_cov_mtx;
	}

	CovarianceMatrix::CovarianceMatrix(const std::vector<Point> & map_state_)
	{
        std::vector<double> robot_state_cov{0.0, 0.0, 0.0}; // init to 0,0,0
        // 3*3
        // construct EigenVector from Vector
        Eigen::VectorXd robot_state_cov_vct = Eigen::VectorXd::Map(robot_state_cov.data(), robot_state_cov.size());
        Eigen::MatrixXd robot_cov_mtx = robot_state_cov_vct.asDiagonal();

        map_state = map_state_;
        // 2n*2n diag matrix since map containts x1,y1, ... xn,yn
        // std::vector<double> map_state_cov(2 * map_state.size(), std::numeric_limits<double>::infinity()); // init to 0,0,0
        std::vector<double> map_state_cov(2 * map_state.size(), 1e12); // init to 0,0,0
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
        right_mtx << top_right, map_cov_mtx;

        // Merge two matrixes to construct Covariance Matrix
        Eigen::MatrixXd mtx(left_mtx.rows(), left_mtx.cols() + right_mtx.cols());
        mtx << left_mtx, right_mtx;

        cov_mtx = mtx;
	}

	CovarianceMatrix::CovarianceMatrix(const std::vector<Point> & map_state_, \
                         			   const std::vector<double> & robot_state_cov_,\
			                           const std::vector<double> & map_state_cov_)
	{
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
        right_mtx << top_right, map_cov_mtx;

        // Merge two matrixes to construct Covariance Matrix
        Eigen::MatrixXd mtx(left_mtx.rows(), left_mtx.cols() + right_mtx.cols());
        mtx << left_mtx, right_mtx;

        cov_mtx = mtx;
	}

	// Process Noise
	ProcessNoise::ProcessNoise()
	{
		map_size = 0;

		xyt_noise = Pose2D();

		Eigen::VectorXd xyt_vct(3);

		xyt_vct << xyt_noise.theta, xyt_noise.x, xyt_noise.y;

		// assume x,y,theta noise decoupled from each other, 3*3
		Eigen::MatrixXd q = xyt_vct.asDiagonal();

		Q = q;

	}

	ProcessNoise::ProcessNoise(const Pose2D & xyt_noise_var, const unsigned long int & map_size_)
	{
		map_size = map_size_;
		// std::vector<double> noise_vect = get_3d_noise(xyt_noise_mean, xyt_noise_var, cov_mtx);
		xyt_noise = Pose2D(xyt_noise_var.x, xyt_noise_var.y, xyt_noise_var.theta);

		Eigen::VectorXd xyt_vct(3);

		xyt_vct << xyt_noise.theta, xyt_noise.x, xyt_noise.y;

		// assume x,y,theta noise decoupled from each other, 3*3
		Eigen::MatrixXd q = xyt_vct.asDiagonal();

		// 2n*3
        Eigen::MatrixXd bottom_left = Eigen::MatrixXd::Zero(2 * map_size, 3);
        // 3*2n
        Eigen::MatrixXd top_right = Eigen::MatrixXd::Zero(3, 2 * map_size);
        // 2*2n
        Eigen::MatrixXd bottom_right = Eigen::MatrixXd::Zero(2 * map_size, 2 * map_size);

        // Construct left half of matrix
        Eigen::MatrixXd left_mtx(q.rows() + bottom_left.rows(), q.cols());
        left_mtx << q, bottom_left;
        // Construct right half of matrix
        Eigen::MatrixXd right_mtx(top_right.rows() + bottom_right.rows(), top_right.cols());
        right_mtx << top_right, bottom_right;

        // Merge two matrixes to construct Covariance Matrix
        Eigen::MatrixXd mtx(left_mtx.rows(), left_mtx.cols() + right_mtx.cols());
        mtx << left_mtx, right_mtx;

        Q = mtx;

	}

	// Measurement Noise
	MeasurementNoise::MeasurementNoise()
	{
		rb_noise_var = RangeBear();

		Eigen::VectorXd rb_vct(2);

		rb_vct << sampleNormalDistribution(rb_noise_var.range), sampleNormalDistribution(rb_noise_var.bearing);

		R = rb_vct.asDiagonal();
	}

	MeasurementNoise::MeasurementNoise(const RangeBear & rb_noise_var_)
	{
		rb_noise_var = rb_noise_var_;

		Eigen::VectorXd rb_vct(2);

		rb_vct << sampleNormalDistribution(rb_noise_var.range), sampleNormalDistribution(rb_noise_var.bearing);

		R = rb_vct.asDiagonal();
	}


	// Random Sampling Functions
	std::mt19937 & get_random()
    {
        // static variables inside a function are created once and persist for the remainder of the program
        static std::random_device rd{}; 
        static std::mt19937 mt{rd()};
        // we return a reference to the pseudo-random number genrator object. This is always the
        // same object every time get_random is called
        return mt;
    }

    double sampleNormalDistribution(double var)
    {
		std::normal_distribution<> d(0, var);
		return d(get_random());
    }

    std::vector<double> get_3d_noise(const Pose2D & xyt_noise_mean, const ProcessNoise & proc_noise)
    {
    	Eigen::MatrixXd Sigma = proc_noise.Q;

    	// Cholesky Decomposition
    	Eigen::MatrixXd L(Sigma.llt().matrixL());

    	// Sample Random Noise
    	// x
    	const double x_normal = sampleNormalDistribution();
    	// y
    	const double y_normal = sampleNormalDistribution();
    	// theta
    	const double theta_normal = sampleNormalDistribution();

    	// Get noise from 3D normal distribution and store in vector for each individual dimension
    	// mean noise is generally zero
    	const auto theta_noise = xyt_noise_mean.theta + L(0,0) * theta_normal + L(0,1) * x_normal + L(0,2) * y_normal;
    	const auto x_noise = xyt_noise_mean.x + L(1,0) * theta_normal + L(1,1) * x_normal + L(1,2) * y_normal;
		const auto y_noise = xyt_noise_mean.y + L(2,0) * theta_normal + L(2,1) * x_normal + L(2,2) * y_normal;

		std::vector<double> noise_vect;

		noise_vect.push_back(theta_noise);
		noise_vect.push_back(x_noise);
		noise_vect.push_back(y_noise);

		return noise_vect;
    }

    //EKF
    EKF::EKF()
    {
    	State = Eigen::VectorXd::Zero(3);
    	max_range = 3.5;
    	robot_state = Pose2D();
    	proc_noise = ProcessNoise();
    	msr_noise = MeasurementNoise();
    	cov_mtx = CovarianceMatrix();
    	sigma_bar = cov_mtx;
    }

    EKF::EKF(const Pose2D & robot_state_, const std::vector<Point> & map_state_, const Pose2D & xyt_noise_var, const RangeBear & rb_noise_var_, const double & max_range_)
    {
    	State = Eigen::VectorXd::Zero(3 + 2 * map_state_.size());
    	max_range = max_range_;
    	robot_state = robot_state_;
    	map_state = map_state_;
    	cov_mtx = CovarianceMatrix(map_state_);
    	sigma_bar = cov_mtx;
    	proc_noise = ProcessNoise(xyt_noise_var, map_state_.size());
    	msr_noise = MeasurementNoise(rb_noise_var_);
    }

    void EKF::predict(const Twist2D & twist, const Pose2D & xyt_noise_mean)
    {
    	// Angle Wrap Robot Theta
    	robot_state.theta = rigid2d::normalize_angle(robot_state.theta);
    	// First, update the estimate using the forward model
    	std::vector<double> noise_vect = get_3d_noise(xyt_noise_mean, proc_noise);

    	Pose2D belief;

    	if (rigid2d::almost_equal(twist.w_z, 0.0))
    	// If dtheta = 0
    	{
    		belief = Pose2D(robot_state.x + (twist.v_x * cos(robot_state.theta)) + noise_vect.at(1),\
						    robot_state.y + (twist.v_x * sin(robot_state.theta)) + noise_vect.at(2),\
    						robot_state.theta + noise_vect.at(0));
    	} else {
		// If dtheta != 0
    		belief = Pose2D(robot_state.x + ((-twist.v_x / twist.w_z) * sin(robot_state.theta) + (twist.v_x / twist.w_z) * sin(robot_state.theta + twist.w_z)) + noise_vect.at(1),\
						    robot_state.y + ((twist.v_x / twist.w_z) * cos(robot_state.theta) + (-twist.v_x / twist.w_z) * cos(robot_state.theta + twist.w_z)) + noise_vect.at(2),\
    						robot_state.theta + twist.w_z + noise_vect.at(0));
    	}
    	// Angle Wrap Robot Theta
    	belief.theta = rigid2d::normalize_angle(belief.theta);

    	// Next, we propagate the uncertainty using the linearized state transition model
    	// (3+2n)*(3+2n)
    	Eigen::MatrixXd g = Eigen::MatrixXd::Zero(3 + (2 * map_state.size()), 3 + (2 * map_state.size()));
    	if (rigid2d::almost_equal(twist.w_z, 0.0))
    	// If dtheta = 0
    	{
    		// Now replace non-zero entries
    		// using theta,x,y
    		g(1, 0) = -twist.v_x * sin(robot_state.theta);
    		g(2, 0) = twist.v_x * cos(robot_state.theta);
    	} else {
		// If dtheta != 0
    		// Now replace non-zero entries
    		// using theta,x,y
    		g(1, 0) = (-twist.v_x / twist.w_z) * cos(robot_state.theta) + (twist.v_x / twist.w_z) * cos(robot_state.theta + twist.w_z);
    		g(2, 0) = (-twist.v_x / twist.w_z) * sin(robot_state.theta) + (twist.v_x / twist.w_z) * sin(robot_state.theta + twist.w_z);
    	}

    	Eigen::MatrixXd G = Eigen::MatrixXd::Identity(3 + (2 * map_state.size()), 3 + (2 * map_state.size())) + g;

    	sigma_bar.cov_mtx = G * cov_mtx.cov_mtx * G.transpose() + proc_noise.Q;

    	// store belief as new robot state for update operation
    	robot_state = belief;

    	// Update State Matrix
    	State(0) = robot_state.theta;
    	State(1) = robot_state.x;
    	State(2) = robot_state.y;
    }

    void EKF::msr_update(std::vector<Point> & measurements_)
    {
    	for (auto iter = measurements_.begin(); iter != measurements_.end(); iter++)
    	{
    		// std::cout << "cov_mtx.cov_mtx: \n" << cov_mtx.cov_mtx << std::endl;
    		// Current Landmark Index
    		auto j = std::distance(measurements_.begin(), iter);
    		// Angle Wrap Bearing
    		State(0) = rigid2d::normalize_angle(State(0));
	    	iter->range_bear.bearing = rigid2d::normalize_angle(iter->range_bear.bearing);
    		// convert landmark cartesian position from robot-relative to world relative
    		iter->pose.x += State(1);
    		iter->pose.y += State(2);

    		// If a landmark has range > tolerance, skip
    		// std::cout << "Landmark #" << j << "\tRange: " << iter->range_bear.range << std::endl;
    		if (!(iter->range_bear.range > max_range))
    		// skip landmark if outside of range
    		{

    		// If a landmark has not been initialized internally, initialize its position
    		if (!map_state.at(j).init)
    		{
    		map_state.at(j).pose.x = iter->pose.x;
    		map_state.at(j).pose.y = iter->pose.y;
    		map_state.at(j).init = true;

    		// Update State
    		State(3 + 2*j) = iter->pose.x;
    		State(4 + 2*j) = iter->pose.y;
    		}

    		// First, get theoretical expected measurement based on belief in [r,b] format
	    	// Pose difference between robot and landmark
	    	Vector2D cartesian_measurement = Vector2D(State(3 + 2*j) - State(1), State(4 + 2*j) - State(2));
	    	RangeBear polar_measurement = cartesianToPolar(cartesian_measurement);
	    	// Angle Wrap Bearing
	    	polar_measurement.bearing = rigid2d::normalize_angle(polar_measurement.bearing);
	    	// Subtract robot heading from bearing
	    	polar_measurement.bearing -= State(0);
	    	// Angle Wrap Bearing
	    	polar_measurement.bearing = rigid2d::normalize_angle(polar_measurement.bearing);

	    	Eigen::VectorXd z_hat(2);
    		z_hat << polar_measurement.range, polar_measurement.bearing;
    		// std::cout << "z_hat: \n" << z_hat << std::endl;

    		//  Add noise to actual range, bearing measurement
	    	// Note, the nuslam::Point struct contains cartesian and polar measurements, so we just use iter
	    	// Add Noise
    		Eigen::VectorXd z(2);
    		z << iter->range_bear.range + msr_noise.R(0, 0), iter->range_bear.bearing + msr_noise.R(1, 1);
    		z(1) = rigid2d::normalize_angle(z(1));
    		// std::cout << "z: " << z << std::endl;

	    	// http://andrewjkramer.net/intro-to-the-ekf-step-3/
	    	// Note first 3x3 is IX matrix and there is a 2x2 ID matrix on the bottom two rows starting at column 2j + 3 with j from 0 to n
	    	// j is the current examined landmark
	    	// Eigen::MatrixXd Fxj = Eigen::MatrixXd::Zero(5, 3 + (2 * map_state.size()));
	    	// // top left 3x3
	    	// Fxj(0, 0) = 1;
	    	// Fxj(1, 1) = 1;
	    	// Fxj(2, 2) = 1;
	    	// // bottom 2 rows 2x2
	    	// Fxj(3, 2 * j + 3) = 1;
	    	// Fxj(4, 2 * j + 4) = 1;
	    	// std::cout << "Fxj: \n" << Fxj << std::endl;

	    	// Compute the measurement Jacobian
	    	double x_diff = State(3 + 2*j) - State(1);
	    	double y_diff = State(4 + 2*j) - State(2);
	    	double squared_diff = pow(x_diff, 2) + pow(y_diff, 2);
	    	// Eigen::MatrixXd h(2, 5);
	    	// h << 0.0, (-x_diff / sqrt(squared_diff)), (-y_diff / sqrt(squared_diff)), (x_diff / sqrt(squared_diff)), (y_diff / sqrt(squared_diff)),
	    	// 	 -1.0, (y_diff / sqrt(squared_diff)), (-x_diff / sqrt(squared_diff)), (-y_diff / sqrt(squared_diff)), (x_diff / sqrt(squared_diff));
	    	// std::cout << "h: \n" << h << std::endl;
	    	// 2*(2n+3)
			Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 3 + 2 * map_state.size());
			// H constructed from four Matrices: https://nu-msr.github.io/navigation_site/slam.pdf
			// NOTE: j starts at 1 in slam.pdf
			Eigen::MatrixXd h_left(2, 3);
			h_left << 0.0, (-x_diff / sqrt(squared_diff)), (-y_diff / sqrt(squared_diff)), -1.0, (y_diff / sqrt(squared_diff)), (-x_diff / sqrt(squared_diff));

			Eigen::MatrixXd h_mid_left = Eigen::MatrixXd::Zero(2, 2*j);

			Eigen::MatrixXd h_mid_right(2, 2);
			h_mid_right << (x_diff / sqrt(squared_diff)), (y_diff / sqrt(squared_diff)), (-y_diff / sqrt(squared_diff)), (x_diff / sqrt(squared_diff));

			Eigen::MatrixXd h_right = Eigen::MatrixXd::Zero(2, 2 * map_state.size() - 2*(j + 1));

			// Construct H - Measurement Jacobian
			Eigen::MatrixXd h_mtx(h_left.rows(), h_left.cols() + h_mid_left.cols() + h_mid_right.cols() + h_right.cols());
			h_mtx << h_left, h_mid_left, h_mid_right, h_right;
			// std::cout << "h_left: \n\n" << h_left << std::endl;
			// std::cout << "h_mid_left: \n\n" << h_mid_left << std::endl;
			// std::cout << "h_mid_right: \n\n" << h_mid_right << std::endl;
			// std::cout << "h_right: \n\n" << h_right << std::endl;
			// std::cout << "size: " << map_state.size() << std::endl;
			H = h_mtx;

	    	// Eigen::MatrixXd H = h * Fxj;
	    	// std::cout << "H: \n\n" << H << std::endl;

	    	// std::cout << "H SIZE: (" << H.rows() << "," << H.cols() << ")" << std::endl;

	    	// Compute the Kalman gain from the linearized measurement model
	    	// (2n+3)*2
			Eigen::MatrixXd K = sigma_bar.cov_mtx * H.transpose() * (H * sigma_bar.cov_mtx * H.transpose() + msr_noise.R).inverse(); // NOTE: FIND MORE EFFICIENT INV
			// std::cout << "H Transpose: \n" << H.transpose() << std::endl;
			// std::cout << "cov mtx * H.T: \n" << cov_mtx.cov_mtx * H.transpose() << std::endl;
			// std::cout << "Matrix to Invert: \n" << (H * cov_mtx.cov_mtx * H.transpose() + msr_noise.R) << std::endl;
			// std::cout << "K: \n" << K << std::endl;

	    	// Compute the posterior state update
    		Eigen::VectorXd z_diff(2);
    		z_diff = z - z_hat;
    		// Angle Wrap Bearing
	    	z_diff(1) = rigid2d::normalize_angle(z_diff(1));
    		// std::cout << "z_diff: \n" << z_diff << std::endl;

    		// (2n+3)*1
    		Eigen::VectorXd K_update = K * z_diff;
    		// std::cout << "K_update: \n" << K_update << std::endl;
	    	State += K_update;
	    	State(0) = rigid2d::normalize_angle(State(0));
	    	// Update returnable values (robot and map state)
	    	robot_state.theta = State(0);
    		robot_state.theta = rigid2d::normalize_angle(robot_state.theta);
    		robot_state.x = State(1);
    		robot_state.y = State(2);
    		// Map data starts at index 3
    		// Perform update for full map state
    		for (long unsigned int i = 0; i < map_state.size(); i++)
			{
    		map_state.at(i).pose.x = State(3 + 2*i);
    		map_state.at(i).pose.y = State(4 + 2*i);
	    	}
	    	// Compute the posterior covariance
	    	sigma_bar.cov_mtx = (Eigen::MatrixXd::Identity(3 + (2 * map_state.size()), 3 + (2 * map_state.size())) - K * H) * sigma_bar.cov_mtx;
	    	// std::cout << "cov_mtx.cov_mtx: \n" << cov_mtx.cov_mtx << std::endl;
    		}

    		cov_mtx.cov_mtx = sigma_bar.cov_mtx;
    	}
    }

    Pose2D EKF::return_pose()
    {
    	return robot_state;
    }

    std::vector<Point> EKF::return_map()
    {
    	return map_state;
    }

    void EKF::reset_pose(const Pose2D & pose)
    {
    	robot_state = pose;
    }
}