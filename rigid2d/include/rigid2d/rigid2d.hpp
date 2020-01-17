#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include<iostream>

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    // constexpr are all define in .hpp
    // constexpr allows fcn to be run at compile time and interface with 
    // static_assert tests.
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (fabs(d1 - d2) < epsilon)
        {
            return true;
        } else {
            return false;
    }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
        return deg * PI / 180.0;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad * 180.0 / PI;
    }

    /// \brief wraps an angle between +- PI
    /// \param rad - angle in radians
    /// \returns the wrapped angle in radians
    constexpr double normalize_angle(double rad)
    {
        double lim  = std::floor((rad + PI) / (2 * PI));
        rad = rad + PI - lim * 2 * PI;
        if (rad < 0)
        {
            rad += 2 * PI;
        }
        rad -= PI;
        return rad;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    static_assert(almost_equal(0.0001, 0.0005, 0.001), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
    static_assert(almost_equal(rad2deg(deg2rad(2.1)), 2.1), "rad2deg failed");

    static_assert(almost_equal(normalize_angle(deg2rad(270)), normalize_angle(deg2rad(-90))), "normalize_angle failed");
    static_assert(almost_equal(normalize_angle(deg2rad(360)), normalize_angle(deg2rad(0))), "normalize_angle failed");
    static_assert(almost_equal(normalize_angle(deg2rad(370)), normalize_angle(deg2rad(10))), "normalize_angle failed");
    static_assert(almost_equal(normalize_angle(deg2rad(350)), normalize_angle(deg2rad(-10))), "normalize_angle failed");
    static_assert(almost_equal(normalize_angle(deg2rad(150)), normalize_angle(deg2rad(150))), "normalize_angle failed");
    static_assert(almost_equal(normalize_angle(deg2rad(-150)), normalize_angle(deg2rad(-150))), "normalize_angle failed");


    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x;
        double y;
        double norm_x;
        double norm_y;

        // \brief constructor for Vector2D with no inputs, creates a zero vector
        Vector2D();

        // \brief constructor for Vector2D with inputs
        Vector2D(double x_, double y_);

        // \brief fcn prototype to compute the norm of Vector2D
        void normalize();

        /// \brief perform vector addition
        /// \param rhs - the vector to add
        /// \returns a reference to the newly transformed operator
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief perform vector subtraction
        /// \param rhs - the vector to subtract
        /// \returns a reference to the newly transformed operator
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief perform scalar multiplication on a vector
        /// \param rhs - the vector to add
        /// \returns a reference to the newly transformed operator
        Vector2D & operator*=(const double & scalar);
    };

    /// \brief compute the length of a Vector2D
    /// \param v - the Vector2D whose length is computed
    /// \returns a length (double)
    // add const to end of member function if it doesn't change data members
    double length(const Vector2D & v);

    /// \brief compute the distance between two Vector2Ds
    /// \param v1 - the first Vector2D
    /// \param v2 - the second Vector2D
    /// \returns a distance (double)
    double distance(const Vector2D & v1, const Vector2D & v2);

    /// \brief compute the angle of a Vector2D
    /// \param v - the Vector2D whose angle is computed
    /// \returns a angle (double)
    double angle(const Vector2D & v);

    /// \brief perform vector addition
    /// \param lhs - the vector to be added to
    /// \param rhs - the vector to add (const)
    /// \returns a reference to the newly transformed operator
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief perform vector subtraction
    /// \param lhs - the vector to be subtracted from
    /// \param rhs - the vector to subtract (const)
    /// \returns a reference to the newly transformed operator
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief perform scalar multiplication on a vector from LHS
    /// \param v - the vector
    /// \param scalar - the scalar
    /// \return the scaled Vector2D
    /// HINT: This function can be implemented in terms of *=
    Vector2D operator*(Vector2D v, const double & scalar);

    /// \brief perform scalar multiplication on a vector from RHS
    /// \param v - the vector
    /// \param scalar - the scalar
    /// \return the scaled Vector2D
    /// HINT: This function can be implemented in terms of *=
    Vector2D operator*(const double & scalar, Vector2D v);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief Struct version of Transform2D to return private params
    struct Transform2DS
    {
        double theta, x, y; // angle, sin, cos, x, and y

        // \brief constructor for Transform2DS with no inputs, creates a zero vector
        Transform2DS();

        // \brief constructor for Transform2DS with inputs
        Transform2DS(double theta_, double x_, double y_);
    };

    /// \brief Screw Axis
    struct Screw2D
    {
        double w_z, v_x, v_y;

        // \brief constructor for Screw2D with no inputs, creates a zero vector
        Screw2D();

        // \brief constructor for Screw2D with inputs
        Screw2D(double w_z_, double v_x_, double v_y_);
    };

    // declare Twist2D here so Transform2D can see it
    class Twist2D;
    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    /// \brief declare Twist2D as friend so it can access Transform2D's private params
    friend class Twist2D;
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        // safeguard to ensure fcn is overloaded correctly
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation
        Transform2D inv() const;

        /// \brief compute transformation corresponding to a rigid body
        /// following a constant twist for one time unit
        /// \param tw - Twist2D which the transform follows
        /// \return new transformation of a rigid body following a twist
        Transform2D integrateTwist(const Twist2D & tw) const;

        /// \brief return theta, x, y of Transform
        /// \return Transform2DS struct with theta, x, y values. 
        rigid2d::Transform2DS displacement() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description.
        /// friend tag allows non-member functions to access private params.
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

        /// \brief \see operator>>(...) (declared outside this class)
        /// for a description.
        /// friend tag allows non-member functions to access private params.
        friend std::istream & operator>>(std::istream & is, Transform2D & tf);
    private:
        /// directly initialize, useful params for forming the inverse
        Transform2D(double theta, double ctheta, double stheta, double x, double y);
        double theta, ctheta, stheta, x, y; // angle, sin, cos, x, and y
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function can be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief a two-dimensional twist
    class Twist2D
    {
    /// \brief declare Transform2D as friend so it can access Twist2D's private params
    friend class Transform2D;
    public:
        /// \brief Create a zero-Twist
        Twist2D();

        /// \brief Create a non-zero Twist
        Twist2D(double w_z_, double v_x_, double v_y_);

        /// \brief convert the twist using an adjoint
        /// \param tf - the frame to which the twist is converted.
        /// \return the converted twist. 
        Twist2D convert(const Transform2D & tf) const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description.
        /// friend tag allows non-member functions to access private params.
        friend std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

        /// \brief \see operator>>(...) (declared outside this class)
        /// for a description.
        /// friend tag allows non-member functions to access private params.
        friend std::istream & operator>>(std::istream & is, Twist2D & tw);

    private:
        double v_x, v_y, w_z; // linear and angular components
    };

    /// \brief should print a human readable version of the twist:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

    /// \brief Read a twist from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (v_x, v_y, w_z) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Twist2D & tw);
}

#endif
