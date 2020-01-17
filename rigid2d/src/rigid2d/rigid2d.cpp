#include "rigid2d/rigid2d.hpp"
#include <iostream>

// Vector2D
rigid2d::Vector2D::Vector2D()
{
    x = 0;
    y = 0;
    norm_x = 0;
    norm_y = 0;
    rigid2d::Vector2D::normalize();
}

rigid2d::Vector2D::Vector2D(double x_, double y_)
{
    x = x_;
    y = y_;
    rigid2d::Vector2D::normalize();
}

void rigid2d::Vector2D::normalize()
{
	if (x != 0)
    {
    	norm_x = x / sqrt(pow(x, 2) + pow(y, 2));
    } else {
    	norm_x = 0;
    }

    if (y != 0)
    {
    	norm_y = y / sqrt(pow(x, 2) + pow(y, 2));
    } else {
    	norm_y = 0;
    }
}

double rigid2d::length(const rigid2d::Vector2D & v)
{
	return sqrt(pow(v.x, 2) + pow(v.y, 2));
}

double rigid2d::distance(const rigid2d::Vector2D & v1, const rigid2d::Vector2D & v2)
{
	return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
}

double rigid2d::angle(const rigid2d::Vector2D & v)
{
	return atan(v.y / v.x);
}

rigid2d::Vector2D & rigid2d::Vector2D::operator+=(const rigid2d::Vector2D & rhs)
{
	x += rhs.x;
	y += rhs.y;
	rigid2d::Vector2D::normalize();
	
	return *this;
}

rigid2d::Vector2D rigid2d::operator+(rigid2d::Vector2D lhs, const rigid2d::Vector2D & rhs)
{
	// call operator+=() member function of lhs object (just above)
	lhs+=rhs;
	return lhs;
}

rigid2d::Vector2D & rigid2d::Vector2D::operator-=(const rigid2d::Vector2D & rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	rigid2d::Vector2D::normalize();
	
	return *this;
}

rigid2d::Vector2D rigid2d::operator-(rigid2d::Vector2D lhs, const rigid2d::Vector2D & rhs)
{
	// call operator-=() member function of lhs object (just above)
	lhs-=rhs;
	return lhs;
}

rigid2d::Vector2D & rigid2d::Vector2D::operator*=(const double & scalar)
{
	x *= scalar;
	y *= scalar;
	rigid2d::Vector2D::normalize();
	
	return *this;
}

rigid2d::Vector2D rigid2d::operator*(rigid2d::Vector2D v, const double & scalar)
{
	// alternate definition (left multiply)
	// call operator*=() member function of lhs object (just above)
	v*=scalar;
	return v;
}

rigid2d::Vector2D rigid2d::operator*(const double & scalar, rigid2d::Vector2D v)
{
	// alternate definition (right multiply)
	// call operator*=() member function of lhs object (just above)
	v*=scalar;
	return v;
}


std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v)
{
	os << "[" << v.x << ", " << v.y << "]" << "\n";

	return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v)
{
	std::cout << "Enter x:" << std::endl;
	is >> v.x;

	std::cout << "Enter y:" << std::endl;
	is >> v.y;

	// compute norm
	v.normalize();

	return is;
}

// Transform2DS
rigid2d::Transform2DS::Transform2DS()
{
    theta = 0;
    x = 0;
    y = 0;
}

rigid2d::Transform2DS::Transform2DS(double theta_, double x_, double y_)
{
	theta = theta_;
    x = x_;
    y = y_;
}

// Screw2D
rigid2d::Screw2D::Screw2D()
{
    w_z = 0;
    v_x = 0;
    v_y = 0;
}

rigid2d::Screw2D::Screw2D(double w_z_, double v_x_, double v_y_)
{
	w_z = w_z_;
    v_x = v_x_;
    v_y = v_y_;
}

// Transform2D
rigid2d::Transform2D::Transform2D()
{
	// Below are values of class params for ID Transform.
	x = 0;
	y = 0;
	theta = 0;
	ctheta = cos(theta);
	stheta = sin(theta);
}

rigid2d::Transform2D::Transform2D(const Vector2D & trans)
{	
	// Pure Translational Transform
	x = trans.x;
	y = trans.y;
	theta = 0;
	ctheta = cos(theta);
	stheta = sin(theta);
}

rigid2d::Transform2D::Transform2D(double radians)
{
	// Pure Rotational Transform
	x = 0;
	y = 0;
	theta = radians;
	ctheta = cos(theta);
	stheta = sin(theta);
}

rigid2d::Transform2D::Transform2D(const Vector2D & trans, double radians)
{
	// Mixed Transform
	x = trans.x;
	y = trans.y;
	theta = radians;
	ctheta = cos(theta);
	stheta = sin(theta);
}

rigid2d::Transform2D::Transform2D(double theta, double ctheta, double stheta, double x, double y)
{
	this->x = x;
	this->y = y;
	this->theta = theta;
	this->ctheta = ctheta;
	this->stheta = stheta;
}

rigid2d::Vector2D rigid2d::Transform2D::operator()(rigid2d::Vector2D v) const
{
	// Transform vector v into vector vp

	Vector2D vp;

	vp.x = v.x * ctheta - v.y * stheta + x;
	vp.y = v.x * stheta + v.y * ctheta + y;

	// Check if anything is almost zero
	if (almost_equal(vp.x, 0))
	{
		vp.x = 0;
	}

	if (almost_equal(vp.y, 0))
	{
		vp.y = 0;
	}

	return vp;
}

rigid2d::Transform2D rigid2d::Transform2D::inv() const
{
	// Create temp of 2d tranform (clone it essentially)
	// creating pointer to temp2d class with 'new'
	rigid2d::Transform2D temp2d = rigid2d::Transform2D(theta, ctheta, stheta, x, y);

	// for transpose, flip sintheta (pg 90 modern robotics)
	temp2d.stheta = -temp2d.stheta;
	temp2d.theta = atan2(temp2d.stheta, temp2d.ctheta);
	temp2d.ctheta = cos(temp2d.theta);

	// this performs p' = -R.T*p (pg90 modern robotics)
	temp2d.x = -(temp2d.ctheta * x - temp2d.stheta * y);
	temp2d.y = -(temp2d.stheta * x + temp2d.ctheta * y);

	// Check if anything is almost zero
	if (almost_equal(temp2d.x, 0))
	{
		temp2d.x = 0;
	}

	if (almost_equal(temp2d.y, 0))
	{
		temp2d.y = 0;
	}

	if (almost_equal(temp2d.theta, 0))
	{
		temp2d.theta = 0;
		temp2d.stheta = sin(temp2d.theta);
		temp2d.ctheta = cos(temp2d.theta);
	}

	return temp2d;
}

rigid2d::Transform2D rigid2d::Transform2D::integrateTwist(const rigid2d::Twist2D & tw) const
{
	// init theta dot
	// note theta = theta dot because we integrate for one timestep
	double th = 0;

	// Initialize Screw
	rigid2d::Screw2D S = rigid2d::Screw2D();

	// Check if w = 0 or not
	if (!almost_equal(tw.w_z, 0))
	{
		// Twist is mixed angular and linear
		S.w_z = tw.w_z / fabs(tw.w_z);
		S.v_x = tw.v_x / fabs(tw.w_z);
		S.v_y = tw.v_y / fabs(tw.w_z);
		// init theta dot
		// note theta = theta dot because we integrate for one timestep
		th = fabs(tw.w_z);
	} else {
		// Twist is purely linear
		S.w_z = 0;
		S.v_x = tw.v_x / sqrt(pow(tw.v_x, 2) + pow(tw.v_y, 2));
		S.v_y = tw.v_y / sqrt(pow(tw.v_x, 2) + pow(tw.v_y, 2));
		// init theta dot
		// note theta = theta dot because we integrate for one timestep
		th = sqrt(pow(tw.v_x, 2) + pow(tw.v_y, 2));
	}
	// Calculate new transform
	rigid2d::Transform2D T_twist = rigid2d::Transform2D(\
		atan2(sin(th) * S.w_z, 1 - (1 - cos(th)) * pow(S.w_z, 2))\
		, cos(atan2(sin(th) * S.w_z, 1 - (1 - cos(th)) * pow(S.w_z, 2)))\
		, sin(atan2(sin(th) * S.w_z, 1 - (1 - cos(th)) * pow(S.w_z, 2)))\
		, S.v_x * (th - (th - sin(th)) * pow(S.w_z, 2))\
		- S.v_y * (1 - cos(th)) * S.w_z\
		, S.v_x * (1 - cos(th)) * S.w_z\
		+ S.v_y * (th - (th - sin(th)) * pow(S.w_z, 2)));

	rigid2d::Transform2D T_original(theta, ctheta, stheta, x, y);
	rigid2d::Transform2D T = T_original * T_twist;
	return T;
}

rigid2d::Transform2DS rigid2d::Transform2D::displacement() const
{
	rigid2d::Transform2DS T = rigid2d::Transform2DS(theta, x, y);
	return T;
}

rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const rigid2d::Transform2D & rhs)
{
	x = ctheta * rhs.x - stheta * rhs.y + x;
	y = stheta * rhs.x + ctheta * rhs.y + y;
	// theta = acos(ctheta * rhs.ctheta - stheta * rhs.stheta);
	theta += rhs.theta;
	ctheta = cos(theta);
	stheta = sin(theta);

	// Check if anything is almost zero
	if (almost_equal(x, 0))
	{
		x = 0;
	}

	if (almost_equal(y, 0))
	{
		y = 0;
	}

	if (almost_equal(theta, 0))
	{
		theta = 0;
		stheta = sin(theta);
		ctheta = cos(theta);
	}

	// `this` is a pointer to our object, which we dereference for the object itself
	return *this;
}

rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D & rhs)
{
	// call operator*=() member function of lhs object (just above)
	lhs*=rhs;
	return lhs;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf)
{
	/// dtheta (degrees): 90 dx: 3 dy: 5
	os << "dtheta (degrees): " << rigid2d::rad2deg(tf.theta) << "\t" << "dx: " << tf.x << "\t"\
	<< "dy: " << tf.y << "\n";

	return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf)
{
	// using friend function so that only this input fcn can overwrite
	// private params of Tranform2D
	std::cout << "Enter theta in degrees:" << std::endl;
	double deg;
	is >> deg;

	std::cout << "Enter x:" << std::endl;
	is >> tf.x;

	std::cout << "Enter y:" << std::endl;
	is >> tf.y;

	tf.theta = rigid2d::deg2rad(deg);
	tf.stheta = sin(tf.theta);
	tf.ctheta = cos(tf.theta);

	return is;
}

// Twist2D
rigid2d::Twist2D::Twist2D()
{
	w_z = 0;
	v_x = 0;
	v_y = 0;
}

rigid2d::Twist2D::Twist2D(double w_z_, double v_x_, double v_y_)
{
	w_z = w_z_;
	v_x = v_x_;
	v_y = v_y_;
}

rigid2d::Twist2D rigid2d::Twist2D::convert(const rigid2d::Transform2D & tf) const
{
	// create Twist2D
	rigid2d::Twist2D tw_s = rigid2d::Twist2D(w_z\
		, (v_x * tf.ctheta - v_y * tf.stheta + w_z * tf.y)\
		, (v_y * tf.ctheta + v_x * tf.stheta - w_z * tf.x));
	// notation: tw_b = twist object
	// Vs = [AdTsb]Vb

	// Check if anything is almost zero
	if (almost_equal(tw_s.w_z, 0))
	{
		tw_s.w_z = 0;
	}

	if (almost_equal(tw_s.v_x, 0))
	{
		tw_s.v_x = 0;
	}

	if (almost_equal(tw_s.v_y, 0))
	{
		tw_s.v_y = 0;
	}

	return tw_s;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Twist2D & tw)
{
	os << "w_z (rad/s): " << tw.w_z << "\t" << "v_x (m/s): " << tw.v_x << "\t"\
	<< "v_y (m/s): " << tw.v_y << "\n";

	return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Twist2D & tw)
{
	// using friend function so that only this input fcn can overwrite
	// private params of Twist2D
	std::cout << "Enter w_z in rad/s:" << std::endl;
	is >> tw.w_z;

	std::cout << "Enter v_x in m/s:" << std::endl;
	is >> tw.v_x;

	std::cout << "Enter v_y in m/s:" << std::endl;
	is >> tw.v_y;

	return is;
}



