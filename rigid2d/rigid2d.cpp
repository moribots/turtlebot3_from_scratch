#include "rigid2d.hpp"
#include <iostream>

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v)
{
	os << "[" << v.x << " " << v.y << "]" << "\n";

	return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v)
{
	std::cout << "Enter x component of Vector" << std::endl;
	is >> v.x;

	std::cout << "Enter y component of Vector" << std::endl;
	is >> v.y;

	return is;
}

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

rigid2d::Vector2D rigid2d::Transform2D::operator()(rigid2d::Vector2D v) const
{
	// Transform vector v into vector vp

	Vector2D vp;

	vp.x = v.x * ctheta - v.y * stheta;
	vp.y = v.x * stheta + v.y * ctheta;

	return vp;
}

rigid2d::Transform2D rigid2d::Transform2D::inv() const
{
	// Create temp of 2d tranform (clone it essentially)
	rigid2d::Transform2D temp2d(theta, ctheta, stheta, x, y);

	// for transpose, flip sintheta (pg 90 modern robotics)
	temp2d.stheta = -temp2d.stheta;

	// create new temp vector v
	rigid2d::Vector2D v;
	v.x = -temp2d.x;
	v.y = -temp2d.y;
	// this performs p' = -R.T*p (pg90 modern robotics)
	rigid2d::Vector2D vp = temp2d.operator()(v);

	temp2d.x = vp.x;
	temp2d.y = vp.y;

	return temp2d;
}

rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const rigid2d::Transform2D & rhs)
{
	x = ctheta * rhs.x - stheta * rhs.y + x;
	y = stheta * rhs.x + ctheta * rhs.y + y;
	theta = acos(ctheta * rhs.ctheta - stheta * rhs.stheta);
	ctheta = cos(theta);
	stheta = sin(theta);

	// `this` is a pointer to our object, which we dereference for the object itself
	return *this;
}

rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D & rhs)
{
	// call operator*=() member function of lhs object (just above)
	lhs.operator*=(rhs);
	return lhs;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf)
{
	/// dtheta (degrees): 90 dx: 3 dy: 5
	os << "dtheta (degrees): " << tf.theta << "\t" << "dx: " << tf.x << "\t"\
	<< "dy: " << tf.y << "\n";

	return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf)
{
	// using friend function so that only this input fcn can overwrite
	// private params of Tranform2D
	std::cout << "Enter x component of Transform2D" << std::endl;
	is >> tf.x;

	std::cout << "Enter y component of Transform2D" << std::endl;
	is >> tf.y;

	std::cout << "Enter theta component of Transform2D" << std::endl;
	is >> tf.theta;

	tf.stheta = sin(tf.theta);
	tf.ctheta = cos(tf.theta);

	return is;
}



