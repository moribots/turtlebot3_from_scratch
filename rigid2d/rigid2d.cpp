

std::ostream & rigid2d::operator<<(std::ostream & os, const Vector2D & v)
{
	os << "[" << v.x << " " << v.y << "]" << "\n";
}

std::istream & rigid2d::operator>>(std::istream & is, Vector2D & v)
{
	std::cout << "Enter x component of Vector" << std::endl;
	is >> v.x;

	std::cout << "Enter y component of Vector" << std::endl;
	is >> v.y;
}

Transform2D::Transform2D()
{
	// Below are values of class params for ID Transform.
	x = 0;
	y = 0;
	theta = 0;
	ctheta = std::cos(theta);
	stheta = std::sin(theta);
}

Transform2D::Transform2D(const Vector2D & trans)
{	
	// Pure Translational
	x = trans.x;
	y = trans.y;
	theta = 0;
	ctheta = std::cos(theta);
	stheta = std::sin(theta);
}

Transform2D::Transform2D(double radians)
{
	// Pure Rotational
	x = 0;
	y = 0;
	theta = radians;
	ctheta = std::cos(theta);
	stheta = std::sin(theta);
}

Transform2D::Transform2D(const Vector2D & trans, double radians)
{
	// Mixed Transform
	x = trans.x;
	y = trans.y;
	theta = radians;
	ctheta = std::cos(theta);
	stheta = std::sin(theta);
}

Vector2D Transform2D::operator()(Vector2D v) const
{
	// Transform vector v into vector vp

	Vector2D vp;

	vp.x = v.x * std::cos(theta) - v.y * std::sin(theta);
	vp.y = v.x * std::sin(theta) + v.y * std::cos(theta);

	return vp;
}

Transform2D Transform2D::Transform2D inv() const
{
	// Create temp of 2d tranform (clone it essentially)
	Transform2D temp2d(theta, ctheta, stheta, x, y);

	// for transpose, flip sintheta
	temp2d.stheta = -temp2d.stheta;

	// create new temp vector v
	Vector2D v;
	v.x = -temp2d.x;
	v.y = -temp2d.y;
	// this performs p' = R.T*p
	vp = this->Transform2D::operator()(v);

	temp2d.x = vp.x;
	temp2d.y = vp.y;

	return temp2d;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{

}

Transform2D rigid2d::operator*(Transform2D lhs, const Transform2D & rhs)
{
	// call operator*=() member function of lhs object (just above)
	lhs.operator*=(rhs);
	return lhs;
}

std::ostream & rigid2d::operator>>(std::istream & is, Transform2D & tf)
{
	// using friend function so that only this input fcn can overwrite
	// private params of Tranform2D
	std::cout << "Enter x component of Transform2D" << std::endl;
	is >> tf.x;

	std::cout << "Enter y component of Transform2D" << std::endl;
	is >> tf.y;

	std::cout << "Enter theta component of Transform2D" << std::endl;
	is >> tf.theta;
}

std::istream & rigid2d::operator>>(std::istream & is, Transform2D & tf)
{
	/// dtheta (degrees): 90 dx: 3 dy: 5
	os << "dtheta (degrees): " << tf.theta << "\t" << "dx: " << tf.x << "\t"\
	<< "dy: " << tf.y << "\n";
}



