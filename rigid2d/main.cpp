#include <iostream>  // cin, cout, endl, getline
#include <vector>    // vector
#include <string>    // string
#include <math.h>
#include "rigid2d.hpp"

int main()
{	
	std::cout << "Enter Transform Tab as instructed below:" << std::endl;

	rigid2d::Transform2D Tab;
	rigid2d::operator>>(std::cin, Tab);

	std::cout << "\nEnter Transform Tbc as instructed below:" << std::endl;

	rigid2d::Transform2D Tbc;
	rigid2d::operator>>(std::cin, Tbc);

	// Now we calculate Tab, Tbc (given), and Tac, Tca
	rigid2d::Transform2D Tac {rigid2d::operator*(Tab, Tbc)};

	// rigid2d::Transform2D Tca {Tac.inv()};
	// Also calculate Tba and Tcb
	rigid2d::Transform2D Tba {Tab.inv()};
	rigid2d::Transform2D Tcb {Tbc.inv()};
	rigid2d::Transform2D Tca {rigid2d::operator*(Tcb, Tba)};

	std::cout << "\nNow, enter a Vector as instructed below:" << std::endl;

	rigid2d::Vector2D v;
	rigid2d::operator>>(std::cin, v);

	std::cout << "\nNow, enter a Twist as instructed below:" << std::endl;

	rigid2d::Twist2D tw;
	rigid2d::operator>>(std::cin, tw);

	std::cout << "\nNow, enter the frame of these inputs (a, b, or c):" << std::endl;
	char vframe;
	std::cin >> vframe;

	// Now display each Transform 2D: Tab, Tbc, Tac, Tca
	std::cout << "\nDisplaying Tab" << std::endl;
	rigid2d::operator<<(std::cout, Tab);

	std::cout << "\nDisplaying Tba" << std::endl;
	rigid2d::operator<<(std::cout, Tba);

	std::cout << "\nDisplaying Tbc" << std::endl;
	rigid2d::operator<<(std::cout, Tbc);

	std::cout << "\nDisplaying Tcb" << std::endl;
	rigid2d::operator<<(std::cout, Tcb);

	std::cout << "\nDisplaying Tac" << std::endl;
	rigid2d::operator<<(std::cout, Tac);

	std::cout << "\nDisplaying Tca" << std::endl;
	rigid2d::operator<<(std::cout, Tca);

	// Now, output vector in frames a, b, and c depending on response (frame)
	if (vframe == 'a')
	{
		// VECTORS
		rigid2d::Vector2D vb = Tba.operator()(v);
		rigid2d::Vector2D vc = Tca.operator()(v);

		std::cout << "\nDisplaying the Vector in frame a" << std::endl;
		rigid2d::operator<<(std::cout, v);

		std::cout << "\nDisplaying the Vector in frame b" << std::endl;
		rigid2d::operator<<(std::cout, vb);

		std::cout << "\nDisplaying the Vector in frame c" << std::endl;
		rigid2d::operator<<(std::cout, vc);

		// TWISTS
		rigid2d::Twist2D tw_b = tw.convert(Tba);
		rigid2d::Twist2D tw_c = tw.convert(Tca);

		std::cout << "\nDisplaying the Twist in frame a" << std::endl;
		rigid2d::operator<<(std::cout, tw);

		std::cout << "\nDisplaying the Twist in frame b" << std::endl;
		rigid2d::operator<<(std::cout, tw_b);

		std::cout << "\nDisplaying the Twist in frame c" << std::endl;
		rigid2d::operator<<(std::cout, tw_c);

	} else if (vframe == 'b')
	{
		// VECTORS
		rigid2d::Vector2D va = Tab.operator()(v);
		rigid2d::Vector2D vc = Tcb.operator()(v);

		std::cout << "\nDisplaying the Vector in frame a" << std::endl;
		rigid2d::operator<<(std::cout, va);

		std::cout << "\nDisplaying the Vector in frame b" << std::endl;
		rigid2d::operator<<(std::cout, v);

		std::cout << "\nDisplaying the Vector in frame c" << std::endl;
		rigid2d::operator<<(std::cout, vc);

		// TWISTS
		rigid2d::Twist2D tw_a = tw.convert(Tab);
		rigid2d::Twist2D tw_c = tw.convert(Tcb);

		std::cout << "\nDisplaying the Twist in frame a" << std::endl;
		rigid2d::operator<<(std::cout, tw_a);

		std::cout << "\nDisplaying the Twist in frame b" << std::endl;
		rigid2d::operator<<(std::cout, tw);

		std::cout << "\nDisplaying the Twist in frame c" << std::endl;
		rigid2d::operator<<(std::cout, tw_c);



	} else {
		// assume frame is c

		// VECTORS
		rigid2d::Vector2D va = Tac.operator()(v);
		rigid2d::Vector2D vb = Tbc.operator()(v);

		std::cout << "\nDisplaying the Vector in frame a" << std::endl;
		rigid2d::operator<<(std::cout, va);

		std::cout << "\nDisplaying the Vector in frame b" << std::endl;
		rigid2d::operator<<(std::cout, vb);

		std::cout << "\nDisplaying the Vector in frame c" << std::endl;
		rigid2d::operator<<(std::cout, v);

		// TWISTS
		rigid2d::Twist2D tw_a = tw.convert(Tac);
		rigid2d::Twist2D tw_b = tw.convert(Tbc);

		std::cout << "\nDisplaying the Twist in frame a" << std::endl;
		rigid2d::operator<<(std::cout, tw_a);

		std::cout << "\nDisplaying the Twist in frame b" << std::endl;
		rigid2d::operator<<(std::cout, tw_b);

		std::cout << "\nDisplaying the Twist in frame c" << std::endl;
		rigid2d::operator<<(std::cout, tw);

	}


    return 0;
}