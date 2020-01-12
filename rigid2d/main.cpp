#include <iostream>  // cin, cout, endl, getline
#include <vector>    // vector
#include <string>    // string
#include <math.h>
#include "rigid2d.hpp"

int main()
{	
	std::cout << "Enter Transform Tab as instructed below\n" << std::endl;

	rigid2d::Transform2D Tab;
	rigid2d::operator>>(std::cin, Tab);

	std::cout << "\nEnter Transform Tbc as instructed below\n" << std::endl;

	rigid2d::Transform2D Tbc;
	rigid2d::operator>>(std::cin, Tbc);

	// Now we calculate Tab, Tbc (given), and Tac, Tca
	rigid2d::Transform2D Tac {rigid2d::operator*(Tab, Tbc)};
	rigid2d::Transform2D Tca {Tac.inv()};
	// Also calculate Tba and Tcb
	rigid2d::Transform2D Tba {Tab.inv()};
	rigid2d::Transform2D Tcb {Tbc.inv()};

	// Now display each Transform 2D: Tab, Tbc, Tac, Tca
	std::cout << "\nDisplaying Tab" << std::endl;
	rigid2d::operator<<(std::cout, Tab);

	std::cout << "\nDisplaying Tbc" << std::endl;
	rigid2d::operator<<(std::cout, Tbc);

	std::cout << "\nDisplaying Tba" << std::endl;
	rigid2d::operator<<(std::cout, Tba);

	std::cout << "\nDisplaying Tcb" << std::endl;
	rigid2d::operator<<(std::cout, Tcb);

	std::cout << "\nDisplaying Tac" << std::endl;
	rigid2d::operator<<(std::cout, Tac);

	std::cout << "\nDisplaying Tca" << std::endl;
	rigid2d::operator<<(std::cout, Tca);

	std::cout << "\nNow, enter a Vector as instructed below\n" << std::endl;

	rigid2d::Vector2D v;
	rigid2d::operator>>(std::cin, v);

	std::cout << "\nNow, enter the frame of this vector (a, b, or c)\n" << std::endl;
	char vframe;
	std::cin >> vframe;

	// Now, output vector in frames a, b, and c depending on response (frame)

	if (vframe == 'a')
	{
		rigid2d::Vector2D vb = Tba.operator()(v);
		rigid2d::Vector2D vc = Tca.operator()(v);

		std::cout << "\nDisplaying the Vector in frame a" << std::endl;
		rigid2d::operator<<(std::cout, v);

		std::cout << "\nDisplaying the Vector in frame b" << std::endl;
		rigid2d::operator<<(std::cout, vb);

		std::cout << "\nDisplaying the Vector in frame c" << std::endl;
		rigid2d::operator<<(std::cout, vc);

	} else if (vframe == 'b')
	{
		rigid2d::Vector2D va = Tab.operator()(v);
		rigid2d::Vector2D vc = Tcb.operator()(v);

		std::cout << "\nDisplaying the Vector in frame a" << std::endl;
		rigid2d::operator<<(std::cout, va);

		std::cout << "\nDisplaying the Vector in frame b" << std::endl;
		rigid2d::operator<<(std::cout, v);

		std::cout << "\nDisplaying the Vector in frame c" << std::endl;
		rigid2d::operator<<(std::cout, vc);


	} else {
		// assume frame is c
		rigid2d::Vector2D va = Tac.operator()(v);
		rigid2d::Vector2D vb = Tbc.operator()(v);

		std::cout << "\nDisplaying the Vector in frame a" << std::endl;
		rigid2d::operator<<(std::cout, va);

		std::cout << "\nDisplaying the Vector in frame b" << std::endl;
		rigid2d::operator<<(std::cout, vb);

		std::cout << "\nDisplaying the Vector in frame c" << std::endl;
		rigid2d::operator<<(std::cout, v);
	}


    return 0;
}