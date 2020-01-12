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

	// // Now we calculate Tab, Tbc (given), and Tac, Tca
	rigid2d::Transform2D Tac {rigid2d::operator*(Tab, Tbc)};
	rigid2d::Transform2D Tca {Tac.inv()};

	// Now display each Transform 2D: Tab, Tbc, Tac, Tca
	std::cout << "\nDisplaying Tab" << std::endl;
	rigid2d::operator<<(std::cout, Tab);

	std::cout << "\nDisplaying Tbc" << std::endl;
	rigid2d::operator<<(std::cout, Tbc);

	std::cout << "\nDisplaying Tac" << std::endl;
	rigid2d::operator<<(std::cout, Tac);

	std::cout << "\nDisplaying Tca" << std::endl;
	rigid2d::operator<<(std::cout, Tca);


    return 0;
}