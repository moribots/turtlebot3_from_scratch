#include <iostream>  // cin, cout, endl, getline
#include <vector>    // vector
#include <string>    // string
#include <math.h>
#include "rigid2d.hpp"

int main()
{	
	std::cout << "Enter Transform Tab as instructed below" << std::endl;

	rigid2d::Transform2D Tab;
	rigid2d::operator>>(std::cin, Tab);

	// cout << "Enter Transform Tbc as instructed below" << endl;

	// rigid2d::Transform2D Tbc;
	// rigid2d::operator>>(cin, &Tbc);

	// // Now we calculate Tab, Tbc (given), and Tac, Tca
	// rigid2d::Transform2D Tac {rigid2d::operator*(Tab, &Tbc)};
	// rigid2d::Transform2D Tca {Tac.inv()};

	// Now display each Transform 2D: Tab, Tbc, Tac, Tca
	std::cout << "Displaying Tab" << std::endl;
	rigid2d::operator<<(std::cout, Tab);

	// cout << "Displaying Tbc" << endl;
	// rigid2d::operator<<(cout, Tbc);

	// cout << "Displaying Tac" << endl;
	// rigid2d::operator<<(cout, Tac);

	// cout << "Displaying Tca" << endl;
	// rigid2d::operator<<(cout, Tca);


    return 0;
}