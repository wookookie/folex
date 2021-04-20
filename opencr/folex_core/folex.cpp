//

#include "folex.h"


Folex::Folex()
{

}

Folex::~Folex()
{

}

bool Folex::initFolex()
{
	//diagnosis.init();
	
	dynamixel_driver.init();
	dynamixel_driver.enableDynamixel();
	//dynamixel_driver.resetPosition();
	//dynamixel_driver.test();

	return true;
}

void Folex::gaitTestFirst()
{
	dynamixel_driver.gaitTestFirst();
}

void Folex::gaitTestSecond()
{
	dynamixel_driver.gaitTestSecond();
}

void Folex::gaitTrot()
{
	dynamixel_driver.gaitTrot();
}


