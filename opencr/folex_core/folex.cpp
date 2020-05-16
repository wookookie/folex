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
	diagnosis.init();
	
	dynamixel_driver.init();
	dynamixel_driver.enableDynamixel();
	dynamixel_driver.initPosition();	

	return true;
}

