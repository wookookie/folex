//

#ifndef FOLEX_H
#define FOLEX_H

#include "folex_diagnosis.h"
#include "folex_dynamixel_driver.h"

typedef struct
{
	std::vector<uint16_t> id;
	

} DynamixelArray;


class Folex
{
private:
	FolexDynamixelDriver *dynamixel_driver;




public:
	Folex();
	~Folex();

	void initFolex();



};



#endif // FOLEX_H
