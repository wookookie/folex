//

#ifndef FOLEX_H
#define FOLEX_H

#include "folex_diagnosis.h"
#include "folex_dynamixel_driver.h"
#include "kinematics.h"

typedef struct
{
	std::vector<uint16_t> id;
	

} DynamixelArray;


class Folex
{
private:
	FolexDiagnosis diagnosis;
	FolexDynamixelDriver dynamixel_driver;
	Kinematics kinematics;




public:
	Folex();
	~Folex();

	bool initFolex();
	void gaitTestFirst();
	void gaitTestSecond();
	void gaitTrot();
};



#endif // FOLEX_H
