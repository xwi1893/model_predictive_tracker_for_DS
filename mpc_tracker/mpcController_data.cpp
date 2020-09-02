#include "mpcController.h"

P_vehicle MpcController::CClassHatchback{
	//Mass
	1100,

	//Inertia Z
	2942,

	//lf
	1.0,

	//lr
	1.635,

	//Cf
	1.6395e05,

	//Cr
	1.4642e05,

	//length
	4.4,

	//width
	1.725
};

P_controller MpcController::P_MPC{
	// Weight Q
	{0,0,2.7183,13.5914},

	// Weight Ru
	2.7183,

	// Weight Rdu
	9.1970
};
