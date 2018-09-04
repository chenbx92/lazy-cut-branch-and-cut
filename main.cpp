#include "ObjMgr.h"
#include<iostream>
#include "CVRPmodel.h"

int main(int argc, char* argv[])
{
	try {
		ObjMgr::instance()->loadDemandNode();
		CVRPmodel cvrpmodel(ObjMgr::instance()->getDepot(), ObjMgr::instance()->getDemandNode());
		cvrpmodel.solve();
	}
	catch (GRBException e) { std::cout << e.getMessage() << "\n"; }
	catch (int i) { std::cout << i; }
	return 0;
}