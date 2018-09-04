#include"ObjMgr.h"
#include<fstream>
#include<iostream>
#include<sstream>
#include "Node.h"
#include <string>

using namespace std;

ObjMgr* ObjMgr:: _instance=0;

ObjMgr* ObjMgr::instance()
{
	if (!_instance)
		_instance = new ObjMgr();
	return _instance;
}
void ObjMgr::loadDemandNode()
{
	ifstream openf("DemandNode.input");
	if (!openf)
	{
		cout << "Error:openfile";
		throw(0);
	}
	string aline;
	while (openf.good())
	{
		getline(openf, aline);
		if (aline.empty())
		{
			continue;
		}
		string nodeid;
		double positionx;
		double positiony;
		double demand;
		istringstream pausealine(aline);
		pausealine >> nodeid;
		pausealine >> positionx;
		pausealine >> positiony;
		pausealine >> demand;
		Node* tempnode = new Node();
		tempnode->setnodeid(nodeid);
		pair<double, double> position(positionx, positiony);
		tempnode->setposition(position);
		tempnode->setdemand(demand);
		_demandnode.push_back(tempnode);
	}
	openf.close();
	_depot = new Node();
	_depot->setnodeid("depot");
	_depot->setposition(make_pair(0, 0));
	_depot->setdemand(0);

}