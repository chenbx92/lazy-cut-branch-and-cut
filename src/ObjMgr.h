#pragma once
#include<vector>
#include"Node.h"
using namespace std;

class ObjMgr
{
public:
	static ObjMgr* instance();
protected:
	static ObjMgr* _instance;
public:
	void loadDemandNode();
	vector<Node*> getDemandNode() { return _demandnode; };
	Node* getDepot() { return _depot; };
private:
	vector<Node*> _demandnode;
	Node* _depot;

};