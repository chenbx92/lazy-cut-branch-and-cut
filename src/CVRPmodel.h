#pragma once
#include<vector>
#include"Node.h"
#include<gurobi_c++.h>

using namespace std;

class CVRPmodel
{
public:
	CVRPmodel(Node* depot,vector<Node*> demandnode)
	{
		_Depot = depot;
		_DemandNode.assign(demandnode.begin(),demandnode.end());
	}
	void solve();
	void addvars(GRBModel* model);
	void adddegreeconstr(GRBModel* model);
	void addcapacityconstr(GRBModel* model);
	void addvehicledepotstartconstr(GRBModel* model);
	void addnodeserivedonce(GRBModel* model);
	string getxijkvarname(Node* node1, Node * node2,int vehicletag);
private:
	vector<Node*> _DemandNode;
	Node* _Depot;
	int _vehicleNo;
};
class subtourelim : public GRBCallback
{
public:
	subtourelim(GRBModel* model, vector<Node*> demandnode, Node* depot, int vehicleno)
	{
		_model = model;
		_demandnode = demandnode;
		_depot = depot;
		_vehicleno = vehicleno;
	}
	void callback();
	string getxijkvarname(Node* node1, Node * node2, int vehicletag);
	void findsubtour(vector<Node*> &subtour,int k);
private:
	GRBModel* _model;
	vector<Node*> _demandnode;
	Node* _depot;
	int _vehicleno;
};