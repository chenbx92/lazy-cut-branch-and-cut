#include"CVRPmodel.h"
#include<math.h>
#include<sstream>
#include<fstream>

using namespace std;

void CVRPmodel::solve()
{
	GRBEnv* env = new GRBEnv();
	GRBModel* model = new GRBModel(*env);
	double totaldemand=0;
	for (size_t i = 0; i < _DemandNode.size(); i++)
	{
		totaldemand += _DemandNode[i]->getdemand();
	}
	_vehicleNo = ceil(totaldemand / 100)+2;
	std::cout<<_vehicleNo;
	model->set(GRB_IntParam_LazyConstraints, 1);
	addvars(model);//xijk&zk
	adddegreeconstr(model);
	addnodeserivedonce(model);
	addcapacityconstr(model);
	addvehicledepotstartconstr(model);
	model->set(GRB_IntAttr_ModelSense, 1);
	subtourelim* cb = new subtourelim(model,_DemandNode,_Depot,_vehicleNo);
	model->setCallback(cb);
	model->write("CVRP.lp");
	model->optimize();
	// print result
	if (model->get(GRB_IntAttr_Status) == 2)
	{
		ofstream of("CVRP result.output");
		for (int k = 0; k < _vehicleNo; k++)
		{
			ostringstream ostring;
			ostring << "vehicleused" << k;
			string varname = ostring.str();
			std::cout<<varname;
			if (model->getVarByName(varname).get(GRB_DoubleAttr_X) > 0.001)
			{of << "vehicle" << k << "is used, the route is " << endl;
			for (size_t j = 0; j < _DemandNode.size(); j++)
			{
				if (model->getVarByName(getxijkvarname(_Depot, _DemandNode[j], k)).get(GRB_DoubleAttr_X) > 0.001)
				{
					of << "depot->" << _DemandNode[j]->getnodeid() << endl;
				}
				if (model->getVarByName(getxijkvarname(_DemandNode[j], _Depot, k)).get(GRB_DoubleAttr_X) > 0.001)
				{
					of << _DemandNode[j]->getnodeid() << "->depot" << endl;
				}
				for (size_t i = 0; i < _DemandNode.size(); i++)
				{
					if(i!=j)
					{
					if (model->getVarByName(getxijkvarname(_DemandNode[i], _DemandNode[j], k)).get(GRB_DoubleAttr_X) > 0.001)
					{
						of << _DemandNode[i]->getnodeid() << "->" << _DemandNode[j]->getnodeid() << endl;
					}}
				}
			}}
		}
		of.close();
	}
}

void CVRPmodel::addvars(GRBModel* model)
{
	//zk
	for (int k = 0; k < _vehicleNo; k++)
	{
		ostringstream ostring;
		ostring << "vehicleused" << k;
		string varname = ostring.str();
		model->addVar(0, 1, 10, GRB_BINARY, varname);
		model->update();
	}
	//xdepotjk,xjdepotk
	for (int k = 0; k < _vehicleNo; k++)
	{
		for (size_t j = 0; j < _DemandNode.size(); j++)
		{
			pair<double,double> p = _DemandNode[j]->getposition();
			double cost = sqrt(p.first*p.first + p.second*p.second);
			string varname1 = getxijkvarname(_Depot, _DemandNode[j],k);
			model->addVar(0, 1, cost, GRB_BINARY, varname1);
			string varname2 = getxijkvarname(_DemandNode[j], _Depot, k);
			model->addVar(0, 1, cost, GRB_BINARY, varname2);
			model->update();
		}
	}
	//xijk
	for (int k = 0; k < _vehicleNo; k++)
	{
		for (size_t i = 0; i < _DemandNode.size(); i++)
		{
			for (size_t j = 0; j < _DemandNode.size(); j++)
			{
				if (i != j)
				{
					pair<double, double> p1 = _DemandNode[i]->getposition();
					pair<double, double> p2 = _DemandNode[j]->getposition();
					double dx = p1.first - p2.first;
					double dy = p1.second - p2.second;
					double cost = sqrt(dx*dx + dy * dy);
					string varname1 = getxijkvarname(_DemandNode[i], _DemandNode[j], k);
					model->addVar(0, 1, cost, GRB_BINARY, varname1);
					string varname2 = getxijkvarname(_DemandNode[j], _DemandNode[i], k);
					model->addVar(0, 1, cost, GRB_BINARY, varname2);
					model->update();
				}

			}
		}

	}


}
void CVRPmodel::adddegreeconstr(GRBModel* model)
{
	for (int k = 0; k < _vehicleNo; k++)
	{
		for (size_t i = 0; i < _DemandNode.size(); i++)
		{
			GRBLinExpr lhsexpr=0;
			GRBLinExpr rhsexpr=0;
			for (size_t j = 0; j < _DemandNode.size(); j++)
			{
				if (i != j)
				{
					lhsexpr += model->getVarByName(getxijkvarname(_DemandNode[i], _DemandNode[j], k));
					rhsexpr += model->getVarByName(getxijkvarname(_DemandNode[j], _DemandNode[i], k));
				}
			}
			lhsexpr += model->getVarByName(getxijkvarname(_DemandNode[i], _Depot, k));
			rhsexpr += model->getVarByName(getxijkvarname(_Depot, _DemandNode[i], k));
			ostringstream ostring;
			ostring << "vehicle" << k << _DemandNode[i]->getnodeid() << "degree constr";
			string constrname = ostring.str();
			string constrname1=_DemandNode[i]->getnodeid()+"serviced";
			model->addConstr(lhsexpr,GRB_EQUAL,rhsexpr,constrname);
			model->update();
		}
	}

}
void CVRPmodel::addnodeserivedonce(GRBModel* model)
{

	for (size_t i = 0; i < _DemandNode.size(); i++)
	{
		GRBLinExpr lhsexpr=0;
		for (int k = 0; k < _vehicleNo; k++)
		{
			for (size_t j = 0; j < _DemandNode.size(); j++)
			{
				if (i != j)
				{
					lhsexpr += model->getVarByName(getxijkvarname(_DemandNode[j], _DemandNode[i], k));
				}
			}
					lhsexpr+=model->getVarByName(getxijkvarname(_Depot, _DemandNode[i], k));
		}
		string constrname=_DemandNode[i]->getnodeid()+"serviced";
		model->addConstr(lhsexpr,GRB_EQUAL,1,constrname);
		model->update();
	}
}
void CVRPmodel::addcapacityconstr(GRBModel* model)
{
	for (int k = 0; k < _vehicleNo; k++)
	{
		GRBLinExpr lhsexpr = 0;
		for (size_t j = 0; j < _DemandNode.size(); j++)
		{
			double cost = _DemandNode[j]->getdemand();
			lhsexpr += _DemandNode[j]->getdemand()*model->getVarByName(getxijkvarname(_Depot, _DemandNode[j], k));
			for (size_t i = 0; i < _DemandNode.size(); i++)
			{
				if (i!=j)
				{
					lhsexpr += _DemandNode[j]->getdemand()*model->getVarByName(getxijkvarname(_DemandNode[i], _DemandNode[j], k));}
			}
		}

		ostringstream ostring;
		ostring << "vehicleused" << k;
		string varname = ostring.str();
		GRBLinExpr rhsexpr = 100*model->getVarByName(varname);
		ostringstream ostring1;
		ostring1 << "capacity vehicle" << k;
		string constrname = ostring1.str();
		model->addConstr(lhsexpr, GRB_LESS_EQUAL, rhsexpr, constrname);
		model->update();
	}
}
void CVRPmodel::addvehicledepotstartconstr(GRBModel* model)
{
	for (int k=0;k<_vehicleNo;k++)
	{
		GRBLinExpr lhsexpr = 0;
		for (size_t j = 0; j < _DemandNode.size(); j++)
		{
			lhsexpr += model->getVarByName(getxijkvarname(_Depot, _DemandNode[j], k));
		}
		ostringstream ostring;
		ostring << "vehicleused" << k;
		string varname = ostring.str();
		GRBLinExpr rhsexpr = model->getVarByName(varname);
		ostringstream ostring1;
		ostring1 << "depotconstr4vehicle" << k;
		string constrname = ostring1.str();
		model->addConstr(lhsexpr, GRB_GREATER_EQUAL, rhsexpr, constrname);
		model->update();
	}
}
string CVRPmodel::getxijkvarname(Node* node1, Node * node2,int vehicletag)
{
	ostringstream ostring;
	ostring << node1->getnodeid()<<"2" << node2->getnodeid() << "byvehicle" << vehicletag;
	return ostring.str();
}