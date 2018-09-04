#include "CVRPmodel.h"
#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>


void subtourelim::callback()
{
	try{
		if (where == GRB_CB_MIPSOL)
		{
			cout<<"callback"<<endl;
			for (int k = 0; k < _vehicleno; k++)
			{
				vector<Node*> nodesvehiclekserved;
				int vehiclektotalserved=0;
				for (size_t j = 0; j < _demandnode.size(); j++)
				{
					if (getSolution(_model->getVarByName(getxijkvarname(_depot, _demandnode[j], k)))>0.9)
					{
						vehiclektotalserved ++;
						nodesvehiclekserved.push_back(_demandnode[j]);
					}
					for (size_t i = 0; i < _demandnode.size(); i++)
					{
						if(i!=j){
							if(getSolution(_model->getVarByName(getxijkvarname(_demandnode[i], _demandnode[j], k)))>0.9)
							{
								vehiclektotalserved ++;
								nodesvehiclekserved.push_back(_demandnode[j]);}					
						}
					}
				}

				cout<<vehiclektotalserved<<endl;
				if (vehiclektotalserved>0)
				{
					vector<Node*> subtour;//depot³ö·¢µÄsubtour
					vector<Node*> leftdemandnode;
					findsubtour(subtour,k);
					for (size_t t = 0; t < nodesvehiclekserved.size(); t++)
					{
						vector<Node*>::const_iterator iter = find(subtour.begin(), subtour.end(), nodesvehiclekserved[t]);
						if (iter == subtour.end())
						{
							leftdemandnode.push_back(nodesvehiclekserved[t]);
						}
					}
					cout<<"subtour"<<subtour.size()<<"leftdemandnode"<<leftdemandnode.size()<<endl;
					if (subtour.size()< vehiclektotalserved+1)
					{
						std::cout<<"lazycut"<<endl;
						GRBLinExpr rhsexpr;
						for (size_t i = 0; i < subtour.size(); i++)
						{
							for (size_t j = 0; j < leftdemandnode.size(); j++)
							{
								rhsexpr += _model->getVarByName(getxijkvarname(subtour[i], leftdemandnode[j], k));
							}
						}
						addLazy(rhsexpr, GRB_GREATER_EQUAL,1.0);
				}
				}
			}
		}
	}
	catch (GRBException e) { std::cout << e.getMessage() << "\n"; }
}
string subtourelim::getxijkvarname(Node* node1, Node * node2, int vehicletag)
{
	ostringstream ostring;
	ostring << node1->getnodeid() << "2" << node2->getnodeid() << "byvehicle" << vehicletag;
	return ostring.str();
}
void subtourelim::findsubtour(vector<Node*> &subtour,int k)
{
	subtour.push_back(_depot);
	bool ifcontinue = true;
	while (ifcontinue) 
	{
		Node* currentnode = subtour.back();
		vector<Node*> neighbors;
		int count = 0;
		for (size_t n = 0; n < _demandnode.size(); n++)
		{
			count++;
			if (currentnode!=_demandnode[n])
			{
				if (getSolution(_model->getVarByName(getxijkvarname(currentnode, _demandnode[n], k))) > 0.5)
				{
					subtour.push_back(_demandnode[n]);
					break;
				}
			}
		}
		if (count == _demandnode.size())
		{
			ifcontinue = false;
		}
	}
}