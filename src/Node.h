#pragma once
#include <string>
#include <utility>

using namespace std;

class Node
{
public:
	void setnodeid(string nodeid) { _nodeid = nodeid; };
	void setdemand(double demand) { _demand = demand; };
	void setposition(pair<double, double> position) { _position = position; };
	double getdemand() { return _demand; };
	string getnodeid() { return _nodeid; };
	pair <double, double> getposition() { return _position; };
private:
	pair<double, double> _position;
	string _nodeid;
	double _demand;

};