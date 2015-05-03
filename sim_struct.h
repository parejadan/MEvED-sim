/**
 *  - locate parameters for evolutionary dynamics, hit crtl+f, then type !#EVO#!
 *	- locate parameters for Mixed equilibria, hit ctrl+f, then type !#MIX#!
 *	- locate parameters such as number of drones drones, rounds, and anything that overlaps
 *		between both models, hit crtl+f, then type !#GEN#!
 */
//////////////////////////////////////////////////////////////////////////////
 /* !#GEN#! */
#include "drones/DroneUnits.h"
#include <vector>
#include <math.h>

int len, maxR, END = 100;
int n = 9, devCount = (int)ceil(n/2);

class Master {
public:
	Master(double p); //constructor for prob
	void flipCoin();
	int correct; //number of correct answers master recieves
	int incorrect; //number of incorrect answers master recieves 
	int choice; //master's choice to check computing results
	double prob; //probability of verifying/audeting
	double util;
};

void setCoins(std::vector< DroneUnit* >, Master*, int, double);

double WBA = 1.0;	//drone reward
double WPC = WBA;	//drone punishment for being caught cheating
double WCT = 0.1;	//drone cost for computing the task
double *MCA = &WBA;	//master cost for accepting answer
double MPW = n*WBA; //master punishment for accepting a wrong answer
double MCV = n*WBA; //master cost for verifying 
double MBR = n*WBA;	//master benefit for accepting a right answer

////////////////////////////////////////////////////////////////////////////
/* !#MIX#! */
int minC, maxC;
std::vector<int> counts;
double delta, pc, pv, eps = 0.01;
bool doMixed(int, double, double, Master*, std::vector<DroneUnit*>);
double mx_masterProb();
void mx_mstrUtil(Master*, double);
void os_mstrUtil(Master*, double);
void mx_wrkUtil(DroneUnit*, Master*, double, double);

/////////////////////////////////////////////////////////////////////////////
/* !#EVO#! */
double aspiration = 0.1; //worker i's "a" value
double alphaW = 0.01; //learning rate for worker
double alphaM = 0.01; //learning rate for master
double tou = 0.5;

bool doEvo(std::vector<DroneUnit*>, Master*, int, double, double);
void dy_updatePC(DroneUnit*, Master*, double, double);
void dy_updatePA(Master*);
void dy_wrkUtil(DroneUnit*, Master*, double, double);
void dy_mstrUtil(Master*, double, double);