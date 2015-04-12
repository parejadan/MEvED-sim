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

int len, maxR, END = 200;
int n = 9, devCount = (int)ceil(n/2.0), tou = (int) (n / 2.0);

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

double WBA = 0.01;	//drone reward
double WPC = WBA/n;	//drone punishment for being caught cheating
double WCT = 0.1;	//drone cost for computing the task
double MPW = 0.0; 	//master punishment for accepting a wrong answer
double *MCA = &WBA;	//master cost for accepting answer
double MCV = 20; 	//master cost for verifying 
double MBR = 0;		//master benefit for accepting a right answer

////////////////////////////////////////////////////////////////////////////
/* !#MIX#! */ 
int minC, maxC;
std::vector<int> counts;
double delta, pc, pv, eps = 0.01;
bool doMixed(int, Master*);
double mx_masterProb();
void mx_masterUtil(Master*);
void mx_compteUtil(DroneUnit*, Master*);

/////////////////////////////////////////////////////////////////////////////
/* !#EVO#! */
double aspiration = 0.01; //assumed that all drones have the same learning rate
double alphaW = 0.01; //use computeLearningRate() - assume all drones have the same learning rate
double alphaM = 0.01; //master learning rate;0 = never adjust PA, anything greater shows proportion of override

bool doEvo(std::vector<DroneUnit*>, Master*, int);
void dy_updatePC(DroneUnit*, Master*, double);
void dy_updatePA(Master*);
void dy_computeUtil(DroneUnit*, Master*, double);