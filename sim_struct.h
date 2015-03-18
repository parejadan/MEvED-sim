/**
 * Here contains all the parameters that dictate simulations. Use the following
 * 	anchors for file navigation:
 *
 *  - parameters for evolutionary dynamics, hit crtl+f, then type !#EVO#!
 *	- parameters for Mixed equilibria, hit ctrl+f, then type !#MIX#!
 *	- parameters such as number of drones drones, rounds, and anything that overlaps
 *		between both models, hit crtl+f, then type !#GEN#!
 */


//////////////////////////////////////////////////////////////////////////////
#include "drones/DroneUnits.h"
#include <vector>
 /* !#GEN#! */
int n = 9; //number of drones within overall units (read DroneUnit.h for more info)
int R_MX = 140; //number of times to repeat a game
int tou = (int) (n / 2.0);

class Master {
public:
	Master(double p, double u); //constructor for prob and util
	void flipCoin();
	int divergent; //drones that chose not to compute during this round
	int followers; //drones that chose to compute	
	int choice; //master's choice to check computing results
	int correct; //how frequent is the master getting the correct results
	double prob; //probability of verifying/audeting
	double util;
};

std::vector< DroneUnit > setCoins(std::vector< DroneUnit >, Master*);

double WPC = 1;		//drone punishment for being caught cheating
double WCT = 0.1;	//drone cost for computing the task
double WBY = 1;		//drone reward
double MPW = 0; 	//master punishment for accepting a W answer
double *MCY = &WBY;	//master cost for accepting w's answer
double MCA = 20; 	//master cost for verifying 
double MBR = 0;		//master benefit for accepting a R answer

////////////////////////////////////////////////////////////////////////////
/* !#MIX#! */

//std::string GAME_NAME = "gameN-0n_model-MAJORITY"; //name for mixed equilibrai type
//available worker 
//const std::string type_1 = "_disobedient";
//const std::string type_2 = "_compliant";
//std::string type = type_2; //mixed equilibrai to run for simulations 

/////////////////////////////////////////////////////////////////////////////
/* !#EVO#! */

/*
 * pcComp contains a composite value of all drones probability of cheating.
 *  When the value is 0 at the end of a round, this means all drones are
 *	 honest, so a disturbance must occour. distCnt contains 
 */

double pcComp = 0;
//cycles length is defined as long it took all drones to converge to being honest. 
std::vector<int> cycles; //each item is a cycle length
int distAt = 0, distCnt = 0; //purpose not fully understood a.t.m, will review to see if still needed

//assumed that all drones have the same learning rate
double aspiration = .5;
double alphaW = 0.1; //use computeLearningRate() - assume all drones have the same learning rate

//discount fator that expresses master telerable ratio of cheaters
double PA = 0.5; //master probability fo verifying 
double alphaM = .1; //master learning rate;0 = never adjust PA, anything greater shows proportion of override

void compLearningRate();
void dy_updatePC(DroneUnit*, Master*);
void dy_updatePA(Master*);
void dy_computeUtil(DroneUnit*, Master*);