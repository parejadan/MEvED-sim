#include "sim_struct.h"
#include <stdio.h> //printf
#include <fstream>
//#include <sstream>
#include <ctime> //produce random seeds for srand & rand

using namespace std;

Master::Master(double p, double u) { prob = p; util = u; }

void Master::flipCoin() {
	double r = randFunction();
	if (r < PA) { //master checks
		choice = 1;
	} else { //master doesn't check
		choice = 0;
	}
}

vector< DroneUnit > setCoins(vector< DroneUnit > GRPS, int drnCount, Master* mstr) {
	int followers = 0;
	int divergent = 0;
	for (int i = 0; i < drnCount; i++) {
		GRPS[i].flipCoin();

		if (GRPS[i].choice == 1) { followers += GRPS[i].size; }
		else { divergent += GRPS[i].size; }
	}

	mstr->followers = followers;
	mstr->divergent = divergent;

	return GRPS;
}

int main() {
	srand( time(NULL) );

	string pcDat = "", distDat = "";
	ofstream simData, simData2;
	stringstream ss;

	Master mstr(0.5, 0); //init master with a prob of 0.5, and util 0
	vector<DroneUnit> GRPS = genSingletons(n); //generate groups (drones)
	int len = GRPS.size();
	compLearningRate(); //set a learning rate for groups
	
	for (int r = 1; r < R_MX; r++) {
		GRPS = setCoins(GRPS, len, &mstr); //group decide to follow/deviate
		mstr.flipCoin(); //master decides to check/not-check

		for (int i = 0; i < len; i++) {
			dy_computeUtil(&GRPS[i], &mstr);
			
			dy_updatePC(&GRPS[i], &mstr); //update group's PC
			printf("%d) PC %f| choice %d| util %f\n", i, GRPS[i].pc, GRPS[i].choice, GRPS[i].util);
			// += ss.str(GRPS[i].pc) + ( (i == len-1) ? "\n":",");
			compPC += GRPS[i].pc;
		}

		dy_updatePA(&mstr);
		printf("master) prob %f| choice %d| util %f\n", mstr.prob, mstr.choice, mstr.util);
		//create disturbance when all PCs converge to 0
		if (compPC == 0) {
			distCnt++;
			cycles.push_back(r - distAt); //record length of the cylce
			distAt = r;
		printf("\nReset #%d on round # %d| cycle: %d|\n", distCnt, r, cycles[distCnt-1]);
		}

		printf("compPC) %f\n", compPC);

		for (int i = 0; i < len; i++) {
			GRPS[i].pc = randFunction(); //reset PC randomly
			printf("PC %f|%s", GRPS[i].pc, ( (i == len-1) ? "\n":"") );
		}

		//distDat += ss.str(r) + "," + ss.str(cycles[distCnt-1]) + "\n";
	}
}

/////////////////////////////////////////////////////////////////////////////
/* !#EVO#! */

void compLearningRate() { alphaW = randFunction() * (1 / (aspiration + WPC) ); }

void dy_updatePC(DroneUnit* drn, Master* mstr) {
	double tmp;
	if (mstr->choice) {
		if (drn->choice == 1) { //unit follows
			tmp = (drn->pc + alphaW * (aspiration - (WBY - WCT))) * drn->choice;
		} else { //unit deviated 
			tmp = (drn->pc - alphaW * (aspiration - WPC)) * drn->choice;			
		}
	} else { //master does not check
		if (mstr->divergent > tou) {
			if (drn->choice == 1) { //unit follows but not part of the majority
				tmp = (drn->pc + alphaW * (aspiration + WCT)) * drn->choice;
			} else { //unit diviates and part of the majority
				tmp = (drn->pc + alphaW * (WBY - aspiration)) * drn->choice;
			}
		} else {
			if (drn->choice == 1) { //unit follows and is part of the majority
				tmp = (drn->pc + alphaW * (aspiration - (WBY - WCT))) * drn->choice;				
			} else { //unit deviates but is not part of the majority 
				tmp = (drn->pc - alphaW * aspiration) * drn->choice;
			}
		}
	}

	drn->pc = max( 0.0, min(1.0, tmp) );
}

void dy_updatePA(Master* mstr) {
	if (mstr->choice) {
		double tmp = mstr->prob + alphaM * ( (mstr->divergent * 1.0) / n - tou);
		//printf("PA %f| alphaM %f| divergent %d| n %d| tou %d\n", mstr->prob, alphaM, mstr->divergent, n, tou);
		//printf("updating PA: %f\n", tmp);  
		mstr->prob = min( 1.0, max( mstr->prob, tmp ) );
	}
}

void dy_computeUtil(DroneUnit* drn, Master* mstr) {
	double tmp;
	if (mstr->choice) { //master checks
		if (drn->choice == 1) { //unit follows, provide follwers reward
			tmp = WBY - WCT; 
		} else { //unit deviates, apply punishment 
			tmp = -WPC;
		}
	} else { //master does not check
		if (mstr->divergent > tou) { //majority deviated 
			if (drn->choice == 1) { //unit follows
				tmp = -WCT;
			} else { //unit deviated
				tmp = WBY;
			}
		} else { //majority followed
			if (drn->choice == 1) { //unit followed
				tmp = WBY - WCT;
			} else { //unit deviated 
				tmp = 0;
			}
		}
	}

	//mstr->util += unit.util * (-1); //units reward is negative towards the master's
	mstr->util += tmp;
	drn->util += tmp;
}

