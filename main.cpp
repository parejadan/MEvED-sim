#include "sim_struct.h"
#include <stdio.h> //printf
<<<<<<< HEAD
=======
#include <stdlib.h> //rand
>>>>>>> 73c91422821cd516f483a1d0eed568276e92b3b9
#include <fstream>
#include <sstream>
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

<<<<<<< HEAD
vector< DroneUnit > setCoins(vector< DroneUnit > GRPS, int drnCount, Master* mstr) {
=======
vector<DroneUnit> setCoins(vector< DroneUnit > GRPS, int drnCount, Master* mstr) {
>>>>>>> 73c91422821cd516f483a1d0eed568276e92b3b9
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
<<<<<<< HEAD
	double util_r;
	for (int r = 1; r < R_MX; r++) {
		GRPS = setCoins(GRPS, len, &mstr); //group decide to follow/deviate
		mstr.flipCoin(); //master decides to check/not-check

		for (int i = 0; i < len; i++) {
			util_r = dy_computeUtil(GRPS[i].choice, mstr.choice, mstr.divergent);
			GRPS[i].util += util_r;
			mstr.util += util_r; 
			
			GRPS[i].pc = dy_updatePC(GRPS[i].choice, GRPS[i].pc, mstr.choice, mstr.divergent); //update group's PC
			printf("%d) PC %f| choice %d| util %f\n", i, GRPS[i].pc, GRPS[i].choice, GRPS[i].util);
			// += ss.str(GRPS[i].pc) + ( (i == len-1) ? "\n":",");
			compPC += GRPS[i].pc;
		}

		mstr.prob = dy_updatePA(mstr.choice, mstr.prob, mstr.divergent);
		printf("master) prob %f| choice %d| util %f\n", mstr.prob, mstr.choice, mstr.util);
		//create disturbance when all PCs converge to 0
		if (compPC == 0) {
			distCnt++;
			cycles.push_back(r - distAt); //record length of the cylce
			distAt = r;
		printf("\nReset #%d on round # %d| cycle: %d|\n", distCnt, r, cycles[distCnt-1]);
		}

		printf("compPC) %f\n", compPC);

=======

	for (int r = 1; r < R_MX; r++) {
		GRPS = setCoins(GRPS, &mstr); //group decide to follow/deviate
		mstr.flipCoin(); //master decides to check/not-check

		for (int i = 0; i < len; i++) {
			dy_computeUtil(&GRPS[i], &mstr);
			printf("%d) PC %f| choice %d| util %f\n", i, GRPS[i].pc, GRPS[i].choice, GRPS[i].util);

			dy_updatePC(&GRPS[i], &mstr); //update group's PC
			// += ss.str(GRPS[i].pc) + ( (i == len-1) ? "\n":",");
			pcComp += GRPS[i].pc;
		}

		dy_updatePA(&mstr);

		//create disturbance when all PCs converge to 0
		if (pcComp == 0) {
			distCnt++;
			cycles.push_back(r - distAt); //record length of the cylce
			distAt = r;
		}

		printf("\nReset #%d on round # %d| cycle: %d|\n", distCnt, r, cycles[distCnt-1]);
>>>>>>> 73c91422821cd516f483a1d0eed568276e92b3b9
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

<<<<<<< HEAD
double dy_updatePC(int choice, double pc, int mchoice, int divergent) {
	double tmp;
	if (mchoice) {
		if (choice == 1) { //unit follows
			tmp = (pc + alphaW * (aspiration - (WBY - WCT))) * choice;
		} else { //unit deviated 
			tmp = (pc - alphaW * (aspiration - WPC)) * choice;			
		}
	} else { //master does not check
		if (divergent > tou) {
			if (choice == 1) { //unit follows but not part of the majority
				tmp = (pc + alphaW * (aspiration + WCT)) * choice;
			} else { //unit diviates and part of the majority
				tmp = (pc + alphaW * (WBY - aspiration)) * choice;
			}
		} else {
			if (choice == 1) { //unit follows and is part of the majority
				tmp = (pc + alphaW * (aspiration - (WBY - WCT))) * choice;				
			} else { //unit deviates but is not part of the majority 
				tmp = (pc - alphaW * aspiration) * choice;
=======
void dy_updatePC(DroneUnit* unit, Master* mstr) {
	double tmp;
	if (mstr->choice) {
		if (unit->choice == 1) { //unit follows
			tmp = (unit->pc + alphaW * (aspiration - (WBY - WCT))) * unit->choice;
		} else { //unit deviated 
			tmp = (unit->pc - alphaW * (aspiration - WPC)) * unit->choice;			
		}
	} else { //master does not check
		if (mstr->divergent > tou) {
			if (unit->choice == 1) { //unit follows but not part of the majority
				tmp = (unit->pc + alphaW * (aspiration + WCT)) * unit->choice;
			} else { //unit diviates and part of the majority
				tmp = (unit->pc + alphaW * (WBY - aspiration)) * unit->choice;
			}
		} else {
			if (unit->choice == 1) { //unit follows and is part of the majority
				tmp = (unit->pc + alphaW * (aspiration - (WBY - WCT))) * unit->choice;				
			} else { //unit deviates but is not part of the majority 
				tmp = (unit->pc - alphaW * aspiration) * unit->choice;
>>>>>>> 73c91422821cd516f483a1d0eed568276e92b3b9
			}
		}
	}

<<<<<<< HEAD
	return max( 0.0, min(1.0, tmp) );
}

double dy_updatePA(int choice, double prob, int divergent) {
	if (choice) {
		double tmp = prob + alphaM * ( (divergent * 1.0) / n - tou);
		//printf("PA %f| alphaM %f| divergent %d| n %d| tou %d\n", mstr->prob, alphaM, mstr->divergent, n, tou);
		//printf("updating PA: %f\n", tmp);  
		return min( 1.0, max( prob, tmp ) );
	}
	return prob;
}

double dy_computeUtil(int choice, int mchoice, int divergent) {
	double tmp;
	if (mchoice) { //master checks
		if (choice == 1) { //unit follows, provide follwers reward
			tmp = WBY - WCT; 
		} else { //unit deviates, apply punishment 
			tmp = -WPC;
		}
	} else { //master does not check
		if (divergent > tou) { //majority deviated 
			if (choice == 1) { //unit follows
				tmp = -WCT;
			} else { //unit deviated
				tmp = WBY;
			}
		} else { //majority followed
			if (choice == 1) { //unit followed
				tmp = WBY - WCT;
			} else { //unit deviated 
				tmp = 0;
=======
	unit->pc = max( 0.0, min(1.0, tmp) );
}

void dy_updatePA(Master* mstr) {
	if (mstr->choice) {
		double tmp = mstr->prob + alphaM * ( (mstr->divergent * 1.0) / n - tou);
		//printf("PA %f| alphaM %f| divergent %d| n %d| tou %d\n", mstr->prob, alphaM, mstr->divergent, n, tou);
		//printf("updating PA: %f\n", tmp);  
		mstr->prob = min( 1.0, max( mstr->prob, tmp ) );
	}
}

void dy_computeUtil(DroneUnit* unit, Master* mstr) {
	if (mstr->choice) { //master checks
		if (unit->choice == 1) { //unit follows, provide follwers reward
			unit->util = WBY - WCT; 
		} else { //unit deviates, apply punishment 
			unit->util = -WPC;
		}
	} else { //master does not check
		if (mstr->divergent > tou) { //majority deviated 
			if (unit->choice == 1) { //unit follows
				unit->util = -WCT;
			} else { //unit deviated
				unit->util = WBY;
			}
		} else { //majority followed
			if (unit->choice == 1) { //unit followed
				unit->util = WBY - WCT;
			} else { //unit deviated 
				unit = 0;
>>>>>>> 73c91422821cd516f483a1d0eed568276e92b3b9
			}
		}
	}

<<<<<<< HEAD
	//mstr->util += unit.util * (-1); //units reward is negative towards the master's
	return tmp;
=======
	mstr->util += unit->util * (-1); //units reward is negative towards the master's
>>>>>>> 73c91422821cd516f483a1d0eed568276e92b3b9
}

