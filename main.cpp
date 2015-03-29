#include "sim_struct.h"
#include <fstream>
//#include <sstream>
#include <ctime> //produce random seeds for srand & rand

using namespace std;

Master::Master(double p) { prob = p; util = 0; correct = 0; }

void Master::flipCoin() {
	if (randFunction() < prob) { //master checks
		choice = 1;
	} else { //master doesn't check
		choice = 0;
	}
}

void setCoins(vector< DroneUnit* > GRPS, Master* mstr, int len, double prob) {
	int followers = 0;
	int divergent = 0;
	for (int i = 0; i < len; i++) {
		if (prob != -1.0) //flip coin on a specific probability
			GRPS[i]->flipCoin(prob);
		else //flip coin on drone's own unique probability
			GRPS[i]->flipCoin();

		if (GRPS[i]->choice == 1) { followers += GRPS[i]->size; }
		else { divergent += GRPS[i]->size; }
	}

	mstr->followers = followers;
	mstr->divergent = divergent;
	//printf("\n>followers: %d| divergent: %d\n", followers, divergent);
}

int main() {
	srand( time(NULL) );

	Master mstr(pv); //create masster with pv=0.5
	vector<DroneUnit*> GRPS = genSingletons(n, pc); //generate groups (drones) with pc=1
	int len = GRPS.size();
	//compLearningRate(); //set a learning rate for groups

	for (int rnd = 1; rnd <= END; rnd++) {
		setCoins(GRPS, &mstr, len, pc); //flip worker coins; master recieves results
		counts.push_back(mstr.divergent); //record number of incorret results for round

		for (int i = 1; i <= rnd; i++) {
			for (int r = 1; r <= rnd - i + 1; r++) {
				minDelta = counts[i - 1];

				for (int j = 1; j <= r - 1; j++) if (counts[i + j - 1] < minDelta) { minDelta = counts[i + j - 1]; }

				if (r * pow(minDelta, 2.0) > log(n) / (n * pc)) { pc = 1 ; }
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////
/* !#MIX#! */

void mx_compteUtil(DroneUnit* drn, Master* mstr) {
	double util;
	if ( drn->choice == -1 ) { //deviated
		if ( mstr->divergent > tou ) { //utility (1)
			util = (1 - mstr->prob) * WBA - mstr->prob * WPC * mstr->divergent;
		} else { //utility (3)
			util = - mstr->prob * WPC * mstr->divergent;
		}
	} else {
		if ( mstr->divergent > tou ) { //utility (2)
			util = mstr->prob * WBA - WCT;
		} else { //utility (4)
			util = WBA - WCT;
		}
	}

	drn->util = util;
}

//util for single round computations 
/*void mx_compteUtil(DroneUnit* drn, Master* mstr) {
	double utilD, utilF;
	if (mstr->choice) {
		if (mstr->divergent == n) {
			utilD = - WPC;
			utilF = 0;
		} else {
			utilD = - WPC;
			utilF = WBA - WCT / drn->size;
		}
	} else {
		if ( mstr->divergent > tou ) {
			switch(MODEL) {
				case 1: //Rm
					utilD = WBA;
					utilF = - WCT / drn->size;
					break;
				case 2: //Ra
					utilD = WBA;
					utilF = WBA - WCT / drn->size;
					break;
				default:
					utilD = 0;
					utilF = - WCT / drn->size;
			}
		} else {
			switch(MODEL) {
				case 1: //Rm
					utilD = 0;
					utilF = WBA - WCT / drn->size;
					break;
				case 2: //Ra
					utilD = WBA;
					utilF = WBA - WCT / drn->size;
					break;
				default: //Rn
					utilD = 0;
					utilF = - WCT / drn->size;
			}
		}
	}

	if (drn->choice == 1)
		drn->util = utilF;
	else
		drn->util = utilD;
}
void mx_masterUtil(Master* mstr) {
	if (mstr->choice) { //master checks
		if (mstr->divergent == n) { //everyone diviated
			mstr->util = - MPW - MCV + n * WPC;
		} else { //everyone followed
			mstr->correct++;
			mstr->util = MBR - MCV - (n - mstr->divergent) * (*MCA) + mstr->divergent * WPC;
		}
	} else { //master does not check
		if ( mstr->divergent > tou ) { //majority deviated
			switch(MODEL) {
				case 1: //Rm
					mstr->util = - MPW - mstr->divergent * (*MCA);
					break;
				case 2: //Ra
					mstr->util = - MPW - n * (*MCA);
					break;
				default: //Rn
					mstr->util = - MPW;
			}
		} else { //majority followed
			mstr->correct++;
			switch(MODEL) {
				case 1: //Rm
					mstr->util = MBR - (n - mstr->divergent) * (*MCA);
					break;
				case 2: //Ra
					mstr->util = MBR - n * (*MCA);
					break;
				default: //Rn
					mstr->util = MBR;
			}
		}
	}
}*.

/////////////////////////////////////////////////////////////////////////////
/* !#EVO#! */
void doEvo(vector<DroneUnit*> GRPS, Master* mstr, int r, int len) {
	setCoins(GRPS, mstr, len, -1.0); //group decide to follow/deviate
	mstr->flipCoin(); //master decides to check/not-check
	for (int i = 0; i < len; i++) {
		dy_computeUtil(GRPS[i], mstr);
		dy_updatePC(GRPS[i], mstr); //update group's PC
		printf("%d) PC %f| choice %d| util %f\n", i, GRPS[i]->pc, GRPS[i]->choice, GRPS[i]->util);
		compPC += GRPS[i]->pc;
	}
	dy_updatePA(mstr);
	printf("master) prob %f| choice %d| util %f\n", mstr->prob, mstr->choice, mstr->util);
	//create disturbance when all PCs converge to 0
	if (compPC == 0) {
		distCnt++;
		cycles.push_back(r - distAt); //record length of the cylce
		distAt = r;
	printf("\nReset #%d on round # %d| cycle: %d|\n", distCnt, r, cycles[distCnt-1]);
	}
	printf("compPC) %f\n", compPC);
	for (int i = 0; i < len; i++) {
		GRPS[i]->pc = randFunction(); //reset PC randomly
		printf("PC %f|%s", GRPS[i]->pc, ( (i == len-1) ? "\n":"") );
	}
}

void compLearningRate() { alphaW = randFunction() * (1 / (aspiration + WPC) ); }

void dy_updatePC(DroneUnit* drn, Master* mstr) {
	double tmp;
	if (mstr->choice) {
		if (drn->choice == 1) { //unit follows
			tmp = (drn->pc + alphaW * (aspiration - (WBA - WCT))) * drn->choice;
		} else { //unit deviated 
			tmp = (drn->pc - alphaW * (aspiration - WPC)) * drn->choice;			
		}
	} else { //master does not check
		if (mstr->divergent > tou) {
			if (drn->choice == 1) { //unit follows but not part of the majority
				tmp = (drn->pc + alphaW * (aspiration + WCT)) * drn->choice;
			} else { //unit diviates and part of the majority
				tmp = (drn->pc + alphaW * (WBA - aspiration)) * drn->choice;
			}
		} else {
			if (drn->choice == 1) { //unit follows and is part of the majority
				tmp = (drn->pc + alphaW * (aspiration - (WBA - WCT))) * drn->choice;				
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
			tmp = WBA - WCT; 
		} else { //unit deviates, apply punishment 
			tmp = -WPC;
		}
	} else { //master does not check
		if (mstr->divergent > tou) { //majority deviated 
			if (drn->choice == 1) { //unit follows
				tmp = -WCT;
			} else { //unit deviated
				tmp = WBA;
			}
		} else { //majority followed
			if (drn->choice == 1) { //unit followed
				tmp = WBA - WCT;
			} else { //unit deviated 
				tmp = 0;
			}
		}
	}

	//mstr->util += unit.util * (-1); //units reward is negative towards the master's
	mstr->util += tmp;
	drn->util += tmp;
}