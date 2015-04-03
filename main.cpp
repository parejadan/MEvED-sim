#include "sim_struct.h"
#include <fstream>
#include <ctime> //produce random seeds for srand & rand

using namespace std;

Master::Master(double p) { prob = p; util = 0; correct = 0; incorrect = 0; }

void Master::flipCoin() {
	if (randFunction() < prob) { choice = 1; } //master checks
	else { choice = 0; } //master doesn't check
}

void setCoins(vector< DroneUnit* > GRPS, Master* mstr, int len, double prob) {
	mstr->correct = 0; mstr->incorrect = 0;
	for (int i = 0; i < len; i++) {
		if (prob != -1.0) //flip coin on a specific probability
			GRPS[i]->flipCoin(prob);
		else //flip coin on drone's own unique probability
			GRPS[i]->flipCoin();

		if (GRPS[i]->choice == 1) { mstr->correct += GRPS[i]->size; }
		else { mstr->incorrect += GRPS[i]->size; }
	}
}

int main() {
	srand( time(NULL) );

	//evolutionary setup for simulation
	Master dy_mstr(0.5);
	vector<DroneUnit*> dy_GRPS = genSingletons(n, 0.5);
	bool converged; //true when all units from evolutionary dynamics converge to pc=0
	
	//mixed equilibria setup for simulation
	Master mx_mstr(pv);
	vector<DroneUnit*> mx_GRPS = genSingletons(n, 0.5);
	bool detected; //true when units from mixed equilibria detect

	len = mx_GRPS.size(); //If not generating singletons, this needs to change
	int rsetLim = ceil(len/2.0); //number of units to reset pc=1
	for (int rnd = 1; rnd <= END; rnd++) {
		//simulating evolutaionry dynamics 
		setCoins(dy_GRPS, &dy_mstr, len, -1.0); //reinforcement model coinflip, all units have a unique pc
		converged = doEvo(dy_GRPS, &dy_mstr, rnd, rsetLim);

		//simulating mixed equilibria repeated games
		setCoins(mx_GRPS, &mx_mstr, len, -1.0); //mixed equilibria coin flip, all units have a standard pc
		counts.push_back(mx_mstr.incorrect); //record number of incorret results for round

		for (int i = 1; i <= rnd; i++) {
			for (int r = 1; r <= rnd - i + 1; r++) {
				delta = sqrt( log(n) / (r*n*pc) );
				minC = counts[i - 1];
				maxC = counts[i - 1];

				//printf("\nrnd: %d\t|rnd-i+1: %d\t|minDelta: %d\n",rnd, delta, rnd - i + 1);
				for (int j = 1; j <= r - 1; j++) {
					if ( counts[i + j - 1] < minC ) minC = counts[i + j - 1];
					if ( counts[i + j - 1] > maxC ) maxC = counts[i + j - 1]; 
				}

				if (minC >= (1 + delta) * n * pc || maxC <= (1 - delta) * n * pc)
					detected = true;
			}
		}

		//checking convergence and dectection from mixed and dynamic algorithms
		if (converged) {
			//printf("\n>round length: %d\n", rnd);
			converged = false;
			//exit (EXIT_FAILURE);
			printf("\n> evo units converged at: %d\t|len: %d\n", rnd, rnd - c_prev);
			c_prev = rnd;
		}
		if (detected) {
			///printf("\n>round length: %d\n", rnd);
			detected = false;
			//exit (EXIT_FAILURE);
			printf("\n> mixed units detected at: %d\t|len: %d\n", rnd, rnd - d_prev);
			d_prev = rnd;
		}

		if (converged && detected)  exit (EXIT_FAILURE);

	}
}

////////////////////////////////////////////////////////////////////////////
/* !#MIX#! */
void mx_compteUtil(DroneUnit* drn, Master* mstr) {
	double util;
	if ( drn->choice == -1 ) { //cheated
		if ( mstr->incorrect > tou ) { //utility (1)
			util = (1 - mstr->prob) * WBA - mstr->prob * WPC * mstr->incorrect;
		} else { //utility (3)
			util = - mstr->prob * WPC * mstr->incorrect;
		}
	} else {
		if ( mstr->incorrect > tou ) { //utility (2)
			util = mstr->prob * WBA - WCT;
		} else { //utility (4)
			util = WBA - WCT;
		}
	}
	drn->util = util;
}

/////////////////////////////////////////////////////////////////////////////
/* !#EVO#! */
bool doEvo(vector<DroneUnit*> GRPS, Master* mstr, int rnd, int resetLim) {
	double compPC = 0;
	for (int i = 0; i < len; i++) {
		dy_computeUtil(GRPS[i], mstr);
		dy_updatePC(GRPS[i], mstr); //update unit's PC
		//printf("> pc-%d: %f|", i, GRPS[i]->pc );
		compPC += GRPS[i]->pc;
	}
//	printf("rnd: ","\n");
	dy_updatePA(mstr);
	if (compPC == 0) { //check if units converged
		for (int i = 0; i < ceil(resetLim); i++) { GRPS[i]->pc = 1; }//randFunction();
		return true;
	}
	return false;
}

void dy_updatePC(DroneUnit* drn, Master* mstr) {
	double tmp;
	if (mstr->choice) {
		if (drn->choice == 1) { //unit was honest
			tmp = drn->pc + alphaW * (aspiration - (WBA - WCT));
		} else { //unit cheated
			tmp = drn->pc - alphaW * (aspiration - WPC);			
		}
	} else { //master does not check
		if (mstr->incorrect > tou) {
			if (drn->choice == 1) { //unit was honest but not part of the majority
				tmp = drn->pc + alphaW * (aspiration + WCT);
			} else { //unit cheated and part of the majority
				tmp = drn->pc + alphaW * (WBA - aspiration);
			}
		} else {
			if (drn->choice == 1) { //unit was honest and is part of the majority
				tmp = drn->pc + alphaW * (aspiration - (WBA - WCT));				
			} else { //unit cheated but is not part of the majority 
				tmp = drn->pc - alphaW * aspiration;
			}
		}
	}
	//printf("\ttmp: %f|\talpha: %f|\t|ai: %f|\tWBA: %f|\tWCT: %f|\t prv-P:%f\n", tmp, alphaW, aspiration, WBA, WCT, drn->pc);
	drn->pc = max( 0.0, min(1.0, tmp) );
}

void dy_updatePA(Master* mstr) {
	if (mstr->choice) {
		double tmp = mstr->prob + alphaM * ( (mstr->incorrect * 1.0) / n - tou);
		mstr->prob = min( 1.0, max( mstr->prob, tmp ) );
	}
}

void dy_computeUtil(DroneUnit* drn, Master* mstr) {
	double tmp;
	if (mstr->choice) { //master checks
		if (drn->choice == 1) { //apply honest reward
			tmp = WBA - WCT; 
		} else { //apply cheater's reward
			tmp = -WPC;
		}
	} else { //master does not check
		if (mstr->incorrect > tou) { //majority cheated 
			if (drn->choice == 1) { //apply honest reward
				tmp = -WCT;
			} else { //apply cheater's reward
				tmp = WBA;
			}
		} else { //majority were honest
			if (drn->choice == 1) { //apply honest reward
				tmp = WBA - WCT;
			} else { //apply cheater's reward
				tmp = 0;
			}
		}
	}
	mstr->util += tmp; drn->util += tmp;
}