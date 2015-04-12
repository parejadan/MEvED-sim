#include "sim_struct.h"
#include <fstream>
#include <sstream>
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
		if (prob != -1.0)  {//flip coin on a specific probability
			GRPS[i]->flipCoin(prob);
			//printf("\nflipping bad\n");
		} else {//flip coin on drone's own unique probability
			GRPS[i]->flipCoin();
			//printf("r:%f\t|pc:%f\t|choice: %f\n", dR, dP, dC);
			//printf("\nflipping good\n");

		}
		if (GRPS[i]->choice == 1) { mstr->correct += GRPS[i]->size; }
		else { mstr->incorrect += GRPS[i]->size; }
	}
}

int main() {
	srand( time(NULL) );
	ostringstream mx_data, dy_data, tmp, readme;
	ofstream mx_file, dy_file, readme_dat;
	readme << "columns for both data files are: deviating pc, wba, round of convergence/detection\n";
	readme << "number of workers: " << n << "\ndeviator count: " << devCount <<"\n";
	readme << "if the 3rd column in either file is -1, that means the algorithm/mechanism failed to converge/detect";
	readme_dat.open("readme.txt"); readme_dat << readme.str(); readme_dat.close();

	len = n; //If not using singletons, this needs to change
	pv = 1.0 / sqrt(n);
	pc = 0.24;
	maxR = floor(2*n*pc*log(1/eps));
	Master dy_mstr(0.5);
	Master mx_mstr(pv);

	bool converged; //true when all units from evolutionary dynamics converge to pc=0
	bool detected; //true when units from mixed equilibria detected
	int conv = 0, dect = 0;
	for (double devPC = 0.5; devPC <= 1.0; devPC += 0.1) {
		vector<DroneUnit*> dy_GRPS = genSingletons(n, 0.0);
		vector<DroneUnit*> mx_GRPS = genSingletons(n, pc);

		for(int i = 0; i < devCount; i++) { //create deviators within workers
			dy_GRPS[i]->pc = devPC;
			mx_GRPS[i]->pc = devPC;
		}
		WBA = 0.1;
		for (double i = 0.1; WBA <= 2.0; WBA += i) {
			//reset parameters for simulations
			WPC = WBA/n;
			vector<int>().swap(counts);
			conv = 0, dect = 0;

			for (int rnd = 1; rnd <= END; rnd++) { //start repeated games
				if (conv == 0) { //simulating evolutaionry dynamics until convergence

					setCoins(dy_GRPS, &dy_mstr, len, -1.0);
					converged = doEvo(dy_GRPS, &dy_mstr, rnd);
					
					if (converged) {
						conv = rnd;
						//printf("devPC: %f\t| WBA: %f\t|conv: %d\n", devPC, WBA, conv);
						dy_data << devPC << "," << WBA << "," << rnd << "\n";
					} else if (rnd == END) { dy_data << devPC << "," << WBA << "," << -100 << "\n"; }
				}
				
				if (dect == 0) { //simulating mixed equilibria until detection
					
					setCoins(mx_GRPS, &mx_mstr, len, -1.0);
					detected = doMixed(rnd, &mx_mstr);
					if (detected) {
						dect = rnd;
						//printf("dev's PC: %f\t|# of deviators: %d\t| WBA: %f\t|detection at round: %d\n\n", devPC, (int)ceil(n/2.0), WBA, rnd);
						mx_data << devPC << "," << WBA << "," << rnd << "\n";
					} else if (rnd == END) { mx_data << devPC << "," << WBA << "," << -100 << "\n"; }
				}

				if (converged && detected) rnd = END+1;
			}
		}
		vector<DroneUnit*>().swap(dy_GRPS);
		vector<DroneUnit*>().swap(mx_GRPS);
	}

	mx_file.open("mixed-data.txt");
	mx_file << mx_data.str();
	mx_file.close();
	dy_file.open("dynamic-data.txt");
	dy_file << dy_data.str();
	dy_file.close();
}

////////////////////////////////////////////////////////////////////////////
/* !#MIX#! */
bool doMixed(int rnd, Master* mx_mstr) {
	counts.push_back(mx_mstr->incorrect); //record number of incorret results for round
	if (counts.size() > maxR) counts.erase(counts.begin() + 1); //remove first element from vector

	minC = n;
	maxC = 0;
	int R = min(maxR, rnd);
	for (int r = 1; r <= R; r++) {
		if ( counts[R-r] < minC ) minC = counts[R-r];
		if ( counts[R-r] > maxC ) maxC = counts[R-r];

		delta = sqrt( 3 * log(1.0/eps) / (r*n*pc) );

		if ( minC >= ceil( (1+delta)*n*pc ) || maxC <= floor( (1-delta)*n*pc ) )
			return true;
	}
	return false;
}

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
bool doEvo(vector<DroneUnit*> GRPS, Master* mstr, int rnd) {
	double compPC = 0;
	//printf("round: %d\t|", rnd);
	for (int i = 0; i < len; i++) {
		dy_computeUtil(GRPS[i], mstr, WBA*pc);
		//printf("pc-%d: %f|\t", i, GRPS[i]->pc );
		dy_updatePC(GRPS[i], mstr, WBA*pc); //update unit's PC
		compPC += GRPS[i]->pc;
	}
	//printf("\n");
	dy_updatePA(mstr);
	if (compPC == 0) { //check if units converged
		return true;
	}
	return false;
}

void dy_updatePC(DroneUnit* drn, Master* mstr, double wpc) {
	double tmp;
	if (mstr->choice) {
		if (drn->choice == 1) { //unit was honest
			tmp = drn->pc + alphaW * (aspiration - (WBA - WCT));
		} else { //unit cheated
			tmp = drn->pc - alphaW * (aspiration - wpc);			
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

void dy_computeUtil(DroneUnit* drn, Master* mstr, double wpc) {
	double tmp;
	if (mstr->choice) { //master checks
		if (drn->choice == 1) { //apply honest reward
			tmp = WBA - WCT; 
		} else { //apply cheater's reward
			tmp = -wpc;
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