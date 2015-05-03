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
	ostringstream data;
	ofstream dat_file;

	len = n; //If not using singletons, this needs to change
	pc = 0.1;
	maxR = floor(3*n*pc*log(1/eps));
	Master dy_mstr( 0.5 );
	Master mx_mstr(0.17);
	Master os_mstr( (WBA + 0.1) / (3* WBA) + 0.01 );

	bool converged, detected;
	int conv = 0, dect = 0;
	for (double devPC = 0.5; devPC <= 1.0; devPC += 0.1) {

		for (double inc = 0.1, wba = WBA; wba <= 2.1; wba += inc) {
			//start with a fresh batch of workers for iteration
			vector<DroneUnit*> dy_GRPS = genSingletons(n, 0.0);
			vector<DroneUnit*> mx_GRPS = genSingletons(n, pc);
			vector<DroneUnit*> os_GRPS = genSingletons(n, 0.0);

			//create deviators within workers
			for(int i = 0; i < devCount; i++) {
				dy_GRPS[i]->pc = devPC; mx_GRPS[i]->pc = devPC; os_GRPS[i]->pc = devPC;
			}

			//reset parameters for simulations
			MPW = n*WBA; //master punishment for accepting a wrong answer
			MCV = n*WBA; //master cost for verifying 
			MBR = n*WBA;	//master benefit for accepting a right answer
			vector<int>().swap(counts);
			conv = 0, dect = 0;

			/** BEGIN REPEATED GAMES **/
			for (int rnd = 1; rnd <= END; rnd++) {
				//if (dect == 0) { //simulating mixed equilibria until detection
				setCoins(mx_GRPS, &mx_mstr, len, -1.0);
                if (mx_mstr.incorrect < n) detected = doMixed(rnd, wba, wba/(n*pc), &mx_mstr, mx_GRPS);
				else detected = doMixed(rnd, wba, 0, &mx_mstr, mx_GRPS);
				//printf("\n");
                if (detected){
                    if(dect == 0){      // if this is the first time detected
                        dect = rnd;     // mark the round of detection
                        pc = 1;         // punish for one round
                    }
                    else{                                   // false detection
                        pc = 1;                             // punish for one round
                        for(int i = 0; i < devCount; i++) { // false detection -> all punish
                            mx_GRPS[i]->pc = pc;
                        }
                    }
                }
                else{
                    if (pc == 1){                           // there was a punishment
                        pc = 0.1;                           // stop punishment (0.1 should be constant)
                        for(int i = 0; i < devCount; i++) { // make all workers followers
                            mx_GRPS[i]->pc = pc;
                        }
                    }
                    if (rnd == END) {               // if this is the last round and it was not detected
                        dect = rnd+1;               // mark round of detection = infinity
                    }
                }
				//}
				
				//printf("%f\n", dy_mstr.prob);
				//if (conv == 0) { //simulating evolutaionry dynamics until convergence

                setCoins(dy_GRPS, &dy_mstr, len, -1.0);
                converged = doEvo(dy_GRPS, &dy_mstr, rnd, wba, wba);
					
                if (conv == 0){
                    if (converged) {       // if this is the first convergence
                        conv = rnd;        // mark the round of convergence
                    }
					else if (rnd == END) {     // if this is the last round and it did not converge
                        conv = rnd+1;          // mark round of convergence = infinity
                    }
                }
                else if (!converged){         // diverged after convergence
                        printf("dy OSCILLATING\n");
                }
				//}


				//simulate one shot computations
				for (int i = 0; i < devCount; i++)
					dy_wrkUtil(os_GRPS[i], &os_mstr, wba, wba);
				os_mstrUtil(&os_mstr, wba);
			}
			/** END REPEATED GAMES **/

			//record data from simulation
			data << devPC << "," << wba << ","; //x and y values
			data << conv << "," << dy_GRPS[len-1]->util/END << "," << dy_mstr.util/END << ","; //evol data
			data << dect << "," << mx_GRPS[len-1]->util/END << "," << mx_mstr.util/END << ","; //mixed data
			data << os_GRPS[len-1]->util/END << "," << os_mstr.util/END << "\n"; //one-shot data


			//dumps all worker objects to free up memory
			vector<DroneUnit*>().swap(dy_GRPS);
			vector<DroneUnit*>().swap(mx_GRPS);
			vector<DroneUnit*>().swap(os_GRPS);
		}
	}

	dat_file.open("data.csv");
	dat_file << "devP,wba,convergance,dy_wrkr_util,dy_mstr_util,detection,mx_wrkr_util,mx_mstr_util,os_wrkr_util,os_mstr_util\n";
	dat_file << data.str();
	dat_file.close();
}

////////////////////////////////////////////////////////////////////////////
/* !#MIX#! */
bool doMixed(int rnd, double wba, double wpc, Master* mx_mstr, vector<DroneUnit*> GRPS) {
	//compute utilities
	for(int i = 0; i < len; i++)
		mx_wrkUtil(GRPS[i], mx_mstr, wba, wpc);
	mx_mstrUtil(mx_mstr, wpc);

	//check for deviators
	counts.push_back(mx_mstr->incorrect); //record number of incorret results for round
	if (counts.size() > maxR) counts.erase(counts.begin() + 1); //remove first element from vector

	minC = n; maxC = 0;
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

void mx_wrkUtil(DroneUnit* drn, Master* mstr, double wba, double wpc) {
	double util;
	if ( drn->choice == -1 ) { //cheated
		if ( mstr->incorrect > (int) (n / 2.0) ) { //utility (1)
			util += (1 - mstr->prob) * wba - mstr->prob * wpc * mstr->incorrect;
		} else { //utility (3)
			util += - mstr->prob * wpc * mstr->incorrect;
		}
	} else {
		if ( mstr->incorrect > (int) (n / 2.0) ) { //utility (2)
			util += mstr->prob * wba - WCT;
		} else { //utility (4)
			util += wba - WCT;
		}
	}
	drn->util += util;
	//printf("%f\t|", drn->util);
}

void mx_mstrUtil(Master* mstr, double wpc) {
	if ( mstr->choice ) { //verified
		if ( mstr->incorrect < n ) { //utility (1)
			mstr->util += MBR - MCV - (n - mstr->incorrect) * (*MCA) + pow(mstr->incorrect, 2.0) * wpc;
		} else if ( mstr->incorrect == n ) { //utility (2)
			mstr->util += -  MCV + pow(n, 2.0) * wpc;
		}
	} else { //did not verified
		if ( mstr->incorrect > (int) (n / 2.0) ) { //utility (3)
			mstr->util += - MPW - mstr->incorrect * (*MCA);
		} else { //utility (4)
			mstr->util += MBR - (n - mstr->incorrect) * (*MCA);
		}
	}
}

void os_mstrUtil(Master* mstr, double wpc) {
	if ( mstr->choice ) { //verified
		if ( mstr->incorrect < n ) { //utility (1)
			mstr->util += MBR - MCV - (n - mstr->incorrect) * (*MCA) + mstr->incorrect * wpc;
		} else if ( mstr->incorrect == n ) { //utility (2)
			mstr->util += -  MCV + pow(n, 2.0) * wpc;
		}
	} else { //did not verified
		if ( mstr->incorrect > (int) (n / 2.0) ) { //utility (3)
			mstr->util += - MPW - mstr->incorrect * (*MCA);
		} else { //utility (4)
			mstr->util += MBR - (n - mstr->incorrect) * (*MCA);
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
/* !#EVO#! */
bool doEvo(vector<DroneUnit*> GRPS, Master* mstr, int rnd, double wba, double wpc) {
	//compute utilities
	double compPC = 0;
	//printf("round: %d\t|", rnd);
	for (int i = 0; i < len; i++) {
		dy_wrkUtil(GRPS[i], mstr, wba, wpc);
		//printf("pc-%d: %f|\t", i, GRPS[i]->pc );
		dy_updatePC(GRPS[i], mstr, wba, wba); //update unit's PC
		compPC += GRPS[i]->pc;
	}
	//printf("\n");

	//update coins
	dy_mstrUtil(mstr, wba, wpc);
	dy_updatePA(mstr);
	if (compPC <= 0.01) { //check if units converged
		return true;
	}
	return false;
}

void dy_updatePC(DroneUnit* drn, Master* mstr, double wpc, double wba) {
	double tmp;
	if (mstr->choice) {
		if (drn->choice == 1) { //unit was honest
			tmp = drn->pc + alphaW * (aspiration - (wba - WCT));
		} else { //unit cheated
			tmp = drn->pc - alphaW * (aspiration + wpc);   // MIGUEL: fixed to PLUS wpc  **********************		
		}
	} else { //master does not check
		if (mstr->incorrect > (int) (n / 2.0)) {
			if (drn->choice == 1) { //unit was honest but not part of the majority
				tmp = drn->pc + alphaW * (aspiration + WCT);
			} else { //unit cheated and part of the majority
				tmp = drn->pc + alphaW * (wba - aspiration);
			}
		} else {
			if (drn->choice == 1) { //unit was honest and is part of the majority
				tmp = drn->pc + alphaW * (aspiration - (wba - WCT));				
			} else { //unit cheated but is not part of the majority 
				tmp = drn->pc - alphaW * aspiration;
			}
		}
	}
	//printf("\ttmp: %f|\talpha: %f|\t|ai: %f|\twba: %f|\tWCT: %f|\t prv_P:%f\n", tmp, alphaW, aspiration, wba, WCT, drn->pc);
	drn->pc = max( 0.0, min(1.0, tmp) );
}

void dy_updatePA(Master* mstr) {
	if (mstr->choice) {
		double tmp = mstr->prob + alphaM * ( mstr->incorrect*1.0/ n - tou);
		//printf("pv-%f\n", tmp);
		//printf("%d-cheated|\t%f-tou|\t%f\n", mstr->incorrect, tou, (mstr->incorrect*1.0/ n) - tou);
		mstr->prob = min( 1.0, max( 0.01, tmp ) );
	}
}

void dy_wrkUtil(DroneUnit* drn, Master* mstr, double wba, double wpc) {
	//utility from table 2 on the draft
	if ( drn->choice == -1 ) { //cheated
		if ( mstr->incorrect > (int) (n / 2.0) ) { //utility (1)
			drn->util += (1 - mstr->prob) * wba - mstr->prob * wpc;
		} else { //utility (3)
			drn->util += - mstr->prob * wpc;
		}
	} else {
		if ( mstr->incorrect > (int) (n / 2.0) ) { //utility (2)
			drn->util += mstr->prob * wba - WCT;
		} else { //utility (4)
			drn->util += wba - WCT;
		}
	}
}

void dy_mstrUtil(Master* mstr, double wba, double wpc) {
	if ( mstr->choice ) { //verified
		if ( mstr->incorrect < n ) { //utility (1)
			mstr->util += MBR - MCV - (n - mstr->incorrect) * (*MCA) + mstr->incorrect * wpc;
		} else if ( mstr->incorrect == n) {  //utility (2)
			mstr->util += MBR - MCV;
		}
	} else { //did not verify
		if ( mstr->incorrect > (int) (n / 2.0) ) {  //utility (3)
			mstr->util += - MPW - mstr->incorrect * (*MCA);
		} else { //utility (4)
			mstr->util += - MBR - (n - mstr->incorrect) * (*MCA);
		}
	}
}