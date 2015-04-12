#include <stdlib.h>
#include <vector> // declared here for optimization reasons not completely cleare A.T.M

extern double dR, dP, dC;
double randFunction(); //returns random value between range (0,1)
class DroneUnit {
public:
	DroneUnit( double p , int sz);
	void flipCoin(); //unit randomly chooses to cheat or not, based on their cheating prob.
	void flipCoin(double); //unit flips coin on a specific probability
	int size; //unit site of droneUnit, unit can consist of 5 drones (size=5), or 1 drone(size=1)
	int choice; //set by flipCoin() module. -1 indicates cheat, 1 indicates unit is honest 
	double pc; //probability that influences how likely a unit cheats
	double util; //cumulative utility receieved by the master mechanism

};

std::vector<DroneUnit*> genSingletons(int, double); //generates a given # of units (all size=1)
std::vector<DroneUnit*> genRandColluders(int, double); //generate units until thier composite size equals the number specified