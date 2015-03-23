#include <stdlib.h>
#include <vector> // declared here for optimization reasons not completely cleare A.T.M

double randFunction(); //returns random value between range (0,1)
class DroneUnit {
public:
	DroneUnit( double p); //constructor for pc
	DroneUnit( double p , int sz);
	void flipCoin(); //unit randomly chooses to cheat or not, based on their cheating prob.
	int size; //unit site of droneUnit, unit can consist of 5 drones (size=5), or 1 drone(size=1)
	int choice; //set by flipCoin() module. -1 indicates cheat, 1 indicates unit is honest 
	double pc; //probability that influences how likely a unit cheats
	double util; //cumulative utility receieved by the master mechanism

};

// Generates a given number of drone units (size=1 for all)
std::vector<DroneUnit*> genSingletons(int, double);
// Generate droneunits until the composite size for all created equals the number specified
std::vector<DroneUnit*> genRandColluders(int, double);