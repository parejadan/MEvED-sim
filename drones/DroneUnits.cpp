#include "DroneUnits.h"


double randFunction() { return (double)rand() / RAND_MAX; }

double initPC = 0.5, initUtil = 0.0;  //drones default pc value and utility
DroneUnit::DroneUnit(double p, double u) { pc = p; util = u; } //constructor

void DroneUnit::flipCoin() {
	double r = randFunction();
	if (r < pc) { choice = -1; } //unit cheated
	else { choice = 1; } //unit is honest
}

std::vector<DroneUnit> genSingletons(int limit) {
	DroneUnit grp(initPC, initUtil);
	std::vector<DroneUnit> GRPS;

	for (int i = 0; i < limit; i++) {
		grp.size = 1;
		GRPS.push_back(grp);
	}

	return GRPS;
}

std::vector<DroneUnit> genRandColluders(int limit) {
	DroneUnit grp (initPC, initUtil);
	std::vector<DroneUnit> GRPS;

	int size, count = 0;
	while (count < limit) {
		size = (int)(randFunction() * limit);

		if (size + count <= limit && size != 0) {
			grp.size = size;
			GRPS.push_back(grp);
			count += size;
		}
		if ((size + count > limit) && (limit - count != 0)) {
			grp.size = limit - count;
			GRPS.push_back(grp);
			count += grp.size;
		}
	}

	return GRPS;
}