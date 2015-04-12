#include "DroneUnits.h"

double dR, dP, dC;
double randFunction() { return (double)rand() / RAND_MAX; }

DroneUnit::DroneUnit(double p, int sz) {
	pc = p; util = 0;
	choice = 0; size = sz;
}

void DroneUnit::flipCoin() {
	double r = randFunction();
	if (r < pc) { choice = -1; } //unit cheated
	else { choice = 1; } //unit computed
	dR = r; dP = pc; dC = choice;
}

void DroneUnit::flipCoin(double p) { //overload drone unit's pc with a specific pc
 	if (randFunction() < p) { choice = -1; } //unit cheated
	else { choice = 1; } //unit computed	
}

std::vector<DroneUnit*> genSingletons(int limit, double initPC) {
	std::vector<DroneUnit*> GRPS;

	for (int i = 0; i < limit; i++)
		GRPS.push_back(new DroneUnit(initPC, 1));

	return GRPS;
}

std::vector<DroneUnit*> genRandColluders(int limit, double initPC) {
	std::vector<DroneUnit*> GRPS;

	int size, count = 0;
	while (count < limit) {
		size = (int)(randFunction() * limit);

		if (size + count <= limit && size != 0) {
			GRPS.push_back(new DroneUnit(initPC, size));
			count += size;
		}
		if ((size + count > limit) && (limit - count != 0)) {
			GRPS.push_back(new DroneUnit(initPC, limit - count));
			count += GRPS.back()->size;
		}
	}

	return GRPS;
}