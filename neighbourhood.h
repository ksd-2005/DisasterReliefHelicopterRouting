#ifndef NEIGHBOURHOOD_H
#define NEIGHBOURHOOD_H

// #include <string>
 #include "structures.h"



Solution get_random_neighbor(const Solution &HelicopterPlans,const ProblemData &problemData, double current_obj);
Solution get_best_neighbor(const Solution &HelicopterPlans,const ProblemData &problemData, double current_obj);
double compute_objective(const Solution &HelicopterPlans, const ProblemData &data, bool *is_valid_ptr);
#endif