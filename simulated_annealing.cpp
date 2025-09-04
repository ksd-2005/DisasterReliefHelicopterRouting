#include <random>
#include <chrono>
#include <cmath>
#include <iostream>
#include "structures.h"
#include "neighbourhood.h"
// A helper function to compute objective for a solution.
// You need to update meta-data in each HelicopterPlan and Trip.
// For simplicity, we compute values on the fly here.
// double computeObjective(const Solution & sol, const ProblemData & data) {
//     double totalObjective = 0.0;
//     // For each helicopter plan:
//     for (const auto &plan : sol) {
//         // Get helicopter F and alpha; indices are 1-indexed.
//         const Helicopter &heli = data.helicopters[plan.helicopter_id - 1];
//         double planDeliveredValue = 0.0;
//         double planTripCost = 0.0;
//         // For each trip in the plan
//         for (const auto &trip : plan.trips) {
//             // Compute trip_distance starting from home city
//             const Point &home = data.cities[heli.home_city_id - 1];
//             double tripDistance = 0.0;
//             Point prev = home;
//             for (const auto &drop : trip.drops) {
//                 const Point &villagePoint = data.villages[drop.village_id - 1].coords;
//                 tripDistance += distance(prev, villagePoint);
//                 prev = villagePoint;
//             }
//             tripDistance += distance(prev, home); // return to home
//             double tripCost = heli.fixed_cost + heli.alpha * tripDistance;
//             planTripCost += tripCost;
            
//             // Compute delivered value using package values.
//             int totalDry = 0, totalPerc = 0, totalOther = 0;
//             for (const auto &drop : trip.drops) {
//                 totalDry += drop.dry_food;
//                 totalPerc += drop.perishable_food;
//                 totalOther += drop.other_supplies;
//             }
//             double deliveredValue = totalDry * data.packages[0].value +
//                                     totalPerc * data.packages[1].value +
//                                     totalOther * data.packages[2].value;
//             planDeliveredValue += deliveredValue;
//         }
//         totalObjective += (planDeliveredValue - planTripCost);
//     }
//     return totalObjective;
// }

// Simulated Annealing: maximize objective value.
double simulated_annealing(Solution &startState, const ProblemData & data) {
    // Initialize random generator.
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Settings: initial temperature and decay rate.
    double T0 = 1000.0;
    double T = T0;
    double decay_rate = 0.995;
    
    // Time-based stopping: run up to 90% of data.time_limit_minutes.
    using clock = std::chrono::steady_clock;
    auto t_start = clock::now();
    double minutes = data.time_limit_minutes;
    if (minutes <= 0.0) {
        // Safe fallback: if time limit is not set, run for 1 second.
        minutes = 1.0 / 60.0;
    }
    double allowed_seconds = minutes * 0.9 * 60.0; // 90% of minutes -> seconds
    auto t_end = t_start + std::chrono::duration<double>(allowed_seconds);

    // Current state and its objective.
    Solution currentState = startState;
    double currentObj = compute_objective(currentState, data, nullptr);
    // Best seen state.
    Solution bestState = currentState;
    double bestObj = currentObj;
    
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);

    // Time-driven loop (instead of fixed maxIterations)
    while (clock::now() < t_end) {
        // Generate a neighbour using your neighbourhood function.
        Solution neighbour = get_random_neighbor(currentState, data, currentObj);
        double neighbourObj = compute_objective(neighbour, data, nullptr);
        //cout << (neighbour == currentState) << endl;
        //if(!(neighbour == currentState)) std::cout<< neighbourObj<< std::endl;
        // If neighbour is better, accept.
        if (neighbourObj > currentObj) {
            currentState = neighbour;
            currentObj = neighbourObj;
            if (currentObj > bestObj) {
                bestState = currentState;
                bestObj = currentObj;
                //cout<<bestObj<<endl;
            }
            // cout<<"Best obj than current:"<<bestObj<<endl;
        } else {
            // Accept with probability exp((neighbourObj - currentObj)/T)
            double acceptanceProb = std::exp((neighbourObj - currentObj) / T);
            double randomVal = uniform_dist(gen);
            if (randomVal < acceptanceProb) {
                currentState = neighbour;
                currentObj = neighbourObj;
                 //cout<<"Took lower value:"<<currentObj<<endl;
            }
        }
        
        // Update temperature (per iteration)
        T *= decay_rate;
        if (T < 1e-6)
            T = 1e-6;
    }
    
    // Update startState with best solution found.
    startState = bestState;
    return bestObj;
}
