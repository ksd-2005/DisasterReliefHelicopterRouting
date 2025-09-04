#include <random>
#include <chrono>
#include <cmath>
#include "structures.h"
#include "neighbourhood.h"

// A helper function to compute objective for a solution.
// You need to update meta-data in each HelicopterPlan and Trip.
// For simplicity, we compute values on the fly here.
double computeObjective(const Solution & sol, const ProblemData & data) {
    double totalObjective = 0.0;
    // For each helicopter plan:
    for (const auto &plan : sol) {
        // Get helicopter F and alpha; indices are 1-indexed.
        const Helicopter &heli = data.helicopters[plan.helicopter_id - 1];
        double planDeliveredValue = 0.0;
        double planTripCost = 0.0;
        // For each trip in the plan
        for (const auto &trip : plan.trips) {
            // Compute trip_distance starting from home city
            const Point &home = data.cities[heli.home_city_id - 1];
            double tripDistance = 0.0;
            Point prev = home;
            for (const auto &drop : trip.drops) {
                const Point &villagePoint = data.villages[drop.village_id - 1].coords;
                tripDistance += distance(prev, villagePoint);
                prev = villagePoint;
            }
            tripDistance += distance(prev, home); // return to home
            double tripCost = heli.fixed_cost + heli.alpha * tripDistance;
            planTripCost += tripCost;
            
            // Compute delivered value using package values.
            int totalDry = 0, totalPerc = 0, totalOther = 0;
            for (const auto &drop : trip.drops) {
                totalDry += drop.dry_food;
                totalPerc += drop.perishable_food;
                totalOther += drop.other_supplies;
            }
            double deliveredValue = totalDry * data.packages[0].value +
                                    totalPerc * data.packages[1].value +
                                    totalOther * data.packages[2].value;
            planDeliveredValue += deliveredValue;
        }
        totalObjective += (planDeliveredValue - planTripCost);
    }
    return totalObjective;
}

// Simulated Annealing: maximize objective value.
long long simulated_annealing(Solution &startState, const ProblemData & data) {
    // Initialize random generator.
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Settings: initial temperature, decay rate, max iterations.
    double T0 = 1000.0;
    double T = T0;
    double decay_rate = 0.995;
    int maxIterations = 10000;
    
    // Current state and its objective.
    Solution currentState = startState;
    double currentObj = computeObjective(currentState, data);
    // Best seen state.
    Solution bestState = currentState;
    double bestObj = currentObj;
    
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
    
    for (int iter = 0; iter < maxIterations; iter++) {
        // Generate a neighbour. You can choose any neighbourhood function.
        // For example, use move_visit neighbourhood.
        Solution neighbour = get_best_neighbor(currentState, data, currentObj);
        double neighbourObj = computeObjective(neighbour, data);
        
        // If neighbour is better, accept.
        if (neighbourObj > currentObj) {
            currentState = neighbour;
            currentObj = neighbourObj;
            if (currentObj > bestObj) {
                bestState = currentState;
                bestObj = currentObj;
            }
        } else {
            // Accept with probability exp((neighbourObj - currentObj)/T)
            double acceptanceProb = exp((neighbourObj - currentObj) / T);
            double randomVal = uniform_dist(gen);
            if (randomVal < acceptanceProb) {
                currentState = neighbour;
                currentObj = neighbourObj;
            }
        }
        
        // Update temperature
        T *= decay_rate;
        if (T < 1e-6)
            T = 1e-6;
    }
    
    // Optionally, update startState with best solution.
    startState = bestState;
    
    // You may return best objective value in some integer form.
    return static_cast<long long>(bestObj);
}