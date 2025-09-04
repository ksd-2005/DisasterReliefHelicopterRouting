#include "solver.h"
#include <iostream>
#include <chrono>
#include <limits>
#include <algorithm>
#include <numeric>
#include "simulated_annealing.h"
using namespace std;

static double computeTripDistanceFromHome(const Point &home, const vector<Drop> &drops, const ProblemData &data) {
    Point prev = home;
    double dist = 0.0;
    for (const auto &drop : drops) {
        const Point &vp = data.villages[drop.village_id - 1].coords;
        dist += distance(prev, vp);
        prev = vp;
    }
    dist += distance(prev, home);
    return dist;
}

static double computeTripWeight(const vector<Drop> &drops, const ProblemData &data) {
    // Each drop lists counts of packets actually dropped at villages.
    long long totalDry = 0, totalPerc = 0, totalOther = 0;
    for (const auto &d : drops) {
        totalDry += d.dry_food;
        totalPerc += d.perishable_food;
        totalOther += d.other_supplies;
    }
    double wDry = data.packages[0].weight;
    double wPerc = data.packages[1].weight;
    double wOther = data.packages[2].weight;
    return totalDry * wDry + totalPerc * wPerc + totalOther * wOther;
}

Solution solve(const ProblemData& problem) {
    cout << "Starting solver with Greedy Construction Heuristic (validated)..." << endl;

    Solution solution;
    // Track which villages have been fully served (we will not exceed demand)
    vector<bool> served(problem.villages.size(), false);
    // Track remaining demand in packet counts (we won't try to exceed useful amounts)
    vector<int> remaining_food_need(problem.villages.size(), 0); // prefer perishable
    vector<int> remaining_other_need(problem.villages.size(), 0);

    // Precompute per-village needs:
    for (size_t i = 0; i < problem.villages.size(); ++i) {
        int pop = problem.villages[i].population;
        // total meals needed = 9 * pop (can be perishable or dry).
        remaining_food_need[i] = 9 * pop;
        remaining_other_need[i] = 1 * pop;
    }

    double globalDMax = problem.d_max;

    // For each helicopter, build a plan greedily while ensuring feasibility.
    for (const Helicopter &heli : problem.helicopters) {
        HelicopterPlan plan;
        plan.helicopter_id = heli.id;
        plan.trips.clear();
        double heli_total_distance = 0.0; // across all trips (must remain <= globalDMax)

        // Attempt to generate trips until no feasible trip can be found or globalDMax exceeded.
        while (true) {
            // If all villages are served (no remaining useful need), break.
            bool anyUsefulLeft = false;
            for (size_t i = 0; i < problem.villages.size(); ++i) {
                if (remaining_food_need[i] > 0 || remaining_other_need[i] > 0) {
                    anyUsefulLeft = true;
                    break;
                }
            }
            if (!anyUsefulLeft) break;

            Trip trip;
            trip.drops.clear();
            trip.dry_food_pickup = trip.perishable_food_pickup = trip.other_supplies_pickup = 0;
            double currentTripDistance = 0.0;
            Point home = problem.cities[heli.home_city_id - 1];
            Point currentLocation = home;
            double currentLoadWeight = 0.0;

            bool addedAny = false;

            // Greedy loop: pick best next village by value/distance ratio (respecting remaining need)
            while (true) {
                int bestIdx = -1;
                double bestScore = -numeric_limits<double>::infinity();
                int bestDryToDrop = 0, bestPercToDrop = 0, bestOtherToDrop = 0;
                double bestAdditionalDistance = 0.0;
                double bestNewLoadWeight = 0.0;

                for (size_t i = 0; i < problem.villages.size(); ++i) {
                    if (remaining_food_need[i] <= 0 && remaining_other_need[i] <= 0) continue;

                    const Village &v = problem.villages[i];
                    const Point &vp = v.coords;

                    // distance if we insert this village at the end:
                    double withoutCandidate = distance(currentLocation, home);
                    double withCandidate = distance(currentLocation, vp) + distance(vp, home);
                    double deltaDist = withCandidate - withoutCandidate;
                    double tentativeTripDist = currentTripDistance + deltaDist;

                    // per-trip distance check:
                    if (tentativeTripDist > heli.distance_capacity) continue;

                    // compute how many packets we'd drop here (greedy: prefer perishable first)
                    int percCanDrop = min(remaining_food_need[i], remaining_food_need[i]); // we may choose to drop perishable if we had perishable stock; but at start we drop dry by default
                    // For initial feasible start state keep it simple:
                    // drop all remaining food as dry food (0th packet) unless weights/values suggest otherwise.
                    int dryToDrop = remaining_food_need[i];          // try to satisfy food need entirely
                    int percToDrop = 0;
                    int otherToDrop = remaining_other_need[i];

                    // compute candidate weight increase (weights in grams or kg as per input, same units)
                    double addWeight = dryToDrop * problem.packages[0].weight +
                                       percToDrop * problem.packages[1].weight +
                                       otherToDrop * problem.packages[2].weight;

                    if (currentLoadWeight + addWeight > heli.weight_capacity) {
                        // Cannot fully satisfy; try to partially satisfy proportionally to remaining capacity.
                        double spareWeight = heli.weight_capacity - currentLoadWeight;
                        if (spareWeight <= 0) continue;
                        // greedily allocate to other supplies first? keep simple: allocate to food then other.
                        // Determine how many dry packets we can still add
                        int canAddDry = static_cast<int>(floor(spareWeight / problem.packages[0].weight));
                        if (canAddDry <= 0) {
                            // try other supplies
                            int canAddOther = static_cast<int>(floor(spareWeight / problem.packages[2].weight));
                            if (canAddOther <= 0) continue;
                            dryToDrop = 0;
                            otherToDrop = min<int>(otherToDrop, canAddOther);
                        } else {
                            // add as many dry as possible (cap at remaining_food_need)
                            dryToDrop = min<int>(dryToDrop, canAddDry);
                            // recompute leftover spare weight if desired (skip perishable).
                        }
                        addWeight = dryToDrop * problem.packages[0].weight + otherToDrop * problem.packages[2].weight;
                        if (addWeight <= 0) continue; // nothing to add
                    }

                    // Score: value delivered per additional distance (avoid divide by zero)
                    double deliveredValue = dryToDrop * problem.packages[0].value +
                                            percToDrop * problem.packages[1].value +
                                            otherToDrop * problem.packages[2].value;
                    double travelCostDenom = distance(currentLocation, vp) + 1e-6;
                    double score = deliveredValue / travelCostDenom;

                    if (score > bestScore) {
                        bestScore = score;
                        bestIdx = (int)i;
                        bestDryToDrop = dryToDrop;
                        bestPercToDrop = percToDrop;
                        bestOtherToDrop = otherToDrop;
                        bestAdditionalDistance = deltaDist;
                        bestNewLoadWeight = addWeight;
                    }
                } // end for all villages

                if (bestIdx == -1) break; // no feasible next village

                // If adding this village would violate global DMax (heli_total_distance + tentative trip dist), skip it.
                double tentativeTripDistance = currentTripDistance + bestAdditionalDistance;
                double tripDistanceIfCommitted = tentativeTripDistance; // trip length (we will add return to home in compute)
                double totalIfCommitted = heli_total_distance + tripDistanceIfCommitted;
                if (totalIfCommitted > globalDMax) {
                    // cannot add more trips for this helicopter (global DMax would be violated)
                    break;
                }

                // Accept the candidate
                Drop drop;
                drop.village_id = problem.villages[bestIdx].id;
                drop.dry_food = bestDryToDrop;
                drop.perishable_food = bestPercToDrop;
                drop.other_supplies = bestOtherToDrop;

                // Apply the drop to trip and update state
                trip.drops.push_back(drop);
                currentTripDistance += bestAdditionalDistance;
                currentLoadWeight += bestNewLoadWeight;
                currentLocation = problem.villages[bestIdx].coords;

                // reduce remaining needs for that village (but do not mark fully served unless both types are zero)
                remaining_food_need[bestIdx] = max(0, remaining_food_need[bestIdx] - (bestDryToDrop + bestPercToDrop));
                remaining_other_need[bestIdx] = max(0, remaining_other_need[bestIdx] - bestOtherToDrop);

                if (remaining_food_need[bestIdx] == 0 && remaining_other_need[bestIdx] == 0) {
                    served[bestIdx] = true;
                }

                addedAny = true;

                // if helicopter trip is at capacity (weight near limit or distance near limit), stop adding further villages
                if (currentLoadWeight >= heli.weight_capacity - 1e-9) break;
                if (currentTripDistance >= heli.distance_capacity - 1e-9) break;
            } // end greedy building of single trip

            // Finalize trip if we added any drops and checks pass
            if (addedAny && !trip.drops.empty()) {
                // compute full trip distance (including return to home)
                trip.trip_distance = computeTripDistanceFromHome(problem.cities[heli.home_city_id - 1], trip.drops, problem);
                // per-trip distance constraint
                if (trip.trip_distance > heli.distance_capacity + 1e-9) {
                    // If this happens skip this trip (shouldn't normally happen because we checked while adding), rollback changes by restoring remaining needs:
                    // For simplicity: abort adding more trips for this heli
                    break;
                }
                // compute trip weight
                double tripWeight = computeTripWeight(trip.drops, problem);
                if (tripWeight > heli.weight_capacity + 1e-9) {
                    // similarly abort if overweight
                    break;
                }

                // Check global DMax
                if (heli_total_distance + trip.trip_distance > globalDMax + 1e-9) {
                    break;
                }

                // Set pickup counts (sum of drops)
                long long sumDry = 0, sumPerc = 0, sumOther = 0;
                for (const auto &d : trip.drops) {
                    sumDry += d.dry_food;
                    sumPerc += d.perishable_food;
                    sumOther += d.other_supplies;
                }
                trip.dry_food_pickup = static_cast<int>(sumDry);
                trip.perishable_food_pickup = static_cast<int>(sumPerc);
                trip.other_supplies_pickup = static_cast<int>(sumOther);

                // compute trip cost
                trip.trip_cost = heli.fixed_cost + heli.alpha * trip.trip_distance;

                // add trip to plan and update helicopter totals
                plan.trips.push_back(trip);
                heli_total_distance += trip.trip_distance;
            } else {
                // No feasible trip could be built for this helicopter -> break
                break;
            }

            // stop if this helicopter would exceed globalDMax or cannot make further trips
            if (heli_total_distance >= globalDMax - 1e-9) break;
        } // end while per-helicopter

        // compute plan meta-data (delivered value, trip cost, objective)
        if (!plan.trips.empty()) {
            double planDeliveredValue = 0.0;
            double planTripCost = 0.0;
            for (const Trip &trip : plan.trips) {
                long long totalDry = 0, totalPerc = 0, totalOther = 0;
                for (const Drop &drop : trip.drops) {
                    totalDry += drop.dry_food;
                    totalPerc += drop.perishable_food;
                    totalOther += drop.other_supplies;
                }
                planDeliveredValue += totalDry * problem.packages[0].value +
                                      totalPerc * problem.packages[1].value +
                                      totalOther * problem.packages[2].value;
                planTripCost += trip.trip_cost;
            }
            plan.total_delivered_value = planDeliveredValue;
            plan.total_trip_cost = planTripCost;
            plan.objective_value = planDeliveredValue - planTripCost;
            solution.push_back(plan);
        } else {
            // push empty plan (no trips) to preserve per-helicopter ordering if you want
            HelicopterPlan emptyPlan;
            emptyPlan.helicopter_id = heli.id;
            emptyPlan.trips = {};
            emptyPlan.total_delivered_value = 0.0;
            emptyPlan.total_trip_cost = 0.0;
            emptyPlan.objective_value = 0.0;
            solution.push_back(emptyPlan);
        }
    } // end for each helicopter

    cout << "Greedy validated construction complete. Generated solution with " << solution.size() << " helicopter plans." << endl;
    cout<<"greedy objective: "<<computeObjective(solution,problem)<<endl;
    // Run simulated annealing starting from this valid start state (optional)
    double final_obj_simulated_annealing = simulated_annealing(solution, problem);
    cout << "Simulated annealing returned objective: " << final_obj_simulated_annealing << endl;

    cout << "Solver finished." << endl;
    return solution;
}
