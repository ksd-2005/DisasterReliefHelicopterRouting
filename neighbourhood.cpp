#include "neighbourhood.h"
#include "structures.h"
#include <algorithm>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <vector>
#include <array>
#include <iostream>
double compute_trip_dist(const Trip &trip, const Point &home,
                         const ProblemData &problemData) {
    if (trip.drops.empty())
        return 0.0;
    double totalDistance = 0.0;
    Point lastPoint = home;
    for (const auto &drop : trip.drops) {
        const Point &villagePoint =
            problemData.villages[drop.village_id - 1].coords;
        totalDistance += distance(lastPoint, villagePoint);
        lastPoint = villagePoint;
    }
    totalDistance += distance(lastPoint, home);
    return totalDistance;
}

double compute_village_value(const std::vector<int> &delivered,
                             const ProblemData &data, int pop) {
    int K_food = 9 * pop;
    int K_other = 1 * pop;
    int dry = delivered[0];
    int perish = delivered[1];
    int other = delivered[2];
    double food_value;
    int meals = dry + perish;
    if (meals <= K_food) {
        food_value =
            dry * data.packages[0].value + perish * data.packages[1].value;
    } else {
        int effective_perish = std::min(perish, K_food);
        int effective_dry = std::min(dry, K_food - effective_perish);
        food_value = effective_dry * data.packages[0].value +
                     effective_perish * data.packages[1].value;
    }
    double other_value = std::min(other, K_other) * data.packages[2].value;
    return food_value + other_value;
}

double compute_max_village_value(const ProblemData &data, int pop) {
    double max_food_value = 9.0 * pop * data.packages[1].value; // prefer perishable
    double max_other_value = 1.0 * pop * data.packages[2].value;
    return max_food_value + max_other_value;
}

// ---------- new helper: validate single helicopter plan ----------
static bool validate_plan_for_heli(const HelicopterPlan &plan,
                                   const ProblemData &problemData) {
    if (plan.helicopter_id < 1 ||
        plan.helicopter_id > static_cast<int>(problemData.helicopters.size()))
        return false;
    const Helicopter &heli = problemData.helicopters[plan.helicopter_id - 1];
    const Point &home = problemData.cities[heli.home_city_id - 1];

    double heli_total_dist = 0.0;
    for (const auto &trip : plan.trips) {
        int sum_drops[3] = {0, 0, 0};
        for (const auto &drop : trip.drops) {
            if (drop.village_id < 1 ||
                drop.village_id > static_cast<int>(problemData.villages.size()))
                return false;
            sum_drops[0] += drop.dry_food;
            sum_drops[1] += drop.perishable_food;
            sum_drops[2] += drop.other_supplies;
        }
        if (sum_drops[0] != trip.dry_food_pickup ||
            sum_drops[1] != trip.perishable_food_pickup ||
            sum_drops[2] != trip.other_supplies_pickup)
            return false;

        double trip_weight = trip.dry_food_pickup * problemData.packages[0].weight +
                             trip.perishable_food_pickup * problemData.packages[1].weight +
                             trip.other_supplies_pickup * problemData.packages[2].weight;
        if (trip_weight > heli.weight_capacity + 1e-9) return false;

        double trip_dist = compute_trip_dist(trip, home, problemData);
        if (trip_dist > heli.distance_capacity + 1e-9) return false;

        heli_total_dist += trip_dist;
    }
    if (heli_total_dist > problemData.d_max + 1e-9) return false;
    return true;
}

// ---------- small helper: compute full village_delivered for a solution ----------
static std::vector<std::vector<int>> compute_village_delivered(const Solution &sol,
                                                               const ProblemData &data) {
    std::vector<std::vector<int>> village_delivered(data.villages.size(),
                                                    std::vector<int>(3, 0));
    for (const auto &plan : sol) {
        for (const auto &trip : plan.trips) {
            for (const auto &drop : trip.drops) {
                int vid = drop.village_id - 1;
                if (vid >= 0 && static_cast<size_t>(vid) < village_delivered.size()) {
                    village_delivered[vid][0] += drop.dry_food;
                    village_delivered[vid][1] += drop.perishable_food;
                    village_delivered[vid][2] += drop.other_supplies;
                }
            }
        }
    }
    return village_delivered;
}

double compute_objective(const Solution &sol, const ProblemData &data,
                         bool *is_valid_ptr = nullptr) {
    bool is_valid = true;
    double total_value = 0.0;
    double total_cost = 0.0;
    std::vector<std::vector<int>> village_delivered(data.villages.size(),
                                                    std::vector<int>(3, 0));

    for (const auto &plan : sol) {
        if (plan.helicopter_id < 1 ||
            plan.helicopter_id > static_cast<int>(data.helicopters.size())) {
            is_valid = false;
            break;
        }
        const Helicopter &heli = data.helicopters[plan.helicopter_id - 1];
        const Point &home = data.cities[heli.home_city_id - 1];
        double heli_total_dist = 0.0;
        for (const auto &trip : plan.trips) {
            double trip_weight =
                trip.dry_food_pickup * data.packages[0].weight +
                trip.perishable_food_pickup * data.packages[1].weight +
                trip.other_supplies_pickup * data.packages[2].weight;
            if (trip_weight > heli.weight_capacity) {
                is_valid = false;
            }
            double trip_dist = compute_trip_dist(trip, home, data);
            if (trip_dist > heli.distance_capacity) {
                is_valid = false;
            }
            heli_total_dist += trip_dist;
            total_cost += heli.fixed_cost + heli.alpha * trip_dist;
            int sum_drops[3] = {0, 0, 0};
            for (const auto &drop : trip.drops) {
                if (drop.village_id < 1 ||
                    drop.village_id > static_cast<int>(data.villages.size())) {
                    is_valid = false;
                }
                sum_drops[0] += drop.dry_food;
                sum_drops[1] += drop.perishable_food;
                sum_drops[2] += drop.other_supplies;
                village_delivered[drop.village_id - 1][0] += drop.dry_food;
                village_delivered[drop.village_id - 1][1] +=
                    drop.perishable_food;
                village_delivered[drop.village_id - 1][2] +=
                    drop.other_supplies;
            }
            if (sum_drops[0] != trip.dry_food_pickup ||
                sum_drops[1] != trip.perishable_food_pickup ||
                sum_drops[2] != trip.other_supplies_pickup) {
                is_valid = false;
            }
        }
        if (heli_total_dist > data.d_max) {
            is_valid = false;
        }
    }

    if (!is_valid) {
        if (is_valid_ptr)
            *is_valid_ptr = false;
        return -std::numeric_limits<double>::infinity();
    }

    for (size_t v = 0; v < data.villages.size(); ++v) {
        total_value += compute_village_value(village_delivered[v], data,
                                             data.villages[v].population);
    }

    double objective = total_value - total_cost;
    if (is_valid_ptr)
        *is_valid_ptr = true;
    return objective;
}

Solution reorder_visits(const Solution &HelicopterPlans,
                        const ProblemData &problemData, double current_obj,
                        bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int attempt = 0; attempt < 10; ++attempt) {
        Solution reorderedPlans = HelicopterPlans;
        if (reorderedPlans.empty())
            continue;

        std::uniform_int_distribution<size_t> heli_dist(
            0, reorderedPlans.size() - 1);
        size_t h = heli_dist(gen);
        auto &plan = reorderedPlans[h];

        if (plan.trips.empty())
            continue;

        std::uniform_int_distribution<size_t> t_dist(0, plan.trips.size() - 1);
        size_t t = t_dist(gen);
        auto &trip = plan.trips[t];

        if (trip.drops.size() < 2)
            continue;

        std::uniform_int_distribution<size_t> pos_dist(0,
                                                       trip.drops.size() - 1);
        size_t i = pos_dist(gen);
        size_t j = pos_dist(gen);
        if (i > j)
            std::swap(i, j);
        if (i == j)
            continue;

        const Helicopter &heli =
            problemData.helicopters[plan.helicopter_id - 1];
        const Point home = problemData.cities[heli.home_city_id - 1];

        double heli_total_dist = 0.0;
        for (const auto &tr : plan.trips) {
            heli_total_dist += compute_trip_dist(tr, home, problemData);
        }

        double old_dist = compute_trip_dist(trip, home, problemData);
        std::reverse(trip.drops.begin() + i, trip.drops.begin() + j + 1);
        double new_dist = compute_trip_dist(trip, home, problemData);
        double delta_dist = new_dist - old_dist;
        double new_heli_total = heli_total_dist + delta_dist;

        if (new_dist > heli.distance_capacity ||
            new_heli_total > problemData.d_max) {
                //////std::cout<<"Wrong state failed"<<std::endl;
            std::reverse(trip.drops.begin() + i,
                         trip.drops.begin() + j + 1); // Revert
            continue;
        }

        double delta_cost = heli.alpha * delta_dist;
        double delta_obj = -delta_cost;
        if (!improve_only || delta_obj > 0) { // Better objective (lower cost)
            //std::cout<<compute_objective(reorderedPlans,problemData)<<" "<<current_obj<<std::endl;
             ////std::cout<<"cmae :: reorder_visits"<<std::endl;
            return reorderedPlans;
        }
        std::reverse(trip.drops.begin() + i,
                     trip.drops.begin() + j + 1); // Revert if not accepted
    }
    return HelicopterPlans;
}

Solution reallocate_packages(const Solution &HelicopterPlans,
                             const ProblemData &problemData,
                             double current_obj, bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> factor_dist(0.1, 0.9);

    for (int attempt = 0; attempt < 5; ++attempt) {
        double donationFactor = factor_dist(gen);
        Solution reallocatedPlans = HelicopterPlans;

        for (size_t p = 0; p < reallocatedPlans.size(); ++p) {
            auto &plan = reallocatedPlans[p];
            const Helicopter &heli = problemData.helicopters[plan.helicopter_id - 1];
            const Point home = problemData.cities[heli.home_city_id - 1];

            for (size_t t = 0; t < plan.trips.size(); ++t) {
                Trip tripCopy = plan.trips[t];
                if (tripCopy.drops.empty()) continue;

                std::map<int, std::array<int, 3>> old_contrib;
                for (const auto &drop : tripCopy.drops) {
                    auto &c = old_contrib[drop.village_id];
                    c[0] += drop.dry_food;
                    c[1] += drop.perishable_food;
                    c[2] += drop.other_supplies;
                }

                size_t farIndex = 0;
                double maxDist = -1.0;
                for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
                    int vid = tripCopy.drops[i].village_id;
                    const Point &vpt = problemData.villages[vid - 1].coords;
                    double d = distance(home, vpt);
                    if (d > maxDist) {
                        maxDist = d;
                        farIndex = i;
                    }
                }

                int totalDonationFood = 0;
                std::vector<int> donationFood(tripCopy.drops.size(), 0);
                for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
                    if (i == farIndex) continue;
                    int currentFood = tripCopy.drops[i].dry_food + tripCopy.drops[i].perishable_food;
                    int donate = static_cast<int>(donationFactor * currentFood);
                    donationFood[i] = donate;
                    totalDonationFood += donate;
                }

                int farVillageId = tripCopy.drops[farIndex].village_id;
                int farCapFood = problemData.villages[farVillageId - 1].population * 9;
                int currentFarFood = tripCopy.drops[farIndex].dry_food + tripCopy.drops[farIndex].perishable_food;
                int gapFood = std::max(0, farCapFood - currentFarFood);
                int actualDonationFood = std::min(totalDonationFood, gapFood);

                if (actualDonationFood > 0 && totalDonationFood > 0) {
                    for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
                        if (i == farIndex) continue;
                        if (donationFood[i] > 0) {
                            int removal = static_cast<int>(std::round(
                                donationFood[i] * (actualDonationFood / static_cast<double>(totalDonationFood))));
                            int currentFood = tripCopy.drops[i].dry_food + tripCopy.drops[i].perishable_food;
                            if (currentFood > 0) {
                                double ratioDry = tripCopy.drops[i].dry_food / static_cast<double>(currentFood);
                                int removeDry = static_cast<int>(std::round(removal * ratioDry));
                                int removePerc = removal - removeDry;
                                tripCopy.drops[i].dry_food = std::max(0, tripCopy.drops[i].dry_food - removeDry);
                                tripCopy.drops[i].perishable_food = std::max(0, tripCopy.drops[i].perishable_food - removePerc);
                            }
                        }
                    }
                    if (currentFarFood > 0) {
                        double ratioDryFar = tripCopy.drops[farIndex].dry_food / static_cast<double>(currentFarFood);
                        int addDry = static_cast<int>(std::round(actualDonationFood * ratioDryFar));
                        int addPerc = actualDonationFood - addDry;
                        tripCopy.drops[farIndex].dry_food += addDry;
                        tripCopy.drops[farIndex].perishable_food += addPerc;
                    } else {
                        tripCopy.drops[farIndex].dry_food += actualDonationFood;
                    }
                }

                int totalDonationOther = 0;
                std::vector<int> donationOther(tripCopy.drops.size(), 0);
                for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
                    if (i == farIndex) continue;
                    int currentOther = tripCopy.drops[i].other_supplies;
                    int donate = static_cast<int>(donationFactor * currentOther);
                    donationOther[i] = donate;
                    totalDonationOther += donate;
                }
                int farCapOther = problemData.villages[farVillageId - 1].population * 1;
                int currentFarOther = tripCopy.drops[farIndex].other_supplies;
                int gapOther = std::max(0, farCapOther - currentFarOther);
                int actualDonationOther = std::min(totalDonationOther, gapOther);

                if (actualDonationOther > 0 && totalDonationOther > 0) {
                    for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
                        if (i == farIndex) continue;
                        if (donationOther[i] > 0) {
                            int removal = static_cast<int>(std::round(donationOther[i] * (actualDonationOther / static_cast<double>(totalDonationOther))));
                            tripCopy.drops[i].other_supplies = std::max(0, tripCopy.drops[i].other_supplies - removal);
                        }
                    }
                    tripCopy.drops[farIndex].other_supplies += actualDonationOther;
                }

                tripCopy.dry_food_pickup = 0;
                tripCopy.perishable_food_pickup = 0;
                tripCopy.other_supplies_pickup = 0;
                for (const auto &drop : tripCopy.drops) {
                    tripCopy.dry_food_pickup += drop.dry_food;
                    tripCopy.perishable_food_pickup += drop.perishable_food;
                    tripCopy.other_supplies_pickup += drop.other_supplies;
                }

                HelicopterPlan planCopy = plan;
                planCopy.trips[t] = tripCopy;

                if (!validate_plan_for_heli(planCopy, problemData)) {
                    continue;
                }

                reallocatedPlans[p] = planCopy;
                bool valid;
                double obj = compute_objective(reallocatedPlans, problemData, &valid);
                if (valid && (!improve_only || obj > current_obj)) {
                    //std::cout<<obj<<" "<<current_obj<<std::endl;
                     ////std::cout<<"cmae :: reallocate_packages"<<std::endl;
                    return reallocatedPlans;
                } else {
                    reallocatedPlans[p] = plan;
                }
            } // end trips
        } // end plans
    } // end attempts

    return HelicopterPlans;
}

Solution move_visit(const Solution &HelicopterPlans,
                    const ProblemData &problemData, double current_obj,
                    bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int attempt = 0; attempt < 10; ++attempt) {
        Solution movedPlans = HelicopterPlans;
        if (movedPlans.empty())
            continue;

        std::uniform_int_distribution<size_t> helicopterDist(
            0, movedPlans.size() - 1);
        size_t heliIndex = helicopterDist(gen);
        auto &plan = movedPlans[heliIndex];

        if (plan.trips.size() < 2)
            continue;

        const Helicopter &heli =
            problemData.helicopters[plan.helicopter_id - 1];
        const Point home = problemData.cities[heli.home_city_id - 1];

        struct TripLoad {
            size_t tripIndex;
            double weightLoad;
            double distanceLoad;
            double weightRatio;
            double distanceRatio;
            double combinedRatio;
        };

        std::vector<TripLoad> tripLoads;
        for (size_t i = 0; i < plan.trips.size(); ++i) {
            auto &trip = plan.trips[i];
            if (trip.drops.empty())
                continue;

            double weight =
                trip.dry_food_pickup * problemData.packages[0].weight +
                trip.perishable_food_pickup * problemData.packages[1].weight +
                trip.other_supplies_pickup * problemData.packages[2].weight;

            double totalDistance = compute_trip_dist(trip, home, problemData);

            double weightRatio = weight / heli.weight_capacity;
            double distanceRatio = totalDistance / heli.distance_capacity;
            double combinedRatio = std::max(weightRatio, distanceRatio);

            tripLoads.push_back({i, weight, totalDistance, weightRatio,
                                 distanceRatio, combinedRatio});
        }

        if (tripLoads.size() < 2)
            continue;

        auto sourceIt =
            std::max_element(tripLoads.begin(), tripLoads.end(),
                             [](const TripLoad &a, const TripLoad &b) {
                                 return a.combinedRatio < b.combinedRatio;
                             });

        auto targetIt =
            std::min_element(tripLoads.begin(), tripLoads.end(),
                             [](const TripLoad &a, const TripLoad &b) {
                                 return a.combinedRatio < b.combinedRatio;
                             });

        if (sourceIt->tripIndex == targetIt->tripIndex) {
            std::sort(tripLoads.begin(), tripLoads.end(),
                      [](const TripLoad &a, const TripLoad &b) {
                          return a.combinedRatio < b.combinedRatio;
                      });
            if (tripLoads.size() >= 2) {
                targetIt = tripLoads.begin() + 0; // Smallest
                sourceIt = tripLoads.begin() + 1; // Next
            } else {
                continue;
            }
        }

        auto &sourceTrip = plan.trips[sourceIt->tripIndex];
        auto &targetTrip = plan.trips[targetIt->tripIndex];

        std::uniform_int_distribution<size_t> dropDist(
            0, sourceTrip.drops.size() - 1);
        size_t dropIndex = dropDist(gen);
        if (dropIndex >= sourceTrip.drops.size()) continue;

        // operate by copying selected drop to make revert easy
        Drop selectedDrop = sourceTrip.drops[dropIndex];

        // mutate copies first
        Trip sourceCopy = sourceTrip;
        Trip targetCopy = targetTrip;

        // remove drop from sourceCopy, add to targetCopy
        sourceCopy.drops.erase(sourceCopy.drops.begin() + dropIndex);
        sourceCopy.dry_food_pickup -= selectedDrop.dry_food;
        sourceCopy.perishable_food_pickup -= selectedDrop.perishable_food;
        sourceCopy.other_supplies_pickup -= selectedDrop.other_supplies;

        targetCopy.drops.push_back(selectedDrop);
        targetCopy.dry_food_pickup += selectedDrop.dry_food;
        targetCopy.perishable_food_pickup += selectedDrop.perishable_food;
        targetCopy.other_supplies_pickup += selectedDrop.other_supplies;

        HelicopterPlan planCopy = plan;
        planCopy.trips[sourceIt->tripIndex] = sourceCopy;
        planCopy.trips[targetIt->tripIndex] = targetCopy;

        if (!validate_plan_for_heli(planCopy, problemData)) {
            continue;
        }

        movedPlans[heliIndex] = planCopy;
        bool valid;
        double obj = compute_objective(movedPlans, problemData, &valid);
        if (valid && (!improve_only || obj > current_obj)) {
            //std::cout<<obj<<" "<<current_obj<<std::endl;
             ////std::cout<<"cmae :: move_visits"<<std::endl;
            return movedPlans;
        } else {
            continue;
        }
    }
    return HelicopterPlans;
}

// -------- split_trip (copy-based) -----------
Solution split_trip(const Solution &sol, const ProblemData &problemData,
                    double current_obj, bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int attempt = 0; attempt < 10; ++attempt) {
        Solution neighbor = sol;
        if (neighbor.empty()) continue;

        std::uniform_int_distribution<size_t> h_dist(0, neighbor.size() - 1);
        size_t h_idx = h_dist(gen);
        auto &plan = neighbor[h_idx];
        if (plan.trips.empty()) continue;

        std::uniform_int_distribution<size_t> t_dist(0, plan.trips.size() - 1);
        size_t t_idx = t_dist(gen);
        Trip trip = plan.trips[t_idx];
        if (trip.drops.size() <= 1) continue;

        std::uniform_int_distribution<size_t> d_dist(0, trip.drops.size() - 1);
        size_t d_idx = d_dist(gen);

        if (d_idx >= trip.drops.size()) continue;

        Trip tripCopy = trip;
        Drop d = tripCopy.drops[d_idx];
        tripCopy.drops.erase(tripCopy.drops.begin() + d_idx);
        tripCopy.dry_food_pickup -= d.dry_food;
        tripCopy.perishable_food_pickup -= d.perishable_food;
        tripCopy.other_supplies_pickup -= d.other_supplies;

        Trip newTrip;
        newTrip.dry_food_pickup = d.dry_food;
        newTrip.perishable_food_pickup = d.perishable_food;
        newTrip.other_supplies_pickup = d.other_supplies;
        newTrip.drops.push_back(d);

        HelicopterPlan planCopy = plan;
        planCopy.trips[t_idx] = tripCopy;
        planCopy.trips.push_back(newTrip);

        if (!validate_plan_for_heli(planCopy, problemData)) continue;

        neighbor[h_idx] = planCopy;
        bool valid;
        double obj = compute_objective(neighbor, problemData, &valid);
        if (valid && (!improve_only || obj > current_obj)) {
            //std::cout<<obj<<" "<<current_obj<<std::endl;
             ////std::cout<<"cmae :: split_trip"<<std::endl;
            return neighbor;
        }
    }
    return sol;
}

// -------- merge_trips (copy-based) ---------
Solution merge_trips(const Solution &sol, const ProblemData &problemData,
                     double current_obj, bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int attempt = 0; attempt < 10; ++attempt) {
        Solution neighbor = sol;
        if (neighbor.empty())
            continue;

        std::uniform_int_distribution<size_t> h_dist(0, neighbor.size() - 1);
        size_t h_idx = h_dist(gen);
        auto &plan = neighbor[h_idx];

        if (plan.trips.size() < 2)
            continue;

        const Helicopter &heli =
            problemData.helicopters[plan.helicopter_id - 1];
        const Point home = problemData.cities[heli.home_city_id - 1];

        std::uniform_int_distribution<size_t> t_dist(0, plan.trips.size() - 1);
        size_t t1 = t_dist(gen);
        size_t t2 = t_dist(gen);

        size_t cnt = 0;
        while (t1 == t2 && cnt < 5) {
            cnt++;
            t2 = t_dist(gen);
        }
        if (t1 == t2)
            continue;

        if (t1 > t2) std::swap(t1, t2);

        auto trip1 = plan.trips[t1]; // copy
        auto trip2 = plan.trips[t2];

        Trip merged;
        merged.dry_food_pickup = trip1.dry_food_pickup + trip2.dry_food_pickup;
        merged.perishable_food_pickup =
            trip1.perishable_food_pickup + trip2.perishable_food_pickup;
        merged.other_supplies_pickup =
            trip1.other_supplies_pickup + trip2.other_supplies_pickup;

        merged.drops = std::move(plan.trips[t1].drops);
        merged.drops.insert(merged.drops.end(),
                            std::make_move_iterator(plan.trips[t2].drops.begin()),
                            std::make_move_iterator(plan.trips[t2].drops.end()));

        HelicopterPlan planCopy = plan;
        planCopy.trips.erase(planCopy.trips.begin() + t2);
        planCopy.trips.erase(planCopy.trips.begin() + t1);
        planCopy.trips.push_back(merged);

        if (!validate_plan_for_heli(planCopy, problemData)) continue;

        neighbor[h_idx] = planCopy;
        bool valid;
        double obj = compute_objective(neighbor, problemData, &valid);
        if (valid && (!improve_only || obj > current_obj)) {
            //cout<<obj<<" "<<current_obj<<std::endl;
            return neighbor;

             ////std::cout<<"cmae :: merge_trips"<<std::endl;
        }
    }
    return sol;
}

Solution reassign_visit(const Solution &sol, const ProblemData &problemData,
                        double current_obj, bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int attempt = 0; attempt < 10; ++attempt) {
        Solution neighbor = sol;
        if (neighbor.size() < 2)
            continue;

        std::uniform_int_distribution<size_t> h_dist(0, neighbor.size() - 1);
        size_t src_h = h_dist(gen);
        auto &planFrom = neighbor[src_h];

        if (planFrom.trips.empty())
            continue;

        std::uniform_int_distribution<size_t> t_dist(0,
                                                     planFrom.trips.size() - 1);
        size_t t_idx = t_dist(gen);
        auto &srcTrip = planFrom.trips[t_idx];

        if (srcTrip.drops.empty())
            continue;

        const Helicopter &src_heli =
            problemData.helicopters[planFrom.helicopter_id - 1];
        const Point src_home = problemData.cities[src_heli.home_city_id - 1];

        double src_heli_total_dist = 0.0;
        for (const auto &tr : planFrom.trips) {
            src_heli_total_dist += compute_trip_dist(tr, src_home, problemData);
        }

        double old_dist_src = compute_trip_dist(srcTrip, src_home, problemData);

        std::uniform_int_distribution<size_t> d_dist(0,
                                                     srcTrip.drops.size() - 1);
        size_t d_idx = d_dist(gen);

        Drop d = std::move(srcTrip.drops[d_idx]);
        srcTrip.drops.erase(srcTrip.drops.begin() + d_idx);

        srcTrip.dry_food_pickup -= d.dry_food;
        srcTrip.perishable_food_pickup -= d.perishable_food;
        srcTrip.other_supplies_pickup -= d.other_supplies;

        bool src_trip_empty = srcTrip.drops.empty();
        if (src_trip_empty) {
            planFrom.trips.erase(planFrom.trips.begin() + t_idx);
        }

        size_t dst_h = h_dist(gen);
        size_t cnt = 0;
        while (src_h == dst_h && cnt < 5) {
            cnt++;
            dst_h = h_dist(gen);
        }
        if (src_h == dst_h)
            continue;

        auto &planTo = neighbor[dst_h];

        const Helicopter &dst_heli =
            problemData.helicopters[planTo.helicopter_id - 1];
        const Point dst_home = problemData.cities[dst_heli.home_city_id - 1];

        double dst_heli_total_dist = 0.0;
        for (const auto &tr : planTo.trips) {
            dst_heli_total_dist += compute_trip_dist(tr, dst_home, problemData);
        }

        bool append_existing = !planTo.trips.empty() && (gen() % 5 != 0);

        double delta_dist_src =
            compute_trip_dist(srcTrip, src_home, problemData) - old_dist_src;
        double new_src_heli_total = src_heli_total_dist + delta_dist_src;

        double delta_dist_dst = 0.0;
        double new_dst_weight = 0.0;
        double new_dst_dist = 0.0;
        double new_dst_heli_total = 0.0;
        double delta_cost = 0.0;
        size_t t_to = 0;
        size_t insert_pos = 0;
        bool new_trip_added = false;

        if (append_existing) {
            std::uniform_int_distribution<size_t> t_to_dist(
                0, planTo.trips.size() - 1);
            t_to = t_to_dist(gen);
            auto &dstTrip = planTo.trips[t_to];

            double old_dist_dst =
                compute_trip_dist(dstTrip, dst_home, problemData);

            if (!dstTrip.drops.empty()) {
                std::uniform_int_distribution<size_t> pos_dist(
                    0, dstTrip.drops.size());
                insert_pos = pos_dist(gen);
            }
            dstTrip.drops.insert(dstTrip.drops.begin() + insert_pos,
                                 Drop{d.village_id, d.dry_food, d.perishable_food, d.other_supplies});

            dstTrip.dry_food_pickup += d.dry_food;
            dstTrip.perishable_food_pickup += d.perishable_food;
            dstTrip.other_supplies_pickup += d.other_supplies;

            new_dst_dist = compute_trip_dist(dstTrip, dst_home, problemData);
            delta_dist_dst = new_dst_dist - old_dist_dst;
            new_dst_heli_total = dst_heli_total_dist + delta_dist_dst;
            new_dst_weight =
                dstTrip.dry_food_pickup * problemData.packages[0].weight +
                dstTrip.perishable_food_pickup *
                    problemData.packages[1].weight +
                dstTrip.other_supplies_pickup * problemData.packages[2].weight;
            delta_cost = dst_heli.alpha * delta_dist_dst +
                         src_heli.alpha * delta_dist_src;
            if (src_trip_empty) {
                delta_cost -= src_heli.fixed_cost; // Removed a trip
            }
        } else {
            Trip newTrip;
            newTrip.dry_food_pickup = d.dry_food;
            newTrip.perishable_food_pickup = d.perishable_food;
            newTrip.other_supplies_pickup = d.other_supplies;
            newTrip.drops.push_back(Drop{d.village_id, d.dry_food, d.perishable_food, d.other_supplies});
            planTo.trips.push_back(newTrip);
            new_trip_added = true;
            new_dst_dist =
                compute_trip_dist(planTo.trips.back(), dst_home, problemData);
            delta_dist_dst = new_dst_dist;
            new_dst_heli_total = dst_heli_total_dist + delta_dist_dst;
            new_dst_weight =
                planTo.trips.back().dry_food_pickup * problemData.packages[0].weight +
                planTo.trips.back().perishable_food_pickup *
                    problemData.packages[1].weight +
                planTo.trips.back().other_supplies_pickup * problemData.packages[2].weight;
            delta_cost = dst_heli.fixed_cost + dst_heli.alpha * delta_dist_dst +
                         src_heli.alpha * delta_dist_src;
            if (src_trip_empty) {
                delta_cost -= src_heli.fixed_cost; // Removed a trip
            }
        }

        double new_src_dist = compute_trip_dist(srcTrip, src_home, problemData);
        double new_src_weight =
            srcTrip.dry_food_pickup * problemData.packages[0].weight +
            srcTrip.perishable_food_pickup * problemData.packages[1].weight +
            srcTrip.other_supplies_pickup * problemData.packages[2].weight;

        bool valid = (new_src_dist <= src_heli.distance_capacity) &&
                     (new_dst_dist <= dst_heli.distance_capacity) &&
                     (new_src_heli_total <= problemData.d_max) &&
                     (new_dst_heli_total <= problemData.d_max) &&
                     (new_src_weight <= src_heli.weight_capacity) &&
                     (new_dst_weight <= dst_heli.weight_capacity);

        if (valid) {
            double delta_obj = -delta_cost;
            if (!improve_only || delta_obj > 0) {
                //cout<<current_obj + delta_obj<<" "<<current_obj<<std::endl;
                 ////std::cout<<"cmae :: reassign_visit"<<std::endl;
                return neighbor;
            }
        }

        // Revert
        if (append_existing) {
            auto &dstTrip = planTo.trips[t_to];
            dstTrip.drops.erase(dstTrip.drops.begin() + insert_pos);
            dstTrip.dry_food_pickup -= d.dry_food;
            dstTrip.perishable_food_pickup -= d.perishable_food;
            dstTrip.other_supplies_pickup -= d.other_supplies;
        } else if (new_trip_added) {
            planTo.trips.pop_back();
        }
        if (src_trip_empty) {
            planFrom.trips.insert(planFrom.trips.begin() + t_idx, srcTrip);
        }
        auto &revertedSrcTrip = src_trip_empty ? planFrom.trips[t_idx] : srcTrip;
        revertedSrcTrip.drops.insert(revertedSrcTrip.drops.begin() + d_idx, std::move(d));
        revertedSrcTrip.dry_food_pickup += d.dry_food;
        revertedSrcTrip.perishable_food_pickup += d.perishable_food;
        revertedSrcTrip.other_supplies_pickup += d.other_supplies;
    }
    return sol;
}

Solution add_new_village(const Solution &HelicopterPlans,
                         const ProblemData &problemData, double current_obj,
                         bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int attempt = 0; attempt < 10; ++attempt) {
        Solution addedPlans = HelicopterPlans;
        if (addedPlans.empty()) continue;

        auto village_delivered = compute_village_delivered(HelicopterPlans, problemData);

        std::vector<size_t> underserved;
        for (size_t v = 0; v < problemData.villages.size(); ++v) {
            double current_val = compute_village_value(village_delivered[v], problemData, problemData.villages[v].population);
            double max_val = compute_max_village_value(problemData, problemData.villages[v].population);
            if (current_val < max_val) underserved.push_back(v);
        }
        if (underserved.empty()) continue;

        std::uniform_int_distribution<size_t> heli_dist(0, addedPlans.size() - 1);
        size_t h = heli_dist(gen);
        auto &plan = addedPlans[h];
        if (plan.trips.empty()) continue;

        std::uniform_int_distribution<size_t> t_dist(0, plan.trips.size() - 1);
        size_t t = t_dist(gen);
        auto trip = plan.trips[t];
        if (trip.drops.empty()) continue;

        std::uniform_int_distribution<size_t> pos_dist(0, trip.drops.size());
        size_t insert_pos = pos_dist(gen);
        std::uniform_int_distribution<size_t> u_dist(0, underserved.size() - 1);
        size_t u_idx = u_dist(gen);
        int new_vid = static_cast<int>(underserved[u_idx]) + 1; // 1-based

        bool already_in_trip = false;
        for (const auto &drop : trip.drops) if (drop.village_id == new_vid) { already_in_trip = true; break; }
        if (already_in_trip) continue;

        Trip tripCopy = trip;
        tripCopy.drops.insert(tripCopy.drops.begin() + insert_pos, Drop{new_vid, 0, 0, 0});

        double new_dist = compute_trip_dist(tripCopy, problemData.cities[problemData.helicopters[plan.helicopter_id - 1].home_city_id - 1], problemData);
        if (new_dist > problemData.helicopters[plan.helicopter_id - 1].distance_capacity) continue;

        std::uniform_real_distribution<double> factor_dist(0.1, 0.9);
        double donationFactor = factor_dist(gen);

        std::vector<Drop> old_drops = tripCopy.drops;
        std::map<int, std::array<int, 3>> old_contrib;
        for (const auto &drop : old_drops) {
            auto &c = old_contrib[drop.village_id];
            c[0] += drop.dry_food;
            c[1] += drop.perishable_food;
            c[2] += drop.other_supplies;
        }

        int totalDonationFood = 0;
        std::vector<int> donationFood(tripCopy.drops.size(), 0);
        for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
            if (i == insert_pos) continue;
            int currentFood = tripCopy.drops[i].dry_food + tripCopy.drops[i].perishable_food;
            int donate = static_cast<int>(donationFactor * currentFood);
            donationFood[i] = donate;
            totalDonationFood += donate;
        }

        int newCapFood = problemData.villages[new_vid - 1].population * 9;
        int currentNewFood = tripCopy.drops[insert_pos].dry_food + tripCopy.drops[insert_pos].perishable_food;
        int gapFood = std::max(0, newCapFood - currentNewFood);
        int actualDonationFood = std::min(totalDonationFood, gapFood);

        bool changed = false;
        if (actualDonationFood > 0 && totalDonationFood > 0) {
            for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
                if (i == insert_pos) continue;
                if (donationFood[i] > 0) {
                    int removal = static_cast<int>(std::round(
                        donationFood[i] * (actualDonationFood / static_cast<double>(totalDonationFood))));
                    int currentFood = tripCopy.drops[i].dry_food + tripCopy.drops[i].perishable_food;
                    if (currentFood > 0) {
                        double ratioDry = tripCopy.drops[i].dry_food / static_cast<double>(currentFood);
                        int removeDry = static_cast<int>(std::round(removal * ratioDry));
                        int removePerc = removal - removeDry;
                        tripCopy.drops[i].dry_food = std::max(0, tripCopy.drops[i].dry_food - removeDry);
                        tripCopy.drops[i].perishable_food = std::max(0, tripCopy.drops[i].perishable_food - removePerc);
                        changed = true;
                    }
                }
            }
            if (currentNewFood > 0) {
                double ratioDryNew = tripCopy.drops[insert_pos].dry_food / static_cast<double>(currentNewFood);
                int addDry = static_cast<int>(std::round(actualDonationFood * ratioDryNew));
                int addPerc = actualDonationFood - addDry;
                tripCopy.drops[insert_pos].dry_food += addDry;
                tripCopy.drops[insert_pos].perishable_food += addPerc;
            } else {
                tripCopy.drops[insert_pos].perishable_food += actualDonationFood;
            }
        }

        int totalDonationOther = 0;
        std::vector<int> donationOther(tripCopy.drops.size(), 0);
        for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
            if (i == insert_pos) continue;
            int currentOther = tripCopy.drops[i].other_supplies;
            int donate = static_cast<int>(donationFactor * currentOther);
            donationOther[i] = donate;
            totalDonationOther += donate;
        }
        int newCapOther = problemData.villages[new_vid - 1].population * 1;
        int currentNewOther = tripCopy.drops[insert_pos].other_supplies;
        int gapOther = std::max(0, newCapOther - currentNewOther);
        int actualDonationOther = std::min(totalDonationOther, gapOther);

        if (actualDonationOther > 0 && totalDonationOther > 0) {
            for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
                if (i == insert_pos) continue;
                if (donationOther[i] > 0) {
                    int removal = static_cast<int>(std::round(donationOther[i] * (actualDonationOther / static_cast<double>(totalDonationOther))));
                    tripCopy.drops[i].other_supplies = std::max(0, tripCopy.drops[i].other_supplies - removal);
                    changed = true;
                }
            }
            tripCopy.drops[insert_pos].other_supplies += actualDonationOther;
        }

        if (!changed) {
            continue;
        }

        tripCopy.dry_food_pickup = 0;
        tripCopy.perishable_food_pickup = 0;
        tripCopy.other_supplies_pickup = 0;
        for (const auto &drop : tripCopy.drops) {
            tripCopy.dry_food_pickup += drop.dry_food;
            tripCopy.perishable_food_pickup += drop.perishable_food;
            tripCopy.other_supplies_pickup += drop.other_supplies;
        }

        HelicopterPlan planCopy = plan;
        planCopy.trips[t] = tripCopy;
        if (!validate_plan_for_heli(planCopy, problemData)) {
            continue;
        }

        addedPlans[h] = planCopy;
        bool valid;
        double obj = compute_objective(addedPlans, problemData, &valid);
        if (valid && (!improve_only || obj > current_obj)) {
            //std::cout<<obj<<" "<<current_obj<<std::endl;
            ////std::cout<<"cmae :: add_new_village"<<std::endl;
            return addedPlans;
        } else {
            continue;
        }
    }
    return HelicopterPlans;
}

// -------- remove_village (copy-based) ----------
Solution remove_village(const Solution &HelicopterPlans,
                        const ProblemData &problemData, double current_obj,
                        bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int attempt = 0; attempt < 10; ++attempt) {
        Solution removedPlans = HelicopterPlans;
        if (removedPlans.empty()) continue;

        std::uniform_int_distribution<size_t> h_dist(0, removedPlans.size() - 1);
        size_t h = h_dist(gen);
        auto &plan = removedPlans[h];
        if (plan.trips.empty()) continue;

        std::uniform_int_distribution<size_t> t_dist(0, plan.trips.size() - 1);
        size_t t = t_dist(gen);
        auto trip = plan.trips[t];

        if (trip.drops.size() < 2) continue;

        std::uniform_int_distribution<size_t> d_dist(0, trip.drops.size() - 1);
        size_t remove_idx = d_dist(gen);

        Trip tripCopy = trip;
        Drop removed_drop = tripCopy.drops[remove_idx];
        tripCopy.drops.erase(tripCopy.drops.begin() + remove_idx);

        int df_to_redist = removed_drop.dry_food;
        int pf_to_redist = removed_drop.perishable_food;
        int os_to_redist = removed_drop.other_supplies;

        std::vector<size_t> candidates;
        for (size_t i = 0; i < tripCopy.drops.size(); ++i) {
            int vid = tripCopy.drops[i].village_id - 1;
            int current_food = 0;
            for (const auto &pl : HelicopterPlans) {
                for (const auto &tr : pl.trips) {
                    for (const auto &dp : tr.drops) {
                        if (dp.village_id - 1 == vid) {
                            current_food += dp.dry_food + dp.perishable_food;
                        }
                    }
                }
            }
            int cap_food = problemData.villages[vid].population * 9;
            if (current_food < cap_food) candidates.push_back(i);
        }

        if (!candidates.empty()) {
            std::uniform_int_distribution<size_t> c_dist(0, candidates.size() - 1);
            size_t target_idx = candidates[c_dist(gen)];
            tripCopy.drops[target_idx].dry_food += df_to_redist;
            tripCopy.drops[target_idx].perishable_food += pf_to_redist;
            tripCopy.drops[target_idx].other_supplies += os_to_redist;
        }

        tripCopy.dry_food_pickup = 0;
        tripCopy.perishable_food_pickup = 0;
        tripCopy.other_supplies_pickup = 0;
        for (const auto &drop : tripCopy.drops) {
            tripCopy.dry_food_pickup += drop.dry_food;
            tripCopy.perishable_food_pickup += drop.perishable_food;
            tripCopy.other_supplies_pickup += drop.other_supplies;
        }

        HelicopterPlan planCopy = plan;
        planCopy.trips[t] = tripCopy;

        if (!validate_plan_for_heli(planCopy, problemData)) {
            continue;
        }

        removedPlans[h] = planCopy;
        bool valid;
        double obj = compute_objective(removedPlans, problemData, &valid);
        if (valid && (!improve_only || obj > current_obj)) {
            //std::cout<<obj<<" "<<current_obj<<std::endl;
                ////std::cout<<"cmae :: remove_village"<<std::endl;
            return removedPlans;
        }
    }
    return HelicopterPlans;
}

Solution get_best_neighbor(const Solution &HelicopterPlans,
                           const ProblemData &problemData, double current_obj) {
    Solution best = HelicopterPlans;
    double best_obj = current_obj;

    auto update_best = [&best, &best_obj, &problemData](const Solution &neigh) {
        bool valid;
        double obj = compute_objective(neigh, problemData, &valid);
        if (valid && obj > best_obj) {
            best = neigh;
            best_obj = obj;
        }
    };

    update_best(reorder_visits(HelicopterPlans, problemData, current_obj, true));
    update_best(reallocate_packages(HelicopterPlans, problemData, current_obj, true));
    update_best(move_visit(HelicopterPlans, problemData, current_obj, true));
    update_best(split_trip(HelicopterPlans, problemData, current_obj, true));
    update_best(merge_trips(HelicopterPlans, problemData, current_obj, true));
    update_best(reassign_visit(HelicopterPlans, problemData, current_obj, true));
    update_best(add_new_village(HelicopterPlans, problemData, current_obj, true));
    update_best(remove_village(HelicopterPlans, problemData, current_obj, true));

    return best;
}

Solution get_random_neighbor(const Solution &HelicopterPlans,
                             const ProblemData &problemData, double current_obj) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<double> weights = {10, 8, 6, 5, 4, 3, 10, 5};
    std::discrete_distribution<int> op_dist(weights.begin(), weights.end());
    int op = op_dist(gen);
    ////std::cout<<"op :: "<<op<<std::endl;
    switch (op) {
    case 0:
        return reorder_visits(HelicopterPlans, problemData, current_obj, false);
    case 1:
        return reallocate_packages(HelicopterPlans, problemData, current_obj, false);
    case 2:
        return move_visit(HelicopterPlans, problemData, current_obj, false);
    case 3:
        return split_trip(HelicopterPlans, problemData, current_obj, false);
    case 4:
        return merge_trips(HelicopterPlans, problemData, current_obj, false);
    case 5:
        return reassign_visit(HelicopterPlans, problemData, current_obj, false);
    case 6:
        return add_new_village(HelicopterPlans, problemData, current_obj, false);
    case 7:
        return remove_village(HelicopterPlans, problemData, current_obj, false);
    default:
        return HelicopterPlans;
    }
}
