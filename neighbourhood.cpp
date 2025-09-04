#include "neighbourhood.h"
#include "structures.h"
#include <algorithm>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <vector>

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
    // Max value assuming all perishable food and all other
    double max_food_value = 9.0 * pop * data.packages[1].value; // prefer perishable
    double max_other_value = 1.0 * pop * data.packages[2].value;
    return max_food_value + max_other_value;
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
            std::reverse(trip.drops.begin() + i,
                         trip.drops.begin() + j + 1); // Revert
            continue;
        }

        double delta_cost = heli.alpha * delta_dist;
        double delta_obj = -delta_cost;
        if (!improve_only || delta_obj > 0) { // Better objective (lower cost)
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
        std::vector<std::vector<int>> village_delivered(
            problemData.villages.size(), std::vector<int>(3, 0));
        for (const auto &plan : HelicopterPlans) {
            for (const auto &trip : plan.trips) {
                for (const auto &drop : trip.drops) {
                    int v = drop.village_id - 1;
                    village_delivered[v][0] += drop.dry_food;
                    village_delivered[v][1] += drop.perishable_food;
                    village_delivered[v][2] += drop.other_supplies;
                }
            }
        }

        bool changed = false;
        for (auto &plan : reallocatedPlans) {
            const Helicopter &heli =
                problemData.helicopters[plan.helicopter_id - 1];
            const Point home = problemData.cities[heli.home_city_id - 1];
            for (auto &trip : plan.trips) {
                if (trip.drops.empty())
                    continue;

                std::vector<Drop> old_drops = trip.drops;
                std::map<int, std::array<int, 3>> old_contrib;
                for (const auto &drop : old_drops) {
                    auto &c = old_contrib[drop.village_id];
                    c[0] += drop.dry_food;
                    c[1] += drop.perishable_food;
                    c[2] += drop.other_supplies;
                }
                std::set<int> affected;
                for (const auto &kv : old_contrib)
                    affected.insert(kv.first);

                double old_capped_sum = 0.0;
                for (int vid : affected) {
                    int v = vid - 1;
                    old_capped_sum += compute_village_value(
                        village_delivered[v], problemData,
                        problemData.villages[v].population);
                }

                size_t farIndex = 0;
                double maxDist = 0.0;
                for (size_t i = 0; i < trip.drops.size(); ++i) {
                    int vid = trip.drops[i].village_id;
                    const Point &villagePoint =
                        problemData.villages[vid - 1].coords;
                    double d = distance(home, villagePoint);
                    if (d > maxDist) {
                        maxDist = d;
                        farIndex = i;
                    }
                }

                int totalDonationFood = 0;
                std::vector<int> donationFood(trip.drops.size(), 0);
                for (size_t i = 0; i < trip.drops.size(); ++i) {
                    if (i == farIndex)
                        continue;
                    int currentFood =
                        trip.drops[i].dry_food + trip.drops[i].perishable_food;
                    int donate = static_cast<int>(donationFactor * currentFood);
                    donationFood[i] = donate;
                    totalDonationFood += donate;
                }

                int farVillageId = trip.drops[farIndex].village_id;
                int farCapFood =
                    problemData.villages[farVillageId - 1].population * 9;
                int currentFarFood = trip.drops[farIndex].dry_food +
                                     trip.drops[farIndex].perishable_food;
                int gapFood = (farCapFood > currentFarFood)
                                  ? (farCapFood - currentFarFood)
                                  : 0;
                int actualDonationFood = std::min(totalDonationFood, gapFood);

                if (actualDonationFood > 0 && totalDonationFood > 0) {
                    for (size_t i = 0; i < trip.drops.size(); ++i) {
                        if (i == farIndex)
                            continue;
                        if (donationFood[i] > 0) {
                            int removal = static_cast<int>(std::round(
                                donationFood[i] *
                                (actualDonationFood /
                                 static_cast<double>(totalDonationFood))));
                            int currentFood = trip.drops[i].dry_food +
                                              trip.drops[i].perishable_food;
                            if (currentFood > 0) {
                                double ratioDry =
                                    trip.drops[i].dry_food /
                                    static_cast<double>(currentFood);
                                int removeDry = static_cast<int>(
                                    std::round(removal * ratioDry));
                                int removePerc = removal - removeDry;
                                trip.drops[i].dry_food -= removeDry;
                                trip.drops[i].perishable_food -= removePerc;
                                if (trip.drops[i].dry_food < 0)
                                    trip.drops[i].dry_food = 0;
                                if (trip.drops[i].perishable_food < 0)
                                    trip.drops[i].perishable_food = 0;
                            }
                        }
                    }
                    if (currentFarFood > 0) {
                        double ratioDryFar =
                            trip.drops[farIndex].dry_food /
                            static_cast<double>(currentFarFood);
                        int addDry = static_cast<int>(
                            std::round(actualDonationFood * ratioDryFar));
                        int addPerc = actualDonationFood - addDry;
                        trip.drops[farIndex].dry_food += addDry;
                        trip.drops[farIndex].perishable_food += addPerc;
                    } else {
                        trip.drops[farIndex].dry_food += actualDonationFood;
                    }
                    changed = true;
                }

                int totalDonationOther = 0;
                std::vector<int> donationOther(trip.drops.size(), 0);
                for (size_t i = 0; i < trip.drops.size(); ++i) {
                    if (i == farIndex)
                        continue;
                    int currentOther = trip.drops[i].other_supplies;
                    int donate =
                        static_cast<int>(donationFactor * currentOther);
                    donationOther[i] = donate;
                    totalDonationOther += donate;
                }
                int farCapOther =
                    problemData.villages[farVillageId - 1].population * 1;
                int currentFarOther = trip.drops[farIndex].other_supplies;
                int gapOther = (farCapOther > currentFarOther)
                                   ? (farCapOther - currentFarOther)
                                   : 0;
                int actualDonationOther =
                    std::min(totalDonationOther, gapOther);

                if (actualDonationOther > 0 && totalDonationOther > 0) {
                    for (size_t i = 0; i < trip.drops.size(); ++i) {
                        if (i == farIndex)
                            continue;
                        if (donationOther[i] > 0) {
                            int removal = static_cast<int>(std::round(
                                donationOther[i] *
                                (actualDonationOther /
                                 static_cast<double>(totalDonationOther))));
                            trip.drops[i].other_supplies -= removal;
                            if (trip.drops[i].other_supplies < 0)
                                trip.drops[i].other_supplies = 0;
                        }
                    }
                    trip.drops[farIndex].other_supplies += actualDonationOther;
                    changed = true;
                }

                trip.dry_food_pickup = 0;
                trip.perishable_food_pickup = 0;
                trip.other_supplies_pickup = 0;
                for (const auto &drop : trip.drops) {
                    trip.dry_food_pickup += drop.dry_food;
                    trip.perishable_food_pickup += drop.perishable_food;
                    trip.other_supplies_pickup += drop.other_supplies;
                }

                std::map<int, std::array<int, 3>> new_contrib;
                for (const auto &drop : trip.drops) {
                    auto &c = new_contrib[drop.village_id];
                    c[0] += drop.dry_food;
                    c[1] += drop.perishable_food;
                    c[2] += drop.other_supplies;
                }

                double new_capped_sum = 0.0;
                for (int vid : affected) {
                    int v = vid - 1;
                    std::array<int, 3> temp = {
                        village_delivered[v][0] - old_contrib[vid][0] +
                            new_contrib[vid][0],
                        village_delivered[v][1] - old_contrib[vid][1] +
                            new_contrib[vid][1],
                        village_delivered[v][2] - old_contrib[vid][2] +
                            new_contrib[vid][2]};
                    new_capped_sum += compute_village_value(
                        {temp[0], temp[1], temp[2]}, problemData,
                        problemData.villages[v].population);
                }

                double delta = new_capped_sum - old_capped_sum;
                if (!improve_only || delta > 0) {
                    changed = true;
                    for (int vid : affected) {
                        int v = vid - 1;
                        village_delivered[v][0] = village_delivered[v][0] -
                                                  old_contrib[vid][0] +
                                                  new_contrib[vid][0];
                        village_delivered[v][1] = village_delivered[v][1] -
                                                  old_contrib[vid][1] +
                                                  new_contrib[vid][1];
                        village_delivered[v][2] = village_delivered[v][2] -
                                                  old_contrib[vid][2] +
                                                  new_contrib[vid][2];
                    }
                } else {
                    trip.drops = std::move(old_drops);
                    trip.dry_food_pickup = 0;
                    trip.perishable_food_pickup = 0;
                    trip.other_supplies_pickup = 0;
                    for (const auto &drop : trip.drops) {
                        trip.dry_food_pickup += drop.dry_food;
                        trip.perishable_food_pickup += drop.perishable_food;
                        trip.other_supplies_pickup += drop.other_supplies;
                    }
                }
            }
        }
        if (changed) {
            return reallocatedPlans;
        }
    }
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

        double heli_total_dist = 0.0;
        for (const auto &trip : plan.trips) {
            heli_total_dist += compute_trip_dist(trip, home, problemData);
        }

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

        double old_dist_s = sourceIt->distanceLoad;
        double old_dist_t = targetIt->distanceLoad;

        double old_weight_s = sourceIt->weightLoad;
        double old_weight_t = targetIt->weightLoad;

        Drop selectedDrop = std::move(sourceTrip.drops[dropIndex]);
        int df = selectedDrop.dry_food;
        int pf = selectedDrop.perishable_food;
        int os = selectedDrop.other_supplies;
        sourceTrip.dry_food_pickup -= df;
        sourceTrip.perishable_food_pickup -= pf;
        sourceTrip.other_supplies_pickup -= os;
        targetTrip.dry_food_pickup += df;
        targetTrip.perishable_food_pickup += pf;
        targetTrip.other_supplies_pickup += os;
        sourceTrip.drops.erase(sourceTrip.drops.begin() + dropIndex);
        targetTrip.drops.push_back(std::move(selectedDrop));

        double new_dist_s = compute_trip_dist(sourceTrip, home, problemData);
        double new_dist_t = compute_trip_dist(targetTrip, home, problemData);
        double delta_dist = new_dist_s + new_dist_t - old_dist_s - old_dist_t;
        double new_heli_total = heli_total_dist + delta_dist;

        double new_weight_s =
            sourceTrip.dry_food_pickup * problemData.packages[0].weight +
            sourceTrip.perishable_food_pickup * problemData.packages[1].weight +
            sourceTrip.other_supplies_pickup * problemData.packages[2].weight;
        double new_weight_t =
            targetTrip.dry_food_pickup * problemData.packages[0].weight +
            targetTrip.perishable_food_pickup * problemData.packages[1].weight +
            targetTrip.other_supplies_pickup * problemData.packages[2].weight;

        bool valid = (new_dist_s <= heli.distance_capacity) &&
                     (new_dist_t <= heli.distance_capacity) &&
                     (new_heli_total <= problemData.d_max) &&
                     (new_weight_s <= heli.weight_capacity) &&
                     (new_weight_t <= heli.weight_capacity);

        if (valid) {
            double delta_cost = heli.alpha * delta_dist;
            double delta_obj = -delta_cost;
            if (!improve_only || delta_obj > 0) {
                return movedPlans;
            }
        }
        // Revert if not accepted
        targetTrip.drops.pop_back();
        sourceTrip.drops.insert(sourceTrip.drops.begin() + dropIndex, std::move(selectedDrop));
        sourceTrip.dry_food_pickup += df;
        sourceTrip.perishable_food_pickup += pf;
        sourceTrip.other_supplies_pickup += os;
        targetTrip.dry_food_pickup -= df;
        targetTrip.perishable_food_pickup -= pf;
        targetTrip.other_supplies_pickup -= os;
    }
    return HelicopterPlans;
}

Solution split_trip(const Solution &sol, const ProblemData &problemData,
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

        if (plan.trips.empty())
            continue;

        std::uniform_int_distribution<size_t> t_dist(0, plan.trips.size() - 1);
        size_t t_idx = t_dist(gen);
        auto &trip = plan.trips[t_idx];

        if (trip.drops.size() <= 1)
            continue;

        const Helicopter &heli =
            problemData.helicopters[plan.helicopter_id - 1];
        const Point home = problemData.cities[heli.home_city_id - 1];

        double heli_total_dist = 0.0;
        for (const auto &tr : plan.trips) {
            heli_total_dist += compute_trip_dist(tr, home, problemData);
        }

        double old_dist = compute_trip_dist(trip, home, problemData);

        std::uniform_int_distribution<size_t> d_dist(0, trip.drops.size() - 1);
        size_t d_idx = d_dist(gen);

        Drop d = std::move(trip.drops[d_idx]);
        trip.drops.erase(trip.drops.begin() + d_idx);

        trip.dry_food_pickup -= d.dry_food;
        trip.perishable_food_pickup -= d.perishable_food;
        trip.other_supplies_pickup -= d.other_supplies;

        Trip newTrip;
        newTrip.dry_food_pickup = d.dry_food;
        newTrip.perishable_food_pickup = d.perishable_food;
        newTrip.other_supplies_pickup = d.other_supplies;
        newTrip.drops.push_back(std::move(d));

        plan.trips.push_back(std::move(newTrip));

        double new_old_dist = compute_trip_dist(trip, home, problemData);
        double new_trip_dist =
            compute_trip_dist(plan.trips.back(), home, problemData);
        double delta_dist = new_old_dist + new_trip_dist - old_dist;
        double new_heli_total = heli_total_dist + delta_dist;

        double new_old_weight =
            trip.dry_food_pickup * problemData.packages[0].weight +
            trip.perishable_food_pickup * problemData.packages[1].weight +
            trip.other_supplies_pickup * problemData.packages[2].weight;
        double new_trip_weight =
            plan.trips.back().dry_food_pickup * problemData.packages[0].weight +
            plan.trips.back().perishable_food_pickup *
                problemData.packages[1].weight +
            plan.trips.back().other_supplies_pickup *
                problemData.packages[2].weight;

        bool valid = (new_old_dist <= heli.distance_capacity) &&
                     (new_trip_dist <= heli.distance_capacity) &&
                     (new_heli_total <= problemData.d_max) &&
                     (new_old_weight <= heli.weight_capacity) &&
                     (new_trip_weight <= heli.weight_capacity);

        if (valid) {
            double delta_cost = heli.fixed_cost + heli.alpha * delta_dist;
            double delta_obj = -delta_cost;
            if (!improve_only || delta_obj > 0) {
                return neighbor;
            }
        }
        // Revert if not accepted
        plan.trips.pop_back();
        trip.drops.insert(trip.drops.begin() + d_idx, std::move(d));
        trip.dry_food_pickup += d.dry_food;
        trip.perishable_food_pickup += d.perishable_food;
        trip.other_supplies_pickup += d.other_supplies;
    }
    return sol;
}

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

        double heli_total_dist = 0.0;
        for (const auto &tr : plan.trips) {
            heli_total_dist += compute_trip_dist(tr, home, problemData);
        }

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

        auto trip1 = plan.trips[t1]; // Copy to revert if needed
        auto trip2 = plan.trips[t2];

        double old_dist1 = compute_trip_dist(trip1, home, problemData);
        double old_dist2 = compute_trip_dist(trip2, home, problemData);

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

        double merged_dist = compute_trip_dist(merged, home, problemData);

        plan.trips.erase(plan.trips.begin() + t2);
        plan.trips.erase(plan.trips.begin() + t1);
        plan.trips.push_back(std::move(merged));

        double delta_dist = merged_dist - (old_dist1 + old_dist2);
        double new_heli_total = heli_total_dist + delta_dist;

        double merged_weight =
            merged.dry_food_pickup * problemData.packages[0].weight +
            merged.perishable_food_pickup * problemData.packages[1].weight +
            merged.other_supplies_pickup * problemData.packages[2].weight;

        bool valid = (merged_dist <= heli.distance_capacity) &&
                     (new_heli_total <= problemData.d_max) &&
                     (merged_weight <= heli.weight_capacity);

        if (valid) {
            double delta_cost = -heli.fixed_cost + heli.alpha * delta_dist;
            double delta_obj = -delta_cost;
            if (!improve_only || delta_obj > 0) {
                return neighbor;
            }
        }
        // Revert
        plan.trips.pop_back();
        plan.trips.insert(plan.trips.begin() + t1, std::move(trip1));
        plan.trips.insert(plan.trips.begin() + t2, std::move(trip2));
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
        if (addedPlans.empty())
            continue;

        std::vector<std::vector<int>> village_delivered(
            problemData.villages.size(), std::vector<int>(3, 0));
        for (const auto &plan : HelicopterPlans) {
            for (const auto &trip : plan.trips) {
                for (const auto &drop : trip.drops) {
                    int v = drop.village_id - 1;
                    village_delivered[v][0] += drop.dry_food;
                    village_delivered[v][1] += drop.perishable_food;
                    village_delivered[v][2] += drop.other_supplies;
                }
            }
        }

        std::vector<size_t> underserved;
        for (size_t v = 0; v < problemData.villages.size(); ++v) {
            double current_val = compute_village_value(village_delivered[v], problemData, problemData.villages[v].population);
            double max_val = compute_max_village_value(problemData, problemData.villages[v].population);
            if (current_val < max_val) {
                underserved.push_back(v);
            }
        }
        if (underserved.empty())
            continue;

        std::uniform_int_distribution<size_t> heli_dist(0, addedPlans.size() - 1);
        size_t h = heli_dist(gen);
        auto &plan = addedPlans[h];

        if (plan.trips.empty())
            continue;

        std::uniform_int_distribution<size_t> t_dist(0, plan.trips.size() - 1);
        size_t t = t_dist(gen);
        auto &trip = plan.trips[t];

        if (trip.drops.empty())
            continue;

        const Helicopter &heli =
            problemData.helicopters[plan.helicopter_id - 1];
        const Point home = problemData.cities[heli.home_city_id - 1];

        double heli_total_dist = 0.0;
        for (const auto &tr : plan.trips) {
            heli_total_dist += compute_trip_dist(tr, home, problemData);
        }

        double old_dist = compute_trip_dist(trip, home, problemData);
        double old_weight = trip.dry_food_pickup * problemData.packages[0].weight +
                            trip.perishable_food_pickup * problemData.packages[1].weight +
                            trip.other_supplies_pickup * problemData.packages[2].weight;

        std::uniform_int_distribution<size_t> pos_dist(0, trip.drops.size());
        size_t insert_pos = pos_dist(gen);

        std::uniform_int_distribution<size_t> u_dist(0, underserved.size() - 1);
        size_t u_idx = u_dist(gen);
        int new_vid = underserved[u_idx] + 1; // 1-based

        // Check if already in trip
        bool already_in_trip = false;
        for (const auto& drop : trip.drops) {
            if (drop.village_id == new_vid) {
                already_in_trip = true;
                break;
            }
        }
        if (already_in_trip) continue;

        Drop new_drop{new_vid, 0, 0, 0};
        trip.drops.insert(trip.drops.begin() + insert_pos, new_drop);

        double new_dist = compute_trip_dist(trip, home, problemData);
        double delta_dist = new_dist - old_dist;
        double new_heli_total = heli_total_dist + delta_dist;

        if (new_dist > heli.distance_capacity || new_heli_total > problemData.d_max) {
            trip.drops.erase(trip.drops.begin() + insert_pos);
            continue;
        }

        // Now, reallocate to the new drop
        std::uniform_real_distribution<double> factor_dist(0.1, 0.9);
        double donationFactor = factor_dist(gen);

        std::vector<Drop> old_drops = trip.drops;
        std::map<int, std::array<int, 3>> old_contrib;
        for (const auto &drop : old_drops) {
            auto &c = old_contrib[drop.village_id];
            c[0] += drop.dry_food;
            c[1] += drop.perishable_food;
            c[2] += drop.other_supplies;
        }
        std::set<int> affected;
        for (const auto &kv : old_contrib)
            affected.insert(kv.first);

        double old_capped_sum = 0.0;
        for (int vid : affected) {
            int v = vid - 1;
            old_capped_sum += compute_village_value(
                village_delivered[v], problemData,
                problemData.villages[v].population);
        }

        // The new drop is at insert_pos
        int totalDonationFood = 0;
        std::vector<int> donationFood(trip.drops.size(), 0);
        for (size_t i = 0; i < trip.drops.size(); ++i) {
            if (i == insert_pos)
                continue;
            int currentFood =
                trip.drops[i].dry_food + trip.drops[i].perishable_food;
            int donate = static_cast<int>(donationFactor * currentFood);
            donationFood[i] = donate;
            totalDonationFood += donate;
        }

        int newCapFood =
            problemData.villages[new_vid - 1].population * 9;
        int currentNewFood = trip.drops[insert_pos].dry_food +
                             trip.drops[insert_pos].perishable_food;
        int gapFood = (newCapFood > currentNewFood)
                          ? (newCapFood - currentNewFood)
                          : 0;
        int actualDonationFood = std::min(totalDonationFood, gapFood);

        bool changed = false;
        if (actualDonationFood > 0 && totalDonationFood > 0) {
            for (size_t i = 0; i < trip.drops.size(); ++i) {
                if (i == insert_pos)
                    continue;
                if (donationFood[i] > 0) {
                    int removal = static_cast<int>(std::round(
                        donationFood[i] *
                        (actualDonationFood /
                         static_cast<double>(totalDonationFood))));
                    int currentFood = trip.drops[i].dry_food +
                                      trip.drops[i].perishable_food;
                    if (currentFood > 0) {
                        double ratioDry =
                            trip.drops[i].dry_food /
                            static_cast<double>(currentFood);
                        int removeDry = static_cast<int>(
                            std::round(removal * ratioDry));
                        int removePerc = removal - removeDry;
                        trip.drops[i].dry_food -= removeDry;
                        trip.drops[i].perishable_food -= removePerc;
                        if (trip.drops[i].dry_food < 0)
                            trip.drops[i].dry_food = 0;
                        if (trip.drops[i].perishable_food < 0)
                            trip.drops[i].perishable_food = 0;
                        changed = true;
                    }
                }
            }
            if (currentNewFood > 0) {
                double ratioDryNew =
                    trip.drops[insert_pos].dry_food /
                    static_cast<double>(currentNewFood);
                int addDry = static_cast<int>(
                    std::round(actualDonationFood * ratioDryNew));
                int addPerc = actualDonationFood - addDry;
                trip.drops[insert_pos].dry_food += addDry;
                trip.drops[insert_pos].perishable_food += addPerc;
            } else {
                trip.drops[insert_pos].perishable_food += actualDonationFood; // Prefer perishable
            }
        }

        int totalDonationOther = 0;
        std::vector<int> donationOther(trip.drops.size(), 0);
        for (size_t i = 0; i < trip.drops.size(); ++i) {
            if (i == insert_pos)
                continue;
            int currentOther = trip.drops[i].other_supplies;
            int donate =
                static_cast<int>(donationFactor * currentOther);
            donationOther[i] = donate;
            totalDonationOther += donate;
        }
        int newCapOther =
            problemData.villages[new_vid - 1].population * 1;
        int currentNewOther = trip.drops[insert_pos].other_supplies;
        int gapOther = (newCapOther > currentNewOther)
                           ? (newCapOther - currentNewOther)
                           : 0;
        int actualDonationOther =
            std::min(totalDonationOther, gapOther);

        if (actualDonationOther > 0 && totalDonationOther > 0) {
            for (size_t i = 0; i < trip.drops.size(); ++i) {
                if (i == insert_pos)
                    continue;
                if (donationOther[i] > 0) {
                    int removal = static_cast<int>(std::round(
                        donationOther[i] *
                        (actualDonationOther /
                         static_cast<double>(totalDonationOther))));
                    trip.drops[i].other_supplies -= removal;
                    if (trip.drops[i].other_supplies < 0)
                        trip.drops[i].other_supplies = 0;
                    changed = true;
                }
            }
            trip.drops[insert_pos].other_supplies += actualDonationOther;
        }

        if (!changed) {
            trip.drops = old_drops;
            continue;
        }

        // Update pickups
        trip.dry_food_pickup = 0;
        trip.perishable_food_pickup = 0;
        trip.other_supplies_pickup = 0;
        for (const auto &drop : trip.drops) {
            trip.dry_food_pickup += drop.dry_food;
            trip.perishable_food_pickup += drop.perishable_food;
            trip.other_supplies_pickup += drop.other_supplies;
        }

        // Compute new value
        std::map<int, std::array<int, 3>> new_contrib;
        for (const auto &drop : trip.drops) {
            auto &c = new_contrib[drop.village_id];
            c[0] += drop.dry_food;
            c[1] += drop.perishable_food;
            c[2] += drop.other_supplies;
        }

        double new_capped_sum = 0.0;
        for (int vid : affected) {
            int v = vid - 1;
            std::array<int, 3> temp = {
                village_delivered[v][0] - old_contrib[vid][0] +
                    new_contrib[vid][0],
                village_delivered[v][1] - old_contrib[vid][1] +
                    new_contrib[vid][1],
                village_delivered[v][2] - old_contrib[vid][2] +
                    new_contrib[vid][2]};
            new_capped_sum += compute_village_value(
                {temp[0], temp[1], temp[2]}, problemData,
                problemData.villages[v].population);
        }
        if (new_contrib.count(new_vid)) {
            int v = new_vid - 1;
            if (affected.find(new_vid) == affected.end()) {
                std::array<int, 3> temp = {
                    village_delivered[v][0] + new_contrib[new_vid][0],
                    village_delivered[v][1] + new_contrib[new_vid][1],
                    village_delivered[v][2] + new_contrib[new_vid][2]};
                new_capped_sum += compute_village_value(
                    {temp[0], temp[1], temp[2]}, problemData,
                    problemData.villages[v].population);
                new_capped_sum -= compute_village_value(village_delivered[v], problemData,
                                                        problemData.villages[v].population);
            }
        }

        double delta_value = new_capped_sum - old_capped_sum;
        double delta_cost = heli.alpha * delta_dist;
        double delta_obj = delta_value - delta_cost;

        if (!improve_only || delta_obj > 0) {
            return addedPlans;
        }

        // Revert
        trip.drops = std::move(old_drops);
        trip.dry_food_pickup = 0;
        trip.perishable_food_pickup = 0;
        trip.other_supplies_pickup = 0;
        for (const auto &drop : trip.drops) {
            trip.dry_food_pickup += drop.dry_food;
            trip.perishable_food_pickup += drop.perishable_food;
            trip.other_supplies_pickup += drop.other_supplies;
        }
    }
    return HelicopterPlans;
}

Solution remove_village(const Solution &HelicopterPlans,
                        const ProblemData &problemData, double current_obj,
                        bool improve_only = true) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int attempt = 0; attempt < 10; ++attempt) {
        Solution removedPlans = HelicopterPlans;
        if (removedPlans.empty())
            continue;

        std::vector<std::vector<int>> village_delivered(
            problemData.villages.size(), std::vector<int>(3, 0));
        for (const auto &plan : HelicopterPlans) {
            for (const auto &trip : plan.trips) {
                for (const auto &drop : trip.drops) {
                    int v = drop.village_id - 1;
                    village_delivered[v][0] += drop.dry_food;
                    village_delivered[v][1] += drop.perishable_food;
                    village_delivered[v][2] += drop.other_supplies;
                }
            }
        }

        std::uniform_int_distribution<size_t> heli_dist(0, removedPlans.size() - 1);
        size_t h = heli_dist(gen);
        auto &plan = removedPlans[h];

        if (plan.trips.empty())
            continue;

        std::uniform_int_distribution<size_t> t_dist(0, plan.trips.size() - 1);
        size_t t = t_dist(gen);
        auto &trip = plan.trips[t];

        if (trip.drops.size() < 2) // Keep at least one
            continue;

        const Helicopter &heli =
            problemData.helicopters[plan.helicopter_id - 1];
        const Point home = problemData.cities[heli.home_city_id - 1];

        double heli_total_dist = 0.0;
        for (const auto &tr : plan.trips) {
            heli_total_dist += compute_trip_dist(tr, home, problemData);
        }

        double old_dist = compute_trip_dist(trip, home, problemData);

        std::uniform_int_distribution<size_t> d_dist(0, trip.drops.size() - 1);
        size_t remove_idx = d_dist(gen);

        Drop removed_drop = std::move(trip.drops[remove_idx]);
        trip.drops.erase(trip.drops.begin() + remove_idx);

        double new_dist = compute_trip_dist(trip, home, problemData);
        double delta_dist = new_dist - old_dist;
        double new_heli_total = heli_total_dist + delta_dist;

        // Redistribute the removed drop's packages to other drops
        int df_to_redist = removed_drop.dry_food;
        int pf_to_redist = removed_drop.perishable_food;
        int os_to_redist = removed_drop.other_supplies;

        std::vector<size_t> candidates;
        for (size_t i = 0; i < trip.drops.size(); ++i) {
            int vid = trip.drops[i].village_id - 1;
            int current_food = village_delivered[vid][0] + village_delivered[vid][1];
            int cap_food = problemData.villages[vid].population * 9;
            if (current_food < cap_food) {
                candidates.push_back(i);
            }
        }

        if (!candidates.empty()) {
            std::uniform_int_distribution<size_t> c_dist(0, candidates.size() - 1);
            size_t target_idx = candidates[c_dist(gen)];
            trip.drops[target_idx].dry_food += df_to_redist;
            trip.drops[target_idx].perishable_food += pf_to_redist;
            trip.drops[target_idx].other_supplies += os_to_redist;
        } else {
            // If no candidates, discard (value loss)
        }

        trip.dry_food_pickup = 0;
        trip.perishable_food_pickup = 0;
        trip.other_supplies_pickup = 0;
        for (const auto& drop : trip.drops) {
            trip.dry_food_pickup += drop.dry_food;
            trip.perishable_food_pickup += drop.perishable_food;
            trip.other_supplies_pickup += drop.other_supplies;
        }

        double new_weight = trip.dry_food_pickup * problemData.packages[0].weight +
                            trip.perishable_food_pickup * problemData.packages[1].weight +
                            trip.other_supplies_pickup * problemData.packages[2].weight;

        bool valid = (new_dist <= heli.distance_capacity) &&
                     (new_heli_total <= problemData.d_max) &&
                     (new_weight <= heli.weight_capacity);

        if (!valid) {
            // Revert not implemented for simplicity, skip
            continue;
        }

        // Compute delta_obj
        std::set<int> affected;
        affected.insert(removed_drop.village_id);
        for (const auto& drop : trip.drops) {
            affected.insert(drop.village_id);
        }

        double old_capped_sum = 0.0;
        for (int vid : affected) {
            int v = vid - 1;
            old_capped_sum += compute_village_value(village_delivered[v], problemData,
                                                    problemData.villages[v].population);
        }

        // Simulate new delivered
        for (int vid : affected) {
            village_delivered[vid - 1][0] = 0;
            village_delivered[vid - 1][1] = 0;
            village_delivered[vid - 1][2] = 0;
        }
        for (const auto &p : removedPlans) {
            if (p.helicopter_id == plan.helicopter_id) continue;
            for (const auto &tr : p.trips) {
                for (const auto &drop : tr.drops) {
                    int v = drop.village_id - 1;
                    if (affected.count(drop.village_id)) {
                        village_delivered[v][0] += drop.dry_food;
                        village_delivered[v][1] += drop.perishable_food;
                        village_delivered[v][2] += drop.other_supplies;
                    }
                }
            }
        }
        for (const auto &tr : plan.trips) {
            for (const auto &drop : tr.drops) {
                int v = drop.village_id - 1;
                if (affected.count(drop.village_id)) {
                    village_delivered[v][0] += drop.dry_food;
                    village_delivered[v][1] += drop.perishable_food;
                    village_delivered[v][2] += drop.other_supplies;
                }
            }
        }

        double new_capped_sum = 0.0;
        for (int vid : affected) {
            int v = vid - 1;
            new_capped_sum += compute_village_value(village_delivered[v], problemData,
                                                    problemData.villages[v].population);
        }

        double delta_value = new_capped_sum - old_capped_sum;
        double delta_cost = heli.alpha * delta_dist;
        double delta_obj = delta_value - delta_cost;

        if (!improve_only || delta_obj > 0) {
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
    std::vector<double> weights = {10, 8, 6, 5, 4, 3, 10, 5}; // Non-uniform, higher for reorder, reallocate, add_new
    std::discrete_distribution<int> op_dist(weights.begin(), weights.end());
    int op = op_dist(gen);

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