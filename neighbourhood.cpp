#include<vector>
#include<iostream>
#include<map>
#include<queue>
#include "structures.h"
#include <random>
#include "neighbourhood.h"
Solution reorder_visits(Solution & HelicopterPlans, const ProblemData & problemData) {
    Solution reorderedPlans = HelicopterPlans; // Create a copy to store reordered plans
    for (auto & helicopterPlan : reorderedPlans) {
        for (auto & trip : helicopterPlan.trips) {
            std::sort(trip.drops.begin(), trip.drops.end(), [&](const Drop & a, const Drop & b) {
                const Point & pointA = problemData.villages[a.village_id].coords;
                const Point & pointB = problemData.villages[b.village_id].coords;
                const Point & homeCity = problemData.cities[problemData.helicopters[helicopterPlan.helicopter_id].home_city_id];
                return distance(homeCity, pointA) < distance(homeCity, pointB);
            });
        }
    }

    return reorderedPlans;
}




Solution reallocate_packages(Solution & HelicopterPlans, const ProblemData & problemData) {
    // Tunable donation factor (50% of packets are reallocated by default)
    const double donationFactor = 0.5;
    Solution reallocatedPlans = HelicopterPlans; // Create a copy to store reallocated plans
    
    // For each helicopter plan
    for (auto & plan : reallocatedPlans) {
        // Get the helicopter's starting city coordinates (both are 1-indexed)
        const Helicopter &heli = problemData.helicopters[plan.helicopter_id - 1];
        const Point home = problemData.cities[heli.home_city_id - 1];
        
        // Process each trip
        for (auto & trip : plan.trips) {
            if(trip.drops.empty())
                continue;
            
            // Identify the farthest drop from the start city
            int farIndex = 0;
            double maxDist = 0.0;
            for (size_t i = 0; i < trip.drops.size(); i++) {
                int vid = trip.drops[i].village_id; // village ids are 1-indexed
                const Point & villagePoint = problemData.villages[vid - 1].coords;
                double d = distance(home, villagePoint);
                if(d > maxDist) {
                    maxDist = d;
                    farIndex = i;
                }
            }
            
            // ---------------- Reallocate Food Packets ----------------
            // Food is the sum of dry_food and perishable_food.
            int totalDonationFood = 0;
            std::vector<int> donationFood(trip.drops.size(), 0);
            // For every drop except the farthest, decide donation = floor(factor * currentFood)
            for (size_t i = 0; i < trip.drops.size(); i++) {
                if(i == farIndex) continue;
                int currentFood = trip.drops[i].dry_food + trip.drops[i].perishable_food;
                int donate = static_cast<int>(donationFactor * currentFood);
                donationFood[i] = donate;
                totalDonationFood += donate;
            }
            
            // Determine how many food packets the far drop can accept without reaching excess.
            int farVillageId = trip.drops[farIndex].village_id;
            int farCapFood = problemData.villages[farVillageId - 1].population * 9;  // beneficial cap for food
            int currentFarFood = trip.drops[farIndex].dry_food + trip.drops[farIndex].perishable_food;
            int gapFood = (farCapFood > currentFarFood) ? (farCapFood - currentFarFood) : 0;
            int actualDonationFood = std::min(totalDonationFood, gapFood);
            
            if(actualDonationFood > 0 && totalDonationFood > 0) {
                // Remove donation packets from each near drop proportionally.
                for (size_t i = 0; i < trip.drops.size(); i++){
                    if(i == farIndex) continue;
                    if(donationFood[i] > 0) {
                        int removal = static_cast<int>(std::round(donationFood[i] * (actualDonationFood / static_cast<double>(totalDonationFood))));
                        int currentFood = trip.drops[i].dry_food + trip.drops[i].perishable_food;
                        if(currentFood > 0) {
                            // Remove proportionally from dry and perishable packets.
                            double ratioDry = trip.drops[i].dry_food / static_cast<double>(currentFood);
                            int removeDry = static_cast<int>(std::round(removal * ratioDry));
                            int removePerc = removal - removeDry; // ensure the sum is removal
                            trip.drops[i].dry_food -= removeDry;
                            trip.drops[i].perishable_food -= removePerc;
                            if(trip.drops[i].dry_food < 0)
                                trip.drops[i].dry_food = 0;
                            if(trip.drops[i].perishable_food < 0)
                                trip.drops[i].perishable_food = 0;
                        }
                    }
                }
                // Add the donated food to the far drop.
                if(currentFarFood > 0) {
                    double ratioDryFar = trip.drops[farIndex].dry_food / static_cast<double>(currentFarFood);
                    int addDry = static_cast<int>(std::round(actualDonationFood * ratioDryFar));
                    int addPerc = actualDonationFood - addDry;
                    trip.drops[farIndex].dry_food += addDry;
                    trip.drops[farIndex].perishable_food += addPerc;
                } else {
                    // If no food exists at far drop, assign all to dry_food
                    trip.drops[farIndex].dry_food += actualDonationFood;
                }
            }
            
            // ---------------- Reallocate Other Supplies Packets ----------------
            int totalDonationOther = 0;
            std::vector<int> donationOther(trip.drops.size(), 0);
            for (size_t i = 0; i < trip.drops.size(); i++){
                if(i == farIndex) continue;
                int currentOther = trip.drops[i].other_supplies;
                int donate = static_cast<int>(donationFactor * currentOther);
                donationOther[i] = donate;
                totalDonationOther += donate;
            }
            int farCapOther = problemData.villages[farVillageId - 1].population * 1;  // beneficial cap for other supplies
            int currentFarOther = trip.drops[farIndex].other_supplies;
            int gapOther = (farCapOther > currentFarOther) ? (farCapOther - currentFarOther) : 0;
            int actualDonationOther = std::min(totalDonationOther, gapOther);
            
            if(actualDonationOther > 0 && totalDonationOther > 0) {
                for (size_t i = 0; i < trip.drops.size(); i++){
                    if(i == farIndex) continue;
                    if(donationOther[i] > 0) {
                        int removal = static_cast<int>(std::round(donationOther[i] * (actualDonationOther / static_cast<double>(totalDonationOther))));
                        trip.drops[i].other_supplies -= removal;
                        if(trip.drops[i].other_supplies < 0)
                            trip.drops[i].other_supplies = 0;
                    }
                }
                trip.drops[farIndex].other_supplies += actualDonationOther;
            }
        }
    }
    
    return reallocatedPlans;
}



Solution move_visit(Solution & HelicopterPlans, const ProblemData & problemData) {
    Solution movedPlans = HelicopterPlans; // Create a copy to store modified plans
 
    // Set up random generator
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // If there are no plans, return
    if (movedPlans.empty())
        return movedPlans;
    
    // Pick a random helicopter plan
    std::uniform_int_distribution<size_t> helicopterDist(0, movedPlans.size() - 1);
    size_t heliIndex = helicopterDist(gen);
    auto &plan = movedPlans[heliIndex];
    
    // We require at least 2 trips to move a visit from one to the other
    if (plan.trips.size() < 2)
        return movedPlans;
    
    // Get helicopter details and capacities
    const Helicopter &heli = problemData.helicopters[plan.helicopter_id - 1];
    const Point home = problemData.cities[heli.home_city_id - 1];
    
    // Calculate load metrics for each trip
    struct TripLoad {
        size_t tripIndex;
        double weightLoad;       // Sum of all packages
        double distanceLoad;     // Estimate of route length
        double weightRatio;      // weightLoad / weight_capacity
        double distanceRatio;    // distanceLoad / distance_capacity
        double combinedRatio;    // max(weightRatio, distanceRatio)
    };
    
    std::vector<TripLoad> tripLoads;
    
    for (size_t i = 0; i < plan.trips.size(); i++) {
        auto &trip = plan.trips[i];
        
        // Skip empty trips
        if (trip.drops.empty())
            continue;
        
        // Calculate weight load
        double weight = 0;
        for (const auto &drop : trip.drops) {
            weight += drop.dry_food + drop.perishable_food + drop.other_supplies;
        }
        
        // Calculate distance load (approximate route length)
        double totalDistance = 0;
        Point lastPoint = home;
        for (const auto &drop : trip.drops) {
            const Point &villagePoint = problemData.villages[drop.village_id - 1].coords;
            totalDistance += distance(lastPoint, villagePoint);
            lastPoint = villagePoint;
        }
        // Add return to home city
        totalDistance += distance(lastPoint, home);
        
        // Calculate ratios
        double weightRatio = weight / heli.weight_capacity;
        double distanceRatio = totalDistance / heli.distance_capacity;
        double combinedRatio = std::max(weightRatio, distanceRatio);
        
        tripLoads.push_back({i, weight, totalDistance, weightRatio, distanceRatio, combinedRatio});
    }
    
    // If we have fewer than 2 trips with drops, we can't move anything
    if (tripLoads.size() < 2)
        return movedPlans;
    
    // Find source trip (most overloaded) and target trip (least loaded)
    auto sourceIt = std::max_element(tripLoads.begin(), tripLoads.end(),
        [](const TripLoad &a, const TripLoad &b) { return a.combinedRatio < b.combinedRatio; });
    
    auto targetIt = std::min_element(tripLoads.begin(), tripLoads.end(),
        [](const TripLoad &a, const TripLoad &b) { return a.combinedRatio < b.combinedRatio; });
    
    // If source and target are the same, pick second-least loaded as target
    if (sourceIt->tripIndex == targetIt->tripIndex) {
        std::vector<TripLoad> sorted = tripLoads;
        std::sort(sorted.begin(), sorted.end(), 
            [](const TripLoad &a, const TripLoad &b) { return a.combinedRatio < b.combinedRatio; });
        
        if (sorted.size() >= 2) {
            targetIt = std::find_if(tripLoads.begin(), tripLoads.end(),
                [&sorted](const TripLoad &load) { return load.tripIndex == sorted[1].tripIndex; });
        } else {
            return movedPlans; // Cannot find distinct source and target
        }
    }
    
    // Get source and target trips
    auto &sourceTrip = plan.trips[sourceIt->tripIndex];
    auto &targetTrip = plan.trips[targetIt->tripIndex];
    
    // Choose a random drop from the source trip
    std::uniform_int_distribution<size_t> dropDist(0, sourceTrip.drops.size() - 1);
    size_t dropIndex = dropDist(gen);
    Drop selectedDrop = sourceTrip.drops[dropIndex];
    
    // Remove the selected drop from the source trip
    sourceTrip.drops.erase(sourceTrip.drops.begin() + dropIndex);
    
    // Add the drop to the target trip
    targetTrip.drops.push_back(selectedDrop);
    
    return movedPlans;
}


Solution split_trip(Solution& sol) {
    Solution neighbor = sol;

    int h_idx = rand() % neighbor.size();
    auto& plan = neighbor[h_idx];

    if (plan.trips.empty()) return neighbor;

    int t_idx = rand() % plan.trips.size();
    auto& trip = plan.trips[t_idx];

    if (trip.drops.size() <= 1) return neighbor;

    int d_idx = rand() % trip.drops.size();
    Drop d = trip.drops[d_idx];

    trip.drops.erase(trip.drops.begin() + d_idx);

    trip.dry_food_pickup -= d.dry_food;
    trip.perishable_food_pickup -= d.perishable_food;
    trip.other_supplies_pickup -= d.other_supplies;

    Trip newTrip;
    newTrip.dry_food_pickup = d.dry_food;
    newTrip.perishable_food_pickup = d.perishable_food;
    newTrip.other_supplies_pickup = d.other_supplies;
    newTrip.drops.push_back(d);

    plan.trips.push_back(newTrip);

    return neighbor;
}

Solution merge_trips(const Solution& sol) {
    Solution neighbor = sol;

    int h_idx = rand() % neighbor.size();
    auto& plan = neighbor[h_idx];

    if (plan.trips.size() < 2) return neighbor;

    int t1 = rand() % plan.trips.size();
    int t2 = rand() % plan.trips.size();

    int cnt = 0;
    while (t1 == t2 && cnt < 5) {
        cnt++;
        t2 = rand() % plan.trips.size();
    }
    if(t1 == t2) return neighbor;

    auto& trip1 = plan.trips[t1];
    auto& trip2 = plan.trips[t2];

    Trip merged;
    merged.dry_food_pickup = trip1.dry_food_pickup + trip2.dry_food_pickup;
    merged.perishable_food_pickup = trip1.perishable_food_pickup + trip2.perishable_food_pickup;
    merged.other_supplies_pickup = trip1.other_supplies_pickup + trip2.other_supplies_pickup;

    merged.drops = trip1.drops;
    merged.drops.insert(merged.drops.end(), trip2.drops.begin(), trip2.drops.end());

    plan.trips.erase(plan.trips.begin() + max(t1, t2));
    plan.trips.erase(plan.trips.begin() + min(t1, t2));
    plan.trips.push_back(merged);

    return neighbor;
}

Solution reassign_visit(Solution& sol) {
    Solution neighbor = sol;

    if (neighbor.size() < 2) return neighbor;

    int src_h = rand() % neighbor.size();
    auto& planFrom = neighbor[src_h];

    if (planFrom.trips.empty()) return neighbor;

    int t_idx = rand() % planFrom.trips.size();
    auto& srcTrip = planFrom.trips[t_idx];

    if (srcTrip.drops.empty()) return neighbor;

    int d_idx = rand() % srcTrip.drops.size();
    Drop d = srcTrip.drops[d_idx];

    srcTrip.drops.erase(srcTrip.drops.begin() + d_idx);
    srcTrip.dry_food_pickup -= d.dry_food;
    srcTrip.perishable_food_pickup -= d.perishable_food;
    srcTrip.other_supplies_pickup -= d.other_supplies;

    if (srcTrip.drops.empty()) {
        planFrom.trips.erase(planFrom.trips.begin() + t_idx);
    }

    int dst_h = rand() % neighbor.size();
    int cnt = 0;
    while (src_h == dst_h && cnt < 5) {
        cnt++;
        dst_h = rand() % neighbor.size();
    }
    if (src_h == dst_h) return neighbor;

    auto& planTo = neighbor[dst_h];

    bool append_existing = !planTo.trips.empty() && (rand() % 5 != 0);

    if (append_existing) {
        int t_to = rand() % planTo.trips.size();
        auto& dstTrip = planTo.trips[t_to];

        int insert_pos = 0;
        if (!dstTrip.drops.empty()) {
            insert_pos = rand() % (static_cast<int>(dstTrip.drops.size()) + 1);
        }
        dstTrip.drops.insert(dstTrip.drops.begin() + insert_pos, d);

        dstTrip.dry_food_pickup += d.dry_food;
        dstTrip.perishable_food_pickup += d.perishable_food;
        dstTrip.other_supplies_pickup += d.other_supplies;
    } else {
        Trip newTrip;
        newTrip.dry_food_pickup = d.dry_food;
        newTrip.perishable_food_pickup = d.perishable_food;
        newTrip.other_supplies_pickup = d.other_supplies;
        newTrip.drops.push_back(d);
        planTo.trips.push_back(newTrip);
    }

    return neighbor;
}
