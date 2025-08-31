#include "neighbourhood.h"

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
