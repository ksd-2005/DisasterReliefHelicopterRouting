#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <vector>
#include <cmath>
using namespace std;

// --- GEOMETRIC & ENTITY STRUCTURES ---

struct Point {
    double x, y;
};

// --- UTILITY FUNCTIONS ---

/**
 * @brief Calculates the Euclidean distance between two points.
 */
inline double distance(const Point& p1, const Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// Tolerance for floating point comparisons.
static const double EPS = 1e-6;

// --- PROBLEM & SOLUTION STRUCTURES ---

struct PackageInfo {
    double weight, value;
};

struct Village {
    int id;
    Point coords;
    int population;
};

struct Helicopter {
    int id;
    int home_city_id;
    double weight_capacity;
    double distance_capacity;
    double fixed_cost; // F
    double alpha;      // fuel efficiency factor
};

struct ProblemData {
    double time_limit_minutes;
    double d_max;
    vector<PackageInfo> packages;
    vector<Point> cities;
    vector<Village> villages;
    vector<Helicopter> helicopters;
};

struct Drop {
    int village_id;
    int dry_food;
    int perishable_food;
    int other_supplies;
};

// Equality operator for Drop.
inline bool operator==(const Drop &lhs, const Drop &rhs) {
    return lhs.village_id == rhs.village_id &&
           lhs.dry_food == rhs.dry_food &&
           lhs.perishable_food == rhs.perishable_food &&
           lhs.other_supplies == rhs.other_supplies;
}

struct Trip {
    int dry_food_pickup;
    int perishable_food_pickup;
    int other_supplies_pickup;
    vector<Drop> drops;
    double trip_distance;
    double trip_cost;
};

// Equality operator for Trip.
inline bool operator==(const Trip &lhs, const Trip &rhs) {
    if (lhs.dry_food_pickup != rhs.dry_food_pickup ||
        lhs.perishable_food_pickup != rhs.perishable_food_pickup ||
        lhs.other_supplies_pickup != rhs.other_supplies_pickup)
        return false;
    if (lhs.drops != rhs.drops)
        return false;
    if (fabs(lhs.trip_distance - rhs.trip_distance) > EPS)
        return false;
    if (fabs(lhs.trip_cost - rhs.trip_cost) > EPS)
        return false;
    return true;
}

struct HelicopterPlan {
    int helicopter_id;
    vector<Trip> trips;
    double total_delivered_value;
    double total_trip_cost;
    double objective_value;
};

// Equality operator for HelicopterPlan.
inline bool operator==(const HelicopterPlan &lhs, const HelicopterPlan &rhs) {
    if (lhs.helicopter_id != rhs.helicopter_id)
        return false;
    if (lhs.trips != rhs.trips)
        return false;
    if (fabs(lhs.total_delivered_value - rhs.total_delivered_value) > EPS)
        return false;
    if (fabs(lhs.total_trip_cost - rhs.total_trip_cost) > EPS)
        return false;
    if (fabs(lhs.objective_value - rhs.objective_value) > EPS)
        return false;
    return true;
};

// Since Solution is defined as:
using Solution = vector<HelicopterPlan>;
// The standard vector== operator will now work because HelicopterPlan has an operator==.

#endif // STRUCTURES_H