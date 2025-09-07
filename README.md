# Agent-Based Modelling of Taxiing Operations
This writeup contains an agent-based modelling project that explored three different airport taxiway co-ordination solutions: prioritised, conflict-based search and a distributed approach.
![Airport Simulation](Airport.gif)

## Problem Statement
You're given a small airport with one departure runway, one arrival runway and five terminals. The challenge is to construct and compare aircraft taxiing co-ordination solutions subject to the following constraints:
1. There are no collisions between aircraft
2. All aircraft move at the same speed
3. Aircraft are not allowed to backtrack
4. There is a maximum number of aircraft on the taxiways, determined by the demand scenario

The problem is best understood by taking a look at the grid-based airport illustrated below. 
![Airport Diagram](Airport.png)
- **Arriving aircraft** start from the arrival runway, entering the grid at nodes {37,38} and look to exit at one of the airport terminal nodes {34, 35, 36, 97, 98}.
- **Departing aircraft** start at terminals, entering the grid at nodes {34, 35, 36, 97, 98} and look to exit at the departure runway nodes {1,2}.
#### Simulation details
A few clarifying remarks are given here about the simulation behaviour.
##### Aircraft Spawning
Aircraft are spawned as either arrival or departing aircraft with a fixed cooldown between spawns. If the number of aircraft on the runway is below the maximum, there's a 45% chance that it will spawn as arriving, a 45% chance it will spawn as departing and a 10% chance that an arriving and departing aircraft spawn simultaneously.
##### Aircraft Exiting
Once an aircraft reaches its goal state (terminal for arriving, departure runway for departing), the aircraft instantaneously leaves the grid. Of course, in reality this is not accurate as for example there's a gap between when an aircraft uses a runway to depart and when it's clear for departure again.
##### Aircraft Rejection
If an aircraft spawns but finds no possible path plan to reach its goal state from any candidate starting node, it's simply rejected from entering the grid. In reality this would correspond to delaying an arrival/departure until it is logistically suitable. 
##### Demand Scenarios
The two demand scenarios considered were nominal demand and high demand. 
For normal demand, the maximum number of aircraft on the taxiways was ≤ 5 and the simulation ran until a quota of 50 (or 51 if simultaneous spawning occurred) aircraft was met.

For high demand, the maximum number of aircraft on the taxiways was ≤ 6 and the simulation ran until a quota of 80 (or 81 if spawned simultaneously) aircraft was met. The marginal increase in the number of aircraft on the taxiways is due to the fact that the airport is small, congested and any further increase on the number simply results on a higher number of aircraft rejections.
## Strategies
Below is a brief synopsis of how each solution strategy worked.
#### Prioritised
Prioritised planning worked in a first-come first-serve manner, meaning whichever aircraft entered first would simply plan out the shortest path (using A*) and set constraints banning subsequent aircraft from being at the same place and same time. If an aircraft spawned but no suitable path was found for it, it was rejected.

#### Conflict-Based Search (CBS)
For conflict-based search, there was no inherent priority. Instead aircraft plan their shortest routes (using A*) every time a new aircraft spawns and if a conflict is found between paths, it's added to a list of constraints and the shortest paths re-calculated until either:
- A resolution is found → aircraft is allowed to enter the grid, all aircraft adjust their routes
- No resolution is found → aircraft is not allowed to spawn, routes remain unadjusted

#### Distributed
For the distributed strategy, the agents co-ordinated continuously by changing the weights of neighbouring nodes in order to signal nearby aircraft of their intentions. If an aircraft was within its vicinity, the weights of surrounding nodes would inform both aircraft about their plans and collisions would be avoided implicitly through co-ordination rules. The rules were based on pre-established protocols such as imposed constraints (no backtracking) and movement constraints like blocking out an edge once an aircraft decides they'll take it and the right of way at intersections.

In real life this could be implemented by means of a radar that signals aircraft close to each other how they should proceed on the taxiways in a co-ordinated collision free way. The information that is used by every plane to decide their next move one is never more than four edges away. This is enough information to prevent the planes from colliding or being stuck.

## Performance Indicators
The following metrics were tracked in order to compare strategies:
- Total taxi time, to track how long it took the strategy to meet the quota
- Average route time per aircraft, to track how direct routes were
- Maximum route time per run, to track if some strategies consistently made some aircraft take longer routes
- Maximum algorithm calculation time, to track the worst-case scenario computing power per strategy

The simulation was run 100 times per strategy (all coefficients of variance converged within 100 trials) to get a representative picture of how each strategy performed. The analysis and sensitivity analysis of these metrics were documented in a report and are left out of this write-up for brevity.
