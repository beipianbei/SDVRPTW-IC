# RLGHGS for SDVRPTW-IC

A C++ implementation of **RLGHGS** (Reinforcement Learning Guided Hybrid Genetic Search) for the **Split Delivery Vehicle Routing Problem with Time Windows and Commodity Incompatibility (SDVRPTW-IC)** — a rich VRP variant motivated by the commodities transportation system of a large petrochemical group.

This repository contains the source code of the algorithm described in our paper. The framework is built upon the **HGS-CVRP** algorithm of [Vidal (2022)](https://github.com/vidalt/HGS-CVRP) (Hybrid Genetic Search, MIT License), extended with demand pre-splitting, a reinforcement-learning-guided search mechanism, and hard commodity-incompatibility constraints.

---

## 1. Problem Description

The SDVRPTW-IC is defined on a directed graph $G=(V,A)$, where node $0$ is the depot and $N=\{1,\dots,n\}$ is the customer set. It jointly considers three practical features:

- **Split deliveries (SD)** — a customer's demand may be fulfilled by multiple vehicles (unavoidable when demand exceeds vehicle capacity).
- **Time windows (TW)** — service at customer $i$ must start within $[e_i, l_i]$; early arrivals wait.
- **Commodity incompatibility (CI)** — each customer requests one commodity type; conflicting type pairs $(g,g')\in H$ cannot be loaded on the same vehicle. This is enforced by a conflict customer-pair set $P$ derived from $H$.

**Objective:** minimize the total transportation cost (distance) of all vehicle routes such that every customer's demand is fully delivered, all time windows are respected, vehicle capacity is never exceeded, and no two conflicting commodities share a vehicle.

A mixed-integer programming formulation is given in the paper; small instances can be solved to optimality with Gurobi as a benchmark.

## 2. Algorithm Overview

RLGHGS keeps the population-based "genetic algorithm + local search" framework of HGS, and adds the following components:

1. **PASA — Prior Adaptive Splitting Algorithm** (`main.cpp`, `splitDemand` / `Split_Node`).
   Before the genetic search starts, each customer's demand is decomposed once into feasible loads that never exceed vehicle capacity. A greatest-common-divisor benchmark of all demands and the capacity defines a normalized unit; a geometric-series strategy (base 2) generates a descending sequence of split ratios, applied greedily; residual demand is covered by unit loads so that splits always sum exactly to the original demand. Every resulting load becomes a *virtual customer* inheriting the original coordinates, time window, and service time. E.g., capacity 100 and demand 76 → 64 + 8 + 4. The routing algorithm then treats virtual customers independently, realizing split delivery.

2. **Constructive heuristic for initial solutions** (adapted from Penna et al., 2013).
   The number of routes is estimated from aggregate demand; each route is seeded with the earliest-time-window customer; remaining customers are greedily inserted at the feasible position of least marginal cost, skipping any route that would create a commodity conflict.

3. **Reinforcement-learning-guided evolution.**
   A probability matrix records each customer's preference over possible successors and is updated by reward/penalty signals collected during neighborhood search (with a forgetting and smoothing mechanism to avoid premature convergence). This learning signal guides parent generation and crossover toward promising routing regions while preserving diversity. Parent selection combines the standard binary tournament selection (BTS) with learning-guided construction; offspring are produced by **OX crossover** (`Genetic.cpp`, `crossoverOX`) and completed by the **Split** procedure.

4. **VNSa — Variable Neighborhood Search with SWAP\*** (`LocalSearch.cpp`).
   Each offspring is refined by the two-stage local search of Vidal (2022): route improvement (RI) with nine move operators (relocations, swaps, 2-Opt / 2-Opt* variants M1–M9), restricted to granular neighborhoods of the $\Gamma$ geographically closest customers, plus the **SWAP\*** operator that exchanges two customers between routes at their best insertion positions.

5. **Strict feasibility for commodity conflicts.**
   A binary conflict matrix is built at instance-loading time (`InstanceCVRPLIB.cpp`) and consulted both by the Split procedure (`Split.cpp`) and by every neighborhood move (`LocalSearch.cpp`): any move that would place incompatible commodities on the same vehicle is rejected outright.

Population management (biased fitness balancing cost and diversity, survivor selection, adaptive penalty management, restarts) follows the original HGS design (`Population.cpp`, `Individual.cpp`).

## 3. Repository Structure

```
Program/
├── main.cpp                 Entry point; PASA demand pre-splitting (splitDemand, Split_Node); instance solving driver
├── Genetic.{h,cpp}          Genetic algorithm loop and OX crossover
├── Population.{h,cpp}       Population management, binary tournament, biased fitness, solution export
├── Individual.{h,cpp}       Solution (individual) representation and evaluation
├── LocalSearch.{h,cpp}      VNSa local search: RI moves (M1–M9) and SWAP*, with commodity-conflict checks
├── Split.{h,cpp}            Giant-tour Split procedure (feasible route extraction with conflict checks)
├── Params.{h,cpp}           Problem data and evaluation structures
├── InstanceCVRPLIB.{h,cpp}  Instance reader (extended CVRPLIB format, incl. commodity types & TYPE_CONFLICT)
├── AlgorithmParameters.{h,cpp}  Default algorithm parameters
├── commandline.h            Command-line argument parsing
├── C_Interface.{h,cpp}      C interface wrappers
└── CircleSector.h           Geometry utilities for route-sector tracking
```

Test instances live in `../Instances/`:

```
Instances/
├── SDVRPTW/                 Base split-delivery VRPTW instances (25 / 50 / 100 customers, various capacities)
├── SDVRPTW(M)/              Instances with commodity types attached to demands
└── SDVRPTW(M-5-100)/        Multi-commodity instances with TYPE_CONFLICT sections (5–100% conflict density)
```

The instances are derived from the Solomon VRPTW benchmarks (`c1xx`, `c2xx`, ...), augmented with demands, commodity types, and incompatibility pairs.

## 4. Instance Format

Instances extend the CVRPLIB text format:

```
NAME : C101
TYPE : SDVRPTW
DIMENSION : 101
EDGE_WEIGHT_TYPE : EUC_2D
CAPACITY : 100
NODE_COORD_SECTION
1 40 50
...
DEMAND_SECTION
1 0 0            <- node, demand, commodity type
2 10 3
...
TYPE_CONFLICT    <- optional section: incompatible commodity-type pairs
2 5
4 6
TIME_WINDOW_SECTION
1 0 1236         <- node, earliest, latest
...
SERVICE_TIME_SECTION
1 0
...
DEPOT_SECTION
1
-1
EOF
```

Commodity types are integers in `{1, ..., 6}` (0 = depot). Each `TYPE_CONFLICT` line declares one incompatible pair; the section ends at `TIME_WINDOW_SECTION`.

## 5. Building

Requires a **C++17** compiler. The code is developed and tested on **Windows with MSVC** (Visual Studio):

```powershell
cl /std:c++17 /O2 /EHsc *.cpp /Fe:hgs.exe
```

> Note: `main.cpp` currently includes Windows headers (`windows.h`); on Linux/macOS, remove these includes (and the Windows-only `CreateFolder`/`IterateFolder` helpers) and compile with `g++ -std=c++17 -O3 *.cpp -o hgs`.

## 6. Usage

```
./hgs instancePath solPath [options]
```

Example:

```
./hgs ../Instances/SDVRPTW(M-5-100)/100_customer/100_customer_100/c101.vrp solution.sol -t 600 -seed 0
```

In the current driver (`main.cpp`), the program is invoked with two arguments — the instance path and the output-solution path; the solver parameters (`-t 600`, `-nbGranular 30`, `-it 40000`, ...) are set in `solve_vrp_instance()` and can be adjusted there.

| Option | Meaning | Default |
|---|---|---|
| `-t <double>` | Time limit in seconds (runs iteratively until reached) | 0 (off) |
| `-it <int>` | Max iterations without improvement | 20,000 |
| `-seed <int>` | Random seed (0 = not fixed) | 0 |
| `-veh <int>` | Prescribed fleet size | unlimited |
| `-round <0/1>` | Round distances to integer | 1 |
| `-log <0/1>` | Verbose log | 1 |
| `-nbGranular <int>` | Granular neighborhood size Γ for RI moves | 20 |
| `-mu <int>` | Minimum population size | 25 |
| `-lambda <int>` | Generation size | 40 |
| `-nbElite <int>` | Number of elite individuals | 5 |
| `-nbClose <int>` | Closest individuals for diversity contribution | 4 |
| `-nbIterTraces <int>` | Iterations between log traces | 500 |
| `-nbIterPenaltyManagement <int>` | Iterations between penalty updates | 100 |
| `-targetFeasible <double>` | Target ratio of feasible individuals | 0.2 |
| `-penaltyIncrease / -penaltyDecrease <double>` | Adaptive penalty factors | 1.2 / 0.85 |

## 7. Output

The best solution is written to the solution file in the form

```
Route #1: 5 13 27 ...
Route #2: 8 21 40 ...
Cost 1234.5
```

For split-delivery runs, the console log additionally reports, per route, each (virtual) customer with its delivered quantity and any time-window violation, mapping virtual customers back to their original customer IDs.

## 8. Citation

If you use this code in your research, please cite our paper:

> *(Paper under review — citation information will be added upon publication.)*

## 9. Acknowledgments & License

This project is built upon the **HGS-CVRP** implementation by Thibaut Vidal (MIT License); the original license headers are retained in the corresponding source files. The PASA splitting scheme is adapted from Torkzaban et al. (2024), and the constructive heuristic from Penna et al. (2013).

This code is released under the **MIT License**.
