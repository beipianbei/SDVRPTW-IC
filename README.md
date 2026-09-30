# RLHGS for SDVRPTW-IC

This repository contains a C++17 implementation of **RLHGS** (reinforcement-learning-guided hybrid genetic search) for the **Split Delivery Vehicle Routing Problem with Time Windows and Commodity Incompatibility (SDVRPTW-IC)**.

SDVRPTW-IC extends vehicle routing with three practical requirements:

- **Split delivery:** the demand of an original customer may be served by more than one vehicle.
- **Hard time windows:** service must start within the specified customer time window; early arrivals may wait.
- **Commodity incompatibility:** incompatible commodity types cannot be transported by the same vehicle, while compatible commodities may share a vehicle.

The objective is to minimize total two-decimal Euclidean routing distance while satisfying exact original demand, vehicle capacity, time-window, service-time, depot, and commodity-compatibility constraints.

The current frozen algorithm version reported at startup is:

```text
Version=FINAL_DR_V1_CAP30_FIX
Objective=TWO_DECIMAL_EUCLIDEAN
SplitStrategy=CURRENT_RLHGS
WarmFraction=0.5
DRTrigger=500
DRRemovalRatio=0.15
```

## 1. Algorithm overview

The implementation retains the population-based genetic search, ordered crossover, local search, diversity management, adaptive penalties, and restart mechanism of HGS-CVRP, and extends that framework with the following SDVRPTW-IC components.

1. **Split-aware representation.** Original-customer demand is mapped to quantity-carrying pseudo-customers while retaining the correspondence between every piece and its original customer. The formal configuration uses `CURRENT_RLHGS` splitting.
2. **No-split warm start.** When every original demand fits within vehicle capacity, an initial no-split HGS phase supplies a seed to the split-enabled main search. If `maxOriginalDemand > capacity`, this phase is structurally impossible and is skipped automatically; the split-enabled search then starts at time zero.
3. **Adaptive large-neighborhood search.** The A6 configuration activates the implemented ALNS intensification/diversification path within the HGS search.
4. **Successor-learning guidance.** Search information on promising customer-successor relations is retained and used as guidance without replacing the standard HGS population framework.
5. **Stagnation-triggered destroy--repair.** During the main HGS stage, related original-customer service blocks are removed across routes and reinserted by regret-2 repair, followed by the existing HGS local search. A reconstructed candidate enters the population only after validation. If no feasible parent is available, the trigger is skipped and HGS continues.
6. **Independent validation.** Final candidates are recomputed from scratch and checked for exact demand, positive delivered quantities, capacity, hard time windows, service propagation, commodity conflicts, duplicate original-customer visits within a route, depot feasibility, and objective consistency.

The repository also contains isolated diagnostic implementations for adaptive resplitting, joint resplitting, bounded-uphill dynamic local search, dynamic elite, historical-solution reevaluation, and trajectory auditing. These paths are disabled unless their corresponding command-line switches are explicitly supplied; they are not part of the formal command shown below.

## 2. Repository structure

```text
SDVRPTW-IC/
├── Program/
│   ├── main.cpp                         Program entry point and experiment driver
│   ├── Genetic.{h,cpp}                  HGS genetic-search loop and OX crossover
│   ├── Population.{h,cpp}               Population, diversity, survivor selection, output
│   ├── Individual.{h,cpp}               Fixed and dynamic solution representations
│   ├── LocalSearch.{h,cpp}              HGS local-search neighborhoods and SWAP*
│   ├── Split.{h,cpp}                    Giant-tour route partitioning
│   ├── DemandSplit.{h,cpp}              Demand-splitting strategies and diagnostics
│   ├── AdaptiveLNS.{h,cpp}              Adaptive large-neighborhood search
│   ├── MinimalDestroyRepair.{h,cpp}      Related destroy and regret-2 repair
│   ├── SolutionValidator.{h,cpp}         Independent from-scratch validator
│   ├── Params.{h,cpp}                   Problem data and search state
│   ├── InstanceCVRPLIB.{h,cpp}          Extended instance reader
│   ├── AlgorithmParameters.{h,cpp}       Algorithm parameter definitions
│   ├── AdaptiveResplitting.{h,cpp}       Dynamic resplitting diagnostics
│   ├── JointResplitting.{h,cpp}          Two-customer joint-resplitting diagnostics
│   ├── DynamicLocalImprove.{h,cpp}       Dynamic-quantity local improvement
│   ├── BoundedUphillDynamicLS.{h,cpp}    Bounded bridge-search diagnostics
│   ├── DynamicElite.{h,cpp}              Isolated dynamic side-elite path
│   ├── OldSolutionReevaluation.{h,cpp}   Read-only historical-solution reevaluation
│   ├── DistanceUtils.h                   Two-decimal Euclidean distance utilities
│   ├── NumericFormat.h                   Stable numerical output formatting
│   ├── commandline.h                     Command-line parsing
│   └── C_Interface.{h,cpp}               C-compatible interface
├── Instances/
├── LICENSE
└── README.md
```

The two local development backups `main(生成exe专用).cpp` and `main原始.cpp` are intentionally excluded from this repository. `Program/main.cpp` is the authoritative entry point.

## 3. Instance data

The supplied instances are organized as follows:

```text
Instances/
├── SDVRPTW/
├── SDVRPTW(M)/
└── SDVRPTW(M-5-100)/
    └── 100_customer/
        ├── 100_customer_30/
        ├── 100_customer_50/
        └── 100_customer_100/
```

Instances are derived from Solomon VRPTW classes and augmented with delivery demand, commodity types, and commodity-incompatibility pairs.

### Extended instance format

```text
NAME : C101
TYPE : SDVRPTW
DIMENSION : 101
EDGE_WEIGHT_TYPE : EUC_2D
CAPACITY : 100
NODE_COORD_SECTION
1 40 50
...
DEMAND_SECTION
1 0 0
2 10 3
...
TYPE_CONFLICT
2 5
4 6
TIME_WINDOW_SECTION
1 0 1236
...
SERVICE_TIME_SECTION
1 0
...
DEPOT_SECTION
1
-1
EOF
```

Each demand record contains the node ID, demand, and commodity type. Commodity type `0` is reserved for the depot. Each line in `TYPE_CONFLICT` declares a pair of commodity types that cannot share a vehicle.

## 4. Building on Windows

The source requires a C++17 compiler. The tested build environment is Windows x64 with MSVC.

Open **x64 Native Tools Command Prompt for Visual Studio**, then run:

```powershell
cd Program
cl /std:c++17 /O2 /EHsc /MT /DNOMINMAX *.cpp /Fe:rlhgs.exe /link Psapi.lib
```

This produces:

```text
Program\rlhgs.exe
```

The `/MT` option links the release MSVC runtime statically. Debug builds and Debug runtime libraries should not be used for computational experiments.

## 5. Running the solver

General syntax:

```text
rlhgs.exe <instance.vrp> <solution.sol> [option value] ...
```

### Formal configuration

The formal configuration used for the reported experiments is:

```powershell
.\rlhgs.exe `
  ".\Instances\SDVRPTW(M-5-100)\100_customer\100_customer_100\c101.vrp" `
  ".\output\c101_seed2026.sol" `
  -variant A6 `
  -splitStrategy CURRENT_RLHGS `
  -seed 2026 `
  -t 600 `
  -warmFraction 0.5 `
  -warmLegacyFraction 0 `
  -warmInitialInfeasibleRepair 0 `
  -drTrigger 500 `
  -drRemovalRatio 0.15 `
  -searchGuide OFFICIAL `
  -log 1
```

Options are parsed from left to right. `-variant A6` configures an ablation preset and may change the split preset, so `-splitStrategy CURRENT_RLHGS` must appear **after** `-variant A6` in the formal command.

For the capacity-30 instances, the executable checks the original demands before starting the no-split warm phase. When an original demand exceeds capacity, the expected log entry is:

```text
[WARM-START] enabled=0 reason=ORIGINAL_DEMAND_EXCEEDS_CAPACITY ...
```

The complete 600-second budget is then used by the split-enabled search.

### Main command-line options

| Option | Description | Code default | Formal value |
|---|---|---:|---:|
| `-t <seconds>` | Global search time limit | `0` | `600` |
| `-seed <integer>` | Reproducible random seed | `0` | experiment seed |
| `-variant <A0..A6>` | Reproducible algorithm/ablation preset | `CUSTOM` | `A6` |
| `-splitStrategy <name>` | Demand-splitting representation | `CURRENT_RLHGS` | `CURRENT_RLHGS` |
| `-warmFraction <ratio>` | Fraction of the global budget assigned to the no-split warm phase when feasible | `0.10` | `0.50` |
| `-warmLegacyFraction <ratio>` | Fraction of warm time using legacy integer decision guidance | unspecified | `0` |
| `-warmInitialInfeasibleRepair <0/1>` | Enable the legacy initial infeasible-repair path | `0` | `0` |
| `-alns <0/1>` | Enable adaptive large-neighborhood search | `0` | enabled by A6 |
| `-rl <0/1>` | Enable successor-learning guidance | `1` | `1` |
| `-drTrigger <iterations>` | Stagnation threshold for destroy--repair; `0` disables it | `0` | `500` |
| `-drRemovalRatio <ratio>` | Fraction of original-customer blocks removed by destroy--repair | `0.15` | `0.15` |
| `-searchGuide <mode>` | Search guidance: `OFFICIAL` or `INTEGER_SURROGATE` | `OFFICIAL` | `OFFICIAL` |
| `-it <iterations>` | Iterations without improvement before restart/termination logic | `20000` | `20000` |
| `-log <0/1>` | Print search traces and diagnostics | `1` | `1` |

Other HGS population, penalty, split-diagnostic, dynamic-diagnostic, and audit options are listed by the help output in `Program/commandline.h`.

## 6. Distance, feasibility, and validation

The official edge distance and travel time are computed as:

```text
round(sqrt((x_i-x_j)^2 + (y_i-y_j)^2) * 100) / 100
```

The official objective is always the sum of these two-decimal Euclidean route distances. The final independent validator checks:

- exact total delivery to every original customer;
- no overdelivery or underdelivery;
- strictly positive delivered quantities;
- vehicle capacity;
- hard customer and depot time windows;
- service-time propagation;
- commodity incompatibility;
- no duplicate visit to the same original customer within one route;
- route structure and depot feasibility; and
- agreement between stored and recomputed objective values.

A run should be treated as valid only when the final log reports `Validator=PASS`.

## 7. Output

The `.sol` file records original-customer IDs and delivered quantities:

```text
Route #1: 32(30) 33(40) 30(10)
Route #2: 18(20) 41(30)
Cost 1494.49
```

The console output includes live HGS traces and machine-readable summaries, including:

```text
[VALIDATOR-COST] ... validator=PASS
[RUN-SUMMARY] ... Validator=PASS SearchRuntime=...
[DR-SUMMARY] DRTriggers=... DRSuccessfulRuns=... DRSkippedNoFeasibleParent=...
[CURRENT-DR-RESULT] BestCost=... Routes=... SplitCustomers=... Validator=PASS ...
RESULT,<instance>,<variant>,<seed>,<feasible>,<cost>,...
```

All search work, including warm start, HGS, ALNS, destroy--repair, local search, and validation associated with solution production, is accounted for within the same global time budget.

## 8. Read-only reevaluation of historical solutions

An existing `.sol` file can be reevaluated without running any optimization:

```powershell
.\rlhgs.exe ".\Instances\path\c101.vrp" ".\output\unused.sol" `
  -reevaluateSolution ".\old_solution.sol"
```

This mode reconstructs the routes, recomputes the official two-decimal objective, runs the current validator, prints the diagnostic result, and exits without modifying or improving the supplied solution.

## 9. Reproducibility notes

- Use a Release x64 executable.
- Keep the complete command line, executable hash, instance files, seed, and log for every run.
- The formal repeated experiments use deterministic seeds; the published experimental package uses seeds `2026` through `2035` where ten repetitions are required.
- Do not replace the official two-decimal objective or official double-precision time-window propagation with an integer surrogate when reporting final results.
- Do not count a run as complete unless its independent validator reports `PASS`.

## 10. Citation

If you use this implementation, please cite the accompanying paper. Full bibliographic information will be added after publication.

## 11. Acknowledgments and license

This implementation builds on the HGS-CVRP framework by Thibaut Vidal:

- [vidalt/HGS-CVRP](https://github.com/vidalt/HGS-CVRP)

The inherited source files retain their original MIT license notices. This repository is distributed under the terms of the [MIT License](LICENSE).
