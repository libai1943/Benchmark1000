# Benchmark1000

Large-scale numerical benchmark accompanying the survey

## A Survey of Collision Avoidance Constraint Modeling Strategies in Optimization-based Trajectory Planning for Autonomous Driving

This repository provides the large-scale benchmark used for the unified numerical evaluation in our survey manuscript:

**"A Survey of Collision Avoidance Constraint Modeling Strategies in Optimization-based Trajectory Planning for Autonomous Driving," IEEE Intelligent Transportation Systems Magazine (ITSM), 2026.**

The survey focuses on **collision avoidance constraint modeling for optimization-based trajectory planning**. In addition to reviewing and classifying representative modeling strategies, the survey performs a unified numerical comparison to investigate how different collision avoidance formulations affect solution success, computational efficiency, convergence behavior, and solution quality.

This repository contains the batch-testing implementation of that numerical study.

A companion repository containing the individual implementations of the eight trajectory planners is available at:

https://github.com/libai1943/Eight_Trajectory_Planners

---

## Benchmark Overview

The benchmark evaluates eight representative collision avoidance formulations under a common trajectory-planning framework.

For each method, the same:

- vehicle model;
- trajectory-planning task;
- dynamic constraints;
- objective function;
- temporal discretization;
- IPOPT settings;
- set of 1,000 test cases;
- three initial-guess conditions; and
- solution-validity criterion

are used.

The principal difference among the eight planners is therefore the **collision avoidance constraint formulation**.

Each method is evaluated on:

**1,000 test cases × 3 initial-guess conditions**

and the complete benchmark contains:

**8 methods × 1,000 cases × 3 initializations = 24,000 method-case-initialization trials.**

This large-scale evaluation complements the representative examples and theoretical analysis presented in the survey.

---

## Evaluated Methods

The benchmark is organized around the same eight methods reproduced in our companion repository.

| Folder | Survey Ref. | Method |
|---|---:|---|
| [`OBCA14`](./OBCA14) | [14] | Basic optimization-based collision avoidance (OBCA) |
| [`sdOBCA14`](./sdOBCA14) | [14] | Signed-distance OBCA (sd-OBCA) |
| [`BOMP15`](./BOMP15) | [15] | Bilevel optimal motion planning (BOMP) |
| [`Lutz16`](./Lutz16) | [16] | Lutz-Meurer collision avoidance formulation |
| [`Fan18`](./Fan18) | [18] | Fan et al. separation-based formulation |
| [`Liu20`](./Liu20) | [20] | Liu et al. Farkas-lemma-based formulation |
| [`Triangle21`](./Triangle21) | [21] | Li-Shao triangle-area formulation |
| [`Fan22`](./Fan22) | [22] | Fan et al. polygonal collision avoidance formulation |

The reference numbers follow those used in the survey.

---

## Benchmark Cases

The 1,000 test cases are stored in:

```text
case_data.mat
```

Each method uses the same case index:

```matlab
LoadCaseBatch(case_id);
```

where

```matlab
case_id = 1, 2, ..., 1000
```

therefore `case_id = k` corresponds to the same geometric test case for every collision avoidance method.

The test-case loader constructs the convex polygonal obstacle associated with each case and uses the same prescribed initial and terminal vehicle configurations for all methods.

This common test set is essential for ensuring that differences in the numerical results arise from the collision avoidance formulations rather than from different test scenarios.

---

## Initial-Guess Conditions

Three substantially different initial-guess conditions are examined.

| Mode | Initial guess | Purpose |
|---:|---|---|
| `0` | Static initial guess | Tests the method from a highly uninformative stationary initialization |
| `1` | Conflicting straight-line initial guess | Tests convergence from a dynamically meaningful trajectory that conflicts with the obstacle |
| `2` | Completely feasible initial trajectory | Tests the method when initialized from a high-quality collision-free trajectory |

The three modes are generated through:

```matlab
GenerateInitialGuess(mode);
```

Using different initialization qualities allows the benchmark to reveal differences in numerical robustness that may not be visible when only a favorable initial guess is considered.

---

## Running the Benchmark

Unlike the companion repository, this benchmark repository does **not** use `RunMe.m` as its main entry point.

For each method, enter the corresponding folder in MATLAB and execute:

```matlab
RunBatch
```

For example:

```matlab
cd OBCA14
RunBatch
```

One execution of `RunBatch.m` performs the complete batch evaluation for that method:

```text
Mode 0: 1,000 cases
Mode 1: 1,000 cases
Mode 2: 1,000 cases
```

and generates three MATLAB result files.

For example, OBCA generates:

```text
result_obca_mode_0.mat
result_obca_mode_1.mat
result_obca_mode_2.mat
```

Other method folders follow the same naming principle.

Because the complete benchmark involves thousands of nonlinear programming problems, the execution time can be substantial.

---

## Recorded Data

For each initial-guess mode, `RunBatch.m` creates:

```matlab
data_collection = zeros(1000, 3);
```

Each row corresponds to one test case.

| Column | Quantity | Description |
|---:|---|---|
| 1 | Solution validity | `1` if a valid solution is obtained and `0` otherwise |
| 2 | Computation time | Total computation time for the method on that test case |
| 3 | Terminal time | Optimized terminal time for a valid solution; zero for an invalid solution |

Thus,

```matlab
data_collection(k, :)
```

contains the benchmark result for test case `k`.

The validity check is performed independently after optimization through:

```matlab
IsCurSolValid();
```

For methods involving an internal iterative procedure, the recorded computation time covers the **complete method-level solution process**, rather than only the final nonlinear programming call. For example, the BOMP timing includes its continuation iterations.

---

## Generated Result Files

After all eight methods have been executed, the benchmark produces three result files for each method:

```text
result_<method>_mode_0.mat
result_<method>_mode_1.mat
result_<method>_mode_2.mat
```

The complete experiment therefore contains:

```text
8 methods × 3 initial-guess modes = 24 result sets
```

with each result set containing the outcomes of 1,000 test cases.

These `.mat` files are generated locally and form the input to the final statistical analysis.

---

## One-Click Analysis

After the batch experiments for all eight methods have been completed, return to the root directory and execute:

```matlab
TestAndScore
```

`TestAndScore.m` automatically loads the generated result files from all method folders and performs the unified analysis used in the survey.

Its role is to aggregate the raw batch results and evaluate the methods in terms of quantities such as:

- solution success rate;
- computation time;
- convergence behavior across different initialization conditions; and
- terminal-time-based solution quality.

This separates the workflow into two stages:

```text
RunBatch.m
    |
    |--- solve 1,000 cases under three initial guesses
    |--- save raw .mat results
    |
    v
TestAndScore.m
    |
    |--- collect results from all eight methods
    |--- compute benchmark statistics
    |--- generate the final comparative results
```

The raw optimization experiments therefore only need to be performed once. Subsequent analysis can be reproduced directly from the saved `.mat` files.

---

## Repository Structure

The completed repository follows the structure:

```text
Benchmark1000/
|
|-- OBCA14/
|   |-- RunBatch.m
|   |-- case_data.mat
|   |-- perfect_ig.mat
|   |-- NLP.mod
|   |-- ...
|
|-- sdOBCA14/
|   |-- RunBatch.m
|   |-- case_data.mat
|   |-- perfect_ig.mat
|   |-- NLP.mod
|   |-- ...
|
|-- BOMP15/
|   |-- RunBatch.m
|   |-- case_data.mat
|   |-- perfect_ig.mat
|   |-- NLP.mod
|   |-- ...
|
|-- Lutz16/
|-- Fan18/
|-- Liu20/
|-- Triangle21/
|-- Fan22/
|
|-- TestAndScore.m
|
`-- README.md
```

Each method folder is self-contained and follows the same general benchmark interface.

---

## Relationship to the Companion Repository

Two repositories are provided for different purposes.

### Eight_Trajectory_Planners

https://github.com/libai1943/Eight_Trajectory_Planners

This repository contains the individual implementations of the eight collision avoidance formulations and is intended for:

- inspecting the mathematical implementation of each method;
- running representative individual examples;
- understanding the differences among the collision avoidance constraints; and
- reproducing individual trajectory-planning solutions.

### Benchmark1000

https://github.com/libai1943/Benchmark1000

This repository is intended for:

- large-scale quantitative evaluation;
- testing all methods on the same 1,000 cases;
- comparing three different initialization conditions;
- collecting computation-time and solution-quality data; and
- reproducing the statistical results reported in the survey.

Together, the two repositories provide both **method-level reproducibility** and **large-scale experimental reproducibility**.

---

## Original References

**[14]** X. Zhang, A. Liniger, and F. Borrelli, "Optimization-Based Collision Avoidance," *IEEE Transactions on Control Systems Technology*, vol. 29, no. 3, pp. 972-983, 2021.

**[15]** S. Shi, Y. Xiong, J. Chen, and C. Xiong, "A Bilevel Optimal Motion Planning (BOMP) Model With Application to Autonomous Parking," *International Journal of Intelligent Robotics and Applications*, vol. 3, pp. 370-382, 2019.

**[16]** M. Lutz and T. Meurer, "Efficient Formulation of Collision Avoidance Constraints in Optimization-Based Trajectory Planning and Control," in *Proc. 2021 IEEE Conference on Control Technology and Applications (CCTA)*, San Diego, CA, USA, pp. 228-233, 2021.

**[18]** J. Fan, N. Murgovski, and J. Liang, "Efficient Optimization-Based Trajectory Planning for Unmanned Systems in Confined Environments," *IEEE Transactions on Intelligent Transportation Systems*, vol. 25, no. 11, pp. 18547-18560, Nov. 2024.

**[20]** C. Liu, S. Lee, S. Varnhagen, and H. E. Tseng, "Path Planning for Autonomous Vehicles Using Model Predictive Control," in *Proc. 2017 IEEE Intelligent Vehicles Symposium (IV)*, Los Angeles, CA, USA, pp. 174-179, Jun. 2017.

**[21]** B. Li and Z. Shao, "A Unified Motion Planning Method for Parking an Autonomous Vehicle in the Presence of Irregularly Placed Obstacles," *Knowledge-Based Systems*, vol. 86, pp. 11-20, 2015.

**[22]** J. Fan, N. Murgovski, and J. Liang, "Efficient Collision Avoidance for Autonomous Vehicles in Polygonal Domains," *IEEE Transactions on Transportation Electrification*, vol. 11, no. 2, pp. 5396-5406, Jun. 2025.

---

## Software

The benchmark is implemented using:

- MATLAB;
- AMPL; and
- IPOPT.

The current scripts are prepared primarily for a Windows environment.

Because the benchmark repeatedly solves a large number of nonlinear programming problems, computational time depends substantially on the processor, solver configuration, and individual collision avoidance formulation. Meaningful runtime comparisons should therefore be performed on the same computer under the same software configuration.

---

## Purpose of This Repository

The purpose of this benchmark is to provide a reproducible quantitative basis for the experimental conclusions in our survey.

The benchmark does not aim to compare complete autonomous-driving planning systems assembled from different papers. Instead, representative collision avoidance formulations are embedded into a unified trajectory-planning framework so that their numerical behavior can be examined under controlled and consistent conditions.

For the mathematical taxonomy, detailed derivations, modeling properties, limitations, and interpretation of the numerical results, please refer to:

> **A Survey of Collision Avoidance Constraint Modeling Strategies in Optimization-based Trajectory Planning for Autonomous Driving**  
> *IEEE Intelligent Transportation Systems Magazine (ITSM), 2026.*

---

## Citation

If this benchmark or its source code is useful for your research, please cite the survey and the original publication corresponding to the collision avoidance formulation being used.

The complete bibliographic information of the survey will be updated after final publication.
