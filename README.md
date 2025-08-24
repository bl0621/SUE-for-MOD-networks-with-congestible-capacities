# An operation-agnostic stochastic user equilibrium model for mobility-on-demand networks with congestible capacities

This repository contains code accompanying the paper:

> **An operation-agnostic stochastic user equilibrium model for mobility-on-demand networks with congestible capacities**  
> Bingqing Liu, David Watling, Joseph Y. J. Chow  
> *European Journal of Operational Research (2025), 323(2), 504–524*  
> [DOI: 10.1016/j.ejor.2024.12.038](https://doi.org/10.1016/j.ejor.2024.12.038)

We model multimodal equilibrium with MoD systems in an operation-agnostic manner based on empirical observations of flow and capacity. This is done with a Flow-Capacity Interaction (FC) matrix that captures systematic effect of congestible capacities, a phenomenon in MoD systems where capacities are affected by flows. 

## 📖 Overview

Mobility-on-Demand (MoD) services (e.g., ride-hailing, bikeshare, carshare) present challenges for equilibrium modeling because operators’ internal policies are often proprietary.  
This project introduces an **operation-agnostic stochastic user equilibrium (SUE) model** that:

- Models **congestible capacities** via a **Flow-Capacity Interaction (FC) matrix**, capturing how flows influence available capacities without explicit operator policy assumptions.  
- Provides both a **forward model** (SUE with congestible capacities) and an **inverse model** (calibrating the FC matrix from observed data).  
- Proposes a **solution algorithm** based on a Frank–Wolfe method with a ρ-bounded k-shortest-path generation.  
- Demonstrates applications with numerical examples and a real-world case study using NYC yellow taxi data.
