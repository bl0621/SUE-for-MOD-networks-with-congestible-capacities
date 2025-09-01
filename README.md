# An operation-agnostic stochastic user equilibrium model for mobility-on-demand networks with congestible capacities

This repository contains code accompanying the paper:

> **An operation-agnostic stochastic user equilibrium model for mobility-on-demand networks with congestible capacities**  
> Bingqing Liu, David Watling, Joseph Y. J. Chow  
> *European Journal of Operational Research (2025), 323(2), 504–524*  
> [DOI: 10.1016/j.ejor.2024.12.038](https://doi.org/10.1016/j.ejor.2024.12.038)

## 📖 Overview

Mobility-on-Demand (MoD) services (e.g., ride-hailing, bikeshare, carshare) present challenges for equilibrium modeling because operators’ internal policies are often proprietary.  
This project introduces an **operation-agnostic stochastic user equilibrium (SUE) model** that:

- Models **congestible capacities** via a **Flow-Capacity Interaction (FC) matrix**, capturing how flows influence available capacities without explicit operator policy assumptions.  
- Provides both a **forward model** (SUE with congestible capacities) and an **inverse model** (calibrating the FC matrix from observed data).  
- Proposes a **solution algorithm** based on a Frank–Wolfe method with a ρ-bounded k-shortest-path generation.  
- Demonstrates applications with numerical examples and a real-world case study using NYC yellow taxi data.

## 🧪 N-D Network Example (Notebook)

The `N-D_network_example.ipynb` notebook demonstrates the **toy numerical experiment** from Section 3 of the paper.  
It walks through the following steps:

1. **Imports and setup**  
   - Load Python packages for optimization, plotting, and data handling.  

2. **Read in the network**  
   - Load `Link_list.csv` containing link IDs, start/end nodes, capacities, and costs.  

3. **Build link–node matrix**  
   - Construct `[Link, From, To, Capacity]` representation.  
   - Replace `999999` capacity entries with ∞ to indicate uncapacitated links.  

4. **Build adjacency dictionary**  
   - Create mapping `N[i]` → successor nodes, used later for path generation.  

5. **Define OD pairs and travel demand**  
   - Four OD pairs: (1,2), (1,3), (4,2), (4,3).  
   - Demand vector `r = [400, 800, 600, 200]`.  

6. **Generate cost adjacency matrix**  
   - Initialize `c_int` with uncongested link costs.  
   - Used for initial shortest path calculations.  

7. **Define Flow–Capacity Interaction (FC) matrix**  
   - Construct `F` to capture how flows on one link affect capacities of others (Eq. (1)–(3)).  
   - Includes both negative (capacity consumption) and positive (capacity contribution) effects.  

8. **Run algorithms**  
   - Import `k_path_finding` (Algorithm 1) to generate candidate paths.  
   - Import `FW_SUECC` (Algorithm 2) to solve the forward SUE model via Frank–Wolfe.  


## 📜 Citation

```bibtex
@article{liu2025operation,
  title={An operation-agnostic stochastic user equilibrium model for mobility-on-demand networks with congestible capacities},
  author={Liu, Bingqing and Watling, David and Chow, Joseph YJ},
  journal={European Journal of Operational Research},
  volume={323},
  number={2},
  pages={504--524},
  year={2025},
  publisher={Elsevier}
}

## 📄 License

This software is released under a **Research-Only Non-Commercial License**:

- Free to use, copy, and modify **solely for internal, non-commercial research and evaluation purposes**.  
- **Redistribution, sublicensing, and commercial use are not permitted.**  
- For commercial licensing opportunities, please contact the authors.  
- If you use this code in academic work, please cite the paper above.  

See the [LICENSE](LICENSE) file for full terms.
