# Agent-in-the-Loop_System
## Description
This repository is based on the paper, titled "On Feedback Design for Systems With an Agent-in-the-Loop". Starting from recreating the same simulation result featured in the paper to extending the framework. For convenience, we would like to refer to this framework by its abbreviated term, AITL. Please refer to the original paper via the DOI below. 

DOI: [10.1109/LCSYS.2025.3586635](https://doi.org/10.1109/LCSYS.2025.3586635)
## Content
### AITL
- The mathematical description is identical to that in the paper above.
- For the older version of `design_lmi_controller.m` utilizing MATLAB's Robust Control Toolbox solvers (e.g., `mincx`, `feasp`, `setlmi` etc...), please refer to the `deprecated` folder.
- TODO: insert gif here of aitl animation.

## Requirements

### MOSEK Solver
MOSEK is a commercial convex optimization solver. It is highly recommended for semidefinite programming (SDP) and LMI problems due to its speed and robustness. 

**License:** MOSEK requires a license to run. It is available for **free** for academic use.

* **Download Installer:** [https://www.mosek.com/downloads/](https://www.mosek.com/downloads/)
* **Request Academic License:** [https://www.mosek.com/license/request/](https://www.mosek.com/license/request/)

#### Installation & MATLAB Setup
1.  **Install:** Download and extract the MOSEK installer to your home directory (e.g., `/home/username/mosek/`).
2.  **License:** Place the `mosek.lic` file in a folder named `mosek` inside your user home directory:
    * Linux: `/home/username/mosek/mosek.lic`
    * Windows: `C:\Users\Username\mosek\mosek.lic`
3.  **MATLAB Path:** Add the toolbox path in MATLAB so YALMIP can find the solver. Run the following in the MATLAB Command Window (Ubuntu):

    ```matlab
    % Adjust path version (e.g., 11.1) and internal folder (e.g., r2019b) as needed
    addpath(genpath('/home/username/mosek/11.1/toolbox/r2019b'));
    savepath;
    ```
4.  **Verify:** Run `mosekdiag` and `yalmiptest` in MATLAB to confirm the solver is detected.

