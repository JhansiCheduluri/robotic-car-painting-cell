# Robotic Car Painting Cell Simulation

An industrial robotic automation simulation of a car painting workcell utilizing dual **FANUC P-250iB RH** robotic manipulators integrated with **RoboDK** and automated Python trajectory generation.

📄 **[Download / View Full Technical Assessment Report (PDF)](Robotic_Car_Painting_Cell_Report.pdf)**

![Robotic Car Painting Cell](screenshots/Screenshot%202026-10-17%20194455.png)

---

## 📌 Project Overview

This project simulates a complete automotive spray-painting booth featuring:
- **Dual FANUC P-250iB RH Robots**: Specialized 6-axis painting robots with hollow-wrist designs.
- **Linear Transfer Rail**: Auxiliary track axis (**Brooks PreciseFlex Linear Rail 2000**) extending reach across the vehicle envelope.
- **Automated Trajectory & Path Generation**: Custom Python algorithms calculating raster spray paths, standoff orientation, approach/retract moves, and gun trigger signals.
- **Real-Time Visual Paint Ribbons**: Progressive 3D paint ribbon generation (`.stl`) deposited dynamically onto the vehicle surface during simulation.
- **Process Analytics**: Reachability studies, cycle time calculations, and paint coverage metrics.

---

## 🚀 Key Features

- **Geometric Raster Toolpaths**:
  - Covers all primary exterior surfaces: **Hood, Roof, Trunk, Front Bumper, Rear Bumper, Left Side, and Right Side**.
  - Strict tool Z-axis alignment normal to each surface patch.
  - Safe 60 mm standoff clearances around vehicle window boundaries.
- **Paint Process Specifications**:
  - **Nozzle Standoff Distance**: 250 mm
  - **Spray Fan Pattern Width**: 300 mm
  - **Pattern Overlap**: 40% (180 mm step between adjacent passes)
  - **Process Painting Speed**: 400 mm/s
  - **Approach / Retract Speed**: 800 mm/s
- **Reachability & Coverage Validation**:
  - Inverse kinematics verification for all targets.
  - Output summary reporting painted millimeter metrics vs. nominal design.
  - Over 100% effective coverage accounting for nominal 40% spray overlap.

---

## 📊 Coverage Summary

Results from automated trajectory generation (`results_summary.csv`):

| Surface Section | Designed (mm) | Painted (mm) | Coverage (%) | Pieces Done / Total |
| :--- | :---: | :---: | :---: | :---: |
| **Hood** | 14,520.0 | 19,653.3 | 135.4% | 33 / 33 |
| **Roof** | 12,460.0 | 10,680.0 | 85.7% | 18 / 21 |
| **Trunk** | 12,200.0 | 13,962.2 | 114.4% | 29 / 30 |
| **Front** | 8,900.0 | 9,196.7 | 103.3% | 20 / 20 |
| **Rear** | 8,900.0 | 12,756.7 | 143.3% | 19 / 20 |
| **Side Right** | 23,400.0 | 23,400.0 | 100.0% | 40 / 40 |
| **Side Left** | 23,400.0 | 23,400.0 | 100.0% | 40 / 40 |
| **TOTAL** | **103,780.0** | **113,048.9** | **108.9%** | **199 / 204** |

---

## 📁 Repository Structure

```text
robotic-car-painting-cell/
├── Simulation_Files/
│   ├── car_painting_cell_v1.rdk          # RoboDK simulation station
│   ├── Fanuc-P-250iB-RH.robot            # FANUC robot kinematics model
│   ├── Brooks-PreciseFlex-Linear-Rail-2000.robot # Linear auxiliary axis
│   ├── simple_car.stl                    # Car body CAD model
│   ├── spray_gun.stl                     # Paint applicator tool model
│   └── stripes/                          # Dynamic visual paint ribbon STLs
├── scripts/
│   └── generate_paint_paths.py           # Core trajectory and simulation script
├── screenshots/
│   └── Screenshot 2026-10-17 194455.png  # Workcell visualization
├── reach_study.csv                       # Kinematic reach verification log
├── reachability_log.csv                  # Target-by-target IK solve log
├── results_summary.csv                   # Surface coverage statistics
├── run_log.csv                           # Execution timeline
└── README.md
```

---

## 🛠️ How to Run

### Prerequisites
- [RoboDK](https://robodk.com/) simulation software
- Python 3.8+ with `robodk` Python API installed:
  ```bash
  pip install robodk
  ```

### Running the Simulation
1. Launch **RoboDK** and open the station file:
   ```text
   Simulation_Files/car_painting_cell_v1.rdk
   ```
2. Run the trajectory generator script:
   ```bash
   cd scripts
   python generate_paint_paths.py
   ```
3. Watch the dual robots execute synchronized paint strokes with real-time visual paint deposition.

---

## 👤 Author
- **Jhansi Cheduluri** ([@JhansiCheduluri](https://github.com/JhansiCheduluri))
