# Flying Car Simulation (Riders) — HezarFen Team

> Autonomous flight controllers, reports, and documentation for a Flying Car Simulation Competition.  
> Built and tested on the **Riders** platform and simulation environment.

---

## Table of Contents

- [Overview](#overview)
- [Competition Context](#competition-context)
- [Repository Structure](#repository-structure)
- [Quick Start (Riders)](#quick-start-riders)
- [Tasks](#tasks)
  - [Task-1 — Baseline Controller](#task-1--baseline-controller)
  - [Task-2 — Advanced Scenarios](#task-2--advanced-scenarios)
- [Algorithms & Architecture](#algorithms--architecture)
  - [Common Control Loop](#common-control-loop)
  - [Core Algorithms](#core-algorithms)
- [Configuration](#configuration)
- [Reports](#reports)
- [Demo Video](#demo-video)
- [Disclosure](#disclosure)
- [Team & Roles](#team--roles)
- [Reproducibility & Simulation Notes](#reproducibility--simulation-notes)
- [FAQ](#faq)
- [License](#license)
- [Acknowledgements & Contact](#acknowledgements--contact)

---

## Overview

This repository collects our deliverables for a **Flying Car Simulation Competition**.  
It includes two tasks:

- **Task-1**: A baseline autonomous controller (`Cezeri`) with altitude gating, target pursuit, obstacle avoidance, battery-aware diversion, and emergency divert to hospitals.  
- **Task-2**: An advanced controller for multi-entity scenarios (e.g., highway “pists”, cargo, fire brigade), adding route staging, radar-guided descent, and automatic charging/mission-resumption logic.

The competition focuses on **autonomous flight algorithms**, strategic control, and responses to **environmental factors, malfunctions, and multi-vehicle traffic** within a shared city simulation.

---

## Competition Context

- Design and demonstrate **autonomous air traffic management**, **trajectory planning**, **inter-vehicle coordination**, **battery management**, and **robustness to non-ideal conditions** in realistic city scenarios.
- Multi-phase process including **Preliminary Design Report (PDR)** and **Final Design Report (FDR)** with required **proof videos**, culminating in a **final stage** presentation and jury evaluation.
- Development and testing are carried out on the **Riders** platform and official competition simulation environment.

---

## Repository Structure

```
.
├── task1/
│   ├── code/
│   │   └── task1.py
│   ├── reports/
│   │   ├── PDR/                  # if applicable
│   │   └── FDR/
│   └── info/
│       └── info.txt
├── task2/
│   ├── code/
│   │   └── task2.py
│   ├── reports/
│   │   ├── PDR/
│   │   └── FDR/
│   └── info/
│       └── info.txt
└── competition/
    └── flying_car_competition_specifications.pdf
```

> Place the **competition specifications** in `/competition` for easy reference.  
> Keep both **TR** and **EN (auto-translated)** reports under each task’s `/reports` folder for accessibility.

---

## Quick Start (Riders)

1. Open **Riders** and load the competition city scenario (provided by the committee).
2. Import the task code into your Riders project:  
   - Task-1: `task1/code/task1.py`  
   - Task-2: `task2/code/task2.py`
3. Ensure the sim exposes expected **sensors & map data** (GNSS, magnetometer, radar; hospitals, charging stations; local grid) and **motion commands**.
4. Run and record any **proof video** required for PDR/FDR.

> The simulation incorporates simplified dynamics and stochastic elements for realism; results may vary slightly run-to-run.

---

## Tasks

### Task-1 — Baseline Controller

Implements a single-agent controller with:
- **Altitude gate** then navigation toward the active target.  
- **Obstacle avoidance** using neighbor-cell checks.  
- **Battery-aware diversion** to nearest charging station for `id == 2`.  
- **Emergency divert** to nearest hospital.  
- **Arrival / landing** handling.

See `task1.py` for the complete logic. The file header includes our team and author list.

### Task-2 — Advanced Scenarios

Extends the framework to multi-entity scenarios and staged routes:
- **Pist / highway mapping** to index waypoints inside the sim.  
- **Charging detection on route** and automatic **charging mode** with **radar-guided descent**, then **resume** toward the next stage.  
- **Higher altitude gate** before switching to highway navigation.  
- **Arrival handling** at each pist, including terminal landing sequence.

See `task2.py` for the full implementation details.

---

## Algorithms & Architecture

### Common Control Loop

All controllers follow a loop structured around:
1. **Climb to safe altitude** (gate), otherwise keep ascending.  
2. If **landing mode** is active (target reached), **descend** and return.  
3. **Compute heading** to the current target and **turn via shortest angle**.  
4. **Obstacle / no-fly check** on the next neighbor; **pick a free direction** if blocked.  
5. **Move forward** (speed class by agent).  
6. **Arrival check** and **landing / charging handling**.

### Core Algorithms

- **Heading estimation (degrees)** via a fast **Taylor-series arctangent** approximation, normalized to [0, 360).  
- **Degree↔radian** helpers and a minimal **π** constant.  
- **Minimum battery estimate**: linear in Euclidean distance with a +20 buffer.  
- **Nearest charging station** selection by distance.  
- **Emergency hospital divert** picks the nearest hospital and retargets.  

**Task-2 additions:**
- **Pist mapping** from pist numbers to highway indices.  
- **Charging check** aligned with target pist coordinates.  
- **Altitude gate (e.g., 310)** before highway navigation.  
- **Radar-guided descent** and **resume** after full charge or terminal landing.

---

## Configuration

- `HEDEF_YAKINLIK` — distance threshold for “arrived” checks.  
  - Task-1 uses `0.5` (fine proximity), while Task-2 defines `4` for route-staging.  
- `PI` — minimal π constant used for degree→radian conversion.

> You can refactor these parameters into environment variables or a config file if your Riders template supports it.

---

## Reports

- **Team chart & task distribution** — roles aligned to Python, algorithms, and Riders expertise.  
- **Sandbox design & testing** — covers landing zones, charging points, hospital areas, etc., designed to demonstrate the most challenging task variations within time-limited proof videos.  
- **TR & EN (auto-translated)** — include both to maximize accessibility for reviewers.

Place reports under each task’s `/reports` directory.

---

## Demo Video

A demonstration is available on YouTube: **https://www.youtube.com/watch?v=cEvQ3RQkKEs**.

---

## Disclosure

To support future fairness, the **final (“master”) code** is approved for release in later years.  
This repository provides the **previous version** that successfully completes **Task-2**, optimized for **speed and reliability**.  
**Platform:** Riders · **Simulation:** Riders environment.

---

## Team & Roles

- **Team:** **HezarFen Team**  
- **Code Authors:** Adil Sevim, Utku Özcan, Melek Serkaya, Cansu Melek Karpuz

Our team comprises experienced members in **Python, algorithms, and Riders**, with tasks distributed to leverage individual strengths; coordination ensured consistent progress across reports and code.

---

## Reproducibility & Simulation Notes

- The simulation includes **randomness/stochastic disturbances** for realism and uses simplified vehicle dynamics integrated into the environment.  
- Due to stochastic effects, please expect minor run-to-run variations.  
- When recording proof videos, use consistent seeds/settings if available in your Riders workspace.

---

## FAQ

**Q: Which environment is this code intended for?**  
A: The **Riders** simulation environment provided for the competition, with required sensors/map data and motion APIs.

**Q: Do I need to submit proof videos?**  
A: Yes—both PDR and FDR phases require **proof videos** demonstrating scenario completion.

**Q: How are final scores determined?**  
A: Scoring spans PDR, FDR, online simulation tasks, and the final stage; see the official specifications for splits and award definitions.

**Q: What about licensing?**  
A: See the license section below. If you adopt code, please retain the author credits and disclose modifications.

---

## License

Add your preferred open-source license (e.g., **MIT**) here. If omitted, the repository defaults to “All rights reserved.”

---

## Acknowledgements & Contact

We thank the **Flying Car Simulation Competition** organizers and the **Riders** platform team for the environment and documentation.

**Contact:** For organizational queries, follow official announcements/channels from the committee. For technical questions about this repository, please open an issue or contact the authors.

---

### Turkish Summary (Kısa)

Bu depo, **Uçan Araba Simülasyon Yarışması** kapsamında hazırlanan Task-1 ve Task-2 kodları ile raporları içerir. Geliştirme ve testler **Riders** platformunda yapılmıştır. Nihai “master” sürüm gelecekte paylaşılacaktır; burada **Task-2’yi başarıyla tamamlayan** bir önceki sürüm paylaşılmaktadır.
