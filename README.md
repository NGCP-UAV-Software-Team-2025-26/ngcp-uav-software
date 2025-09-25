# NGCP UAV Software Team – 2025–26

Welcome to the official software repository for the **Northrop Grumman Collaboration Project (NGCP)** at Cal Poly Pomona, 2025–2026 academic year.

This project will focus on developing and integrating software systems for an unmanned aerial vehicle (UAV) in collaboration with Northrop Grumman engineers and CPP faculty advisors.

---

## 👥 Team Members

| Name             | Role             |
|------------------|------------------|
| Areesha Imtiaz   | Software Lead    |
| Ivan Trinh       | Deputy           |
| Arun Nambiar     | Member           |
| Edwin Estrada    | Member           |
| Elijah Chan      | Member           |
| Len Sakimukai    | Member           |
| Medha            | Member           |  
| Wesley Dam       | Member           |

---

## 📅 Weekly Meetings

- **Day:** Every Friday  
- **Location:** In-person at the Cal Poly Pomona Library (Room 5439) & [Zoom](https://cpp.zoom.us/my/aimtiaz)  

Weekly meeting notes will be stored in the `/docs/meetings/` folder.

---

## 🚀 [Project Timeline (Software Subteam)](https://livecsupomona-my.sharepoint.com/:x:/r/personal/cjmaw_cpp_edu/Documents/NGCP%2025-26/LEADS%20-%20NGCP%20UAV%2025-26/Kansas%20City%20Chiefs%20Stuff/NGCP20252026_Timeline.xlsx?d=w27149f4d3b2941e9b8a0f7bb0f6a083d&csf=1&web=1&e=LyBxGs)

> This is the software-only timeline aligned to the overall NGCP master schedule (RFP ≈ Oct, PDR/CDR ≈ late Mar, Demo ≈ May). Rows are grouped with subheadings like the master Gantt.

### 🏗️ Setup & Onboarding *(Aug–Sept 2025)*
- **Aug 22:** Team intros, repo + Kanban setup, personality test, Discord naming
- **Aug 29:** Tech stack chosen (**Python**), Ubuntu 24.04 installs begin
- **Sept 5:** Install **PX4** + **QGroundControl**; run first **Gazebo** fixed-wing sim
- **Sept 12:** **PX4 SITL** demo; **MAVSDK** intro; liaisons confirmed
- **Sept 19:** Gazebo basics demo; confirm everyone’s toolchain
- **Sept 26:** Mission-planning demo prep; RFP readiness

### 📑 Requirements & Architecture *(Oct–Dec 2025)*
- **Oct 3:** RFP review + requirement extraction (software scope)
- **Oct 10:** Define **GCS** features; assign roles (mission, comms, autonomy, UI)
- **Oct 17:** Set up **CI/CD** (GitHub Actions); repo skeleton + coding standards
- **Oct 24:** **SRR prep:** requirement doc + traceability matrix
- **Oct 31:** **System Requirements Review (SRR)**; incorporate feedback
- **Nov 7:** Draft software architecture; interface map (PX4 ↔ MAVSDK ↔ GCS)
- **Nov 14:** Basic **MAVSDK waypoint mission** in sim
- **Nov 21:** Start test automation (telemetry/log parsing)
- **Nov 28:** Light week (Thanksgiving) — documentation + backlog grooming
- **Dec 5:** Finalize architecture; prototype mission planner (Python)
- **Dec 12–19:** Finals/Winter Break — keep sim runnable for all members

### ⚙️ Development & Integration *(Jan–Feb 2026)*
- **Jan 9:** Resume; kick off integration with **Electrical** (sensors/comms)
- **Jan 16:** Autonomous **takeoff/landing** in Gazebo
- **Jan 23:** **GCS UI** prototype (telemetry/status)
- **Jan 30:** Manufacturing starts → prepare **HIL** path
- **Feb 6:** **HIL**: Raspberry Pi 5 + PX4 + MAVSDK
- **Feb 13:** Multi-vehicle mission logic; inter-vehicle coordination
- **Feb 20:** Regression test suite for autonomy
- **Feb 27:** PDR/CDR document + demo prep

### 📝 Design Reviews *(Mar 2026)*
- **Mar 6:** Subsystem integration demo (with Electrical/Aero)
- **Mar 13:** Polish models/docs (reqs ↔ design ↔ tests)
- **Mar 20:** Final prep for reviews
- **Mar 27:** **Combined PDR/CDR (NGC site)**

### 🛫 Testing & Validation *(Apr 2026)*
- **Apr 3:** Spring Break — light progress
- **Apr 10:** Post-review fixes; flight-test prep
- **Apr 17:** Full mission simulation + partial hardware run
- **Apr 24:** Demo Day prep; imminent **code freeze**

### 🎉 Demonstration & Wrap-Up *(May 2026)*
- **Early May:** **Demo Day**
- **Post-Demo:** Teardown; final documentation/retrospective

---

## 🛠️ Tech Stack

- **Operating System:** Ubuntu 24.04 LTS
- **Programming Languages:** Python, C++ (for MAVSDK and PX4 integration)
- **Flight Control Software:** PX4 Autopilot (planes interface)
- **UAV SDK:** MAVSDK (Python/C++ bindings)
- **Simulation Environment:** Gazebo (plane simulation)
- **Onboard Computer:** Raspberry Pi 5 (ARM)
- **Development & Testing:** x86 computer recommended for simulation and builds
- **Hardware Interfaces:** Radio Frequency module (for patient search)
- **Collaboration & Storage:** OneDrive (shared folder for UAV team files)

[**UAV OneDrive Folder 2025-26**](https://livecsupomona-my.sharepoint.com/:f:/r/personal/cjmaw_cpp_edu/Documents/NGCP%2025-26/NGCP%20UAV%2025-26?csf=1&web=1&e=91sZNR)     

---


## 📁 Repository Structure  

```plaintext
ngcp-uav-software/
├── docs/               # Project documentation
│   ├── images/         # Events & Flyers information
│   └── meetings/       # Weekly meeting notes
├── scripts/            # Utility scripts and automation
├── src/                # Source code
│   ├── current/        # Active development code
│   └── past/           # Placeholder for older code
├── tests/              # Unit and integration tests
└── README.md           # You're here
