## AI Grand Prix (AI-GP) Development Kit
Conceived by Anduril founder Palmer Luckey and partnered with the Drone Champions League (DCL), Neros Technologies, and JobsOhio, AI-GP is a premier autonomous drone racing competition.
This global challenge invites elite engineers and teams of up to 8 people to design, build, and deploy autonomy software capable of piloting high-speed racing drones through professional-grade courses—with absolutely zero human intervention.
For complete competition details and updates, visit the official website at www.theaigrandprix.com.

## 🏆 Competition Highlights

* The Stakes: Compete for a share of a $500,000 prize pool and career opportunities at Anduril.
* The Hardware: Complete competitive parity. All teams utilize identical racing drones built by Neros Technologies incorporating DCL's AI vector module.
* The Mission: Program the ultimate AI pilot to conquer dynamic, real-world flight conditions using onboard vision sensing—no GPS or absolute coordinate data will be provided.

------------------------------
## 📁 Repository Contents
This package contains the foundational tools required to develop, test, and qualify your autonomous flight software.
## 1. AIGP_X.zip (The Simulator)
This archive contains the official AI-GP flight simulator environment for Windows.

* Setup: Extract the ZIP archive to your local directory.
* Execution: Launch the simulator by running FlightSim.exe from the unzipped root folder.
* Authentication: Access the virtual qualifier within the simulator by logging in with your official simulator account credentials.

## 2. PyAIPilotExample-v2.zip (The Code Template)
This archive provides a starter template to help you interface with the simulator and write your autonomous flight algorithms.

* Environment: Tested and verified on Python 3.14.2.
* Setup:
1. Unzip the archive.
   2. Install the required dependencies:
   
   pip install -r requirements.txt
   
   * Execution: Run the primary script to connect to the simulator:

python main.py


------------------------------
## 💻 System Requirements
The simulator environment has been successfully tested on Windows 11 with a GeForce RTX 3070. For stable performance, your system should meet or exceed the following hardware specifications:

| Requirement | Minimum Specification |
|---|---|
| OS | 64-bit Windows 10 / 11 |
| Processor | Intel Core i7 4770k (or AMD equivalent) |
| Memory | 8 GB RAM |
| Graphics | NVIDIA GeForce GTX 970 |
| Network | Broadband Internet connection |
| Storage | 12 GB available space |

------------------------------
## 📅 Timeline & Structure

* Virtual Qualifier Round 1: Simple, high-contrast, desaturated gate environment to test core flight logic.
* Virtual Qualifier Round 2: High-fidelity, visually complex 3D-scanned environments.
* Physical Qualifier (September 2026): Top teams advance to a live, indoor testing phase in Southern California.
* The Finals (November 2026): The premier AI Grand Prix live event in Ohio.

------------------------------
## ℹ️ Technical Specification & More Information
Can be found here:

https://www.theaigrandprix.com/previousupdates/