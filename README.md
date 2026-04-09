# Procedural Generation of Multi-Level Road Networks in 3D Space 

## 📖 About the Project
This project is a system for the procedural generation of multi-level road networks in three-dimensional space, designed for futuristic urban structures

## ✨ Key Features
* **Terrain Generation:** Creates a deterministic and natural-looking landscape using Perlin Noise.
* **Evolutionary Pathfinding:** Utilizes an evolutionary (genetic) algorithm to optimally generate main ground roads and elevated highways.
* **Smart Connecting Ramps:** Inter-layer connections are generated using the A* algorithm with a dedicated heuristic that optimizes paths based on maximum slope limits and obstacle avoidance
* **Advanced Post-processing:** The system smooths the generated paths using Chaikin's algorithm, reduces unnecessary points via String Pulling, and applies Catmull-Rom interpolation to ensure realistic and smooth curves.
* **Full Configurability:** Users can dynamically manipulate parameters within the editor, such as map size (`MapSize`), elevation scale (`HeightMult`), maximum road slope (`MaxSlope`), randomization seed (`Seed`), and the total number of layers (`NumberOfLayers`).

## 🛠️ Technologies
The project was developed using the following tools:
* **Unreal Engine 5** 
*  **C++** (implementation of core logic and pathfinding algorithms, including A* and the genetic algorithm)
* **Blueprint System**

## 📸 Showcase

**Video Walkthrough:**
A short video showing the traversal of the procedurally generated network.
YT: https://youtu.be/Voa3f1PfMAc

**Color-Coded Network Overview:**
A bird's-eye view showing the generated structure. For better visibility, Layer 0 is marked in red, Layer 1 in orange, Layer 2 in yellow, and connecting ramps are highlighted in blue.
<img width="1050" height="667" alt="image" src="https://github.com/user-attachments/assets/9958ecfd-a2f7-482a-9bb2-373aca439979" />

**Street-Level View:**
A perspective straight from the generated road.
<img width="1051" height="553" alt="image" src="https://github.com/user-attachments/assets/9c02c842-589d-4098-b397-2ecd1f821b6b" />


## ⚙️ How It Works
The system architecture is Event-Driven and is divided into two main stages synchronized by the main controller:
1. **Environment Generation:** Generates a vertex grid based on the Perlin Noise function and creates the physical terrain geometry.
2. **Road Network Generation:**
    * The algorithm first determines roads at the ground level (Layer 0), which follow the topography of the terrain and naturally avoid steep peaks.
    * Next, elevated roads (highways) are generated, maintaining a constant height while verifying a safe collision margin with the landscape below.
    * Finally, the system identifies optimal connection points and links them with ramps using the A* algorithm.
