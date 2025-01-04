# Multi-Robot SLAM: Pose-Graph vs Surfel-Based Approaches

## 1. Project Overview
This project aims to **develop and evaluate** two distinct SLAM approaches—**Pose-Graph SLAM** and **Surfel-Based SLAM**—for multi-robot systems using LiDAR, IMU, and odometry data. The goal is to assess their **accuracy, scalability, computational efficiency, and robustness** in dynamic and large-scale environments.

---

## 2. Objectives
- **Implementation**: Build both Pose-Graph SLAM and Surfel-Based SLAM frameworks.
- **Evaluation**: Test performance in **simulation** and compare results based on key metrics.
- **Multi-Robot Integration**: Support collaborative mapping and data merging between robots.
- **Scalability Testing**: Analyze computational load and accuracy for large-scale maps.

---

## 3. SLAM Approaches
### **A. Pose-Graph SLAM**
1. **Key Features**:
   - Keyframe-based optimization for scalability.
   - Graph representation with nodes (poses) and edges (constraints).
   - Efficient global consistency through **loop closure detection**.
2. **Libraries/Tools**:
   - Optimization: **GTSAM**, **g2o**, **Ceres Solver**.
   - Frameworks: **Cartographer**, **RTAB-Map**.

### **B. Surfel-Based SLAM**
1. **Key Features**:
   - Dense 3D surface representation using **surfels (surface elements)**.
   - Compact and memory-efficient map storage.
   - Handles dynamic environments through surfel updates.
2. **Libraries/Tools**:
   - Libraries: **ElasticFusion**, **InfiniTAM**.
   - Adaptations for **LiDAR** based on research papers.

---

## 4. Workflow

### **Phase 1: Development**
1. Implement both SLAM methods:
   - Pose-Graph SLAM with keyframes, loop closures, and graph optimization.
   - Surfel SLAM with surfel maps and projective data association.
2. Integrate sensors:
   - **LiDAR** for spatial measurements.
   - **IMU** for orientation tracking.
   - **Odometry** for motion estimates.

### **Phase 2: Simulation and Testing**
1. Setup simulation in **Gazebo** or a similar platform.
2. Generate test environments with static and dynamic obstacles.
3. Test multi-robot collaboration scenarios, including map merging.

### **Phase 3: Evaluation**
1. **Metrics**:
   - **Map Accuracy**: Compare generated maps with ground truth.
   - **Pose Error**: Use **Absolute Trajectory Error (ATE)** and **Relative Pose Error (RPE)**.
   - **Computational Performance**: Measure CPU/GPU utilization and runtime.
   - **Scalability**: Analyze performance for large maps and multiple robots.
   - **Loop Closure Robustness**: Evaluate drift correction and loop closure rates.
2. **Visualization**:
   - Plot trajectories and maps in **RViz** or **MATLAB**.
   - Graph performance metrics over time.

---

## 5. Tools and Frameworks
- **SLAM Libraries**:
  - **GTSAM**, **g2o**, **Ceres Solver** for Pose-Graph optimization.
  - **ElasticFusion**, **InfiniTAM** for Surfel SLAM.
- **Simulation Tools**:
  - **Gazebo**, **ROS/ROS2**, **RViz** for multi-robot simulation and visualization.
- **Analysis Tools**:
  - **MATLAB**, **Python (matplotlib, pandas)** for evaluating metrics.
- **Sensor Integration**:
  - Support for **LiDAR**, **IMU**, and **Odometry** through ROS drivers.

---

## 6. Challenges and Solutions
| **Challenge**                         | **Mitigation**                                                              |
|---------------------------------------|----------------------------------------------------------------------------|
| Real-Time Performance                 | Perform incremental graph updates and sparse optimization.                 |
| False Loop Closures                    | Use **RANSAC filtering** and **geometric consistency checks**.             |
| Map Drift During Large-Scale Motion    | Increase loop closure frequency and add **GPS/Beacon Constraints** if needed.|
| Multi-Robot Synchronization            | Synchronize timestamps using **ROS time sync protocols**.                   |
| High Memory Usage for Graphs           | Prune old keyframes and use sparse optimization techniques.                |

---

## 7. Comparison Metrics
| **Metric**                | **Pose-Graph SLAM**                       | **Surfel-Based SLAM**                     |
|---------------------------|-------------------------------------------|-------------------------------------------|
| **Mapping Accuracy**      | Sparse but accurate global consistency    | Dense and detailed 3D reconstruction     |
| **Loop Closure Handling** | Robust with graph optimization            | Handles loop closure via surfel updates   |
| **Memory Usage**          | Lower due to keyframe-based optimization  | Higher due to dense surfel storage        |
| **Scalability**           | Suitable for large-scale maps             | Better for medium-scale maps              |
| **Dynamic Environments**  | Moderate handling via keyframe pruning    | Strong handling through surfel updates    |
| **Computation Speed**     | Faster due to sparse processing           | Higher demands for dense processing       |

---

## 8. Recommendations
- **Pose-Graph SLAM**:
  - Preferred for **large-scale environments** requiring scalability.
  - Ideal for **multi-robot systems** with **map merging** and **loop closure detection**.
- **Surfel-Based SLAM**:
  - Best for **dense 3D reconstruction** in **small to medium spaces**.
  - Suitable for scenarios needing **detailed surface modeling**.

---

## 9. Testing and Results
- Test both SLAM methods under **identical conditions**.
- Collect and visualize metrics to compare **accuracy, runtime, memory usage, and scalability**.
- Analyze trade-offs to decide suitability based on the application.

---

## 10. Conclusion
This project provides insights into the performance of **Pose-Graph SLAM** and **Surfel-Based SLAM** for multi-robot systems. The results will guide future implementations, enabling the selection of the best-suited SLAM framework based on task requirements.

For additional details, see individual implementation files and experiment logs.

