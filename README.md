# Hybrid Spacecraft Attitude Control System (v7.2)

A robust, type-safe spacecraft attitude control simulation written in Haskell, synthesizing multiple generations of flight software architecture (v2.0 through v6.0).

## Key Architectural Features

* **Quaternion Double Cover & Geodesic Shortest Path:** Automatically checks the inner product between current and target orientations, negating the target quaternion when necessary to avoid 360-degree unwinding[cite: 7, 8].
* **Lie Algebra & Tangent Space Mapping:** Utilizes exponential and logarithmic maps for seamless conversions between rotation manifolds and tangent space vectors ($\mathfrak{so}(3)$)[cite: 8].
* **Four-Regime Gain Scheduling:** Smoothly transitions proportional and derivative gains across distinct flight phases—Acquisition ($>1.5\text{ rad}$), Tracking ($0.5\text{--}1.5\text{ rad}$), Settling ($0.1\text{--}0.5\text{ rad}$), and Fine-Pointing ($<0.1\text{ rad}$)[cite: 12].
* **Kinetic-to-Potential Energy Predictive Braking:** Continuously monitors the ratio of kinetic energy ($\omega^2$) to potential error magnitude, dynamically scaling damping gains to prevent overshoot and enforce monotonic convergence[cite: 7].
* **Feed-Forward Nutation Compensation:** Incorporates gyroscopic coupling terms derived from Euler's rotation equations to stabilize asymmetrical inertia tensors (`Ixx=100`, `Iyy=120`, `Izz=80`)[cite: 9, 11].
* **Noisy Sensor Simulation:** Simulates realistic telemetry degradation including star tracker jitter, IMU drift, and sun vector noise[cite: 8].

## Running the Simulation

Ensure you have GHC installed, then compile and run the core executable:

```bash
ghc -Wall Main.hs
./Main
