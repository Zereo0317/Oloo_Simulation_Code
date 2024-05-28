import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from Entity import OlooScooter, Rider
from Plotter import ScenarioPlotter
import os
import cv2

# Ensure the directory exists
os.makedirs('SimData/Videos', exist_ok=True)

# Simulation class that manages the setup and execution
class Simulation:
    def __init__(self, oloo, rider, surface_friction_static, surface_friction_dynamic):
        self.oloo = oloo
        self.rider = rider
        self.surface_friction_static = surface_friction_static
        self.surface_friction_dynamic = surface_friction_dynamic
        self.client = p.connect(p.GUI)  # Changed to GUI for visualization
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

    def setup_environment(self):
        if not p.isConnected():
            self.client = p.connect(p.DIRECT)
            if not p.isConnected():
                raise Exception("Failed to connect to physics server.")
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        plane_id = p.loadURDF("plane.urdf")
        oloo_id = p.loadURDF("Entity/E_Scooter.urdf")
        rider_id = p.loadURDF("Entity/Rider.urdf")
        return plane_id, oloo_id, rider_id

    def run_scenario(self, scenario_function, *args):
        print(f"Running scenario: {scenario_function.__name__}")
        plane_id, oloo_id, rider_id = self.setup_environment()
        results = scenario_function(plane_id, oloo_id, rider_id, *args)
        p.disconnect()
        return results

    def apply_friction(self, plane_id):
        p.changeDynamics(plane_id, -1, lateralFriction=self.surface_friction_static)

    def compute_displacement_and_check_stability(self, oloo_id, slope_angle):
        # Get the position and orientation of the oloo
        pos, ori = p.getBasePositionAndOrientation(oloo_id)
        displacement = np.linalg.norm(pos[:2])
        
        # Apply slope transformation to the displacement
        R_theta = np.array([
            [np.cos(np.radians(slope_angle)), -np.sin(np.radians(slope_angle)), 0],
            [np.sin(np.radians(slope_angle)), np.cos(np.radians(slope_angle)), 0],
            [0, 0, 1]
        ])
        transformed_pos = np.dot(R_theta, pos)
        
        # Check if COM exceeds the front or rear wheel positions
        if transformed_pos[0] < -self.oloo.wheelbase / 2 or transformed_pos[0] > self.oloo.wheelbase / 2:
            return displacement, False  # oloo will tilt over
        return displacement, True  # oloo is stable

    def simulate_pothole(self, plane_id, oloo_id, rider_id, depth):
        results = []
        for speed in np.linspace(0, 15, 45):
            p.resetBaseVelocity(oloo_id, [speed * 1000 / 3600, 0, 0])
            self.apply_friction(plane_id)

            # Apply downward force to simulate pothole encounter
            for i in range(1000):
                if i == 500:  # Simulate hitting the pothole at halfway
                    p.applyExternalForce(oloo_id, -1, [0, 0, -depth * self.oloo.mass * 9.8], [0, 0, 0], p.WORLD_FRAME)
                p.stepSimulation()

            displacement, is_stable = self.compute_displacement_and_check_stability(oloo_id, 0)  # Example with no slope
            results.append((speed, 0, "pothole", displacement, is_stable))
        return results

    def simulate_pinecone(self, plane_id, oloo_id, rider_id, height):
        results = []
        for speed in np.linspace(0, 15, 45):
            p.resetBaseVelocity(oloo_id, [speed * 1000 / 3600, 0, 0])
            self.apply_friction(plane_id)

            # Apply upward force to simulate rolling over pinecone
            for i in range(1000):
                if i == 500:  # Simulate hitting the pinecone at halfway
                    p.applyExternalForce(oloo_id, -1, [0, 0, height * self.oloo.mass * 9.8], [0, 0, 0], p.WORLD_FRAME)
                p.stepSimulation()

            displacement, is_stable = self.compute_displacement_and_check_stability(oloo_id, 0)  # Example with no slope
            results.append((speed, 0, "pinecone", displacement, is_stable))
        return results

    def simulate_wind(self, plane_id, oloo_id, rider_id, force):
        results = []
        for speed in np.linspace(0, 15, 45):
            p.resetBaseVelocity(oloo_id, [speed * 1000 / 3600, 0, 0])
            self.apply_friction(plane_id)

            # Apply constant wind force
            for i in range(1000):
                p.applyExternalForce(oloo_id, -1, [-force, 0, 0], [0, 0, 0], p.WORLD_FRAME)
                p.stepSimulation()

            displacement, is_stable = self.compute_displacement_and_check_stability(oloo_id, 0)  # Example with no slope
            results.append((speed, 0, "wind", displacement, is_stable))
        return results

    def simulate_acceleration(self, plane_id, oloo_id, rider_id, accel_range):
        results = []
        for speed in np.linspace(0, 15, 45):
            for accel in accel_range:
                p.resetBaseVelocity(oloo_id, [speed * 1000 / 3600, 0, 0])
                self.apply_friction(plane_id)

                # Apply sudden forward force for acceleration
                for i in range(1000):
                    if i == 500:  # Simulate sudden acceleration at halfway
                        p.applyExternalForce(oloo_id, -1, [accel * self.oloo.mass, 0, 0], [0, 0, 0], p.WORLD_FRAME)
                    p.stepSimulation()

                displacement, is_stable = self.compute_displacement_and_check_stability(oloo_id, 0)  # Example with no slope
                results.append((speed, accel, "acceleration", displacement, is_stable))
        return results

    def simulate_braking(self, plane_id, oloo_id, rider_id, decel_range):
        results = []
        for speed in np.linspace(0, 15, 45):
            for decel in decel_range:
                p.resetBaseVelocity(oloo_id, [speed * 1000 / 3600, 0, 0])
                self.apply_friction(plane_id)

                # Apply sudden backward force for braking
                for i in range(1000):
                    if i == 500:  # Simulate sudden braking at halfway
                        p.applyExternalForce(oloo_id, -1, [-decel * self.oloo.mass, 0, 0], [0, 0, 0], p.WORLD_FRAME)
                    p.stepSimulation()

                displacement, is_stable = self.compute_displacement_and_check_stability(oloo_id, 0)  # Example with no slope
                results.append((speed, decel, "braking", displacement, is_stable))
        return results

    # Function to visualize the simulation
    # Function to visualize the simulation and save frames
    def visualize_scenario(self, scenario_function, scenario_name, *args):
        plane_id, oloo_id, rider_id = self.setup_environment()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        
        # Set the camera parameters
        cam_target = [0, 0, 0]
        cam_distance = 3
        cam_yaw = 50
        cam_pitch = -35

        video_writer = cv2.VideoWriter(f'/SimData/Videos/{scenario_name}.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (640, 480))

        for i in range(1000):
            if i == 500 and scenario_function.__name__ == 'simulate_pothole':
                p.applyExternalForce(oloo_id, -1, [0, 0, -args[0] * self.oloo.mass * 9.8], [0, 0, 0], p.WORLD_FRAME)
            if i == 500 and scenario_function.__name__ == 'simulate_pinecone':
                p.applyExternalForce(oloo_id, -1, [0, 0, args[0] * self.oloo.mass * 9.8], [0, 0, 0], p.WORLD_FRAME)
            if i == 500 and scenario_function.__name__ == 'simulate_wind':
                p.applyExternalForce(oloo_id, -1, [-args[0], 0, 0], [0, 0, 0], p.WORLD_FRAME)
            if i == 500 and scenario_function.__name__ == 'simulate_acceleration':
                p.applyExternalForce(oloo_id, -1, [args[0] * self.oloo.mass, 0, 0], [0, 0, 0], p.WORLD_FRAME)
            if i == 500 and scenario_function.__name__ == 'simulate_braking':
                p.applyExternalForce(oloo_id, -1, [-args[0] * self.oloo.mass, 0, 0], [0, 0, 0], p.WORLD_FRAME)
            p.stepSimulation()
            
            # Capture and save the frame
            width, height, img_arr, depth_arr, seg_arr = p.getCameraImage(
                width=640, height=480, viewMatrix=p.computeViewMatrixFromYawPitchRoll(cam_target, cam_distance, cam_yaw, cam_pitch, 0, 2),
                projectionMatrix=p.computeProjectionMatrixFOV(60, 640/480, 0.1, 100)
            )
            img = img.astype(np.uint8)
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            video_writer.write(img)

        video_writer.release()
        p.disconnect()

# Parameters
oloo = OlooScooter(mass=15, friction=0.5, com=np.array([0, 0.5, 0]), wheelbase=0.5)
rider = Rider(mass=70, friction=0.5, com=np.array([0, 1, 0]), tilt_angle=10)
surface_friction_static = 0.8
surface_friction_dynamic = 0.6

# Simulation
sim = Simulation(oloo, rider, surface_friction_static, surface_friction_dynamic)
speed_variation = np.linspace(0, 15, 45)  # Speed variation from 0 to 15 km/hr
angle_variation = np.linspace(-20, 20, 40)  # Variation of human tilt angle relative to oloo
friction_variation = np.linspace(0.2, 1.0, 5)  # Variation of surface friction coefficient
slope_variation = np.linspace(-45, 45, 90)  # Variation of slope incline from -45 to 45 degrees

# Run simulations for different scenarios
results_pothole = sim.run_scenario(sim.simulate_pothole, 0.05)
results_pinecone = sim.run_scenario(sim.simulate_pinecone, 0.05)
results_wind = sim.run_scenario(sim.simulate_wind, 10)  # Example wind force
results_acceleration = sim.run_scenario(sim.simulate_acceleration, np.linspace(0, 5, 10))  # Example acceleration range
results_braking = sim.run_scenario(sim.simulate_braking, np.linspace(0, 5, 10))  # Example deceleration range

# Plot results
pothole_plotter = ScenarioPlotter(results_pothole, "pothole")
pinecone_plotter = ScenarioPlotter(results_pinecone, "pinecone")
wind_plotter = ScenarioPlotter(results_wind, "wind")
acceleration_plotter = ScenarioPlotter(results_acceleration, "acceleration")
braking_plotter = ScenarioPlotter(results_braking, "braking")

pothole_plotter.plot()
pinecone_plotter.plot()
wind_plotter.plot()
acceleration_plotter.plot()
braking_plotter.plot()

# Visualize Pothole scenario
sim.visualize_scenario(sim.simulate_pothole, 0.05)
# Visualize Pinecone scenario
sim.visualize_scenario(sim.simulate_pinecone, 0.05)
# Visualize Wind scenario
sim.visualize_scenario(sim.simulate_wind, 10)
# Visualize Acceleration scenario
sim.visualize_scenario(sim.simulate_acceleration, np.linspace(0, 5, 10))
# Visualize Braking scenario
sim.visualize_scenario(sim.simulate_braking, np.linspace(0, 5, 10))