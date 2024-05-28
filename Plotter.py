import numpy as np
import matplotlib.pyplot as plt

# Base Class for plotting
class Plotter:
    def __init__(self, results):
        self.speeds = np.array([res[0] for res in results])
        self.angles = np.array([res[1] for res in results])
        self.scenarios = np.array([res[2] for res in results])
        self.stability_measures = np.array([res[3] for res in results])

    def plot(self):
        raise NotImplementedError("Subclasses should implement this method")

# Derived class for specific scenario plots
class ScenarioPlotter(Plotter):
    def __init__(self, results, scenario):
        super().__init__(results)
        self.scenario = scenario

    def plot(self):
        # Subplot 1: Speed vs. Displacement
        fig, ax1 = plt.subplots(figsize=(7, 5))
        mask = self.scenarios == self.scenario
        ax1.plot(self.speeds[mask], self.stability_measures[mask], label=self.scenario)
        ax1.set_xlabel('Speed (km/hr)')
        ax1.set_ylabel('Displacement (m)')
        ax1.legend()
        ax1.set_title(f'{self.scenario} - Speed vs. Displacement')
        fig.savefig(f'SimData/{self.scenario}_speed_vs_displacement.png')  # Save subplot 1
        plt.close(fig)

        # Subplot 2: Slope Angle vs. Critical Rider Tilt Angle
        fig, ax2 = plt.subplots(figsize=(7, 5))
        for speed in [5, 10, 15]:
            mask_speed = (self.speeds == speed) & mask
            ax2.scatter(self.angles[mask_speed], self.stability_measures[mask_speed], label=f'{speed} km/hr')
        ax2.set_xlabel('Slope Angle (degrees)')
        ax2.set_ylabel('Critical Rider Tilt Angle (degrees)')
        ax2.legend()
        ax2.set_title(f'{self.scenario} - Slope Angle vs. Critical Rider Tilt Angle')
        fig.savefig(f'SimData/{self.scenario}_slope_angle_vs_rider_tilt_angle.png')  # Save subplot 2
        plt.close(fig)