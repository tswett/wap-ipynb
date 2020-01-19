from math import pi
import matplotlib.pyplot as plt
import numpy as np

class AircraftSimulator:
    def __init__(self):
        self.resolution = resolution = 100

        self.elevator_max_slew = 2
        self.elevator_delay_ticks = resolution * 1 / 10

        self.alpha_elevator_yintercept = 2.5
        self.alpha_elevator_slope = 7.5

        self.alpha_period = 1
        self.alpha_qfactor = 50

        # Signals
        self.alpha = {0: 0.0}
        self.alpha_vel = {0: 0.0}
        self.alpha_acc = {0: 0.0}
        self.elevator_actual = {0: 0.0}
        self.elevator_input = {0: 0.0}
        
    def run_sim(self, sim_length, control_callback):
        elevator_max_change = self.elevator_max_slew / self.resolution

        alpha_angular_frequency = 2 * pi / self.alpha_period
        # Technically, the above formula is for the *undamped* angular frequency, but no matter. It's close enough.
        alpha_damping_coefficient = - alpha_angular_frequency / self.alpha_qfactor
        alpha_restoring_coefficient = - alpha_angular_frequency ** 2
        
        for t in range(1, sim_length * self.resolution):
            aircraft_state = {'time': t / self.resolution, 'alpha': self.alpha[t-1], 'alpha_vel': self.alpha_vel[t-1]}
            controls = control_callback(aircraft_state)
            self.elevator_input[t] = controls['elevator']
            
            elevator_delayed_input = self.elevator_input.get(t - self.elevator_delay_ticks, 0.0)

            elevator_lower_limit = max(-1, self.elevator_actual[t-1] - elevator_max_change)
            elevator_upper_limit = min(1, self.elevator_actual[t-1] + elevator_max_change)
            self.elevator_actual[t] = max(elevator_lower_limit, min(elevator_upper_limit, elevator_delayed_input))

            commanded_alpha = self.elevator_actual[t] * self.alpha_elevator_slope + self.alpha_elevator_yintercept

            self.alpha_acc[t] = alpha_damping_coefficient * self.alpha_vel[t-1] + alpha_restoring_coefficient * (self.alpha[t-1] - commanded_alpha)

            self.alpha_vel[t] = self.alpha_vel[t-1] + self.alpha_acc[t] / self.resolution
            self.alpha[t] = self.alpha[t-1] + self.alpha_vel[t] / self.resolution
            
        self.alpha_nparray = np.array([(k / self.resolution,v) for k,v in self.alpha.items()])
        self.elevator_actual_nparray = np.array([(k / self.resolution,v) for k,v in self.elevator_actual.items()])
        self.elevator_input_nparray = np.array([(k / self.resolution,v) for k,v in self.elevator_input.items()])
        
    def pyplot(self):
        plt.figure().patch.set_facecolor('white')
        
        plt.plot(self.alpha_nparray[:,0], self.alpha_nparray[:,1])
        
        commanded_alpha_actual = (self.elevator_actual_nparray[:,1] * self.alpha_elevator_slope) + self.alpha_elevator_yintercept
        #plt.plot(self.elevator_actual_nparray[:,0], commanded_alpha_actual)
        
        commanded_alpha_input = (self.elevator_input_nparray[:,1] * self.alpha_elevator_slope) + self.alpha_elevator_yintercept
        #plt.plot(self.elevator_input_nparray[:,0], commanded_alpha_input)
        
        plt.show()