import numpy as np
import matplotlib.pyplot as plt

class SevenSegmentTrajectory:
    def __init__(self):
        # Input data
        self.TrajectoryType = 'q2q'
        self.InitialPosition = np.array([1, 3, 8])  # [x, y, z]
        self.FinalPosition = np.array([4, 6, 9])    # [x, y, z]
        self.q0 = 0
        self.q1 = 10
        self.v0 = 1
        self.v1 = 0
        self.v_max = 10
        self.a_max = 10
        self.j_max = 30
        self.dt = 0.001
        self.lambda_val = 0.99
        self.Error = []
        self.TimeTrajectory = []
        self.q = []
        self.v = []
        self.a = []
        self.j = []
        self.Position = []
        self.Velocity = []
        self.Acceleration = []
        self.Jerk = []
        self.Error_List = []

    def build_error_list(self):
        self.Error_List = [
            'Error = 1 : Initial and final velocities have to be less than maximum velocity (v_max).',
            'Error = 2 : The starting and ending points are the same',
            'Error = 3 : The maximum values of velocity, acceleration, and jerk must be positive',
            'Error = 4 : It is not possible to create Seven Segment trajectory for this case. Please change one of the maximum speed or acceleration or jerk values',
            'Error = 5 : It is not possible to create Seven Segment trajectory for this case. Please change one of the maximum speed or acceleration or jerk values',
            'Error = 6 : (Delta = 0) It is not possible to create Seven Segment trajectory for this case. Please change one of the maximum speed or acceleration or jerk values',
            'Error = 7 : In this case summation of v0 and v1 must not be zero',
            'Error = 8 : It is not possible to create Seven Segment trajectory for this case. Please change one of the maximum speed or acceleration or jerk values',
            'Error = 9 : In this case summation of v0 and v1 must not be zero',
            'Error = 10: It is not possible to create Seven Segment trajectory for this case. Please change one of the maximum speed or acceleration or jerk values',
            'Error = 11: It is not possible to create Seven Segment trajectory for this case. Please change one of the maximum speed or acceleration or jerk values'
        ]

    def coordinate_conversion(self, trajectory_type):
        if trajectory_type == 'p2q':
            self.q0 = 0
            self.q1 = ((self.InitialPosition[0] - self.FinalPosition[0]) ** 2 +
                       (self.InitialPosition[1] - self.FinalPosition[1]) ** 2 +
                       (self.InitialPosition[2] - self.FinalPosition[2]) ** 2) ** 0.5
        else:
            m = [(self.FinalPosition[0] - self.InitialPosition[0]) / self.q1,
                 (self.FinalPosition[1] - self.InitialPosition[1]) / self.q1,
                 (self.FinalPosition[2] - self.InitialPosition[2]) / self.q1]
            self.Position = np.dot(self.q, np.transpose(m)) + np.transpose(self.InitialPosition)
            self.Velocity = np.dot(self.v, np.transpose(m))
            self.Acceleration = np.dot(self.a, np.transpose(m))
            self.Jerk = np.dot(self.j, np.transpose(m))

    def BuildTrajectory(self):
        self.build_error_list()
        
        if self.TrajectoryType == 'p2p':
            self.coordinate_conversion(self, 'p2q')
        
        error = 0
        
        for one_counter in range(1):
            
            if max(self.v0, self.v1) > self.v_max:
                error = 1
                continue
            if self.q0 == self.q1:
                error = 2
                continue
            if min(self.v_max, self.a_max, self.j_max) <= 0:
                error = 3
                continue

            v_min = -self.v_max
            a_min = -self.a_max
            j_min = -self.j_max

            try:
                sigma = 2
                if self.q1 < self.q0:
                    sigma = np.sign(self.q1 - self.q0)
                    self.q0 = sigma * self.q0
                    self.q1 = sigma * self.q1
                    self.v0 = sigma * self.v0
                    self.v1 = sigma * self.v1

                    self.v_max = (sigma + 1) / 2 * self.v_max + (sigma - 1) / 2 * v_min
                    v_min = (sigma + 1) / 2 * v_min + (sigma - 1) / 2 * self.v_max
                    self.a_max = (sigma + 1) / 2 * self.a_max + (sigma - 1) / 2 * a_min
                    a_min = (sigma + 1) / 2 * a_min + (sigma - 1) / 2 * self.a_max
                    self.j_max = (sigma + 1) / 2 * self.j_max + (sigma - 1) / 2 * j_min
                    j_min = (sigma + 1) / 2 * j_min + (sigma - 1) / 2 * self.j_max

                # functions
                Tj_star = min(np.sqrt(abs(self.v1 - self.v0) / self.j_max), self.a_max / self.j_max)

                if Tj_star < self.a_max / self.j_max:
                    if (self.q1 - self.q0) <= Tj_star * (self.v0 + self.v1):
                        error = 4
                        continue
                else:
                    if (self.q1 - self.q0) <= 0.5 * (self.v0 + self.v1) * (Tj_star + abs(self.v1 - self.v0) / self.a_max):
                        error = 5
                        continue

                # Case 1 v_lim = self.v_max
                if (self.v_max - self.v0) * self.j_max < self.a_max ** 2:
                    Tj_1 = np.sqrt((self.v_max - self.v0) / self.j_max)
                    Ta = 2 * Tj_1
                else:
                    Tj_1 = self.a_max / self.j_max
                    Ta = Tj_1 + (self.v_max - self.v0) / self.a_max

                if (self.v_max - self.v1) * self.j_max < self.a_max ** 2:
                    Tj_2 = np.sqrt((self.v_max - self.v1) / self.j_max)
                    Td = 2 * Tj_2
                else:
                    Tj_2 = self.a_max / self.j_max
                    Td = Tj_2 + (self.v_max - self.v1) / self.a_max

                Tv = (self.q1 - self.q0) / self.v_max - Ta / 2 * (1 + self.v0 / self.v_max) - Td / 2 * (1 + self.v1 / self.v_max)
                # end Case 1

                if Tv <= 0:  # Case 2 v_lim < self.v_max
                    Tv = 0
                    # suppose self.a_max is reached
                    while True:
                        Tj = self.a_max / self.j_max
                        Tj_1 = Tj
                        Tj_2 = Tj

                        delta = self.a_max ** 4 / self.j_max ** 2 + 2 * (self.v0 ** 2 + self.v1 ** 2) + self.a_max * (
                                    4 * (self.q1 - self.q0) - 2 * self.a_max / self.j_max * (self.v0 + self.v1))

                        if delta < 0:
                            error = 6
                            continue

                        Ta = (self.a_max ** 2 / self.j_max - 2 * self.v0 + np.sqrt(delta)) / (2 * self.a_max)

                        Td = (self.a_max ** 2 / self.j_max - 2 * self.v1 + np.sqrt(delta)) / (2 * self.a_max)

                        if Ta < 0:
                            if (self.v1 + self.v0) == 0:
                                error = 7
                                continue
                            Ta = 0
                            Tj_1 = 0
                            Td = 2 * ((self.q1 - self.q0) / (self.v1 + self.v0))
                            Tj_2 = (self.j_max * (self.q1 - self.q0) - np.sqrt(
                                    self.j_max * (self.j_max * (self.q1 - self.q0) ** 2 + (self.v1 + self.v0) ** 2 * (
                                    self.v1 - self.v0)))) / (self.j_max * (self.v1 + self.v0))
                            if Tj_2 <= 0:
                                error = 8
                                continue
                            break

                        if Td < 0:
                            if (self.v1 + self.v0) == 0:
                                error = 9
                                continue
                            Td = 0
                            Tj_2 = 0
                            Ta = 2 * ((self.q1 - self.q0) / (self.v1 + self.v0))
                            Tj_1 = (self.j_max * (self.q1 - self.q0) - np.sqrt(
                                    self.j_max * (self.j_max * (self.q1 - self.q0) ** 2 - (self.v1 + self.v0) ** 2 * (
                                    self.v1 - self.v0)))) / (self.j_max * (self.v1 + self.v0))

                            if Tj_1 <= 0:
                                error = 10
                                continue

                            if Ta < 0:
                                Ta = 0
                                Tj_1 = 0
                                Td = 2 * ((self.q1 - self.q0) / (self.v1 + self.v0))
                                Tj_2 = (self.j_max * (self.q1 - self.q0) - np.sqrt(
                                        self.j_max * (self.j_max * (self.q1 - self.q0) ** 2 + (self.v1 + self.v0) ** 2 * (
                                        self.v1 - self.v0)))) / (self.j_max * (self.v1 + self.v0))
                                if Tj_2 <= 0:
                                    error = 11
                                    continue

                                break
                            break

                        if Ta < 2 * Tj or Td < 2 * Tj:
                            self.a_max = self.lambda_val * self.a_max
                        else:
                            break

                T = Ta + Tv + Td
                a_lim_a = self.j_max * Tj_1
                a_lim_d = -self.j_max * Tj_2
                v_lim = self.v1 - (Td - Tj_2) * a_lim_d

                Time_P1 = np.arange(0, Tj_1 - self.dt, self.dt)
                Time_P2 = np.arange(Tj_1, Ta - Tj_1 - self.dt, self.dt)
                Time_P3 = np.arange(Ta - Tj_1, Ta - self.dt, self.dt)
                Time_P4 = np.arange(Ta, Ta + Tv - self.dt, self.dt)
                Time_P5 = np.arange(T - Td, T - Td + Tj_2 - self.dt, self.dt)
                Time_P6 = np.arange(T - Td + Tj_2, T - Tj_2 - self.dt, self.dt)
                Time_P7 = np.arange(T - Tj_2, T + self.dt, self.dt)

                Q_P1 = self.q0 + self.v0 * Time_P1 + self.j_max / 6 * Time_P1**3
                VQ_P1 = self.v0 + self.j_max / 2 * Time_P1**2
                AQ_P1 = self.j_max * Time_P1
                JQ_P1 = self.j_max * (np.abs(Time_P1) + 1) / (np.abs(Time_P1) + 1)

                Q_P2 = self.q0 + self.v0 * Time_P2 + a_lim_a / 6 * (3 * Time_P2**2 - 3 * Tj_1 * Time_P2 + Tj_1**2)
                VQ_P2 = self.v0 + a_lim_a * (Time_P2 - Tj_1 / 2)
                AQ_P2 = a_lim_a * (np.abs(Time_P2) + 1) / (np.abs(Time_P2) + 1)
                JQ_P2 = np.zeros_like(Time_P2)

                Q_P3 = self.q0 + (v_lim + self.v0) * Ta / 2 - v_lim * (Ta - Time_P3) - j_min / 6 * (Ta - Time_P3)**3
                VQ_P3 = v_lim + j_min / 2 * (Ta - Time_P3)**2
                AQ_P3 = -j_min * (Ta - Time_P3)
                JQ_P3 = -self.j_max * (np.abs(Time_P3) + 1) / (np.abs(Time_P3) + 1)

                Q_P4 = self.q0 + (v_lim + self.v0) * Ta / 2 + v_lim * (Time_P4 - Ta)
                VQ_P4 = v_lim * (np.abs(Time_P4) + 1) / (np.abs(Time_P4) + 1)
                AQ_P4 = np.zeros_like(Time_P4)
                JQ_P4 = np.zeros_like(Time_P4)

                Q_P5 = self.q1 - (v_lim + self.v1) * Td / 2 + v_lim * (Time_P5 - T + Td) - self.j_max / 6 * (Time_P5 - T + Td)**3
                VQ_P5 = v_lim - self.j_max / 2 * (Time_P5 - T + Td)**2
                AQ_P5 = -self.j_max * (Time_P5 - T + Td)
                JQ_P5 = -self.j_max * (np.abs(Time_P5) + 1) / (np.abs(Time_P5) + 1)

                Q_P6 = self.q1 - (v_lim + self.v1) * Td / 2 + v_lim * (Time_P6 - T + Td) + a_lim_d / 6 * (3 * (Time_P6 - T + Td)**2 - 3 * Tj_2 * (Time_P6 - T + Td) + Tj_2**2)
                VQ_P6 = v_lim + a_lim_d * (Time_P6 - T + Td - Tj_2 / 2)
                AQ_P6 = a_lim_d * (np.abs(Time_P6) + 1) / (np.abs(Time_P6) + 1)
                JQ_P6 = np.zeros_like(Time_P6)

                Q_P7 = self.q1 - self.v1 * (T - Time_P7) - self.j_max / 6 * (T - Time_P7)**3
                VQ_P7 = self.v1 + self.j_max / 2 * (T - Time_P7)**2
                AQ_P7 = -self.j_max * (T - Time_P7)
                JQ_P7 = self.j_max * (np.abs(Time_P7) + 1) / (np.abs(Time_P7) + 1)

                Final_Q = np.concatenate((Q_P1, Q_P2, Q_P3, Q_P4, Q_P5, Q_P6, Q_P7))
                Final_V = np.concatenate((VQ_P1, VQ_P2, VQ_P3, VQ_P4, VQ_P5, VQ_P6, VQ_P7))
                Final_A = np.concatenate((AQ_P1, AQ_P2, AQ_P3, AQ_P4, AQ_P5, AQ_P6, AQ_P7))
                Final_J = np.concatenate((JQ_P1, JQ_P2, JQ_P3, JQ_P4, JQ_P5, JQ_P6, JQ_P7))
                
                if sigma != 2:
                    Final_Q = sigma * Final_Q
                    Final_V = sigma * Final_V
                    Final_A = sigma * Final_A
                    Final_J = sigma * Final_J

                self.TimeTrajectory = np.arange(0, T + self.dt, self.dt)
                self.q = Final_Q
                self.v = Final_V
                self.a = Final_A
                self.j = Final_J

                if self.TrajectoryType == 'p2p':
                     self.coordinate_conversion(self, 'q2p')

            except:
                error = -1

        self.Error = error
        if error > 0:
            print(self.Error_List[error])

        if error == -1:
            print('oops! Syntax Error. Check that the entries are correct')
        
        return self

