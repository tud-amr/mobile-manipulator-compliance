import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
import pandas as pd

import importlib.resources as pkg_resources
import casadi
import compliant_control.control.symbolics as symbolics


RATIOS = np.array([1.03, 0.31, 1.03, 1.90, 2.09, 1.99])
L = 0.260


class Plotter:
    """Plot the data."""

    def __init__(self) -> None:
        self.load_symbolics()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        directory = "2023-12-14"
        name = "compliant"
        self.data = pd.read_csv(dir_path + "/" + directory + "/" + name + ".csv")
        self.data.columns = self.data.columns.str.replace("/record/", "")

        self.processed_data = {}

        self.process_data()
        # self.plot_xy()
        self.plot_force()
        # self.plot_torques()
        # self.calculate_error()

        df = pd.DataFrame(self.processed_data)
        df.to_csv("processed.csv", index=False)

    def F(self, q: np.ndarray, torques: np.ndarray) -> np.ndarray:
        """End-effector force."""
        return np.reshape(self.casadi_F(q, torques), (3))

    def process_data(self) -> None:
        """Process the data."""
        self.pos_x = self.data[["pos_x" + f".{n}" for n in range(3)]].to_numpy()
        self.pos_t = self.data[["pos_t" + f".{n}" for n in range(3)]].to_numpy()
        self.pos_q = self.data[["pos_q" + f".{n}" for n in range(6)]].to_numpy()
        self.cur_comp = self.data[["cur_comp" + f".{n}" for n in range(6)]].to_numpy()
        self.cur_fb = self.data[["cur_fb" + f".{n}" for n in range(6)]].to_numpy()
        self.time = self.data["__time"].to_numpy()
        self.time -= self.time[0]
        self.processed_data["time"] = self.time

    def plot_xy(self) -> None:
        """Plot xy."""
        plt.gca().set_aspect("equal")
        plt.plot(self.pos_x[:, 1], self.pos_x[:, 0])
        plt.plot(self.pos_t[:, 1], self.pos_t[:, 0])
        origin = (0, 0.145)
        l_rope = 0.260
        circle = plt.Circle(origin, l_rope, fill=False)
        fig = plt.gcf()
        ax = fig.gca()
        ax.add_patch(circle)
        plt.show()

    def plot_force(self) -> None:
        """Plot force."""
        error = self.pos_t - self.pos_x
        abs_error = np.linalg.norm(error, axis=1)
        abs_error *= 1000  # m -> mm

        torques = self.cur_comp / RATIOS
        force = np.array(
            [self.F(self.pos_q[n], torques[n]) for n in range(len(self.pos_q))]
        )
        abs_force = np.linalg.norm(force, axis=1)
        plt.scatter(abs_error, abs_force, s=1)

        x = np.unique(abs_error)
        y = np.poly1d(np.polyfit(abs_error, abs_force, 1))(x)
        plt.plot(x, y, c="orange")
        plt.show()

        self.processed_data["abs_error"] = abs_error
        self.processed_data["abs_force"] = abs_force
        self.processed_data["fit_x"] = x
        self.processed_data["fit_y"] = y

    def plot_torques(self) -> None:
        """Plot torques."""
        plt.plot(self.time, self.cur_fb)
        plt.show()

    def calculate_error(self) -> None:
        """Plot error."""
        error = self.pos_x - self.pos_t
        abs_error = np.linalg.norm(error, axis=1)

        l = self.pos_t[:, :2] - np.array([0.315, 0])
        l = np.linalg.norm(l, axis=1)
        l = np.where(l >= L, 0, None)

        inside_error = np.mean(abs_error[l != 0])
        outside_error = np.mean(abs_error[l == 0])
        print(inside_error, outside_error)

        self.processed_data["x_x"] = self.pos_x[:, 1]
        self.processed_data["t_x"] = self.pos_t[:, 1]
        self.processed_data["x_y"] = self.pos_x[:, 0]
        self.processed_data["t_y"] = self.pos_t[:, 0]
        self.processed_data["l"] = l

    def load_symbolics(self) -> None:
        """Load the symbolics."""
        input_dir = str(pkg_resources.files(symbolics))
        current_dir = os.getcwd()
        os.chdir(input_dir)
        for file_name in os.listdir(input_dir):
            if file_name.endswith(".so"):
                name = file_name.replace(".so", "")
                f = casadi.external(name, file_name)
                setattr(self, f"casadi_{name}", f)
        os.chdir(current_dir)


Plotter()
