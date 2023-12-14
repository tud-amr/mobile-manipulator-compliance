import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

import importlib.resources as pkg_resources
import casadi
import compliant_control.control.symbolics as symbolics


class Plotter:
    """Plot the data."""

    def __init__(self) -> None:
        self.load_symbolics()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        directory = "2023-12-14"
        name = "no_null_no_fric"
        self.data = pd.read_csv(dir_path + "/" + directory + "/" + name + ".csv")
        self.data.columns = self.data.columns.str.replace("/record/", "")

        self.process_data()
        # self.plot_xy()
        # self.plot_force()
        # self.plot_torques()
        self.plot_error()

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

    def plot_xy(self) -> None:
        """Plot xy."""
        plt.gca().set_aspect("equal")
        plt.plot(self.pos_x[:, 0], self.pos_x[:, 1])
        plt.plot(self.pos_t[:, 0], self.pos_t[:, 1])
        plt.show()

    def plot_force(self) -> None:
        """Plot force."""
        error = self.pos_t - self.pos_x
        abs_error = np.linalg.norm(error, axis=1)
        force = np.array(
            [self.F(self.pos_q[n], self.cur_comp[n]) for n in range(len(self.pos_q))]
        )
        abs_force = np.linalg.norm(force, axis=1)
        plt.scatter(abs_error, abs_force, s=1)
        plt.plot(
            np.unique(abs_error),
            np.poly1d(np.polyfit(abs_error, abs_force, 1))(np.unique(abs_error)),
            c="orange",
        )
        plt.show()

    def plot_torques(self) -> None:
        """Plot torques."""
        plt.plot(self.time, self.cur_fb)
        plt.show()

    def plot_error(self) -> None:
        """Plot error."""
        error = self.pos_t - self.pos_x
        abs_error = np.linalg.norm(error, axis=1)
        plt.plot(self.time, abs_error)
        plt.show()

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
