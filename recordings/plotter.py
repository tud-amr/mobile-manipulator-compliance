import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

L = 0.195
D = 0.145
R = 0.315
K = 0.04


class Plotter:
    """Plot the data."""

    def __init__(self) -> None:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        directory = "2024-01-05/4"
        name = "compliant"
        self.data = pd.read_csv(dir_path + "/" + directory + "/" + name + ".csv")
        self.data.columns = self.data.columns.str.replace("/record/", "")

        self.processed_data = {}

        self.process_data()
        # self.plot_xy()
        self.plot_force()
        # self.calculate_error()

        df = pd.DataFrame(self.processed_data)
        # df.to_csv("processed.csv", index=False)

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
        origin = (0, D)
        circle = plt.Circle(origin, L, fill=False)
        fig = plt.gcf()
        ax = fig.gca()
        ax.add_patch(circle)
        plt.show()

    def plot_force(self) -> None:
        """Plot force."""
        error = np.linalg.norm(self.pos_x[:, :2] - self.pos_t[:, :2], axis=1)
        error *= 1000  # m -> mm

        length = np.linalg.norm(self.pos_x[:, :2] - np.array([D, 0]), axis=1)
        stretch = length - L
        stretch *= 1000  # m -> mm
        force = K * stretch

        error_filtered = []
        force_filtered = []
        time = []

        # Filter out negative stretch values, since the spring has no negative stretch:
        for n in range(len(stretch)):
            if stretch[n] > 0:
                error_filtered.append(error[n])
                force_filtered.append(force[n])
                time.append(self.time[n])

        plt.scatter(error_filtered, force_filtered, s=1)

        print(np.poly1d(np.polyfit(error_filtered, force_filtered, 1)))

        x = error_filtered
        y = np.poly1d(np.polyfit(error_filtered, force_filtered, 1))(x)
        plt.plot(x, y, c="orange")
        plt.show()

        goal_force = np.array(error_filtered) * 0.04
        actual_force = np.array(force_filtered)
        error = np.absolute(goal_force - actual_force)
        print(np.mean(error))

        self.processed_data["error"] = error_filtered
        self.processed_data["force"] = force_filtered
        self.processed_data["fit_x"] = x
        self.processed_data["fit_y"] = y
        self.processed_data["time"] = time

    def calculate_error(self) -> None:
        """Plot error."""
        length = np.linalg.norm(self.pos_x[:, :2] - np.array([D, 0]), axis=1)
        inside = self.pos_x[:, :2][length <= L]

        r_target = np.mean(np.linalg.norm(self.pos_t[:, :2], axis=1))
        r_actual = np.linalg.norm(inside[:, :2], axis=1)
        errors = np.absolute(r_actual - r_target)

        avg_error = np.mean(errors)
        print(round(avg_error * 1000, 2))


Plotter()
