import rclpy
from rclpy.node import Node
from tum_msgs.msg import TUMDebugSignalNames, TUMDebugValues
import csv
import os
import sys
import threading
from datetime import datetime
from collections import deque
from time import time
import argparse

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QListWidget,
    QListWidgetItem, QPushButton, QMessageBox, QLabel
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class PlotWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time Vehicle Signal Plotter")
        self.resize(1000, 800)
        self.DATA_LEN_MAX = 10000

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.signal_list_widget = QListWidget()
        self.signal_list_widget.setSelectionMode(QListWidget.MultiSelection)
        self.layout.addWidget(QLabel("📋 Select up to 4 signals to plot:"))
        self.layout.addWidget(self.signal_list_widget)

        self.plot_button = QPushButton("Plot Selected Signals")
        self.plot_button.clicked.connect(self.update_selected_signals)
        self.layout.addWidget(self.plot_button)

        self.canvas = FigureCanvas(Figure(figsize=(8, 6)))
        self.layout.addWidget(self.canvas)

        self.axes = []
        self.canvas.figure.tight_layout()

        self.all_signal_names = []
        self.selected_signals = []
        self.data_buffers = {}
        self.time_buffer = deque(maxlen=self.DATA_LEN_MAX)

        self.timer = self.canvas.new_timer(100)  # 100 ms update interval
        self.timer.add_callback(self.update_plot)
        self.timer.start()

    def populate_signals(self, names):
        self.signal_list_widget.clear()
        self.all_signal_names = names
        for name in names:
            item = QListWidgetItem(name)
            self.signal_list_widget.addItem(item)

    def update_selected_signals(self):
        selected_items = self.signal_list_widget.selectedItems()
        if len(selected_items) > 4:
            QMessageBox.warning(self, "Limit Exceeded", "Please select up to 4 signals.")
            return

        self.selected_signals = [item.text() for item in selected_items]
        self.data_buffers = {name: deque(maxlen=self.DATA_LEN_MAX) for name in self.selected_signals}
        self.time_buffer.clear()

        self.axes.clear()
        self.canvas.figure.clf()

        n = len(self.selected_signals)
        for i in range(n):
            ax = self.canvas.figure.add_subplot(2, 2, i + 1)
            ax.set_title(self.selected_signals[i])
            self.axes.append(ax)

        self.canvas.draw()

    def update_data(self, row):
        if not self.selected_signals:
            return
        t = rclpy.clock.Clock().now().nanoseconds / 1e9
        self.time_buffer.append(t)
        for name in self.selected_signals:
            self.data_buffers[name].append(row.get(name, 0.0))

    def update_plot(self):
        if not self.selected_signals:
            return
        for i, name in enumerate(self.selected_signals):
            ax = self.axes[i]
            ax.clear()
            y = list(self.data_buffers[name])
            x = list(self.time_buffer)[-len(y):]
            ax.plot(x, y)
            ax.set_title(name)
        self.canvas.draw()


class VehicleSignalLogger(Node):
    def __init__(self, plot_window: PlotWindow, name: str):
        super().__init__('vehicle_signal_logger')
        self.plot_window = plot_window
        self.name = name

        self.signal_names = []
        self.csv_file = None
        self.csv_writer = None
        self.rows_logged = 0
        self.last_sample_time = time()  # For 100 ms sampling

        self.signal_names_sub = self.create_subscription(
            TUMDebugSignalNames,
            '/debug/simulation/VehicleModel/signal_names',
            self.signal_names_callback,
            10
        )

        self.values_sub = self.create_subscription(
            TUMDebugValues,
            '/debug/simulation/VehicleModel/values',
            self.values_callback,
            10
        )

        logs_dir = os.path.join(os.getcwd(), 'logs')
        os.makedirs(logs_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filepath = os.path.join(logs_dir, f"vehicle_signals_{self.name}_{timestamp}.csv")

        self.get_logger().info("✅ Vehicle signal logger node started.")
        self.get_logger().info("⏳ Waiting for signal names...")

    def signal_names_callback(self, msg: TUMDebugSignalNames):
        if not self.signal_names:
            self.signal_names = msg.names
            self.plot_window.populate_signals(self.signal_names)
            self.start_csv_logging()
            self.get_logger().info(f"📋 Received {len(self.signal_names)} signal names.")

    def start_csv_logging(self):
        self.csv_file = open(self.csv_filepath, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.signal_names)
        self.csv_writer.writeheader()
        self.get_logger().info(f"📝 Logging data to: {self.csv_filepath}")

    def values_callback(self, msg: TUMDebugValues):
        now = time()
        if now - self.last_sample_time < 0.1:  # 100 ms
            return
        self.last_sample_time = now

        if not self.signal_names or self.csv_writer is None:
            self.get_logger().warn("⚠️ Waiting for signal names before logging values...")
            return

        values = msg.values
        if len(values) != len(self.signal_names):
            self.get_logger().warn(f"⚠️ Mismatch: {len(values)} values vs {len(self.signal_names)} signal names.")
            return

        row = dict(zip(self.signal_names, values))
        self.csv_writer.writerow(row)
        self.rows_logged += 1

        self.plot_window.update_data(row)

        if self.rows_logged % 50 == 0:
            self.get_logger().info(f"✅ Logged {self.rows_logged} rows...")

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def ros_spin(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    app = QApplication(sys.argv)
    parser = argparse.ArgumentParser()
    plot_window = PlotWindow()
    plot_window.show()

    parser.add_argument("name", type=str, help="Name to append on output log file.")
    args = parser.parse_args()
    name = args.name

    rclpy.init()
    node = VehicleSignalLogger(plot_window, name)

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
