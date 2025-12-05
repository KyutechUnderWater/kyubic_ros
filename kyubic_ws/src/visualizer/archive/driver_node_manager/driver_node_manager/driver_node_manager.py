import subprocess
import time
import tkinter as tk
from threading import Thread
from tkinter import ttk


class NodeManager:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS 2 Node Manager")

        self.processes = {}  # ノードプロセスを管理
        self.nodes = [
            {
                "name": "Depth Node",
                "package": "depth_driver",
                "executable": "depth_driver",
                "node_name": "/depth",
                "topic": "/depth",
            },
        ]
        self.node_status = {}  # ノード状態を管理
        self.build_ui()
        self.update_status_thread = Thread(target=self.update_status_loop, daemon=True)
        self.update_status_thread.start()

    def build_ui(self):
        for node in self.nodes:
            frame = ttk.Frame(self.root)
            frame.pack(fill=tk.X, padx=5, pady=5)

            # 起動/停止ボタン
            toggle_button = ttk.Button(
                frame,
                text=f"Start {node['name']}",
                command=lambda n=node: self.toggle_node(n),
            )
            toggle_button.pack(side=tk.LEFT, padx=5)

            # 状態表示ラベル
            status_label = ttk.Label(frame, text="Stopped", width=10)
            status_label.pack(side=tk.LEFT, padx=5)

            # topic echo ボタン
            echo_button = ttk.Button(
                frame,
                text=f"Echo {node['name']}",
                command=lambda n=node: self.run_topic_echo(n),
            )
            echo_button.pack(side=tk.LEFT, padx=5)

            self.node_status[node["node_name"]] = {
                "label": status_label,
                "button": toggle_button,
            }

        # Quitボタン
        quit_button = ttk.Button(self.root, text="Quit", command=self.close_application)
        quit_button.pack(pady=10)

    def toggle_node(self, node):
        node_name = node["node_name"]
        if node_name not in self.processes:
            # ノードを起動
            process = subprocess.Popen(["ros2", "run", node["package"], node["executable"]])
            self.processes[node_name] = process
            self.node_status[node_name]["label"].config(text="Running")
            self.node_status[node_name]["button"].config(text=f"Stop {node['name']}")
        else:
            # ノードを停止
            self.stop_node(node)

    def stop_node(self, node):
        node_name = node["node_name"]
        process = self.processes.get(node_name)
        if process:
            subprocess.run(["pkill", "-f", node["executable"]])
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            if process.poll() is not None:
                del self.processes[node_name]
        self.node_status[node_name]["label"].config(text="Stopped")
        self.node_status[node_name]["button"].config(text=f"Start {node['name']}")

    def run_topic_echo(self, node):
        topic = node.get("topic", f"/{node['node_name']}")
        subprocess.Popen(["gnome-terminal", "--", "ros2", "topic", "echo", topic])

    def update_status_loop(self):
        while True:
            try:
                active_nodes = subprocess.check_output(
                    ["ros2", "node", "list"], text=True
                ).splitlines()
            except subprocess.CalledProcessError:
                active_nodes = []

            for node in self.nodes:
                node_name = node["node_name"]
                if node_name in active_nodes:
                    self.node_status[node_name]["label"].config(text="Running")
                    self.node_status[node_name]["button"].config(text=f"Stop {node['name']}")
                else:
                    self.node_status[node_name]["label"].config(text="Stopped")
                    self.node_status[node_name]["button"].config(text=f"Start {node['name']}")

            time.sleep(1)

    def close_application(self):
        for node in self.nodes:
            self.stop_node(node)
        self.root.destroy()


def main():
    root = tk.Tk()
    app = NodeManager(root)
    root.mainloop()
