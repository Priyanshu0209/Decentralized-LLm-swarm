import sys
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                               QSplitter, QTabWidget)
from PySide6.QtCore import Qt

from .toolbar import Toolbar
from .mission_panel import MissionPanel
from .parameter_panel import ParameterPanel
from .map_panel import MapPanel
from .swarm_panel import SwarmPanel
from .telemetry_panel import TelemetryPanel
from .log_panel import LogPanel

class MainWindow(QMainWindow):
    def __init__(self, agent):
        super().__init__()
        self.agent = agent
        self.setWindowTitle("DroneAgent Ground Control Station")
        self.setMinimumSize(1280, 800)
        
        self.init_ui()
        
    def init_ui(self):
        # Main central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # 1. Top Toolbar
        self.toolbar = Toolbar(self.agent)
        main_layout.addWidget(self.toolbar)
        
        # Main Splitter (Vertical, splits Map/Panels from Logs)
        v_splitter = QSplitter(Qt.Vertical)
        
        # Upper Splitter (Horizontal, splits Left, Center, Right)
        h_splitter = QSplitter(Qt.Horizontal)
        
        # -- Left Panel (Tabs) --
        left_tabs = QTabWidget()
        self.mission_panel = MissionPanel(self.agent)
        self.parameter_panel = ParameterPanel(self.agent)
        left_tabs.addTab(self.mission_panel, "Mission")
        left_tabs.addTab(self.parameter_panel, "Parameters")
        h_splitter.addWidget(left_tabs)
        
        # -- Center (Map) --
        self.map_panel = MapPanel(self.agent)
        h_splitter.addWidget(self.map_panel)
        
        # -- Right Panel (Splitter/Tabs) --
        right_tabs = QTabWidget()
        
        # Combine Swarm, Telemetry, Movement in Right Panel
        self.swarm_panel = SwarmPanel(self.agent)
        self.telemetry_panel = TelemetryPanel(self.agent)
        
        right_tabs.addTab(self.telemetry_panel, "Telemetry")
        right_tabs.addTab(self.swarm_panel, "Swarm Control")
        
        h_splitter.addWidget(right_tabs)
        
        # Set Splitter Ratios (e.g. 20% left, 60% center, 20% right)
        h_splitter.setSizes([300, 800, 300])
        
        v_splitter.addWidget(h_splitter)
        
        # -- Bottom Panel (Logs) --
        self.log_panel = LogPanel(self.agent)
        v_splitter.addWidget(self.log_panel)
        v_splitter.setSizes([700, 200])
        
        main_layout.addWidget(v_splitter)

    def closeEvent(self, event):
        # Graceful shutdown when window is closed
        if self.agent and hasattr(self.agent, 'shutdown'):
            self.agent.shutdown()
        event.accept()
