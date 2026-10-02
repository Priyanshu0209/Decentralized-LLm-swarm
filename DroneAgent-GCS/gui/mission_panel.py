from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                               QGroupBox, QTableWidget, QTableWidgetItem, 
                               QProgressBar, QHeaderView)

class MissionPanel(QWidget):
    def __init__(self, agent, parent=None):
        super().__init__(parent)
        self.agent = agent
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        group = QGroupBox("Mission Management")
        vbox = QVBoxLayout()
        
        # Action Buttons
        hbox_actions = QHBoxLayout()
        self.btn_upload = QPushButton("Mission Upload")
        self.btn_cancel = QPushButton("Mission Cancel")
        hbox_actions.addWidget(self.btn_upload)
        hbox_actions.addWidget(self.btn_cancel)
        vbox.addLayout(hbox_actions)
        
        # Waypoint Editor
        self.table_waypoints = QTableWidget(0, 4)
        self.table_waypoints.setHorizontalHeaderLabels(["Lat", "Lon", "Alt (m)", "Speed"])
        self.table_waypoints.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        vbox.addWidget(self.table_waypoints)
        
        # Editor controls
        hbox_editor = QHBoxLayout()
        self.btn_add_wp = QPushButton("+ Waypoint")
        self.btn_rem_wp = QPushButton("- Waypoint")
        self.btn_clear_wp = QPushButton("Clear All")
        hbox_editor.addWidget(self.btn_add_wp)
        hbox_editor.addWidget(self.btn_rem_wp)
        hbox_editor.addWidget(self.btn_clear_wp)
        vbox.addLayout(hbox_editor)
        
        # Mission Progress
        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Progress: %p%")
        vbox.addWidget(self.progress_bar)
        
        # Mission History
        self.btn_history = QPushButton("Mission History")
        vbox.addWidget(self.btn_history)
        
        group.setLayout(vbox)
        layout.addWidget(group)
        
        # Connections
        self.btn_add_wp.clicked.connect(self.add_waypoint)
        self.btn_rem_wp.clicked.connect(self.remove_waypoint)
        self.btn_clear_wp.clicked.connect(self.clear_waypoints)
        self.btn_upload.clicked.connect(self.upload_mission)
        self.btn_cancel.clicked.connect(self.cancel_mission)
        self.btn_history.clicked.connect(self.show_history)
        
    def add_waypoint(self):
        row = self.table_waypoints.rowCount()
        self.table_waypoints.insertRow(row)
        for col in range(4):
            self.table_waypoints.setItem(row, col, QTableWidgetItem("0.0"))
            
    def remove_waypoint(self):
        row = self.table_waypoints.currentRow()
        if row >= 0:
            self.table_waypoints.removeRow(row)
            
    def clear_waypoints(self):
        self.table_waypoints.setRowCount(0)
        
    def upload_mission(self):
        wps = []
        for row in range(self.table_waypoints.rowCount()):
            try:
                lat = float(self.table_waypoints.item(row, 0).text())
                lon = float(self.table_waypoints.item(row, 1).text())
                alt = float(self.table_waypoints.item(row, 2).text())
                speed = float(self.table_waypoints.item(row, 3).text())
                wps.append({'lat': lat, 'lon': lon, 'alt': alt, 'speed': speed})
            except Exception as e:
                if hasattr(self.agent, 'logger'):
                    self.agent.logger.error(f"Invalid waypoint data at row {row}: {e}")
                
        if hasattr(self.agent, 'mission_manager'):
            self.agent.mission_manager.upload_mission(wps)
            
        if hasattr(self.agent, 'logger'):
            self.agent.logger.info(f"Mission uploaded with {len(wps)} waypoints")
            
    def cancel_mission(self):
        if hasattr(self.agent, 'mission_manager'):
            self.agent.mission_manager.cancel_mission()
            
        if hasattr(self.agent, 'logger'):
            self.agent.logger.info("Mission cancelled from GUI")
            
    def show_history(self):
        if hasattr(self.agent, 'logger'):
            self.agent.logger.info("Mission history view is not yet implemented.")
