from PySide6.QtWidgets import QWidget, QVBoxLayout, QFormLayout, QDoubleSpinBox, QGroupBox, QPushButton

class ParameterPanel(QWidget):
    def __init__(self, agent, parent=None):
        super().__init__(parent)
        self.agent = agent
        self.params = {}
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        group = QGroupBox("Live Parameters")
        form = QFormLayout()
        
        param_configs = {
            "Speed (m/s)": (0.0, 30.0, 5.0),
            "Acceleration (m/s^2)": (0.0, 10.0, 2.0),
            "Yaw Rate (deg/s)": (0.0, 180.0, 45.0),
            "Altitude (m)": (0.0, 500.0, 10.0),
            "RTL Altitude (m)": (10.0, 100.0, 30.0),
            "Formation Spacing (m)": (1.0, 100.0, 5.0),
            "Formation Heading (deg)": (0.0, 360.0, 0.0),
            "Collision Radius (m)": (0.5, 20.0, 2.0),
            "Safe Distance (m)": (1.0, 50.0, 5.0),
            "Neighbor Timeout (s)": (0.5, 10.0, 3.0),
            "Heartbeat Freq (Hz)": (1.0, 50.0, 10.0),
            "Comm Retry": (0.0, 10.0, 3.0),
            "Telemetry Rate (Hz)": (1.0, 100.0, 20.0),
            "GPS Filter": (0.0, 1.0, 0.5)
        }
        
        for name, (min_val, max_val, default) in param_configs.items():
            spin = QDoubleSpinBox()
            spin.setRange(min_val, max_val)
            spin.setValue(default)
            form.addRow(name, spin)
            self.params[name] = spin
            
        self.btn_update = QPushButton("Update Parameters")
        form.addRow(self.btn_update)
        
        group.setLayout(form)
        layout.addWidget(group)
        layout.addStretch()
        
        self.btn_update.clicked.connect(self.update_parameters)
        
    def update_parameters(self):
        # Update configurations based on user inputs dynamically
        if not self.agent:
            return
            
        if not hasattr(self.agent, 'config'):
            self.agent.config = {}
            
        params_to_send = {}
        
        for name, spinbox in self.params.items():
            val = spinbox.value()
            if name == "Speed (m/s)":
                self.agent.config['max_speed'] = val
                params_to_send['max_velocity'] = val
            elif name == "Altitude (m)":
                if 'mission' not in self.agent.config: self.agent.config['mission'] = {}
                self.agent.config['mission']['takeoff_altitude'] = val
                params_to_send['takeoff_altitude'] = val
            elif name == "RTL Altitude (m)":
                params_to_send['rtl_altitude'] = val
            elif name == "Formation Spacing (m)":
                params_to_send['formation_spacing'] = val
            elif name == "Formation Heading (deg)":
                params_to_send['formation_heading'] = val * 3.14159265 / 180.0
            elif name == "Collision Radius (m)":
                self.agent.config['collision_radius'] = val
                params_to_send['collision_radius'] = val
            elif name == "Heartbeat Freq (Hz)":
                self.agent.config['heartbeat_frequency'] = val
                params_to_send['heartbeat_rate'] = val
                
        if hasattr(self.agent, 'communication'):
            self.agent.communication.broadcast_command('parameter_update', params_to_send)
                
        if hasattr(self.agent, 'logger'):
            self.agent.logger.info("Live parameters updated from GUI and broadcast to swarm.")
