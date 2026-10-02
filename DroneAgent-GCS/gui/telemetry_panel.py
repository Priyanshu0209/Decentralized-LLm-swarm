from PySide6.QtWidgets import QWidget, QVBoxLayout, QFormLayout, QLabel, QGroupBox
from PySide6.QtCore import QTimer

class TelemetryPanel(QWidget):
    def __init__(self, agent, parent=None):
        super().__init__(parent)
        self.agent = agent
        self.labels = {}
        self.init_ui()
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(500) # Update at 2Hz
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        group = QGroupBox("Live Telemetry")
        form = QFormLayout()
        
        fields = [
            "Battery", "GPS", "Latitude", "Longitude", "Altitude", 
            "Velocity", "Heading", "Mode", "Health", "RSSI", 
            "Heartbeat", "Neighbor Count"
        ]
        
        for field in fields:
            label = QLabel("N/A")
            # Style values slightly differently to stand out
            label.setStyleSheet("color: #4da6ff; font-weight: bold;")
            form.addRow(f"{field}:", label)
            self.labels[field] = label
            
        group.setLayout(form)
        layout.addWidget(group)
        layout.addStretch()
        
    def update_telemetry(self):
        if not self.agent:
            return
            
        try:
            import math
            import time
            
            # GCS acts as drone_id = 0. The drone to track is usually 1 (or the currently selected drone).
            drone_id = 1
            if hasattr(self.agent, 'communication') and self.agent.communication:
                neighbors = self.agent.communication.neighbors
                
                # Dynamic drone selection
                if drone_id not in neighbors or time.time() - neighbors.get(drone_id, {}).get('last_heard', 0) >= 5.0:
                    for n_id, data in neighbors.items():
                        if time.time() - data.get('last_heard', 0) < 5.0:
                            drone_id = n_id
                            break
                            
                count = len([n for n, d in neighbors.items() if time.time() - d.get('last_heard', 0) < 5.0])
                self.labels["Neighbor Count"].setText(str(count))
                
                # Check if drone_id is in neighbors and not stale
                if drone_id in neighbors and time.time() - neighbors[drone_id]['last_heard'] < 5.0:
                    packet = neighbors[drone_id]['last_packet']
                    
                    self.labels["Battery"].setText(f"{packet.battery * 100:.1f} %")
                    self.labels["Latitude"].setText(f"{packet.gps_lat:.6f}")
                    self.labels["Longitude"].setText(f"{packet.gps_lon:.6f}")
                    self.labels["Altitude"].setText(f"{packet.gps_alt:.2f} m")
                    
                    speed = math.sqrt(packet.velocity_x**2 + packet.velocity_y**2 + packet.velocity_z**2)
                    self.labels["Velocity"].setText(f"{speed:.2f} m/s")
                    
                    self.labels["Heading"].setText(f"{math.degrees(packet.heading):.1f} deg")
                    
                    # status flags decoding
                    from packet import STATUS_ARMED, STATUS_OFFBOARD, STATUS_GUIDED_MODE
                    if packet.status_flags & STATUS_OFFBOARD:
                        mode = "OFFBOARD"
                    elif packet.status_flags & STATUS_GUIDED_MODE:
                        mode = "GUIDED"
                    elif packet.status_flags & STATUS_ARMED:
                        mode = "ARMED"
                    else:
                        mode = "READY"
                        
                    self.labels["Mode"].setText(mode)
                    
                    self.labels["GPS"].setText("3D Fix") 
                    self.labels["Health"].setText(f"{packet.health * 100:.0f}%")
                    self.labels["RSSI"].setText("-65 dBm")
                else:
                    # Drone disconnected or no data
                    self.labels["Battery"].setText("N/A")
                    self.labels["Latitude"].setText("N/A")
                    self.labels["Longitude"].setText("N/A")
                    self.labels["Altitude"].setText("N/A")
                    self.labels["Velocity"].setText("N/A")
                    self.labels["Heading"].setText("N/A")
                    self.labels["Mode"].setText("DISCONNECTED")
                    self.labels["GPS"].setText("N/A")
                    self.labels["Health"].setText("N/A")
                    self.labels["RSSI"].setText("N/A")
            
        except Exception:
            pass # Fail gracefully if agent is shutting down or state is unavailable
