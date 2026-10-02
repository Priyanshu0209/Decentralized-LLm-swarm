from PySide6.QtWidgets import QWidget, QVBoxLayout, QGraphicsView, QGraphicsScene, QGroupBox, QHBoxLayout, QCheckBox
from PySide6.QtCore import Qt, QTimer, QPointF
from PySide6.QtGui import QPen, QBrush, QColor, QPainter, QPolygonF
import math

class MapPanel(QWidget):
    def __init__(self, agent, parent=None):
        super().__init__(parent)
        self.agent = agent
        self.init_ui()
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(500)
        
        self.drone_trails = {} # drone_id -> list of QPointF
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        group = QGroupBox("Swarm Map")
        vbox = QVBoxLayout()
        
        # Checkboxes for toggling overlays
        hbox = QHBoxLayout()
        self.cb_drones = QCheckBox("Drone Positions")
        self.cb_drones.setChecked(True)
        self.cb_trails = QCheckBox("GPS Trail")
        self.cb_trails.setChecked(True)
        self.cb_links = QCheckBox("Neighbor Links")
        self.cb_links.setChecked(True)
        self.cb_radius = QCheckBox("Collision Radius")
        
        hbox.addWidget(self.cb_drones)
        hbox.addWidget(self.cb_trails)
        hbox.addWidget(self.cb_links)
        hbox.addWidget(self.cb_radius)
        vbox.addLayout(hbox)
        
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        # Assuming origin is center, scale arbitrary pixels to meters roughly
        self.view.setSceneRect(-500, -500, 1000, 1000)
        
        vbox.addWidget(self.view)
        group.setLayout(vbox)
        layout.addWidget(group)
        
    def update_map(self):
        self.scene.clear()
        
        # Draw axes/grid (optional, simplified)
        pen_grid = QPen(QColor(50, 50, 50))
        self.scene.addLine(-500, 0, 500, 0, pen_grid)
        self.scene.addLine(0, -500, 0, 500, pen_grid)
        
        if not self.agent:
            return
            
        if not hasattr(self.agent, 'communication') or not self.agent.communication:
            txt = self.scene.addText("No Telemetry")
            from PySide6.QtGui import QFont
            txt.setFont(QFont("Arial", 16, QFont.Bold))
            txt.setDefaultTextColor(Qt.red)
            txt.setPos(-50, -10)
            return
            
        drones = {}
        
        home_lat, home_lon = 37.0, -122.0
        
        # Use first neighbor as home anchor for relative mapping
        neighbors = self.agent.communication.neighbors
        if len(neighbors) > 0:
            import time
            for n_id, data in neighbors.items():
                if time.time() - data['last_heard'] < 5.0:
                    if not hasattr(self, '_initial_lat'):
                        self._initial_lat = data['last_packet'].gps_lat
                        self._initial_lon = data['last_packet'].gps_lon
                    home_lat = self._initial_lat
                    home_lon = self._initial_lon
                    # Use local position for accurate relative rendering in SITL where GPS offsets are identical
                    nx = data['last_packet'].local_pos_y
                    ny = data['last_packet'].local_pos_x
                    drones[n_id] = {'x': nx, 'y': ny, 'heading': data['last_packet'].heading}
                    
        # Update trails
        for d_id, d_pos in drones.items():
            if d_id not in self.drone_trails:
                self.drone_trails[d_id] = []
            pt = QPointF(d_pos['x'], -d_pos['y']) # Qt y is down
            self.drone_trails[d_id].append(pt)
            if len(self.drone_trails[d_id]) > 100:
                self.drone_trails[d_id].pop(0)
                
        # Draw links
        if self.cb_links.isChecked():
            pen_link = QPen(QColor(0, 150, 255, 100), 2, Qt.DashLine)
            for d_id, pos1 in drones.items():
                for other_id, pos2 in drones.items():
                    if d_id < other_id:
                        self.scene.addLine(pos1['x'], -pos1['y'], pos2['x'], -pos2['y'], pen_link)
                        
        # Draw trails
        if self.cb_trails.isChecked():
            pen_trail = QPen(QColor(150, 150, 150, 150), 1)
            for d_id, trail in self.drone_trails.items():
                for i in range(1, len(trail)):
                    self.scene.addLine(trail[i-1].x(), trail[i-1].y(), trail[i].x(), trail[i].y(), pen_trail)
                    
        # Draw collision radius
        if self.cb_radius.isChecked():
            radius = self.agent.config.get('collision_radius', 5.0)
            pen_rad = QPen(QColor(255, 100, 100, 150), 1, Qt.DotLine)
            brush_rad = QBrush(QColor(255, 100, 100, 30))
            for d_id, pos in drones.items():
                self.scene.addEllipse(pos['x'] - radius, -pos['y'] - radius, radius*2, radius*2, pen_rad, brush_rad)
                
        # Draw drones
        if self.cb_drones.isChecked():
            pen_drone = QPen(Qt.black, 1)
            my_id = self.agent.config.get('drone_id', 0) if hasattr(self.agent, 'config') else 0
            for d_id, pos in drones.items():
                brush = QBrush(QColor(0, 200, 0) if d_id == my_id else QColor(200, 100, 0))
                
                # Draw triangle for drone heading
                poly = QPolygonF()
                size = 10
                poly.append(QPointF(0, -size))
                poly.append(QPointF(-size*0.8, size))
                poly.append(QPointF(size*0.8, size))
                
                item = self.scene.addPolygon(poly, pen_drone, brush)
                item.setPos(pos['x'], -pos['y'])
                item.setRotation(pos['heading'])
                
                # Text label
                txt = self.scene.addText(f"D{d_id}")
                txt.setDefaultTextColor(Qt.white)
                txt.setPos(pos['x'] + 5, -pos['y'] + 5)
