from PySide6.QtWidgets import (QWidget, QVBoxLayout, QGridLayout, QPushButton, 
                               QGroupBox, QRadioButton, QButtonGroup, QScrollArea)
from PySide6.QtCore import Qt

class SwarmPanel(QWidget):
    def __init__(self, agent, parent=None):
        super().__init__(parent)
        self.agent = agent
        self.init_ui()
        
    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)
        
        # 1. Target Selection
        group_sel = QGroupBox("Target Selection")
        vbox_sel = QVBoxLayout()
        self.btn_group = QButtonGroup(self)
        
        self.radio_all = QRadioButton("All Drones")
        self.radio_all.setChecked(True)
        self.btn_group.addButton(self.radio_all, 0)
        vbox_sel.addWidget(self.radio_all)
        
        for i in range(1, 4):
            radio = QRadioButton(f"Drone{i}")
            self.btn_group.addButton(radio, i)
            vbox_sel.addWidget(radio)
            
        self.btn_group.idClicked.connect(self.on_drone_selected)
        group_sel.setLayout(vbox_sel)
        layout.addWidget(group_sel)
        
        # 2. Movement
        group_mov = QGroupBox("Movement")
        grid_mov = QGridLayout()
        
        self.btn_forward = QPushButton("Forward")
        self.btn_backward = QPushButton("Backward")
        self.btn_left = QPushButton("Left")
        self.btn_right = QPushButton("Right")
        self.btn_up = QPushButton("Up")
        self.btn_down = QPushButton("Down")
        self.btn_rot_left = QPushButton("Rotate L")
        self.btn_rot_right = QPushButton("Rotate R")
        self.btn_hover = QPushButton("Hover")
        self.btn_stop = QPushButton("Stop")
        self.btn_speed_plus = QPushButton("Speed +")
        self.btn_speed_minus = QPushButton("Speed -")
        
        grid_mov.addWidget(self.btn_forward, 0, 1)
        grid_mov.addWidget(self.btn_backward, 2, 1)
        grid_mov.addWidget(self.btn_left, 1, 0)
        grid_mov.addWidget(self.btn_right, 1, 2)
        grid_mov.addWidget(self.btn_hover, 1, 1)
        
        grid_mov.addWidget(self.btn_up, 0, 3)
        grid_mov.addWidget(self.btn_down, 2, 3)
        grid_mov.addWidget(self.btn_rot_left, 1, 3)
        grid_mov.addWidget(self.btn_rot_right, 1, 4)
        
        grid_mov.addWidget(self.btn_stop, 3, 1)
        grid_mov.addWidget(self.btn_speed_plus, 3, 3)
        grid_mov.addWidget(self.btn_speed_minus, 3, 4)
        
        group_mov.setLayout(grid_mov)
        layout.addWidget(group_mov)
        
        # 3. Formation
        group_form = QGroupBox("Formation")
        grid_form = QGridLayout()
        
        formations = ["line", "triangle", "circle", "v", "diamond", "grid"]
        for i, shape in enumerate(formations):
            btn = QPushButton(shape.capitalize())
            btn.clicked.connect(lambda checked=False, s=shape: self.apply_formation(s))
            grid_form.addWidget(btn, i // 2, i % 2)
            
        self.btn_space_plus = QPushButton("Spacing +")
        self.btn_space_minus = QPushButton("Spacing -")
        self.btn_rotate = QPushButton("Rotate Form")
        
        grid_form.addWidget(self.btn_space_plus, 3, 0)
        grid_form.addWidget(self.btn_space_minus, 3, 1)
        grid_form.addWidget(self.btn_rotate, 4, 0, 1, 2)
        
        group_form.setLayout(grid_form)
        layout.addWidget(group_form)
        
        # 4. Flight Control
        group_flight = QGroupBox("Flight Control")
        grid_flight = QGridLayout()
        
        self.btn_arm = QPushButton("Arm")
        self.btn_takeoff = QPushButton("Takeoff")
        self.btn_land = QPushButton("Land")
        self.btn_rtl = QPushButton("RTL")
        self.btn_emergency = QPushButton("Emergency Stop")
        self.btn_emergency.setObjectName("btn_emergency")
        
        grid_flight.addWidget(self.btn_arm, 0, 0)
        grid_flight.addWidget(self.btn_takeoff, 0, 1)
        grid_flight.addWidget(self.btn_land, 1, 0)
        grid_flight.addWidget(self.btn_rtl, 1, 1)
        grid_flight.addWidget(self.btn_emergency, 2, 0, 1, 2)
        
        group_flight.setLayout(grid_flight)
        layout.addWidget(group_flight)
        
        # 5. Mission
        group_mission = QGroupBox("Mission")
        grid_mission = QGridLayout()
        
        self.btn_upload = QPushButton("Upload")
        self.btn_start = QPushButton("Start")
        self.btn_pause = QPushButton("Pause")
        self.btn_resume = QPushButton("Resume")
        self.btn_cancel = QPushButton("Cancel")
        
        grid_mission.addWidget(self.btn_upload, 0, 0)
        grid_mission.addWidget(self.btn_start, 0, 1)
        grid_mission.addWidget(self.btn_pause, 1, 0)
        grid_mission.addWidget(self.btn_resume, 1, 1)
        grid_mission.addWidget(self.btn_cancel, 2, 0, 1, 2)
        
        group_mission.setLayout(grid_mission)
        layout.addWidget(group_mission)
        
        layout.addStretch()
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)
        
        # Connect Signals
        self._connect_signals()
        
    def _connect_signals(self):
        # Movement Timer
        from PySide6.QtCore import QTimer
        self._move_timer = QTimer(self)
        self._move_timer.setInterval(100) # 10Hz
        self._move_timer.timeout.connect(self._on_move_timer)
        
        for btn, dir_str in [
            (self.btn_forward, "forward"),
            (self.btn_backward, "backward"),
            (self.btn_left, "left"),
            (self.btn_right, "right"),
            (self.btn_up, "up"),
            (self.btn_down, "down"),
            (self.btn_rot_left, "rotate_left"),
            (self.btn_rot_right, "rotate_right")
        ]:
            btn.pressed.connect(lambda d=dir_str: self._start_move(d))
            btn.released.connect(self._stop_move)
            
        self.btn_hover.clicked.connect(lambda: self.move_cmd("hover"))
        self.btn_stop.clicked.connect(lambda: self.move_cmd("stop"))
        
        # Flight
        self.btn_arm.clicked.connect(lambda: self.flight_cmd("arm"))
        self.btn_takeoff.clicked.connect(lambda: self.flight_cmd("takeoff"))
        self.btn_land.clicked.connect(lambda: self.flight_cmd("land"))
        self.btn_rtl.clicked.connect(lambda: self.flight_cmd("rtl"))
        self.btn_emergency.clicked.connect(lambda: self.flight_cmd("emergency"))
        
        # Mission
        self.btn_upload.clicked.connect(lambda: self.mission_cmd("upload"))
        self.btn_start.clicked.connect(lambda: self.mission_cmd("start"))
        self.btn_pause.clicked.connect(lambda: self.mission_cmd("pause"))
        self.btn_resume.clicked.connect(lambda: self.mission_cmd("resume"))
        self.btn_cancel.clicked.connect(lambda: self.mission_cmd("cancel"))
        
    def _start_move(self, direction):
        self._current_direction = direction
        self.move_cmd(direction)
        self._move_timer.start()
        
    def _stop_move(self):
        self._move_timer.stop()
        self._current_direction = None
        self.move_cmd("stop")
        
    def _on_move_timer(self):
        if hasattr(self, '_current_direction') and self._current_direction:
            self.move_cmd(self._current_direction)

    def on_drone_selected(self, id):
        self.agent.target_id = id
        if hasattr(self.agent, 'logger'):
            self.agent.logger.info(f"Swarm target updated to: {id}")
            
    def move_cmd(self, direction):
        if not hasattr(self.agent, 'backend') or not self.agent.backend:
            return
            
        speed = self.agent.config.get('max_speed', 5.0) if self.agent.config else 5.0
        vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0
        
        if direction == "forward": vx = speed
        elif direction == "backward": vx = -speed
        elif direction == "right": vy = speed
        elif direction == "left": vy = -speed
        elif direction == "up": vz = -speed
        elif direction == "down": vz = speed
        elif direction == "rotate_left": yaw = -45.0
        elif direction == "rotate_right": yaw = 45.0
        elif direction == "hover":
            self.agent.send_command_and_wait('hover', target_id=self.agent.target_id)
            return
        elif direction == "stop":
            pass # zero velocity
            
        # Broadcast continuous velocity without waiting for ACK to avoid blocking GCS
        if hasattr(self.agent, 'communication') and self.agent.communication:
            self.agent.communication.broadcast_command('velocity_ned', 
                {'vx': vx, 'vy': vy, 'vz': vz, 'yaw': yaw}, 
                target_id=self.agent.target_id)
            
    def apply_formation(self, shape):
        self.agent.config['formation'] = shape
        self.agent.send_command_and_wait('formation_change', shape, target_id=self.agent.target_id)
        
    def flight_cmd(self, cmd):
        if cmd == 'takeoff':
            self.agent.send_command_and_wait('takeoff', {'alt': 10.0}, target_id=self.agent.target_id)
        elif cmd == 'emergency':
            self.agent.send_command_and_wait('emergency', target_id=self.agent.target_id)
        else:
            self.agent.send_command_and_wait(cmd, target_id=self.agent.target_id)
            
    def mission_cmd(self, cmd):
        # We assume mission panel used 'mission_upload', 'mission_start' etc.
        self.agent.send_command_and_wait(f"mission_{cmd}", target_id=self.agent.target_id)
