import logging
from PySide6.QtWidgets import QWidget, QHBoxLayout, QPushButton
from PySide6.QtCore import Qt

class Toolbar(QWidget):
    def __init__(self, agent, parent=None):
        super().__init__(parent)
        self.agent = agent
        self.logger = logging.getLogger("Toolbar")
        self.init_ui()
        
    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setAlignment(Qt.AlignLeft)
        
        # Connection Controls
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_sim = QPushButton("Simulation")
        self.btn_real = QPushButton("Real Drone")
        
        # Flight Controls
        self.btn_arm = QPushButton("Arm")
        self.btn_arm.setObjectName("btn_arm")
        self.btn_disarm = QPushButton("Disarm")
        
        self.btn_takeoff = QPushButton("Takeoff")
        self.btn_takeoff.setObjectName("btn_takeoff")
        self.btn_land = QPushButton("Land")
        self.btn_rtl = QPushButton("RTL")
        
        self.btn_pause = QPushButton("Pause")
        self.btn_resume = QPushButton("Resume")
        
        # Emergency Controls
        self.btn_emergency = QPushButton("Emergency")
        self.btn_emergency.setObjectName("btn_emergency")
        self.btn_kill = QPushButton("Kill")
        self.btn_kill.setObjectName("btn_kill")
        self.btn_shutdown = QPushButton("Shutdown")
        
        # Add to layout
        for btn in [
            self.btn_connect, self.btn_disconnect, self.btn_sim, self.btn_real,
            self.btn_arm, self.btn_disarm, self.btn_takeoff, self.btn_land, self.btn_rtl,
            self.btn_pause, self.btn_resume, self.btn_emergency, self.btn_kill, self.btn_shutdown
        ]:
            layout.addWidget(btn)
            
        # Connections
        self.btn_connect.clicked.connect(self.on_connect)
        self.btn_disconnect.clicked.connect(self.on_disconnect)
        self.btn_sim.clicked.connect(self.on_sim)
        self.btn_real.clicked.connect(self.on_real)
        
        self.btn_arm.clicked.connect(self.on_arm)
        self.btn_disarm.clicked.connect(self.on_disarm)
        self.btn_takeoff.clicked.connect(self.on_takeoff)
        self.btn_land.clicked.connect(self.on_land)
        self.btn_rtl.clicked.connect(self.on_rtl)
        
        self.btn_pause.clicked.connect(self.on_pause)
        self.btn_resume.clicked.connect(self.on_resume)
        
        self.btn_emergency.clicked.connect(self.on_emergency)
        self.btn_kill.clicked.connect(self.on_kill)
        self.btn_shutdown.clicked.connect(self.on_shutdown)


    def _handle_future(self, cmd_name, future):
        if future is None:
            return
        self.logger.info(f"Command Sent: {cmd_name}")
        def callback(fut):
            try:
                fut.result()
                self.logger.info(f"Command Success: {cmd_name}")
            except Exception as e:
                self.logger.error(f"Command Failed: {cmd_name} - {e}")
        future.add_done_callback(callback)

    def on_connect(self):
        # Implementation depends on how agent handles connection
        if hasattr(self.agent, 'backend') and hasattr(self.agent.backend, 'connect'):
            self.agent.backend.connect()
            
    def on_disconnect(self):
        if hasattr(self.agent, 'backend') and hasattr(self.agent.backend, 'stop'):
            self.agent.backend.stop()

    def on_sim(self):
        self.logger.warning("Mode switching must be done via configuration YAML before startup.")

    def on_real(self):
        self.logger.warning("Mode switching must be done via configuration YAML before startup.")

    def on_arm(self):
        self.logger.info("Button Clicked: ARM")
        if hasattr(self.agent, 'backend') and self.agent.backend:
            fut = self.agent.backend.arm()
            self._handle_future('ARM', fut)

    def on_disarm(self):
        self.logger.info(f"Button Clicked: DISARM")
        if hasattr(self.agent, 'backend') and self.agent.backend:
            fut = self.agent.backend.disarm()
            self._handle_future('DISARM', fut)

    def on_takeoff(self):
        self.logger.info("Button Clicked: TAKEOFF")
        if hasattr(self.agent, 'backend') and self.agent.backend:
            alt = self.agent.config.get('mission', {}).get('takeoff_altitude', 10.0)
            fut = self.agent.backend.takeoff(alt)
            self._handle_future('TAKEOFF', fut)

    def on_land(self):
        self.logger.info(f"Button Clicked: LAND")
        if hasattr(self.agent, 'backend') and self.agent.backend:
            fut = self.agent.backend.land()
            self._handle_future('LAND', fut)

    def on_rtl(self):
        self.logger.info(f"Button Clicked: RTL")
        if hasattr(self.agent, 'backend') and self.agent.backend:
            fut = self.agent.backend.return_to_launch()
            self._handle_future('RTL', fut)

    def on_pause(self):
        self.logger.info("Button Clicked: PAUSE")
        if hasattr(self.agent, 'backend') and self.agent.backend:
            fut = self.agent.backend.pause()
            self._handle_future('PAUSE', fut)

    def on_resume(self):
        self.logger.info("Button Clicked: RESUME")
        if hasattr(self.agent, 'backend') and self.agent.backend:
            fut = self.agent.backend.resume()
            self._handle_future('RESUME', fut)

    def on_emergency(self):
        self.logger.info("Button Clicked: EMERGENCY")
        if hasattr(self.agent, 'state_machine'):
            self.agent.state_machine.transition('emergency')

    def on_kill(self):
        self.logger.info(f"Button Clicked: KILL")
        if hasattr(self.agent, 'backend') and self.agent.backend:
            fut = self.agent.backend.emergency_stop()
            self._handle_future('KILL', fut)

    def on_shutdown(self):
        if hasattr(self.agent, 'shutdown'):
            self.agent.shutdown()
