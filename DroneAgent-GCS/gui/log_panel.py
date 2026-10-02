from PySide6.QtWidgets import QWidget, QVBoxLayout, QTabWidget, QTextEdit, QGroupBox
from PySide6.QtCore import Qt

class LogPanel(QWidget):
    def __init__(self, agent, parent=None):
        super().__init__(parent)
        self.agent = agent
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        group = QGroupBox("System Logs")
        vbox = QVBoxLayout()
        
        self.tabs = QTabWidget()
        self.text_edits = {}
        
        categories = ["All", "Warnings", "Errors", "Communication", "Telemetry", "Mission", "Formation", "Decision Engine"]
        
        for cat in categories:
            edit = QTextEdit()
            edit.setReadOnly(True)
            edit.setStyleSheet("font-family: monospace; font-size: 10pt;")
            self.tabs.addTab(edit, cat)
            self.text_edits[cat] = edit
            
        vbox.addWidget(self.tabs)
        group.setLayout(vbox)
        layout.addWidget(group)
        
    def append_log(self, category, message):
        """Append a log message to the specified category and the 'All' tab."""
        if category in self.text_edits:
            self.text_edits[category].append(message)
        self.text_edits["All"].append(f"[{category}] {message}")
        
    # In a real implementation, you would attach a custom logging handler to the python logger
    # that emits Qt signals to trigger `append_log`.
