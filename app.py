import sys
from PyQt6.QtWidgets import QApplication, QWidget

class GroundStationCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("Axiom Ground Station")
        self.resize(800, 500) 
        self.setStyleSheet("background-color: #121214;")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GroundStationCanvas()
    window.show()
    sys.exit(app.exec())
