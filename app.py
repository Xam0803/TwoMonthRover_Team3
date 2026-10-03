import sys
from PyQt6.QtWidgets import QApplication, QWidget

# 1. blank window
class GroundStationCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        
    def init_ui(self):
        # 2. window's basic traits
        self.setWindowTitle("Axiom Ground Station")
        self.resize(800, 500) # Width, Height in pixels
        
        #  clean dark background color
        self.setStyleSheet("background-color: #121214;")

# 3. Initialize and run the application
if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Create an instance of blank window class
    window = GroundStationCanvas()
    window.show()
    sys.exit(app.exec())
