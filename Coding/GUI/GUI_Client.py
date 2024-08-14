import sys
import requests
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit

class App(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.layout = QVBoxLayout()

        self.input = QLineEdit(self)
        self.layout.addWidget(self.input)

        self.button = QPushButton('Send Data', self)
        self.button.clicked.connect(self.sendData)
        self.layout.addWidget(self.button)

        self.setLayout(self.layout)
        self.setWindowTitle('Client')
        self.show()

    def sendData(self):
        data = {'message': self.input.text()}
        try:
            response = requests.post('http://192.168.100.49:5000/data', json=data)
            print(response.json())
        except requests.exceptions.RequestException as e:
            print("Error:", e)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
