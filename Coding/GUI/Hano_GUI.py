import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLineEdit, QPushButton, QLabel, QMessageBox
from PyQt5.QtGui import QPixmap, QPainter, QColor
from PyQt5.QtCore import Qt

class HomeScreen(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Home Screen')
        self.setFixedSize(800, 800)  # Set window size to 1920x1080

        # Adding background image
        background_label = QLabel(self)
        background_label.setPixmap(QPixmap('logo.jpeg').scaled(self.size()))  # Scale image to window size
        background_label.setGeometry(0, 0, self.width(), self.height())

        # Adding username and password fields
        self.username_input = QLineEdit(self)
        self.username_input.setPlaceholderText("Enter username")
        self.username_input.setGeometry(300, 300, 200, 30)
        self.password_input = QLineEdit(self)
        self.password_input.setPlaceholderText("Enter password")
        self.password_input.setGeometry(300, 350, 200, 30)
        self.password_input.setEchoMode(QLineEdit.Password)

        # Adding buttons
        login_button = QPushButton('Login', self)
        login_button.setGeometry(350, 400, 100, 30)
        login_button.clicked.connect(self.checkCredentials)
        forget_password_button = QPushButton('Forget Password', self)
        forget_password_button.setGeometry(325, 450, 150, 30)
        forget_password_button.clicked.connect(self.forgetPassword)

    def checkCredentials(self):
        username = self.username_input.text()
        password = self.password_input.text()
        if username == "hi" and password == "hi":
            self.openNumberInputScreen(self.width())
        else:
            QMessageBox.warning(self, 'Invalid Credentials', 'Invalid username or password. Please try again.')

    def openNumberInputScreen(self, width):
        self.number_input_screen = NumberInputScreen(width)
        self.number_input_screen.show()  # Show the NumberInputScreen
        self.close()

    def forgetPassword(self):
        # Placeholder for forget password logic
        print("Forget Password button clicked")


class NumberInputScreen(QWidget):
    def __init__(self, width):
        super().__init__()
        self.width = width
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Number Input Screen')
        self.setFixedSize(self.width, 800)  # Set window size with the same width as HomeScreen

        # Adding background image
        background_label = QLabel(self)
        background_label.setPixmap(QPixmap('logo.jpeg').scaled(self.size()))  # Set background image
        background_label.setGeometry(0, 0, self.width, 800)

        self.number_input = QLineEdit(self)
        self.number_input.setPlaceholderText("Enter room numbers")
        self.number_input.setGeometry(300, 300, 200, 30)
        self.submit_button = QPushButton('Submit', self)
        self.submit_button.setGeometry(300, 350, 100, 30)

        self.submit_button.clicked.connect(self.processInput)

    def processInput(self):
        numbers_text = self.number_input.text()
        numbers = [num.strip() for num in numbers_text.split(',')]
        for idx, num in enumerate(numbers):
            try:
                number = float(num)
                # Print to the terminal
                print(f"Number {idx + 1}: {number}")
                 # Close the current screen and open the room states screen
                self.close()
                self.openRoomStatesScreen(len(numbers) + 1)

            except ValueError:
                QMessageBox.warning(self, 'Invalid Number', 'Please enter valid numbers.')
                print(f"Invalid input '{num}'. Please enter valid numbers.")
        
       
    def openRoomStatesScreen(self, num_circles):
        self.room_states_screen = RoomStatesScreen(self.width, num_circles)
        self.room_states_screen.show()


class RoomStatesScreen(QWidget):
    def __init__(self, width, num_circles):
        super().__init__()
        self.width = width
        self.num_circles = num_circles
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Room States Screen')
        self.setFixedSize(self.width, 800)  # Set window size with the same width as HomeScreen

    def paintEvent(self, event):
        painter = QPainter(self)
        
        # Draw background image
        background = QPixmap('logo.jpeg')
        painter.drawPixmap(0, 0, self.width, 800, background)

        # Displaying circles
        circle_radius = 30
        circle_spacing = 50
        start_x = (self.width - (self.num_circles * (2 * circle_radius + circle_spacing))) / 2
        start_y = (self.width - (self.num_circles * (2 * circle_radius + circle_spacing))) / 2
        for i in range(self.num_circles):
            circle_x = start_x + i * (2 * circle_radius + circle_spacing)
            circle_y = start_y
            circle_color = QColor(Qt.red) if i % 2 == 0 else QColor(Qt.yellow)
            painter.setBrush(circle_color)
            painter.drawEllipse(circle_x, circle_y, circle_radius * 2, circle_radius * 2)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    home_screen = HomeScreen()
    home_screen.show()
    sys.exit(app.exec_())
