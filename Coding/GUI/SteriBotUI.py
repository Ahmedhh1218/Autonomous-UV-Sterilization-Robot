import sys
import sqlite3
import requests
import threading
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

pressed_key = ""

class DatabaseManager:
    def __init__(self, database_name):
        self.connection = sqlite3.connect(database_name)
        self.cursor = self.connection.cursor()

    def get_user(self, username):
        query = "SELECT * FROM users WHERE username = ?"
        self.cursor.execute(query, (username,))
        user_data = self.cursor.fetchone()
        return user_data if user_data else None

class SplashScreen(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SteriBot Splash Screen")
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setGeometry(0, 0, QApplication.desktop().screenGeometry().width(), QApplication.desktop().screenGeometry().height())
        
        # Set background image
        bg_image = QLabel(self)
        bg_pixmap = QPixmap("Utils/3D-Hospital-Environment.png")
        screen_size = QApplication.desktop().screenGeometry().size()
        scaled_bg_pixmap = bg_pixmap.scaled(screen_size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
        bg_image.setPixmap(scaled_bg_pixmap)
        bg_image.setGeometry(0, 0, screen_size.width(), screen_size.height())
        
        # Dark overlay
        dark_overlay = QWidget(self)
        dark_overlay.setStyleSheet("background-color: rgba(0, 0, 0, 150);")
        dark_overlay.setGeometry(0, 0, screen_size.width(), screen_size.height())
        
        # Layout for splash screen content
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        
        # Logo
        logo_label = QLabel()
        logo_pixmap = QPixmap("Utils/logo2.jpeg")
        logo_pixmap = logo_pixmap.scaled(400, 400, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # Circle the logo
        circled_pixmap = QPixmap(logo_pixmap.size())
        circled_pixmap.fill(Qt.transparent)
        painter = QPainter(circled_pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        path = QPainterPath()
        path.addEllipse(0, 0, logo_pixmap.width(), logo_pixmap.height())
        painter.setClipPath(path)
        painter.drawPixmap(0, 0, logo_pixmap)
        painter.end()
        
        logo_label.setPixmap(circled_pixmap)
        logo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)
        
        # Slogan
        slogan_label = QLabel("YOUR GUARDIAN OF PURITY IN THE BATTLE OF GERMS")
        slogan_label.setStyleSheet("color: white; font-size: 22px;")
        slogan_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(slogan_label)
        
        # Get Started button
        get_started_button = QPushButton("Get Started")
        get_started_button.setStyleSheet("font-size: 24px; padding: 15px 30px;")
        get_started_button.clicked.connect(self.show_login_screen)
        layout.addWidget(get_started_button)
        
        # Center the content
        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        central_widget.setGeometry(0, 0, screen_size.width(), screen_size.height())
        
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(central_widget)
        main_layout.setAlignment(Qt.AlignCenter)
        self.setLayout(main_layout)
        
    def show_login_screen(self):
        self.accept()

class LoginScreen(QDialog):
    def __init__(self, db_manager):
        super().__init__()
        self.setWindowTitle("Login")
        self.db_manager = db_manager
        self.setWindowFlags(Qt.FramelessWindowHint)  # Remove window frame
        self.setGeometry(0, 0, QApplication.desktop().screenGeometry().width(), QApplication.desktop().screenGeometry().height())  # Set window size to fullscreen

        # Set background image
        bg_image = QLabel(self)
        pixmap = QPixmap("Utils/logo.jpeg")
        screen_size = QApplication.desktop().screenGeometry().size()
        scaled_pixmap = pixmap.scaled(screen_size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)

        # Create a semi-transparent pixmap
        transparent_pixmap = QPixmap(scaled_pixmap.size())
        transparent_pixmap.fill(Qt.transparent)
        painter = QPainter(transparent_pixmap)
        painter.setOpacity(0.3)  # Set transparency level for the image
        painter.drawPixmap(0, 0, scaled_pixmap)
        painter.end()

        bg_image.setPixmap(transparent_pixmap)
        bg_image.setGeometry(0, 0, screen_size.width(), screen_size.height())

        # Add dark shade overlay
        dark_overlay = QWidget(self)
        dark_overlay.setStyleSheet("background-color: rgba(0, 0, 0, 150);")  # Semi-transparent black
        dark_overlay.setGeometry(0, 0, screen_size.width(), screen_size.height())

        # Layout for the login form
        form_layout = QVBoxLayout()
        form_layout.setContentsMargins(0, 0, 0, 0)  # Adjust margins as needed
        form_layout.setAlignment(Qt.AlignCenter)  # Center align the widgets

        # Center widget for the form
        form_widget = QWidget()
        form_widget.setLayout(form_layout)
        form_widget.setFixedSize(400, 200)  # Increased fixed size for the form

        self.username_input = QLineEdit()
        self.username_input.setPlaceholderText("Username")
        self.username_input.setFixedHeight(40)  # Increase height of username input
        self.username_input.setStyleSheet("font-size: 18px;")  # Increase font size
        form_layout.addWidget(self.username_input)

        self.password_input = QLineEdit()
        self.password_input.setPlaceholderText("Password")
        self.password_input.setEchoMode(QLineEdit.Password)  # Mask password
        self.password_input.setFixedHeight(40)  # Increase height of password input
        self.password_input.setStyleSheet("font-size: 18px;")  # Increase font size
        form_layout.addWidget(self.password_input)

        login_button = QPushButton("Login")
        login_button.setFixedHeight(40)  # Increase height of login button
        login_button.setStyleSheet("font-size: 18px; padding: 10px 20px;")  # Increase font size and padding
        login_button.clicked.connect(self.check_credentials)
        form_layout.addWidget(login_button)

        # Main layout
        main_layout = QVBoxLayout()
        main_layout.addWidget(form_widget)
        main_layout.setAlignment(Qt.AlignCenter)  # Center align the form widget
        self.setLayout(main_layout)

    def check_credentials(self):
        username = self.username_input.text()
        password = self.password_input.text()

        user = self.db_manager.get_user(username)
        if user and user[2] == password:
            self.accept()
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid credentials")


class ManualControlWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)

        # Camera View Placeholder
        self.camera_view = QLabel()
        self.camera_view.setFixedSize(640, 480)  # Set a fixed size for the camera view
        self.camera_view.setStyleSheet("background-color: black; color: white;")  # Set background color to black
        self.camera_view.setAlignment(Qt.AlignCenter)
        self.camera_view.setText("Camera View")  # Placeholder text
        layout.addWidget(self.camera_view, alignment=Qt.AlignCenter)

        # Instruction Label
        instruction_label = QLabel("Use keyboard arrows for movement control.\nPress 'Q' for clockwise rotation, 'E' for anti-clockwise rotation, and 'S' to stop moving.")
        instruction_label.setAlignment(Qt.AlignCenter)
        
        # Increase font size
        instruction_label.setStyleSheet("font-size: 20px;")  # Set font size to 20px
        layout.addWidget(instruction_label)

        # Alternative Controls Group
        alternative_group = QGroupBox("Alternative Controls")
        alternative_group.setStyleSheet("font-size: 18px;")  # Increase font size of the group box title
        alternative_layout = QVBoxLayout()        

        # Movement Buttons
        movement_layout = QHBoxLayout()
        movement_group = QGroupBox("Movement Control")
        movement_group.setStyleSheet("font-size: 16px;")  # Increase font size of the movement group title
        movement_layout = QHBoxLayout()

        up_button = QPushButton()
        up_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowUp))
        up_button.setStyleSheet("font-size: 14px; padding: 10px;")  # Increase font size and padding
        up_button.clicked.connect(lambda: self.handle_button_click(Qt.Key_Up))
        movement_layout.addWidget(up_button)

        left_button = QPushButton()
        left_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowLeft))
        left_button.setStyleSheet("font-size: 14px; padding: 10px;")  # Increase font size and padding
        left_button.clicked.connect(lambda: self.handle_button_click(Qt.Key_Left))
        movement_layout.addWidget(left_button)

        down_button = QPushButton()
        down_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowDown))
        down_button.setStyleSheet("font-size: 14px; padding: 10px;")  # Increase font size and padding
        down_button.clicked.connect(lambda: self.handle_button_click(Qt.Key_Down))
        movement_layout.addWidget(down_button)

        right_button = QPushButton()
        right_button.setIcon(self.style().standardIcon(QStyle.SP_ArrowRight))
        right_button.setStyleSheet("font-size: 14px; padding: 10px;")  # Increase font size and padding
        right_button.clicked.connect(lambda: self.handle_button_click(Qt.Key_Right))
        movement_layout.addWidget(right_button)

        # Stop Button
        stop_button = QPushButton("STOP")
        stop_button.setStyleSheet("font-size: 14px; padding: 10px;")  # Increase font size and padding
        stop_button.clicked.connect(lambda: self.handle_button_click(Qt.Key_S))
        movement_layout.addWidget(stop_button)

        movement_group.setLayout(movement_layout)
        alternative_layout.addWidget(movement_group)

        # Rotation Buttons
        rotation_layout = QHBoxLayout()
        rotation_group = QGroupBox("Rotation Control")
        rotation_group.setStyleSheet("font-size: 16px;")  # Increase font size of the rotation group title
        rotation_layout = QHBoxLayout()

        cw_button = QPushButton("CW")
        cw_button.setStyleSheet("font-size: 14px; padding: 10px;")  # Increase font size and padding
        cw_button.clicked.connect(lambda: self.handle_button_click(Qt.Key_Q))
        rotation_layout.addWidget(cw_button)

        ccw_button = QPushButton("CCW")
        ccw_button.setStyleSheet("font-size: 14px; padding: 10px;")  # Increase font size and padding
        ccw_button.clicked.connect(lambda: self.handle_button_click(Qt.Key_E))
        rotation_layout.addWidget(ccw_button)

        rotation_group.setLayout(rotation_layout)
        alternative_layout.addWidget(rotation_group)

        alternative_group.setLayout(alternative_layout)
        layout.addWidget(alternative_group)

        self.setLayout(layout)
        self.setFocusPolicy(Qt.StrongFocus)  # Ensure the widget can receive focus

    def handle_button_click(self, key):
        event = QKeyEvent(QEvent.KeyPress, key, Qt.NoModifier)
        self.keyPressEvent(event)

    def keyPressEvent(self, event):
        global pressed_key
        key = event.key()
        
        if key == Qt.Key_Up:
            print("Move forward")
            pressed_key = "Move forward"
        elif key == Qt.Key_Down:
            print("Move backward")
            pressed_key = "Move backward"
        elif key == Qt.Key_Left:
            print("Move left")
            pressed_key = "Move left"
        elif key == Qt.Key_Right:
            print("Move right")
            pressed_key = "Move right"
        elif key == Qt.Key_Q:
            print("Clockwise rotation")
            pressed_key = "Clockwise rotation"
        elif key == Qt.Key_E:
            print("Anti-clockwise rotation")
            pressed_key = "Anti-clockwise rotation"
        elif key == Qt.Key_S:
            print("Stop")
            pressed_key = "Stop"

    def update_camera_view(self, image):
        """Update the camera view with the provided image (as QPixmap)."""
        self.camera_view.setPixmap(image.scaled(self.camera_view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

class PasskeyDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Enter Passkey")
        self.passkey_line_edit = QLineEdit()
        self.passkey_line_edit.setPlaceholderText("Enter passkey")
        self.passkey_line_edit.setEchoMode(QLineEdit.Password)
        self.ok_button = QPushButton("OK")
        self.ok_button.clicked.connect(self.check_passkey)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Enter passkey to proceed:"))
        layout.addWidget(self.passkey_line_edit)
        layout.addWidget(self.ok_button)
        self.setLayout(layout)

    def check_passkey(self):
        passkey = self.passkey_line_edit.text()
        # Preset passkey for demonstration, replace with your actual passkey
        preset_passkey = "0000"
        if passkey == preset_passkey:
            self.accept()
        else:
            QMessageBox.warning(self, "Invalid Passkey", "Incorrect passkey. Please try again.")

class FlaskListener(QObject):
    roomStatusUpdated = pyqtSignal(str, str)  # Signal to update room status

    def __init__(self, ip_address):
        super().__init__()
        self.ip_address = ip_address
        self.running = True

    def start_listening(self):
        while self.running:
            try:
                response = requests.get(f'http://{self.ip_address}:5000/status')
                if response.status_code == 200:
                    data = response.json()
                    room_number = data['room_number']
                    status = data['status']
                    self.roomStatusUpdated.emit(room_number, status)
                time.sleep(1)  # Polling interval, adjust as necessary
            except requests.exceptions.RequestException as e:
                print("Error:", e)
                time.sleep(5)  # Wait before retrying on error

    def stop(self):
        self.running = False

class SteriBotGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Database Initialization
        self.db_manager = DatabaseManager("Utils/SteriBot-DB.db")

        # Splash Screen
        splash = SplashScreen()
        if splash.exec_() != QDialog.Accepted:
            sys.exit()  # Quit if splash screen is closed

        # Login Screen
        login = LoginScreen(self.db_manager)
        if login.exec_() != QDialog.Accepted:
            sys.exit()  # Quit if login fails

        # Get the logged-in user
        self.logged_in_user = login.username_input.text()

        self.setWindowTitle("SteriBot Dashboard")
        self.setGeometry(0, 0, QApplication.desktop().screenGeometry().width(), QApplication.desktop().screenGeometry().height())  # Set window size to fullscreen

        # Set window icon
        icon = QIcon("Utils/logo2.jpeg")
        self.setWindowIcon(icon)

        # Maximize window
        self.showMaximized()

        # Central Widget and Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Add user dropdown menu
        self.user_menu = QMenu()
        self.logout_action = QAction("Logout", self)
        self.logout_action.triggered.connect(self.logout)
        self.user_menu.addAction(self.logout_action)  # Add the action to the menu
        user_button = QPushButton(self.logged_in_user)
        user_button.setMenu(self.user_menu)  # Attach the menu to the button
        user_button.setFixedSize(150, 40)
        main_layout.addWidget(user_button, alignment=Qt.AlignRight)

        # Tab Widget
        tabs = QTabWidget()
        self.apply_tab_stylesheet(tabs)  # Apply custom styles to the tabs
        main_layout.addWidget(tabs)

        self.ip_address = ""  # Add this line to store the IP address

        # Tab 1: Automated Sterilization
        self.setup_sterilization_tab(tabs)

        # Tab 2: Coordinates Control
        self.setup_coordinates_tab(tabs)

        # Tab 3: Robot Configurations
        self.setup_configurations_tab(tabs)

        # Flask Listener
        self.flask_listener = FlaskListener(self.ip_address)
        self.flask_listener_thread = threading.Thread(target=self.flask_listener.start_listening)
        self.flask_listener.roomStatusUpdated.connect(self.update_room_status)
        self.flask_listener_thread.start()

    def closeEvent(self, event):
        self.flask_listener.stop()
        self.flask_listener_thread.join()
        event.accept()

    def update_room_status(self, room_number, status):
        for i in range(self.room_indicators_widget.layout().count()):
            widget = self.room_indicators_widget.layout().itemAt(i).widget()
            if widget.text() == f"Room {room_number}":
                if status == "sterilized":
                    widget.setStyleSheet("background-color: green; border-radius: 10px; padding: 5px; color: white;")
                elif status == "not sterilized":
                    widget.setStyleSheet("background-color: gray; border-radius: 10px; padding: 5px; color: white;")
                break
            
    def logout(self):
        # Close the current window
        self.close()

    def apply_tab_stylesheet(self, tabs):
        tabs.setStyleSheet("""
            QTabBar::tab {
                font-size: 14px;
                padding: 15px 30px;
                min-width: 150px;
            }
            QTabBar::tab:selected {
                font-weight: bold;
                background: #ccc;
            }
            QTabBar::tab:hover {
                background: #ddd;
            }
        """)

    def setup_coordinates_tab(self, tabs):
        tab = ManualControlWidget()
        tabs.addTab(tab, "Manual Control")

    def setup_sterilization_tab(self, tabs):
        tab = QWidget()
        tabs.addTab(tab, "Sterilization")
        layout = QVBoxLayout()
        tab.setLayout(layout)

        # Add image above "Enter Room Numbers" label
        image_label = QLabel()
        pixmap = QPixmap("Utils/hospital-layout.png")
        pixmap = pixmap.scaledToWidth(800)  # Resize the image width to 200 pixels
        image_label.setPixmap(pixmap)
        image_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(image_label)

        room_label = QLabel("Enter Room Numbers (comma-separated):")
        room_label.setFixedHeight(30)  # Set a fixed height for the label
        room_label.setStyleSheet("font-size: 16px;")  # Increase font size
        layout.addWidget(room_label)

        self.room_input = QLineEdit()
        self.room_input.setStyleSheet("font-size: 16px;")  # Increase font size of text inside the text box
        layout.addWidget(self.room_input)
        layout.setSpacing(20)  # Increase the vertical spacing between widgets

        sterilize_button = QPushButton("Start Sterilization")
        sterilize_button.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px 20px; font-size: 18px; border: none; border-radius: 5px;")
        sterilize_button.setIcon(QIcon("Utils/logo3.jpeg"))  # Replace with actual icon path
        sterilize_button.setIconSize(QSize(24, 24))
        sterilize_button.clicked.connect(self.start_sterilization)
        layout.addWidget(sterilize_button)

        # Widget to hold room indicators
        self.room_indicators_widget = QWidget()
        room_indicators_layout = QVBoxLayout()
        self.room_indicators_widget.setLayout(room_indicators_layout)
        layout.addWidget(self.room_indicators_widget)


    def start_sterilization(self):
        global pressed_key
        room_numbers = self.room_input.text().split(',')

        # Print entered room numbers
        print("Entered room numbers:", room_numbers)
        print("Pressed keys:", pressed_key)

        # Send room numbers to Raspberry Pi server
        try:
            response = requests.post(f'http://{self.ip_address}:5000/data', json={'room_numbers': room_numbers})
            keys_response = requests.post(f'http://{self.ip_address}:5000/pressed_key', json={'pressed_keys': pressed_key})
            if response.status_code == 200:
                print("Room numbers and pressed keys sent successfully.")
            else:
                print("Failed to send room numbers and pressed keys.")
            if keys_response.status_code == 200:
                print("Pressed keys sent successfully.")
            else:
                print("Failed to send pressed keys.")
        except requests.exceptions.RequestException as e:
            print("Error:", e)
        
        # Clear previous room indicators
        for i in reversed(range(self.room_indicators_widget.layout().count())):
            self.room_indicators_widget.layout().itemAt(i).widget().setParent(None)

        # Add room indicators for each room number
        for room_number in room_numbers:
            room_label = QLabel(f"Room {room_number}")
            room_label.setStyleSheet("background-color: gray; border-radius: 10px; padding: 5px; color: white;")
            room_label.setAlignment(Qt.AlignCenter)
            self.room_indicators_widget.layout().addWidget(room_label)
    
    
    def setup_configurations_tab(self, tabs):
        tab = QWidget()
        tabs.addTab(tab, "Robot Configurations")
        layout = QVBoxLayout()
        tab.setLayout(layout)

        # Velocity Acceptable Range Message
        velocity_range_label = QLabel("Velocity Acceptable Range: 40-170 RPM")
        velocity_range_label.setStyleSheet("color: red; font-size: 18px;")  # Set the color of the text to red
        layout.addWidget(velocity_range_label)


        # Grid layout for velocity settings
        grid_layout = QGridLayout()
        layout.addLayout(grid_layout)

        # Velocity settings
        labels = ["Forward Motion Velocity (RPM):", "Backward Motion Velocity (RPM):",
                "Right Motion Velocity (RPM):", "Left Motion Velocity (RPM):",
                "Clockwise Rotation Velocity (RPM):", "Counterclockwise Rotation Velocity (RPM):"]
        spinboxes = [QSpinBox() for _ in labels]

        for i, (label, spinbox) in enumerate(zip(labels, spinboxes)):
            spinbox.setMinimum(40)
            spinbox.setMaximum(170)
            spinbox.setSingleStep(5)
            spinbox.valueChanged.connect(self.show_passkey_dialog)
            grid_layout.addWidget(QLabel(label), i // 3, (i % 3) * 2)
            grid_layout.addWidget(spinbox, i // 3, (i % 3) * 2 + 1)

        self.forward_spinbox, self.backward_spinbox, self.right_spinbox, \
            self.left_spinbox, self.clockwise_spinbox, self.counterclockwise_spinbox = spinboxes
                
        # PID Parameters
        pid_label = QLabel("PID Parameters for Wheel Velocity Control:")
        pid_label.setStyleSheet("font-size: 16px;")
        layout.addWidget(pid_label)

        pid_grid_layout = QGridLayout()
        layout.addLayout(pid_grid_layout)

        pid_labels = ["Proportional Gain (P):", "Integral Gain (I):", "Derivative Gain (D):"]
        pid_spinboxes = [QDoubleSpinBox() for _ in pid_labels]

        # Set initial values
        initial_values = [1.0, 5.0, 0.05]

        for i, (label, spinbox, value) in enumerate(zip(pid_labels, pid_spinboxes, initial_values)):
            spinbox.setMinimum(0.0)
            spinbox.setMaximum(100.0)
            spinbox.setSingleStep(0.1)
            spinbox.setValue(value)  # Set initial value
            spinbox.valueChanged.connect(self.show_passkey_dialog)
            pid_grid_layout.addWidget(QLabel(label), i, 0)
            pid_grid_layout.addWidget(spinbox, i, 1)

        self.proportional_gain_spinbox, self.integral_gain_spinbox, self.derivative_gain_spinbox = pid_spinboxes

        # UV Light Acceptable Range Message
        uv_range_label = QLabel("UV Light Intensity Acceptable Range: 40-100%")
        uv_range_label.setStyleSheet("color: red; font-size: 18px;")  # Set the color of the text to red
        layout.addWidget(uv_range_label)

        # UV Light Intensity
        uv_label = QLabel("UV Light Intensity:")
        layout.addWidget(uv_label)

        uv_layout = QHBoxLayout()

        self.uv_slider = QSlider(Qt.Horizontal)
        self.uv_slider.setMinimum(40)
        self.uv_slider.setMaximum(100)
        self.uv_slider.setTickInterval(10)
        self.uv_slider.setTickPosition(QSlider.TicksBelow)
        self.uv_slider.valueChanged.connect(self.update_uv_intensity)  # Connect slider valueChanged signal
        uv_layout.addWidget(self.uv_slider)

        self.uv_label = QLabel("40%")
        uv_layout.addWidget(self.uv_label)

        layout.addLayout(uv_layout)

        # Connect valueChanged signal to show_passkey_dialog
        self.uv_slider.valueChanged.connect(self.show_passkey_dialog)

        # Raspberry Pi IP Address
        ip_label = QLabel("Raspberry Pi IP Address:")
        layout.addWidget(ip_label)

        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("Enter Raspberry Pi IP Address")
        self.ip_input.setStyleSheet("font-size: 16px;")  # Increase font size
        layout.addWidget(self.ip_input)

        # Update the IP address when the text changes
        self.ip_input.textChanged.connect(self.update_ip_address)

    def update_ip_address(self):
        self.ip_address = self.ip_input.text()


    def update_uv_intensity(self, value):
        self.uv_label.setText(f"{value}%")

    def show_passkey_dialog(self):
        passkey_dialog = PasskeyDialog(self)
        if passkey_dialog.exec_() == QDialog.Accepted:
            # If the passkey is correct, allow the user to proceed
            pass


# Main execution
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SteriBotGUI()
    window.show()
    sys.exit(app.exec_())
