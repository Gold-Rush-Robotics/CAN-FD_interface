#!/usr/bin/env python3
import sys
import can
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                            QHBoxLayout, QListWidget, QGroupBox, QLabel,
                            QPushButton, QSpinBox, QDoubleSpinBox, QTabWidget,
                            QSlider)
from PyQt6.QtCore import Qt, QTimer
import struct

class TeensyManager(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teensy CAN Manager")
        self.setMinimumSize(800, 600)
        
        # CAN Bus setup
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        except Exception as e:
            print(f"Failed to open CAN bus: {e}")
            sys.exit(1)

        # Store discovered Teensy devices
        self.teensy_devices = {}  # {id: last_heartbeat_time}
        self.selected_teensy = None
        
        self.setup_ui()
        
        # Start heartbeat monitoring
        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.timeout.connect(self.check_heartbeats)
        self.heartbeat_timer.start(100)  # Check every 100ms
        
        # Start CAN message receiving
        self.can_timer = QTimer()
        self.can_timer.timeout.connect(self.receive_can_messages)
        self.can_timer.start(10)  # Check every 10ms

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)

        # Left panel - Teensy devices list
        devices_group = QGroupBox("Available Teensy Devices")
        devices_layout = QVBoxLayout()
        self.device_list = QListWidget()
        self.device_list.itemClicked.connect(self.select_teensy)
        devices_layout.addWidget(self.device_list)
        devices_group.setLayout(devices_layout)
        devices_group.setMaximumWidth(200)
        layout.addWidget(devices_group)

        # Right panel - Control tabs
        control_tabs = QTabWidget()
        
        # Motor Control Tab
        motor_tab = QWidget()
        motor_layout = QVBoxLayout()
        
        # Motor Speed Control
        speed_group = QGroupBox("Motor Speed Control")
        speed_layout = QVBoxLayout()
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setMinimum(-100)
        self.speed_slider.setMaximum(100)
        self.speed_slider.valueChanged.connect(self.send_motor_command)
        
        self.speed_label = QLabel("Speed: 0%")
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        speed_group.setLayout(speed_layout)
        motor_layout.addWidget(speed_group)

        # PID Control
        pid_group = QGroupBox("PID Control")
        pid_layout = QVBoxLayout()
        
        # PID parameters
        self.pid_params = {}
        for param in ['P', 'I', 'D']:
            param_layout = QHBoxLayout()
            param_layout.addWidget(QLabel(f"{param}:"))
            spin = QDoubleSpinBox()
            spin.setRange(0, 100)
            spin.setSingleStep(0.1)
            self.pid_params[param] = spin
            param_layout.addWidget(spin)
            pid_layout.addLayout(param_layout)

        self.update_pid_button = QPushButton("Update PID")
        self.update_pid_button.clicked.connect(self.send_pid_parameters)
        pid_layout.addWidget(self.update_pid_button)
        
        pid_group.setLayout(pid_layout)
        motor_layout.addWidget(pid_group)
        motor_tab.setLayout(motor_layout)

        # Servo Control Tab
        servo_tab = QWidget()
        servo_layout = QVBoxLayout()
        
        # Servo position control
        self.servo_slider = QSlider(Qt.Orientation.Horizontal)
        self.servo_slider.setMinimum(0)
        self.servo_slider.setMaximum(180)
        self.servo_slider.valueChanged.connect(self.send_servo_command)
        
        self.servo_label = QLabel("Position: 90°")
        servo_layout.addWidget(self.servo_label)
        servo_layout.addWidget(self.servo_slider)
        
        servo_tab.setLayout(servo_layout)

        # Add tabs to control panel
        control_tabs.addTab(motor_tab, "Motor Control")
        control_tabs.addTab(servo_tab, "Servo Control")
        
        layout.addWidget(control_tabs)

    def check_heartbeats(self):
        """Monitor CAN bus for Teensy heartbeat messages"""
        msg = self.bus.recv(timeout=0)
        if msg and msg.arbitration_id == 0x001:  # Heartbeat message ID
            teensy_id = struct.unpack('I', msg.data[:4])[0]
            self.teensy_devices[teensy_id] = msg.timestamp
            
            # Update device list if needed
            device_items = [self.device_list.item(i).text() for i in range(self.device_list.count())]
            device_text = f"Teensy #{teensy_id}"
            if device_text not in device_items:
                self.device_list.addItem(device_text)

    def select_teensy(self, item):
        """Handle Teensy selection from list"""
        teensy_id = int(item.text().split('#')[1])
        self.selected_teensy = teensy_id

    def receive_can_messages(self):
        """Receive and decrypt CAN messages"""
        if not self.selected_teensy:
            return

        msg = self.bus.recv(timeout=0)
        if msg:
            # Check if message is from selected Teensy
            if (msg.arbitration_id >> 8) == self.selected_teensy:
                self.process_can_message(msg)

    def process_can_message(self, msg):
        """Process received CAN message based on its type"""
        message_type = msg.arbitration_id & 0xFF
        
        if message_type == 0x10:  # Motor status
            speed, current = struct.unpack('ff', msg.data)
            # Update motor status display
            pass
        elif message_type == 0x11:  # PID status
            p, i, d = struct.unpack('fff', msg.data)
            # Update PID display
            pass
        elif message_type == 0x12:  # Servo status
            position = struct.unpack('f', msg.data[:4])[0]
            # Update servo display
            pass

    def send_motor_command(self):
        """Send motor speed command"""
        if not self.selected_teensy:
            return
            
        speed = self.speed_slider.value()
        self.speed_label.setText(f"Speed: {speed}%")
        
        msg = can.Message(
            arbitration_id=(self.selected_teensy << 8) | 0x20,
            data=struct.pack('f', speed/100.0),
            is_extended_id=True
        )
        self.bus.send(msg)

    def send_pid_parameters(self):
        """Send PID parameters"""
        if not self.selected_teensy:
            return
            
        p = self.pid_params['P'].value()
        i = self.pid_params['I'].value()
        d = self.pid_params['D'].value()
        
        msg = can.Message(
            arbitration_id=(self.selected_teensy << 8) | 0x21,
            data=struct.pack('fff', p, i, d),
            is_extended_id=True
        )
        self.bus.send(msg)

    def send_servo_command(self):
        """Send servo position command"""
        if not self.selected_teensy:
            return
            
        position = self.servo_slider.value()
        self.servo_label.setText(f"Position: {position}°")
        
        msg = can.Message(
            arbitration_id=(self.selected_teensy << 8) | 0x22,
            data=struct.pack('f', position),
            is_extended_id=True
        )
        self.bus.send(msg)

def main():
    app = QApplication(sys.argv)
    window = TeensyManager()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
