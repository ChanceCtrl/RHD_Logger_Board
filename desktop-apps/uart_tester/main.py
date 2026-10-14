import sys
from datetime import datetime

import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, QThread, Signal, Slot
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class SerialWorker(QObject):
    received = Signal(bytes)
    disconnected = Signal()
    error = Signal(str)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial = None

    @Slot()
    def run(self):
        self.running = True

        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1,
            )

            while self.running:
                data = self.serial.read(self.serial.in_waiting or 1)
                if data:
                    self.received.emit(data)

        except Exception as e:
            self.error.emit(str(e))

        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()

            self.disconnected.emit()

    def stop(self):
        self.running = False

    def send(self, data: bytes):
        if self.serial and self.serial.is_open:
            self.serial.write(data)


class MainWindow(QWidget):
    MAX_TRANSACTIONS = 20

    def __init__(self):
        super().__init__()

        self.setWindowTitle("UART Monitor")
        self.resize(800, 600)

        self.worker = None
        self.thread = None

        # Port selection
        self.port_combo = QComboBox()

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)

        self.baud_rate_box = QLineEdit()
        self.baud_rate_box.setPlaceholderText("Enter baud rate... (115200)")

        # Log
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)

        # TX controls
        self.tx_box = QLineEdit()
        self.tx_box.setPlaceholderText("Enter data to send...")

        self.format_combo = QComboBox()
        self.format_combo.addItems(
            [
                "ASCII",
                "HEX",
                "Binary",
            ]
        )

        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_data)
        self.send_button.setEnabled(False)

        # Layout
        top = QHBoxLayout()
        top.addWidget(QLabel("Port"))
        top.addWidget(self.port_combo)
        top.addWidget(self.baud_rate_box)
        top.addWidget(self.refresh_button)
        top.addWidget(self.connect_button)

        bottom = QHBoxLayout()
        bottom.addWidget(QLabel("TX"))
        bottom.addWidget(self.tx_box)
        bottom.addWidget(self.format_combo)
        bottom.addWidget(self.send_button)

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(QLabel("Last 20 UART Transactions"))
        layout.addWidget(self.log)
        layout.addLayout(bottom)

        self.refresh_ports()

    def refresh_ports(self):
        current = self.port_combo.currentText()

        self.port_combo.clear()

        for port in serial.tools.list_ports.comports():
            self.port_combo.addItem(port.device)

        idx = self.port_combo.findText(current)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    def toggle_connection(self):
        if self.thread is None:
            self.connect_uart()
        else:
            self.disconnect_uart()

    def connect_uart(self):
        port = self.port_combo.currentText()

        if not port:
            self.add_transaction("No serial port selected.")
            return

        baud = self.baud_rate_box.text().strip()

        if not baud:
            baud = 115200

        self.thread = QThread()

        self.worker = SerialWorker(port, int(baud))
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.received.connect(self.on_receive)
        self.worker.error.connect(lambda msg: self.add_transaction(f"ERROR: {msg}"))
        self.worker.disconnected.connect(self.cleanup)

        self.thread.start()

        self.connect_button.setText("Disconnect")
        self.port_combo.setEnabled(False)
        self.send_button.setEnabled(True)

        self.add_transaction(f"Connected to {port}")

    def disconnect_uart(self):
        if self.worker:
            self.worker.stop()

    def cleanup(self):
        if self.thread:
            self.thread.quit()
            self.thread.wait()

        self.thread = None
        self.worker = None

        self.connect_button.setText("Connect")
        self.port_combo.setEnabled(True)
        self.send_button.setEnabled(False)

        self.add_transaction("Disconnected")

    def send_data(self):
        if self.worker is None:
            return

        text = self.tx_box.text().strip()

        if not text:
            return

        mode = self.format_combo.currentText()

        try:
            if mode == "ASCII":
                payload = text.encode()

            elif mode == "HEX":
                cleaned = text.replace(" ", "").replace("-", "").replace("0x", "")

                payload = bytes.fromhex(cleaned)

            elif mode == "Binary":
                bits = text.replace(" ", "")

                if len(bits) % 8 != 0:
                    raise ValueError("Binary input must contain a multiple of 8 bits.")

                payload = bytes(int(bits[i : i + 8], 2) for i in range(0, len(bits), 8))

            self.worker.send(payload)

            self.add_transaction(f"TX  {self.timestamp()}   {payload.hex(' ').upper()}")

            self.tx_box.clear()

        except Exception as e:
            self.add_transaction(f"TX ERROR: {e}")

    def on_receive(self, data: bytes):
        self.add_transaction(f"RX  {self.timestamp()}   {data.hex(' ').upper()}")

    def timestamp(self):
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def add_transaction(self, text):
        lines = self.log.toPlainText().splitlines()

        lines.append(text)

        if len(lines) > self.MAX_TRANSACTIONS:
            lines = lines[-self.MAX_TRANSACTIONS :]

        self.log.setPlainText("\n".join(lines))

        scrollbar = self.log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())
