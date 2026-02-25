import os

from ament_index_python.packages import get_package_share_directory

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QLabel, QMessageBox, QProgressBar,
    QPushButton, QWidget,
)
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import Bool

from .ansi_text_edit import AnsiTextEdit
from .ssh_manager import sshDroneManager


# ──────────────────────────────────────────────────────────────────────────────
#  DroneCardManager
# ──────────────────────────────────────────────────────────────────────────────

class DroneCardManager:
    """
    Binds a single droneCard QFrame widget to a DroneController instance.

    When loadUi() loads the .ui file, all named widgets become attributes of
    the *top-level* QWidget, NOT of their parent QFrame.  Therefore we use
    QWidget.findChild() on the QFrame to locate its children explicitly and
    cache them as instance attributes.
    """

    # colour constants
    _RED    = 'color: #e74c3c;'
    _GREEN  = 'color: #2ecc71;'
    _GREY   = 'color: #95a5a6;'

    def __init__(self, card_widget: QWidget, parent_widget: QWidget, info_callback=None):
        """
        Parameters
        ----------
        card_widget   The droneCard QFrame as loaded from the .ui file.
        parent_widget The top-level QWidget (used for dialogs / QTimer parent).
        info_callback Optional callable(str) to forward log messages (e.g. to InfoBrowser).
        """
        self._card          = card_widget
        self._parent        = parent_widget
        self._info_callback = info_callback

        # ── Cache all child widget references via findChild ────────────── #
        def _w(cls, name):
            wgt = card_widget.findChild(cls, name)
            if wgt is None:
                raise AttributeError(
                    f"droneCard has no child widget '{name}' of type {cls.__name__}"
                )
            return wgt

        # self._statusDot     = _w(QLabel,       'statusDot')
        self._droneIdLabel  = _w(QLabel,       'droneIdLabel')
        self._ipLabel       = _w(QLabel,       'ipLabel')
        self._userLabel     = _w(QLabel,       'userLabel')
        self._passwordLabel = _w(QLabel,       'passwordLabel')
        self._sshStatus     = _w(QLabel,       'sshStatus')
        self._agentStatus   = _w(QLabel,       'agentStatus')
        self._batteryBar    = _w(QProgressBar, 'batteryBar')
        self._batteryValue  = _w(QLabel,       'batteryValue')
        self._connectBtn    = _w(QPushButton,  'connectBtn')
        self._startAgentBtn = _w(QPushButton,  'startAgentBtn')
        self._stopAgentBtn  = _w(QPushButton,  'stopAgentBtn')
        self._shutdownBtn   = _w(QPushButton,  'shutdownBtn')
        self._statusTipLbl  = _w(QLabel,       'statusTip')

        # ── Read static drone info from UI labels ──────────────────────── #
        self._drone_id = self._droneIdLabel.text()   # e.g. "UAV1"
        self._ip       = self._ipLabel.text()        # e.g. "10.42.0.1"
        self._user     = self._userLabel.text()      # e.g. "luenberger"
        self._password = self._passwordLabel.text()  # e.g. "111"

        self._controller: sshDroneManager = None

        # ── QTimer: drain the output queue every 200 ms ────────────────── #
        self._poll_timer = QTimer(parent_widget)
        self._poll_timer.setInterval(200)
        self._poll_timer.timeout.connect(self._drain_queue)
        self._poll_timer.start()

        # ── Wire UI signals ────────────────────────────────────────────── #
        self._connectBtn.toggled.connect(self._on_connect_toggled)
        self._startAgentBtn.clicked.connect(self._on_start_agent)
        self._stopAgentBtn.clicked.connect(self._on_stop_agent)
        self._shutdownBtn.clicked.connect(self._on_shutdown)

        # ── Initial UI state ───────────────────────────────────────────── #
        self._apply_disconnected_state()

    # ------------------------------------------------------------------ #
    #  Button slots
    # ------------------------------------------------------------------ #

    def _on_connect_toggled(self, checked: bool):
        if checked:
            self._do_connect()
        else:
            self._do_disconnect()

    def _do_connect(self):
        """Create controller using credentials from UI labels, then SSH in background."""
        if self._controller is None:
            self._controller = sshDroneManager(
                drone_id=self._drone_id,
                ip=self._ip,
                username=self._user,
                password=self._password or None,
            )

        self._set_status_tip('Connecting...')

        import threading
        threading.Thread(target=self._connect_worker, daemon=True).start()

    def _connect_worker(self):
        success = self._controller.connect()
        self._controller.output_queue.put(
            '__CONNECTED__' if success else '__FAILED__'
        )

    def _do_disconnect(self):
        if self._controller:
            self._controller.disconnect()
        self._apply_disconnected_state()
        self._controller = None
        self._connectBtn.setText('Connect')

    def _on_start_agent(self):
        if self._controller:
            self._set_status_tip('Starting MicroXRCEAgent…')
            import threading
            threading.Thread(
                target=self._controller.start_agent, daemon=True
            ).start()

    def _on_stop_agent(self):
        if self._controller:
            self._set_status_tip('Stopping agent…')
            import threading
            threading.Thread(
                target=self._controller.stop_agent, daemon=True
            ).start()

    def _on_shutdown(self):
        reply = QMessageBox.question(
            self._parent,
            f'Shutdown {self._drone_id}?',
            f'Are you sure you want to shut down {self._ip}?',
        )
        if reply == QMessageBox.Yes and self._controller:
            self._set_status_tip('Sending shutdown…')
            import threading
            threading.Thread(
                target=self._controller.shutdown_remote, daemon=True
            ).start()

    # ------------------------------------------------------------------ #
    #  Queue polling & UI state helpers
    # ------------------------------------------------------------------ #

    def _drain_queue(self):
        if self._controller is None:
            return

        while not self._controller.output_queue.empty():
            try:
                msg = self._controller.output_queue.get_nowait()
            except Exception:
                break

            if msg == '__CONNECTED__':
                self._apply_connected_state()
                continue
            if msg == '__FAILED__':
                self._connectBtn.blockSignals(True)
                self._connectBtn.setChecked(False)
                self._connectBtn.blockSignals(False)
                self._controller = None   # allow retry with updated label values
                self._apply_disconnected_state()
                continue

            # self._set_status_tip(msg)
            if self._info_callback:
                self._info_callback(msg)

        # Keep agent status label live.
        self._set_agent_status(running=self._controller.is_agent_running)

    def _apply_connected_state(self):
        # self._statusDot.setStyleSheet(self._GREEN + ' font-size: 16px;')
        self._sshStatus.setText('Connected')
        self._sshStatus.setStyleSheet(self._GREEN + ' font-weight: bold;')
        self._connectBtn.setText('Disconnect')
        self._startAgentBtn.setEnabled(True)
        self._shutdownBtn.setEnabled(True)
        self._set_status_tip('SSH connection established.')

    def _apply_disconnected_state(self):
        # self._statusDot.setStyleSheet(self._RED + ' font-size: 16px;')
        self._sshStatus.setText('Disconnected')
        self._sshStatus.setStyleSheet(self._RED + ' font-weight: bold;')
        self._connectBtn.setText('Connect')
        self._startAgentBtn.setEnabled(False)
        self._stopAgentBtn.setEnabled(False)
        self._shutdownBtn.setEnabled(False)
        self._set_agent_status(running=False)

    def _set_agent_status(self, running: bool):
        if running:
            self._agentStatus.setText('Running')
            self._agentStatus.setStyleSheet(self._GREEN)
            self._stopAgentBtn.setEnabled(True)
            self._startAgentBtn.setEnabled(False)
        else:
            self._agentStatus.setText('Stopped')
            self._agentStatus.setStyleSheet(self._GREY)
            self._stopAgentBtn.setEnabled(False)
            if self._controller and self._controller.is_connected:
                self._startAgentBtn.setEnabled(True)

    def _set_status_tip(self, msg: str):
        last_line = msg.strip().splitlines()[-1] if msg.strip() else msg
        self._statusTipLbl.setText(last_line)

    def cleanup(self):
        self._poll_timer.stop()
        if self._controller:
            self._controller.cleanup()


# ──────────────────────────────────────────────────────────────────────────────
#  RQT Plugin
# ──────────────────────────────────────────────────────────────────────────────

class UavQgcPlugin(Plugin):
    def __init__(self, context):
        super(UavQgcPlugin, self).__init__(context)
        self.setObjectName('UavQgcPlugin')

        # ── Load UI ──────────────────────────────────────────────────────── #
        self._widget = QWidget()

        pkg_share_dir = get_package_share_directory('rqt_uav_qgc')
        ui_file = os.path.join(pkg_share_dir, 'resource/UavQgc.ui')

        self._widget.setMinimumSize(1200, 800)
        self._widget.resize(1200, 800)

        if not os.path.exists(ui_file):
            ui_file = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                '../resource/UavQgc.ui',
            )

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('UavQgcPluginUI')

        # ── Replace QPlainTextEdit with AnsiTextEdit ──────────────────────── #
        # loadUi creates infoPlainTextEdit as a plain QPlainTextEdit.
        # We swap it out for our ANSI-capable AnsiTextEdit, preserving geometry.
        _old = self._widget.infoPlainTextEdit
        self._info_edit = AnsiTextEdit(self._widget)
        self._info_edit.setObjectName('infoPlainTextEdit')
        self._info_edit.setGeometry(_old.geometry())
        _old.hide()
        _old.setParent(None)
        self._info_edit.show()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        context.add_widget(self._widget)

        # ── ROS 2 publishers (kept for backwards compat) ─────────────────── #
        self._node = context.node
        self.button1_pub = self._node.create_publisher(Bool, 'button1', 1)
        self.button2_pub = self._node.create_publisher(Bool, 'button2', 1)

        self.count_button_1 = 0
        self.count_button_2 = 0

        # Connect signals
        # Assuming the UI has pushButton1 and pushButton2 based on ros2_rqt_plugin's UI
        self._widget.pushButton1.clicked.connect(self.on_pushButton1_clicked)
        self._widget.pushButton2.clicked.connect(self.on_pushButton2_clicked)

        # ── Drone card managers ───────────────────────────────────────────── #
        # One DroneCardManager per physical droneCard widget in the UI.
        self._drone_managers = []
        self._drone_managers.append(
            DroneCardManager(self._widget.droneCard, self._widget,
                             info_callback=self._append_info)
        )
        # Add more managers here for additional drone cards as the UI grows.

    # ── Utility exposed to DroneCardManager ─────────────────────────────── #

    def _append_info(self, msg: str):
        """Append a line (may contain ANSI codes) to the log panel."""
        self._info_edit.appendAnsi(msg)
        self._node.get_logger().info(msg)

    # ── Original button handlers ─────────────────────────────────────────── #

    def on_pushButton1_clicked(self):
        self._node.get_logger().info('Published to button1 topic!')
        msg = Bool()
        msg.data = True
        self.button1_pub.publish(msg)
        self.count_button_1 += 1
        self._info_edit.appendAnsi(f'Button 1 was clicked {self.count_button_1}')

    def on_pushButton2_clicked(self):
        self._node.get_logger().info('Published to button2 topic!')
        msg = Bool()
        msg.data = True
        self.button2_pub.publish(msg)
        self.count_button_2 += 1
        self._info_edit.appendAnsi(f'Button 2 was clicked {self.count_button_2}')

    def shutdown_plugin(self):
        for mgr in self._drone_managers:
            mgr.cleanup()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('count_button_1', str(self.count_button_1))
        instance_settings.set_value('count_button_2', str(self.count_button_2))

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('count_button_1'):
            self.count_button_1 = int(instance_settings.value('count_button_1'))
            self._info_edit.appendAnsi(f'Button 1 was clicked {self.count_button_1}')
        if instance_settings.contains('count_button_2'):
            self.count_button_2 = int(instance_settings.value('count_button_2'))
            self._info_edit.appendAnsi(f'Button 2 was clicked {self.count_button_2}')


def main():
    import sys
    from rqt_gui.main import Main
    plugin = 'rqt_uav_qgc'
    main = Main()
    sys.exit(main.main(sys.argv, standalone=plugin))


if __name__ == '__main__':
    main()
