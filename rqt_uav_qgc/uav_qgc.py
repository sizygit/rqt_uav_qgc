import os
import re

from ament_index_python.packages import get_package_share_directory

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QLabel, QLineEdit, QMessageBox, QProgressBar,
    QPushButton, QWidget,
)
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import Bool, String

from .ansi_text_edit import AnsiTextEdit
from .ssh_manager import sshDroneManager
from .topic_monitor import TopicMonitor, resolve_msg_type


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

    def __init__(self, card_widget: QWidget, parent_widget: QWidget, suffix: str = "", info_callback=None):
        """
        Parameters
        ----------
        card_widget   The droneCard QFrame as loaded from the .ui file.
        parent_widget The top-level QWidget (used for dialogs / QTimer parent).
        suffix        The name suffix for child widgets (e.g. "", "_2", "_3").
        info_callback Optional callable(str) to forward log messages.
        """
        self._card          = card_widget
        self._parent        = parent_widget
        self._suffix        = suffix
        self._info_callback = info_callback

        # ── Cache all child widget references via findChild ────────────── #
        def _w(cls, name):
            # When copying in Designer, children get the same suffix as the parent.
            runtime_name = f"{name}{suffix}"
            wgt = card_widget.findChild(cls, runtime_name)
            if wgt is None:
                # Fallback: sometimes if the frame was copied, only the frame's name changed.
                # But looking at User's .ui file, the children ARE renamed (e.g. droneIdLabel_2).
                raise AttributeError(
                    f"droneCard{suffix} has no child widget '{runtime_name}' of type {cls.__name__}"
                )
            return wgt

        self._droneIdLineEdit  = _w(QLineEdit,    'droneIdLineEdit')
        self._ipLineEdit       = _w(QLineEdit,    'ipLineEdit')
        self._userLineEdit     = _w(QLineEdit,    'userLineEdit')
        self._passwordLineEdit = _w(QLineEdit,    'passwordLineEdit')
        self._sshStatus     = _w(QLabel,       'sshStatus')
        self._agentStatus   = _w(QLabel,       'agentStatus')
        self._batteryBar    = _w(QProgressBar, 'batteryBar')
        self._batteryValue  = _w(QLabel,       'batteryValue')
        self._connectBtn    = _w(QPushButton,  'connectBtn')
        self._startAgentBtn = _w(QPushButton,  'startAgentBtn')
        self._stopAgentBtn  = _w(QPushButton,  'stopAgentBtn')
        self._shutdownBtn   = _w(QPushButton,  'shutdownBtn')
        self._statusTipLbl  = _w(QLabel,       'statusTip')

        # ── Read static drone info ─────────────────────────────────────── #
        full_text = self._droneIdLineEdit.text()        # e.g. "UAV1"
        self._drone_id = full_text
        
        # Extract numeric ID (e.g., "UAV1" -> 1)
        match = re.search(r'\d+', full_text)
        self.numeric_id = int(match.group()) if match else 0
        
        self._ip       = self._ipLineEdit.text()        # e.g. "10.42.0.1"
        self._user     = self._userLineEdit.text()      # e.g. "luenberger"
        self._password = self._passwordLineEdit.text()  # e.g. "111"

        self._controller: sshDroneManager = None

        self._pending_agent_start = False

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

    def _on_connect_toggled(self, checked: bool):
        if checked:
            self._do_connect()
        else:
            self._do_disconnect()

    def _do_connect(self):
        """Create controller using credentials from UI line edits, then SSH."""
        # Update credentials from UI before connecting
        self._drone_id = self._droneIdLineEdit.text()
        self._ip       = self._ipLineEdit.text()
        self._user     = self._userLineEdit.text()
        self._password = self._passwordLineEdit.text()

        # Extract numeric ID for logic
        import re
        match = re.search(r'\d+', self._drone_id)
        self.numeric_id = int(match.group()) if match else 0

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
        controller = self._controller
        if controller is None:
            return

        # Snapshot the queue to avoid race conditions or None errors during loop
        q = controller.output_queue
        while not q.empty():
            try:
                msg = q.get_nowait()
            except Exception:
                break

            if msg == '__CONNECTED__':
                self._apply_connected_state()
                if self._pending_agent_start:
                    self._on_start_agent()
                    self._pending_agent_start = False
                continue
            if msg == '__FAILED__':
                self._pending_agent_start = False
                self._connectBtn.blockSignals(True)
                self._connectBtn.setChecked(False)
                self._connectBtn.blockSignals(False)
                self._controller = None   # allow retry with updated label values
                self._apply_disconnected_state()
                return  # Exit immediately since controller is now None

            if self._info_callback:
                self._info_callback(msg)

        # Keep agent status label live if still connected
        if self._controller:
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

    def start_all_sequence(self):
        """Helper for 'Start All' button: Connect if needed, then start agent."""
        if self._controller and self._controller.is_connected:
            if not self._controller.is_agent_running:
                self._on_start_agent()
        else:
            self._pending_agent_start = True
            # Explicitly trigger the toggle if not checked
            if not self._connectBtn.isChecked():
                self._connectBtn.setChecked(True)
            else:
                # If checked but not connected, ensure the worker is running
                if not (self._controller and self._controller.is_connected):
                    self._do_connect()


# ──────────────────────────────────────────────────────────────────────────────
#  TopicMonitorCardManager
# ──────────────────────────────────────────────────────────────────────────────

class TopicMonitorCardManager:
    """
    Binds a single monitorFrame QFrame widget to a TopicMonitor instance.

    Widget naming convention (same suffix pattern as DroneCardManager):
      Frame       : monitorFrame{suffix}
      Children    : startMonitorBtn{suffix}, topicNameEdit{suffix},
                    monHzLabel{suffix},      monDataLabel{suffix}

    Duplicate the frame in Qt Designer and give the copy the name
    ``monitorFrame_2``; its children will automatically be found by suffix.
    """

    def __init__(self, frame_widget: QWidget, node, suffix: str = "", msg_type=String):
        """
        Parameters
        ----------
        frame_widget  The monitorFrame QFrame as loaded from the .ui file.
        node          The rclpy Node used for creating subscriptions.
        suffix        Widget-name suffix (e.g. "", "_2", "_3").
        msg_type      ROS 2 message type to subscribe with (default: String).
        """
        self._frame    = frame_widget
        self._node     = node
        self._suffix   = suffix
        self._msg_type = msg_type
        self._monitor: TopicMonitor = None

        def _w(cls, base_name):
            name = f"{base_name}{suffix}"
            wgt = frame_widget.findChild(cls, name)
            if wgt is None:
                raise AttributeError(
                    f"monitorFrame{suffix} has no child '{name}' of type {cls.__name__}"
                )
            return wgt

        self._startBtn    = _w(QPushButton, 'startMonitorBtn')
        self._topicEdit   = _w(QLineEdit,   'topicNameEdit')
        self._hzLabel     = _w(QLabel,      'monHzLabel')
        self._dataLabel   = _w(QLabel,      'monDataLabel')

        self._startBtn.setCheckable(True)
        self._startBtn.toggled.connect(self._on_toggled)

    # ------------------------------------------------------------------ #

    def _on_toggled(self, checked: bool):
        if checked:
            topic_name = self._topicEdit.text().strip()
            if not topic_name:
                self._node.get_logger().error(
                    f"[MonitorCard{self._suffix}] Topic name is empty!"
                )
                self._startBtn.blockSignals(True)
                self._startBtn.setChecked(False)
                self._startBtn.blockSignals(False)
                return

            # Auto-detect message type from live ROS 2 graph
            msg_class, type_info = resolve_msg_type(self._node, topic_name)
            if msg_class is None:
                err = f"[MonitorCard{self._suffix}] {type_info}"
                self._node.get_logger().error(err)
                self._dataLabel.setText(type_info)
                self._startBtn.blockSignals(True)
                self._startBtn.setChecked(False)
                self._startBtn.blockSignals(False)
                return

            self._node.get_logger().info(
                f"[MonitorCard{self._suffix}] Detected type: {type_info}"
            )
            self._dataLabel.setText(f"Type: {type_info}  —  waiting for data...")

            self._monitor = TopicMonitor(
                self._node,
                topic_name,
                msg_class,
                callback=self._on_msg,
            )
            self._node.get_logger().info(
                f"[MonitorCard{self._suffix}] Monitoring '{topic_name}'"
            )
            self._startBtn.setText("Stop Monitor")
        else:
            if self._monitor:
                self._monitor.stop()
                self._monitor = None
            self._startBtn.setText("ROS2 Monitor")
            self._hzLabel.setText("0.0 Hz")
            self._dataLabel.setText("Monitor stopped.")

    def _on_msg(self, monitor: TopicMonitor):
        """Called from the TopicMonitor subscription thread; updates UI labels."""
        self._hzLabel.setText(f"{monitor.hz:.1f} Hz")
        data_str = str(monitor.last_msg)
        if len(data_str) > 200:
            data_str = data_str[:200] + "..."
        self._dataLabel.setText(data_str)

    def cleanup(self):
        """Stop any active subscription."""
        if self._monitor:
            self._monitor.stop()
            self._monitor = None


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

        # Connect Start All button using findChild to double check
        btn_start_all = self._widget.findChild(QPushButton, 'stratAllBtn')
        if btn_start_all:
            btn_start_all.clicked.connect(self._on_start_all_clicked)
            self._node.get_logger().info("Successfully bound stratAllBtn")
        else:
            self._node.get_logger().warning("Could not find stratAllBtn in UI!")

        # ── Topic Monitor card managers ──────────────────────────────────── #
        # One TopicMonitorCardManager per monitorFrame widget in the UI.
        # Add more frames in Qt Designer (monitorFrame_2, monitorFrame_3 …)
        # and they will be auto-discovered here.
        self._monitor_managers = []
        monitor_configs = [
            ("monitorFrame",   ""),
            ("monitorFrame_2", "_2"),
            ("monitorFrame_3", "_3"),
            ("monitorFrame_4", "_4"),
        ]
        for frame_name, suffix in monitor_configs:
            frame_w = self._widget.findChild(QWidget, frame_name)
            if frame_w:
                try:
                    mgr = TopicMonitorCardManager(
                        frame_w, self._node, suffix=suffix
                    )
                    self._monitor_managers.append(mgr)
                    self._node.get_logger().info(
                        f"Bound TopicMonitorCardManager for '{frame_name}'"
                    )
                except AttributeError as exc:
                    self._node.get_logger().warning(str(exc))

        # ── Drone card managers ───────────────────────────────────────────── #
        # One DroneCardManager per physical droneCard widget in the UI.
        self._drone_managers = []
        
        # We look only for droneCard (2 and 3 removed per user request).
        configs = [
            ("droneCard", ""),
            ("droneCard_2", "_2"),
            ("droneCard_3", "_3"),
        ]

        for card_name, suffix in configs:
            card_w = getattr(self._widget, card_name, None)
            if card_w:
                mgr = DroneCardManager(
                    card_w, self._widget, suffix=suffix, 
                    info_callback=self._append_info
                )
                self._drone_managers.append(mgr)

    # ── Utility exposed to DroneCardManager ─────────────────────────────── #

    def _append_info(self, msg: str):
        """Append a line (may contain ANSI codes) to the log panel."""
        self._info_edit.appendAnsi(msg)
        self._node.get_logger().info(msg)

    def _on_start_all_clicked(self):
        """Iterate through all drone managers and trigger connect/start agent."""
        self._node.get_logger().info("Start All clicked")
        print("Start All clicked") # In case of RQT console
        for mgr in self._drone_managers:
            mgr.start_all_sequence()

    # ── Original button handlers ─────────────────────────────────────────── #
    def shutdown_plugin(self):
        for mgr in self._drone_managers:
            mgr.cleanup()
        for mgr in self._monitor_managers:
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
