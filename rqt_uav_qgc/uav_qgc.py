import os
from ament_index_python.packages import get_package_share_directory

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import Bool

class UavQgcPlugin(Plugin):
    def __init__(self, context):
        super(UavQgcPlugin, self).__init__(context)
        self.setObjectName('UavQgcPlugin')

        # Create QWidget
        self._widget = QWidget()
        
        # Get path to UI file
        # Check for development environment or installed environment
        pkg_share_dir = get_package_share_directory('rqt_uav_qgc')
        ui_file = os.path.join(pkg_share_dir, 'resource/UavQgc.ui')

        # set minimum and default size for the plugin UI
        self._widget.setMinimumSize(1200, 800)  
        self._widget.resize(1200, 800)          
        
        # In some development setups, we might need a fallback
        if not os.path.exists(ui_file):
             # Fallback to local path relative to this file
             ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../resource/UavQgc.ui')

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('UavQgcPluginUI')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        context.add_widget(self._widget)

        # ROS2 Node
        self._node = context.node
        self.button1_pub = self._node.create_publisher(Bool, 'button1', 1)
        self.button2_pub = self._node.create_publisher(Bool, 'button2', 1)

        self.count_button_1 = 0
        self.count_button_2 = 0

        # Connect signals
        # Assuming the UI has pushButton1 and pushButton2 based on ros2_rqt_plugin's UI
        self._widget.pushButton1.clicked.connect(self.on_pushButton1_clicked)
        self._widget.pushButton2.clicked.connect(self.on_pushButton2_clicked)

    def on_pushButton1_clicked(self):
        self._node.get_logger().info('Published to button1 topic!')
        msg = Bool()
        msg.data = True
        self.button1_pub.publish(msg)
        self.count_button_1 += 1
        # self._widget.label_1.setText(f"Button 1 was clicked {self.count_button_1}")
        self._widget.InfoBrowser.append(f"Button 1 was clicked {self.count_button_1}")

    def on_pushButton2_clicked(self):
        self._node.get_logger().info('Published to button2 topic!')
        msg = Bool()
        msg.data = True
        self.button2_pub.publish(msg)
        self.count_button_2 += 1
        self._widget.InfoBrowser.append(f"Button 2 was clicked {self.count_button_2}")
        # self._widget.label_2.setText(f"Button 2 was clicked {self.count_button_2}")

    def shutdown_plugin(self):
        # Clean up
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('count_button_1', str(self.count_button_1))
        instance_settings.set_value('count_button_2', str(self.count_button_2))

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('count_button_1'):
            self.count_button_1 = int(instance_settings.value('count_button_1'))
            self._widget.InfoBrowser.append(f"Button 1 was clicked {self.count_button_1}")
        
        if instance_settings.contains('count_button_2'):
            self.count_button_2 = int(instance_settings.value('count_button_2'))
            self._widget.InfoBrowser.append(f"Button 2 was clicked {self.count_button_2}")

def main():
    import sys
    from rqt_gui.main import Main
    plugin = 'rqt_uav_qgc'
    main = Main()
    sys.exit(main.main(sys.argv, standalone=plugin))

if __name__ == '__main__':
    main()
