import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class IiwasManager(Plugin):

    def __init__(self, context):
        super().__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Iiwas Manager')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-v", "--verbose", action="store_true",
                            dest="verbose",
                            help="Put plugin in verbose mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if args.verbose:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('iiwas_rqt'), 'resource', 'IiwasManager.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('IiwasManagerUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog