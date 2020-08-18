import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from iiwas_srv.srv import *


class IiwasManager(Plugin):

    def __init__(self, context):
        super(IiwasManager, self).__init__(context)
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

        self._set_iw_services()

        self._connect_services_to_buttons()

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

    def _set_iw_services(self):
        cancel_srv_name = 'cancel_motion'
        ptp_srv_name = 'ptp'
        light_srv_name = 'set_blue_light'
        handguiding_srv_name = 'start_handguiding'
        position_control_srv_name = 'start_position_control'

        # rospy.wait_for_service(cancel_srv_name)
        # rospy.wait_for_service(ptp_srv_name)
        # rospy.wait_for_service(light_srv_name)
        # rospy.wait_for_service(handguiding_srv_name)
        # rospy.wait_for_service(position_control_srv_name)

        self._cancel_srv = rospy.ServiceProxy(cancel_srv_name, CancelMotion, persistent=True)
        self._ptp_srv = rospy.ServiceProxy(ptp_srv_name, PTP, persistent=True)
        self._light_srv = rospy.ServiceProxy(light_srv_name, SetBlueLight, persistent=True)
        self._handguiding_srv = rospy.ServiceProxy(handguiding_srv_name, StartHandguiding, persistent=True)
        self._position_control_srv = rospy.ServiceProxy(position_control_srv_name, StartPositionControl,
                                                        persistent=True)

    def _cancel_f_cb(self):
        res = self._cancel_srv.call(CancelMotionRequest(which_iiwa=1))

    def _cancel_b_cb(self):
       res = self._cancel_srv.call(CancelMotionRequest(which_iiwa=2))

    def _handguiding_f_cb(self):
        res = self._handguiding_srv.call(StartHandguidingRequest(which_iiwa=1))

    def _handguiding_b_cb(self):
        res = self._handguiding_srv.call(StartHandguidingRequest(which_iiwa=2))

    def _position_f_cb(self):
        res = self._position_control_srv.call(StartPositionControlRequest(which_iiwa=1))

    def _position_b_cb(self):
        res = self._position_control_srv.call(StartPositionControlRequest(which_iiwa=2))

    def _connect_services_to_buttons(self):
        self._widget.cancel_f.clicked[bool].connect(self._cancel_f_cb)
        self._widget.cancel_b.clicked[bool].connect(self._cancel_b_cb)

        self._widget.handguiding_f.clicked[bool].connect(self._handguiding_f_cb)
        self._widget.handguiding_b.clicked[bool].connect(self._handguiding_b_cb)

        self._widget.position_f.clicked[bool].connect(self._position_f_cb)
        self._widget.position_b.clicked[bool].connect(self._position_b_cb)



