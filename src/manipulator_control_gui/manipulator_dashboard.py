import os
import rospy
import rospkg

from cartesian_impedance_msgs.srv import *
from cartesian_impedance_msgs.msg import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class ManipulatorDashboard(Plugin):
    def __init__(self, context):
        super(ManipulatorDashboard, self).__init__(context)
        self.setObjectName('ManipulatorDashboard')

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('manipulator_control_gui'), 'resources',
                               'manipulator_dashboard.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ManipulatorDashboard')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)
        self.active_dimension = "X"
        self.configure_cartesian_impedance = None
        self.direction_control_laws = DirectionControlLaws()
        self.set_cartesian_impedance = SetCartesianImpedance()
        self.set_cartesian_force_control = SetCartesianForceCtrl()
        self.init_ros()
        self.init_gui_elements()
        self.init_signals()

    def init_ros(self):
        self.configure_cartesian_impedance = rospy.ServiceProxy('configure_cartesian_impedance_blocking',
                                                                ConfigureForceControl)

    def init_signals(self):
        self._widget.damping_slider.valueChanged.connect(self.damping_changed)
        self._widget.stiffness_slider.valueChanged.connect(self.stiffness_changed)
        self._widget.max_velocity_slider.valueChanged.connect(self.max_velocity_changed)
        self._widget.max_force_slider.valueChanged.connect(self.max_force_changed)
        self._widget.max_path_dev_slider.valueChanged.connect(self.max_path_deviation_changed)
        self._widget.dimension_dropdown.currentIndexChanged.connect(self.dimension_changed)
        self._widget.control_law_dropdown.currentIndexChanged.connect(self.control_law_changed)

    def init_gui_elements(self):
        self._widget.dimension_dropdown.addItem("X")
        self._widget.dimension_dropdown.addItem("Y")
        self._widget.dimension_dropdown.addItem("Z")
        self._widget.dimension_dropdown.addItem("RX")
        self._widget.dimension_dropdown.addItem("RY")
        self._widget.dimension_dropdown.addItem("RZ")

        self._widget.control_law_dropdown.addItem("Disabled")
        self._widget.control_law_dropdown.addItem("Compliant move")
        self._widget.control_law_dropdown.addItem("Follower")
        self._widget.control_law_dropdown.addItem("Spring")

    def set_value(self, dimension, value, orig_value):
        return value if dimension == self.active_dimension else orig_value

    def control_law_changed(self):
        control_law_dict = {'Disabled' : ControlLaw.TYPE_UNKNOWN,
                   'Compliant move' : ControlLaw.TYPE_COMPLIANT_MOVE,
                   'Follower' : ControlLaw.TYPE_FOLLOWER,
                   'Spring' : ControlLaw.TYPE_SPRING}
        value = control_law_dict[self._widget.control_law_dropdown.currentText()]

        self.direction_control_laws.x.type = self.set_value("X", value, self.direction_control_laws.x.type)
        self.direction_control_laws.y.type = self.set_value("Y", value, self.direction_control_laws.y.type)
        self.direction_control_laws.z.type = self.set_value("Z", value, self.direction_control_laws.z.type)
        self.direction_control_laws.rx.type = self.set_value("RX", value, self.direction_control_laws.rx.type)
        self.direction_control_laws.ry.type = self.set_value("RY", value, self.direction_control_laws.ry.type)
        self.direction_control_laws.rz.type = self.set_value("RZ", value, self.direction_control_laws.rz.type)

        self._widget.overview.setText("{}\n{}".format(self.direction_control_laws, self.set_cartesian_impedance))
        rospy.loginfo(self.direction_control_laws)

    def dimension_changed(self):
        self.active_dimension = self._widget.dimension_dropdown.currentText()
        rospy.loginfo("Active dimension changed to {}".format(self.active_dimension))

    def damping_changed(self):
        value = self._widget.damping_slider.value()

        self.set_cartesian_impedance.damping.rotational.x = self.set_value("RX", value,
                                                                           self.set_cartesian_impedance.damping.rotational.x)
        self.set_cartesian_impedance.damping.rotational.y = self.set_value("RY", value,
                                                                           self.set_cartesian_impedance.damping.rotational.y)
        self.set_cartesian_impedance.damping.rotational.z = self.set_value("RZ", value,
                                                                           self.set_cartesian_impedance.damping.rotational.z)
        self.set_cartesian_impedance.damping.translational.x = self.set_value("X", value,
                                                                              self.set_cartesian_impedance.damping.translational.x)
        self.set_cartesian_impedance.damping.translational.y = self.set_value("Y", value,
                                                                              self.set_cartesian_impedance.damping.translational.y)
        self.set_cartesian_impedance.damping.translational.z = self.set_value("Z", value,
                                                                              self.set_cartesian_impedance.damping.translational.z)
        self._widget.current_damping.setText("Damping: {}".format(value))

    def stiffness_changed(self):
        value = self._widget.stiffness_slider.value()

        self.set_cartesian_impedance.stiffness.translational.x = self.set_value("X", value, self.set_cartesian_impedance.stiffness.translational.x)
        self.set_cartesian_impedance.stiffness.translational.y = self.set_value("Y", value, self.set_cartesian_impedance.stiffness.translational.y)
        self.set_cartesian_impedance.stiffness.translational.z = self.set_value("Z", value, self.set_cartesian_impedance.stiffness.translational.z)
        self.set_cartesian_impedance.stiffness.rotational.x = self.set_value("RX", value, self.set_cartesian_impedance.stiffness.rotational.x)
        self.set_cartesian_impedance.stiffness.rotational.y = self.set_value("RY", value, self.set_cartesian_impedance.stiffness.rotational.y)
        self.set_cartesian_impedance.stiffness.rotational.z = self.set_value("RZ", value, self.set_cartesian_impedance.stiffness.rotational.z)

        self._widget.current_stiffness.setText("Stiffness: {}".format(value))

    def max_velocity_changed(self):
        value = self._widget.max_velocity_slider.value() / 1000.0

        self.set_cartesian_impedance.max_cart_vel.set.linear.x = self.set_value("X", value, self.set_cartesian_impedance.max_cart_vel.set.linear.x)
        self.set_cartesian_impedance.max_cart_vel.set.linear.y = self.set_value("Y", value, self.set_cartesian_impedance.max_cart_vel.set.linear.y)
        self.set_cartesian_impedance.max_cart_vel.set.linear.z = self.set_value("Z", value, self.set_cartesian_impedance.max_cart_vel.set.linear.z)
        self.set_cartesian_impedance.max_cart_vel.set.angular.x = self.set_value("RX", value, self.set_cartesian_impedance.max_cart_vel.set.angular.x)
        self.set_cartesian_impedance.max_cart_vel.set.angular.y = self.set_value("RY", value, self.set_cartesian_impedance.max_cart_vel.set.angular.y)
        self.set_cartesian_impedance.max_cart_vel.set.angular.z = self.set_value("RZ", value, self.set_cartesian_impedance.max_cart_vel.set.angular.z)

        self._widget.current_max_velocity.setText("Max velocity: {}".format(value))

    def max_force_changed(self):
        value = self._widget.max_force_slider.value()

        self.set_cartesian_impedance.max_ctrl_force.set.force.x = self.set_value("X", value, self.set_cartesian_impedance.max_ctrl_force.set.force.x)
        self.set_cartesian_impedance.max_ctrl_force.set.force.y = self.set_value("Y", value, self.set_cartesian_impedance.max_ctrl_force.set.force.y)
        self.set_cartesian_impedance.max_ctrl_force.set.force.z = self.set_value("Z", value, self.set_cartesian_impedance.max_ctrl_force.set.force.z)
        self.set_cartesian_impedance.max_ctrl_force.set.torque.x = self.set_value("RX", value, self.set_cartesian_impedance.max_ctrl_force.set.torque.x)
        self.set_cartesian_impedance.max_ctrl_force.set.torque.y = self.set_value("RY", value, self.set_cartesian_impedance.max_ctrl_force.set.torque.y)
        self.set_cartesian_impedance.max_ctrl_force.set.torque.z = self.set_value("RZ", value, self.set_cartesian_impedance.max_ctrl_force.set.torque.z)

        self._widget.current_max_force.setText("Max force: {}".format(value))

    def max_path_deviation_changed(self):
        value = self._widget.max_path_dev_slider.value() / 1000.0

        self.set_cartesian_impedance.max_path_deviation.translation.x = self.set_value("X", value, self.set_cartesian_impedance.max_path_deviation.translation.x)
        self.set_cartesian_impedance.max_path_deviation.translation.y = self.set_value("Y", value, self.set_cartesian_impedance.max_path_deviation.translation.y)
        self.set_cartesian_impedance.max_path_deviation.translation.z = self.set_value("Z", value, self.set_cartesian_impedance.max_path_deviation.translation.z)

        self.set_cartesian_impedance.max_path_deviation.rotation.x = self.set_value("RX", value, self.set_cartesian_impedance.max_path_deviation.rotation.x)
        self.set_cartesian_impedance.max_path_deviation.rotation.y = self.set_value("RY", value, self.set_cartesian_impedance.max_path_deviation.rotation.y)
        self.set_cartesian_impedance.max_path_deviation.rotation.z = self.set_value("RZ", value, self.set_cartesian_impedance.max_path_deviation.rotation.z)

        self._widget.current_max_path_dev.setText("Max path deviation: {}".format(value))

    def send_service(self):
        try:
            response = self.configure_cartesian_impedance(self.set_cartesian_impedance,
                                                          self.set_cartesian_force_control,
                                                          self.direction_control_laws)
            rospy.loginfo(response)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s" % e)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
