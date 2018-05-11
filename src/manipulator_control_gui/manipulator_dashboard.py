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
        self._widget.dimension_dropdown.currentIndexChanged.connect(self.dimension_changed)

    def init_gui_elements(self):
        self._widget.dimension_dropdown.addItem("X")
        self._widget.dimension_dropdown.addItem("Y")
        self._widget.dimension_dropdown.addItem("Z")
        self._widget.dimension_dropdown.addItem("RX")
        self._widget.dimension_dropdown.addItem("RY")
        self._widget.dimension_dropdown.addItem("RZ")

    def control_law_changed(self):
        self.direction_control_laws.x.type = ControlLaw.TYPE_UNKNOWN
        self.direction_control_laws.y.type = ControlLaw.TYPE_COMPLIANT_MOVE
        self.direction_control_laws.z.type = ControlLaw.TYPE_SPRING
        self.direction_control_laws.rx.type = ControlLaw.TYPE_UNKNOWN
        self.direction_control_laws.ry.type = ControlLaw.TYPE_UNKNOWN
        self.direction_control_laws.rz.type = ControlLaw.TYPE_UNKNOWN

    def dimension_changed(self):
        self.active_dimension = self._widget.dimension_dropdown.currentText()
        rospy.loginfo("Active dimension changed to {}".format(self.active_dimension))

    def damping_changed(self):
        value = self._widget.damping_slider.value()

        if "R" in self.active_dimension:
            self.set_cartesian_impedance.damping.rotational.x = value if (
                    "X" in self.active_dimension) else self.set_cartesian_impedance.damping.rotational.x
            self.set_cartesian_impedance.damping.rotational.y = value if (
                    "Y" in self.active_dimension) else self.set_cartesian_impedance.damping.rotational.y
            self.set_cartesian_impedance.damping.rotational.z = value if (
                    "Z" in self.active_dimension) else self.set_cartesian_impedance.damping.rotational.z
        else:
            self.set_cartesian_impedance.damping.translational.x = value if (
                    "X" in self.active_dimension) else self.set_cartesian_impedance.damping.translational.x
            self.set_cartesian_impedance.damping.translational.y = value if (
                    "Y" in self.active_dimension) else self.set_cartesian_impedance.damping.translational.y
            self.set_cartesian_impedance.damping.translational.z = value if (
                    "Z" in self.active_dimension) else self.set_cartesian_impedance.damping.translational.z

        rospy.loginfo("Damping changed")
        rospy.loginfo(self.set_cartesian_impedance.damping)

    def max_velocity_changed(self):
        self.set_cartesian_impedance.max_cart_vel.set.linear.x = 0.05
        self.set_cartesian_impedance.max_cart_vel.set.linear.y = 0.05
        self.set_cartesian_impedance.max_cart_vel.set.linear.z = 0.05
        self.set_cartesian_impedance.max_cart_vel.set.angular.x = 0.0
        self.set_cartesian_impedance.max_cart_vel.set.angular.y = 0.0
        self.set_cartesian_impedance.max_cart_vel.set.angular.z = 0.0

    def max_control_force_changed(self):
        self.set_cartesian_impedance.max_ctrl_force.set.force.x = 25.0
        self.set_cartesian_impedance.max_ctrl_force.set.force.y = 25.0
        self.set_cartesian_impedance.max_ctrl_force.set.force.z = 25.0
        self.set_cartesian_impedance.max_ctrl_force.set.torque.x = 25.0
        self.set_cartesian_impedance.max_ctrl_force.set.torque.y = 25.0
        self.set_cartesian_impedance.max_ctrl_force.set.torque.z = 25.0

    def max_path_deviation_changed(self):
        self.set_cartesian_impedance.max_path_deviation.translation.x = 0.02
        self.set_cartesian_impedance.max_path_deviation.translation.y = 0.15
        self.set_cartesian_impedance.max_path_deviation.translation.z = 0.2  # Change this to edit distance from door
        self.set_cartesian_impedance.max_path_deviation.rotation.z = 0.0
        self.set_cartesian_impedance.max_path_deviation.rotation.z = 0.0
        self.set_cartesian_impedance.max_path_deviation.rotation.z = 0.0

    def stiffness_changed(self):
        self.set_cartesian_impedance.stiffness.translational.x = 10.0
        self.set_cartesian_impedance.stiffness.translational.y = 0.0
        self.set_cartesian_impedance.stiffness.translational.z = 10.0
        self.set_cartesian_impedance.stiffness.rotational.x = 0.0
        self.set_cartesian_impedance.stiffness.rotational.y = 0.0
        self.set_cartesian_impedance.stiffness.rotational.z = 0.0

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
