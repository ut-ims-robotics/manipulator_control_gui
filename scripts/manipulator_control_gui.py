#!/usr/bin/env python
import sys

from rqt_gui.main import Main

main = Main()
sys.exit(main.main(sys.argv, standalone='manipulator_control_gui.manipulator_dashboard.ManipulatorDashboard'))