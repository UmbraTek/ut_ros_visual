#!/usr/bin/env python  
from python_qt_binding.QtCore import QCoreApplication, Qt
try:
    from python_qt_binding.QtGui import QWidget, QApplication, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, \
        QPushButton,QCheckBox
except ImportError:
    from python_qt_binding.QtWidgets import QWidget, QApplication, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, \
        QPushButton,QCheckBox
import rospy
import sys
from Utra import Utra

class Graspe(QWidget):
    NOT_INITED_YET = 0
    BAD_PLAN = 1
    GOOD_PLAN = 2
    MOVED_TO_POSE = 3
    BAD_STARTING_POSITION = 4
    GOOD_STARTING_POSITION = 5
    CHECKING_STARTING_POSITION = 6
    MOVEMENT_FAILED = 7

    def __init__(self):
        super(Graspe, self).__init__()
        self.current_target_pose = -1  # -1 is home
        self.target_poses = None
        self.Utra = Utra()
        self.plan_was_successful = None
        self.state = Graspe.NOT_INITED_YET

        self.layout = QVBoxLayout()
        self.labels_layout = QHBoxLayout()
        self.buttons_layout = QHBoxLayout()

        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximum(4)
        self.pose_number_lbl = QLabel('0/4')
        self.bad_plan_lbl = QLabel('No plan yet')
        self.bad_plan_lbl.setAlignment(Qt.AlignCenter)
        self.auto = QCheckBox('Auto')
        self.auto.clicked.connect(self.handle_auto)

        self.guide_lbl = QLabel('Ready')
        self.guide_lbl.setWordWrap(True)

        self.check_start_pose_btn = QPushButton('start pose')
        self.check_start_pose_btn.clicked.connect(self.handle_start_pose)

        self.next_pose_btn = QPushButton('Next Pose')
        self.next_pose_btn.clicked.connect(self.handle_next_pose)

        self.plan_btn = QPushButton('Plan')
        self.plan_btn.clicked.connect(self.handle_plan)

        self.execute_btn = QPushButton('Execute')
        self.execute_btn.clicked.connect(self.handle_execute)

        self.labels_layout.addWidget(self.pose_number_lbl)
        self.labels_layout.addWidget(self.bad_plan_lbl)

        self.buttons_layout.addWidget(self.check_start_pose_btn)
        self.buttons_layout.addWidget(self.next_pose_btn)
        self.buttons_layout.addWidget(self.plan_btn)
        self.buttons_layout.addWidget(self.execute_btn)

        self.layout.addWidget(self.progress_bar)
        self.layout.addLayout(self.labels_layout)
        self.layout.addWidget(self.guide_lbl)
        self.layout.addLayout(self.buttons_layout)

        self.setLayout(self.layout)

        self.execute_btn.setEnabled(False)
        self.plan_btn.setEnabled(False)

        self.setWindowTitle('Local Mover')
        self.show()

    def handle_start_pose(self):
        step,msg = self.Utra.plan_start_pose()
        self.guide_lbl.setText(msg)
        self.pose_number_lbl.setText(str(step)+'/4')
        self.progress_bar.setValue(step)
        self.execute_btn.setEnabled(False)
        self.plan_btn.setEnabled(True)
        

    def handle_next_pose(self):
        step,msg = self.Utra.next_pose()
        self.guide_lbl.setText(msg)
        self.pose_number_lbl.setText(str(step)+'/4')
        self.progress_bar.setValue(step)
        self.execute_btn.setEnabled(False)

    def handle_plan(self):
        ret,msg = self.Utra.plan()
        if(ret):
            self.execute_btn.setEnabled(True)
        else:
            self.execute_btn.setEnabled(False)
        self.bad_plan_lbl.setText(msg)

    def handle_execute(self):
        self.Utra.execute_plan()

    def handle_auto(self):
        state = self.auto.checkState()
        if(state == 2):
            self.check_start_pose_btn.setEnabled(False)
            self.next_pose_btn.setEnabled(False)
            self.plan_btn.setEnabled(False)
            self.execute_btn.setEnabled(False)
        if(state == 0):
            self.check_start_pose_btn.setEnabled(True)
            self.next_pose_btn.setEnabled(True)
            self.plan_btn.setEnabled(True)
            self.execute_btn.setEnabled(True)

if __name__ == '__main__':

    NODE_NAME = 'umbratek_rqt_graspe'

    rospy.init_node(NODE_NAME)
    while rospy.get_time() == 0.0:
        pass

    qapp = QApplication(sys.argv)
    graspe = Graspe()
    graspe.show()
    sys.exit(qapp.exec_())