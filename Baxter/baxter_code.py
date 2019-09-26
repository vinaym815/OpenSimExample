# Using this code you can interactively program baxter waypoints to baxter
# robot and replay them as many times as required.
# This code was written for NHK TV demonstration 

import rospy, baxter_interface

class Assist:
    def __init__(self, arm, speed=0.3, accuracy=baxter_interface.JOINT_ANGLE_TOLERANCE):

        self._arm = arm
        self._speed = speed
        self._accuracy = accuracy
        self._limb = baxter_interface.Limb(self._arm)

        self._path = list()
        self._path_recording = True
        self._provide_assist = True

        # creating a navigator io object
        self._navigator_io = baxter_interface.Navigator(self._arm)

        # checking the robot state   
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled

        # Enabling the robot if it not already enabled
        if not self._init_state:
            self._rs.enable()

    def _add_path_point(self, value):
        if value:
            self._path.append(self._limb.joint_angles())
            if len (self._path) == 1:
                print('Add one more path point')
            elif len(self._path) == 2:
                print('Press button 2 to finish or button 0 to restart path registration')
            else :
                self._reset_path(True)
                print('Added more than 3 path points.')
                print('Restarting path registration.\nPress button 0 to add path point.\nPress button 1 to restart\n\n')

    def _reset_path(self, value):
        if value:
            self._path = list()
            print('Reset the path list')
    
    def _finish_path_recording(self, value):
        if value:
            self._path_recording = False
            print(self._path)

    def record_path(self):
        self._navigator_io.button0_changed.connect(self._add_path_point)
        self._navigator_io.button1_changed.connect(self._reset_path)
        self._navigator_io.button2_changed.connect(self._finish_path_recording)

        print('Starting path registration.\nPress button 0 to add path point.\nPress button 1 to restart')
        while self._path_recording and not rospy.is_shutdown():
            rospy.sleep(1)

        self._navigator_io.button0_changed.disconnect(self._add_path_point)
        self._navigator_io.button1_changed.disconnect(self._reset_path)
        self._navigator_io.button2_changed.disconnect(self._finish_path_recording)

    def playback_path(self):
        rospy.sleep(2)
        self._limb.set_joint_position_speed(self._speed)

        for path_point in self._path:
            if rospy.is_shutdown():
                break
            self._limb.move_to_joint_positions(path_point, timeout=20.0,threshold=self._accuracy)
        rospy.sleep(2)

    def _start_assist(self, value):
        if value:
            rospy.sleep(2)
            self._limb.set_joint_position_speed(self._speed)
    
            for path_point in self._path:
                if rospy.is_shutdown():
                    break
                self._limb.move_to_joint_positions(path_point, timeout=20.0,threshold=self._accuracy)
    
    def _stop_assist(self, value):
        if value:
            self._provide_assist = False

    def provide_assistance(self):
        self._navigator_io.button0_changed.connect(self._start_assist)
        self._navigator_io.button2_changed.connect(self._stop_assist)

        print('Press button 0 to run assistance path. \nPress button 2 to stop the program.')
        while self._provide_assist and not rospy.is_shutdown():
            rospy.sleep(1)
        
        self._navigator_io.button0_changed.disconnect(self._start_assist)
        self._navigator_io.button2_changed.disconnect(self._stop_assist)

    def clean_shutdown(self):
        # Disabling robot if it was not enabled by this script
        if not self._init_state:
            self._rs.disable()

def main():
    # starting a node
    rospy.init_node('STS_Assistance')

    assist = Assist('right')
    rospy.on_shutdown(assist.clean_shutdown)

    assist.record_path()
#    assist.playback_path()
    assist.provide_assistance()

    # Waypoint
if __name__ == "__main__":
    main()
