#!/usr/bin/env python3

import rospy
import roslaunch
import _thread

class LuanchLuachFile():
    def __init__(self, package, executable, parameters=None):
        self.isRunning = False
        self.package = package
        self.executable = executable
        self.parameters = parameters

    def start(self):
        if self.isRunning == False:
            try:
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                if self.parameters:
                    cli_args1 = [self.package, self.executable, self.parameters]
                    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
                    roslaunch_args1 = cli_args1[2:]
                    launch_files = [(roslaunch_file1, roslaunch_args1)]
                else:
                    cli_args1 = [self.package, self.executable]
                    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
                    launch_files = [roslaunch_file1]

                self.process = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

                self.process.start()
                self.isRunning = True

            except Exception as e:
                print(e)
                print('Failed to start', self.executable)
        else:
            print(self.executable, ' is allready running')

    def checkState(self):
        return self.isRunning

    ''' # Doesn't seem stable to shutdown luanch file, will be killed on exit
    def stop(self):
        if self.isRunning == True:
            try:
                self.process.shutdown()
                self.isRunning = False
            except Exception as e:
                print(e)
                print('Failed to stop ', self.executable)
        else:
            print('Faild to stop ', self.executable, ' as it is not running' )
    '''


class LuanchNode():
    def __init__(self, package, executable):
        self.isRunning = False
        self.package = package
        self.executable = executable

    def start(self):
        if self.isRunning == False:
            try:
                node = roslaunch.core.Node(package=self.package, node_type=self.executable, output='screen')

                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()

                self.Node = launch.launch(node)
                self.isRunning = True
            except Exception as e:
                print(e)
                print('Failed to start', self.executable)
        else:
            print(self.executable, ' is allready running')

    def checkState(self):
        try:
            self.isRunning = self.Node.is_alive()
            return self.isRunning
        except: 
            return False

    def stop(self):
        if self.isRunning == True:
            try:
                self.Node.stop()
            except Exception as e:
                print(e)
                print('Failed to stop ', self.executable)
        else:
            print('Faild to stop ', self.executable, ' as it is not running' )


class States():
    def __init__(self):
        self.rvizRunning = False
        self.kdlKinematicsRunning = False
        self.tfTreeRunning = False
        self.contollerRunning = False
        self.resetPoseRunning = False
        self.simulationRunning = False
        self.yumiRunning = False
        self.egmRunning = False
        
        self.Rviz = LuanchLuachFile('yumi_description', 'display.launch')
        self.Rviz.start()
        self.tf = LuanchNode('robot_setup_tf', 'tf_broadcaster')
        self.tf.start()
        self.kinematics = LuanchNode('controller', 'kdl_kinematics')
        self.kinematics.start()

        self.controller = LuanchNode('controller', 'controllerMaster.py')
        self.resetPose = LuanchNode('controller', 'initialPoseJointController.py')
        self.simulation = LuanchNode('simulation_rviz', 'yumi_simulator.py')
        self.EGM = LuanchNode('robot_setup_tf', 'set_yumi_settings_and_start.py')
        self.connect = LuanchLuachFile('abb_robot_bringup_examples', 'ex3_rws_and_egm_yumi_robot.launch', 'robot_ip:=192.168.125.1')

    def keyBoard(self):
        while not rospy.is_shutdown():    
            input_ = input()
            if input_ == 'shutDown':
                print('Shuting down ...')
                self.stop()
                rospy.sleep(2)
                rospy.signal_shutdown("done")

            elif input_ == 'stopController':
                print('stopping contoller')
                self.controller.stop()

            elif input_ == 'startController':
                print('Starting contoller')
                if self.resetPoseRunning == False:
                    self.controller.start()
                else:
                    print('Reset pose already running, wait for it to finish')

            elif input_ == 'resetPose':
                if self.egmRunning or self.simulationRunning:
                    if self.contollerRunning == False:
                        print('Resetting pose')
                        self.resetPose.start() 
                    else:
                        print('Controller running, stop controller before reseting pose!')
                else:
                    print('Simulaiton or EGM must be active!')

            elif input_ == 'startSimulation':
                if not self.egmRunning:
                    print("Starting simulation")
                    self.simulation.start()
                else: 
                    print('Do not use simulation when robot has EGM active!')

            elif input_ == 'stopSimulation':
                print("Stopping simulation")
                self.simulation.stop()

            elif input_ == 'connectYumi':
                try:
                    print("Connecting to YuMi")    
                    self.connect.start()
                except:
                    print('Failed to connect to YuMi')

            elif input_ == 'startEGM':
                if self.yumiRunning:
                    print("starting EGM")
                    self.EGM.start()
                else:
                    print('Must be connectied to YuMi before starting EGM')
            
            elif input_ == 'stopEGM':
                print("stopEGM")
                self.EGM.stop()
            elif input_ == '':
                self.update(True)
            else:
                print('Not Vaild command!', input_)
                print('Enter command:')

    def update(self, forcePrint=False):
        if self.checkDifference() or forcePrint:
            print()
            print()
            print()
            print()
            print()
            print('-----------------------------------------------------------------------')
            print('YuMi, nodes that are running:')
            print('    Rviz: ', self.rvizRunning, ', Kinematics: ', self.kdlKinematicsRunning, ', Tf broadcaster: ', self.tfTreeRunning)
            print('    Contoller: ', self.contollerRunning, ', Reset pose: ', self.resetPoseRunning)
            print('    Simulation: ', self.simulationRunning, ', Yumi connection: ', self.yumiRunning, ', EGM: ', self.egmRunning)
            print('Vaild commands:')
            print('    "shutDown", "startController", "stopController", "resetPose",...')
            print('    "startSimulation", "stopSimulation", "connectYumi", "startEGM",..')
            print('    "stopEGM"')
            print('Enter command:')


    def checkDifference(self):
        rvizRunning = self.Rviz.checkState()
        kdlKinematicsRunning =  self.kinematics.checkState()
        tfTreeRunning = self.tf.checkState()
        contollerRunning = self.controller.checkState()
        simulationRunning = self.simulation.checkState()
        resetPoseRunning = self.resetPose.checkState()
        yumiRunning = self.connect.checkState()
        egmRunning = self.EGM.checkState()

        change = 0
        if rvizRunning != self.rvizRunning:
            self.rvizRunning = rvizRunning
            change = 1
        if kdlKinematicsRunning != self.kdlKinematicsRunning:
            self.kdlKinematicsRunning = kdlKinematicsRunning
            change = 1
        if tfTreeRunning != self.tfTreeRunning:
            self.tfTreeRunning = tfTreeRunning
            change = 1
        if contollerRunning != self.contollerRunning:
            self.contollerRunning = contollerRunning
            change = 1
        if simulationRunning != self.simulationRunning:
            self.simulationRunning = simulationRunning
            change = 1
        if resetPoseRunning != self.resetPoseRunning:
            self.resetPoseRunning = resetPoseRunning
            change = 1
        if yumiRunning != self.yumiRunning:
            self.yumiRunning = yumiRunning
            change = 1
        if egmRunning != self.egmRunning:
            self.egmRunning = egmRunning
            change = 1

        if change:
            return True
        else:
            return False


    def stop(self):
        try:
            #self.Rviz.stop()
            self.tf.stop()
            self.kinematics.stop()
            self.controller.stop()
            self.resetPose.stop()
            self.simulation.stop()
            self.EGM.stop()
            #self.connect.stop()
            rospy.sleep(2)
        except:
            print("ShutDown fail")


if __name__ == '__main__':
    rospy.init_node('Interface', anonymous=True)
    runningNodes = []
    states = States()

    try:
        _thread.start_new_thread(states.keyBoard, ())
    except:
        print('Failed to create thread')

    while not rospy.is_shutdown():    
        states.update()
        rospy.sleep(1)

    rospy.spin()