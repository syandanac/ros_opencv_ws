import subprocess
import time
import rospy

def start_opencv_node(workspace_path, package_name, node_name):
    # Command to source the correct ROS workspace and launch the node
    command = f"cd ~/{workspace_path} && source devel/setup.bash && rosrun {package_name} {node_name}"
    
    # Run the command using subprocess
    opencv_process = subprocess.Popen(command, shell=True, executable='/bin/bash')
    return opencv_process

def stop_opencv_node(opencv_process):
    # Terminate the process
    opencv_process.terminate()

def wait_for_node_to_start(node_name, timeout=3):
    # Wait until the node is up and running
    start_time = time.time()
    while time.time() - start_time < timeout:
        nodes = subprocess.check_output(['rosnode', 'list']).decode('utf-8').split('\n')
        if node_name in nodes:
            rospy.loginfo(f"Node {node_name} is running")
            return True
        time.sleep(0.5)
    rospy.logwarn(f"Node {node_name} failed to start within {timeout} seconds")
    return False