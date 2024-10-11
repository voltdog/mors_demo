# ------------------------------------------------------------
# Файл содержит пример кода управления двигателями
# шагающего робота МОРС. Во время демонстрационного примера 
# робот вращает суставами по синусоидальной траектории с 
# разными амплитудами для каждого сустава. 
# Обязательно поставьте робота на стойку перед тем, как 
# запускать это пример. Будьте аккуратны, так как робот
# достаточно сильно размахивает ногами.
# Если вы запускаете этот пример на симуляторе, то лучше в config-
# файле пакета mors_sim указать параметр on_rack: True
# Запуск: rosrun mors_demo demo_joints.py
# ------------------------------------------------------------

import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mors.srv import QuadrupedCmd, JointsCmd

# call robot_mode service
def set_mode_client(mode):
    rospy.wait_for_service('robot_mode')
    try:
        set_mode = rospy.ServiceProxy('robot_mode', QuadrupedCmd)
        resp = set_mode(mode)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# call joints_kpkd service
def set_joints_kp(kp):
    rospy.wait_for_service("joints_kp")
    try:
        set_kp_srv = rospy.ServiceProxy('joints_kp', JointsCmd)
        resp = set_kp_srv(kp)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def set_joints_kd(kd):
    rospy.wait_for_service("joints_kd")
    try:
        set_kd_srv = rospy.ServiceProxy('joints_kd', JointsCmd)
        resp = set_kd_srv(kd)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# call robot_action service
def set_action_client(action):
    rospy.wait_for_service('robot_action')
    try:
        set_action = rospy.ServiceProxy('robot_action', QuadrupedCmd)
        resp = set_action(action)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    # инициализация ROS
    rospy.init_node("mors_demo")
    rate = rospy.Rate(100)
    t = 0.0
    dt = 1.0/100.0

    cmd_joint_pos_pub = rospy.Publisher("/head/joint_group_position_controller/command", JointTrajectoryPoint, queue_size=10)
    status_pub = rospy.Publisher("/head/status", Bool, queue_size=10)
    cmd_joint_msg = JointTrajectoryPoint()
    cmd_joint_msg.velocities = [0]*12
    cmd_joint_msg.effort = [0]*12
    cmd_joint_msg.positions = [0]*12
 
    rospy.loginfo("Demo Joints: Start")

    # обнуляем коэффициенты обратной связи для моторов
    set_joints_kp([0.0]*12)
    set_joints_kd([0.0]*12)

    # устанавливаем режим управления двигателями
    set_mode_client(3)

    # постепенно увеличиваем коэффициенты обратной связи для моторов,
    # чтобы выпрямить ноги
    rospy.sleep(0.2)
    set_joints_kp([0.2]*12)
    set_joints_kd([0.2]*12)
    rospy.sleep(1.5)
    set_joints_kp([2.0]*12)
    set_joints_kd([0.2]*12)
    rospy.sleep(1.5)
    set_joints_kp([16.0]*12)
    set_joints_kd([0.4]*12)
    rospy.sleep(1.5)

    while not rospy.is_shutdown():

        # формируем вектор желаемых углов
        theta1 = 0.4*np.sin(0.5*2*np.pi*t)
        theta2 = 1.0*np.sin(0.5*2*np.pi*t)
        theta3 = 1.57*np.sin(0.5*2*np.pi*t)

        cmd_joint_msg.positions = [theta1, theta2, theta3,
                                   theta1, theta2, theta3,
                                   theta1, theta2, theta3,
                                   theta1, theta2, theta3]

        # прерываем цикл
        if t > 20.0:
            break
        
        # публикуем вектор желаемых углов
        cmd_joint_pos_pub.publish(cmd_joint_msg)
        # информируем cmd_commutator, что нода включена
        status_pub.publish(True)
        # добавляем инкримент ко времени
        t += dt
        # ждем следующей итерации
        rate.sleep()

    # информируем cmd_commutator, что нода выключена
    status_pub.publish(False)
    # обнуляем коэффициенты
    set_joints_kp([0.0]*12)
    set_joints_kd([0.2]*12)
    rospy.sleep(2.0)
    set_joints_kp([0.0]*12)
    set_joints_kd([0.0]*12)
    rospy.loginfo("Demo Joints: Stop")