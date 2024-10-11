# ------------------------------------------------------------
# Файл содержит пример кода управления стопами
# шагающего робота МОРС. Во время демонстрационного примера 
# робот двигает стопами по окружности. Каждая нога 
# отрабатывает движение в своей плоскости. Поставьте робота
# на стойку перед тем, как запускать пример.
# Если вы запускаете этот пример на симуляторе, то лучше в config-
# файле пакета mors_sim указать параметр on_rack: True
# Запуск: rosrun mors_demo demo_legs.py
# ------------------------------------------------------------

import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose

from mors.srv import QuadrupedCmd

# call robot_mode service
def set_mode_client(mode):
    rospy.wait_for_service('robot_mode')
    try:
        set_mode = rospy.ServiceProxy('robot_mode', QuadrupedCmd)
        resp = set_mode(mode)
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

    cmd_pose_pub = rospy.Publisher("/head/ef_position/command", PoseArray, queue_size=10)
    status_pub = rospy.Publisher("/head/status", Bool, queue_size=10)
    cmd_ef_msg = PoseArray()
    cmd_one_ef_msg = Pose()
    for _ in range(4):
        cmd_ef_msg.poses.append(cmd_one_ef_msg)
 
    rospy.loginfo("Demo Legs: Start")

    # встаем
    set_action_client(1)

    # устанавливаем режим управления ногами
    set_mode_client(1)
    
    phi = 0.0

    while not rospy.is_shutdown():

        # постепенно увеличиваем фазовый сдвиг в начале
        if t < 1.0 and phi < np.pi:
            phi += dt
        # постепенно уменьшаем фазовый сдвиг в конце
        if t > 19.0 and phi > 0.0:
            phi -= dt
        
        # LEG R1
        r1_msg = Pose()
        r1_msg.position.x = 0.07 * np.sin(0.5*2*np.pi*t)
        r1_msg.position.y = 0.07 * np.sin(0.5*2*np.pi*t + phi)
        cmd_ef_msg.poses[0] = r1_msg

        # LEG L1
        l1_msg = Pose()
        l1_msg.position.x = 0.06 * np.sin(0.5*2*np.pi*t)
        l1_msg.position.z = 0.06 * np.sin(0.5*2*np.pi*t + phi)
        cmd_ef_msg.poses[1] = l1_msg

        # LEG R2
        r2_msg = Pose()
        r2_msg.position.y = 0.06 * np.sin(0.5*2*np.pi*t)
        r2_msg.position.z = 0.06 * np.sin(0.5*2*np.pi*t + phi)
        cmd_ef_msg.poses[2] = r2_msg

        # LEG L2
        l2_msg = Pose()
        l2_msg.position.x = 0.06 * np.sin(0.5*2*np.pi*t)
        l2_msg.position.y = 0.06 * np.sin(0.5*2*np.pi*t)
        l2_msg.position.z = 0.06 * np.sin(0.5*2*np.pi*t + phi)
        cmd_ef_msg.poses[3] = l2_msg

        # прерываем цикл
        if t > 20.0:
            break
        
        # публикуем вектор желаемых скоростей
        cmd_pose_pub.publish(cmd_ef_msg)
        # информируем cmd_commutator, что нода включена
        status_pub.publish(True)
        # добавляем инкримент ко времени
        t += dt
        # ждем следующей итерации
        rate.sleep()

    # информируем cmd_commutator, что нода выключена
    status_pub.publish(False)
    # ложимся и готовимся к выключению
    set_action_client(2)
    rospy.loginfo("Demo Legs: Stop")