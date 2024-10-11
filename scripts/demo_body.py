# ------------------------------------------------------------
# Файл содержит пример кода управления корпусом
# шагающего робота МОРС
# Запуск: rosrun mors_demo demo_body.py
# ------------------------------------------------------------

import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

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

    cmd_pose_pub = rospy.Publisher("/head/cmd_pose", Twist, queue_size=10)
    status_pub = rospy.Publisher("/head/status", Bool, queue_size=10)
    cmd_pose_msg = Twist()
 
    rospy.loginfo("Demo Body: Start")

    # встаем
    set_action_client(1)

    # устанавливаем режим управления корпусом
    set_mode_client(2)

    # обнуляем вектор желаемого положения
    cmd_pose_msg.linear.x = 0.0
    cmd_pose_msg.linear.y = 0
    cmd_pose_msg.linear.z = 0
    cmd_pose_msg.angular.x = 0
    cmd_pose_msg.angular.y = 0
    cmd_pose_msg.angular.z = 0
    phi = 0.0

    while not rospy.is_shutdown():
        # t=1сек: меняем положение корпуса вдоль вертикальной оси
        if t < 6.0:
            cmd_pose_msg.linear.z = 0.05*np.sin(0.5*2*np.pi*t)
        # t=6сек: вращаем корпус по рысканью
        elif 6.0 < t < 12.0:
            cmd_pose_msg.angular.z = 0.4*np.sin(0.5*2*np.pi*t)
        # t=12сек: вращаем корпус по крену и тангажу
        elif 12.0 < t < 18.0:
            if t > 17.0 and phi > 0.0:
                phi -= dt
            if t < 13.0 and phi < np.pi/2:
                phi += dt
            cmd_pose_msg.angular.x = 0.4*np.sin(0.5*2*np.pi*t)
            cmd_pose_msg.angular.y = 0.3*np.sin(0.5*2*np.pi*t + phi)
        # t=18сек: перемещаем корпус по овалу в плоскости XY
        elif 18.0 < t < 28.0:
            if t < 19.0 and phi < np.pi:
                phi += dt
            if t > 27.0 and phi > 0.0:
                phi -= dt
            cmd_pose_msg.linear.x = 0.05*np.sin(0.5*2*np.pi*t)
            cmd_pose_msg.linear.y = 0.03*np.sin(0.5*2*np.pi*t + phi)
        # прерываем цикл
        else:
            break
        
        # публикуем вектор желаемых скоростей
        cmd_pose_pub.publish(cmd_pose_msg)
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
    rospy.loginfo("Demo Body: Stop")