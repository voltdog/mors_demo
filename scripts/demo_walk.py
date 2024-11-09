# ------------------------------------------------------------
# Файл содержит пример кода управления ходьбой
# шагающего робота МОРС
# Внимание! При включении этого примера, убедитесь, что спереди
#           у робота есть 3 метра свободного пространства
# Запуск: rosrun mors_demo demo_walk.py
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

def set_stride_height_client(height):
    rospy.wait_for_service('stride_height')
    try:
        set_height = rospy.ServiceProxy('stride_height', QuadrupedCmd)
        resp = set_height(height)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    # инициализация ROS
    rospy.init_node("mors_demo")

    rate = rospy.Rate(300)
    t = 0.0
    dt = 1.0/300.0

    cmd_vel_pub = rospy.Publisher("/head/cmd_vel", Twist, queue_size=10)
    status_pub = rospy.Publisher("/head/status", Bool, queue_size=10)

    cmd_vel_msg = Twist()
 
    rospy.loginfo("Demo Walk: Start")

    # устанавливаем режим ходьбы
    set_mode_client(0)

    # встаем
    set_action_client(1)

    # делаем высоту шага 4 см
    set_stride_height_client(0.04)

    # обнуляем вектор желаемой скорости
    cmd_vel_msg.linear.x = 0.0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.angular.z = 0

    while not rospy.is_shutdown():
        # t=1сек: идем вперед со скоростью 0.4
        if np.abs(t-1) < 0.003:
            cmd_vel_msg.linear.x = 0.4
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.angular.z = 0
        # t=3сек: идем назад со скоростью 0.4
        elif np.abs(t-3) < 0.003:
            cmd_vel_msg.linear.x = -0.4
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.angular.z = 0
        # t=5сек: идем влево со скоростью 0.2
        elif np.abs(t-5) < 0.003:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.2
            cmd_vel_msg.angular.z = 0
        # t=7сек: идем вправо и назад со скоростью 0.2 и 0.4 соответственно
        elif np.abs(t-7) < 0.003:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = -0.2
            cmd_vel_msg.angular.z = -0.4
        # t=9сек: поворачиваем налево с радиусом поворота 0.7 метра
        elif np.abs(t-9) < 0.003:
            cmd_vel_msg.linear.x = 0.3
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = 0.3
        # t=10сек: поворачиваем направо с радиусом поворота 0.2 метра
        elif np.abs(t-11) < 0.003:
            cmd_vel_msg.linear.x = 0.3
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = -0.8
        # t=13сек: прерываем цикл
        elif np.abs(t-13) < 0.003:
            break
        
        # публикуем вектор желаемых скоростей
        cmd_vel_pub.publish(cmd_vel_msg)
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
    rospy.loginfo("Demo Stop: Start")