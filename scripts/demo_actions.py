# ------------------------------------------------------------
# Файл содержит пример кода для вызова скриптов действий  
# шагающим роботом МОРС
# Внимание! При включении этого примера, убедитесь, что слева 
#           от робота есть 1.5 м свободного пространства
# Запуск: rosrun mors_demo demo_actions.py
# ------------------------------------------------------------

import rospy
from mors.srv import QuadrupedCmd

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
    # инициализируем ноду
    rospy.init_node("demo_actions")

    rospy.loginfo("Demo Actions: Start")
    # Встать
    set_action_client(1)
    # Дай лапу
    set_action_client(3)
    # Дай другую лапу
    set_action_client(7)
    # Кувырок
    set_action_client(4)
    # Помахай лапой
    set_action_client(5)
    # Сидеть
    set_action_client(6)
    # Лечь на пол и снять питание с двигателей
    set_action_client(2)

    rospy.loginfo("Demo Actions: Finish")
