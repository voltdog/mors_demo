import lcm
import rospy
import numpy as np
from lcm_msgs.servo_cmd_msg import servo_cmd_msg
from lcm_msgs.servo_state_msg import servo_state_msg


def servo_state_callback(channel , data):
    state_msg = servo_state_msg.decode(data)
    servo_pos = state_msg.position
    print("---------------------------------------")
    print(f"R1: sh={servo_pos[0]:.2f} | hip={servo_pos[1]:.2f} | knee={servo_pos[2]:.2f}")
    print(f"R2: sh={servo_pos[3]:.2f} | hip={servo_pos[4]:.2f} | knee={servo_pos[5]:.2f}")
    print(f"L1: sh={servo_pos[6]:.2f} | hip={servo_pos[7]:.2f} | knee={servo_pos[8]:.2f}")
    print(f"L2: sh={servo_pos[9]:.2f} | hip={servo_pos[10]:.2f} | knee={servo_pos[11]:.2f}")
    print("---------------------------------------")
    print(f"\033[7A") # спецсимвол \033[1A используется для возврата на одну строку выше 


if __name__ == '__main__':
    # инициализация ROS
    rospy.init_node("mors_demo")
    rate = rospy.Rate(100)
    t = 0.0
    dt = 1.0/100.0

    # инициализация LCM
    cmd_msg = servo_cmd_msg()
    lc_cmd = lcm.LCM()
    lc_state = lcm.LCM()
    subscription = lc_state.subscribe("SERVO_STATE", servo_state_callback)

    # начинаем
    rospy.loginfo("Demo LCM: Start")

    cmd_msg.position = [0.0]*12
    cmd_msg.velocity = [0.0]*12
    cmd_msg.torque = [0.0]*12
    cmd_msg.kp = [0.0]*12
    cmd_msg.kd = [0.0]*12

    # инициализируем коэффициенты обратной связи приводов
    rospy.sleep(1.5)
    cmd_msg.kp = [0.2]*12
    cmd_msg.kd = [0.05]*12
    lc_cmd.publish("SERVO_CMD", cmd_msg.encode())
    rospy.sleep(1.5)
    cmd_msg.kp = [2.0]*12
    cmd_msg.kd = [0.2]*12
    lc_cmd.publish("SERVO_CMD", cmd_msg.encode())
    rospy.sleep(1.5)
    cmd_msg.kp = [16.0]*12
    cmd_msg.kd = [0.4]*12
    lc_cmd.publish("SERVO_CMD", cmd_msg.encode())
    rospy.sleep(1.5)

    while not rospy.is_shutdown():
        # формируем вектор желаемых углов
        theta1 = 0.4*np.sin(1.0*2*np.pi*t)
        theta2 = 1.0*np.sin(1.0*2*np.pi*t)
        theta3 = 1.57*np.sin(1.0*2*np.pi*t)

        cmd_msg.position = [theta1, theta2, theta3,
                            theta1, theta2, theta3,
                            theta1, theta2, theta3,
                            theta1, theta2, theta3]
        # прерываем цикл
        if t > 20.0:
            break
        
        # отправляем данные по LCM
        lc_cmd.publish("SERVO_CMD", cmd_msg.encode())
        # проверяем наличие данных во входящем LCM-канале
        lc_state.handle()
        # добавляем инкримент ко времени
        t += dt
        # ждем следующей итерации
        rate.sleep()

    # уменьшаем Kp и Kd
    rospy.sleep(1.5)
    cmd_msg.kp = [0.2]*12
    cmd_msg.kd = [0.0]*12
    lc_cmd.publish("SERVO_CMD", cmd_msg.encode())
    rospy.sleep(1.5)
    cmd_msg.kp = [0.0]*12
    cmd_msg.kd = [0.0]*12
    lc_cmd.publish("SERVO_CMD", cmd_msg.encode())
    print(f"\033[7B")
    rospy.loginfo("Demo LCM: Stop")


    