# ------------------------------------------------------------
# Файл содержит пример кода для получения данных с датчиков 
# робота МОРС. Во время работы данные выводятся в консоль. 
# Вы можете управлять роботом в любом режиме во время работы 
# с этой нодой.
# Данные о напряжении, токе, температуре и состоянии кнопки
# не будут отображаться при работе с симулятором.
# Запуск: rosrun mors_demo demo_sensors.py
# ------------------------------------------------------------

import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState, Imu, Temperature, JointState
from mors.msg import PowerButtons

freq = 40

servo_pos = [0]*12
orientation = [0]*4
acceleration = [0]*3
temperature = 0.0
user_btn = False
voltage = 0.0
current = 0.0

# колбэк для датчиков приводов
def servo_state_callback(msg : JointState):
    global servo_pos
    servo_pos = msg.position

# колбэк для данных с IMU
def imu_callback(msg : Imu):
    global orientation
    global acceleration
    orientation[0] = msg.orientation.x
    orientation[1] = msg.orientation.y
    orientation[2] = msg.orientation.z
    orientation[3] = msg.orientation.w
    acceleration[0] = msg.linear_acceleration.x
    acceleration[1] = msg.linear_acceleration.y
    acceleration[2] = msg.linear_acceleration.z

# колбэк для температуры
def temperature_callback(msg : Temperature):
    global temperature
    temperature = msg.temperature

# данные с аккумулятора
def battery_callback(msg : BatteryState):
    global voltage
    global current
    voltage = msg.voltage
    current = -msg.current

# информация о кнопках
def buttons_callback(msg : PowerButtons):
    global user_btn
    user_btn = msg.user

if __name__ == '__main__':
    # инициализация ROS
    rospy.init_node("mors_demo")
    rate = rospy.Rate(freq)
    t = 0.0
    dt = 1.0/freq

    rospy.Subscriber("joint_states", JointState, servo_state_callback, queue_size=1)
    rospy.Subscriber("imu/data", Imu, imu_callback, queue_size=1)
    rospy.Subscriber("imu/temp", Temperature, temperature_callback, queue_size=1)
    rospy.Subscriber("bat", BatteryState, battery_callback, queue_size=1)
    rospy.Subscriber("power/buttons", PowerButtons, buttons_callback, queue_size=1)

    rospy.loginfo("Demo Sensors: Start")

    while not rospy.is_shutdown():
        print("---------------------------------------")
        print(f"t={t:.2f} | voltage={voltage:.2f} | current={current:.2f} | temperature={temperature:.2f} | button={user_btn} |")
        print(f"orientation: x={orientation[0]:.2f} | y={orientation[1]:.2f} | z={orientation[2]:.2f} | w={orientation[3]:.2f}")
        print(f"acceleration: x={acceleration[0]:.2f} | y={acceleration[1]:.2f} | z={acceleration[2]:.2f}")
        print(f"R1: sh={servo_pos[0]:.2f} | hip={servo_pos[1]:.2f} | knee={servo_pos[2]:.2f}")
        print(f"R2: sh={servo_pos[3]:.2f} | hip={servo_pos[4]:.2f} | knee={servo_pos[5]:.2f}")
        print(f"L1: sh={servo_pos[6]:.2f} | hip={servo_pos[7]:.2f} | knee={servo_pos[8]:.2f}")
        print(f"L2: sh={servo_pos[9]:.2f} | hip={servo_pos[10]:.2f} | knee={servo_pos[11]:.2f}")
        print("---------------------------------------", end="")
        print(f"\033[9A") # спецсимвол \033[1A используется для возврата на одну строку выше 
        
        # добавляем инкримент ко времени
        t += dt
        # ждем следующей итерации
        rate.sleep()

    print("\033[9B")
    rospy.loginfo("Demo Sensors: Stop")