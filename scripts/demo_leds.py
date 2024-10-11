# ------------------------------------------------------------
# Файл содержит пример кода взаимодействия с пьезодинамиком и 
# светодиодом кнопки включения
# Этот пример не будет работать в симуляторе
# Запуск: rosrun mors_demo demo_leds.py
# ------------------------------------------------------------

import rospy
import numpy as np
from mors.msg import HMIBeeper, HMILed

def set_led(interface, r=10, g=0, b=0, dur=0, freq=0):
    global led_msg
    led_msg.r = r
    led_msg.g = g
    led_msg.b = b
    led_msg.duration = dur
    led_msg.frequency = freq
    led_msg.interface = interface
    led_pub.publish(led_msg)

def set_beep(duration, frequency):
    beep_msg = HMIBeeper()
    beep_msg.duration = duration
    beep_msg.frequency = frequency
    beep_pub.publish(beep_msg)

if __name__ == '__main__':
    # инициализация ROS
    rospy.init_node("mors_demo")
    rate = rospy.Rate(100)
    t = 0.0
    dt = 1.0/100.0

    led_pub = rospy.Publisher('hmi/led', HMILed, queue_size=1)
    beep_pub = rospy.Publisher('hmi/beeper', HMIBeeper, queue_size=10)
    led_msg = HMILed()
    beep_msg = HMIBeeper()

    # длительность горения каждого цвета составляет 3 сек
    led_sequence = [[10, 0,  0,  3, 0], # красный, горит постоянно
                    [0,  10, 0,  3, 0], # зеленый, горит постоянно
                    [0,  0,  10, 3, 0], # синий, горит постоянно
                    [10,  0,  0, 3, 1], # красный, моргает
                    [10,  0,  0, 3, 3], # красный, моргает быстрее
                    [10,  0,  0, 3, 9]] # красный, моргает совсем быстро
    
    # длительность каждого звукового сигнала составляет 2 сек
    # за количество подаваемых сигналов отвечает второй параметр
    piezo_sequence = [[2, 1],
                      [2, 2],
                      [2, 4],
                      [2, 8],
                      [2, 12],
                      [2, 16],]
    i = 0

 
    rospy.loginfo("Demo LEDs: Start")

    while not rospy.is_shutdown():
        if t%3 < 0.01:
            if i > 5:
                break
            set_beep(piezo_sequence[i][0], piezo_sequence[i][1])
            set_led(0, led_sequence[i][0], led_sequence[i][1], led_sequence[i][2], led_sequence[i][3], led_sequence[i][4])
            i += 1

        # добавляем инкримент ко времени
        t += dt
        # ждем следующей итерации
        rate.sleep()

    rospy.loginfo("Demo LEDs: Stop")