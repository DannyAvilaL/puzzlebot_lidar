#!/usr/bin/env python
import rospy
import actionlib
from bloque.msg import DoCarWashAction,DoCarWashGoal

def callback(msg):
    print('Feedback received -> ' + str(msg)+'%')

def node():
    rospy.init_node('nodo_action_client')
    client = actionlib.SimpleActionClient('do_wash_car', DoCarWashAction)
    client.wait_for_server()
    goal = DoCarWashGoal()
    goal.number_of_cars = 5
    client.send_goal(goal, feedback_cb=callback)
    client.wait_for_result()
    result = client.get_result()
    print("The result is: ", result)

if __name__ == "__main__":
    try:
        node()
    except ROSInterruptException:
        pass