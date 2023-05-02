#!/usr/bin/env python
import rospy
import actionlib
from bloque.msg import DoCarWashAction,DoCarWashFeedback,DoCarWashResult

def execute(goal): #funcion que se ejecuta cuando se arranca la accion. Recibe variables de clase Goal
    print("Working on a client request ...")
    feedback = DoCarWashFeedback() #se crea una variable de clase Feedback
    result = DoCarWashResult() # se crea una variable de la clase Result
    rate = rospy.Rate(1)

    for x in range(0, goal.number_of_cars):
        result.total_cars_cleaned += 1
        feedback.percent_cars_complete = (result.total_cars_cleaned*100.0) / goal.number_of_cars
        server.publish_feedback(feedback) #publica el avance
        rate.sleep()
    
    server.set_succeeded(result) # Cuando termina publica el resultado final
    print("Done...")

if __name__ == "__main__":
    try:
        #nombre del nodo que lanza el servidor accion
        rospy.init_node('nodo_action_server')
        #                  (nombre de la accion, tipo de accion, funcion que tiene la accion, arranque automatico)
        server = actionlib.SimpleActionServer('do_wash_car', DoCarWashAction, execute, False)
        server.start() #arranca el servidor-accion
        print("Running action server do_wash_car")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass