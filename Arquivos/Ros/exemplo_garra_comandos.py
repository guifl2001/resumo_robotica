import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64

if __name__ == "__main__":
    rospy.init_node("garra")
    ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
    garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

    try:
        while not rospy.is_shutdown():   #Ombro para frente  =0 ; Ombro recolhido = -1.0; Ombro levantado = 1.5;
            ombro.publish(-1.0)
            garra.publish(0.0)            #Garra(pinca) fechada = 0; Garra(pinca) aberta = -1.0;
            rospy.sleep(3.0)
            ombro.publish(1.5)
            garra.publish(-1.0) 
            rospy.sleep(3.0)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")