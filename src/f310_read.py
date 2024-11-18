#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from f310_controller.msg import GoToInfo

import pygame
import time

# Initialize pygame's joystick module
pygame.init()
pygame.joystick.init()

# Check if a joystick is connected
if pygame.joystick.get_count() < 1:
    print("No joystick connected")
    pygame.quit()
    exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick initialized: {joystick.get_name()}")

# Automated forward motion variable
cruise_cntl = False

# Light toggle
led_on = False

# Publish to the "/cmd_vel" topic
motor_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
led_pub = rospy.Publisher('/led_pwm', String, queue_size=2)
cc_pub = rospy.Publisher('/cruise_cntl', String, queue_size=1)

crawler = Twist()

cc = String()
cc.data = "off"
cc_value = 0.0

jog_mode = False
jog_start = False
jog_target = 0.0

def callback(msg):
    global cruise_cntl, cc_value, jog_mode, jog_start, jog_target
    
    if rospy.get_param("/goto_enabled"):
        
        proximity = msg.current_pos - msg.target_pos
        print(f"C pos: {msg.current_pos}, T Pos {msg.target_pos}, Prox: {abs(proximity)}, Thresh: {msg.target_pos_threshold}")

        if abs(proximity) <= msg.target_pos_threshold:
            cruise_cntl = False
            print(cruise_cntl)
            rospy.set_param("/goto_enabled", False)
        
        elif msg.target_pos >= msg.current_pos:
            # Move forward
            cruise_cntl = True
            print("Forward CC On")
            cc_value = rospy.get_param("/cc_speed") / 90

        elif msg.target_pos <= msg.current_pos:
            # Move backward
            cruise_cntl = True
            print("Reverse CC On")
            cc_value = (rospy.get_param("/cc_speed") / 90) * -1

    if jog_mode:
        
        if jog_start == False:
            jog_start = True
            jog_target = msg.current_pos + rospy.get_param("/jog_increment")

        jog_proximity = msg.current_pos - jog_target

        if abs(jog_proximity) <= msg.target_pos_threshold:
            cruise_cntl = False
            jog_mode = False
            jog_start = False
            print("Jog Mode Off")
        
        elif jog_target >= msg.current_pos:
            # Move forward
            cruise_cntl = True
            print("Forward CC On")
            cc_value = rospy.get_param("/cc_speed") / 90

        elif jog_target <= msg.current_pos:
            # Move backward
            cruise_cntl = True
            print("Reverse CC On")
            cc_value = (rospy.get_param("/cc_speed") / 90) * -1

    rospy.sleep(0.1)
        


    # elif rospy.get_param("/goto_enabled") == False:
    #     cruise_cntl = False

    # elif jog_mode:
        
    #     proximity = msg.current_pos - rospy.get_param("/jog_increment")
        
    #     if abs(proximity) <= msg.target_pos_threshold:
    #         pause = True
    #     else:
    #         pause = False

        

def joystick_control():
    global cruise_cntl, led_on, cc_value, jog_mode, jog_start
    
    # Loop to continuously check for events
    try:
        while not rospy.is_shutdown():
            
            if not rospy.get_param("/backup_controller_active") or not rospy.get_param("/omni_controller_active"):
                
                pygame.event.pump()  # Process event queue

                led = String()

                # Buttons
                for i in range(joystick.get_numbuttons()):
                    button = joystick.get_button(i)
                    if button:
                        
                        # if i == 0: # A
                        #     print("Photo")
                        #     crawler.linear.z = 1.0
                        
                        # elif i == 1: # B
                        #     print("B")

                        # if i == 2: # X
                        #     print("X")
                        
                        if i == 3: # Y
                            if led_on:              
                                led_on = False
                                print("Light Off")
                                led = "0,0,0"

                            else:
                                led_on = True
                                print("Light On")
                                led = "150,150,150"
                            
                            led_pub.publish(led)
                            rospy.sleep(0.2)

                        # if i == 4: # Left Button
                        #     print("LB")
                        
                        # if i == 5: # Right Button
                        #     print("RB")

                        # if i == 6: # Back
                        #     print("Back")
                        
                        # if i == 7: # Start
                        #     print("Start")

                        if i == 8: # Logitech
                            if jog_mode:
                                jog_mode = False
                                jog_start = False
                                cruise_cntl = False
                                print("Jog Mode Off")
                                rospy.sleep(0.1)

                            else:
                                jog_mode = True
                                print("Jog Mode On")
                                rospy.sleep(0.1)
                        
                        # if i == 9: # L. Joystick
                        #     print("L. Joystick")

                        # if i == 10: # R. Joystick
                        #     print("R. Joystick")

                # Axes (joysticks)
                for i in range(joystick.get_numaxes()):
                    axis = joystick.get_axis(i)
                    if abs(axis) > 0.1:  # Threshold to ignore small movements
                        
                        if i == 0 and axis < 0.0: # L. Joystick left
                            print(f"Left turn: {axis:.2f}")
                            crawler.angular.z = axis

                        elif i == 0 and axis > 0.0: # L. Joystick Right
                            print(f"Right turn: {axis:.2f}")
                            crawler.angular.z = axis

                        elif i == 1 and axis < 0.0 and not cruise_cntl: # L. Joystick Up
                            print(f"Forward: {axis:.2f}")
                            crawler.linear.x = abs(axis)

                        elif i == 1 and axis > 0.0 and not cruise_cntl: # L. Joystick Down
                            print(f"Reverse: {axis:.2f}")
                            crawler.linear.x = axis * -1

                        # if i == 2: # Left Trigger
                        #     print(f"Left Trigger: {axis:.2f}")

                        # if i == 3 & axis < 0: # R. Joystick left
                        #     print(f"Left turn: {axis:.2f}")

                        # if i == 3 & axis > 0: # R. Joystick Right
                        #     print(f"Right turn: {axis:.2f}")

                        # if i == 4 & axis < 0: # R. Joystick Up
                        #     print(f"Forward: {axis:.2f}")

                        # if i == 4 & axis > 0: # R. Joystick Down
                        #     print(f"Reverse: {axis:.2f}")

                        # if i == 2: # Right Trigger
                        #     print(f"Right Trigger: {axis:.2f}")

                    else:
                        if i == 0:
                            crawler.angular.z = 0.0

                        if i ==1 and not cruise_cntl:
                            crawler.linear.x = 0.0

                # Hats (D-pad)
                for i in range(joystick.get_numhats()):
                    hat = joystick.get_hat(i)
                    if hat != (0, 0):  # Only print if the D-pad is being pressed
                        
                        if hat == (0, 1):
                            if cruise_cntl == True:
                                cruise_cntl = False
                                print("CC Off")
                                rospy.sleep(0.1)

                            else:
                                cruise_cntl = True
                                print("Forward CC On")
                                cc_value = rospy.get_param("/cc_speed") / 90
                                rospy.sleep(0.1)

                        # if hat == (1, 1):
                        #     print("NE")

                        # if hat == (1, 0):
                        #     print("E")

                        # if hat == (1, -1):
                        #     print("SE")

                        if hat == (0, -1):
                            if cruise_cntl == True:
                                cruise_cntl = False
                                print("CC Off")
                                rospy.sleep(0.1)

                            else:
                                cruise_cntl = True
                                print("Reverse CC On")
                                cc_value = (rospy.get_param("/cc_speed") / 90) * -1
                                rospy.sleep(0.1)

                        # if hat == (-1, -1):
                        #     print("SW")

                        # if hat == (-1, 0):
                        #     print("W")

                        # if hat == (-1, 1):
                        #     print("NW")

                if crawler.angular.z != 0.0 and cruise_cntl:
                    crawler.linear.x = 0.0

                elif crawler.angular.z == 0.0 and cruise_cntl:
                    crawler.linear.x = cc_value

                # if pause:
                #     linear_value = crawler.linear.x
                #     crawler.linear.x = 0.0
                #     motor_pub.publish(crawler)
                #     rospy.sleep(3) 

                #     crawler.linear.x = linear_value
                #     motor_pub.publish(crawler)
                #     rospy.sleep(1)  # Sleep to reduce CPU usage
                    

                #     current_value = rospy.get_param("/jog_increment")
                #     rospy.set_param("/jog_increment", (current_value + initial_value))
                #     print(rospy.get_param("/jog_increment"))

                #     pause = False
                    
                motor_pub.publish(crawler)
                rospy.sleep(0.1)  # Sleep to reduce CPU usage

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()

def main():
    rospy.init_node('f310_input_node')

    rospy.Subscriber('/goto_status', GoToInfo, callback, queue_size=1)

    joystick_control()


if __name__ == "__main__":
    main()
