import paho.mqtt.client as mqtt
import time
from pycreate2 import Create2
import numpy as np
from random import choice
import keyboard

#-----------------------------------------------------------------
#THIS SCRIPT CONTROLS THE ENEMY ROOMBA USING COMPUTER INPUT
#-----------------------------------------------------------------


def read_file(filename):
    new_data = np.loadtxt(filename)
    new_data = new_data.reshape((121, 121, 5))
    return new_data


def on_connect(client,userdata,flags,rc):

    print("Connected with result code" + str(rc))

def on_message(client, userdata, msg):

    value = msg.payload.decode('utf-8')

    print(str(value))


    if(msg.topic == '/roomba2/state_friendly'):

        global state_enemy
        global state_friendly
        try:

            if (value == "start"):

                state_enemy = 39
                state_friendly = 6

            else:

                states_list = value.split(",")
                state_enemy = int(states_list[0])
                state_friendly = int(states_list[1])
                print("State friendly has been updated {0}".format(state_friendly))
                print("I also received my own position which is: {0}".format(state_enemy))


        except Exception as ex:
            print(ex)


    if(msg.topic == "/roomba2/turn_enemy"):

        if(value == "0"):

            print("hmmmmmmm I really want to make my move but it's the friendly's turn...")

        else:
            print("my move, asking human for help...")
            legal_actions = get_legal_actions(state_enemy)
            client.publish("/roomba2/computer_control", str(legal_actions))


    if(msg.topic == "/roomba2/computer_action"):
        action = value
        print("human chose: {0}".format(action))


        print("making my move")
        print(state_friendly)
        print("Making my move with the updated state of anakin {0}".format(state_friendly))
        step(state_friendly, state_enemy, action)
        print("made my move")




    if(msg.topic == "/roomba2/victory_friendly"):

        print("fuck I lost, better luck next game I guess.. Reset incoming!")
        reset()


    else:
        pass

def get_legal_actions(state):

    if (state == 0):
        actions = ["F", "R", "S"]
        return actions

    elif(state == 10):
        actions = ["F", "L", "S"]

    elif(state == 110):
        actions = ["B", "R", "S"]

    elif(state == 121):
        actions = ["B","L","S"]

    elif(state == 11 or state == 22 or state == 33 or state == 44 or state == 55 or state == 66 or state == 77 or state == 88 or state ==99):
        actions = ["F", "B", "R", "S"]

    elif(state == 21 or state == 32 or state == 43 or state == 54 or state == 65 or state == 76 or state == 87 or state == 98 or state == 109):
        actions = ["F", "B", "L","S"]

    else:
        actions = ["F", "B", "L", "R", "S"]

    return actions


def check_victory(state_enemy):

    if(state_enemy == state_friendly):

        return True

    else:

        return False


def drive(action, state_enemy):

    print("Chosen action: {0}".format(action))

    bot = Create2('/dev/ttyUSB0', 115200)
    bot.start()
    bot.safe()

    if (action == "F"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
        bot.drive_distance(1, 100)
        # kan zijn dat dit niet gaat werken en ik dit naar nex_state moet hernoemen, eens kijken

        bot.drive_stop()
        bot.close()

        state_enemy += 11

        done = check_victory(state_enemy)


    elif (action == "B"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
        bot.drive_distance(-1, 100)

        bot.drive_stop()
        bot.close()

        state_enemy -= 11

        done = check_victory(state_enemy)

    elif (action == "L"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

        bot.turn_angle(-90,50)

        bot.drive_stop()
        bot.safe()

        bot.drive_distance(1,100)

        bot.turn_angle(90,50)

        bot.drive_stop()
        bot.close()

        state_enemy -= 1

        done = check_victory(state_enemy)




    elif (action == "R"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

        bot.turn_angle(90, 50)

        bot.drive_stop()
        bot.safe()

        bot.drive_distance(1, 100)

        bot.turn_angle(-90, 50)

        bot.drive_stop()
        bot.close()

        state_enemy += 1

        done = check_victory(state_enemy)

    elif (action == "S"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

        state_enemy = state_enemy

        done = check_victory(state_enemy)

    else:
        pass

    return state_enemy, done


def step(state_friendly, state_enemy, action):

    state_enemy, done = drive(action, state_enemy)

    if(done == True):

        imperial_march()
        time.sleep(1)
        reset()

    else:

        states = str(state_enemy) + "," + str(state_friendly)
        print(states)
        client.publish('/roomba2/state_enemy', states, retain=False)
        print("not my turn anymore")
        client.publish('/roomba2/turn_friendly', 1, retain=False)
        client.publish('/roomba2/turn_enemy', 0, retain=False)


def reset():

    done = False


def imperial_march():

    pass


if __name__ == '__main__':


    client = mqtt.Client()


    try:

        print("script enemy pc control")


        client.on_connect = on_connect
        client.on_message = on_message
        client.connect("78.22.164.90", 1883, 60)
        msg = client.subscribe('/roomba2/#')

        actions = ["F", "B", "L", "R", "S"]

        done = False


        client.loop_forever()
        print("connected to mqtt client")




    except KeyboardInterrupt:
        print("Bye")

    except Exception as e:
        print(e)

    finally:
        if client:
            client.disconnect()


