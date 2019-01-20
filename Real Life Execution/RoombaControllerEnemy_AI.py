import paho.mqtt.client as mqtt
import time
from pycreate2 import Create2
import numpy as np
from random import choice
import keyboard

#THIS SCRIPT CONTROLS THE ENEMY ROOMBA USING Q LEARNING

#OPMERKING: zal iedere keer de posities moeten doorsturen tussen de roombas

def read_file(filename):
    new_data = np.loadtxt(filename)
    new_data = new_data.reshape((121, 121, 5))
    return new_data


def on_connect(client,userdata,flags,rc):

    print("Connected with result code" + str(rc))

def on_message(client, userdata, msg):

    value = msg.payload.decode('utf-8')

    # print(str(value))

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


    elif(msg.topic == "/roomba2/turn_enemy"):

        value = int(value)

        try:

            if (value == 0):
                print("hmmm I really want to make my move but it's the friendly 's turn... ")

            if(value == 1):

                print("making my move")
                print(state_friendly)
                print("Making my move with the updated state of anakin {0}".format(state_friendly))
                step(state_friendly, state_enemy)
                print("made my move")

        except Exception as ex:
            print(ex)

    elif(msg.topic == "/roomba2/victory_friendly"):

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

    action = actions[action]
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


def step(state_friendly, state_enemy):


    y = q_values[state_enemy][state_friendly]

    print("Q_values: {0}".format(y))
    non_zero_indices = np.nonzero(y)
    k = np.argmax(y[non_zero_indices])
    original_indice = non_zero_indices[0][k]
    highest_q_action = original_indice

    action = highest_q_action

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

    q_values = read_file("Q_enemy.txt")
    print("Q_values read")


    try:

        print("script enemy q learning")

        print("bot gestart")

        print("in safe mode geplaatst")

        client.on_connect = on_connect
        client.on_message = on_message
        client.connect("78.22.164.90", 1883, 60)
        msg = client.subscribe('/roomba2/#')

        actions = ["F", "B", "L", "R", "S"]

        done = False

        # client.publish("/roomba/state_friendly", "start")



        client.loop_forever()
        print("connected to mqtt client")

        #hier zelfde probleem gaat hij dit wel doen (staat achter de loopforever)


    except KeyboardInterrupt:
        print("Bye")

    except Exception as e:
        print(e)

    finally:
        if bot:
            bot.stop()
            bot.close()
        if client:
            client.disconnect()


