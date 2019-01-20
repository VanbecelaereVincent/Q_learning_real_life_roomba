import paho.mqtt.client as mqtt
import time
from pycreate2 import Create2
import numpy as np
from random import choice
import keyboard


# THIS SCRIPT CONTROLS THE ENEMY ROOMBA USING Q LEARNING

# OPMERKING: zal iedere keer de posities moeten doorsturen tussen de roombas

def read_file(filename):
    new_data = np.loadtxt(filename)
    new_data = new_data.reshape((121, 121, 5))
    return new_data


def on_connect(client, userdata, flags, rc):
    print("Connected with result code" + str(rc))


def on_message(client, userdata, msg):

    value = msg.payload.decode('utf-8')

    # print(str(value))

    if (msg.topic == '/roomba2/state_enemy'):

        global state_enemy
        global state_friendly
        try:

            if(value == "start"):

                state_enemy = 39
                state_friendly = 6

            else:


                states_list = value.split(",")
                state_enemy = int(states_list[0])
                state_friendly = int(states_list[1])
                print("State enemy has been updated {0}".format(state_enemy))
                print("I also received my own position which is: {0}".format(state_friendly))


        except Exception as ex:
            print(ex)

    elif (msg.topic == "/roomba2/turn_friendly"):

        value = int(value)

        if(value == 0):

            print("hmmm I really want to another move but it's the enemies' turn... ")

        elif(value == 1):
            print("My Turn: making my move")

            try:
                print("Making a move with the update enemy state: {0}".format(state_enemy))
                step(state_friendly, state_enemy)

            except Exception as error:
                print(error)

            print("move made")

        else:
            print("pass")
            print(value)
            pass


    elif (msg.topic == "/roomba2/victory_enemy"):

        print("Fuck I lost, better luck next game I guess.. Reset incoming!")
        reset()


    else:
        pass


def get_legal_actions(state):
    if (state == 0):
        actions = ["F", "R", "S"]
        return actions

    elif (state == 10):
        actions = ["F", "L", "S"]

    elif (state == 110):
        actions = ["B", "R", "S"]

    elif (state == 121):
        actions = ["B", "L", "S"]

    elif (
            state == 11 or state == 22 or state == 33 or state == 44 or state == 55 or state == 66 or state == 77 or state == 88 or state == 99):
        actions = ["F", "B", "R", "S"]

    elif (
            state == 21 or state == 32 or state == 43 or state == 54 or state == 65 or state == 76 or state == 87 or state == 98 or state == 109):
        actions = ["F", "B", "L", "S"]

    else:
        actions = ["F", "B", "L", "R", "S"]

    return actions


def check_victory(state_friendly):

    if (state_friendly >= 110):

        return True

    else:

        return False


def drive(action, state_friendly):

    action = actions[action]

    bot = Create2('/dev/ttyUSB0', 115200)
    bot.start()
    bot.safe()

    if (action == "F"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
        bot.drive_distance(1, 100)
        # kan zijn dat dit niet gaat werken en ik dit naar nex_state moet hernoemen, eens kijken

        bot.drive_stop()
        bot.close()

        state_friendly += 11

        done = check_victory(state_friendly)


    elif (action == "B"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
        bot.drive_distance(-1, 100)

        bot.drive_stop()
        bot.safe()
        bot.close()

        state_friendly -= 11

        done = check_victory(state_friendly)

    elif (action == "L"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

        bot.turn_angle(-90, 50)

        bot.drive_stop()

        bot.safe()

        bot.drive_distance(1, 100)


        bot.turn_angle(90, 50)

        bot.drive_stop()

        bot.close()

        state_friendly -= 1

        done = check_victory(state_friendly)

    elif (action == "R"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

        bot.turn_angle(90, 50)

        bot.drive_stop()

        bot.safe()

        bot.drive_distance(1, 100)

        bot.turn_angle(-90, 50)

        bot.drive_stop()

        bot.close()

        state_friendly += 1

        done = check_victory(state_friendly)

    elif (action == "S"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

        state_friendly = state_friendly
        bot.close()

        done = check_victory(state_enemy)

    else:
        pass

    return state_friendly, done

def step(state_friendly, state_enemy):

    y = q_values[state_enemy][state_friendly]

    print("Q_values: {0}".format(y))
    non_zero_indices = np.nonzero(y)
    k = np.argmax(y[non_zero_indices])
    original_indice = non_zero_indices[0][k]
    highest_q_action = original_indice

    action = highest_q_action

    print("Chosen action: {0}".format(action))

    print("Lets drive")
    state_friendly, done = drive(action, state_friendly)
    print("I drove")

    if (done == True):
        imperial_march()
        time.sleep(1)
        reset()

    else:

        #nu even de state van de enemy publishen maar mag ik eigenlijk niet doen van hieruit (ze moeten alletwee aanstaan maar doe ik nu niet)
        states = str(state_enemy) + "," + str(state_friendly)
        print("fuck 't is hier dat misloopt  {0}".format(states))
        client.publish('/roomba2/state_friendly', states, retain=False)
        client.publish('/roomba2/turn_enemy', 1, retain=False)
        client.publish('/roomba2/turn_friendly', 0, retain=False)

def reset():
    # hier nog een hele mooie berekening om terug naar zijn startpositie te rijden
    done = False


def imperial_march():

    pass


if __name__ == '__main__':



    client = mqtt.Client()

    global q_values

    global done

    try:


        print("bot gestart")

        index = 0
        print("in safe mode geplaatst")

        client.on_connect = on_connect
        client.on_message = on_message
        client.connect("78.22.164.90", 1883, 60)
        msg = client.subscribe('/roomba2/#')

        actions = ["F", "B", "L", "R", "S"]

        done = False

        q_values = read_file("Q_friendly.txt")
        print("Q_values read")

        client.publish("/roomba2/state_enemy", "start", retain=False)

        client.publish("/roomba2/turn_enemy", 0, retain=False)
        client.publish("/roomba2/turn_friendly", 1, retain=False)



        client.loop_forever()
        print("connected to mqtt client")

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


