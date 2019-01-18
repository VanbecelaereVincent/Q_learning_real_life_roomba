import paho.mqtt.client as mqtt
import time
from pycreate2 import Create2
import numpy as np
from random import choice
import keyboard


# THIS SCRIPT CONTROLS THE ENEMY ROOMBA USING USER INPUT

# OPMERKING: zal iedere keer de posities moeten doorsturen tussen de roombas

def read_file(filename):
    new_data = np.loadtxt(filename)
    new_data = new_data.reshape((121, 121, 5))
    return new_data


def on_connect(client, userdata, flags, rc):
    print("Connected with result code" + str(rc))


def on_message(client, userdata, msg):
    value = msg.payload.decode('utf-8')

    print(str(value))

    if (msg.topic == '/roomba/state_enemy'):

        state_enemy = value

    elif (msg.topic == "/roomba/turn_friendly"):

        if (value == 0):

            print("hmmm I really want to make my move but it's the enemies' turn... ")

        else:

            step(state_friendly, state_enemy)


    elif (msg.topic == "/roomba/victory_enemy"):

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
    if (action == "F"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
        bot.drive_distance(1, 20)
        # kan zijn dat dit niet gaat werken en ik dit naar nex_state moet hernoemen, eens kijken

        bot.drive_stop()
        bot.safe()

        state_friendly += 11

        done = check_victory(state_friendly)


    elif (action == "B"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
        bot.drive_distance(-1, 20)

        bot.drive_stop()
        bot.safe()

        state_friendly -= 11

        done = check_victory(state_friendly)

    elif (action == "L"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
        angle = 0

        while angle <= 90:
            bot.turn_angle(0.1, 20)
            angle += 0.1
        bot.drive_distance(1, 20)

        while angle != 0:
            bot.turn_angle(-0.1, 20)
            angle -= 0.1

        bot.drive_stop()
        bot.safe()

        state_friendly -= 1

        done = check_victory(state_friendly)




    elif (action == "R"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
        angle = 0

        while angle <= 90:
            bot.turn_angle(-0.1, 20)
            angle += 0.1

        bot.drive_distance(1, 20)

        while angle != 0:
            bot.turn_angle(0.1, 20)
            angle -= 0.1

        bot.drive_stop()
        bot.safe()

        state_friendly += 1

        done = check_victory(state_friendly)

    elif (action == "S"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

        state_friendly = state_friendly

        done = check_victory(state_enemy)

    else:
        pass

    return state_friendly, done


def step(state_friendly, state_enemy):

    actions = get_legal_actions(state_friendly)
    action = ""

    while action not in actions:

        # hier niet zelf keyboard press maar wachten op een berichtje!!!  (dus chosen action meegeven aan step)

        if (keyboard.is_pressed("f")):

            action = "F"

        elif (keyboard.is_pressed("b")):

            action = "B"

        elif (keyboard.is_pressed("l")):

            action = "L"

        elif (keyboard.is_pressed("r")):

            action = "R"

        elif (keyboard.is_pressed("s")):

            action = "S"

        else:

            print("please choose a valid action...")

    state_friendly, done = drive(action, state_friendly)

    if (done == True):
        imperial_march()
        time.sleep(1)
        reset()

    else:

        client.publish('/roomba/state_friendly', state_friendly)
        client.publish('/roomba/turn_friendly', 0)
        client.publish('/roomba/turn_enemy', 1)


def reset():
    done = False
    # hier nog een hele mooie berekening om terug naar zijn startpositie te rijden


def imperial_march():
    pass


if __name__ == '__main__':

    bot = Create2('/dev/ttyUSB0', 115200)
    client = mqtt.Client()

    global state_friendly
    global state_enemy
    global done

    try:

        bot.start()
        bot.open()
        print("bot gestart")
        bot.safe()
        print("in safe mode geplaatst")

        client.on_connect = on_connect
        client.on_message = on_message
        client.connect("78.22.129.133", 1883, 60)
        msg = client.subscribe('/roomba/#')

        client.loop_forever()
        print("connected to mqtt client")

        # gaat hij dit nog uitvoeren? Anders gaak ik de start anders moeten oplossen

        client.publish("/roomba/turn_enemy", 0)
        client.publish("/roomba/turn_friendly", 1)

        state_friendly = 6
        state_enemy = 39
        done = False

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


