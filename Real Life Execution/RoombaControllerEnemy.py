import paho.mqtt.client as mqtt
from pycreate2 import Create2
import numpy as np
from random import choice

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

    print(str(value))

    if(msg.topic == "/roomba/action"):

        step(value, state_friendly, state_enemy)

    elif(msg.topic == "/roomba/victory"):
        pass
        #reset()
        #imperial march

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
        actions = ["F", "B", "L","R","S"]

    return actions


def step(action, state_friendly, state_enemy, AI=True):

    #opmerking: altijd terug draaien ook dat hij terug naar voor kijkt (zal gemakkelijker zijn voor mij)

    actions = get_legal_actions(state_enemy)

    y = q_values[state_enemy][state_friendly]

    print("Q_values: {0}".format(y))
    non_zero_indices = np.nonzero(y)
    k = np.argmax(y[non_zero_indices])
    original_indice = non_zero_indices[0][k]
    highest_q_action = original_indice
    print("gekozen actie: {0}".format(highest_q_action))

    if (highest_q_action not in actions):
        print("dit kan niet maar gebeurt toch")
        print("----------------------------")

        action = choice(actions)

    else:
        print("----------------------------")
        action = highest_q_action

    if(AI == True):

        if (action == "F"):
        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

            state_enemy += 11

        elif(action == "B"):
        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

           state_enemy -= 11

        elif (action == "L"):
        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

           state_enemy -= 1

        elif (action == "R"):
        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

            state_enemy += 1

        elif (action == "S"):
        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

            state_enemy = state_enemy

        else:
            pass

    else:
        pass
        #hier code om met de computer te spelen



    # elif(agent == "friendly"):
    #
    #     if (action == "F"):
    #         # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
    #
    #         state_friendly += 11
    #
    #     elif (action == "B"):
    #         # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
    #
    #         state_friendly -= 11
    #
    #     elif (action == "L"):
    #         # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
    #
    #         state_friendly -= 1
    #
    #     elif (action == "R"):
    #         # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
    #
    #         state_friendly += 1
    #
    #     elif (action == "S"):
    #         # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt
    #
    #         state_friendly = state_friendly

        # else:
        #     pass


def reset():
    pass


if __name__ == '__main__':

    bot = Create2('/dev/ttyUSB0', 115200)
    client = mqtt.Client()

    q_values = read_file("Q_enemy.txt")

    state_friendly = 6
    state_enemy = 39

    try:

        bot.start()
        print("bot gestart")
        bot.safe()
        print("in safe mode geplaatst")

        client.on_connect = on_connect
        client.on_message = on_message
        client.connect("78.22.129.133", 1883, 60)
        msg = client.subscribe('/roomba/#')
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


