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
            client.publish("/roomba2/computer_control", str(legal_actions),qos=2)


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

        bot.turn_angle(90,50)

        bot.drive_stop()
        bot.safe()

        bot.drive_distance(1,100)

        bot.turn_angle(-90,50)

        bot.drive_stop()
        bot.close()

        state_enemy -= 1

        done = check_victory(state_enemy)




    elif (action == "R"):

        # hier moet mijn gyro code komen die zorgt dat hij altijd even ver rijdt

        bot.turn_angle(-90, 50)

        bot.drive_stop()
        bot.safe()

        bot.drive_distance(1, 100)

        bot.turn_angle(90, 50)

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
        client.publish('/roomba2/state_enemy', states, retain=False,qos=2)
        print("not my turn anymore")
        client.publish('/roomba2/turn_friendly', 1, retain=False,qos=2)
        client.publish('/roomba2/turn_enemy', 0, retain=False, qos=2)


def reset():

    pass
    # heb geen tijd meer gehad voor een reset, momenteel is dit gewoon de scripts opnieuw opstarten...
    # zou eigenlijk moeten een fucntie bevatten die de robots terug laat rijden (zonder tegen elkaar te botsen) naar hun originele positie
    # posities terug in code op 6 & 39 zetten
    # done terug op false
    # turn enemy op 0 & turn friendly op 1


def imperial_march():
    import serial
    import time

    # this is the port you're connecting to the irobot with
    # on windows, this is something like COM2
    N = "/dev/ttyUSB0"

    def ints2str(lst):
        '''
        Taking a list of notes/lengths, convert it to a string
        '''
        s = ""
        for i in lst:
            if i < 0 or i > 255:
                raise Exception
            s = s + str(chr(i))
        return s

    # do some initialization magic
    s = serial.Serial(N, 115200, timeout=4)
    print("sending start...")
    s.write(ints2str([128]))
    print("switch to full mode...")
    s.write(ints2str([132]))

    # define silence
    r = 30

    # map note names in the lilypad notation to irobot commands
    c4 = 60
    cis4 = des4 = 61
    d4 = 62
    dis4 = ees4 = 63
    e4 = 64
    f4 = 65
    fis4 = ges4 = 66
    g4 = 67
    gis4 = aes4 = 68
    a4 = 69
    ais4 = bes4 = 70
    b4 = 71
    c5 = 72
    cis5 = des5 = 73
    d5 = 74
    dis5 = ees5 = 75
    e5 = 76
    f5 = 77
    fis5 = ges5 = 78
    g5 = 79
    gis5 = aes5 = 80
    a5 = 81
    ais5 = bes5 = 82
    b5 = 83
    c6 = 84
    cis6 = des6 = 85
    d6 = 86
    dis6 = ees6 = 87
    e6 = 88
    f6 = 89
    fis6 = ges6 = 90

    # define some note lengths
    # change the top MEASURE (4/4 time) to get faster/slower speeds
    MEASURE = 160
    HALF = MEASURE / 2
    Q = MEASURE / 4
    E = MEASURE / 8
    Ed = MEASURE * 3 / 16
    S = MEASURE / 16

    MEASURE_TIME = MEASURE / 64.

    # send song
    # [140, num, len, (note, dur)_1, ...]
    # durations are multiples of 1/64
    # a4 a a f8. c'16 | a4 f8. c'16 a2
    # e2 e e f8. c'16 | aes4 f8. c'16 a2
    # a'4 a,8. a16 a'4 aes8 g | ges16 f g8 r8 bes, ees4 d8 des
    # c16 b c8 r8 f,8 aes4 f8. aes16 | c4 a8. c16 e2
    # a4 a,8. a16 a'4 aes8 g | ges16 f g8 r8 bes, ees4 d8 des
    # c16 b c8 r8 f,8 aes4 f8. c'16 | a4 f8. c,16 a2
    # 40/64 bps
    print("send songs...")
    # first upload the songs to the irobot...
    s.write(ints2str([140, 0, 9,
                      a4, Q, a4, Q, a4, Q, f4, Ed, c5, S,
                      a4, Q, f4, Ed, c5, S, a4, HALF]))
    s.write(ints2str([140, 1, 9,
                      e5, Q, e5, Q, e5, Q, f5, Ed, c5, S,
                      aes4, Q, f4, Ed, c5, S, a4, HALF]))
    s.write(ints2str([140, 2, 9,
                      a5, Q, a4, Ed, a4, S, a5, Q, aes5, E, g5, E,
                      ges5, S, f5, S, ges5, S]))
    s.write(ints2str([140, 3, 8,
                      r, E, bes4, E, ees5, Q, d5, E, des5, E,
                      c5, S, b4, S, c5, E]))
    s.write(ints2str([140, 4, 9,
                      r, E, f4, E, aes4, Q, f4, Ed, aes4, S,
                      c5, Q, a4, Ed, c5, S, e5, HALF]))
    # play 2 again
    # play 3 again
    s.write(ints2str([140, 5, 9,
                      r, E, f4, E, aes4, Q, f4, Ed, c5, S,
                      a4, Q, f4, Ed, c5, S, a4, HALF]))

    # once all the songs are uploaded, play them at the right times
    # add a little extra time, b/c otherwise cuts off the end
    print("play songs...")
    s.write(ints2str([141, 0]))
    time.sleep(MEASURE_TIME * 2.01)

    s.write(ints2str([141, 1]))
    time.sleep(MEASURE_TIME * 2.01)

    s.write(ints2str([141, 2]))
    time.sleep(MEASURE_TIME * 1.26)

    s.write(ints2str([141, 3]))
    time.sleep(MEASURE_TIME * 1.01)

    s.write(ints2str([141, 4]))
    time.sleep(MEASURE_TIME * 1.76)

    s.write(ints2str([141, 2]))
    time.sleep(MEASURE_TIME * 1.26)

    s.write(ints2str([141, 3]))
    time.sleep(MEASURE_TIME * 1.01)

    s.write(ints2str([141, 5]))
    time.sleep(MEASURE_TIME * 1.76)


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


