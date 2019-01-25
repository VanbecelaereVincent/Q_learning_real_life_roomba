import paho.mqtt.client as mqtt

#-------------------------------------------------------------------------------------------
#THIS SCRIPT IS USED FOR PLAYING EITHER AS ENEMY OR FRIENDLY AGAINST AN AI TRAINED AGENT
#-------------------------------------------------------------------------------------------

def on_connect(client,userdata,flags,rc):

    print("Connected with result code" + str(rc))

def on_message(client, userdata, msg):

    value = msg.payload.decode('utf-8')

    if(msg.topic == "/roomba2/computer_control"):
        print(value)

        if(value[0] == "["):

            try:
                b = "[]', "
                for char in b:
                    value = value.replace(char, "")

                legal_actions = list(value)

                print("Please choose an action. Valid actions are: ")

                for _action in legal_actions:

                    print("Action: {0}".format(_action.lower()))

                chosen_action = action(legal_actions)


                client.publish("/roomba2/computer_action", chosen_action)

            except Exception as ex:
                print(ex)

        else: pass

def action(legal_actions):


        action = str(input("Please choose an action: "))


        if(action == "f"):

            chosen_action = "F"
            if(chosen_action in legal_actions):
                action = chosen_action
            else:
                print("This is a valid action, but not from this position.. Try again.")
            print("returning action to robot...")
            return action

        elif(action == 'b'):

            chosen_action = "B"
            if (chosen_action in legal_actions):
                action = chosen_action
            else:
                print("This is a valid action, but not from this position.. Try again.")
            print("returning action to robot...")
            return action

        elif(action == "l"):

            chosen_action = "L"
            if (chosen_action in legal_actions):
                action = chosen_action
            else:
                print("This is a valid action, but not from this position.. Try again.")
            print("returning action to robot...")
            return action

        elif(action == "r"):

            chosen_action = "R"
            if (chosen_action in legal_actions):
                action = chosen_action
            else:
                print("This is a valid action, but not from this position.. Try again.")
            print("returning action to robot...")
            return action

        elif(action == "s"):

            chosen_action = "S"
            if (chosen_action in legal_actions):
                action = chosen_action
            else:
                print("This is a valid action, but not from this position.. Try again.")
            print("returning action to robot...")
            return action

        else:
            print("Don't be silly.. ")



if __name__ == '__main__':

    client = mqtt.Client()

    try:

        print("script enemy pc control")

        client.on_connect = on_connect
        client.on_message = on_message
        client.connect("78.22.164.90", 1883, 60)
        msg = client.subscribe('/roomba2/#')

        actions = ["F", "B", "L", "R", "S"]

        client.loop_forever()
        print("connected to mqtt client")


    except KeyboardInterrupt:
        print("Bye")

    except Exception as e:
        print(e)

    finally:
        if client:
            client.disconnect()
