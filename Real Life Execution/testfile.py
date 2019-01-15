from pycreate2 import Create2

index=0
angle=208


try:

    bot = Create2("/dev/ttyUSB0", 115200)
    bot.start()
    bot.safe()


    while True:


        # if(index%2 == 0):
        #     angle = 208
        #
        # if(index+2==1):
        #     angle=-208

        bot.drive_distance(1,100,True)

        # bot.drive_stop()

        bot.safe()

        bot.turn_angle(angle,100)

        bot.drive_stop()

        bot.safe()

        index+=1

except KeyboardInterrupt:
    print("Bye")


except Exception as e:
    print(e)

finally:
    if bot:
        bot.stop()