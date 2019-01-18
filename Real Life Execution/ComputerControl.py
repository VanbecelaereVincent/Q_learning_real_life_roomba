#THIS SCRIPT IS USED FOR PLAYING EITHER AS ENEMY OR FRIENDLY AGAINST AN AI TRAINED AGENT
#configured for windows
import keyboard


#je krijgt binnen jouw turn my young padawan & dan kan je een key indrukken en dus een actie kiezen

def action():

    my_turn = 0

    #hier nog ergens heeltijd naar mqtt luisteren om die my_turn op 1 te krijgen en dan terug op nul

    if my_turn == 1:

        if(keyboard.is_pressed("f")):

            #code die de command doorstuurt naar een script dat luistert (mss dus twee scripts op de pies, eentje voor q learning
            #, eentje voor besturing vanaf de pc!

            my_turn=0

        if(keyboard.is_pressed("b")):

            my_turn=0

        if(keyboard.is_pressed("l")):

            my_turn=0

        if(keyboard.is_pressed("r")):

            my_turn=0

        if(keyboard.is_pressed("s")):

            my_turn = 0

        else:

            pass