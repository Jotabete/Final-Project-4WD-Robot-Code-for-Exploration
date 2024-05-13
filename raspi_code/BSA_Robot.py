# Library
import random
import numpy as np

# Left Spiral Algorithm
def nextmove(lastmove, arena, xpos, ypos):
    if (lastmove == 0): #utara
        if (xpos-1 != -1): #barat
            if (arena[ypos][xpos-1] == 0):
                return 3
        if (ypos-1 != -1): #utara
            if (arena[ypos-1][xpos] == 0):
                return 0
        if (xpos+1 != 15): #timur
            if (arena[ypos][xpos+1] == 0):
                return 1
        if (ypos+1 != 9): #selatan
            if (arena[ypos+1][xpos] == 0):
                return 2
        return 5 #backtrack
    elif (lastmove == 1): #timur
        if (ypos-1 != -1): #utara
            if (arena[ypos-1][xpos] == 0):
                return 0
        if (xpos+1 != 15): #timur
            if (arena[ypos][xpos+1] == 0):
                return 1
        if (ypos+1 != 9): #selatan
            if (arena[ypos+1][xpos] == 0):
                return 2
        if (xpos-1 != -1): #barat
            if (arena[ypos][xpos-1] == 0):
                return 3
        return 5 #backtrack
    elif (lastmove == 2): #selatan
        if (xpos+1 != 15): #timur
            if (arena[ypos][xpos+1] == 0):
                return 1
        if (ypos+1 != 9): #selatan
            if (arena[ypos+1][xpos] == 0):
                return 2
        if (xpos-1 != -1): #barat
            if (arena[ypos][xpos-1] == 0):
                return 3
        if (ypos-1 != -1): #utara
            if (arena[ypos-1][xpos] == 0):
                return 0
        return 5 #backtrack
    else: #barat
        if (ypos+1 != 9): #selatan
            if (arena[ypos+1][xpos] == 0):
                return 2
        if (xpos-1 != -1): #barat
            if (arena[ypos][xpos-1] == 0):
                return 3
        if (ypos-1 != -1): #utara
            if (arena[ypos-1][xpos] == 0):
                return 0
        if (xpos+1 != 15): #timur
            if (arena[ypos][xpos+1] == 0):
                return 1
        return 5 #backtrack

# Reinforcement Learning Backtracking Algorithm
def searchzero(arena, xpos, ypos):
    # Hyperparameter dan Q Table
    Qtable = [[0 for i in range(4)] for i in range(135)]
    learningrate = 0.1
    discountrate = 1.0
    explorationrate = 0.1
    episode = 100
    maxstep = 100
    movement = [0, 1, 2, 3]
    doit = False
    
    # Training
    for i in range(episode):
        state = xpos + 15*ypos
        newypos = ypos
        newxpos = xpos
        reward = -1
        istep = 0
        while True:
            # Eksplorasi dan eksploitasi
            if (random.uniform(0, 1) < explorationrate):
                myaction = random.choice(movement)
            else:
                myaction = np.argmax(Qtable[state])
            
            # Mengubah posisi state
            if (myaction == 0):
                if (newypos -1 != -1 and arena[newypos-1][newxpos] != 6):
                    newypos -= 1
            elif (myaction == 1):
                if (newxpos + 1 != 15 and arena[newypos][newxpos+1] != 6):
                    newxpos += 1
            elif (myaction == 2):
                if (newypos + 1 != 9 and arena[newypos+1][newxpos] != 6):
                    newypos += 1
            elif (myaction == 3):
                if (newxpos - 1 != -1 and arena[newypos][newxpos-1] != 6):    
                    newxpos -= 1

            # Update Q table
            newstate = newxpos + 15*newypos
            if (arena[newypos][newxpos] == 0):
                reward += 10
            oldvalue = Qtable[state][myaction]
            nextvalue = max(Qtable[newstate])
            newvalue = (1-learningrate)*oldvalue + learningrate*(reward + discountrate*nextvalue)
            Qtable[state][myaction] = newvalue
            state = newstate
            
            istep += 1
            # Tujuan tercapai
            if (arena[newypos][newxpos] == 0 or istep == maxstep):
                break
            
    # Eksploitasi untuk mengeluarkan output pergerakan backtracking
    changexpos = xpos
    changeypos = ypos
    outputmovement = []
    step = 0
    while (not doit):
        statenow = changexpos + 15*changeypos
        goaction = np.argmax(Qtable[statenow])
        if (goaction == 0):
            changeypos -= 1
        elif (goaction == 1):
            changexpos += 1
        elif (goaction == 2):
            changeypos += 1
        elif (goaction == 3):
            changexpos -= 1
        outputmovement.append(goaction)
        step += 1
        if (arena[changeypos][changexpos] == 0):
            doit = True
        if (step == 50):
            outputmovement = "reject"
            doit = True
            
    return(outputmovement)

# Dummy arena    
myarena = [[0 for i in range(15)] for i in range(9)]
xpos = 0
ypos = 0
myarena[ypos][xpos] = 1
lastaction = 1

# Bergerak di arena
while True:
    action = nextmove(lastaction, myarena, xpos, ypos)
    if (action == 0):
        ypos -= 1
        myarena[ypos][xpos] = 1
    elif (action == 1):
        xpos += 1
        myarena[ypos][xpos] = 1
    elif (action == 2):
        ypos += 1
        myarena[ypos][xpos] = 1
    elif (action == 3):
        xpos -= 1
        myarena[ypos][xpos] = 1
    else:
        rlmove = searchzero(myarena, xpos, ypos)
        if (rlmove == "reject"):
            break
        else:
            for i in rlmove:
                if (i == 0):
                    ypos -= 1
                    myarena[ypos][xpos] = 1
                elif (i == 1):
                    xpos += 1
                    myarena[ypos][xpos] = 1
                elif (i == 2):
                    ypos += 1
                    myarena[ypos][xpos] = 1
                else:
                    xpos -= 1
                    myarena[ypos][xpos] = 1
                action = i
    lastaction = action