from ServoSimulator import SimpleArm, State, get_end_point
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from sklearn import linear_model
from sklearn.naive_bayes import MultinomialNB
from sklearn.model_selection import train_test_split


#   length: coordsX,CoordsY :angles: transition_heading = next angles, depth

def get_flatimage_string(str):
    data = str.split('=')
    state = []
    adj_angles = []
    for item in data[0].split(','):        
        state.append(item)
    for theta in data[1].split(','):  
        if len(theta) > 0:                        
            adj_angles.append(float(theta))                    
    return state, adj_angles

def main():
    inFile = open("NavigatorData1.txt", "r")                    # Create Regrssion model
    raw_data = inFile.read()
    inFile.close()
    all_data = raw_data.split('\n')
    Ydata = []
    Xdata = []
    state = []
    for line in all_data:
        state, angles = get_flatimage_string(line)
        Xdata.append(np.array(state))
        Ydata.append(np.array(angles))
        if len(angles) > 5:
            print(line)
    print('---Data Collected---')
    Xdata = np.array(Xdata).astype(float)
    Ydata = np.array(Ydata).astype(float)
    x_train, x_test, y_train, y_test = train_test_split(Xdata, Ydata, test_size=.001)
    print("---Data Prepared---")
    print(x_train.shape, y_train.shape)
    pos_reg = linear_model.LinearRegression()
    pos_reg.fit(list(x_train), y_train)
    print("---Model Fitted---")
    
    angles = pos_reg.predict(x_test)
    
    system = SimpleArm(5, 5, (0,0))
    fig = plt.figure() 
    plt.xlim(-2, 30)
    plt.ylim(-10, 30)
    plt.title('Arman Simulation')
    print('===Results===')
    for i,angle in enumerate(angles):
        #print(x_test[i], '->', angles)
        #print(x_test[i], '->', y_test[i])
        system.set_angles(x_test[i])
        plt.xlim(-2, 30)
        plt.ylim(-10, 30)
        (x, y) = zip(*system.joints)
        plt.plot(x, y, 'g', lw=2)               # plot original line (green)
        system.set_angles(angle)
        plt.xlim(-2, 30)
        plt.ylim(-10, 30)
        (x, y) = zip(*system.joints)
        plt.plot(x, y, 'r', lw=2)               # plot prediction (red)
        system.set_angles(y_test[i])
        plt.xlim(-2, 30)
        plt.ylim(-10, 30)
        (x, y) = zip(*system.joints)
        plt.plot(x, y, 'b', lw=2)               # plot actual (blue)
        plt.show()

if __name__ == "__main__":
    main()