from numpy import gradient, where, diff, sign, ones, sqrt, sum, mean
from control import tf, step_response, forced_response
#Import necessary libraries 

#A new class that handle the estimation of a model from a step response
class estimateModel():
    #The estimateModel's constructor
    def __init__(self, t, y):
        self.__t = t
        self.__y = y
        self.__u = 0

        self.__type = ""

        self.__estimateY = 0 
        self.__tf = 0
        self.__e = 0

        self.__ISE = 0

        #First determinate the type of the system (overdamped, underdamped)
        self.__whatType()
        self.__calculateISEIndex()
    #This method identify what's is the system's behavior
    def __whatType(self):
        #Declaring t, y(t) & y(inf)
        t = self.__t
        y = self.__y
        yinf = y[-1]

        #Evaluate if the maximum value is equals to the las value in the vector y, in order to know
        #if the system is overdamped or underdamped
        if (max(y) == yinf):
            self.__type = "overdamped"
            #If the system is overdamped, apply the Miller's tangent method
            self.__tangetMethod()
        else:
            self.__type = "underdamped"
            #If the system is overdamped, apply the Stark's three points method method
            self.__threePointsMethod()
    #This method apply the Miller's tanget method to find a estimate model of the overdamped system
    def __tangetMethod(self):
        #In order to obtein a overdamped second order transfer function
        #it's neccesary to calculate the following parameters
        #Kp = it't the gain of the transfer function output/input
        #tm = it's the time when the tangent line is equals to zero
        #T1 = it's the time from the system starts rising until the output reaches 63.2% of the steady value
        #T2 = it's the time from the system starts rising until the tangent line reaches the steady value

        #Initializing all variables with 0
        tm, T1, T2 = 0, 0, 0

        #Declaring t, y(t) & y(inf)
        t = self.__t
        y = self.__y
        yinf = y[-1]

        #Calculating gain value
        Kp = yinf - y[0]

        #Calculating the tanget to inflection point
        tangetLine = self.__findTanget()

        #Calculating the tm, T1 & T2 parameters
        tm = t[where(tangetLine == 0)[0][-1]]
        T1 = t[where(y >= yinf*0.632)[0][0]] - tm
        T2 = t[where(tangetLine == 1)[0][0]] - tm

        s = tf("s")
        G = (Kp)/((T1*s+1)*(T2*s+1)) #Missing to multiplicate Kp * e^(-tm*s)

        #Saving the estimate transfer function of the system
        self.__tf = G

        #Creating a new input (step) that starts affter tm
        input = ones(len(t))
        for i in range(len(t)):
            if (t[i] < tm):
                input[i] = 0

        #Saving the tricky input to simulate deadtime
        self.__u = input

        #Simulating dead time forcing the input to start later than tm
        response = forced_response(G, t, input)

        #Saving the estimate y step response in those vectors
        self.__estimateY = response.outputs
    #This method find a tanget line to a specific curve in a inflection point    
    def __findTanget(self):
        #Declaring t & y(t)
        t = self.__t
        y = self.__y

        #Finding first derivate
        yFirstDerivate = gradient(y, t)
        #Finding second derivate
        ySecondDerivate = gradient(yFirstDerivate, t)
        #Just asking for the first element of the tuple
        #Inflection point found!!!
        inflectionIndex = where(diff(sign(ySecondDerivate)))[0][0]
        print(inflectionIndex)
        #Finding P(t, y(t)) at the inflection point
        tInflection = t[inflectionIndex]
        yInflection = y[inflectionIndex]

        #Finding the "equation" of the tangent line at inflection point
        tangent = yFirstDerivate[inflectionIndex]*(t - tInflection) + yInflection

        #Limitating the tangent line, so it will not have values above zero and greater than one
        for i in range(len(t)):
            if (tangent[i] < 0):
                tangent[i] = 0
            elif (tangent[i] >= 1):
                tangent[i] = 1

        #Return the tangent line       
        return tangent
    #This method apply the Stark's three points method to find a estimate model of the underdamped system
    def __threePointsMethod(self):
        #In order to obtein a overdamped second order transfer function
        #it's neccesary to calculate the following parameters
        #Kp = it't the gain of the transfer function output/input
        #tm = it's the time when the tangent line is equals to zero
        #T1 = it's the time from the system starts rising until the output reaches 63.2% of the steady value
        #T2 = it's the time from the system starts rising until the tangent line reaches the steady value

        #Initializing all variables with 0
        tm, T1, T2 = 0, 0, 0

        #Declaring t, y(t) & y(inf)
        t = self.__t
        y = self.__y
        yinf = y[-1]

        #Calculating gain value
        Kp = yinf - y[0]
        
        #Calculating index of the three points
        index15 = where(y >= yinf*0.15)[0][0] #The time when the ouput value is more than 2% of steady value
        index45 = where(y >= yinf*0.45)[0][0] #The time when the ouput value is more than 70% of steady value
        index75 = where(y >= yinf*0.75)[0][0] #The time when the ouput value is more than 90% of steady value

        #Calculating n value
        x = (t[index45] - t[index15])/(t[index75] - t[index15])

        #Calculating zeta value depending
        zeta = (0.0805 - 5.547*((0.475 - x)**2))/(x-0.356)
        f_2 = 0
        if (zeta <= 1):
            f_2 = 0.708*((2.811)**2)
        else:
            f_2 = 2.6*zeta - 0.6

        wn = f_2/(t[index75] - t[index15])

        #Calculating T value
        f_3 = 0.922*(1.66)**zeta

        tm = t[index45] - ((f_3)/(wn))

        T1 = (zeta + sqrt(zeta*zeta-1))/(wn)

        T2 = (zeta - sqrt(zeta*zeta-1))/(wn)

        s = tf("s")
        G = 0
        if (zeta >= 1):
            G = (Kp)/((T1*s+1)*(T2*s+1))
        else:
            G = (Kp*wn**2)/((s**2)+2*zeta*wn*s+wn**2) #Missing to multiplicate Kp * e^(-tm*s)
        

        print(G)

        #Saving the estimate transfer function of the system
        self.__tf = G

        #Creating a new input (step) that starts affter tm
        input = ones(len(t))
        for i in range(len(t)):
            if (t[i] < tm):
                input[i] = 0

        #Saving the tricky input to simulate deadtime
        self.__u = input

        #Simulating dead time forcing the input to start later than tm
        response = step_response(G, t)

        #Saving the estimate y step response in those vectors
        self.__estimateY = response.outputs
    #Calculate the error indexes
    def __calculateISEIndex(self):
        def integral(func):
            return sum(func)*(self.__t[1]-self.__t[0])
        #Define the error signal e(t) as the difference between r(t) and y(t)
        error = self.__y - self.__estimateY

        ISE = integral(error**2)

        #
        ISEmax = sum(self.__y - mean(self.__y)**2)
        ISEporcentaje = (1 - ISE/ISEmax)

        #Calculate the error index
        self.__ISE = ISEporcentaje
    #This method returns the estimate transfer function found
    def getTransferFunc(self):
        #Return the estimate transfer function of the model
        return self.__tf
    #This method returns the step response of the estimate transfer function
    def getStepResponse(self):
        #The output it's just the y axis values, for x axis use the time vector
        return self.__estimateY
    #This method returns the type of the estimate system
    def getType(self):
        return self.__type
    #This method returns the value of the error indexs
    def getErrorIndex(self):
        return self.__ISE