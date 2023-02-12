from Motor import *
from Led import *
from Buzzer import *
import time

PWM = Motor()
led = Led()
buzzer=Buzzer()

def test():
    try:
        PWM.setMotorModel(1000,1000,1000,1000) # forward
        time.sleep(2) 
        PWM.setMotorModel(0,0,0,0) #pause for traction 
        time.sleep(0.5)  
        PWM.setMotorModel(-2500,-2500,2500,2500) # left, 90 degrees
        time.sleep(0.45)   
        PWM.setMotorModel(0,0,0,0)  #pause for traction
        time.sleep(0.5)  
        led.ledIndex(0x01,255,0,0) # LED 0 to red

        PWM.setMotorModel(1000,1000,1000,1000) # forward
        time.sleep(2)
        PWM.setMotorModel(0,0,0,0)  
        time.sleep(0.5)  
        PWM.setMotorModel(-2500,-2500,2500,2500) # left, 90 degrees
        time.sleep(0.45)  
        PWM.setMotorModel(0,0,0,0)
        time.sleep(0.5)
        led.ledIndex(0x02,0,0,255) # LED 1 to blue

        PWM.setMotorModel(1000,1000,1000,1000) # forward
        time.sleep(2)
        PWM.setMotorModel(0,0,0,0)  
        time.sleep(0.5)  
        PWM.setMotorModel(-2500,-2500,2500,2500) # left, 90 degrees
        time.sleep(0.45) 
        PWM.setMotorModel(0,0,0,0)
        time.sleep(0.5)
        led.ledIndex(0x04,0,255,0) # LED 2 to green

        PWM.setMotorModel(1000,1000,1000,1000) # forward
        time.sleep(2)
        PWM.setMotorModel(0,0,0,0)  
        time.sleep(0.5)  
        PWM.setMotorModel(-2500,-2500,2500,2500) # left, 90 degrees
        time.sleep(0.45) 
        PWM.setMotorModel(0,0,0,0)
        time.sleep(0.5)
        led.ledIndex(0x08,255,255,0) # LED 3 to yellow
        buzzer.run('1') # buzzer
        time.sleep(1)
        buzzer.run('0')
        
        PWM.setMotorModel(0,0,0,0) # stop
        time.sleep(5.0)
        led.colorWipe(led.strip, Color(0,0,0))
        # 6 inch of error
        print("End, normal")
    except:
        PWM.setMotorModel(0,0,0,0) # stop
        print("End, abrupt")

if __name__ == '__main__':
    test()
