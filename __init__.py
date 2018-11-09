import logging
import time

from modules import cbpi
from modules.core.controller import KettleController
from modules.core.hardware import  SensorActive
from modules.core.props import Property

output_cache = 0

@cbpi.sensor
class PID_Output(SensorActive):
    key = Property.Text(label="Name", configurable=True, description="Enter the name of your Kettle")
    def init(self):
        print ("PID_Output init")
        SensorActive.init(self)
        print (self.is_running())
       
    def get_unit(self):
        return "%" 

    def stop(self):
        pass

    def execute(self):
        global output_cache
        while self.is_running():
            print ("output_cache@Sensor: " + str (output_cache))
            try:
                self.data_received(output_cache)
                    
            except:
                pass
            self.api.socketio.sleep(1)

@cbpi.controller
class PIDBoilWithSensor(KettleController):
    global output_cache
    
    key = Property.Text(label="Name", configurable=True, description="Enter the name of your Kettle")
    a_p = Property.Number("P", True, 102, description="P Value of PID e.g. 102")
    b_i = Property.Number("I", True, 100, description="I Value of PID e.g. 100")
    c_d = Property.Number("D", True, 5, description="D Value of PID e.g 5")
    d_max_out = Property.Number("max. output %", True, 100, description="Power which is set above boil threshold e.g. 100")
    e_boil = Property.Number("Boil Threshold", True, 80,description="Temperatre for Boil threshold. Full power mode! e.g. 80")
    f_sampleTime = Property.Number("sampleTime", True, 5, description="heating_time = sampleTime * heat_percent / 100 e.g. 5")
    g_waitTime = Property.Number("waitTime", True, 5, description="1. heater_on 2. sleep(heating_time) 3. heater_off 4. sleep(waitTime) e.g. 5")
    
    def stop(self):
        '''
        Invoked when the automatic is stopped.
        Normally you switch off the actors and clean up everything
        :return: None
        '''
        super(KettleController, self).stop()
        self.heater_off()


    def run(self):
        global output_cache
        
        sampleTime = float(self.f_sampleTime)
        wait_time = float(self.g_waitTime)
        p = float(self.a_p)
        i = float(self.b_i)
        d = float(self.c_d)
        maxout = float(self.d_max_out)
        pid = PIDArduino(sampleTime, p, i, d, 0, maxout)

        while self.is_running():

            if self.get_target_temp() >= float(self.e_boil):
                self.heater_on(100)
                self.sleep(1)
            else:
                heat_percent = pid.calc(self.get_temp(), self.get_target_temp())
                output_cache = heat_percent
                print ("output_cache@Controller: " + str (output_cache))
                
                heating_time = sampleTime * heat_percent / 100
                wait_time = sampleTime - heating_time
                self.heater_on(100)
                self.sleep(heating_time)
                self.heater_off()
                self.sleep(wait_time)

# Based on Arduino PID Library
# See https://github.com/br3ttb/Arduino-PID-Library
class PIDArduino(object):

    def __init__(self, sampleTimeSec, kp, ki, kd, outputMin=float('-inf'),
                 outputMax=float('inf'), getTimeMs=None):
        if kp is None:
            raise ValueError('kp must be specified')
        if ki is None:
            raise ValueError('ki must be specified')
        if kd is None:
            raise ValueError('kd must be specified')
        if sampleTimeSec <= 0:
            raise ValueError('sampleTimeSec must be greater than 0')
        if outputMin >= outputMax:
            raise ValueError('outputMin must be less than outputMax')

        self._logger = logging.getLogger(type(self).__name__)
        self._Kp = kp
        self._Ki = ki * sampleTimeSec
        self._Kd = kd / sampleTimeSec
        self._sampleTime = sampleTimeSec * 1000
        self._outputMin = outputMin
        self._outputMax = outputMax
        self._iTerm = 0
        self._lastInput = 0
        self._lastOutput = 0
        self._lastCalc = 0

        if getTimeMs is None:
            self._getTimeMs = self._currentTimeMs
        else:
            self._getTimeMs = getTimeMs

    def calc(self, inputValue, setpoint):
        now = self._getTimeMs()

        if (now - self._lastCalc) < self._sampleTime:
            return self._lastOutput

        # Compute all the working error variables
        error = setpoint - inputValue
        dInput = inputValue - self._lastInput

        # In order to prevent windup, only integrate if the process is not saturated
        if self._lastOutput < self._outputMax and self._lastOutput > self._outputMin:
            self._iTerm += self._Ki * error
            self._iTerm = min(self._iTerm, self._outputMax)
            self._iTerm = max(self._iTerm, self._outputMin)

        p = self._Kp * error
        i = self._iTerm
        d = -(self._Kd * dInput)

        # Compute PID Output
        self._lastOutput = p + i + d
        self._lastOutput = min(self._lastOutput, self._outputMax)
        self._lastOutput = max(self._lastOutput, self._outputMin)

        # Log some debug info
        self._logger.debug('P: {0}'.format(p))
        self._logger.debug('I: {0}'.format(i))
        self._logger.debug('D: {0}'.format(d))
        self._logger.debug('output: {0}'.format(self._lastOutput))

        # Remember some variables for next time
        self._lastInput = inputValue
        self._lastCalc = now
        return self._lastOutput

    def _currentTimeMs(self):
        return time.time() * 1000



