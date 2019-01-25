#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.

my_vehicle.py:

Custom Vehicle subclass to add IMU data.
"""

from dronekit import Vehicle


class RawIMU(object):
    """
    The RAW IMU readings for the usual 9DOF sensor setup. 
    This contains the true raw values without any scaling to allow data capture and system debugging.
    
    The message definition is here: https://mavlink.io/en/messages/common.html#RAW_IMU
    
    :param time_boot_us: Timestamp (microseconds since system boot). #Note, not milliseconds as per spec
    :param xacc: X acceleration (mg)
    :param yacc: Y acceleration (mg)
    :param zacc: Z acceleration (mg)
    :param xgyro: Angular speed around X axis (millirad /sec)
    :param ygyro: Angular speed around Y axis (millirad /sec)
    :param zgyro: Angular speed around Z axis (millirad /sec)
    :param xmag: X Magnetic field (milli tesla)
    :param ymag: Y Magnetic field (milli tesla)
    :param zmag: Z Magnetic field (milli tesla)    
    """
    def __init__(self, time_boot_us=None, xacc=None, yacc=None, zacc=None, xygro=None, ygyro=None, zgyro=None, xmag=None, ymag=None, zmag=None):
        """
        RawIMU object constructor.
        """
        self.time_boot_us = time_boot_us
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc
        self.xgyro = zgyro
        self.ygyro = ygyro
        self.zgyro = zgyro
        self.xmag = xmag        
        self.ymag = ymag
        self.zmag = zmag      
        
    def __str__(self):
        """
        String representation used to print the RawIMU object. 
        """
        return "RAW_IMU: time_boot_us={},xacc={},yacc={},zacc={},xgyro={},ygyro={},zgyro={},xmag={},ymag={},zmag={}".format(self.time_boot_us, self.xacc, self.yacc,self.zacc,self.xgyro,self.ygyro,self.zgyro,self.xmag,self.ymag,self.zmag)

# サーボ出力値をモニタリングする。
# Example: create_attribute.py my_vehicle.py内のRawIMUクラスを参考に作る。
#
class ServoOutputRaw(object):
    def __init__(self, port=None, sv1_raw=None, sv2_raw=None, sv3_raw=None, sv4_raw=None, sv5_raw=None, sv6_raw=None, sv7_raw=None, sv8_raw=None, sv9_raw=None, sv10_raw=None, sv11_raw=None, sv12_raw=None, sv13_raw=None, sv14_raw=None, sv15_raw=None, sv16_raw=None ):
        '''
        constructor.
        '''
        self.port = port
        self.sv1_raw = sv1_raw
        self.sv2_raw = sv2_raw
        self.sv3_raw = sv3_raw
        self.sv4_raw = sv4_raw
        self.sv5_raw = sv5_raw
        self.sv6_raw = sv6_raw
        self.sv7_raw = sv7_raw
        self.sv8_raw = sv8_raw
        self.sv9_raw = sv9_raw
        self.sv10_raw = sv10_raw
        self.sv11_raw = sv11_raw
        self.sv12_raw = sv12_raw
        self.sv13_raw = sv13_raw
        self.sv14_raw = sv14_raw
        self.sv15_raw = sv15_raw
        self.sv16_raw = sv16_raw

    def __str__(self):
        '''
        cast to string
        '''
        #return "SERVO_OUTPUT_RAW: sv1={}, sv2={}, sv3={}, sv4={}".format( self.sv1_raw, self.sv2_raw, self.sv3_raw, self.sv4_raw )
        return "SERVO_OUTPUT_RAW: sv5={}, sv6={}, sv7={}, sv8={}".format( self.sv5_raw, self.sv6_raw, self.sv7_raw, self.sv8_raw )



   
class MyVehicle(Vehicle):
    def __init__(self, *args):
        super(MyVehicle, self).__init__(*args)

        # Create an Vehicle.raw_imu object with initial values set to None.
        self._raw_imu = RawIMU()

        # Create an Vehicle.servo_output_raw object with initial values set to None.
        self._servo_output_raw = ServoOutputRaw()

        # Create a message listener using the decorator.   
        @self.on_message('RAW_IMU')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string specified in the decorator,
            passing the vehicle, message name, and the message.
            
            The listener writes the message to the (newly attached) ``vehicle.raw_imu`` object 
            and notifies observers.
            """
            self._raw_imu.time_boot_us=message.time_usec
            self._raw_imu.xacc=message.xacc
            self._raw_imu.yacc=message.yacc
            self._raw_imu.zacc=message.zacc
            self._raw_imu.xgyro=message.xgyro
            self._raw_imu.ygyro=message.ygyro
            self._raw_imu.zgyro=message.zgyro
            self._raw_imu.xmag=message.xmag
            self._raw_imu.ymag=message.ymag
            self._raw_imu.zmag=message.zmag
            
            # Notify all observers of new message (with new value)
            #   Note that argument `cache=False` by default so listeners
            #   are updated with every new message
            self.notify_attribute_listeners('raw_imu', self._raw_imu) 

        # Create a message listener using the decorator.   
        @self.on_message('SERVO_OUTPUT_RAW')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string specified in the decorator,
            passing the vehicle, message name, and the message.
            
            The listener writes the message to the (newly attached) ``vehicle.servo_output_raw`` object 
            and notifies observers.
            """
            self._servo_output_raw.port 	= message.port
            self._servo_output_raw.sv1_raw 	= message.servo1_raw
            self._servo_output_raw.sv2_raw 	= message.servo2_raw
            self._servo_output_raw.sv3_raw 	= message.servo3_raw
            self._servo_output_raw.sv4_raw 	= message.servo4_raw
            self._servo_output_raw.sv5_raw 	= message.servo5_raw
            self._servo_output_raw.sv6_raw 	= message.servo6_raw
            self._servo_output_raw.sv7_raw 	= message.servo7_raw
            self._servo_output_raw.sv8_raw 	= message.servo8_raw
            self._servo_output_raw.sv9_raw 	= message.servo9_raw
            self._servo_output_raw.sv10_raw = message.servo10_raw
            self._servo_output_raw.sv11_raw = message.servo11_raw
            self._servo_output_raw.sv12_raw = message.servo12_raw
            self._servo_output_raw.sv13_raw = message.servo13_raw
            self._servo_output_raw.sv14_raw = message.servo14_raw
            self._servo_output_raw.sv15_raw = message.servo15_raw
            self._servo_output_raw.sv16_raw = message.servo16_raw
            
            # Notify all observers of new message (with new value)
            #   Note that argument `cache=False` by default so listeners
            #   are updated with every new message
            self.notify_attribute_listeners('servo_output_raw', self._servo_output_raw) 
    @property
    def raw_imu(self):
        return self._raw_imu

    @property
    def servo_output_raw(self):
        return self._servo_output_raw
