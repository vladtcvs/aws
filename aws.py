#!/usr/bin/env python
#
# This is a weeWX driver to enable weeWX to read data from an Arduino.
#
# See here for more details:
#
# https://github.com/wrybread/ArduinoWeatherStation
# https://github.com/vladtcvs/ArduinoWeatherStation
#
# by wrybread@gmail.com
# by vtcendrovskii@gmail.com
#
#



"""Driver for Arduion Weather Station.

See here for more info:

https://github.com/vladtcvs/aws

"""

from __future__ import with_statement
import serial
import syslog
import time
import json

import weewx.drivers

DRIVER_NAME = 'AWS'
DRIVER_VERSION = '0.3'

def loader(config_dict, _):
    return AWSDriver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return AWSConfEditor()

INHG_PER_MBAR = 0.0295333727
METER_PER_FOOT = 0.3048
MILE_PER_KM = 0.621371

DEFAULT_PORT = '/dev/ttyUSB0'
DEBUG_READ = 0

def logmsg(level, msg):
    syslog.syslog(level, 'aws: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logwarn(msg):
    logmsg(syslog.LOG_WARNING, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

class AWSDevice(object):
    def __init__(self, port, id):
        self.port = port
        self.id = "%02d" % (int(id))

    def probe(self, nattempts = 1):
        cmd = "%s probe\n" % self.id
        for _ in range(nattempts):
            self.port.write(bytes(cmd, 'utf8'))
            resp = self.port.readline()
            if resp is None:
                continue
            resp = resp.decode('utf8').strip()
            if resp == ("%s response probe" % self.id):
                return True
        return False

    def reset(self):
        cmd = "%s reset\n" % self.id
        self.port.write(bytes(cmd, 'utf8'))
        time.sleep(20)
        return self.probe()

    def parse(self, response):
        response = response.strip()
        kvs = response.split()
        data = {}
        for sub in kvs:
            kv = sub.split(":")
            if len(kv) != 2:
                continue
            key = kv[0].strip()
            val = float(kv[1].strip())
            data[key] = val
        return data

    def measure(self, nattempts=1):
        cmd = "%s measure\n" % self.id
        rh = "%s response measure" % self.id
        for _ in range(nattempts):
            self.port.write(bytes(cmd, 'utf8'))
            resp = self.port.readline()
            if resp is None:
                continue
            resp = resp.decode('utf8').strip()
            if resp.startswith(rh):
                resp = resp[len(rh):].strip()
                return self.parse(resp)
        return None

class AWSDeviceManager(object):
    def __init__(self, port_names, device_ids, baud, timeout):
        self.port_names = port_names
        self.ids = device_ids
        self.devices = []
        self.ports = []
        self.timeout = timeout
        self.baud = baud

    def populate(self):
        for port_name in self.port_names:
            loginf('opening port %s' % port_name)
            try:
                port = serial.Serial(port_name, self.baud, timeout=self.timeout)
                port.flush()
            except Exception as e:
                logwarn('unable to open port %s, skipping' % port_name)
                continue

            time.sleep(20)
            port_used = False
            for id in self.ids:
                device = AWSDevice(port, id)
                if device.probe():
                    loginf('Found device id = %s on port %s' % (id, port))
                    self.devices.append({"id" : id, "device" : device})
                    port_used = True
            if port_used:
                self.ports.append(port)

    def measure(self, nattempts=1):
        responses = {}
        for dev in self.devices:
            id = dev["id"]
            device = dev["device"]
            res = device.measure(nattempts)
            if res is not None:
                responses[id] = res
        return responses

class SubstitutionManager(object):
    def __init__(self):
        self.tables = {}
    
    def add(self, id, original, target):
        if id not in self.tables:
            self.tables[id] = {}
        self.tables[id][original] = target

    def substitute(self, id, measurement):
        if id not in self.tables:
            return measurement
        subm = {}
        for key in measurement:
            if key not in self.tables[id]:
                subm[key] = measurement[key]
            else:
                subm[self.tables[id][key]] = measurement[key]
        return subm

class AWSDriver(weewx.drivers.AbstractDevice):
    """weewx driver that communicates with an Arduino weather station
    
    ports - serial ports
    [Required. Default is /dev/ttyUSB0]
    
    baud - baudrate
    [Required. Default is 9600]

    devices - devices
    [Required. Default is 01]

    polling_interval - how often to query the serial interface, seconds
    [Optional. Default is 2]

    max_tries - how often to retry serial communication before giving up
    [Optional. Default is 5]

    retry_wait - how long to wait, in seconds, before retrying after a failure
    [Optional. Default is 10]
    """
    def __init__(self, **stn_dict):
        self.port_names = stn_dict.get('ports', [DEFAULT_PORT])
        if isinstance(self.port_names, str):
            self.port_names = [self.port_names]

        self.baud = int(stn_dict.get('baud', 9600))

        self.device_ids = stn_dict.get('devices', [1])
        if isinstance(self.device_ids, str):
            self.device_ids = [self.device_ids]

        self.substitutions = SubstitutionManager()
        for key in stn_dict:
            if not key.startswith("substitute_"):
                continue
            keyn = key[len("substitute_"):]
            id      = keyn[:2]
            orig    = keyn[3:]
            target  = stn_dict[key]
            self.substitutions.add(id, orig, target)


        self.polling_interval = float(stn_dict.get('polling_interval', 4))
        self.max_tries = int(stn_dict.get('max_tries', 5))
        self.retry_wait = int(stn_dict.get('retry_wait', 10))
        self.last_rain = None
        loginf('driver version is %s' % DRIVER_VERSION)
        loginf('using serial port %s' % self.port_names)
        loginf('polling interval is %s' % self.polling_interval)
        global DEBUG_READ
        DEBUG_READ = int(stn_dict.get('debug_read', DEBUG_READ))

        self.last_read_time = time.time()
        self.read_counter = 0

        self.timeout = 2 # changed from 60
        self.device_mgr = AWSDeviceManager(self.port_names, self.device_ids, self.baud, self.timeout)
        self.device_mgr.populate()

        
    @property
    def hardware_name(self):
        return "AWS"


    def genLoopPackets(self):

        ntries = 0
        
        while ntries < self.max_tries:
            
            ntries += 1
            
            try:
                
                packet = {'dateTime': int(time.time() + 0.5),
                          'usUnits': weewx.METRIC}
                
                measurements = self.device_mgr.measure()
                for id in measurements:
                    val = measurements[id]
                    val_sub = self.substitutions.substitute(id, val)
                    packet.update(val_sub)
                
                self._augment_packet(packet)
                
                ntries = 0

                '''
                # print the time between reads and the count for debugging for now
                self.read_counter += 1
                time_since_last_read = time.time() - self.last_read_time
                print "%s seconds since last read" % time_since_last_read, self.read_counter
                self.last_read_time = time.time()
                '''
                
                yield packet

                if self.polling_interval:
                    time.sleep(self.polling_interval)
                    
            except (serial.serialutil.SerialException, weewx.WeeWxIOError) as e:
                logerr("Failed attempt %d of %d to get LOOP data: %s" %
                       (ntries, self.max_tries, e))
                time.sleep(self.retry_wait)
                
        else:
            msg = "Max retries (%d) exceeded for LOOP data" % self.max_tries
            logerr(msg)
            raise weewx.RetriesExceeded(msg)



    def _augment_packet(self, packet):

        # no wind direction when wind speed is zero
        if 'windSpeed' in packet and not packet['windSpeed']:
            packet['windDir'] = None

class AWSConfEditor(weewx.drivers.AbstractConfEditor):
    
    @property
    def default_stanza(self):
        return """
[AWS]
    # This section is for an Arduion Weather Station.

    # Serial port such as /dev/ttyACM0, /dev/ttyS0, /dev/ttyUSB0, or /dev/cuaU0
    port = /dev/ttyUSB0

    # The driver to use:
    driver = user.aws
"""

    def prompt_for_settings(self):
        print("Specify the serial port on which the station is connected, for")
        print("example /dev/ttyUSB0 or /dev/ttyACM0.")
        port = self._prompt('port', '/dev/ttyUSB0')
        return {'port': port}

