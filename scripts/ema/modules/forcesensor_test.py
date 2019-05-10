import time
import serial

class ForceSensor:
    def __init__(self, config_dict):
        self.config_dict = config_dict
        self.devices = {}
        self.dongles = []
        self.sensors = []
        self.serialport = serial.Serial()

        for name in config_dict['dev_names']:
            dev_type = config_dict['dev_type'][name]

            if dev_type == 'WRD':
                wired_port = config_dict['wired_port'][name]
                self.serialport = serial.Serial( port=wired_port,
                                                 baudrate = 2000000,
                                                 timeout=0.001 )
                self.devices[name] = {}
                self.devices[name]['calibration'] = config_dict['calibration'][name]
                self.devices[name]['bias'] = 0.0
                self.sensors.append(name)                

            elif dev_type == 'DNG':
                wired_port = config_dict['wired_port'][name]
                self.devices[name] = serial.Serial( port=wired_port,
                                                    baudrate = 115200,
                                                    parity=serial.PARITY_NONE,
                                                    stopbits=serial.STOPBITS_ONE,
                                                    bytesize=serial.EIGHTBITS,
                                                    timeout=1 )
                self.dongles.append(name)

            elif dev_type == 'WL':
                self.devices[name] = {'wireless_dng': config_dict['wireless_dng'][name],
                                      'wireless_id': config_dict['wireless_id'][name],
                                      'calibration': config_dict['calibration'][name],
                                      'bias': 0}
                self.sensors.append(name)

        self.initialize()

    def initialize(self):
        print 'initializing'
        self.serialport.write('s')
        time.sleep(1.5) # delay until it actually starts

        for name in self.sensors:
            self.debias(name)

    def shutdown(self):
        print 'shutting down'
        self.serialport.write('f')
        self.serialport.close()

    def debias(self,name):
        # print 'debiasing'
        n = 5
        measurements = 0.0
        
        for i in range(0,n):
            f = self.getForce(name)
            measurements = measurements + f[1]
        
        bias = measurements/n
        self.devices[name]['bias'] = bias

    def getForce(self, name):
        # print 'getting force from ' + name
        calibration = self.devices[name]['calibration']
        bias = self.devices[name]['bias']
        msg = None

        while not msg:
            msg = self.serialport.readline()

        force = float(msg)*calibration - bias
        force = float("{0:.2f}".format(force))
        return [0.0,force,0.0]

    # def getForce(self, name):
    #     # print "getting ", name
        
    #     wireless_dng = self.devices[self.devices[name]['wireless_dng']] # we need the actual Serial object, not the dng name
    #     wireless_id = self.devices[name]['wireless_id']
    #     calibration = self.devices[name]['calibration']
    #     bias = self.devices[name]['bias']
        
    #     msg = wireless_dng.readline()
    #     msg_id = ""
        
    #     while msg_id != wireless_id:
    #         while len(msg) == 0:
    #             # print "len(force) == 0"
    #             msg = wireless_dng.readline()
                
    #         msg_id = msg[0]
        
    #         if msg_id == wireless_id:
    #             break
    #         else:
    #             # print "msg_id != %s (%s), reading again" % (wireless_id, msg_id)
    #             msg = wireless_dng.readline()
        
    #     force = int(msg[1:])*calibration - bias
    #     # print name, " = " , "{0:.3f}".format(force)
        
    #     return [0.0,force,0.0]

if __name__ == '__main__':
    
    dict_s = {'dev_names': ['loadcell'],
              'dev_type': {'loadcell': 'WRD'},
              'wired_port': {'loadcell': '/dev/ttyACM0'},
              'calibration': {'loadcell': 1.0}
              }

    fs_manager = ForceSensor(dict_s)

    for i in range(0,200):
        for name in fs_manager.sensors:
            f = fs_manager.getForce(name)
            print f

    fs_manager.shutdown()
