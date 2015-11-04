from __future__ import print_function

import re
import sys
import time

from serial.tools.list_ports import comports
from myo_raw import MyoRaw
from common import *

# quick time function
current_milli_time = lambda: int(round(time.time() * 1000))

# Dual flag
global dual

# Detect BLE USB modems and return them
def detect_tty( ):
      detectedMyos = []
      for p in comports():
          if re.search(r'PID=2458:0*1', p[2]):
              detectedMyos.append(p[0])
            # print('Picked up device:', p[0])

      return list(detectedMyos)


def collect_myo_data(detectedMyos): 
    global dual
    
    if(detectedMyos is not None):

        # Closure function to record EMG data
        def proc_emg(timestamp, emg, moving, id):
            emgs = list([e / 2000. for e in emg])
            print (id, '--', timestamp, '--', emgs)
        
        # Set up first myo
        m = MyoRaw(detectedMyos[0], '1')
        m.add_emg_handler(proc_emg)
        m.connect()
        m.add_arm_handler(lambda arm, xdir: print('arm', arm, 'xdir', xdir))
        m.add_pose_handler(lambda p: print('pose', p))

        # Stop Myo from sleeping during data collection
        m.set_sleep_mode(1)
        m.vibrate(3)
        if(len(detectedMyos) == 2):
            m2 = MyoRaw(detectedMyos[1], '2')
            m2.add_emg_handler(proc_emg)
            m2.connect()

            # Stop myo from sleeping during data collection
            m2.set_sleep_mode(1)
            m2.vibrate(3)
            m2.add_arm_handler(lambda arm, xdir: print('arm', arm, 'xdir', xdir))
            m2.add_pose_handler(lambda p: print('pose', p))
            dual = True

        try:
            while True:
                if(dual == True):
                    m.run(1)
                    m2.run(1)
                else:
                    m.run(1)
        except KeyboardInterrupt:
            pass
        finally:
            m.disconnect()
            m2.disconnect()
            print()

if __name__ == "__main__":
    global dual
    dual = False
    detectedMyos = detect_tty()
    print (detectedMyos)
    collect_myo_data(detectedMyos)


