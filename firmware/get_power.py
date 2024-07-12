import asyncio
import numpy as np
from bleak import BleakClient
import time

window = 5      # get average cadence over x seconds
queue = []      # holds x periods (times per revolution) 
prev_time = 0   # previous "last crank event time" value
prev_rev = 0    # previous "cumulative crank revolutions" value

coast_drop = True # how cadence readings react if you don't pedal:
                  # if True, cadence artificially decreases to 0 
                  # if False, cadence holds its last value, and then drops to 0 after x seconds
no_rev = 1        # counter for dropping cadence

# start = 0
# end = 0

'''
(info from these links: 
https://stackoverflow.com/questions/54427537/understanding-ble-characteristic-values-for-cycle-power-measurement-0x2a63
https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_measurement.xml
https://stackoverflow.com/questions/49813704/cycling-speed-and-cadence-sensors-ble-gatt-characteristic-data-parsing-and-oper
)

how "Cycling Power Measurement" characteristic is read:

ex. L = [32, 0, 0, 0, 185, 88, 23, 186]
from right to left, starting at byte 0:
- bytes 0 and 1 are the Flags field. flip them and express in binary, so 32, 0 -> 00000000 00100000
  the 1 means "Crank Revolution Data Present", so the only data you get from the pedals
  is instantaneous power (bytes 2-3) and crank revolution data (bytes 4-7).
- bytes 3 and 2 are instantaneous power in watts (a 16-bit integer)
- bytes 5 and 4 are Cumulative Crank Revolutions 
- bytes 7 and 6 are Last Crank Event Time "in seconds with a resolution of 1/1024" (ie. the unit is 1/1024 seconds)

last crank event time overflows every 65536/1024 = 64 seconds
'''
def get_cadence(new_rev, new_time):
    global window, queue, prev_time, prev_rev, no_rev

    period = 1000
    if (prev_time != new_time): # if there's been a revolution

        # get time diff, fix overflow if needed
        if (prev_time > new_time):
            time_diff = (65535 - prev_time) + new_time + 1
        else:
            time_diff = new_time - prev_time
        
        # get rev diff, fix overflow if needed
        if (prev_rev > new_rev):
            rev_diff = (65535 - prev_rev) + new_rev + 1
        else:
            rev_diff = new_rev - prev_rev
        
        # get period (time per rev)
        period = time_diff/rev_diff
        
        no_rev = 1 # reset no rev counter
        queue.append(period) # queue holds the past few periods
    else: # no rev
        no_rev += 1
    
    if (no_rev > window + 1 or period < 5): # if we haven't been pedalling in approx x seconds or if period<0.005 s
        cadence = 0
        queue = []
    else:
        if (len(queue) > window): # remove old period
            queue.pop(0)
        avg_period = sum(queue) / len(queue) / 1024 # averaged seconds per revolution
        cadence = 1/avg_period * 60 # revolutions per minute
        if (coast_drop):
            cadence = cadence/no_rev # gets artificial drop in cadence if we don't pedal 

    prev_time = new_time
    prev_rev = new_rev
    return cadence

def notification_callback(sender, data): # gets data every ~1 second
    global window, queue, prev_time, no_rev
    # global start,end
    # start = end
    # end = time.time()
    # print(end - start)

    L = list(data) # "Cycling Power Measurement" characteristic
    #print(L)
    power = L[3] * 2**8 + L[2] # concatenating the bytes
    total_revs = L[5] * 2**8 + L[4]
    new_time = L[7] * 2**8 + L[6]
    cadence = get_cadence(total_revs, new_time)
    
    # print("power (W):", power)
    # print("cumulative crank revolutions:", total_revs)
    # print("last crank event time (s):", new_time/1024)
    # print("cadence (rpm):", round(cadence,2))

    print("power (W):", power, " cadence (rpm):", round(cadence,2))
    
async def main():
    address = "0F190F5F-30CD-BF3C-7F90-EED38CBA0CDC" 
    power_uuid = "00002a63-0000-1000-8000-00805f9b34fb"

    async with BleakClient(address) as client:
        # check if connection was successful
        print(f"Client connection: {client.is_connected}") # prints True or False

        await client.start_notify(power_uuid, notification_callback)

        # collect data for X seconds
        await asyncio.sleep(1000.0)
asyncio.run(main())