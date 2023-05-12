import time
import numpy
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE

print("Uploading firmware, please wait...")
vl53 = vl53l5cx.VL53L5CX()
print("Done!")
vl53.set_resolution(8 * 8)
vl53.enable_motion_indicator(8 * 8)
# vl53.set_integration_time_ms(50)

# Enable motion indication at 8x8 resolution
vl53.enable_motion_indicator(8 * 8)

# Default motion distance is quite far, set a sensible range
# eg: 40cm to 1.4m
vl53.set_motion_distance(400, 1400)

vl53.start_ranging()

while True:
    if vl53.data_ready():
        data = vl53.get_data()
        # 2d array of motion data (always 4x4?)
        # motion = numpy.flipud(numpy.array(data.motion_indicator.motion[0:16]).reshape((4, 4)))
        # 2d array of distance
        d = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
        # 2d array of reflectance
        # reflectance = numpy.flipud(numpy.array(data.reflectance).reshape((8, 8)))
        # 2d array of good ranging data
        #status = numpy.isin(numpy.flipud(numpy.array(data.target_status).reshape((8, 8))), (STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE))
        # motion, distance, reflectance,
        avg_distance = (d[2,2]+d[2,3]+d[2,4]+d[2,5]+d[3,2]+d[3,3]+d[3,4]+d[3,5]+d[4,2]+d[4,3]+d[4,4]+d[4,5]+d[5,2]+d[5,3]+d[5,4]+d[5,5])/16 
        
        print(d)
        print(avg_distance)
        print('d[4,4]=',d[4,4])
        print('d[3,3]=',d[3,3])
        

    time.sleep(0.1)
