from time import strftime
import serial
import sys
import datetime

def imu_from_col(str):
    if str[1] != ',':
        return int(str[:2])
    
    return int(str[0])

def main():
    if len(sys.argv) != 2:
        exit(f"Usage: python offline_calibration.py <PORT>\nExample: python3 offline_calibration.py COM3")

    port = sys.argv[1];
    # Serial connection
    serialConn = serial.Serial(port, 115200)
    print("Running...");

    current_imu = -1
    now_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    data_dec = ''
    f = False
    while True:
        data = serialConn.readline()
        data_dec = data.decode("UTF-8").rstrip() + '\n'
        if data_dec[0] == "{":
            if not f.closed:
                f.close()
            break

        imu = imu_from_col(data_dec)
        if imu != current_imu:
            current_imu = imu
            if f and not f.closed:
                f.close()
            f = open(f'calib{current_imu}_{now_str}.csv', 'w')
            f.write("imu,it,ex,ey,ez,ax,ay,az,temp\n")

        print(data_dec)
        f.write(data_dec)

    f.close()
    serialConn.close()

if __name__ == '__main__':
    main()
