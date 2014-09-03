import csv
import fnmatch
import numpy as np
from numpy import linalg
import sys, getopt


### 
# functions
###
def calibrate(x, y, z):
    H = np.array([x, y, z, -y**2, -z**2, np.ones([len(x), 1])])
    H = np.transpose(H)
    w = x**2

    (X, residues, rank, shape) = linalg.lstsq(H, w)

    OSx = X[0] / 2
    OSy = X[1] / (2 * X[3])
    OSz = X[2] / (2 * X[4])

    A = X[5] + OSx**2 + X[3] * OSy**2 + X[4] * OSz**2
    B = A / X[3]
    C = A / X[4]

    SCx = np.sqrt(A)
    SCy = np.sqrt(B)
    SCz = np.sqrt(C)

    bias = np.array([OSx, OSy, OSz])
    scale = np.array([SCx, SCy, SCz])

    return bias, scale 

def read_logfile(filename, 
                 acc_column_names=['*RAWIMUxacc', '*RAWIMUyacc', '*RAWIMUzacc'],
                 mag_column_names=['*RAWIMUxmag', '*RAWIMUymag', '*RAWIMUzmag']):

    # accelerometers and magnetometers values
    acc = []
    mag = []

    #open file
    with open(filename, 'r') as f:
        # autodetect file format    
        log = csv.DictReader(f, dialect=csv.Sniffer().sniff(f.read(1000)))

        # go to first line
        f.seek(0)

        # get acc column names
        xacc_column = fnmatch.filter(log.fieldnames, acc_column_names[0]) 
        yacc_column = fnmatch.filter(log.fieldnames, acc_column_names[1]) 
        zacc_column = fnmatch.filter(log.fieldnames, acc_column_names[2])

        # get mag column names
        xmag_column = fnmatch.filter(log.fieldnames, mag_column_names[0]) 
        ymag_column = fnmatch.filter(log.fieldnames, mag_column_names[1]) 
        zmag_column = fnmatch.filter(log.fieldnames, mag_column_names[2])
        
        # fill values
        for row in log:
            # accelerometer
            if xacc_column and yacc_column and zacc_column:    
                acc.append([int(row[xacc_column[0]]),
                            int(row[yacc_column[0]]),
                            int(row[zacc_column[0]])])
            # magnetometer
            if xmag_column and ymag_column and zmag_column:    
                mag.append([int(row[xmag_column[0]]),
                           int(row[ymag_column[0]]),
                           int(row[zmag_column[0]])])
    
    # turn acc and mag into numpy arrays
    acc = np.array(acc)
    mag = np.array(mag)

    return acc, mag


def main(argv):
    usage = """usage:
            calib_imu.py -f <inputfile>"""

    filename = ''

    try:
        opts, args = getopt.getopt(argv,"hf:")
    except getopt.GetoptError:
        print(usage)
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print(usage)
            sys.exit(2)
        elif opt == '-f':
            filename = arg

    if not filename:
        print(usage)
        sys.exit(2)

    acc, mag = read_logfile(filename)

    acc_bias, acc_scale = np.zeros(3), np.zeros(3)
    mag_bias, mag_scale = np.zeros(3), np.zeros(3)    
        
    if acc.any():
        acc_bias, acc_scale = calibrate(acc[:,0], acc[:,1], acc[:,2])
    if mag.any():
        mag_bias, mag_scale = calibrate(mag[:,0], mag[:,1], mag[:,2])

    print('Acc bias: ')
    print(acc_bias)
    print('Acc scale: ')
    print(acc_scale)
    print('Mag bias: ')
    print(mag_bias)
    print('Mag scale: ')
    print(mag_scale)

if __name__ == '__main__':
    main(sys.argv[1:])