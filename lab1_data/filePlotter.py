# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.legend()
    plt.grid()
    plt.show()
    
import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)




# TODO Part 5: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    ... # just unpack yaw
    return yaw