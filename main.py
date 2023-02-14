from imaplib import _CommandResults
import multiprocessing
from multiprocessing.dummy import Process
from multiprocessing import Pipe
from xml.etree.ElementTree import tostring
import gps_file as gps

from Communications.communications.parent import parent_proc

UAV, comms = Pipe()

def wait_for_rover(args):
    #wait for a recv from the rover suggesting its done
    UAV.recv()

def send_location(gps_coor):
    comms.send(gps_coor)

def calibrate_coordinates(gps_coor):
    #do some math!!!
    latitude = gps_coor[0]
    longitude = gps_coor[1]
    gps_string = tostring(latitude) + " " + tostring(longitude)
    send_location(gps_string)
    
    
function_set = {
    "EX_DONE": comms.send("PICKLE")
}

def main():

    communications = Process(target=parent_proc, args=("192.168.4.3",7777, "192.168.4.10", 7676, function_set))
    communications.start()
    #if red square is found: 
        #gps_coor = gps.gps_setup()
        #calibrate_coordinates(gps_coor)
        


    communications.join()

if __name__ == "__main__":
    main()