from imaplib import _CommandResults
import multiprocessing
from multiprocessing.dummy import Process
from multiprocessing import Pipe

from Communications.communications.parent import parent_proc

UAV, comms = Pipe()

def wait_for_rover(args):
    #wait for a recv from the rover suggesting its done
    UAV.recv()

function_set = {
    #"EXECUTION": wait_for_rover,
    "EX_DONE": comms.send("PICKLE")
}

def main():
    


    communications = Process(target=parent_proc, args=("192.168.4.3",7777, "192.168.4.10", 7676, function_set))
    communications.start()


    communications.join()

if __name__ == "__main__":
    main()