#!/usr/bin/env python

# example of running a slave thread that runs until it's called to stop

import threading
import time

class StoppableThread (threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__ (self):
        super(StoppableThread, self).__init__()
        self._stop = threading.Event()
        self.time_to_run = None

    def stop (self):
        print 'stop called'
        self._stop.set()

    def stopped (self):
        return self._stop.isSet()

    def set_time_to_run(self, time_to_run):
        self.time_to_run = time_to_run

    def run (self):
        if self.time_to_run != None:
            print 'running for %d seconds' % self.time_to_run
            counter = 0
            while not self.stopped() and counter < self.time_to_run:
                print 'slave running!'
                time.sleep(1)
                counter += 1
            print 'stopping'
            return
        else:
            print 'running until stopped'
            while self.stopped() != True:
                print 'slave running!'
                time.sleep(1)
            print 'stopping'
            return

if __name__ == '__main__':
    print 'master thread started'
    s = StoppableThread()
    time.sleep(1)
    print 'starting slave thread'
    s.start()
    print 'master thread sleeping for 5 seconds'
    time.sleep(5) 
    print 'attempting to stop slave'
    s.stop()
    time.sleep(1)
    print 'starting slave thread for 4 seconds'
    s = StoppableThread()
    s.set_time_to_run(4)
    s.start()
    s.join()
    print 'master thread done'
