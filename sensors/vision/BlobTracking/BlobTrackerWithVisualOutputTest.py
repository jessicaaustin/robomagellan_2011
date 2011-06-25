'''
Created on May 26, 2011

@author: kinzle_eric
'''
import cv

from BlobTrackerWithVisualOutput import *

if __name__ == '__main__':
    counter = 0;
    while(counter < 10) :
        print 'starting tracker'
        testBlob = BlobTrackerWithVisualOutput()
        
        cords = testBlob.searchFrameForColorBlob(255, 100, 52, .15)
        
        print cords
        cv.NamedWindow("image Picture", 1)
        # Display the window
        cv.ShowImage("image Picture", cords[1])
        cv.WaitKey(0);
        counter = counter+ 1