'''
Created on May 26, 2011

@author: kinzle_eric
'''

from BlobTracker import *

if __name__ == '__main__':
    print 'starting tracker'
    testBlob = BlobTracker()
    
    cords = testBlob.searchFrameForColorBlob(255, 100, 52, .15)
    print cords