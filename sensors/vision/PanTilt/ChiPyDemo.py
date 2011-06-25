import time
import sys
from PanTilt import *
from BlobTrackerWithVisualOutput import *

print "Connecting to PanTilt"
pt = PanTilt('/dev/ttyUSB0')
pt.setSpeed(500)
print "Connecting to BlobTracker"
bt = BlobTrackerWithVisualOutput()

print "Exercising PanTilt"
pt.exerciseCamera()
pt.centerCamera()

redComponent = 255
greenComponent = 100
blueComponent = 52
scalingFactor = 5
threshold = 0.15
notFound = [-100, -100]

topLeft = notFound

cv.NamedWindow("Blob Image", 1)

print "Calibrating"
while (topLeft == notFound):
	print "No blob found yet"
	print " Trying again"
	blobList = bt.searchFrameForColorBlob(redComponent,
										  greenComponent,
										  blueComponent,
										  threshold)
	topLeft = blobList[0]
	imageObject = blobList[1]
	if (imageObject != None):
		cv.ShowImage("Blob Image", imageObject)
		cv.WaitKey(10)

print "Found the blob at %d, %d" % (topLeft[0], topLeft[1])

print "Entering tracking loop"
while True:
	blobList = bt.searchFrameForColorBlob(redComponent,
										  greenComponent,
										  blueComponent,
										  threshold)
	topLeft = blobList[0]
	imageObject = blobList[1]

	if (imageObject != None):
		cv.ShowImage("Blob Image", imageObject)
		cv.WaitKey(10)

	if (topLeft == notFound):
		continue

	xCoordinate = int(topLeft[0] / scalingFactor)
	yCoordinate = int(topLeft[1] / scalingFactor)

	print "Adjusting to %d, %d" % (xCoordinate, yCoordinate)
	pt.aimCamera(xCoordinate, yCoordinate)

	time.sleep(1.0)

sys.exit(0)
