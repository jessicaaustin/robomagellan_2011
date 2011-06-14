import cv
import sys

sys.setrecursionlimit(25000)
#capture = cv.CaptureFromCAM(0) #Insert camera reference

class BlobTracker :
    
    def __init__( self ) :
        self.capture = cv.CaptureFromCAM(0)
        
    ###########################################
    # Function: searchFrameForColorBlob
    # Purpose:  Look through the frame to look for the largest blob of a certain color
    # Parameters: image and RGB min and max values that define the color
    # Returns:  coordinates
    ############################################

    def searchFrameForColorBlob(self, idealRed, idealGreen, idealBlue, searchColorThreshold):
        img = cv.QueryFrame(self.capture)

            # Process Frame
        #rMin = 170
        #rMax = 255
        #gMin = 40
        #gMax = 150
        #bMin = 0
        #bMax = 80
        # create mask
        mask = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 1)
        
        # set mask to all zeroes
        cv.SetZero(mask)
        
        # Set height and width parameters of image
        totalWidth = img.__getattribute__("width")
        totalHeight = img.__getattribute__("height")
        
        #Initialize color threshold percentage
        #searchColorThreshold = .15 # .10 = 10%
        
        #Initialize step threshold percentage
        searchStepTreshold = .05 # 0.01 = 1% = 640x480 = 6.4 x 4.8
        
        #Initialize color Blob threshold percentage
        #blobColorTreshold = .2 # .05 = 5%
        
        # set up how many steps to skip based on searchStepThreshold and
        # total width and total height.  ex 640 width at 1% = 6 steps (6.4 rounded)
        widthStep = int(totalWidth * searchStepTreshold)
        heightStep = int(totalHeight * searchStepTreshold)
        
        # Set ideal color (orange = R255, G100, B60)
        #idealRed = 255
        #idealGreen = 100
        #idealBlue = 52
        
        # get upper and lower tolerances and store as array of values
        searchColorToleranceArray = createUpperLowerToleranceArray(idealRed, idealGreen, idealBlue, searchColorThreshold)
        
        #Debug Statement
        # (idealRed, idealGreen, idealBlue, rMax, rMin, gMax, gMin, bMax, bMin)
        #print "Color Tolerances: %d %d %d d% d% d% d% d% %d" % (searchColorToleranceArray[0], searchColorToleranceArray[1], searchColorToleranceArray[2], searchColorToleranceArray[3], searchColorToleranceArray[4], searchColorToleranceArray[5], searchColorToleranceArray[6], searchColorToleranceArray[7], searchColorToleranceArray[8])
        
        
        # initialize x and y coordinate variable to start
        xCord = 0  # default zero
        yCord = 0  # default zero
        
        # initialize groupSize (1 is 1 pixel, 2 is 9 pixels, 3 is 25
        groupSize = 2
        
        # initialize blob parameters
        biggestBlobSize = 0    # Total Pixels in the blob
        biggestBlobColor = 75  # Color for first blob in mask
        biggestBlobMeanX = 0   # Mean X position of Blob
        biggestBlobMeanY = 0   # Mean Y position of Blob
        biggestBlobParamArray = [mask, biggestBlobSize, biggestBlobColor, biggestBlobMeanX, biggestBlobMeanY]  # set up array with defaults
        
        # initialize array used to determine size of blob and carry the mask
        # blobParamArray is used to determine the total size of a blob and it's average x and y coordinates
        blobParamArray = [mask, 0, 0, 0]  # mask, totalPixels, xAvg, yAvg. 
        
        # determine mask color to start with and how much it goes up by
        maskColor = 25 # default set for 50.  Dark Gray
        maskColorStep = 1 # default set to 10.
            
        # go through image looking for the matches in color.  Each one it finds, start the grow blob process
        # Once complete, see if it is the biggest blob.  Retain only the biggest blob.  End when all of the image
        # is checked.
        # Loop through height
        while yCord < totalHeight:
            xCord = 0 # must re-initialize width here
            # Loop through width
            while xCord < totalWidth:
                # Recursive call to check surrounding pixels
                blobParamArray = findSurroundingPixelGroups(img, xCord, yCord, searchColorToleranceArray, maskColor, blobParamArray, 0, groupSize)
                # check to see if the blob is bigger than zero, meaning it found a usable blob
                if blobParamArray[1] > 0:
        
                    # check blob to see if it is bigger than the current biggest blob.  If bigger, then make this blob the new biggest blob
                    # the tie goes to the previous blob
                    if blobParamArray[1] > biggestBlobParamArray[1]:
                        # we have a new biggest blob
                        biggestBlobParamArray[0] = blobParamArray[0]
                        biggestBlobParamArray[1] = blobParamArray[1]
                        biggestBlobParamArray[2] = maskColor
                        biggestBlobParamArray[3] = blobParamArray[2]
                        biggestBlobParamArray[4] = blobParamArray[3]
                        #Debug statement:
                        #print 'New biggest blob. size:%d' % (biggestBlobParamArray[1])
                    
                    # Next because we found a valid blob, increment the blob
                    maskColor = maskColor + maskColorStep
                    
                    # Next change the blobWeightArray back to default
                    blobParamArray[1] = 0 # total pixels
                    blobParamArray[2] = 0 # xAvg
                    blobParamArray[3] = 0 # yAvg
                    
                # Now we iterate to the next xCord to test
                xCord = xCord + widthStep # add percent to width
            # Now we iterate to the next yCord to test
            yCord = yCord + heightStep
        
        #print 'Biggest Blob X Coord Avg at %d' % (biggestBlobParamArray[3])
        #print 'Biggest Blob Y Coord Avg at %d' % (biggestBlobParamArray[4])
        #print 'Biggest Blob Size is %d' % (biggestBlobParamArray[1])
        #print 'Biggest Blob Color is %d' % (biggestBlobParamArray[2])
        
        # Processing done... find offset from center
        # avg weight cord * 100 (for percent) / total width gives percent location of where in frame
        # take that and -50 (for center) * 2 (for percent away from center)
        print 'totalWidth = %d' % totalWidth
        print 'biggestBlobParamArray[3] = %d' % biggestBlobParamArray[3]
        percentWidthOffset = ( ( (biggestBlobParamArray[3] * 100) / totalWidth ) - 50 )
        percentHeightOffset = ( ( (biggestBlobParamArray[4] * 100) / totalHeight ) - 50 ) * -1
        # If result is -50,50, then make the worst case -100 -100
        if percentWidthOffset == -50 :
            if percentHeightOffset == 50 :
                trackingCoordinates = [-100, -100]
                return trackingCoordinates
        
        trackingCoordinates = [percentWidthOffset, percentHeightOffset]
        return trackingCoordinates
#        
#        if percentWidthOffset > 0:
#            print 'Offset X at %d to the right' % (percentWidthOffset)
#        else:
#            print 'Offset X at %d to the left' % (percentWidthOffset)
#        if percentHeightOffset > 0:
#            print 'Offset Y at %d percent up' % (percentHeightOffset)
#        else:
#            print 'Offset Y at %d percent down' % (percentHeightOffset)
#            
#        #color final image
#        # Loop through height
#        yCord = 0
#        while yCord < totalHeight:
#            xCord = 0 # must re-initialize width here
#            # Loop through width
#            while xCord < totalWidth:
#                maskCheckPixel = cv.Get2D(mask, yCord, xCord) # get maskImg color array from this pixel
#                maskCheckPixelValue = int(maskCheckPixel[0])
#                if maskCheckPixelValue == biggestBlobParamArray[2]:
#                    cv.Set2D(img, yCord, xCord, (255, 0, 0, 0)) 
#                xCord = xCord + 1
#            yCord = yCord + 1
            
#        # Create a Window
#        cv.NamedWindow("mask Picture", 1)
#        # Display the window
#        cv.ShowImage("mask Picture", blobParamArray[0])
#        
#        # Create a Window
#        cv.NamedWindow("image Picture", 1)
#        # Display the window
#        cv.ShowImage("image Picture", img)
#
#        testme = [trackingCoordinates, img]
#        return testme
#        #return trackingCoordinates
#        return img
    
    
    ##################################################
    # Function: 
    # Purpose:  
    # Notes:    
    # Returns:  
    ##################################################
#    def startCameraAndScan( self ) :
#        cv.NamedWindow("camera", 1)
#        capture = cv.CaptureFromCAM(0)
#        
#        # Get Frame
#        img = cv.QueryFrame(capture)
#
#        # Process Frame
#        processedImg = self.searchFrameForColorBlob(img, 170, 255, 40, 150, 0, 80)
#        
#        # show processed Image
#        cv.ShowImage("camera", processedImg)
#        
#        # If escape key, the break
#        if cv.WaitKey(10) == 27:
#            break
#            
#        cv.DestroyWindow("camera")

##################################################
# Function: createUpperLowerToleranceArray
# Purpose:  Create an array that holds the upper and lower 
#           tolerances of what color is required
# Returns:  Array (idealRed, idealGreen, idealBlue, rMax, rMin, gMax, gMin, bMax, bMin)
##################################################
def createUpperLowerToleranceArray(idealRed, idealGreen, idealBlue, searchColorThreshold):
    colorToleranceArray = [idealRed, idealGreen, idealBlue]
    
    colorErr = int(255 * searchColorThreshold)
    colorErrHalf = int( colorErr / 2)
    
    # Based on the color halves, calculate the tolerances
    redMax = idealRed + colorErrHalf
    if redMax > 255:
        redMax = 255
    colorToleranceArray.append(redMax)
    redMin = idealRed - colorErrHalf
    if redMin < 0:
        redMin = 0
    colorToleranceArray.append(redMin)
    greenMax = idealGreen + colorErrHalf
    if greenMax > 255:
        greenMax = 255
    colorToleranceArray.append(greenMax)
    greenMin = idealGreen - colorErrHalf
    if greenMin < 0:
        greenMin = 0
    colorToleranceArray.append(greenMin)
    blueMax = idealBlue + colorErrHalf
    if blueMax > 255:
        blueMax = 255
    colorToleranceArray.append(blueMax)
    blueMin = idealBlue - colorErrHalf
    if blueMin < 0:
        blueMin = 0
    colorToleranceArray.append(blueMin)
    
    return colorToleranceArray

#######################################
# Function: setGroupInMask
# Purpose:  set a group of pixels in a mask to a certain value
# Returns:  mask image
#######################################
def setGroupInMask(xCord, yCord, mask, pixelValue, groupSize):
    x = 0 - groupSize
    y = 0 - groupSize
    while (x <= groupSize):
        y = 0 - groupSize
        while (y <= groupSize):
            cv.Set2D(mask, (yCord + y), (xCord + x), pixelValue)
            y = y + 1
        x = x + 1
    return mask
     

#######################################
# Function: isCoordGroupInBounds
# Purpose:  test if x and y are within image bounds (factoring in groupSize)
# Returns:  True if in bounds
#           False if out of bounds
#######################################
def isCoordGroupInBounds(img, xCord, yCord, groupSize):
    if (xCord - groupSize) < 0 or (xCord + groupSize) >= img.__getattribute__("width") :
        return False
    if (yCord - groupSize) < 0 or (yCord + groupSize) >= img.__getattribute__("height") :
        return False
    return True


###########################################
# Function: checkMaskForValue
# Inputs:   maskImg(iplimage) the mask to test
#           x and y cord to determine if point is filled or not
# Purpose:  provide a mask and check an x/y coordinate to determine if
#           a value exists
# Returns:  True if NO value exists
#           False if value exists
############################################
def isMaskPointEmpty(maskImg, xCord, yCord):
    # check to see if location is already in mask
    #print "x and y= %d,%d" % (xCord, yCord) #DEBUG
    maskCheckPixel = cv.Get2D(maskImg, yCord, xCord) # get maskImg color array from this pixel
    maskCheckPixelValue = int(maskCheckPixel[0]) # get color value from this pixel array
    if maskCheckPixelValue == 0 : # Check if no this mask is empty. (0)
        return True
    return False


############################################
# Function: checkColorThreshold
# Purpose:  take image, colorToleranceArray and an averageArray and determine
#           if this pixel is within tolerances
# Returns:  True if within threshold, False if any color is
#           outside of threshold.
############################################
def isColorWithinThreshold(colorToleranceArray, colorGroupAverage):
    # Note: format of colorToleranceArray is as follows:
    # (idealRed, idealGreen, idealBlue, rMax, rMin, gMax, gMin, bMax, bMin)
    redPx = colorGroupAverage[0]
    greenPx = colorGroupAverage[1]
    bluePx = colorGroupAverage[2]
    
    # if redColor < red tolerance min AND redColor > red tolerance max
    if redPx >= 170 and redPx <= 255:
        # if blueColor > blue tolerance min AND blueColor < blue tolerance max
        if bluePx >= 0 and bluePx <= 128:
            # if greenColor > green tolerance min AND greenColor < green tolerance max
            if greenPx >= 40 and greenPx <= 175:
                # all 3 colors within tolerance:  return true
                return True;
    
#    # if redColor < red tolerance min AND redColor > red tolerance max
#    if redPx >= colorToleranceArray[4] and redPx <= colorToleranceArray[3]:
#        # if blueColor > blue tolerance min AND blueColor < blue tolerance max
#        if bluePx >= colorToleranceArray[8] and bluePx <= colorToleranceArray[7]:
#            # if greenColor > green tolerance min AND greenColor < green tolerance max
#            if greenPx >= colorToleranceArray[6] and greenPx <= colorToleranceArray[5]:
#                # all 3 colors within tolerance:  return true
#                return True;
    # one of the colors was outside of threshold.  Return false
    return False

############################################
# Function: averageGroupPixels
# Purpose:  get the average color of the group of pixels
# Returns:  Array (RGB color values)
############################################
def averageGroupPixels(img, xCord, yCord, groupSize):
    # Note: format of colorToleranceArray is as follows:
    # (idealRed, idealGreen, idealBlue, rMax, rMin, gMax, gMin, bMax, bMin)
    # Get2D is in this format: (B,G,R,A) Blue, Green, Red, Alpha
    x = 0 - groupSize
    y = 0 - groupSize
    index = 0
    rAvg = 0;
    gAvg = 0;
    bAvg = 0;
    redArray = []
    greenArray = []
    blueArray = []
    
    while (x <= groupSize):
        y = 0 - groupSize
        while (y <= groupSize):
            index = index + 1
            # get pixel value
            colorArray = cv.Get2D(img, (yCord + y), (xCord + x))
            blueArray.append(int(colorArray[0]))
            greenArray.append(int(colorArray[1]))
            redArray.append(int(colorArray[2]))
            y = y + 1
        x = x + 1
    for r in redArray:
        rAvg = rAvg + r
    for g in greenArray:
        gAvg = gAvg + g
    for b in blueArray:
        bAvg = bAvg + b
    rAvg = int(rAvg / index)
    bAvg = int(bAvg / index)
    gAvg = int(gAvg / index)
    colorGroupAverage = [rAvg, gAvg, bAvg]
    return colorGroupAverage     
    

###########################################
# Function: testSurroundingPixels
# Purpose:  check this pixel and then recursive call to surrounding pixels to
#           see if they belong to this blob.  While exiting recursive call,
#           calculate blob weight (average in x,y coordinates)
# Inputs:   img = the actual image (cone)
#           xCord, yCord = int, the coordinates in pixels of x and y with upper left as 0,0
#           colorTolerance = Array (idealRed, idealGreen, idealBlue, rMax, rMin, gMax, gMin, bMax, bMin)
#           maskValue = the int value from 0-255 to determine which shade is mask
#           blobParamArray = [mask, totalPixels, xAvg, yAvg] mask is the imageMask.  Total pixels is all pixels, not just centers
#           recuriveCount = int, counting how deep into recursion the function has traveled
#           groupSize = int, how big the group of pixels to be tested is.  1=1, 2=9, 3=25
# Returns:  blobWeightArray = [mask, totalPixels, xAvg, yAvg]
############################################
def findSurroundingPixelGroups(img, xCord, yCord, colorTolerance, maskValue, blobParamArray, recuriveCount, groupSize):
    # STEP ONE, Test if coordinates are in bounds (inside the frame of an image.)
    # STEP TWO, see if pixels are already in in mask or not
    # STEP THREE, calculate this group's average RGB values
    # STEP FOUR, see if color average values falls within range
    # STEP FIVE, Add new pixel(s) to mask
    # STEP SIX, calculate and update total number of pixels in blob
    # STEP SEVEN, calculate and update the average center of blob
    # STEP EIGHT, update array with new mask and calculations
    
    recuriveCount = recuriveCount + 1 # used to determine how deep inside a recursive function we are
     
    # STEP ONE, Test if coordinates are in bounds (inside the frame of an image.)
    if isCoordGroupInBounds(img, xCord, yCord, groupSize) == False:
        # Cords x or y are out of bounds
        # Just return everything as is (blobParamArray) (lower recursion by 1)
        recuriveCount = recuriveCount - 1
        return blobParamArray
        
    # STEP TWO, see if pixels are already in in mask or not
    mask = blobParamArray[0]
    if isMaskPointEmpty(mask, xCord, yCord) == False:
        # No need to calculate more average, primary pixel is taken
        # Just return everything as is (blobParamArray)
        recuriveCount = recuriveCount - 1
        return blobParamArray
    
    # STEP THREE, calculate this group's average RGB
    colorGroupAverage = averageGroupPixels(img, xCord, yCord, groupSize)
    
    # STEP FOUR, see if color average falls within range.  If not, return original blobParamArray
    if isColorWithinThreshold(colorTolerance, colorGroupAverage) == False:
        # Color does not match, so break out of loop and return recursion.
        recuriveCount = recuriveCount - 1
        return blobParamArray
    
    # STEP FIVE, Add new pixel(s) to mask
    # Add this pixel to mask
    mask = setGroupInMask(xCord, yCord, mask, maskValue, groupSize)
    
    # STEP SIX, calculate and update total number of pixels in blob
    # get total number in a group size (1 = 9, 2 = 25 ...etc)
    # Note, it is Amount up, amount down plus 1 for middle
    totalNumberOfGroupPixels = (groupSize + groupSize + 1) * (groupSize + groupSize + 1)

    # STEP SEVEN, calculate and update the average center of blob
    # Make calculations with THIS pixel and keep passing the array forward
    # calculate average with this pixel
    # Note: blobParamArray = mask, totalPixels, xAvg, yAvg
    totalPixels = blobParamArray[1] # pulling data from array object so it is easier to follow code
    xAvg = blobParamArray[2] # pulling data from array object so it is easier to follow code
    yAvg = blobParamArray[3] # pulling data from array object so it is easier to follow code
    
    # Calculate X Average
    if xAvg == 0: # if there is no blob yet, then the coordinate becomes the center
        xAvg = float(xCord)
    else:
        xAvg = float(((xCord * totalNumberOfGroupPixels) + (xAvg * totalPixels)) / (totalNumberOfGroupPixels + totalPixels))
    
    # if there is no blob yet, then the coordinate becomes the center
    if yAvg == 0:
        yAvg = float(yCord)
    else:
        yAvg = float(((yCord * totalNumberOfGroupPixels) + (yAvg * totalPixels)) / (totalNumberOfGroupPixels + totalPixels))
    
    # Add new group of pixels to total pixels
    totalPixels = totalPixels + totalNumberOfGroupPixels
    
    # STEP EIGHT, update array with new mask and calculations
    blobParamArray = [mask, totalPixels, xAvg, yAvg]
    
    # in order to skip over other pixels due to groupSize, make groupSize * 2 + 1,
    # so the center in groupSize (2=25) would be groupSize 2*2+1 = so the next center will be 5 pixels over.
    dblGroupSize = groupSize * 2 + 1
    
    # Recursively call test again on each pixel
    # Group ONE - Up
    blobParamArray = findSurroundingPixelGroups(img, xCord, (yCord - dblGroupSize), colorTolerance, maskValue, blobParamArray, recuriveCount, groupSize)
    
    # Group TWO - Right
    blobParamArray = findSurroundingPixelGroups(img, (xCord + dblGroupSize), yCord, colorTolerance, maskValue, blobParamArray, recuriveCount, groupSize)
    
    # Group Three - Down
    blobParamArray = findSurroundingPixelGroups(img, xCord, (yCord + dblGroupSize), colorTolerance, maskValue, blobParamArray, recuriveCount, groupSize)
    
    # Group FOUR - Left
    blobParamArray = findSurroundingPixelGroups(img, (xCord - dblGroupSize), yCord, colorTolerance, maskValue, blobParamArray, recuriveCount, groupSize)
    
    # At this point, we checked THIS GROUP, and all Manhattan GROUPS and
    # there are no more new groups to grow.  Send back down.
    recuriveCount = recuriveCount - 1
    return blobParamArray
            

        