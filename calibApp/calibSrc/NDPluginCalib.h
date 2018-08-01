#ifndef NDPluginCalib_H
#define NDPluginCalib_H

#include "NDPluginDriver.h"

#define CALIB_VERSION      1
#define CALIB_REVISION     2
#define CALIB_MODIFICATION 0

/* Output data type */
#define NDPluginCalibLowThresholdString       "LOW_THRESHOLD"     /* (asynFloat32, r/w) Canny sensitivity                      */
#define NDPluginCalibThresholdRatioString     "THRESHOLD_RATIO"   /* (asynFloat32, r/w) low threshold * ratio = high threshold   */
#define NDPluginCalibVerticalFoundString      "VERTICAL_FOUND"    /* (asynInt32,   r/o) 1 we found it, 0 not                      */
#define NDPluginCalibTopEdgeFoundString       "TOP_CALIB_FOUND"    /* (asynInt32,   r/o) 1 we found it, 0 not                      */
#define NDPluginCalibTopPixelString           "TOP_PIXEL"         /* (asynInt32,   r/o) index of pixel or -1 if not found        */
#define NDPluginCalibBottomEdgeFoundString    "BOTTOM_CALIB_FOUND" /* (asynInt32,   r/o) 1 we found it, 0 not                      */
#define NDPluginCalibBottomPixelString        "BOTTOM_PIXEL"      /* (asynInt32,   r/o) index of pixel or -1 if not found        */
#define NDPluginCalibVerticalCenterString     "VERTICAL_CENTER"   /* (asynFloat64, r/o) average of vertical positions            */
#define NDPluginCalibVerticalSizeString       "VERTICAL_SIZE"     /* (asynInt32,   r/o) Differance between top and bottom        */
#define NDPluginCalibHorizontalFoundString    "HORIZONTAL_FOUND"  /* (asynInt32,   r/o) 1 we found it, 0 we did not              */
#define NDPluginCalibLeftEdgeFoundString      "LEFT_CALIB_FOUND"   /* (asynInt32,   r/o) 1 of we found it, 0 if not               */
#define NDPluginCalibLeftPixelString          "LEFT_PIXEL"        /* (asynInt32,   r/o) index of pixel (-1 if not found          */
#define NDPluginCalibRightEdgeFoundString     "RIGHT_CALIB_FOUND"  /* (asynInt32,   r/o) 1 of we found it, 0 if not               */
#define NDPluginCalibRightPixelString         "RIGHT_PIXEL"       /* (asynInt32,   r/o) index of pixel (-1 if not found)         */
#define NDPluginCalibHorizontalCenterString   "HORIZONTAL_CENTER" /* (asynFloat64, r/o) average of horizontal positions          */
#define NDPluginCalibHorizontalSizeString     "HORIZONTAL_SIZE"   /* (asynInt32,   r/o) difference between left and right        */


/** Does image processing operations.
 */
class NDPluginCalib : public NDPluginDriver {
public:
    NDPluginCalib(const char *portName, int queueSize, int blockingCallbacks, 
                 const char *NDArrayPort, int NDArrayAddr,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize);
    /* These methods override the virtual methods in the base class */
    void processCallbacks(NDArray *pArray);
    
protected:
    
    /* edge sensitivity  */
    int NDPluginCalibLowThreshold;
    #define FIRST_NDPLUGIN_CALIB_PARAM NDPluginCalibLowThreshold

    /* threshold ratio (low * ratio = high, ratio is recommended to be 3)   */
    int NDPluginCalibThresholdRatio;

    /* 1 if vertical edge found (top and bottom found and are different     */
    int NDPluginCalibVerticalFound;

    /* if we found a top edge                                               */
    int NDPluginCalibTopEdgeFound;

    /* first edge position from the top  (-1 for not found)                 */
    int NDPluginCalibTopPixel;

    /* if we found a bottom edge                                            */
    int NDPluginCalibBottomEdgeFound;

    /* first edge position from the bottom  (-1 for not found)              */
    int NDPluginCalibBottomPixel;

    /* average of top and bottom                                            */
    int NDPluginCalibVerticalCenter;

    /* difference between top and bottom                                    */
    int NDPluginCalibVerticalSize;

    /* 1 if horizontal edit found (left and right found and are different   */
    int NDPluginCalibHorizontalFound;

    /* 1 if left edge found                                                 */
    int NDPluginCalibLeftEdgeFound;

    /* first edge position from left (-1 for not found)                     */
    int NDPluginCalibLeftPixel;

    /* 1 if right edge found                                                */
    int NDPluginCalibRightEdgeFound;

    /* first edge position from right (-1 for not found)                    */
    int NDPluginCalibRightPixel;

    /* average of left and right positions                                  */
    int NDPluginCalibHorizontalCenter;

    /* difference between left and right positions                          */
    int NDPluginCalibHorizontalSize;

private:

};
    
#endif
