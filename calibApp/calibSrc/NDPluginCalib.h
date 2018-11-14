#ifndef NDPluginCalib_H
#define NDPluginCalib_H

#include <epicsTypes.h>
#include <asynStandardInterfaces.h>

#include <vector>
#include <opencv2/opencv.hpp>

#include "NDPluginDriver.h"

#define CALIB_VERSION      1
#define CALIB_REVISION     2
#define CALIB_MODIFICATION 0

/* Input data */
#define NDPluginCalibLowThresholdString       "LOW_THRESHOLD"     /* (asynFloat32, r/w) Canny sensitivity                      */
#define NDPluginCalibThresholdRatioString     "THRESHOLD_RATIO"   /* (asynFloat32, r/w) low threshold * ratio = high threshold   */
#define NDPluginCalibShowImageString          "SHOW_IMAGE"        // variable to check if the transformed picture should be displayed or not

/* Output data */
#define NDPluginCalibFitX_aString             "FIT_RESULT_X_a"    // fit results X-direction a
#define NDPluginCalibFitX_bString             "FIT_RESULT_X_b"    //                         b 
#define NDPluginCalibFitY_aString             "FIT_RESULT_Y_a"    //             Y-direction a 
#define NDPluginCalibFitY_bString             "FIT_RESULT_Y_b"    //                         b
#define NDPluginCalibMiddlePointXString       "MIDDLE_PT_X"       // middle point after calibration x- coordinate
#define NDPluginCalibMiddlePointYString       "MIDDLE_PT_Y"       // middle point after calibration y- coordinate

static const char* pluginName = "NDPluginTransform";

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
    int NDPluginCalibFitX_a;
    int NDPluginCalibFitX_b;
    int NDPluginCalibFitY_a;
    int NDPluginCalibFitY_b;
    int NDPluginCalibMiddlePointX;
    int NDPluginCalibMiddlePointY;
    int NDPluginCalibShowImage;    
    
 
private:
    double m_LowThreshold;
    double m_ThresholdRatio;
    double m_FitX_a;
    double m_FitX_b;
    double m_FitY_a;
    double m_FitY_b;
    double m_MiddlePointX;
    double m_MiddlePointY;
    int    m_ShowImage;    
    

    size_t userDims_[ND_ARRAY_MAX_DIMS];
    void   transformImage(NDArray *inArray, NDArray *outArray, NDArrayInfo_t *arrayInfo);
    int    fitLinear(std::vector<float>&, std::vector<float>&, std::vector<float>&);
    void   sortPoints(std::vector<cv::Point2f>&);
    void   findTransformationPoints(std::vector<cv::Point2f>&, std::vector<cv::Point2f>&);
    void   calibrate(const cv::Mat&, std::vector<float>&);
    //void writeImageToFile();

};
    
#endif
