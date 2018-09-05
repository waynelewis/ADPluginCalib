/*
 * NDPluginCalib.cpp
 *
 * Image processing plugin
 * Author: Keith Brister
 *
 * Created November 10, 2014
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <epicsString.h>
#include <epicsMutex.h>
#include <iocsh.h>

#include "NDArray.h"
#include "NDPluginCalib.h"
#include <epicsExport.h>

#include <opencv2/opencv.hpp>
#include <vector>

static const char *driverName="NDPluginCalib";


/** Color Mode to CV Matrix

    NDInt8,     Signed 8-bit integer        
    NDUInt8,    Unsigned 8-bit integer        
    NDInt16,    Signed 16-bit integer        
    NDUInt16,   Unsigned 16-bit integer        
    NDInt32,    Signed 32-bit integer        
    NDUInt32,   Unsigned 32-bit integer        
    NDFloat32,  32-bit float                
    NDFloat64   64-bit float                

    NDColorModeMono,    Monochromatic image                                                        
    NDColorModeBayer,   Bayer pattern image, 1 value per pixel but with color filter on detector
    NDColorModeRGB1,    RGB image with pixel color interleave, data array is [3, NX, NY]        
    NDColorModeRGB2,    RGB image with row color interleave, data array is [NX, 3, NY]                
    NDColorModeRGB3,    RGB image with plane color interleave, data array is [NX, NY, 3]        
    NDColorModeYUV444,  YUV image, 3 bytes encodes 1 RGB pixel                                        
    NDColorModeYUV422,  YUV image, 4 bytes encodes 2 RGB pixel                                        
    NDColorModeYUV411   YUV image, 6 bytes encodes 4 RGB pixels                                        


    NDArray         OpenCV
    =========       ==========
    NDInt8          CV_8S
    NDUInt8         CV_8U
    NDInt16         CV_16S
    NDUInt16        CV_16U
    NDInt32         CV_32S
    NDUInt32        CV_32U
    NDFloat32       CV_32F
    NDFloat64       CV_64F

    ND_BayerPatern  OpenCV
    ==============  ======
    NDBayer_RGGB    RG
    NDBayer_GBRG    GB
    NDBayer_GRGB    GR
    NDBayer_BGGR    BG
*/
using namespace cv;
using namespace std;

/** Callback function that is called by the NDArray driver with new NDArray data.
  * Does image processing.
  * \param[in] pArray  The NDArray from the callback.
  */
void NDPluginCalib::processCallbacks(NDArray *pArray)
{
  /* This function does array processing.
   * It is called with the mutex already locked.  It unlocks it during long calculations when private
   * structures don't need to be protected.
   */

  static const char* functionName = "processCallbacks";
  // Check if we have BW image
  if (pArray->ndims != 2) {
    asynPrint(
        this->pasynUserSelf, 
        ASYN_TRACE_ERROR, 
        "%s::%s Please convert edge detection plugin input image format to mono\n", 
        driverName, 
        functionName);
    return;
  }


  NDArray 	*pScratch=NULL;    //- to .h?
  NDArrayInfo 	 arrayInfo;

  // Call the base class method 
  NDPluginDriver::beginProcessCallbacks(pArray);

  // Get info from pArray to set width and height of the picture
  unsigned int height, width;        //- to .h?
  pArray->getInfo(&arrayInfo);
  width = pArray->dims[arrayInfo.xDim].size;
  height= pArray->dims[arrayInfo.yDim].size;

  // get parameters for Canny  algorithm
  double lowThreshold, thresholdRatio;  //- to .h? 
  getDoubleParam( NDPluginCalibLowThreshold,   &lowThreshold);
  getDoubleParam( NDPluginCalibThresholdRatio, &thresholdRatio);



  // Do the computationally expensive code with the lock released 
  this->unlock();
      
  unsigned char *inData, *outData;
  NDDimension_t scratchDim[2];
  pScratch->initDimension(&scratchDim[0], width);
  pScratch->initDimension(&scratchDim[1], height);

  // make the array something we understand 
  this->pNDArrayPool->convert( pArray, &pScratch, NDUInt8, scratchDim);

  pScratch->getInfo(&arrayInfo);
  width = pScratch->dims[arrayInfo.xDim].size;
  height = pScratch->dims[arrayInfo.yDim].size;

  cv::Mat img = cv::Mat( height, width, CV_8UC1);
  cv::Mat detected_edges;

 
  // Initialize the output data array
  //
  inData  = (unsigned char *)pScratch->pData;
  outData = (unsigned char *)img.data;
  memcpy( outData, inData, arrayInfo.nElements * sizeof(unsigned char));

  //std::cout << "width: " << width << "\theight: " << height << std::endl;

  // ---------------------------------------------------------------------------------
  // first slightly blur the image
  cv::Mat img_blur;
  try {
    cv::blur( img, detected_edges, cv::Size(3,3));
  }
  catch( cv::Exception &e) {
    const char* err_msg = e.what();
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s cv::blur exception:  %s\n", 
        driverName, functionName, err_msg);
    this->lock();
    return;
  }

  // ---------------------------------------------------------------------------------
  // Here is the edge detection routine.
  try {
    cv::Canny( detected_edges, detected_edges, lowThreshold, thresholdRatio * lowThreshold, 3);
  }
  catch( cv::Exception &e) {
    const char* err_msg = e.what();

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s cv::Canny exception:  %s\n", 
        driverName, functionName, err_msg);
    this->lock();
    return;
  }

  // ---------------------------------------------------------------------------------
  // find contour and get the bigest one
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;

  try {
    cv::findContours(img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  }
  catch( cv::Exception &e) {
    const char* err_msg = e.what();

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s cv::FindContours exception:  %s\n", 
        driverName, functionName, err_msg);
    this->lock();
    return;
  }

  // ---------------------------------------------------------------------------------
  // get the biggest contour and find its corners 
  int MAX_COUNTOUR_AREA = width * height;
  int maxAreaFound = MAX_COUNTOUR_AREA * 0.3;

  vector<Point>  pageContour; 
  vector<Point>  approx; 
  for (auto cnt:contours){
     double perimeter = cv::arcLength(cnt, true);		    
     cv::approxPolyDP(cnt, approx, 0.03 * perimeter, true);	
     if (approx.size() == 4 and 
             cv::isContourConvex(approx) and 
             maxAreaFound < cv::contourArea(approx) and
             cv::contourArea(approx) < MAX_COUNTOUR_AREA){		
             maxAreaFound = cv::contourArea(approx);		        
             pageContour = approx;
     }
  }
/*
  if(pageContour.size() == 0){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s cv::FindContours exception: ,no contours were found\n", 
        driverName, functionName);
    this->lock();
    return ;
  }
  else{
    cout << "Hello! I am a Calib Plugin!" << endl;
  }
*/

  Point2f pt1, pt2, pt3, pt4;
  Point2f ptc1, ptc2, ptc3, ptc4;
  float maxS = 0;
  float minS = 5000;
  float maxD = 0;
  float minD = 5000;
  float temp = 0;

 // ---------------------------------------------------------------------------------
 // sort corners 1-topleft, 2-bottomleft, 3-bottomright, 4-topright
 //

  for (auto pts:pageContour){
       temp = (pts.x + pts.y);
       if (temp < minS){
           pt1 = pts;
           minS = temp;
       }

       temp = (pts.x + pts.y);
       if (temp > maxS){
           pt3 = pts;
           maxS = temp;
       }

       temp = (pts.x - pts.y);
       if (temp > maxD){
           pt4 = pts;
           maxD = temp;
       }

       temp = (pts.x - pts.y);
       if (temp < minD){
           pt2 = pts;
           minD = temp;
       }
  }

  //-----------------------------------------------------------------------------
  // check the width and the height of the bigest contour, to convert rectangular to square.
  // we need a difference between h and w.
  double h = max(norm(pt1 - pt2), norm(pt3 - pt4));
  double w = max(norm(pt2 - pt3), norm(pt4 - pt1));
 

  double diff = (h - w);
  double resx = 0;
  double resy = 0;
  if (diff > 0){
      resx = diff / 2;
      resy = 0;
  }
  else{
      resx = 0;
      resy = -diff / 2;
  }

  ptc1.x = min(pt1.x, pt2.x) - resx; ptc1.y = min(pt1.y, pt4.y) - resy;
  ptc2.x = min(pt1.x, pt2.x) - resx; ptc2.y = max(pt2.y, pt3.y) + resy;
  ptc3.x = max(pt3.x, pt4.x) + resx; ptc3.y = max(pt2.y, pt3.y) + resy;
  ptc4.x = max(pt3.x, pt4.x) + resx; ptc4.y = min(pt3.y, pt4.y) - resy;

  Point2f ptc0;
  ptc0.x = (ptc4.x - ptc1.x) / 2 + ptc1.x;
  ptc0.y = (ptc2.y - ptc1.y) / 2 + ptc1.y;


  vector<Point2f>  sPoints;
  vector<Point2f>  tPoints;
  sPoints.push_back(pt1);
  sPoints.push_back(pt2);
  sPoints.push_back(pt3);
  sPoints.push_back(pt4);
  tPoints.push_back(ptc1);
  tPoints.push_back(ptc2);
  tPoints.push_back(ptc3);
  tPoints.push_back(ptc4);

  // ---------------------------------------------------------------------------------
  // transform image
  Mat M = getPerspectiveTransform(sPoints, tPoints);
  Mat rotated;
  warpPerspective(img, rotated, M, Size(width, height));
  

  // ---------------------------------------------------------------------------------
  // Take the lock again since we are accessing the parameter library and 
  // these calculations are not time consuming
  this->lock();


  int arrayCallbacks = 0;
  getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
  if (arrayCallbacks == 1) {
    //inData  = (unsigned char *)detected_edges.data;
    inData  = (unsigned char *)rotated.data;
    outData = (unsigned char *)pScratch->pData;
    memcpy(outData, inData, arrayInfo.nElements * sizeof(unsigned char));
    this->getAttributes(pScratch->pAttributeList);
    doCallbacksGenericPointer(pScratch, NDArrayData, 0);
  }

  if (NULL != pScratch)
    pScratch->release();

  callParamCallbacks();
}



/** Constructor for NDPluginCalib; most parameters are simply passed to NDPluginDriver::NDPluginDriver.
 * After calling the base class constructor this method sets reasonable default values for all of the
 * parameters.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] queueSize The number of NDArrays that the input queue for this plugin can hold when
 *            NDPluginDriverBlockingCallbacks=0.  Larger queues can decrease the number of dropped arrays,
 *            at the expense of more NDArray buffers being allocated from the underlying driver's NDArrayPool.
 * \param[in] blockingCallbacks Initial setting for the NDPluginDriverBlockingCallbacks flag.
 *            0=callbacks are queued and executed by the callback thread; 1 callbacks execute in the thread
 *            of the driver doing the callbacks.
 * \param[in] NDArrayPort Name of asyn port driver for initial source of NDArray callbacks.
 * \param[in] NDArrayAddr asyn port driver address for initial source of NDArray callbacks.
 * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
 * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
 * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
 * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
 */
NDPluginCalib::NDPluginCalib(const char *portName, int queueSize, int blockingCallbacks,
    const char *NDArrayPort, int NDArrayAddr,
    int maxBuffers, size_t maxMemory,
    int priority, int stackSize)
  /* Invoke the base class constructor */
  : NDPluginDriver(portName, queueSize, blockingCallbacks,
      NDArrayPort, NDArrayAddr, 1, maxBuffers, maxMemory,
      asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
      asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
      0, 1, priority, stackSize, 1)
{
  char versionString[20];
  //static const char *functionName = "NDPluginCalib";

  createParam( NDPluginCalibLowThresholdString,     asynParamFloat64,  &NDPluginCalibLowThreshold);
  createParam( NDPluginCalibThresholdRatioString,   asynParamFloat64,  &NDPluginCalibThresholdRatio);
  createParam( NDPluginCalibVerticalFoundString,    asynParamInt32,    &NDPluginCalibVerticalFound);
  createParam( NDPluginCalibTopEdgeFoundString,     asynParamInt32,    &NDPluginCalibTopEdgeFound);
  createParam( NDPluginCalibTopPixelString,         asynParamInt32,    &NDPluginCalibTopPixel);
  createParam( NDPluginCalibBottomEdgeFoundString,  asynParamInt32,    &NDPluginCalibBottomEdgeFound);
  createParam( NDPluginCalibBottomPixelString,      asynParamInt32,    &NDPluginCalibBottomPixel);
  createParam( NDPluginCalibVerticalCenterString,   asynParamFloat64,  &NDPluginCalibVerticalCenter);
  createParam( NDPluginCalibVerticalSizeString,     asynParamInt32,    &NDPluginCalibVerticalSize);
  createParam( NDPluginCalibHorizontalFoundString,  asynParamInt32,    &NDPluginCalibHorizontalFound);
  createParam( NDPluginCalibLeftEdgeFoundString,    asynParamInt32,    &NDPluginCalibLeftEdgeFound);
  createParam( NDPluginCalibLeftPixelString,        asynParamInt32,    &NDPluginCalibLeftPixel);
  createParam( NDPluginCalibRightEdgeFoundString,   asynParamInt32,    &NDPluginCalibRightEdgeFound);
  createParam( NDPluginCalibRightPixelString,       asynParamInt32,    &NDPluginCalibRightPixel);
  createParam( NDPluginCalibHorizontalCenterString, asynParamFloat64,  &NDPluginCalibHorizontalCenter);
  createParam( NDPluginCalibHorizontalSizeString,   asynParamInt32,    &NDPluginCalibHorizontalSize);


  /* Set the plugin type string */
  setStringParam(NDPluginDriverPluginType, "NDPluginCalib");

  epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d",
      CALIB_VERSION, CALIB_REVISION, CALIB_MODIFICATION);
  setStringParam(NDDriverVersion, versionString);

  /* Try to connect to the array port */
  connectToArrayPort();
}

/** Configuration command */
extern "C" int NDCalibConfigure(const char *portName, int queueSize, int blockingCallbacks,
    const char *NDArrayPort, int NDArrayAddr,
    int maxBuffers, size_t maxMemory,
    int priority, int stackSize)
{
  NDPluginCalib *pPlugin = new NDPluginCalib(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
      maxBuffers, maxMemory, priority, stackSize);
  return pPlugin->start();
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg initArg1 = { "frame queue size",iocshArgInt};
static const iocshArg initArg2 = { "blocking callbacks",iocshArgInt};
static const iocshArg initArg3 = { "NDArrayPort",iocshArgString};
static const iocshArg initArg4 = { "NDArrayAddr",iocshArgInt};
static const iocshArg initArg5 = { "maxBuffers",iocshArgInt};
static const iocshArg initArg6 = { "maxMemory",iocshArgInt};
static const iocshArg initArg7 = { "priority",iocshArgInt};
static const iocshArg initArg8 = { "stackSize",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
  &initArg1,
  &initArg2,
  &initArg3,
  &initArg4,
  &initArg5,
  &initArg6,
  &initArg7,
  &initArg8};
static const iocshFuncDef initFuncDef = {"NDCalibConfigure",9,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
  NDCalibConfigure(args[0].sval, args[1].ival, args[2].ival,
      args[3].sval, args[4].ival, args[5].ival,
      args[6].ival, args[7].ival, args[8].ival);
}

extern "C" void NDCalibRegister(void)
{
  iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
  epicsExportRegistrar(NDCalibRegister);
}
