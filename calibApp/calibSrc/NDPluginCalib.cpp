/*
 * NDPluginCalib.cpp
 *
 * Image processing plugin
 * Author: Tomasz Brys
 * email:  tomasz.brys@esss.se 
 * the driver is base od ADPluginEdge and ADPluginConvert
 * it includes also procedures found on webpage which a free available
 * Created October 10, 2018
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

//#include <opencv2/opencv.hpp>
//#include <vector>

static const char *driverName="NDPluginCalib";


using namespace cv;
using namespace std;


//==========================================================================================
int NDPluginCalib::fitLinear(vector<float> &dX, vector<float> &dY, vector<float> &result){
// fit line to the data points
// data suppose to be in two vectors, x(mm) and y(pixels)
// the result will be stored in vector result
// algorithm least sqare method with checking verticality/horizontality

  double sumX = 0, sumY = 0, sumXX = 0, sumYY= 0, sumXY = 0;
  int nr = dX.size();
  int nr2 = dY.size();
  if(nr != nr2)
    return -1;

  for (int i = 0; i < nr; i++){
      sumX += dX.at(i);
      sumY += dY.at(i);
      sumXY += dX.at(i) * dY.at(i);
      sumXX += dX.at(i) * dX.at(i);
      sumYY += dY.at(i) * dY.at(i);
   }

   sumX  /= nr;
   sumY  /= nr;
   sumXX /= nr;
   sumYY /= nr;
   sumXY /= nr;

   double A = (sumXY - sumX*sumY);
   double B = 0;
   double Bx = sumXX - sumX * sumX;
   double By = sumYY - sumY * sumY;


   if( fabs( Bx ) < fabs( By ) ) //!< Test verticality/horizontality
      { // Line is more Vertical.
        B = By;
        std::swap(A,B);
     }
     else
     {   // Line is more Horizontal.
        // Classical solution, when we expect more horizontal-like line
        B = Bx;
     }

   double C = - ( A * sumX + B * sumY ); 

   result[0] = (-A/B);
   result[1] = (-C/B);

  //cout << "A=" << A << " B=" << B << " C=" << C << endl;
  cout << "fit function y = a*x + b" << endl;
  cout << "a=" << (-A/B) << " b=" << (-C/B) << endl;

  return 0;
}

//==========================================================================================

void NDPluginCalib::sortPoints(std::vector<cv::Point2f>& points){

  Point2f pt1, pt2, pt3, pt4;
  float maxSum = 0;
  float minSum = 5000;
  float maxDiff = 0;
  float minDiff = 5000;
  float temp = 0;

  for (auto pts:points){
       temp = (pts.x + pts.y);
       if (temp < minSum){
           pt1 = pts;
           minSum = temp;
       }

       temp = (pts.x + pts.y);
       if (temp > maxSum){
           pt3 = pts;
           maxSum = temp;
       }

       temp = (pts.x - pts.y);
       if (temp > maxDiff){
           pt4 = pts;
           maxDiff = temp;
       }

       temp = (pts.x - pts.y);
       if (temp < minDiff){
           pt2 = pts;
           minDiff = temp;
       }
  } // end for

 points[0] = pt1;
 points[1] = pt2;
 points[2] = pt3;
 points[3] = pt4;

}

//==========================================================================================

void NDPluginCalib::findTransformationPoints(std::vector<Point2f>& pt, std::vector<Point2f>& ptc){
  // check the width and the height of the bigest contour, to convert quadrangle to square.
  // we need a difference between h and w.
  double h = max(norm(pt[0] - pt[1]), norm(pt[2] - pt[3]));
  double w = max(norm(pt[1] - pt[2]), norm(pt[3] - pt[0]));
 

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

  ptc[0].x = min(pt[0].x, pt[1].x) - resx; ptc[0].y = min(pt[0].y, pt[3].y) - resy;
  ptc[1].x = min(pt[0].x, pt[1].x) - resx; ptc[1].y = max(pt[1].y, pt[2].y) + resy;
  ptc[2].x = max(pt[2].x, pt[3].x) + resx; ptc[2].y = max(pt[1].y, pt[2].y) + resy;
  ptc[3].x = max(pt[2].x, pt[3].x) + resx; ptc[3].y = min(pt[2].y, pt[3].y) - resy;

}

//==========================================================================================
void NDPluginCalib::calibrate(const cv::Mat& slice, std::vector<float>& result){
  //We want to use zerocrossing algorithm to find where is the transition between colors and thus find the distance between contours.
  // first we calculate average and then move xSlice and ySlice to have pos and neg values. Afterthat zerocrossing algorithm.
  int average = 0;
  for (int i = 0; i < slice.cols; i++){
       average += (int)slice.at<unsigned char>(i); 
     }
  average /= slice.cols;

  vector<float> s;

  for(int i = 0; i < slice.cols-1; i++){
     s.push_back( (float)slice.at<unsigned char>(i) - average);
     }
  
  vector<float> zeroData;
  vector<float> xData;
  vector<float> yData;


  for(unsigned i = 0; i < s.size()-1; i++){
     if ( s[i] * s[i+1] < 0 ){
        float x = i - s[i] / (s[i+1] - s[1]);
        zeroData.push_back(x);
     }
     else if(s[i] * s[i+1] == 0){
        if( s[i] != 0){
          zeroData.push_back(i);
          }
     }
     else{
     }

  }

  // prepare two vectors for fit algorithm
  vector<float> dX;
  vector<float> dY;

  // The real distance between contours varies from 5mm to 2.5mm and less but we neglect it. We do not know where we shoud start. 
  // What we know for sure is that the distance at the beginning between slices are 
  // 5mm and become smaler to 2.5mm. We search for a place where the distance change from ~90 to ~45 (in pixels).
  // l1 is the place where the distance change, so we know that from this point distance become ~45
  // The value ~90 was choosen by calculate diference between first two countors. We have to add some margin to be sure. 

  unsigned l1 = 0, l2 = 0;
  float bDist =  zeroData[1] - zeroData[0];
  for (unsigned i = 0; i < zeroData.size()-1; i++){
    //cout << i << " zeroY(i)= "  << zeroY.at(i) << " " << zeroY.at(i+1) << " diff= " << diff << endl; 
    if( (zeroData[i+1] - zeroData[i] > bDist-4) and (zeroData[i] - zeroData[i+1] < bDist+4) ){
      //dYfit.push_back(zeroData[i]);
    }
    else{
      l1 = i;
      break;
    }
  }
  // now we have to the same but from back

  for (int i = zeroData.size()-1; i > 0; i--){
    //cout << i << " zeroY(i)= "  << zeroY.at(i) << " " << zeroY.at(i-1) << " diff= " << diff << endl; 
    if( (zeroData[i] - zeroData[i-1] > bDist-4) and (zeroData[i] - zeroData[i-1] < bDist+4) ){
      //dYfit.push_back(zeroY.at(i));
    }
    else{
      l2 = i;
      break;
    }

  }

  for (unsigned i = 0; i < zeroData.size(); i++){
      if(i <= l1){
         dX.push_back(i*5. - 10.*l1);
         dY.push_back(zeroData[i]);
      }
      else if(i > l1 and i <= l1+4){
         dX.push_back(dX[i-1] + 2.5 );
         dY.push_back(zeroData[i]);
      }
      else if(i >= l2-4 and i <= l2){
         dX.push_back(20 - (l2-i) * 2.5);
         dY.push_back(zeroData[i]);
      }
      else if(i > l2 ){
         dX.push_back( 20 + (i-l2) * 5);
         dY.push_back(zeroData[i]);
      }

  }

  fitLinear(dX, dY, result);

}

//==========================================================================================

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
 
  NDArray *transformedArray;  // transformArray consists transformed original image in order to perform calibration
  NDArrayInfo_t arrayInfo;
  static const char* functionName = "processCallbacks";

  // Call the base class method 
  NDPluginDriver::beginProcessCallbacks(pArray);


  // Create a pointer to a structure of type NDArrayInfo_t and use it to get information about the input array.
  pArray->getInfo(&arrayInfo);

  this->userDims_[0] = arrayInfo.xDim;
  this->userDims_[1] = arrayInfo.yDim;
  //this->userDims_[2] = arrayInfo.colorDim;
  this->userDims_[2] =1; 

  /* Copy the information from the current array */
  transformedArray = this->pNDArrayPool->copy(pArray, NULL, 1);

  /* Release the lock; this is computationally intensive and does not access any shared data */
  this->unlock();
  if ( pArray->ndims == 2 ){
    this->transformImage(pArray, transformedArray, &arrayInfo);
  }
  else {
    asynPrint( this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s, this method is meant to transform 2Dimages when the number of dimensions is <= 3\n",
          pluginName, functionName);
  }
  this->lock();

   // Set NDArraySizeX and NDArraySizeY appropriately
  setIntegerParam(NDArraySizeX, (int)transformedArray->dims[arrayInfo.xDim].size);
  setIntegerParam(NDArraySizeY, (int)transformedArray->dims[arrayInfo.yDim].size);
  setIntegerParam(NDArraySizeZ, 0);

  NDPluginDriver::endProcessCallbacks(transformedArray, false, true);
  callParamCallbacks();
  }

//===================================================================================================
void NDPluginCalib::transformImage(NDArray *inArray, NDArray *outArray, NDArrayInfo_t *arrayInfo) {

  static const char *functionName="transformImage";

  epicsUInt8 *inData  = (epicsUInt8 *)inArray->pData;
  epicsUInt8 *outData = (epicsUInt8 *)outArray->pData;
  int xSize, ySize, colorSize;

  xSize = (int)arrayInfo->xSize;
  ySize = (int)arrayInfo->ySize;
  colorSize = (int)arrayInfo->colorSize;;
  if (colorSize > 0)
    colorSize = (int)arrayInfo->colorSize;
  else
    colorSize = 1;

  // Assume output array is same dimensions as input.  Handle rotation cases below.
  outArray->dims[arrayInfo->xDim].size = inArray->dims[arrayInfo->xDim].size;
  outArray->dims[arrayInfo->yDim].size = inArray->dims[arrayInfo->yDim].size;
  if (inArray->ndims > 2) 
	  outArray->dims[arrayInfo->colorDim].size = inArray->dims[arrayInfo->colorDim].size;


  // create opencv object, remember first ySize then xSize!!!
  cv::Mat img( ySize, xSize, CV_8UC1);
  memcpy( img.data, inData, arrayInfo->nElements * sizeof(unsigned char));

  // write a Mat object to file as (img.png), very useful if you want to see how algorithm works
  // vector<int> compression_params;
  // compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  // compression_params.push_back(9);
  //
  // try {
  //      imwrite("/mac/pictures/img.png", img, compression_params);
  //  }
  // catch (runtime_error& ex) {
  //      fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
  //  }


  // ---------------------------------------------------------------------------------
  // first slightly blur the image, then find edges. We use the same object
  
  cv::Mat detected_edges;
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
  int lowThreshold = 50;
  int thresholdRatio = 3;
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
  // find all contour in the image
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;

  try {
    cv::findContours(detected_edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
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
  int MAX_COUNTOUR_AREA = xSize * ySize;
  int maxAreaFound = MAX_COUNTOUR_AREA * 0.3;

  vector<Point2f>  pageContour; 
  vector<Point2f>  approx; 
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


  vector<Point2f>  tPoints(4);
  sortPoints(pageContour);
  findTransformationPoints(pageContour, tPoints);
  // ---------------------------------------------------------------------------------
  // transform image
  // Mat M = getPerspectiveTransform(sPoints, tPoints);
  cout << "tPoints=\n" << tPoints << endl;
  Mat M = getPerspectiveTransform(pageContour, tPoints);
  Mat rotated;
  warpPerspective(img, rotated, M, Size(xSize, ySize));

  Point2f ptc0;
  ptc0.x = (tPoints[3].x - tPoints[0].x) / 2 + tPoints[0].x;
  ptc0.y = (tPoints[1].y - tPoints[0].y) / 2 + tPoints[0].y;
  
  cout << "ptc0.x= " << ptc0.x << " ptc0.y= " << ptc0.y << endl; 
  Mat xSlice = rotated.col(static_cast<int>(ptc0.x)).t();
  Mat ySlice = rotated.row(static_cast<int>(ptc0.y));

  vector<float> dResultX(2);
  vector<float> dResultY(2);
  calibrate( rotated.col(static_cast<int>(ptc0.x)).t(), dResultX);
  calibrate( rotated.row(static_cast<int>(ptc0.y)),     dResultY);


  memcpy( outData, (unsigned char*)rotated.data, arrayInfo->nElements * sizeof(unsigned char));

  } // end of NDPluginTransform::transformImag
  // ---------------------------------------------------------------------------------

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
  createParam( NDPluginCalibFitX_aString,           asynParamFloat64,  &NDPluginCalibFitX_a);     
  createParam( NDPluginCalibFitX_bString,           asynParamFloat64,  &NDPluginCalibFitX_b);     
  createParam( NDPluginCalibFitY_aString,           asynParamFloat64,  &NDPluginCalibFitY_a);     
  createParam( NDPluginCalibFitY_bString,           asynParamFloat64,  &NDPluginCalibFitY_b);     
  createParam( NDPluginCalibMiddlePointXString,     asynParamFloat64,  &NDPluginCalibMiddlePointX);
  createParam( NDPluginCalibMiddlePointYString,     asynParamFloat64,  &NDPluginCalibMiddlePointY);



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
