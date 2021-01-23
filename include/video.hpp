/************************************************************************/
/* 本Demo 简单演示SDK 发现相机，连接相机，取图，断开相机的使用*/
/************************************************************************/
#include <arpa/inet.h>
#include "GenICam/System.h"
#include "GenICam/Camera.h"
#include "GenICam/StreamSource.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "Infra/PrintLog.h"
#include "Memory/SharedPtr.h"
#include "Media/ImageConvert.h"

// #include "opencv2/opencv.hpp"
// #include "opencv2/core.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/videoio.hpp"

#include <math.h>
#include "Media/VideoRender.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace Dahua::GenICam;
using namespace Dahua::Infra;

class FrameBuffer
{
  private:
    uint8_t *Buffer_;

    int Width_;

    int Height_;

    int PaddingX_;

    int PaddingY_;

    int DataSize_;

    int PixelFormat_;

    uint64_t TimeStamp_;

    uint64_t BlockId_;

  public:
    FrameBuffer(Dahua::GenICam::CFrame const &frame)
    {
        if (frame.getImageSize() > 0)
        {
            if (frame.getImagePixelFormat() == Dahua::GenICam::gvspPixelMono8)
            {
                Buffer_ = new (std::nothrow) uint8_t[frame.getImageSize()];
            }
            else
            {
                Buffer_ = new (std::nothrow) uint8_t[frame.getImageWidth() * frame.getImageHeight() * 3];
            }
            if (Buffer_)
            {
                Width_ = frame.getImageWidth();
                Height_ = frame.getImageHeight();
                PaddingX_ = frame.getImagePadddingX();
                PaddingY_ = frame.getImagePadddingY();
                DataSize_ = frame.getImageSize();
                PixelFormat_ = frame.getImagePixelFormat();
                BlockId_ = frame.getBlockId();
            }
        }
    }

    ~FrameBuffer()
    {
        if (Buffer_ != NULL)
        {
            delete[] Buffer_;
            Buffer_ = NULL;
        }
    }

    bool Valid()
    {
        if (NULL != Buffer_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int Width()
    {
        return Width_;
    }

    int Height()
    {
        return Height_;
    }

    int PaddingX()
    {
        return PaddingX_;
    }

    int PaddingY()
    {
        return PaddingY_;
    }

    int DataSize()
    {
        return DataSize_;
    }

    uint64_t PixelFormat()
    {
        return PixelFormat_;
    }

    uint64_t TimeStamp()
    {
        return TimeStamp_;
    }

    void setWidth(uint32_t iWidth)
    {
        Width_ = iWidth;
    }

    void setPaddingX(uint32_t iPaddingX)
    {
        PaddingX_ = iPaddingX;
    }

    uint64_t BlockId()
    {
        return BlockId_;
    }

    void setPaddingY(uint32_t iPaddingX)
    {
        PaddingY_ = iPaddingX;
    }

    void setHeight(uint32_t iHeight)
    {
        Height_ = iHeight;
    }

    void setDataSize(int dataSize)
    {
        DataSize_ = dataSize;
    }

    void setPixelFormat(uint32_t pixelFormat)
    {
        PixelFormat_ = pixelFormat;
    }

    void setTimeStamp(uint64_t timeStamp)
    {
        TimeStamp_ = timeStamp;
    }

    uint8_t *bufPtr()
    {
        return Buffer_;
    }
};

/* 4、设置相机采图模式（连续采图、触发采图） */
static int32_t setGrabMode(ICameraPtr &cameraSptr, bool bContious)
{
    int32_t bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptrAcquisitionControl->triggerSelector();
    bRet = enumNode.setValueBySymbol("FrameStart");
    if (false == bRet)
    {
        printf("set TriggerSelector fail.\n");
        return -1;
    }

    if (true == bContious)
    {
        enumNode = sptrAcquisitionControl->triggerMode();
        bRet = enumNode.setValueBySymbol("Off");
        if (false == bRet)
        {
            printf("set triggerMode fail.\n");
            return -1;
        }
    }
    else
    {
        enumNode = sptrAcquisitionControl->triggerMode();
        bRet = enumNode.setValueBySymbol("On");
        if (false == bRet)
        {
            printf("set triggerMode fail.\n");
            return -1;
        }

        /* 设置触发源为软触发（硬触发为Line1） */
        enumNode = sptrAcquisitionControl->triggerSource();
        bRet = enumNode.setValueBySymbol("Software");
        if (false == bRet)
        {
            printf("set triggerSource fail.\n");
            return -1;
        }
    }
    return 0;
}

/* 5、获取相机采图模式 */
static int32_t getGrabMode(ICameraPtr &cameraSptr, bool &bContious)
{
    int32_t bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptrAcquisitionControl->triggerSelector();
    bRet = enumNode.setValueBySymbol("FrameStart");
    if (false == bRet)
    {
        printf("set TriggerSelector fail.\n");
        return -1;
    }

    CString strValue;
    enumNode = sptrAcquisitionControl->triggerMode();
    bRet = enumNode.getValueSymbol(strValue);
    if (false == bRet)
    {
        printf("get triggerMode fail.\n");
        return -1;
    }

    if (strValue == "Off")
    {
        bContious = true;
    }
    else if (strValue == "On")
    {
        bContious = false;
    }
    else
    {
        printf("get triggerMode fail.\n");
        return -1;
    }
    return 0;
}

/* 6、软件触发 */
static int32_t triggerSoftware(ICameraPtr &cameraSptr)
{
    int32_t bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CCmdNode cmdNode = sptrAcquisitionControl->triggerSoftware();
    bRet = cmdNode.execute();
    if (false == bRet)
    {
        printf("triggerSoftware execute fail.\n");
        return -1;
    }
    return 0;
}

/* 9、设置传感器采样率（采集分辨率） */
static int32_t setResolution(ICameraPtr &cameraSptr, int nWidth, int nHeight)
{
    int32_t bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.setValue(nWidth);
    if (false == bRet)
    {
        printf("set width fail.\n");
        return -1;
    }

    intNode = sptrImageFormatControl->height();
    bRet = intNode.setValue(nHeight);
    if (false == bRet)
    {
        printf("set height fail.\n");
        return -1;
    }
    return 0;
}

/* 10、获取传感器采样率 */
static int32_t getResolution(ICameraPtr &cameraSptr, int64_t &nWidth, int64_t &nHeight)
{
    int32_t bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (false == bRet)
    {
        printf("get width fail.\n");
        return -1;
    }

    intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (false == bRet)
    {
        printf("get height fail.\n");
        return -1;
    }
    return 0;
}

/* 设置binning (Off X Y XY) */
static int32_t setBinning(ICameraPtr &cameraSptr)
{
    CEnumNodePtr ptrParam(new CEnumNode(cameraSptr, "Binning"));
    if (ptrParam)
    {
        if (false == ptrParam->isReadable())
        {
            printf("binning not support.\n");
            return -1;
        }

        if (false == ptrParam->setValueBySymbol("XY"))
        {
            printf("set Binning XY fail.\n");
            return -1;
        }

        if (false == ptrParam->setValueBySymbol("Off"))
        {
            printf("set Binning Off fail.\n");
            return -1;
        }
    }
    return 0;
}

/* 11、获取传感器最大分辩率 */
static int32_t getMaxResolution(ICameraPtr &cameraSptr, int64_t &nWidthMax, int64_t &nHeightMax)
{
    /* width */
    {
        CIntNodePtr ptrParam(new CIntNode(cameraSptr, "SensorWidth"));
        if (ptrParam)
        {
            if (false == ptrParam->getValue(nWidthMax))
            {
                printf("get WidthMax fail.\n");
                return -1;
            }
        }
    }

    /* height */
    {
        CIntNodePtr ptrParam(new CIntNode(cameraSptr, "SensorHeight"));
        if (ptrParam)
        {
            if (false == ptrParam->getValue(nWidthMax))
            {
                printf("get WidthMax fail.\n");
                return -1;
            }
        }
    }
    return 0;
}

/* 12、设置图像ROI */
static int32_t setROI(ICameraPtr &cameraSptr, int64_t nX, int64_t nY, int64_t nWidth, int64_t nHeight)
{
    bool bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    /* width */
    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.setValue(nWidth);
    if (!bRet)
    {
        printf("set width fail.\n");
        return -1;
    }

    /* height */
    intNode = sptrImageFormatControl->height();
    bRet = intNode.setValue(nHeight);
    if (!bRet)
    {
        printf("set height fail.\n");
        return -1;
    }

    /* OffsetX */
    intNode = sptrImageFormatControl->offsetX();
    bRet = intNode.setValue(nX);
    if (!bRet)
    {
        printf("set offsetX fail.\n");
        return -1;
    }

    /* OffsetY */
    intNode = sptrImageFormatControl->offsetY();
    bRet = intNode.setValue(nY);
    if (!bRet)
    {
        printf("set offsetY fail.\n");
        return -1;
    }

    return 0;
}

/* 13、获取图像ROI */
static int32_t getROI(ICameraPtr &cameraSptr, int64_t &nX, int64_t &nY, int64_t &nWidth, int64_t &nHeight)
{
    bool bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    /* width */
    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (!bRet)
    {
        printf("get width fail.\n");
    }

    /* height */
    intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (!bRet)
    {
        printf("get height fail.\n");
    }

    /* OffsetX */
    intNode = sptrImageFormatControl->offsetX();
    bRet = intNode.getValue(nX);
    if (!bRet)
    {
        printf("get offsetX fail.\n");
    }

    /* OffsetY */
    intNode = sptrImageFormatControl->offsetY();
    bRet = intNode.getValue(nY);
    if (!bRet)
    {
        printf("get offsetY fail.\n");
    }
    return 0;
}

/* 14、获取采图图像宽度 */
static int32_t getWidth(ICameraPtr &cameraSptr, int64_t &nWidth)
{
    bool bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    CIntNode intNode = sptrImageFormatControl->width();
    bRet = intNode.getValue(nWidth);
    if (!bRet)
    {
        printf("get width fail.\n");
    }
    return 0;
}

/* 15、获取采图图像高度 */
static int32_t getHeight(ICameraPtr &cameraSptr, int64_t &nHeight)
{
    bool bRet;
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);
    if (NULL == sptrImageFormatControl)
    {
        return -1;
    }

    CIntNode intNode = sptrImageFormatControl->height();
    bRet = intNode.getValue(nHeight);
    if (!bRet)
    {
        printf("get height fail.\n");
        return -1;
    }
    return 0;
}

/* 17、设置曝光值(曝光、自动曝光/手动曝光) */
static int32_t setExposureTime(ICameraPtr &cameraSptr, double dExposureTime, bool bAutoExposure = false)
{
    bool bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    if (bAutoExposure)
    {
        CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
        bRet = enumNode.setValueBySymbol("Continuous");
        if (false == bRet)
        {
            printf("set exposureAuto fail.\n");
            return -1;
        }
    }
    else
    {
        CEnumNode enumNode = sptrAcquisitionControl->exposureAuto();
        bRet = enumNode.setValueBySymbol("Off");
        if (false == bRet)
        {
            printf("set exposureAuto fail.\n");
            return -1;
        }

        CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
        bRet = doubleNode.setValue(dExposureTime);
        if (false == bRet)
        {
            printf("set exposureTime fail.\n");
            return -1;
        }
    }
    return 0;
}

/* 18、获取曝光时间 */
static int32_t getExposureTime(ICameraPtr &cameraSptr, double &dExposureTime)
{
    bool bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
    bRet = doubleNode.getValue(dExposureTime);
    if (false == bRet)
    {
        printf("get exposureTime fail.\n");
        return -1;
    }
    return 0;
}

/* 19、获取曝光范围 */
static int32_t getExposureTimeMinMaxValue(ICameraPtr &cameraSptr, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAcquisitionControl->exposureTime();
    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get exposureTime minValue fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get exposureTime maxValue fail.\n");
        return -1;
    }
    return 0;
}

/* 20、设置增益值 */
static int32_t setGainRaw(ICameraPtr &cameraSptr, double dGainRaw)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
    bRet = doubleNode.setValue(dGainRaw);
    if (false == bRet)
    {
        printf("set gainRaw fail.\n");
        return -1;
    }
    return 0;
}

/* 21、获取增益值 */
static int32_t getGainRaw(ICameraPtr &cameraSptr, double &dGainRaw)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
    bRet = doubleNode.getValue(dGainRaw);
    if (false == bRet)
    {
        printf("get gainRaw fail.\n");
        return -1;
    }
    return 0;
}

/* 22、获取增益值范围 */
static int32_t getGainRawMinMaxValue(ICameraPtr &cameraSptr, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gainRaw();
    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get gainRaw minValue fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get gainRaw maxValue fail.\n");
        return -1;
    }
    return 0;
}

/* 23、设置伽马值 */
static int32_t setGamma(ICameraPtr &cameraSptr, double dGamma)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gamma();
    bRet = doubleNode.setValue(dGamma);
    if (false == bRet)
    {
        printf("set gamma fail.\n");
        return -1;
    }
    return 0;
}

/* 24、获取伽马值 */
static int32_t getGamma(ICameraPtr &cameraSptr, double &dGamma)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gamma();
    bRet = doubleNode.getValue(dGamma);
    if (false == bRet)
    {
        printf("get gamma fail.\n");
        return -1;
    }
    return 0;
}

/* 25、获取伽马值范围 */
static int32_t getGammaMinMaxValue(ICameraPtr &cameraSptr, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->gamma();
    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get gamma minValue fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get gamma maxValue fail.\n");
        return -1;
    }
    return 0;
}

/* 26、设置白平衡值（有三个白平衡值） */
static int32_t setBalanceRatio(ICameraPtr &cameraSptr, double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    /* 关闭自动白平衡 */
    CEnumNode enumNode = sptrAnalogControl->balanceWhiteAuto();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = enumNode.setValueBySymbol("Off");
    if (false == bRet)
    {
        printf("set balanceWhiteAuto Off fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Red");
    if (false == bRet)
    {
        printf("set red balanceRatioSelector fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dRedBalanceRatio);
    if (false == bRet)
    {
        printf("set red balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Green");
    if (false == bRet)
    {
        printf("set green balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dGreenBalanceRatio);
    if (false == bRet)
    {
        printf("set green balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Blue");
    if (false == bRet)
    {
        printf("set blue balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.setValue(dBlueBalanceRatio);
    if (false == bRet)
    {
        printf("set blue balanceRatio fail.\n");
        return -1;
    }
    return 0;
}

/* 27、获取白平衡值（有三个白平衡值） */
static int32_t getBalanceRatio(ICameraPtr &cameraSptr, double &dRedBalanceRatio, double &dGreenBalanceRatio, double &dBlueBalanceRatio)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptrAnalogControl->balanceRatioSelector();
    if (false == enumNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = enumNode.setValueBySymbol("Red");
    if (false == bRet)
    {
        printf("set red balanceRatioSelector fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dRedBalanceRatio);
    if (false == bRet)
    {
        printf("get red balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Green");
    if (false == bRet)
    {
        printf("set green balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dGreenBalanceRatio);
    if (false == bRet)
    {
        printf("get green balanceRatio fail.\n");
        return -1;
    }

    enumNode = sptrAnalogControl->balanceRatioSelector();
    bRet = enumNode.setValueBySymbol("Blue");
    if (false == bRet)
    {
        printf("set blue balanceRatioSelector fail.\n");
        return -1;
    }

    doubleNode = sptrAnalogControl->balanceRatio();
    bRet = doubleNode.getValue(dBlueBalanceRatio);
    if (false == bRet)
    {
        printf("get blue balanceRatio fail.\n");
        return -1;
    }
    return 0;
}

/* 28、获取白平衡值范围 */
static int32_t getBalanceRatioMinMaxValue(ICameraPtr &cameraSptr, double &dMinValue, double &dMaxValue)
{
    bool bRet;
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(cameraSptr);
    if (NULL == sptrAnalogControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptrAnalogControl->balanceRatio();
    if (false == doubleNode.isReadable())
    {
        printf("balanceRatio not support.\n");
        return -1;
    }

    bRet = doubleNode.getMinVal(dMinValue);
    if (false == bRet)
    {
        printf("get balanceRatio min value fail.\n");
        return -1;
    }

    bRet = doubleNode.getMaxVal(dMaxValue);
    if (false == bRet)
    {
        printf("get balanceRatio max value fail.\n");
        return -1;
    }

    return 0;
}

/* 29、设置采图速度（秒帧数） */
static int32_t setAcquisitionFrameRate(ICameraPtr &cameraSptr, double dFrameRate)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CBoolNode booleanNode = sptAcquisitionControl->acquisitionFrameRateEnable();
    bRet = booleanNode.setValue(true);
    if (false == bRet)
    {
        printf("set acquisitionFrameRateEnable fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.setValue(dFrameRate);
    if (false == bRet)
    {
        printf("set acquisitionFrameRate fail.\n");
        return -1;
    }
    return 0;
}

/* 30、获取采图速度（秒帧数） */
static int32_t getAcquisitionFrameRate(ICameraPtr &cameraSptr, double &dFrameRate)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    bRet = doubleNode.getValue(dFrameRate);
    if (false == bRet)
    {
        printf("get acquisitionFrameRate fail.\n");
        return -1;
    }
    return 0;
}

/* 31、保存参数 */
static int32_t userSetSave(ICameraPtr &cameraSptr)
{
    bool bRet;
    IUserSetControlPtr sptUserSetControl = CSystem::getInstance().createUserSetControl(cameraSptr);
    if (NULL == sptUserSetControl)
    {
        return -1;
    }

    bRet = sptUserSetControl->saveUserSet(IUserSetControl::userSet1);
    if (false == bRet)
    {
        printf("saveUserSet fail.\n");
        return -1;
    }
    return 0;
}

/* 32、加载参数 */
static int32_t loadUserSet(ICameraPtr &cameraSptr)
{
    bool bRet;
    IUserSetControlPtr sptUserSetControl = CSystem::getInstance().createUserSetControl(cameraSptr);
    if (NULL == sptUserSetControl)
    {
        return -1;
    }

    bRet = sptUserSetControl->setCurrentUserSet(IUserSetControl::userSet1);
    if (false == bRet)
    {
        printf("saveUserSet fail.\n");
        return -1;
    }
    return 0;
}

/* 33、设置外触发延时时间 */
static int32_t setTriggerDelay(ICameraPtr &cameraSptr, double dDelayTime)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->triggerDelay();
    bRet = doubleNode.setValue(dDelayTime);
    if (false == bRet)
    {
        printf("set triggerDelay fail.\n");
        return -1;
    }
    return 0;
}

/* 34、获取外触发延时时间 */
static int32_t getTriggerDelay(ICameraPtr &cameraSptr, double &dDelayTime)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->triggerDelay();
    bRet = doubleNode.getValue(dDelayTime);
    if (false == bRet)
    {
        printf("set triggerDelay fail.\n");
        return -1;
    }
    return 0;
}

/* 35、设置外触发模式（上升沿触发、下降沿触发） */
static int32_t setLineTriggerMode(ICameraPtr &cameraSptr, bool bRisingEdge)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptAcquisitionControl->triggerSelector();
    if (false == enumNode.setValueBySymbol("FrameStart"))
    {
        printf("set triggerSelector fail.\n");
        return -1;
    }

    enumNode = sptAcquisitionControl->triggerMode();
    if (false == enumNode.setValueBySymbol("On"))
    {
        printf("set triggerMode fail.\n");
        return -1;
    }

    enumNode = sptAcquisitionControl->triggerSource();
    if (false == enumNode.setValueBySymbol("Line1"))
    {
        printf("set triggerSource fail.\n");
        return -1;
    }

    enumNode = sptAcquisitionControl->triggerActivation();
    if (true == bRisingEdge)
    {
        bRet = enumNode.setValueBySymbol("RisingEdge");
    }
    else
    {
        bRet = enumNode.setValueBySymbol("FallingEdge");
    }
    return 0;
}

/* 36、获取外触发模式（上升沿触发、下降沿触发） */
static int32_t getLineTriggerMode(ICameraPtr &cameraSptr, bool &bRisingEdge)
{
    bool bRet;
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(cameraSptr);
    if (NULL == sptAcquisitionControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptAcquisitionControl->triggerSelector();
    if (false == enumNode.setValueBySymbol("FrameStart"))
    {
        printf("set triggerSelector fail.\n");
        return -1;
    }

    CString strValue;
    enumNode = sptAcquisitionControl->triggerActivation();
    if (true == bRisingEdge)
    {
        bRet = enumNode.getValueSymbol(strValue);
    }
    else
    {
        bRet = enumNode.getValueSymbol(strValue);
    }

    if (false == bRet)
    {
        printf("get triggerActivation fail.\n");
        return -1;
    }

    if (strValue == "RisingEdge")
    {
        bRisingEdge = true;
    }
    else if (strValue == "FallingEdge")
    {
        bRisingEdge = false;
    }
    else
    {
        printf("get triggerActivation fail.\n");
        return -1;
    }
    return 0;
}

/* 37、设置外触发信号滤波时间 */
static int32_t setLineDebouncerTimeAbs(ICameraPtr &cameraSptr, double dLineDebouncerTimeAbs)
{
    IDigitalIOControlPtr sptDigitalIOControl = CSystem::getInstance().createDigitalIOControl(cameraSptr);
    if (NULL == sptDigitalIOControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptDigitalIOControl->lineSelector();
    if (false == enumNode.setValueBySymbol("Line1"))
    {
        printf("set lineSelector fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptDigitalIOControl->lineDebouncerTimeAbs();
    if (false == doubleNode.setValue(dLineDebouncerTimeAbs))
    {
        printf("set lineDebouncerTimeAbs fail.\n");
        return -1;
    }
    return 0;
}

/* 38、获取外触发信号滤波时间 */
static int32_t getLineDebouncerTimeAbs(ICameraPtr &cameraSptr, double &dLineDebouncerTimeAbs)
{
    IDigitalIOControlPtr sptDigitalIOControl = CSystem::getInstance().createDigitalIOControl(cameraSptr);
    if (NULL == sptDigitalIOControl)
    {
        return -1;
    }

    CEnumNode enumNode = sptDigitalIOControl->lineSelector();
    if (false == enumNode.setValueBySymbol("Line1"))
    {
        printf("set lineSelector fail.\n");
        return -1;
    }

    CDoubleNode doubleNode = sptDigitalIOControl->lineDebouncerTimeAbs();
    if (false == doubleNode.getValue(dLineDebouncerTimeAbs))
    {
        printf("get lineDebouncerTimeAbs fail.\n");
        return -1;
    }
    return 0;
}

/* 39、设置外触发脉冲宽度（不支持） */
/* 40、获取外触发脉冲宽度（不支持） */
/* 41、设置输出信号线（控制光源用）（面阵相机是Line0） */
/* 42、获取输出信号线（面阵相机是Line0） */

/* 43、设置外部光源曝光时间（设置输出值为TRUE的时间） */
static int32_t setOutputTime(ICameraPtr &cameraSptr, int nTimeMS)
{
    IDigitalIOControlPtr sptDigitalIOControl = CSystem::getInstance().createDigitalIOControl(cameraSptr);
    if (NULL == sptDigitalIOControl)
    {
        return -1;
    }

    CEnumNode paramLineSource(cameraSptr, "LineSource");
    if (false == paramLineSource.setValueBySymbol("UserOutput1"))
    {
        printf("set LineSource fail.");
        return -1;
    }

    /* 将输出信号拉高然后拉低 */
    CBoolNode booleanNode = sptDigitalIOControl->userOutputValue();
    if (false == booleanNode.setValue(true))
    {
        printf("set userOutputValue fail.\n");
        return -1;
    }

    CThread::sleep(nTimeMS);

    if (false == booleanNode.setValue(false))
    {
        printf("set userOutputValue fail.\n");
        return -1;
    }
}

/* 44、获取外部光源曝光时间（输出信号的时间由软件侧控制） */

/* 45、设置X轴翻转 */
static int32_t setReverseX(ICameraPtr &cameraSptr, bool flag)
{
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);

    CBoolNode boolNodeReverseX = sptrImageFormatControl->reverseX();
    if (!boolNodeReverseX.setValue(flag))
    {
        printf("set reverseX fail.\n");
        return -1;
    }

    return 0;
}

/* 46、设置Y轴翻转 */
static int32_t setReverseY(ICameraPtr &cameraSptr, bool flag)
{
    IImageFormatControlPtr sptrImageFormatControl = CSystem::getInstance().createImageFormatControl(cameraSptr);

    CBoolNode boolNodeReverseY = sptrImageFormatControl->reverseY();
    if (!boolNodeReverseY.setValue(flag))
    {
        printf("set reverseY fail.\n");
        return -1;
    }

    return 0;
}

/* 47、当相机与网卡处于不同网段时，自动设置相机IP与网卡处于同一网段 （与相机连接之前调用）*/
static int32_t autoSetCameraIP(ICameraPtr &cameraSptr)
{
    IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
    if (NULL == gigeCameraPtr)
    {
        return -1;
    }

    //获取Gige相机相关信息
    CString ip = gigeCameraPtr->getIpAddress();
    CString subnetMask = gigeCameraPtr->getSubnetMask();
    CString gateway = gigeCameraPtr->getGateway();
    CString macAddress = gigeCameraPtr->getMacAddress();
    printf("ip address is %s.\r\n", ip.c_str());
    printf("subnetMask is %s.\r\n", subnetMask.c_str());
    printf("gateway is %s.\r\n", gateway.c_str());
    printf("macAddress is %s.\r\n", macAddress.c_str());
    printf("\n");

    unsigned long devIpValue = ntohl(inet_addr(gigeCameraPtr->getIpAddress().c_str()));
    unsigned long devSubMaskValue = ntohl(inet_addr(gigeCameraPtr->getSubnetMask().c_str()));

    //获取对应接口的网卡信息
    IGigEInterfacePtr gigeInterfaceSPtr = IGigEInterface::getInstance(cameraSptr);
    if (NULL == gigeInterfaceSPtr)
    {
        return -1;
    }

    CString interfaceIp = gigeInterfaceSPtr->getIpAddress();
    CString interfaceSubnetMask = gigeInterfaceSPtr->getSubnetMask();
    CString interfaceGateway = gigeInterfaceSPtr->getGateway();
    CString interfaceMacAddress = gigeInterfaceSPtr->getMacAddress();
    printf("ip address of interface  is %s.\r\n", interfaceIp.c_str());
    printf("subnetMask of interface is %s.\r\n", interfaceSubnetMask.c_str());
    printf("gateway of interface is %s.\r\n", interfaceGateway.c_str());
    printf("macAddress of interface is %s.\r\n", interfaceMacAddress.c_str());
    printf("\n");

    unsigned long InterfaceIpValue = ntohl(inet_addr(gigeInterfaceSPtr->getIpAddress().c_str()));
    unsigned long InterfaceSubMaskValue = ntohl(inet_addr(gigeInterfaceSPtr->getSubnetMask().c_str()));

    if ((devIpValue & devSubMaskValue) != (InterfaceIpValue & InterfaceSubMaskValue))
    {
        //设备与网卡不在同一网段，强制设置设备与网卡在同一网段
        unsigned char newIPStr[20] = {0};

        while (1)
        {
            unsigned long newIpValue = rand() % 254 + 1; //1~254
            if (newIpValue != (InterfaceIpValue & 0xff))
            {
                newIpValue = (InterfaceIpValue & 0xffffff00) + newIpValue;
                struct in_addr stInAddr;
                stInAddr.s_addr = htonl(newIpValue);
                memcpy(newIPStr, inet_ntoa(stInAddr), strlen(inet_ntoa(stInAddr)));
                break;
            }
        }

        if (!gigeCameraPtr->forceIpAddress((const char *)newIPStr, gigeInterfaceSPtr->getSubnetMask().c_str(), gigeInterfaceSPtr->getGateway().c_str()))
        {
            printf("Set device ip failed.\n");
            return -1;
        }
    }

    return 0;
}

/* 48、设置相机IP （与相机连接之前调用）*/
static int32_t setCameraIp(ICameraPtr &cameraSptr, char *ipAddress, char *subnetMask, char *gateway)
{
    IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
    if (NULL == gigeCameraPtr)
    {
        return -1;
    }

    if (!gigeCameraPtr->forceIpAddress(ipAddress, subnetMask, gateway))
    {
        printf("Set device ip failed.\n");
        return -1;
    }

    return 0;
}

/* 49、设置相机静态IP （与相机连接之后调用）*/
static int32_t setCameraPersistentIP(ICameraPtr &cameraSptr)
{
    IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
    if (NULL == gigeCameraPtr)
    {
        printf("gigeCameraPtr is null.\n");
        return -1;
    }

    ITransportLayerControlPtr transportLayerControlPtr = CSystem::getInstance().createTransportLayerControl(cameraSptr);

    if (NULL == transportLayerControlPtr)
    {
        printf("transportLayerControlPtr is null.\n");
        return -1;
    }

    transportLayerControlPtr->gevCurrentIPConfigurationPersistentIP().setValue(true);
    transportLayerControlPtr->gevPersistentDefaultGateway().setValue(gigeCameraPtr->getGateway().c_str());
    transportLayerControlPtr->gevPersistentIPAddress().setValue(gigeCameraPtr->getIpAddress().c_str());
    transportLayerControlPtr->gevPersistentSubnetMask().setValue(gigeCameraPtr->getSubnetMask().c_str());

    return 0;
}

/* 50、修改曝光时间 （与相机连接之后调用）*/
static void modifyCamralExposureTime(CSystem &systemObj, ICameraPtr &cameraSptr)
{
    IAcquisitionControlPtr sptrAcquisitionControl = systemObj.createAcquisitionControl(cameraSptr);
    if (NULL == sptrAcquisitionControl)
    {
        return;
    }

    double exposureTimeValue = 0.0;
    CDoubleNode exposureTime = sptrAcquisitionControl->exposureTime();

    exposureTime.getValue(exposureTimeValue);
    printf("before change ,exposureTime is %f. thread ID :%d\n", exposureTimeValue, CThread::getCurrentThreadID());

    exposureTime.setValue(exposureTimeValue + 2);
    exposureTime.getValue(exposureTimeValue);
    printf("after change ,exposureTime is %f. thread ID :%d\n", exposureTimeValue, CThread::getCurrentThreadID());
}

void LogPrinterFunc(const char *log)
{
    return;
}

// ********************** 这部分处理与SDK操作相机无关，用于显示设备列表 begin*****************************
static void displayDeviceInfo(TVector<ICameraPtr> &vCameraPtrList)
{
    ICameraPtr cameraSptr;
    /* 打印Title行 */
    printf("\nIdx Type Vendor     Model      S/N             DeviceUserID    IP Address    \n");
    printf("------------------------------------------------------------------------------\n");
    for (int cameraIndex = 0; cameraIndex < vCameraPtrList.size(); cameraIndex++)
    {
        cameraSptr = vCameraPtrList[cameraIndex];
        /* Idx 设备列表的相机索引 最大表示字数：3 */
        printf("%-3d", cameraIndex + 1);

        /* Type 相机的设备类型（GigE，U3V，CL，PCIe）*/
        switch (cameraSptr->getType())
        {
        case ICamera::typeGige:
            printf(" GigE");
            break;
        case ICamera::typeU3v:
            printf(" U3V ");
            break;
        case ICamera::typeCL:
            printf(" CL  ");
            break;
        case ICamera::typePCIe:
            printf(" PCIe");
            break;
        default:
            printf("     ");
            break;
        }

        /* VendorName 制造商信息 最大表示字数：10 */
        const char *vendorName = cameraSptr->getVendorName();
        char vendorNameCat[11];
        if (strlen(vendorName) > 10)
        {
            strncpy(vendorNameCat, vendorName, 7);
            vendorNameCat[7] = '\0';
            strcat(vendorNameCat, "...");
            printf(" %-10.10s", vendorNameCat);
        }
        else
        {
            printf(" %-10.10s", vendorName);
        }

        /* ModeName 相机的型号信息 最大表示字数：10 */
        printf(" %-10.10s", cameraSptr->getModelName());

        /* Serial Number 相机的序列号 最大表示字数：15 */
        printf(" %-15.15s", cameraSptr->getSerialNumber());

        /* deviceUserID 自定义用户ID 最大表示字数：15 */
        const char *deviceUserID = cameraSptr->getName();
        char deviceUserIDCat[16] = {0};
        if (strlen(deviceUserID) > 15)
        {
            strncpy(deviceUserIDCat, deviceUserID, 12);
            deviceUserIDCat[12] = '\0';
            strcat(deviceUserIDCat, "...");
            printf(" %-15.15s", deviceUserIDCat);
        }
        else
        {
            //防止console显示乱码,UTF8转换成ANSI进行显示
            memcpy(deviceUserIDCat, deviceUserID, sizeof(deviceUserIDCat));
            printf(" %-15.15s", deviceUserIDCat);
        }

        /* IPAddress GigE相机时获取IP地址 */
        IGigECameraPtr gigeCameraPtr = IGigECamera::getInstance(cameraSptr);
        if (NULL != gigeCameraPtr.get())
        {
            CString ip = gigeCameraPtr->getIpAddress();
            printf(" %s", ip.c_str());
        }
        printf("\n");
    }
}

static char *trim(char *pStr)
{
    char *pDst = pStr;
    char *pTemStr = NULL;
    int ret = -1;

    if (pDst != NULL)
    {
        pTemStr = pDst + strlen(pStr) - 1;
        //除去字符串首部空格
        while (*pDst == ' ')
        {
            pDst++;
        }
        //除去字符串尾部空格
        while ((pTemStr > pDst) && (*pTemStr == ' '))
        {
            *pTemStr-- = '\0';
        }
    }
    return pDst;
}

static int isInputValid(char *pInpuStr)
{
    char numChar;
    char *pStr = pInpuStr;
    while (*pStr != '\0')
    {
        numChar = *pStr;
        if ((numChar > '9') || (numChar < '0'))
        {
            return -1;
        }
        pStr++;
    }
    return 0;
}

static int selectDevice(int cameraCnt)
{
    char inputStr[256] = {0};
    char *pTrimStr;
    char *find = NULL;
    int inputIndex = -1;
    int ret = -1;
    /* 提示用户选择 */
    printf("\nPlease input the camera index: ");
    while (1)
    {
        /* 获取输入内容字符串 */
        memset(inputStr, 0, sizeof(inputStr));
        fgets(inputStr, sizeof(inputStr), stdin);

        /* 清空输入缓存 */
        fflush(stdin);

        /* fgets比gets多吃一个换行符号，取出换行符号 */
        find = strchr(inputStr, '\n');
        if (find)
        {
            *find = '\0';
        }

        /*除去字符串首尾空格 */
        pTrimStr = trim(inputStr);
        //判断输入字符串是否为数字
        ret = isInputValid(pTrimStr);
        if (ret == 0)
        {
            /* 输入的字符串转换成为数字 */
            inputIndex = atoi(pTrimStr);
            /* 判断用户选择合法性 */
            inputIndex -= 1; //显示的Index是从1开始
            if ((inputIndex >= 0) && (inputIndex < cameraCnt))
            {
                break;
            }
        }

        printf("Input invalid! Please input the camera index: ");
    }
    return inputIndex;
}
// ********************** 这部分处理与SDK操作相机无关，用于显示设备列表 end*****************************

// int main()
// {
// #if 0
//     PrintOptions printOptions = {0};
//     printOptions.color = 1;
//     setPrintOptions(printOptions);

//     LogPrinterProc procFun = LogPrinterFunc;
//     setLogPrinter(procFun);
// #endif
//     ICameraPtr cameraSptr;

//     /* 发现设备 */
//     CSystem &systemObj = CSystem::getInstance();
//     TVector<ICameraPtr> vCameraPtrList;
//     bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
//     if (!isDiscoverySuccess)
//     {
//         printf("discovery device fail.\n");
//         return 0;
//     }

//     if (vCameraPtrList.size() == 0)
//     {
//         printf("no devices.\n");
//         return 0;
//     }

//     // print camera info (index,Type,vendor name, model,serial number,DeviceUserID,IP Address)
//     // 打印相机基本信息（序号,类型,制造商信息,型号,序列号,用户自定义ID,IP地址）
//     displayDeviceInfo(vCameraPtrList);
//     // int cameraIndex = selectDevice(vCameraPtrList.size());

//     // cameraSptr = vCameraPtrList[cameraIndex];
//     cameraSptr = vCameraPtrList[0];

//     /* GigE相机时，连接前设置相机Ip与网卡处于同一网段上 */
//     if (ICamera::typeGige == cameraSptr->getType())
//     {
//         if (autoSetCameraIP(cameraSptr) != 0)
//         {
//             printf("set camera Ip failed.\n");
//         }
//     }

//     /* 连接相机 */
//     if (!cameraSptr->connect())
//     {
//         printf("connect cameral failed.\n");
//         return 0;
//     }

//     /* 设置相机为连续拉流模式 */
//     setGrabMode(cameraSptr, true);

// #if 0
//     setGrabMode(cameraSptr, true);
//     setGrabMode(cameraSptr, false);
//     bool bContious = false;
//     getGrabMode(cameraSptr, bContious);
//     triggerSoftware(cameraSptr);

//     int64_t nWidth, nHeight;
//     setResolution(cameraSptr, 640, 480);
//     getResolution(cameraSptr, nWidth, nHeight);

//     setBinning(cameraSptr);

//     getMaxResolution(cameraSptr, nWidth, nHeight);

//     int64_t nX, nY, nROIWidth, nROIHeight;
//     setROI(cameraSptr, 120, 120, 640, 480);
//     getROI(cameraSptr, nX, nY, nROIWidth, nROIHeight);

//     getWidth(cameraSptr, nWidth);
//     getHeight(cameraSptr, nHeight);

//     double dExposureTime = 0;
//     setExposureTime(cameraSptr, 100, true);
//     setExposureTime(cameraSptr, 10000, false);
//     getExposureTime(cameraSptr, dExposureTime);

//     double dMinExposure, dMaxExposure;
//     getExposureTimeMinMaxValue(cameraSptr, dMinExposure, dMaxExposure);

//     double dGainRaw = 0;
//     double dGainRawMin = 0;
//     double dGainRawMax = 0;
//     setGainRaw(cameraSptr, 1.2);
//     getGainRaw(cameraSptr, dGainRaw);
//     getGainRawMinMaxValue(cameraSptr, dGainRawMin, dGainRawMax);

//     double dGamma = 0;
//     double dGammaMin = 0;
//     double dGammaMax = 0;
//     setGamma(cameraSptr, 0.8);
//     getGamma(cameraSptr, dGamma);
//     getGammaMinMaxValue(cameraSptr, dGammaMin, dGammaMax);

//     double dRedBalanceRatio = 0;
//     double dGreenBalanceRatio = 0;
//     double dBlueBalanceRatio = 0;
//     double dMinBalanceRatio = 0;
//     double dMaxBalanceRatio = 0;
//     setBalanceRatio(cameraSptr, 1.5, 1.5, 1.5);
//     getBalanceRatio(cameraSptr, dRedBalanceRatio, dGreenBalanceRatio, dBlueBalanceRatio);
//     getBalanceRatioMinMaxValue(cameraSptr, dMinBalanceRatio, dMaxBalanceRatio);

//     double dFrameRate = 0;
//     setAcquisitionFrameRate(cameraSptr, 20);
//     getAcquisitionFrameRate(cameraSptr, dFrameRate);

//     userSetSave(cameraSptr);
//     loadUserSet(cameraSptr);

//     double dDelayTime = 0;
//     setTriggerDelay(cameraSptr, 20);
//     getTriggerDelay(cameraSptr, dDelayTime);

//     bool bRisingEdge = true;
//     setLineTriggerMode(cameraSptr, bRisingEdge);
//     getLineTriggerMode(cameraSptr, bRisingEdge);

//     double dLineDebouncerTimeAbs = 0;
//     setLineDebouncerTimeAbs(cameraSptr, 20);
//     getLineDebouncerTimeAbs(cameraSptr, dLineDebouncerTimeAbs);

//     setOutputTime(cameraSptr, 1000);

//     setReverseX(cameraSptr, false);
//     setReverseY(cameraSptr, false);
// #endif

//     /* 创建流对象 */
//     IStreamSourcePtr streamPtr = systemObj.createStreamSource(cameraSptr);
//     if (NULL == streamPtr)
//     {
//         printf("create stream obj  fail.\r\n");
//         return 0;
//     }

//     /* 开始取图 */
//     bool isStartGrabbingSuccess = streamPtr->startGrabbing();
//     if (!isStartGrabbingSuccess)
//     {
//         printf("StartGrabbing  fail.\n");
//     }

//     CFrame frame;

//     for (int i = 1; i < 100; i++)
//     {
        
//         streamPtr->getFrame(frame, 300);
//         std::cout << "see: " << frame.valid() << std::endl;

//         // std::cout<<std::setbase(16)<<frame.getImagePixelFormat()<<std::endl;
//         CFrame frameClone = frame.clone();
//         Dahua::Memory::TSharedPtr<FrameBuffer> PtrFrameBuffer(new FrameBuffer(frameClone));
//         if (!PtrFrameBuffer)
//         {
//             printf("create PtrFrameBuffer failed!\n");
//             return false;
//         }

//         uint8_t *pSrcData = new (std::nothrow) uint8_t[frameClone.getImageSize()];
//         if (pSrcData)
//         {
//             memcpy(pSrcData, frameClone.getImage(), frameClone.getImageSize());
//         }
//         else
//         {
//             printf("new pSrcData failed!\n");
//             return false;
//         }

//         int dstDataSize = 0;
//         IMGCNV_SOpenParam openParam;
//         openParam.width = PtrFrameBuffer->Width();
//         openParam.height = PtrFrameBuffer->Height();
//         openParam.paddingX = PtrFrameBuffer->PaddingX();
//         openParam.paddingY = PtrFrameBuffer->PaddingY();
//         openParam.dataSize = PtrFrameBuffer->DataSize();
//         openParam.pixelForamt = PtrFrameBuffer->PixelFormat();
        
//         cv::Mat a1;

//         // cv::Mat by8Mat(cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight), CV_8UC1, m_pbyBuffer);
//         // cv::cvtColor(by8Mat, frame, cv::COLOR_BayerGB2RGB_EA);
//         // cv::Mat by8Mat(cv::Size(openParam.width, openParam.height), CV_8UC1, PtrFrameBuffer->bufPtr());

//         unsigned char *m_pbyBuffer=(uchar*)frame.getImage();
//         cv::Mat by8Mat(cv::Size(openParam.width, openParam.height), CV_8UC1, m_pbyBuffer);

//         // getImage

//         cv::cvtColor(by8Mat, a1, cv::COLOR_BayerGB2RGB_EA);
//         cv::imshow("sa",a1);
//         cv::waitKey(1);

//         frame.reset();
        
//         // IMGCNV_EErr status = IMGCNV_ConvertToBGR24(pSrcData, &openParam, PtrFrameBuffer->bufPtr(), &dstDataSize);
//         // if (IMGCNV_SUCCESS != status)
//         // {
//         //     delete[] pSrcData;
//         //     return false;
//         // }

//         // delete[] pSrcData;

//         // //将读进来的帧数据转化为opencv中的Mat格式操作
//         // cv::Size size;
//         // size.height = PtrFrameBuffer->Height();
//         // size.width = PtrFrameBuffer->Width();
//         // cv::Mat img = cv::Mat(size, CV_8UC3, PtrFrameBuffer->bufPtr()).clone();
//         // //    PtrFrameBuffer.reset();
//         // frameClone.reset();
//     }

//     // /* 创建取流线程 */
//     // Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr(new StreamRetrieve(streamPtr));
//     // if (NULL == streamThreadSptr)
//     // {
//     //     printf("create thread obj failed.\n");
//     //     return 0;
//     // }

//     //  /* 线程开始取图 */
//     // streamThreadSptr->start();

//     //  /* 取图2秒 */
//     // CThread::sleep(2000);

//     // /* 停止拉流线程 */
//     // streamThreadSptr->stop();

//     /* 停止采图 */
//     streamPtr->stopGrabbing();

//     // /* 修改相机曝光时间 */
//     // modifyCamralExposureTime(systemObj, cameraSptr);

//     /* 断开相机 */
//     if (!cameraSptr->disConnect())
//     {
//         printf("disConnect camera failed\n");
//         return 0;
//     }

//     printf("disConnect successfully thread ID :%d\n", CThread::getCurrentThreadID());

//     return 1;
// }
