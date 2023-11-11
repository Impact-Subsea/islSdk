#ifndef XMLSETTINGS_H_
#define XMLSETTINGS_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "files/xmlFile.h"
#include "maths/vector3.h"
#include "maths/matrix3x3.h"
#include "devices/device.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace XmlSettings
    {
        void saveDeviceInfo(const Device::Info& info, XmlElementPtr& xml);
        bool_t loadDeviceInfo(Device::Info& info, const XmlElementPtr& xml);
        void saveBias(const Vector3& bias, XmlElementPtr& xml);
        bool_t loadBias(Vector3& bias, const XmlElementPtr& xml);
        void saveTransform(const Matrix3x3& transform, XmlElementPtr& xml);
        bool_t loadTransform(Matrix3x3& transform, const XmlElementPtr& xml);
        void saveScript(const DeviceScript& script, XmlElementPtr& xml);
        bool_t loadScript(DeviceScript& script, const XmlElementPtr& xml);
    }
}
//--------------------------------------------------------------------------------------------------
#endif
