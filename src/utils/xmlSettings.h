#ifndef XMLSETTINGS_H_
#define XMLSETTINGS_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "files/xmlFile.h"
#include "maths/vector.h"
#include "maths/matrix.h"
#include "devices/device.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace XmlSettings
    {
        void saveDeviceInfo(const Device::Info& info, XmlElementPtr& xml);
        bool_t loadDeviceInfo(Device::Info& info, const XmlElementPtr& xml);
        void saveBias(const Math::Vector3& bias, XmlElementPtr& xml);
        bool_t loadBias(Math::Vector3& bias, const XmlElementPtr& xml);
        void saveTransform(const Math::Matrix3x3& transform, XmlElementPtr& xml);
        bool_t loadTransform(Math::Matrix3x3& transform, const XmlElementPtr& xml);
        void saveScript(const DeviceScript& script, XmlElementPtr& xml);
        bool_t loadScript(DeviceScript& script, const XmlElementPtr& xml);
    }
}
//--------------------------------------------------------------------------------------------------
#endif
