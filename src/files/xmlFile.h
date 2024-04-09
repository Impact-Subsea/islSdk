#ifndef XMLFILE_H_
#define XMLFILE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <memory>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class XmlElement
    {
    public:
        typedef std::shared_ptr<XmlElement> XmlElementPtr;

        XmlElement(const std::string& name, const std::string& content = "");
        void addAttribute(const std::string& key, const std::string& value);
        std::string getAttribute(const std::string& key) const;
        const XmlElementPtr addElement(const std::string& name);
        const XmlElementPtr findElement(const std::string& name);

        XmlElementPtr addString(const std::string& name, const std::string& val);
        XmlElementPtr addBytes(const std::string& name, const uint8_t* val, uint_t size);
        XmlElementPtr addReal(const std::string& name, real_t val, uint_t precision);
        XmlElementPtr addUint(const std::string& name, uint_t val);
        XmlElementPtr addInt(const std::string& name, int_t val);
        XmlElementPtr addBool(const std::string& name, bool_t val);

        std::string getString(const std::string& name, const std::string& defaultVal) const;
        uint_t getBytes(const std::string& name, uint8_t* buf, uint_t bufSize) const;
        real_t getReal(const std::string& name, real_t defaultVal) const;
        uint_t getUint(const std::string& name, uint_t defaultVal) const;
        int_t getInt(const std::string& name, int_t defaultVal) const;
        bool_t getBool(const std::string& name, bool_t defaultVal) const;

        const std::string& name = m_name;
        const std::string& content = m_content;
        const std::vector<XmlElementPtr>& elements = m_elements;

    private:
        const XmlElement* find(const std::string& name) const;

        std::string m_name;
        std::string m_content;
        std::map<std::string, std::string> m_attributes;
        std::vector<XmlElementPtr> m_elements;

        friend class XmlFile;
    };

    typedef XmlElement::XmlElementPtr XmlElementPtr;

    class XmlFile
    {
    public:
        XmlFile();
        bool_t open(const std::string& fileName);
        bool_t save(const std::string& fileName);
        XmlElementPtr setRoot(const std::string& name);
        XmlElementPtr root() { return m_root; }

    private:
        void parseElement(XmlElementPtr parent, std::ifstream& file);
        void parseAttributes(XmlElementPtr& element, const std::string& line);

        std::string parseName(const std::string& name);
        bool_t hasKey(const XmlElementPtr& element, const std::string& key);
        bool_t getValue(const XmlElementPtr& element, const std::string& key, std::string& value);

        void saveElement(std::ofstream& file, XmlElementPtr& element, uint_t depth);
        XmlElementPtr m_root;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
