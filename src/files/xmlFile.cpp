//------------------------------------------ Includes ---------------------------------------------

#include "xmlFile.h"
#include "utils/stringUtils.h"
#include "utils/base64.h"
#include "maths/maths.h"
#include "platform/file.h"

using namespace IslSdk;

//---------------------------------- Private Function Prototypes ----------------------------------

std::string encodeNonAsciiChars(const std::string& content, const std::string& extraEscChars);
std::string decodeEscapeChars(const std::string& str);

//------------------------------------ XmlElement class ---------------------------------------
XmlElement::XmlElement(const std::string& _name, const std::string& _content) : m_name(_name), m_content(_content)
{
}
//-------------------------------------------------------------------------------------------------
void XmlElement::addAttribute(const std::string& key, const std::string& value)
{
    m_attributes[key] = encodeNonAsciiChars(value, "&<\"");
}
//-------------------------------------------------------------------------------------------------
std::string XmlElement::getAttribute(const std::string& key) const
{
    return m_attributes.count(key) ? m_attributes.at(key) : "";
}
//-------------------------------------------------------------------------------------------------
XmlElement::XmlElementPtr XmlElement::addString(const std::string& name, const std::string& val)
{
    XmlElementPtr child = std::make_shared<XmlElement>(name, val);
    m_elements.push_back(child);
    return child;
}
//-------------------------------------------------------------------------------------------------
XmlElement::XmlElementPtr XmlElement::addBytes(const std::string& name, const uint8_t* val, uint_t size)
{
    std::string content = Base64::encode(val, size);

    XmlElementPtr child = std::make_shared<XmlElement>(name, content);
    child->m_attributes["encoding"] = "base64";
    m_elements.push_back(child);

    return child;
}
//-------------------------------------------------------------------------------------------------
XmlElement::XmlElementPtr XmlElement::addReal(const std::string& name, real_t val, uint_t precision)
{
    std::string content = StringUtils::realToStr(val, 0, precision, 0, false);
    XmlElementPtr child = std::make_shared<XmlElement>(name, content);
    m_elements.push_back(child);

    return child;
}
//-------------------------------------------------------------------------------------------------
XmlElement::XmlElementPtr XmlElement::addUint(const std::string& name, uint_t val)
{
    std::string content = StringUtils::uintToStr(val, 0);
    XmlElementPtr child = std::make_shared<XmlElement>(name, content);
    m_elements.push_back(child);

    return child;
}
//-------------------------------------------------------------------------------------------------
XmlElement::XmlElementPtr XmlElement::addInt(const std::string& name, int_t val)
{
    std::string content = StringUtils::intToStr(val, 0, false);
    XmlElementPtr child = std::make_shared<XmlElement>(name, content);
    m_elements.push_back(child);

    return child;
}
//-------------------------------------------------------------------------------------------------
XmlElement::XmlElementPtr XmlElement::addBool(const std::string& name, bool_t val)
{
    std::string content = val ? "true" : "false";
    XmlElementPtr child = std::make_shared<XmlElement>(name, content);
    m_elements.push_back(child);

    return child;
}
//-------------------------------------------------------------------------------------------------
std::string XmlElement::getString(const std::string& name, const std::string& defaultVal) const
{
    const XmlElement* element = find(name);
    if (element)
    {
        return element->content;
    }

    return defaultVal;
}
//-------------------------------------------------------------------------------------------------
uint_t XmlElement::getBytes(const std::string& name, uint8_t* buf, uint_t bufSize) const
{
    const XmlElement* element = find(name);
    if (element)
    {
        if (element->getAttribute("encoding") == "base64")
        {
            return Base64::decode(element->content, buf, bufSize);
        }
        else
        {
            bufSize = Math::min<uint_t>(bufSize, element->content.length());
            Mem::memcpy(buf, element->content.c_str(), bufSize);
            return bufSize;
        }
    }

    return 0;
}
//-------------------------------------------------------------------------------------------------
real_t XmlElement::getReal(const std::string& name, real_t defaultVal) const
{
    const XmlElement* element = find(name);
    if (element)
    {
        bool_t error = false;
        real_t val = StringUtils::toReal(element->content, error);
        if (!error)
        {
            return val;
        }
    }

    return defaultVal;
}
//-------------------------------------------------------------------------------------------------
uint_t XmlElement::getUint(const std::string& name, uint_t defaultVal) const
{
    const XmlElement* element = find(name);
    if (element)
    {
        bool_t error = false;
        uint_t val = StringUtils::toUint(element->content, error);
        if (!error)
        {
            return val;
        }
    }

    return defaultVal;
}
//-------------------------------------------------------------------------------------------------
int_t XmlElement::getInt(const std::string& name, int_t defaultVal) const
{
    const XmlElement* element = find(name);
    if (element)
    {
        bool_t error = false;
        int_t val = StringUtils::toInt(element->content, error);
        if (!error)
        {
            return val;
        }
    }

    return defaultVal;
}
//-------------------------------------------------------------------------------------------------
bool_t XmlElement::getBool(const std::string& name, bool_t defaultVal) const
{
    const XmlElement* element = find(name);
    if (element)
    {
        if (StringUtils::compareNoCase(element->content, "true"))
        {
            return true;
        }
        else if (StringUtils::compareNoCase(element->content, "false"))
        {
            return false;
        }
    }

    return defaultVal;
}
//-------------------------------------------------------------------------------------------------
const XmlElement::XmlElementPtr XmlElement::addElement(const std::string& name)
{
    XmlElementPtr child = std::make_shared<XmlElement>(name, "");
    m_elements.push_back(child);
    return child;
}
//-------------------------------------------------------------------------------------------------
const XmlElement::XmlElementPtr XmlElement::findElement(const std::string& name)
{
    for (const XmlElementPtr& child : m_elements)
    {
        if (child->name == name)
        {
            return child;
        }
    }

    return XmlElementPtr();
}
//-------------------------------------------------------------------------------------------------
const XmlElement* XmlElement::find(const std::string& name) const
{
    for (auto const& child : this->elements)
    {
        if (child->name == name)
        {
            return child.get();
        }
    }

    return nullptr;
}
//-------------------------------------------------------------------------------------------------

//------------------------------------ XmlFile class ----------------------------------------------
XmlFile::XmlFile()
{
    m_root = std::make_shared<XmlElement>("", "");
}
//-------------------------------------------------------------------------------------------------
bool_t XmlFile::open(const std::string& fileName)
{
    std::ifstream file(fileName);
    if (!file.fail())
    {
        parseElement(nullptr, file);
        return true;
    }

    return false;
}
//-------------------------------------------------------------------------------------------------
bool_t XmlFile::save(const std::string& fileName)
{
    if (!m_root) return false;

    File::createDir(fileName);
    std::ofstream file;
    file.open(fileName, std::ofstream::trunc);
    if (file.is_open())
    {
        file << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << std::endl;
        saveElement(file, m_root, 0);
        file.close();

        return true;
    }

    return false;
}
//-------------------------------------------------------------------------------------------------
XmlElementPtr XmlFile::setRoot(const std::string& name)
{
    m_root = std::make_shared<XmlElement>(name, "");
    return m_root;
}
//-------------------------------------------------------------------------------------------------
bool_t XmlFile::hasKey(const XmlElementPtr& element, const std::string& key)
{
    return element && element->m_attributes.count(key);
}
//-------------------------------------------------------------------------------------------------
bool_t XmlFile::getValue(const XmlElementPtr& element, const std::string& key, std::string& value)
{
    if (element)
    {
        std::map<std::string, std::string>::const_iterator it = element->m_attributes.find(key);
        if (it != element->m_attributes.end())
        {
            value = it->second;
            return true;
        }
    }

    return false;
}
//-------------------------------------------------------------------------------------------------
void XmlFile::saveElement(std::ofstream& file, XmlElementPtr& element, uint_t depth)
{
    if (element)
    {
        file << std::string(depth * 4, ' ') << '<' << element->name;

        for (auto const& attrib : element->m_attributes)
        {
            file << ' ' << attrib.first << '=';
            file << '\"' << attrib.second << '\"';
        }

        file << '>';

        if(!element->elements.empty())
        {
            file << std::endl;
            for (XmlElementPtr& child : element->m_elements)
            {
                saveElement(file, child, depth + 1);
            }

            file << std::string(depth * 4, ' ') << "</" << element->name << '>';
        }
        else
        {
            std::string cont = encodeNonAsciiChars(element->content, "&<");
            file << cont << "</" << element->name << '>';
        }

        file << std::endl;
    }
}
//-------------------------------------------------------------------------------------------------
void XmlFile::parseElement(XmlElementPtr parent, std::ifstream& file)
{
    std::string line;

    while (!file.eof())
    {
        std::getline(file, line);
        auto begin = line.find_first_of('<');
        if (std::string::npos == begin) continue;

        line = line.substr(begin);

        if (line.front() == '<')
        {
            if (line.at(1) == '/')
            {
                break; // end of element on newline: </name>
            }

            std::string name = parseName(line);

            if (!name.empty())
            {
                XmlElementPtr element = std::make_shared<XmlElement>(name);

                auto endElem = line.find_first_of('>');
                if (std::string::npos != endElem && (endElem - name.size() > 2))
                {
                    parseAttributes(element, line.substr(name.size() + 2));
                }

                if (!parent)
                {
                    m_root = element;
                }
                else
                {
                    parent->m_elements.push_back(element);
                }

                size_t closing = line.rfind("</" + name + ">");

                if (std::string::npos != closing && closing > endElem++)
                {
                    // end of element with no children
                    element->m_content = decodeEscapeChars(line.substr(endElem, closing - endElem));
                }
                else
                {
                    // recurse on current element to parse nested children
                    parseElement(element, file);
                }
            }
        }
    }
}
//-------------------------------------------------------------------------------------------------
void XmlFile::parseAttributes(XmlElementPtr& element, const std::string& line)
{
    bool_t inKey = true;
    bool_t inValue = false;
    std::string key, value;

    for (char c : line)
    {
        if (inKey)
        {
            if (c == '>')
            {
                break;
            }
            else if (c == ' ' && key.empty())
            {
                continue;   // skip leading whitespace
            }
            else if (c == '=')
            {
                inKey = false;
            }
            else
            {
                key += c;
            }
        }
        else
        {
            if (c == '\'' || c == '\"')
            {
                if (!inValue)
                {
                    inValue = true;
                }
                else
                {
                    // add attribute if key and value are valid
                    if (!key.empty() && !value.empty())
                    {
                        element->m_attributes[key] = decodeEscapeChars(value);
                        key.clear();
                        value.clear();
                    }

                    inKey = true;
                    inValue = false;
                }
            }
            else if (inValue)
            {
                value += c;
            }
        }
    }
}
//-------------------------------------------------------------------------------------------------
std::string XmlFile::parseName(const std::string& line)
{
    std::string name;
    bool_t inName = false;

    for (char c : line)
    {
        if (!inName)
        {
            if (c == '<' || c == ' ' || ((c >= '0' && c <= '9') || c == '.' || c == '-'))
            {
                continue;
            }
            inName = true;
        }

        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_' || ((c >= '0' && c <= '9') || c == '.' || c == '-'))
        {
            name += c;
        }
        else
        {
            break;
        }
    }

    return name;
}
//--------------------------------------------------------------------------------------------------
std::string encodeNonAsciiChars(const std::string& content, const std::string& extraEscChars)
{
    std::string encoded;

    for (const char& c : content)
    {
        if (c < ' ' || c > '~' || std::string::npos != extraEscChars.find(c))
        {
            encoded += "&#";
            encoded += StringUtils::uintToStr(static_cast<uint_t>(c));
            encoded += ";";
        }
        else
        {
            encoded += c;
        }
    }

    return encoded;
}
//--------------------------------------------------------------------------------------------------
std::string decodeEscapeChars(const std::string& str)
{
    bool_t err = false;
    std::string out, seq;

    for (auto const& c : str)
    {
        if (c == '&')
        {
            seq = c;
        }
        else if (!seq.empty())
        {
            if (c == ' ')
            {
                out += seq + c;
                seq.clear();
            }
            else if (c == ';')
            {
                if (0 == seq.find("&#x"))
                {
                    if (seq.size() > 3)
                    {
                        uint_t index = 3;
                        out += static_cast<char>(StringUtils::hexToUint(seq, index, 0, err));
                    }
                }
                else if (0 == seq.find("&#"))
                {
                    if (seq.size() > 2)
                    {
                        uint_t index = 2;
                        out += static_cast<char>(StringUtils::toUint(seq, index, 0, err));
                    }
                }
                else if (seq == "&amp")
                {
                    out += '&';
                }
                else if (seq == "&lt")
                {
                    out += '<';
                }
                else if (seq == "&gt")
                {
                    out += '>';
                }
                else if (seq == "&quot")
                {
                    out += '\"';
                }
                else if (seq == "&apos")
                {
                    out += '\'';
                }

                seq.clear();
            }
            else
            {
                seq += c;
            }
        }
        else
        {
            out += c;
        }
    }

    return out;
}
//-------------------------------------------------------------------------------------------------
