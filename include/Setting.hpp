#pragma once

#include <string>

class Setting
{
private:
    std::string mTag;

public:
    Setting(std::string tag)
    {
        mTag = tag;
    }

    virtual ~Setting()
    {
    }

    std::string getTag()
    {
        return mTag;
    }
};

class IntSetting : public Setting
{
public:
    int value;

    IntSetting(std::string tag) : Setting(tag)
    {
    }
};

class DoubleSetting : public Setting
{
public:
    double value;

    DoubleSetting(std::string tag) : Setting(tag)
    {
    }
};

class BoolSetting : public Setting
{
public:
    bool value;

    BoolSetting(std::string tag) : Setting(tag)
    {
    }
};

class StringSetting : public Setting
{
public:
    std::string value;

    StringSetting(std::string tag) : Setting(tag)
    {
    }
};
