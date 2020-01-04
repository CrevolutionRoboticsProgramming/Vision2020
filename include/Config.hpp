#pragma once

#include <vector>

#include "Setting.hpp"

class Config
{
private:
    std::string mTag;

public:
    std::vector<Setting *> settings{};

    Config(std::string tag)
    {
        mTag = tag;
    }

    std::string getTag()
    {
        return mTag;
    }
};

class SystemConfig : public Config
{
public:
    BoolSetting verbose{"verbose"};
    BoolSetting tuning{"tuning"};
    IntSetting videoPort{"videoPort"};
    IntSetting robotPort{"robotPort"};
    IntSetting receivePort{"receivePort"};

    SystemConfig() : Config("system")
    {
        settings.push_back(std::move(&verbose));
        settings.push_back(std::move(&tuning));
        settings.push_back(std::move(&videoPort));
        settings.push_back(std::move(&robotPort));
        settings.push_back(std::move(&receivePort));
    }
};

class VisionConfig : public Config
{
public:
    IntSetting lowHue{"lowHue"};
    IntSetting lowSaturation{"lowSaturation"};
    IntSetting lowValue{"lowValue"};
    IntSetting highHue{"highHue"};
    IntSetting highSaturation{"highSaturation"};
    IntSetting highValue{"highValue"};
    IntSetting dilationPasses{"dilationPasses"};
    IntSetting minArea{"minArea"};
    IntSetting maxArea{"maxArea"};
    DoubleSetting minContourToBoundingBoxRatio{"minContourToBoundingBoxRatio"};
    DoubleSetting maxContourToBoundingBoxRatio{"maxContourToBoundingBoxRatio"};
    IntSetting allowableError{"allowableError"};

    VisionConfig() : Config("vision")
    {
        settings.push_back(std::move(&lowHue));
        settings.push_back(std::move(&lowSaturation));
        settings.push_back(std::move(&lowValue));
        settings.push_back(std::move(&highHue));
        settings.push_back(std::move(&highSaturation));
        settings.push_back(std::move(&highValue));
        settings.push_back(std::move(&dilationPasses));
        settings.push_back(std::move(&minArea));
        settings.push_back(std::move(&maxArea));
        settings.push_back(std::move(&minContourToBoundingBoxRatio));
        settings.push_back(std::move(&maxContourToBoundingBoxRatio));
        settings.push_back(std::move(&allowableError));
    }
};

class UvccamConfig : public Config
{
public:
    IntSetting width{"width"};
    IntSetting height{"height"};
    IntSetting everyNthFrame{"everyNthFrame"};
    IntSetting exposure{"exposure"};
    IntSetting exposureAuto{"exposureAuto"};

    UvccamConfig() : Config("uvccam")
    {
        settings.push_back(std::move(&width));
        settings.push_back(std::move(&height));
        settings.push_back(std::move(&everyNthFrame));
        settings.push_back(std::move(&exposure));
        settings.push_back(std::move(&exposureAuto));
    }
};

class RaspicamConfig : public Config
{
public:
    IntSetting width{"width"};
    IntSetting height{"height"};
    IntSetting sensorMode{"sensorMode"};
    IntSetting fps{"fps"};

    // -100 to 100; default: 0
    IntSetting sharpness{"sharpness"};

    // -100 to 100; default: 0
    IntSetting contrast{"contrast"};

    // -100 to 100; default: ?
    IntSetting brightness{"brightness"};

    // -100 to 100; default: 0
    IntSetting saturation{"saturation"};

    // in microseconds
    IntSetting shutterSpeed{"shutterSpeed"};

    // 0 - off; 1 - auto; plus more
    IntSetting whiteBalanceMode{"whiteBalanceMode"};

    // 0 - off; 1 - auto; 2 - night; plus more
    IntSetting exposureMode{"exposureMode"};

    // -10 to 10
    IntSetting exposureCompensation{"exposureCompensation"};
    IntSetting horizontalFov{"horizontalFov"};

    RaspicamConfig() : Config("raspicam")
    {
        settings.push_back(std::move(&width));
        settings.push_back(std::move(&height));
        settings.push_back(std::move(&sensorMode));
        settings.push_back(std::move(&fps));
        settings.push_back(std::move(&sharpness));
        settings.push_back(std::move(&contrast));
        settings.push_back(std::move(&brightness));
        settings.push_back(std::move(&saturation));
        settings.push_back(std::move(&shutterSpeed));
        settings.push_back(std::move(&whiteBalanceMode));
        settings.push_back(std::move(&exposureMode));
        settings.push_back(std::move(&exposureCompensation));
        settings.push_back(std::move(&horizontalFov));
    }
};
