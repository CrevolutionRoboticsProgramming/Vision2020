#include <iostream>
#include <string>
#include <fstream>
#include <functional>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>

#include "Config.hpp"
#include "MJPEGWriter/MJPEGWriter.h"
#include "Thread.hpp"
#include "Contour.hpp"
#include "UDPHandler.hpp"

std::string configDir{"resources/config.yaml"};

SystemConfig systemConfig{};
VisionConfig visionConfig{};
UvccamConfig uvccamConfig{};
RaspicamConfig raspicamConfig{};

template <typename T>
T getYamlValue(YAML::Node yaml, std::string category, std::string setting)
{
    if (yaml[setting])
        return yaml[setting].as<T>();

    if (!yaml[category] || !yaml[category][setting])
    {
        std::cout << "Could not find setting " << setting << " in category " << category << '\n';
        return T{};
    }

    return yaml[category][setting].as<T>();
}

void parseConfigs(YAML::Node yamlConfig)
{
    std::vector<Config *> configs{};
    configs.push_back(std::move(&systemConfig));
    configs.push_back(std::move(&visionConfig));
    configs.push_back(std::move(&uvccamConfig));
    configs.push_back(std::move(&raspicamConfig));

    for (Config *config : configs)
    {
        for (Setting *setting : config->settings)
        {
            if (dynamic_cast<IntSetting *>(setting) != nullptr)
            {
                dynamic_cast<IntSetting *>(setting)->value = getYamlValue<int>(yamlConfig, config->getTag(), setting->getTag());
            }
            else if (dynamic_cast<DoubleSetting *>(setting) != nullptr)
            {
                dynamic_cast<DoubleSetting *>(setting)->value = getYamlValue<double>(yamlConfig, config->getTag(), setting->getTag());
            }
            else if (dynamic_cast<BoolSetting *>(setting) != nullptr)
            {
                dynamic_cast<BoolSetting *>(setting)->value = getYamlValue<bool>(yamlConfig, config->getTag(), setting->getTag());
            }
            else if (dynamic_cast<StringSetting *>(setting) != nullptr)
            {
                std::string value = getYamlValue<std::string>(yamlConfig, config->getTag(), setting->getTag());
                if (value != std::string{})
                    dynamic_cast<StringSetting *>(setting)->value = value;
            }
        }
    }

    if (systemConfig.verbose.value)
        std::cout << "Parsed Configs\n";
}

std::string getCurrentConfig()
{
    std::vector<Config *> configs{};
    configs.push_back(std::move(&systemConfig));
    configs.push_back(std::move(&visionConfig));
    configs.push_back(std::move(&uvccamConfig));
    configs.push_back(std::move(&raspicamConfig));

    YAML::Node currentConfig;
    for (Config *config : configs)
    {
        for (Setting *setting : config->settings)
        {
            if (dynamic_cast<IntSetting *>(setting) != nullptr)
            {
                currentConfig[config->getTag()][setting->getTag()] = dynamic_cast<IntSetting *>(setting)->value;
            }
            else if (dynamic_cast<DoubleSetting *>(setting) != nullptr)
            {
                currentConfig[config->getTag()][setting->getTag()] = dynamic_cast<DoubleSetting *>(setting)->value;
            }
            else if (dynamic_cast<BoolSetting *>(setting) != nullptr)
            {
                currentConfig[config->getTag()][setting->getTag()] = dynamic_cast<BoolSetting *>(setting)->value;
            }
            else if (dynamic_cast<StringSetting *>(setting) != nullptr)
            {
                currentConfig[config->getTag()][setting->getTag()] = dynamic_cast<StringSetting *>(setting)->value;
            }
        }
    }

    YAML::Emitter configEmitter;
    configEmitter.SetMapFormat(YAML::Block);
    configEmitter << currentConfig;

    return configEmitter.c_str();
}

bool streamProcessingVideo{false};

// Streamer
class : public Thread
{
public:
    void stop() override
    {
        system("pkill mjpg_streamer");
        Thread::stop();
    }

private:
    void run() override
    {
        std::ostringstream command;

        // Configures camera settings
        command << "v4l2-ctl -c exposure_auto=" << uvccamConfig.exposureAuto.value << " -c exposure_absolute=" << uvccamConfig.exposure.value;
        system(command.str().c_str());

        if (systemConfig.verbose.value)
            std::cout << "Configured Exposure\n";

        command = std::ostringstream{};
        command << "cd ../mjpg-streamer-master/mjpg-streamer-experimental/ && ./mjpg_streamer -i 'input_uvc.so -r "
                << uvccamConfig.width.value << "x" << uvccamConfig.height.value << " -e " << uvccamConfig.everyNthFrame.value
                << "' -o 'output_http.so -p " << systemConfig.videoPort.value << "'";
        system(command.str().c_str());
    }
} streamThread;

// Vision Processing
class : public Thread
{
private:
    void run() override
    {
        cv::Mat morphElement{cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3))};

        UDPHandler robotUDPHandler{9999};
        boost::asio::ip::udp::endpoint robotEndpoint{boost::asio::ip::address::from_string("10.28.51.2"), systemConfig.robotPort.value};

        std::ostringstream pipeline;
        pipeline << "rpicamsrc sensor-mode=7 shutter-speed=" << raspicamConfig.shutterSpeed.value << " exposure-mode=" << raspicamConfig.exposureMode.value
                 << " awb-mode=" << raspicamConfig.whiteBalanceMode.value
                 << " sharpness=" << raspicamConfig.sharpness.value << " contrast=" << raspicamConfig.sharpness.value
                 << " brightness=" << raspicamConfig.brightness.value << " saturation=" << raspicamConfig.saturation.value
                 << " ! video/x-raw,width=" << raspicamConfig.width.value << ",height=" << raspicamConfig.height.value << ",framerate="
                 << raspicamConfig.fps.value << "/1 ! appsink";

        cv::VideoCapture processingCamera{pipeline.str(), cv::CAP_GSTREAMER};
        MJPEGWriter mjpegWriter{systemConfig.videoPort.value};

        if (systemConfig.verbose.value && !processingCamera.isOpened())
            std::cout << "Could not open processing camera!\n";

        cv::Mat streamFrame;
        cv::Mat processingFrame;
        for (int frameNumber{1}; !stopFlag; ++frameNumber)
        {
            if (!processingCamera.isOpened())
                continue;

            if (processingCamera.grab())
                processingCamera.read(processingFrame);
            else
                continue;

            if (processingFrame.empty())
                continue;

            if (systemConfig.verbose.value && frameNumber % 10 == 0)
                std::cout << "Grabbed Frame " + std::to_string(frameNumber) + '\n';

            if (streamProcessingVideo)
            {
                if (!mjpegWriter.isOpened())
                {
                    mjpegWriter.write(processingFrame);
                    mjpegWriter.start();
                }
            }
            else if (mjpegWriter.isOpened())
                mjpegWriter.stop();

            // Writes frame to be streamed when not tuning
            if (streamProcessingVideo && !systemConfig.tuning.value)
                mjpegWriter.write(processingFrame);

            // Extracts the contours
            std::vector<std::vector<cv::Point>> rawContours;
            std::vector<Contour> contours;
            cv::cvtColor(processingFrame, processingFrame, cv::COLOR_BGR2HSV);
            cv::inRange(processingFrame, cv::Scalar{visionConfig.lowHue.value, visionConfig.lowSaturation.value, visionConfig.lowValue.value}, cv::Scalar{visionConfig.highHue.value, visionConfig.highSaturation.value, visionConfig.highValue.value}, processingFrame);
            cv::dilate(processingFrame, processingFrame, morphElement, cv::Point(-1, -1), visionConfig.dilationPasses.value);

            // Writes vision processing frame to be streamed if requested
            if (streamProcessingVideo && systemConfig.tuning.value)
            {
                if (!streamFrame.empty())
                {
                    // Writes the frame prepared last iteration
                    mjpegWriter.write(streamFrame);
                }

                // Begins preparing the new frame
                processingFrame.copyTo(streamFrame);
                cv::cvtColor(streamFrame, streamFrame, cv::COLOR_GRAY2BGR);
            }

            cv::Canny(processingFrame, processingFrame, 0, 0);
            cv::findContours(processingFrame, rawContours, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

            for (std::vector<cv::Point> pointsVector : rawContours)
            {
                Contour newContour{pointsVector, visionConfig.allowableError.value};

                if (newContour.isValid(visionConfig.minArea.value, visionConfig.maxArea.value, visionConfig.minContourToBoundingBoxRatio.value, visionConfig.maxContourToBoundingBoxRatio.value))
                {
                    contours.push_back(newContour);
                }
            }

            Contour target{};

            switch (contours.size())
            {
            case 0:
                continue;
            case 1:
                target = contours.at(0);
                break;
            default:
                target = contours.at(0);
                double leastDistance = std::numeric_limits<double>::max();

                for (Contour contour : contours)
                {
                    double distanceToCenter = std::sqrt(std::pow(contour.center.x - (processingFrame.cols / 2), 2) + std::pow(contour.center.y - (processingFrame.rows / 2), 2));

                    if (distanceToCenter < leastDistance)
                        target = contour;
                }
            }

            double horizontalAngleError{-((processingFrame.cols / 2.0) - target.center.x) / processingFrame.cols * raspicamConfig.horizontalFov.value};

            robotUDPHandler.sendTo(std::to_string(horizontalAngleError), robotEndpoint);

            // Preps frame to be streamed
            if (streamProcessingVideo && systemConfig.tuning.value)
            {
                cv::rectangle(streamFrame, target.boundingBox, cv::Scalar{0, 255, 0}, 2);
                cv::line(streamFrame, cv::Point{target.center.x, target.center.y - 10}, cv::Point{target.center.x, target.center.y + 10}, cv::Scalar{0, 255, 0}, 2);
                cv::line(streamFrame, cv::Point{target.center.x - 10, target.center.y}, cv::Point{target.center.x + 10, target.center.y}, cv::Scalar{0, 255, 0}, 2);
                cv::putText(streamFrame, "Horizontal Angle of Error: " + std::to_string(horizontalAngleError), cv::Point{0, 12}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar{255, 255, 255});
            }

            std::this_thread::sleep_for(std::chrono::milliseconds{10});
        }

        mjpegWriter.stop();
    }
} processVisionThread;

int main()
{
    parseConfigs(YAML::LoadFile(configDir));

    streamThread.start();
    processVisionThread.start();

    UDPHandler communicatorUDPHandler{systemConfig.receivePort.value};

    while (true)
    {
        if (communicatorUDPHandler.getMessage() != "")
        {
            std::string configsLabel{"CONFIGS:"};

            // If we were sent configs
            if (communicatorUDPHandler.getMessage().find(configsLabel) != std::string::npos)
            {
                parseConfigs(YAML::Load(communicatorUDPHandler.getMessage().substr(configsLabel.length()).c_str()));

                // Puts the system on read-write mode
                system("sudo mount -o remount,rw /");

                // Writes the changes to file
                remove(configDir.c_str());
                std::ofstream file;
                file.open(configDir);

                if (!file.is_open())
                    std::cout << "Failed to open configuration file\n";

                file << getCurrentConfig() << '\n';

                file.close();

                // Puts the system back on read-only
                system("sudo mount -o remount,ro /");

                if (systemConfig.verbose.value)
                    std::cout << "Updated Configurations\n";

                if (!systemConfig.tuning.value)
                {
                    streamThread.stop();
                    processVisionThread.stop();

                    while (streamThread.isRunning || processVisionThread.isRunning)
                    {
                        std::cout << "Waiting for streaming and vision processing streams to end...\n";
                        std::this_thread::sleep_for(std::chrono::milliseconds{500});
                    }

                    if (!streamProcessingVideo)
                        streamThread.start();

                    processVisionThread.start();
                }
            }
            else if (communicatorUDPHandler.getMessage() == "get config")
            {
                std::string configTag{"CONFIGS:\n"};

                communicatorUDPHandler.reply(configTag + getCurrentConfig());

                if (systemConfig.verbose.value)
                    std::cout << "Sent Configurations\n";
            }
            else if (communicatorUDPHandler.getMessage() == "switch camera")
            {
                bool newStreamProcessingVideo = !streamProcessingVideo;

                if (!newStreamProcessingVideo)
                    streamThread.start();
                else
                    streamThread.stop();

                streamProcessingVideo = newStreamProcessingVideo;

                if (systemConfig.verbose.value)
                    std::cout << "Switched Camera Stream\n";
            }
            else if (communicatorUDPHandler.getMessage() == "restart program")
            {
                if (systemConfig.verbose.value)
                    std::cout << "Restarting program...\n";

                streamThread.stop();
                processVisionThread.stop();
                break;
            }
            else if (communicatorUDPHandler.getMessage() == "reboot")
            {
                if (systemConfig.verbose.value)
                    std::cout << "Rebooting...\n";

                system("sudo reboot -h now");
            }
            else
            {
                std::cout << "Received unknown command via UDP: " + communicatorUDPHandler.getMessage() + '\n';
            }

            communicatorUDPHandler.clearMessage();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{250});
    }

    return 0;
}
