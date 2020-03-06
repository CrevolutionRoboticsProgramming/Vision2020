#include <iostream>
#include <string>
#include <fstream>
#include <functional>
#include <chrono>
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

UDPHandler *communicatorUDPHandler;

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

        // The Pi recognizes both cameras correctly more often when this command is issued
        system("ls /dev/video*");

        FILE *uname;
        char consoleOutput[300];
        int lastchar;

        // Executes the command supplied to popen and saves the output in the char array
        uname = popen("v4l2-ctl --list-devices", "r");
        lastchar = fread(consoleOutput, 1, 300, uname);
        consoleOutput[lastchar] = '\0';

        // Converts the char array to an std::string
        std::string outputString = consoleOutput;

        /**
	    * Working from the inside out:
	    * 	- Finds where the camera's name is in the string
	    * 	- Uses the location of the first character in that string as the starting point for a new search for the location of the first character in /dev/video
	    * 	- Looks ten characters down the string to find the number that comes after /dev/video
	    * 	- Parses the output character for an integer
	    * 	- Assigns that integer to the appropriate variable
	    */
        int device = outputString.at(outputString.find("/dev/video", outputString.find("USB 2.0 Camera: HD USB Camera")) + 10) - '0';

        pclose(uname);

        command = std::ostringstream{};
        command << "export LD_LIBRARY_PATH=/home/pi/mjpg-streamer-master/mjpg-streamer-experimental/plugins"
                << " && cd /home/pi/mjpg-streamer-master/mjpg-streamer-experimental/ && ./mjpg_streamer -i '/home/pi/mjpg-streamer-master/mjpg-streamer-experimental/plugins/input_uvc/input_uvc.so -d /dev/video" << device << " -r "
                << uvccamConfig.width.value << "x" << uvccamConfig.height.value << " -e " << uvccamConfig.dropEveryNthFrame.value
                << "' -o '/home/pi/mjpg-streamer-master/mjpg-streamer-experimental/plugins/output_http/output_http.so -p " << systemConfig.videoPort.value << "'";
        system(command.str().c_str());
    }
} streamThread;

// Vision Processing
class : public Thread
{
private:
    cv::Mat mMorphElement{cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3))};

    UDPHandler mRobotUDPHandler{9999};
    boost::asio::ip::udp::endpoint mRobotEndpoint{boost::asio::ip::address::from_string("10.28.51.2"), static_cast<short unsigned int>(systemConfig.robotPort.value)};

    std::vector<std::thread> mThreads{};

    MJPEGWriter mMjpegWriter;

    void process(int frameNumber, cv::Mat processingFrame)
    {
        cv::Mat streamFrame;

        // Writes frame to be streamed when not tuning
        if (streamProcessingVideo && !systemConfig.tuning.value)
            mMjpegWriter.write(processingFrame);

        // Extracts the contours
        std::vector<std::vector<cv::Point>> rawContours;
        std::vector<Contour> contours;
        cv::cvtColor(processingFrame, processingFrame, cv::COLOR_BGR2HSV);
        cv::inRange(processingFrame, cv::Scalar{visionConfig.lowHue.value, visionConfig.lowSaturation.value, visionConfig.lowValue.value},
                    cv::Scalar{visionConfig.highHue.value, visionConfig.highSaturation.value, visionConfig.highValue.value}, processingFrame);
        cv::dilate(processingFrame, processingFrame, mMorphElement, cv::Point(-1, -1), visionConfig.dilationPasses.value);

        if (streamProcessingVideo)
        {
            if (systemConfig.tuning.value)
                processingFrame.copyTo(streamFrame);

            if (!mMjpegWriter.isOpened())
            {
                mMjpegWriter.write(processingFrame);
                mMjpegWriter.start();
            }
        }
        else if (mMjpegWriter.isOpened())
            mMjpegWriter.stop();

        //cv::Canny(processingFrame, processingFrame, 0, 0);

        // The program won't be able to properly identify the contours unless we make them a little bigger
        //cv::dilate(processingFrame, processingFrame, mMorphElement, cv::Point(-1, -1), 1);
        cv::findContours(processingFrame, rawContours, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (std::vector<cv::Point> pointsVector : rawContours)
        {
            Contour newContour{pointsVector, visionConfig.allowableError.value};

            if (newContour.isValid(visionConfig.minArea.value, visionConfig.maxArea.value, visionConfig.minContourToBoundingBoxRatio.value, visionConfig.maxContourToBoundingBoxRatio.value))
                contours.push_back(newContour);
        }

        Contour target{};

        switch (contours.size())
        {
        case 0:
            // This sends the message every fifth frame. Sending status messages too fast generates some latency
            if (frameNumber % 5 == 0)
                communicatorUDPHandler->reply("VISION-SEARCHING");

            if (streamProcessingVideo && systemConfig.tuning.value && !streamFrame.empty())
            {
                //cv::resize(streamFrame, streamFrame, cv::Size{}, 0.5, 0.5);
                mMjpegWriter.write(streamFrame);
            }
            return;
        case 1:
            target = contours.at(0);
            break;
        default:
            target = contours.at(0);
            double leastDistance{std::numeric_limits<double>::max()};

            for (Contour contour : contours)
            {
                double distanceToCenter{std::sqrt(std::pow(contour.rotatedBoundingBox.center.x - (processingFrame.cols / 2), 2) + std::pow(contour.center.y - (processingFrame.rows / 2), 2))};

                if (distanceToCenter < leastDistance)
                {
                    target = contour;
                    leastDistance = distanceToCenter;
                }
            }
        }

        double horizontalOffset{(target.center.x - (processingFrame.cols / 2.0)) / processingFrame.cols};
        double verticalOffset{(target.center.y - (processingFrame.rows / 2.0)) / processingFrame.rows};

        mRobotUDPHandler.sendTo("X OFFSET:" + std::to_string(horizontalOffset), mRobotEndpoint);
        mRobotUDPHandler.sendTo("Y OFFSET:" + std::to_string(-verticalOffset), mRobotEndpoint);

        // This sends the message every fifth frame. Sending status messages too fast generates some latency
        if (frameNumber % 5 == 0)
            communicatorUDPHandler->reply("VISION-LOCKED");

        // Preps frame to be streamed
        if (streamProcessingVideo && systemConfig.tuning.value && !streamFrame.empty())
        {
            cv::cvtColor(streamFrame, streamFrame, cv::COLOR_GRAY2BGR);
            cv::rectangle(streamFrame, target.boundingBox, cv::Scalar{0, 255, 0}, 2);
            cv::line(streamFrame, cv::Point{static_cast<int>(target.center.x), static_cast<int>(target.center.y - 10)},
                     cv::Point{static_cast<int>(target.center.x), static_cast<int>(target.center.y + 10)}, cv::Scalar{0, 255, 0}, 2);
            cv::line(streamFrame, cv::Point{static_cast<int>(target.center.x - 10), static_cast<int>(target.center.y)},
                     cv::Point{static_cast<int>(target.center.x + 10), static_cast<int>(target.center.y)}, cv::Scalar{0, 255, 0}, 2);
            cv::resize(streamFrame, streamFrame, cv::Size{}, 0.5, 0.5);

            mMjpegWriter.write(streamFrame);
        }
    }

    void run() override
    {
        std::ostringstream pipeline;
        pipeline << "rpicamsrc sensor-mode=" << raspicamConfig.sensorMode.value << " shutter-speed=" << raspicamConfig.shutterSpeed.value
                 << " exposure-mode=" << raspicamConfig.exposureMode.value << " awb-mode=" << raspicamConfig.whiteBalanceMode.value
                 << " sharpness=" << raspicamConfig.sharpness.value << " contrast=" << raspicamConfig.sharpness.value
                 << " brightness=" << raspicamConfig.brightness.value << " saturation=" << raspicamConfig.saturation.value
                 << " ! video/x-raw,width=" << raspicamConfig.width.value << ",height=" << raspicamConfig.height.value
                 << ",framerate=" << raspicamConfig.maxFps.value << "/1 ! appsink";

        cv::VideoCapture processingCamera{pipeline.str(), cv::CAP_GSTREAMER};

	if (!processingCamera.isOpened())
	{
            std::cout << "Could not open processing camera!\n";
            return;
	}

        mMjpegWriter = MJPEGWriter{systemConfig.videoPort.value};

        if (systemConfig.verbose.value && !processingCamera.isOpened())
            std::cout << "Could not open processing camera!\n";

        long int lastFpsPrintSeconds{std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()};
        int lastFpsFrame{1};

        for (int frameNumber{1}; !stopFlag; ++frameNumber)
        {
            cv::Mat processingFrame;

            if (processingCamera.grab())
                processingCamera.read(processingFrame);
            else
                continue;

            if (processingFrame.empty())
                continue;

            if (systemConfig.verbose.value && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - lastFpsPrintSeconds >= 1)
            {
                std::cout << "FPS: " << frameNumber - lastFpsFrame << '\n';
                lastFpsPrintSeconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                lastFpsFrame = frameNumber;
            }

            mThreads.push_back(std::thread{[=] { process(frameNumber, processingFrame); }});

            while (mThreads.size() > 180)
            {
                mThreads.at(0).join();
                mThreads.erase(mThreads.begin());
            }
        }

        communicatorUDPHandler->reply("VISION-DOWN");

        mMjpegWriter.stop();
    }
} processVisionThread;

int main()
{
    parseConfigs(YAML::LoadFile(configDir));

    UDPHandler tmp{systemConfig.receivePort.value};

    communicatorUDPHandler = &tmp;

    streamThread.start();
    processVisionThread.start();

    class : public UDPHandler::MessageReceiver
    {
    public:
        void run(std::string message) override
        {
            parseConfigs(YAML::Load(message.c_str()));

            // Puts the system on read-write mode
            system("sudo mount -o remount,rw /");

            // Writes the changes to file
            remove(configDir.c_str());
            std::ofstream file;
            file.open(configDir);

            if (!file.is_open())
            {
                std::cout << "Failed to open configuration file\n";
                return;
            }

            file << getCurrentConfig() << '\n';

            file.close();

            // Puts the system back on read-only
            system("sudo mount -o remount,ro /");

            if (systemConfig.verbose.value)
                std::cout << "Updated configurations\n";

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

        std::string getLabel() override
        {
            return "CONFIGS:";
        }
    } configsReceiver;

    class : public UDPHandler::MessageReceiver
    {
    public:
        void run(std::string message) override
        {
            communicatorUDPHandler->reply("CONFIGS:" + getCurrentConfig());

            if (systemConfig.verbose.value)
                std::cout << "Sent Configurations\n";
        }

        std::string getLabel() override
        {
            return "get config";
        }
    } configsSender;

    class : public UDPHandler::MessageReceiver
    {
    public:
        void run(std::string message) override
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

        std::string getLabel() override
        {
            return "switch camera";
        }
    } cameraSwitcher;

    class : public UDPHandler::MessageReceiver
    {
    public:
        void run(std::string message) override
        {
            if (systemConfig.verbose.value)
                std::cout << "Restarting program...\n";

            streamThread.stop();
            processVisionThread.stop();

            system("sudo pkill Vision2020");
        }

        std::string getLabel() override
        {
            return "restart program";
        }
    } programRestarter;

    class : public UDPHandler::MessageReceiver
    {
    public:
        void run(std::string message) override
        {
            if (systemConfig.verbose.value)
                std::cout << "Rebooting...\n";

            system("sudo reboot -h now");
        }

        std::string getLabel() override
        {
            return "reboot";
        }
    } systemRebooter;

    communicatorUDPHandler->addReceiver(&configsReceiver);
    communicatorUDPHandler->addReceiver(&configsSender);
    communicatorUDPHandler->addReceiver(&cameraSwitcher);
    communicatorUDPHandler->addReceiver(&programRestarter);
    communicatorUDPHandler->addReceiver(&systemRebooter);

    while (true)
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

    return 0;
}
