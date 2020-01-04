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
        command << "cd ../mjpg-streamer-master/mjpg-streamer-experimental/ && ./mjpg_streamer -i 'input_uvc.so -d /dev/video" << device << " -r "
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
        pipeline << "rpicamsrc sensor-mode=" << raspicamConfig.sensorMode.value << " shutter-speed=" << raspicamConfig.shutterSpeed.value << " exposure-mode=" << raspicamConfig.exposureMode.value
                 << " awb-mode=" << raspicamConfig.whiteBalanceMode.value
                 << " sharpness=" << raspicamConfig.sharpness.value << " contrast=" << raspicamConfig.sharpness.value
                 << " brightness=" << raspicamConfig.brightness.value << " saturation=" << raspicamConfig.saturation.value
                 << " ! video/x-raw,width=" << raspicamConfig.width.value << ",height=" << raspicamConfig.height.value
                 << ",framerate=" << raspicamConfig.fps.value << "/1 ! appsink";

        cv::VideoCapture processingCamera{pipeline.str(), cv::CAP_GSTREAMER};
        MJPEGWriter mjpegWriter{systemConfig.videoPort.value};

        if (systemConfig.verbose.value && !processingCamera.isOpened())
            std::cout << "Could not open processing camera!\n";

        long int begin = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        int lastFpsFrame = 1;

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

            /*
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() - begin >= 1)
            {
                std::cout << "FPS: " << frameNumber - lastFpsFrame << '\n';
                begin = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                lastFpsFrame = frameNumber;
            }
            */

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
            if (streamProcessingVideo && !systemConfig.tuning.value && frameNumber % 5 == 0)
            {
                mjpegWriter.write(processingFrame);
            }

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
                    cv::resize(streamFrame, streamFrame, cv::Size{}, 0.5, 0.5);
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
                // This sends the message every fifth frame. Sending status messages too fast generates some latency
                if (frameNumber % 5 == 0)
                    communicatorUDPHandler->reply("VISION-SEARCHING");
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

            double verticalFoV{45.0};
            double heightOfTargetInches{15.0};

            Point2f points[4];
            target.rotatedBoundingBox.points(points);

            // bottom-left minus top-left
            double height{points[0].y - points[1].y};

            // Total height of our view on the plane of the vision target is (total height in pixels / target height in pixels) * height of target in inches
            double viewHeight{(processingFrame.rows / height) * heightOfTargetInches};

            // Distance from target = View height / (2 * tan(vertical FoV / 2))
            double distance{viewHeight / (2 * std::tan(verticalFoV / 2))};

            double horizontalOffset{(target.center.x - (processingFrame.cols / 2.0)) / processingFrame.cols};
            double verticalOffset{(target.center.y - (processingFrame.rows / 2.0)) / processingFrame.rows};

            robotUDPHandler.sendTo("X OFFSET:" + std::to_string(horizontalOffset), robotEndpoint);
            robotUDPHandler.sendTo("Y OFFSET:" + std::to_string(verticalOffset), robotEndpoint);

            // We can't read something father than full field
            if (distance < 648)
                robotUDPHandler.sendTo("DISTANCE:" + std::to_string(distance), robotEndpoint);

            // This sends the message every fifth frame. Sending status messages too fast generates some latency
            if (frameNumber % 5 == 0)
                communicatorUDPHandler->reply("VISION-LOCKED");

            // Preps frame to be streamed
            if (streamProcessingVideo && systemConfig.tuning.value)
            {
                cv::rectangle(streamFrame, target.boundingBox, cv::Scalar{0, 255, 0}, 2);
                cv::line(streamFrame, cv::Point{target.center.x, target.center.y - 10}, cv::Point{target.center.x, target.center.y + 10}, cv::Scalar{0, 255, 0}, 2);
                cv::line(streamFrame, cv::Point{target.center.x - 10, target.center.y}, cv::Point{target.center.x + 10, target.center.y}, cv::Scalar{0, 255, 0}, 2);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds{10});
        }

        communicatorUDPHandler->reply("VISION-DOWN");

        mjpegWriter.stop();
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
