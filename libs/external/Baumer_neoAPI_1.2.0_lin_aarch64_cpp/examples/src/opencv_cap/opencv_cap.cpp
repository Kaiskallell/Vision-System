/** \example opencv.cpp
    A simple Program for grabbing video from Baumer camera and converting it to opencv images.
    Copyright (c) by Baumer Optronic GmbH. All rights reserved, please see the provided license for full details.
*/

#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include "neoapi/neoapi.hpp"

int main() {
    int result = 0;
    try {
        NeoAPI::Cam camera = NeoAPI::Cam();
        camera.Connect();
        camera.f().ExposureTime.Set(10000);

        int type= CV_8U;
        bool isColor = true;
        if (camera.f().PixelFormat.GetEnumValueList().IsReadable("BGR8")) {
            camera.f().PixelFormat.SetString("BGR8");
            type = CV_8UC3;
            isColor = true;
        } else if (camera.f().PixelFormat.GetEnumValueList().IsReadable("Mono8")) {
            camera.f().PixelFormat.SetString("Mono8");
            type = CV_8UC1;
            isColor = false;
        } else {
            std::cout << "no supported pixel format";
            return 0;  // Camera does not support pixelformat
        }
        int width = static_cast<int>(camera.f().Width);
        int height = static_cast<int>(camera.f().Height);

        // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
        // Define the fps to be equal to 10. Also frame size is passed.
        // cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
        // cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 10,
        cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 10,
            cv::Size(static_cast<int>(camera.f().Width), static_cast<int>(camera.f().Height)), isColor);

        const cv::String windowName = "Press [Esc] to quit.";
        for (int count = 0; count < 200; ++count) {
            NeoAPI::Image image = camera.GetImage();
            cv::Mat img(cv::Size(width, height), type, image.GetImageData(), cv::Mat::AUTO_STEP);
            cv::namedWindow(windowName);
            cv::imshow(windowName, img);
            video.write(img);

            if (cv::waitKey(1) == 27) {
                break;
            }
        }
        video.release();
        cv::destroyWindow(windowName);
    }
    catch (NeoAPI::NeoException& exc) {
        std::cout << "error: " << exc.GetDescription() << std::endl;
        result = 1;
    }
    catch (...) {
        std::cout << "oops, error" << std::endl;
        result = 1;
    }

    return result;
}
