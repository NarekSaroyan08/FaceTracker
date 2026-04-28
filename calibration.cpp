#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <iostream>
#include <vector>
#include <fstream>

#define SERVO_X 17
#define SERVO_Y 27

struct CalibrationPoint {
    int px, py;
    int sx, sy;
};

std::vector<CalibrationPoint> LUT;

int main()
{
    if (gpioInitialise() < 0) {
        std::cout << "Pigpio error\n";
        return -1;
    }

    cv::VideoCapture cap(2, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    if (!cap.isOpened()) return -1;

    cv::CascadeClassifier face_cascade;
    face_cascade.load("haarcascade_frontalface_default.xml");

    int servoX = 90;
    int servoY = 90;

    std::cout << "\n=== CALIBRATION MODE ===\n";
    std::cout << "A/D = left/right servo\n";
    std::cout << "W/S = up/down servo\n";
    std::cout << "SPACE = save point\n";
    std::cout << "P = save file\n";
    std::cout << "Q = quit\n\n";

    // ==========================
    int stableCount = 0;
    cv::Rect lastFace;

    while (true)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Rect> faces;
        face_cascade.detectMultiScale(gray, faces, 1.2, 3);

        int fx = -1, fy = -1;

        if (!faces.empty())
        {
            cv::Rect f = faces[0];

            // ==========================
            // 🔥 STABILITY CHECK (VERY IMPORTANT)
            if (abs(f.x - lastFace.x) < 10 &&
                abs(f.y - lastFace.y) < 10)
            {
                stableCount++;
            }
            else
            {
                stableCount = 0;
            }

            lastFace = f;

            if (stableCount > 3)
            {
                fx = f.x + f.width / 2;
                fy = f.y + f.height / 2;

                cv::circle(frame, {fx, fy}, 5, {0,255,0}, -1);
            }
        }

        cv::imshow("Calibration PRO", frame);

        gpioServo(SERVO_X, 500 + servoX * 11);
        gpioServo(SERVO_Y, 500 + servoY * 11);

        char key = cv::waitKey(20);

        if (key == 'a') servoX -= 2;
        if (key == 'd') servoX += 2;
        if (key == 'w') servoY -= 2;
        if (key == 's') servoY += 2;

        servoX = std::clamp(servoX, 0, 180);
        servoY = std::clamp(servoY, 0, 180);

        // ==========================
        // SAVE ONLY STABLE POINTS
        if (key == ' ' && fx != -1)
        {
            LUT.push_back({fx, fy, servoX, servoY});

            std::cout << "Saved: "
                      << fx << "," << fy
                      << " -> "
                      << servoX << "," << servoY << std::endl;
        }

        // SAVE FILE
        if (key == 'p')
        {
            std::ofstream file("calibration.txt");

            for (auto &p : LUT)
                file << p.px << " " << p.py << " "
                     << p.sx << " " << p.sy << "\n";

            std::cout << "Saved LUT: " << LUT.size() << " points\n";
        }

        if (key == 'q') break;
    }

    gpioTerminate();
    return 0;
}
