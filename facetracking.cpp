#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>
#include <cmath>
#include <fstream>

#define SERVO_X 17
#define SERVO_Y 27

#define MOTOR_IN1 13
#define MOTOR_IN2 5

// ==========================
struct CalibrationPoint {
    int px, py;
    int sx, sy;
};

std::vector<CalibrationPoint> LUT;

// ==========================
std::atomic<bool> faceDetected(false);
std::atomic<bool> running(true);

// ==========================
// PID STRUCT
struct PID {
    double kp, ki, kd;
    double prevError = 0;
    double integral = 0;

    double update(double error)
    {
        integral += error;
        double derivative = error - prevError;
        prevError = error;

        return kp * error + ki * integral + kd * derivative;
    }
};

// ==========================
PID pidX{0.12, 0.0001, 0.08};
PID pidY{0.12, 0.0001, 0.08};

// ==========================
int servoX = 90;
int servoY = 90;

// ==========================
void loadLUT()
{
    std::ifstream file("calibration.txt");
    CalibrationPoint p;

    while (file >> p.px >> p.py >> p.sx >> p.sy)
        LUT.push_back(p);

    std::cout << "Loaded LUT: " << LUT.size() << std::endl;
}

// ==========================
// KNN LUT (stable mapping)
void getServoFromLUT(int fx, int fy, int &sx, int &sy)
{
    std::vector<std::pair<double, CalibrationPoint>> dist;

    for (auto &p : LUT)
    {
        double dx = p.px - fx;
        double dy = p.py - fy;
        double d = sqrt(dx*dx + dy*dy);
        dist.push_back({d, p});
    }

    std::sort(dist.begin(), dist.end(),
              [](auto &a, auto &b){ return a.first < b.first; });

    int k = std::min(5, (int)dist.size());

    double sumW = 0, sumX = 0, sumY = 0;

    for (int i = 0; i < k; i++)
    {
        double d = dist[i].first + 1.0;
        double w = 1.0 / (d * d);

        sumX += dist[i].second.sx * w;
        sumY += dist[i].second.sy * w;
        sumW += w;
    }

    sx = int(sumX / sumW);
    sy = int(sumY / sumW);
}

// ==========================
int main()
{
    if (gpioInitialise() < 0) return -1;

    loadLUT();

    cv::VideoCapture cap(0, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    if (!cap.isOpened()) return -1;

    cv::CascadeClassifier face_cascade;
    face_cascade.load("haarcascade_frontalface_default.xml");

    // ==========================
    std::thread visionThread([&]() {

        int lastFx = 320, lastFy = 240;

        while (running)
        {
            cv::Mat frame;
            if (!cap.read(frame)) continue;

            cv::Mat gray, small;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::resize(gray, small, cv::Size(), 0.5, 0.5);

            std::vector<cv::Rect> faces;
            face_cascade.detectMultiScale(small, faces, 1.3, 2);

            if (!faces.empty())
            {
                faceDetected = true;

                cv::Rect f = faces[0];

                int fx = (f.x + f.width / 2) * 2;
                int fy = (f.y + f.height / 2) * 2;

                // ==========================
                // 🔥 MOTION PREDICTION (reduces lag)
                int predFx = fx + (fx - lastFx) * 0.5;
                int predFy = fy + (fy - lastFy) * 0.5;

                lastFx = fx;
                lastFy = fy;

                int baseX, baseY;
                getServoFromLUT(predFx, predFy, baseX, baseY);

                // store targets (not direct control)
                servoX = baseX;
                servoY = baseY;

                int centerY = 240;
                int errorY = fy - centerY;

                // motor control
                if (abs(errorY) < 30)
                {
                    gpioWrite(MOTOR_IN1, 0);
                    gpioWrite(MOTOR_IN2, 0);
                }
                else if (errorY < 0)
                {
                    gpioWrite(MOTOR_IN1, 0);
                    gpioWrite(MOTOR_IN2, 1);
                }
                else
                {
                    gpioWrite(MOTOR_IN1, 1);
                    gpioWrite(MOTOR_IN2, 0);
                }
            }
            else
            {
                faceDetected = false;

                gpioWrite(MOTOR_IN1, 0);
                gpioWrite(MOTOR_IN2, 0);

                // auto return center
                servoX = 90;
                servoY = 90;
            }

            cv::imshow("PRO Tracker", frame);
            if (cv::waitKey(1) == 'q') running = false;
        }
    });

    // ==========================
    std::thread servoThread([&]() {

        while (running)
        {
            static double outX = 90;
            static double outY = 90;

            // ==========================
            // 🔥 PID CONTROL
            double errX = servoX - outX;
            double errY = servoY - outY;

            outX += pidX.update(errX);
            outY += pidY.update(errY);

            // clamp
            outX = std::clamp(outX, 0.0, 180.0);
            outY = std::clamp(outY, 0.0, 180.0);

            gpioServo(SERVO_X, 500 + (int)outX * 11);
            gpioServo(SERVO_Y, 500 + (int)outY * 11);

            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    });

    visionThread.join();
    servoThread.join();

    gpioTerminate();
    return 0;
}
