#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

int point_count = 4;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < point_count)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    
    if (control_points.size() == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> next_points;
    
    for(auto i = 0; i < control_points.size() - 1; i++) {
        auto &p_0 = control_points[i];
        auto &p_1 = control_points[i + 1];
        next_points.emplace_back(p_0 + t * (p_1 - p_0));
    }
    return recursive_bezier(next_points, t);
    
    
    
    return cv::Point2f();

}

void bezier_plus(const cv::Point2f &point, cv::Mat &window){
    double circle = 1;
    double maxDistance = sqrt(circle * circle + circle * circle);
    
    for (double i = -circle; i <= circle; i+= 0.1) {
        for(double j = -circle; j <= circle; j+= 0.1) {
            auto newX = point.x + i;
            auto newY = point.y + j;
            auto distance = sqrt(i * i + j * j);
            if (distance >= maxDistance) {
                continue;
            }
            auto color = 255 * (1.0 - (distance / maxDistance));
            window.at<cv::Vec3b>(newY, newX)[1] = color;
        }
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        bezier_plus(point, window);
    }
}




int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == point_count)
        {
            naive_bezier(control_points, window);
               bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
