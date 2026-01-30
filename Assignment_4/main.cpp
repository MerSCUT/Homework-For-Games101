#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
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

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    
    
    std::vector<cv::Point2f> d_points(control_points.size()-1);
    for (int i = 0; i < d_points.size(); i++)
    {
        cv::Point2f vec = control_points[i + 1] - control_points[i];
        
        d_points[i] = control_points[i] + t * vec;
    }
    if (control_points.size() == 2)
    {
        return d_points[0];
    }
    return recursive_bezier(d_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    int de[8][2] = { {1,0}, {0,1}, {-1, 0}, {0, -1}, {1, 1}, {-1,1}, {1, -1}, {-1,-1} };
    for (float t = 0.0f; t <= 1.0f; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        //std::cout << point.x << " " << point.y << std::endl;
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;   // tips : point is float
        // Point will cast into int type.
        // the centre of 4 pixel  
        //std::vector<cv::Point2f> pixel_list;
        //for (int k = 0; k < 8; k++)
        //{
        //    pixel_list.emplace_back((int)point.x + de[k][0] , (int)point.y + de[k][1]);
        //}

        //// Centre
        //for (auto& pixel : pixel_list)
        //{
        //    float dis = std::sqrt((point.x - pixel.x - 0.5) * (point.x - pixel.x - 0.5) +
        //        (point.y - pixel.y - 0.5) * (point.y - pixel.y - 0.5));
        //    // 距离为 0 等于 255, 越远越低
        //    float n = 4.0f;
        //    /*
        //    if (dis >= n)
        //    {
        //        window.at<cv::Vec3b>(pixel.y, pixel.x)[2] += 0;
        //        continue;
        //    }*/
        //    float add = 255.0 * std::pow(1- dis / n, 10);
        //    
        //    auto cur = window.at<cv::Vec3b>(pixel.y, pixel.x)[2];
        //    //std::cout << window.at<cv::Vec3b>(pixel.y, pixel.x)[2] + add << std::endl;
        //    float res = std::min(255.0f, cur + add);
        //    window.at<cv::Vec3b>(pixel.y, pixel.x)[2] = res;
        //}
        
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

        if (control_points.size() >= 4) 
        {
            //naive_bezier(control_points, window);
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
