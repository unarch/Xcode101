//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        return getColorBilinear(u, v);
        u = std::max(u, 0.f);
        u = std::min(u, 1.f);
        v = std::max(v, 0.f);
        v = std::min(v, 1.f);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    
    Eigen::Vector3f getColorBilinear(float u, float v) {
        u = std::max(u, 0.f);
        u = std::min(u, 1.f);
        v = std::max(v, 0.f);
        v = std::min(v, 1.f);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        float u_min = std::floor(u_img), u_max = std::min((float)width, std::ceil(u_img));
        float v_min = std::floor(v_img), v_max = std::min((float)height, std::ceil(v_img));
        auto color11 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto color12 = image_data.at<cv::Vec3b>(v_min, u_max);
        auto color21 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto color22 = image_data.at<cv::Vec3b>(v_max, u_max);
        
        float u_rate = (u_img - u_min) / (u_max - u_min);
        float v_rate = (v_img - v_min) / (v_max - v_min);
        // 越大就越靠近大的那边
        auto mix1 = u_rate * color12 + (1 - u_rate) * color11;
        auto mix2 = u_rate * color22 + (1 - u_rate) * color21;
        auto color = v_rate * mix1 + (1 - v_rate) * mix2;
        
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    

};
#endif //RASTERIZER_TEXTURE_H
