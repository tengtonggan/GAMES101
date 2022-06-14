//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/core>
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
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColor(Eigen::Vector2f uv)
    {
        auto u_img = uv[0] * width;
        auto v_img = (1 - uv[1]) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int u_min = static_cast<int>(u_img - 0.5);
        int v_min = static_cast<int>(v_img - 0.5);
        /*if (u_min + 0.5 < 0 || u_min + 0.5 > width || v_min + 0.5 < 0 || v_min + 0.5 > height)
            return getColor(uv);
        if (u_min + 1.5 < 0 || u_min + 1.5 > width || v_min + 1.5 < 0 || v_min + 1.5 > height)
            return getColor(uv);*/
        auto u00 = image_data.at<cv::Vec3b>(v_min + 0.5, u_min + 0.5);
        auto u10 = image_data.at<cv::Vec3b>(v_min + 1.5, u_min + 0.5);
        auto u01 = image_data.at<cv::Vec3b>(v_min + 0.5, u_min + 1.5);
        auto u11 = image_data.at<cv::Vec3b>(v_min + 1.5, u_min + 1.5);
        float s = (u_img - (u_min + 0.5));
        float t = (v_img - (v_min + 0.5));
        auto u_lerp0 = (1.f - s) * u00 + s * u10;
        auto u_lerp1 = (1.f - s) * u01 + s * u11;
        auto color = (1.f - t) * u_lerp0 + t * u_lerp1;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(Eigen::Vector2f uv)
    {
        auto u_img = uv[0] * width;
        auto v_img = (1 - uv[1]) * height;
        int u_min = static_cast<int>(u_img - 0.5);
        int v_min = static_cast<int>(v_img - 0.5);
        /*if (u_min + 0.5 < 0 || u_min + 0.5 > width || v_min + 0.5 < 0 || v_min + 0.5 > height)
            return getColor(uv);
        if (u_min + 1.5 < 0 || u_min + 1.5 > width || v_min + 1.5 < 0 || v_min + 1.5 > height)
            return getColor(uv);*/
        auto u00 = image_data.at<cv::Vec3b>(v_min + 0.5, u_min + 0.5);
        auto u10 = image_data.at<cv::Vec3b>(v_min + 1.5, u_min + 0.5);
        auto u01 = image_data.at<cv::Vec3b>(v_min + 0.5, u_min + 1.5);
        auto u11 = image_data.at<cv::Vec3b>(v_min + 1.5, u_min + 1.5);
        float s = (u_img - (u_min + 0.5));
        float t = (v_img - (v_min + 0.5));
        auto u_lerp0 = (1.f - s) * u00 + s * u10;
        auto u_lerp1 = (1.f - s) * u01 + s * u11;
        auto color = (1.f - t) * u_lerp0 + t * u_lerp1;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
