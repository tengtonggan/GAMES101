#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

inline float to_radiance(float degree) {
    return degree / 180.0 * MY_PI;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    //Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f model;
    model<<cos(rotation_angle / 180.0 * MY_PI), -sin(rotation_angle / 180.0 * MY_PI), 0, 0,
        sin(rotation_angle / 180.0 * MY_PI), cos(rotation_angle / 180.0 * MY_PI), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

//any axis which crosses origin point
Eigen::Matrix4f get_model_matrix(Eigen::Vector3f n, float rotation_angle)
{
    float angle_radiance = to_radiance(rotation_angle);
    n.normalize();
    Eigen::Matrix3f model3f = cos(angle_radiance) * Eigen::Matrix3f::Identity();
    Eigen::Matrix3f temp3f;
    temp3f << 0, -n.z(), n.y(),
        n.z(), 0, -n.x(),
        -n.y(), n.x(), 0;
    model3f += sin(angle_radiance) * temp3f;

    temp3f = n * n.transpose();
    model3f += (1 - cos(angle_radiance)) * temp3f;

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    model.topLeftCorner<3, 3>() = model3f;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    //Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projection;

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    auto sign_n = -zNear;
    auto sign_f = -zFar;
    //top=znear*tan(\theta/2)
    auto t = zNear * tan(eye_fov / 2 / 180.0 * MY_PI);
    auto b = -t;
    auto r = t * aspect_ratio;
    auto l = -r;
    projection << 2 * sign_n / (r - l), 0, (l + r) / (l - r), 0,
        0, 2 * sign_n / (t - b), (b + t) / (b - t), 0,
        0, 0, (sign_f + sign_n) / (sign_n - sign_f), 2 * sign_f * sign_n / (sign_f - sign_n),
        0, 0, 1, 0;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    bool rotation_default = true;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 20};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        if (key == 'r')
            rotation_default = false;

        if(rotation_default)
            r.set_model(get_model_matrix(angle));
        else {
            Eigen::Vector3f axis = { 0.f,1.f,1.f };
            r.set_model(get_model_matrix(axis, angle));
        }
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
