// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)//int int origin
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f p = Eigen::Vector3f(x, y, 0);
    Eigen::Vector3f v[3] = { Eigen::Vector3f(_v[0].x(), _v[0].y(), 0),
        Eigen::Vector3f(_v[1].x(), _v[1].y(), 0),Eigen::Vector3f(_v[2].x(), _v[2].y(), 0) };
    for (size_t i = 0; i < 2; i++) {
        Eigen::Vector3f vp = p - v[i];
        Eigen::Vector3f vv = v[(i + 1) % 3] - v[i];
        auto vs1=vp.cross(vv);
        vp = p - v[i + 1];
        vv = v[(i + 2) % 3] - v[i + 1];
        auto vs2 = vp.cross(vv);
        if (vs1.dot(vs2) < 0)
            return false;
    }
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = -vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t, rst::Antialiasing::MSAA);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, Antialiasing anti) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    auto box_x1 = static_cast<int>(t.v[0].x());
    auto box_y1 = static_cast<int>(t.v[0].y());
    auto box_x2 = box_x1;
    auto box_y2 = box_y1;
    for (size_t i = 1; i < 3; i++) {
        box_x1 = std::min(box_x1, static_cast<int>(t.v[i].x()));
        box_y1 = std::min(box_y1, static_cast<int>(t.v[i].y()));
        box_x2 = std::max(box_x2, static_cast<int>(t.v[i].x()));
        box_y2 = std::max(box_y2, static_cast<int>(t.v[i].y()));
    }

    if (anti == rst::Antialiasing::None) {
        for (size_t x = box_x1; x <= box_x2; x++) {
            for (size_t y = box_y1; y <= box_y2; y++) {
                if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= -w_reciprocal;
                    float& z_buffer = depth_buf[(height - 1 - y) * width + x];
                    if (z_interpolated < depth_buf[(height - 1 - y) * width + x]) {
                        z_buffer = z_interpolated;
                        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                        set_pixel(point, t.getColor());
                    }

                }
            }
        }
    }
    else if (anti == rst::Antialiasing::MSAA) {
        for (size_t x = box_x1; x <= box_x2; x++) {
            for (size_t y = box_y1; y <= box_y2; y++) {
                //super_sampling antialiasing 2*2
                size_t sn = 2;
                float half_gap = 0.5 / sn;
                bool inside = false;
                int sum = 0;
                for (size_t i = 0; i < sn; i++) {
                    for (size_t j = 0; j < sn; j++) {
                        //x + 1.0 / sn / 2 + i * 1.0 / sn
                        if (insideTriangle(x + (2 * i + 1) * half_gap, y + (2 * j + 1) * half_gap, t.v)) {
                            inside = true;
                            sum++;
                        }
                    }
                }
                if (inside) {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    float& z_buffer = depth_buf[(height - 1 - y) * width + x];
                    if (z_interpolated < depth_buf[(height - 1 - y) * width + x]) {
                        z_buffer = z_interpolated;
                        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
                        float cof = static_cast<float>(sum) / static_cast<float>(sn * sn);
                        //cof = 0.2;
                        set_pixel(point, cof * t.getColor());
                    }
                }
            }
        }
        
    }


            

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
    auto ind = (height-1-point.y())*width + point.x();
    //auto ind = get_index(point.x(), point.y());
    frame_buf[ind] = color;

}

// clang-format on