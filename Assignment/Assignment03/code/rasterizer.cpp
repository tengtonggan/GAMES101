//
// Created by goksu on 4/6/19.
//

#include <algorithm>
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

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//static bool insideTriangle(int x, int y, const Vector4f* _v){
//    Vector3f v[3];
//    for(int i=0;i<3;i++)
//        v[i] = {_v[i].x(),_v[i].y(), 1.0};
//    Vector3f f0,f1,f2;
//    f0 = v[1].cross(v[0]);
//    f1 = v[2].cross(v[1]);
//    f2 = v[0].cross(v[2]);
//    Vector3f p(x,y,1.);
//    if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
//        return true;
//    return false;
//}

static bool insideTriangle(float x, float y, const Vector4f* _v)//int int origin
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f p = Eigen::Vector3f(x, y, 0);
    Eigen::Vector3f v[3] = { Eigen::Vector3f(_v[0].x(), _v[0].y(), 0),
        Eigen::Vector3f(_v[1].x(), _v[1].y(), 0),Eigen::Vector3f(_v[2].x(), _v[2].y(), 0) };
    for (size_t i = 0; i < 2; i++) {
        Eigen::Vector3f vp = p - v[i];
        Eigen::Vector3f vv = v[(i + 1) % 3] - v[i];
        auto vs1 = vp.cross(vv);
        vp = p - v[i + 1];
        vv = v[(i + 2) % 3] - v[i + 1];
        auto vs2 = vp.cross(vv);
        if (vs1.dot(vs2) < 0)
            return false;
    }
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;
        /*input triangle in world space-->output triangle(newtri) in screen to draw*/
        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }
        //viewspace_normal
        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = -vert.z() * f1 + f2;//depth=50(far) when z=-1, depth=0.1(near) when z=1, 
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
    // TODO: From your HW3, get the triangle rasterization code.
    //auto v = t.toVector4();
    
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

    rst::Antialiasing anti = rst::Antialiasing::None;

    if (anti == rst::Antialiasing::None) {
        for (size_t x = box_x1; x <= box_x2; x++) {
            for (size_t y = box_y1; y <= box_y2; y++) {
                if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    float w_reciprocal = 1.0 / (alpha / t.v[0].w() + beta / t.v[1].w() + gamma / t.v[2].w());
                    //normal
                    Eigen::Vector3f interpolated_normal = alpha * t.normal[0] / t.v[0].w() + beta * t.normal[1] / t.v[1].w() + gamma * t.normal[2] / t.v[2].w();
                    interpolated_normal *= w_reciprocal;
                    interpolated_normal.normalize();
                    //shadingcoords
                    Eigen::Vector3f interpolated_shadingcoords= alpha * view_pos[0] / t.v[0].w() + beta * view_pos[1] / t.v[1].w() + gamma * view_pos[2] / t.v[2].w();
                    interpolated_shadingcoords *= w_reciprocal;
                    //texcoords
                    Eigen::Vector2f interpolated_texcoords= alpha * t.tex_coords[0] / t.v[0].w() + beta * t.tex_coords[1] / t.v[1].w() + gamma * t.tex_coords[2] / t.v[2].w();
                    interpolated_texcoords *= w_reciprocal;
                    //color-->kd
                    Eigen::Vector3f interpolated_color = alpha * t.color[0] / t.v[0].w() + beta * t.color[1] / t.v[1].w() + gamma * t.color[2] / t.v[2].w();
                    interpolated_color *= w_reciprocal;

                    float z_interpolated = alpha * t.v[0].z() / t.v[0].w() + beta * t.v[1].z() / t.v[1].w() + gamma * t.v[2].z() / t.v[2].w();
                    z_interpolated *= w_reciprocal;
                    float& z_buffer = depth_buf[(height - 1 - y) * width + x];
                    if (z_interpolated < z_buffer) {
                        z_buffer = z_interpolated;
                        Eigen::Vector2i point = Eigen::Vector2i(x, y);
                        fragment_shader_payload payload(interpolated_color, interpolated_normal, interpolated_texcoords, texture ? &*texture : nullptr);
                        payload.view_pos = interpolated_shadingcoords;
                        // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
                        auto pixel_color = fragment_shader(payload);
                        set_pixel(point, pixel_color);
                    }

                }
            }
        }
    }
    //else if (anti == rst::Antialiasing::MSAA) {
    //    for (size_t x = box_x1; x <= box_x2; x++) {
    //        for (size_t y = box_y1; y <= box_y2; y++) {
    //            //super_sampling antialiasing 2*2
    //            size_t sn = 2;
    //            float half_gap = 0.5 / sn;
    //            bool inside = false;
    //            int sum = 0;
    //            for (size_t i = 0; i < sn; i++) {
    //                for (size_t j = 0; j < sn; j++) {
    //                    //x + 1.0 / sn / 2 + i * 1.0 / sn
    //                    if (insideTriangle(x + (2 * i + 1) * half_gap, y + (2 * j + 1) * half_gap, t.v)) {
    //                        inside = true;
    //                        sum++;
    //                    }
    //                }
    //            }
    //            if (inside) {
    //                auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
    //                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //                z_interpolated *= w_reciprocal;
    //                float& z_buffer = depth_buf[(height - 1 - y) * width + x];
    //                if (z_interpolated < depth_buf[(height - 1 - y) * width + x]) {
    //                    z_buffer = z_interpolated;
    //                    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
    //                    float cof = static_cast<float>(sum) / static_cast<float>(sn * sn);
    //                    //cof = 0.2;
    //                    set_pixel(point, cof * t.getColor());
    //                }
    //            }
    //        }
    //    }
    //    
    //}

    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);

 
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

    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
    auto ind = (height - 1 - point.y()) * width + point.x();
    //auto ind = get_index(point.x(), point.y());
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}

