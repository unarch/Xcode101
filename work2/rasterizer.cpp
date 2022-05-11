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


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f P = Vector3f(x, y, 1.0f);
    const Vector3f& A = _v[0];
    const Vector3f& B = _v[1];
    const Vector3f& C = _v[2];
    
    Vector3f AB = B - A;
    Vector3f BC = C - B;
    Vector3f CA = A - C;
    
    Vector3f AP = P - A;
    Vector3f BP = P - B;
    Vector3f CP = P - C;
    
    float z1 = AB.cross(AP).z();
    float z2 = BC.cross(BP).z();
    float z3 = CA.cross(CP).z();
    
    
    
    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
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
            vert.z() = vert.z() * f1 + f2;
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

        rasterize_triangle(t);
    }
}

Vector4f getBoundingBox(std::array<Eigen::Vector4f, 3> v) {
    float min_x = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));
    float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    min_x = (int)floorf(min_x);
    max_x = (int)ceilf(max_x);
    min_y = (int)floorf(min_y);
    max_y = (int)ceilf(max_y);
    return Vector4f(min_x, max_x, min_y, max_y);
}

void rst::rasterizer::MSAA(const Vector4f& boundBox, const Triangle& t) {
    auto v = t.toVector4();
    int pixel_size = 10;
    
    for (int x = boundBox.x(); x <= boundBox.y(); x++) {
        for(int y = boundBox.z(); y <= boundBox.w(); y++) {
            int count = 0;
            float gap = 1.0 / pixel_size;
            for(float i = gap / 2.0f; i < 1.0f; i += gap) {
                for (float j = gap / 2.0f; j < 1.0f; j += gap) {
                    float xx = float(x + i), yy = float(y + j);
                    if (insideTriangle(xx, yy, t.v)) {
                        count++;
                    }
                }
            }
            if (count > 0) {
                auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5f, y + 0.5f, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if (depth_buf[get_index(x, y)] > z_interpolated) {
                    int size = (pixel_size * pixel_size);
                    Vector3f color = t.getColor() * count / size  + (size - count) * frame_buf[get_index(x, y)] / size;
                    Vector3f point = Vector3f(float(x), float(y), z_interpolated);
                    depth_buf[get_index(x, y)] = z_interpolated;
                    set_pixel(point, color);
                }
            }
            if (count < pixel_size * pixel_size) {
                depth_buf[get_index(x, y)] = std::numeric_limits<float>::infinity();
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    Vector4f boundBox = getBoundingBox(v);
    
    bool msaa = true;
    if (msaa) {
        MSAA(boundBox, t);
        return;
    }
    
    for (int x = boundBox.x(); x <= boundBox.y(); x++) {
        for (int y = boundBox.z(); y <= boundBox.w(); y++) {
            if (insideTriangle(x + 0.5f, y + 0.5f, t.v)) {
                auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5f, y + 0.5f, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if (depth_buf[get_index(x, y)] > z_interpolated) {
                    Vector3f color = t.getColor();
                    Vector3f point = Vector3f(float(x), float(y), z_interpolated);
                    depth_buf[get_index(x, y)] = z_interpolated;
                    set_pixel(point, color);
                    
                }
            }
        }
    }

    
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
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on
