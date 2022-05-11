//
//  main.cpp
//  Xcode101
//
//  Created by unarch on 2022/4/4.
//

#include <iostream>
#include <eigen3/Eigen/Core>

using namespace std;

using Eigen::MatrixXd;
//给定一个点 P =(2,1), 将该点绕原点先逆时针旋转 45◦，再平移 (1,2), 计算出 变换后点的坐标(要求用齐次坐标进行计算)。
int main()
{
    Eigen::Vector3f p(2.0f, 1.0f, 1.0f);
    double delta = (45.0f / 180.0f) * acos(-1);
    
    Eigen::Matrix3f r;
    r << cos(delta), -sin(delta), 1,
         sin(delta), cos(delta), 2,
    0, 0, 1;
    Eigen::Vector3f ans = r * p;
    std::cout << ans << std::endl;
}
