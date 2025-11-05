#include "ui/ui_car.h"
#include <GL/gl.h>      // OpenGL 头文件，OpenGL (Open Graphics Library) 是一个用于渲染 2D 和 3D 矢量图形的跨平台应用程序编程接口（API）。
namespace lightning::ui {

std::vector<Vec3f> UiCar::car_vertices_ = {
    // clang-format off
     { 0, 0, 0}, { 3.0, 0, 0},
     { 0, 0, 0}, { 0, 3.0, 0},
     { 0, 0, 0}, { 0, 0, 3.0},
    // clang-format on
};

void UiCar::SetPose(const SE3& pose) {
    pts_.clear();
    for (auto& p : car_vertices_) {
        pts_.emplace_back(p);
    }

    // 转换到世界系
    auto pose_f = pose.cast<float>();
    for (auto& pt : pts_) {
        pt = pose_f * pt;
    }
}

void UiCar::Render() {
    glLineWidth(5.0);   // 线宽
    glBegin(GL_LINES);  // 画线

    /// x -红, y-绿 z-蓝
    glColor3f(1.0, 0.0, 0.0);   // 红色
    glVertex3f(pts_[0][0], pts_[0][1], pts_[0][2]);     // 画红色线段
    glVertex3f(pts_[1][0], pts_[1][1], pts_[1][2]);

    glColor3f(0.0, 1.0, 0.0);   // 绿色
    glVertex3f(pts_[2][0], pts_[2][1], pts_[2][2]);
    glVertex3f(pts_[3][0], pts_[3][1], pts_[3][2]);

    glColor3f(0.0, 0.0, 1.0);   // 蓝色
    glVertex3f(pts_[4][0], pts_[4][1], pts_[4][2]);
    glVertex3f(pts_[5][0], pts_[5][1], pts_[5][2]);
    glEnd();    // 结束绘制
}

}  // namespace lightning::ui
