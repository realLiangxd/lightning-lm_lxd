#pragma once

#include "common/eigen_types.h"

#include <pangolin/gl/glvbo.h>

namespace lightning::ui {

/// UI中的轨迹绘制
class UiTrajectory {
   public:
    UiTrajectory(const Vec3f& color) : color_(color) { pos_.reserve(max_size_); }

    /// 增加一个轨迹点到opengl缓冲区
    void AddPt(const SE3& pose);

    /// 渲染此轨迹
    void Render();

    void Clear() {
        pos_.clear();
        pos_.reserve(max_size_);
        vbo_.Free();
    }

    Vec3f At(const uint64_t idx) const { return pos_.at(idx); }

   private:
    int max_size_ = 1e6;                               // 记录的最大点数
    std::vector<Eigen::Vector3f> pos_;                 // 轨迹记录数据
    Eigen::Vector3f color_ = Eigen::Vector3f::Zero();  // 轨迹颜色显示
    pangolin::GlBuffer vbo_;                           // 显存顶点信息
    // pangolin::GlBuffer vbo_; 声明了一个来自 Pangolin 库的对象，它用于管理 OpenGL 的顶点缓冲对象 (Vertex Buffer Object, VBO)。

    // 什么是 VBO？
    // VBO 是一种 OpenGL 的特性，它允许你将顶点数据（例如点的位置、颜色、法线等）一次性地从 CPU 内存（RAM）上传到 GPU的显存中。

    // 为什么使用 VBO？
    // 高性能渲染：对于需要绘制大量顶点（比如成千上万个点的轨迹或点云）的场景，每次渲染时都从 CPU 逐点发送数据给 GPU会非常慢。 减少 CPU-GPU 带宽占用：使用 VBO，你可以把所有顶点数据一次性批量发送到显存。在渲染时，你只需要告诉 GPU "去显存的这个位置把数据拿来画"，而不需要再传输数据。这大大提高了渲染效率。
};

}  // namespace lightning::ui
