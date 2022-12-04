#pragma once

#include "Scene.h"

#include <utility>
#include <vector>

#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>

using namespace cg3d;
using namespace std;
using namespace Eigen;
using namespace igl;

class part1 : public cg3d::Scene
{
public:
    explicit part1(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;

    void init_values();
    void simplify();

    void empty_mesh_list();


private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> bunny;
    std::shared_ptr<cg3d::Model> autoModel;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F, E, EF, EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V, C;
    Eigen::MatrixXi OF;
    Eigen::MatrixXd OV;
    Eigen::MatrixXd VN, FN, T;
    igl::min_heap<std::tuple<double, int, int>> Q;
    int num_collapsed;
    int index;
    int total_versions;
};