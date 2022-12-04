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

class part2 : public cg3d::Scene
{
public:
    explicit part2(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void init_first_time();
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
   
    void empty_mesh_list();

    void init_values();
    void Q_calculation();
    void cost_calculation(int edge);
    void simplify();
    bool new_collapse_edge();


private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> bunny, sphere1, cube;
    std::shared_ptr<cg3d::Model> autoModel;

    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F, E, EF, EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V, C;

    Eigen::MatrixXi OF;
    Eigen::MatrixXd OV;
    Eigen::MatrixXd VN, FN, T;

    int index;
    int total_versions;
    typedef std::set<std::pair<double, int>> PriorityQueue;
    PriorityQueue Q;
    std::vector<PriorityQueue::iterator> Q_iter;
    std::vector<Eigen::Matrix4d> Q_matrix;
};