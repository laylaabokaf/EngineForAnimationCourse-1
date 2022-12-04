#include "part2.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"

#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/vertex_triangle_adjacency.h>
#include <Eigen/Core>
#include <iostream>
#include <set>

#include <vector>

#include "AutoMorphingModel.h"

using namespace std;
using namespace cg3d;
using namespace std;
using namespace Eigen;
using namespace igl;

void part2::simplify()
{
    bool something_collapsed = false;

    // Collapse 10% of edges
    const int max_iter = ceil(0.1 * Q.size());
    for (int i = 0; i < max_iter; i++) {
        if (!new_collapse_edge()) {
            break;
        }
        something_collapsed = true;
    }
    if (something_collapsed) {
        total_versions++;
        index++;
        igl::per_vertex_normals(V, F, VN);
        T = MatrixXd::Zero(V.rows(), 2);
        auto mesh = autoModel->GetMeshList();
        mesh[0]->data.push_back({ V, F, VN, T });
        autoModel->SetMeshList(mesh);
        autoModel->meshIndex = index;
    }
}

void part2::init_values()
{
    V = OV;
    F = OF;
    
    igl::edge_flaps(F, E, EMAP, EF, EI);
    C.resize(E.rows(), V.cols());
    Q_iter.resize(E.rows());
    Q_calculation();
    Q.clear();

    for (int i = 0; i < E.rows(); i++) {
        cost_calculation(i);
    }
    index = 0;
    autoModel->meshIndex = 0;
}

void part2::Q_calculation()
{
    vector<vector<int>> VF;
    vector<vector<int>> VFi;
    int n = V.rows();
    Q_matrix.resize(n);
    igl::vertex_triangle_adjacency(n, F, VF, VFi);
    igl::per_face_normals(V, F, FN);

    for (int i = 0; i < n; i++)
    {
        Q_matrix[i] = Matrix4d::Zero();
        for (int j = 0; j < VF[i].size(); j++) { 
            Vector3d normal = FN.row(VF[i][j]).normalized(); 
            double a = normal[0];
            double b = normal[1];
            double c = normal[2];
            double d = -V.row(i) * normal;
            Matrix4d Kp;
            Kp.row(0) = Vector4d(a * a, a * b, a * c, a * d);
            Kp.row(1) = Vector4d(a * b, b * b, b * c, b * d);
            Kp.row(2) = Vector4d(a * c, b * c, c * c, c * d);
            Kp.row(3) = Vector4d(a * d, b * d, c * d, d * d);
            Q_matrix[i] += Kp;
        }
    }
}

void part2::cost_calculation(int edge)
{
    int v1 = E(edge, 0);
    int v2 = E(edge, 1);
    double cost;
    bool isInversable;
    Matrix4d Q_edge = Q_matrix[v1] + Q_matrix[v2], Q_position = Q_edge;
    Q_position.row(3) = Vector4d(0, 0, 0, 1);
    Q_position.computeInverseWithCheck(Q_position, isInversable);
    Vector4d v_pos;
    if (isInversable)
    {
        v_pos = Q_position * (Vector4d(0, 0, 0, 1));
        cost = v_pos.transpose() * Q_edge * v_pos;
    } else {
        double cost1, cost2, cost3;
        Vector4d v1_pos;
        Vector4d v2_pos;
        Vector4d v1_v2_pos;
        v1_pos << V.row(v1), 1;
        cost1 = v1_pos.transpose() * Q_edge * v1_pos;
        v2_pos<< V.row(v2), 1;
        cost2 = v2_pos.transpose() * Q_edge * v2_pos;
        v1_v2_pos << ((V.row(v1) + V.row(v2)) / 2), 1;
        cost3 = v1_v2_pos.transpose() * Q_edge * v1_v2_pos;

        if ((cost1 < cost2) && (cost1 < cost3))
        {
            v_pos = v1_pos;
            cost = cost1;
        }
        else if ((cost2 < cost1) && (cost2 < cost3))
        {
            v_pos = v2_pos;
            cost = cost2;
        } else {
            v_pos = v1_v2_pos;
            cost = cost3;
        }
    }
    Vector3d new_pos;
    new_pos[0] = v_pos[0];
    new_pos[1] = v_pos[1];
    new_pos[2] = v_pos[2];
    C.row(edge) = new_pos;
    Q_iter[edge] = Q.insert(pair<double, int>(cost, edge)).first;
}

bool part2::new_collapse_edge() {
    PriorityQueue& temp_Q = Q;
    vector<PriorityQueue::iterator>& curr_Q_iter = Q_iter;
    int e1, e2, f1, f2; 
    if (temp_Q.empty()) {
        return false;
    }
    pair<double, int> pair = *(temp_Q.begin());
    if (pair.first == numeric_limits<double>::infinity())
    {
        return false;
    }
    temp_Q.erase(temp_Q.begin());
    int e = pair.second;

    int v1 = E.row(e)[0];
    int v2 = E.row(e)[1];
    curr_Q_iter[e] = temp_Q.end();

    vector<int> N = igl::circulation(e, true, EMAP, EF, EI);
    vector<int> Nd = igl::circulation(e, false, EMAP, EF, EI);
    N.insert(N.begin(), Nd.begin(), Nd.end());

    bool is_collapsed = igl::collapse_edge(e, C.row(e), V, F, E, EMAP, EF, EI, e1, e2, f1, f2);
    if (is_collapsed)
    {
        temp_Q.erase(curr_Q_iter[e1]);
        curr_Q_iter[e1] = temp_Q.end();
        temp_Q.erase(curr_Q_iter[e2]);
        curr_Q_iter[e2] = temp_Q.end();

        Q_matrix[v1] = Q_matrix[v1] + Q_matrix[v2];
        Q_matrix[v2] = Q_matrix[v1] + Q_matrix[v2];
        VectorXd new_position;

        for (auto n : N) {
            if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL || F(n, 1) != IGL_COLLAPSE_EDGE_NULL || F(n, 2) != IGL_COLLAPSE_EDGE_NULL) {
                for (int v = 0; v < 3; v++) {
                    const int ei = EMAP(v * F.rows() + n); 
                    temp_Q.erase(curr_Q_iter[ei]); 
                    cost_calculation(ei);
                    new_position = C.row(ei);
                }
            }
        }
    } else {
        pair.first = numeric_limits<double>::infinity();
        curr_Q_iter[e] = temp_Q.insert(pair).first;
    }
    return is_collapsed;
}

void part2::empty_mesh_list() {
    auto mesh = autoModel->GetMeshList();
    for (int i = 1; i < total_versions; i++)
    {
        mesh[0]->data.pop_back();
    }
    autoModel->SetMeshList(mesh);
    total_versions = 1;
}

void part2::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root"));
    auto daylight{ make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = make_shared<Program>("shaders/basicShader");
    auto material{ make_shared<Material>("material", program) };

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };

    sphere1 = Model::Create("sphere", sphereMesh, material);
    sphere1->Scale(2);
    sphere1->showWireframe = true;
    sphere1->Translate({ -3,0,0 });
    camera->Translate(30, Axis::Z);

    auto mesh = sphere1->GetMeshList();

    // Function to reset original mesh and data structures
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;

    // Start of new code
    auto morph_function = [](Model* model, cg3d::Visitor* visitor)
    {
        int current_index = model->meshIndex;
        return (model->GetMeshList())[0]->data.size() * 0 + current_index;
    };
    autoModel = AutoMorphingModel::Create(*sphere1, morph_function);
    root->AddChild(autoModel);
    autoModel->Translate({ 0, 0, 25 });
    init_first_time();
}

void part2::init_first_time() {
    OF = F;
    OV = V;
    index = 0;
    total_versions = 1;
    init_values();

}

void part2::Update(const Program& program, const Matrix4f& proj, const Matrix4f& view, const Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
}

void part2::KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key)
        {
        case GLFW_KEY_SPACE:
            simplify();
            break;
        case GLFW_KEY_R:
            empty_mesh_list();
            init_values();
            break;
        case GLFW_KEY_UP:
            if (index - 1 < 0) index = total_versions - 1;
            else index -= 1;
            autoModel->meshIndex = index;
            break;
        case GLFW_KEY_DOWN:
            if (index == total_versions) index = 0;
            else index += 1;
            autoModel->meshIndex = index;
            break;
        }
    }
}
