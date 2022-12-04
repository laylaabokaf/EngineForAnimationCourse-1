#include "part1.h"
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

void part1::simplify()
{
    // Collapse 10% of edges
    if (!Q.empty())
    {
        bool something_collapsed = false;
        // Collapse edge
        const int max_iter = ceil(0.1 * Q.size());
        for (int j = 0; j < max_iter; j++)
        {
            if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
            {
                break;
            }
            something_collapsed = true;
            num_collapsed++;
        }
        if (something_collapsed)
        {
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
}

void part1::init_values()
{
    F = OF;
    V = OV;
    igl::edge_flaps(F, E, EMAP, EF, EI);
    C.resize(E.rows(), V.cols());
    VectorXd costs(E.rows());
    Q = {};
    EQ = VectorXi::Zero(E.rows());
    {
        VectorXd costs(E.rows());
        igl::parallel_for(E.rows(), [&](const int e)
            {
                double cost = e;
                RowVectorXd p(1, 3);
                shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
                C.row(e) = p;
                costs(e) = cost;
            }, 10000);
        for (int e = 0; e < E.rows(); e++)
        {
            Q.emplace(costs(e), e, 0);
        }
    }
    num_collapsed = 0;
    index = 0;
    autoModel->meshIndex = index;
}

void part1::empty_mesh_list() {

    auto mesh = autoModel->GetMeshList();
    for (int i = 1; i < total_versions; i++)
    {
        mesh[0]->data.pop_back();
    }
    autoModel->SetMeshList(mesh);
    total_versions = 1;
}

void part1::Init(float fov, int width, int height, float near, float far)
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
    auto sphereMesh{ IglLoader::MeshFromFiles("bunny_igl", "data/bunny.off") };

    bunny = Model::Create("bunny", sphereMesh, material);
    bunny->Scale(10);
    bunny->showWireframe = true;
    bunny->Translate({ 5,0,0 });
    camera->Translate(30, Axis::Z);
    auto mesh = bunny->GetMeshList();
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;

    auto morph_function = [](Model* model, cg3d::Visitor* visitor)
    {
        int current_index = model->meshIndex;
        return (model->GetMeshList())[0]->data.size() * 0 + current_index;
    };
    autoModel = AutoMorphingModel::Create(*bunny, morph_function);
    root->AddChild(autoModel);
    autoModel->Translate({ 0, 0, 25 });

    OF = F;
    OV = V;
    index = 0;
    total_versions = 1;

    init_values();
}

void part1::Update(const Program& program, const Matrix4f& proj, const Matrix4f& view, const Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
}

void part1::KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
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
            index = (index - 1 < 0) ? total_versions - 1 : index - 1;
            autoModel->meshIndex = index;
            break;
        case GLFW_KEY_DOWN:
            index = (index == total_versions) ? 0 : index + 1;
            autoModel->meshIndex = index;
            break;
        }
    }
}