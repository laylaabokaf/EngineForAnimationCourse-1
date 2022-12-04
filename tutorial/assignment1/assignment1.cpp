#include "Renderer.h"
#include "Display.h"
#include "part2.h"
#define SCENE part1 // the scene (class name) to display
#define SCENE2 part2 // the scene (class name) to display

#define STRINGIFY(X) #X
#define CLASS_NAME_TO_HEADER(X) STRINGIFY(X.h)
#include CLASS_NAME_TO_HEADER(SCENE)


using namespace cg3d;

int main()
{
    const int DISPLAY_WIDTH = 800;
    const int DISPLAY_HEIGHT = 800;
    const float CAMERA_ANGLE = 45.0f;
    const float NEAR = 0.1f;
    const float FAR = 120.0f;
    char buffer[100];
    std::cout << "Enter part number: (part1|part2)" << std::endl;
    fgets(buffer, 100, stdin);
    buffer[strlen(buffer) - 1] = '\0';
    if (strcmp(buffer, "part1") == 0) {
        Renderer renderer;
        Display display("Part1", DISPLAY_WIDTH, DISPLAY_HEIGHT, &renderer);
        std::cout.setstate(std::ios_base::failbit); // suppress junk output to console from igl::opengl::glfw::Viewer
        auto scene = std::make_shared<SCENE>(STRINGIFY(SCENE), &display);
        std::cout.clear(); // re-enable output to console
        auto viewport = std::make_shared<Viewport>("viewport", 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        scene->Init(CAMERA_ANGLE, DISPLAY_WIDTH, DISPLAY_HEIGHT, NEAR, FAR);
        renderer.AddViewport(scene);
        display.LaunchRendering(true);
    }
    else if (strcmp(buffer, "part2") == 0) {
        Renderer renderer;
        Display display("Part2", DISPLAY_WIDTH, DISPLAY_HEIGHT, &renderer);
        std::cout.setstate(std::ios_base::failbit); // suppress junk output to console from igl::opengl::glfw::Viewer
        auto scene = std::make_shared<SCENE2>(STRINGIFY(SCENE2), &display);
        std::cout.clear(); // re-enable output to console
        auto viewport = std::make_shared<Viewport>("viewport", 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        scene->Init(CAMERA_ANGLE, DISPLAY_WIDTH, DISPLAY_HEIGHT, NEAR, FAR);
        renderer.AddViewport(scene);
        display.LaunchRendering(true);
    }
    else {
        std::cout.clear(); // re-enable output to console
        std::cout << "Wrong part " << std::endl;
    }
    return 0;
}