#include <iostream>
#include <stdlib.h>


#include <shmfw/vector.h>
#include <boost/program_options.hpp>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>

namespace bi = boost::interprocess;
namespace po = boost::program_options;

struct Prarmeters {
    bool clear;
    bool unlock;
    double update_frq;
    int border, widht, height;
    int trigger_channel;
    double trigger_level;
    std::string variable_name;
};

Prarmeters readArgs ( int argc, char **argv ) {
    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "clear", "clears the shm first, default is no" )
    ( "unlock", "unlocks the shared variables related to the oszi" )
    ( "border,b", po::value<int> ( &params.border )->default_value ( 10 ), "screen border" )
    ( "widht,w", po::value<int> ( &params.widht )->default_value ( 600 ), "image width" )
    ( "height,h", po::value<int> ( &params.height )->default_value ( 300 ), "image height" )
    ( "trigger_channel,t", po::value<int> ( &params.trigger_channel )->default_value ( -1 ), "trigger channel, -1 means floating" )
    ( "trigger_level,l", po::value<double> ( &params.trigger_level )->default_value ( 0 ), "trigger level" )
    ( "update_frq,f", po::value<double> ( &params.update_frq )->default_value ( 10.0 ), "oszi update frequenz [Hz]" )
    ( "variable_name,v", po::value<std::string> ( &params.variable_name )->default_value ( "oszi" ), "name of the shared variable" );

    po::variables_map vm;
    try {
        po::store ( po::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const std::exception &ex ) {
        std::cout << desc << std::endl;;
        exit ( 1 );
    }
    po::notify ( vm );

    if ( vm.count ( "help" ) )  {
        std::cout << desc << std::endl;
        exit ( 1 );
    }
    params.clear = ( vm.count ( "clear" ) > 0 );
    params.unlock = ( vm.count ( "unlock" ) > 0 );

    return params;
}

int main ( int, char *[] ) {
    // Create a sphere
    vtkSmartPointer<vtkSphereSource> sphereSource01 = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource01->SetCenter ( 0.0, 0.0, 0.0 );
    sphereSource01->SetRadius ( 1.0 );
    sphereSource01->Update();

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =  vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection ( sphereSource01->GetOutputPort() );
    // mapper->AddInputConnection ( sphereSource02->GetOutputPort() );

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper ( mapper );

    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition ( 0, 0, 20 );
    camera->SetFocalPoint ( 0, 0, 0 );

    // Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

    renderer->SetActiveCamera ( camera );

    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer ( renderer );
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =  vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow ( renderWindow );

    // Add the actor to the scene
    renderer->AddActor ( actor );
    renderer->SetBackground ( 1,1,1 ); // Background color white

    // Render and interact
    renderWindow->Render();
    renderWindowInteractor->Start();

    return EXIT_SUCCESS;
}
