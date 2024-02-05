#include "UI.h"

#include <vsg/all.h>
#include <vsgImGui/imgui.h>

#include "vsgCs/Config.h"
#include "vsgCs/runtimeSupport.h"


using namespace vsgCs;

bool UI::createUI(const vsg::ref_ptr<vsg::Window>& window,
                  const vsg::ref_ptr<vsg::Viewer>& viewer,
                  const vsg::ref_ptr<vsg::Camera>& camera,
                  const vsg::ref_ptr<vsg::EllipsoidModel>& ellipsoidModel,
                  const vsg::ref_ptr<vsg::Options>&)
{
    createImGui(window);
    // Add the ImGui event handler first to handle events early
    viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());
    viewer->addEventHandler(vsg::CloseHandler::create(viewer));

    _trackball = vsg::Trackball::create(camera, ellipsoidModel);

    {
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('1'), 51.50151088842245, -0.14181489107549874, 2000.0, 2.0); // Grenwish Observatory
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('2'), 55.948642740309324, -3.199226855522667, 2000.0, 2.0);  // Edinburgh Castle
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('3'), 48.858264952330764, 2.2945039609604665, 2000.0, 2.0);  // Eiffel Town, Paris
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('4'), 52.5162603714634, 13.377684902745642, 2000.0, 2.0);    // Brandenburg Gate, Berlin
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('5'), 30.047448591298807, 31.236319571791213, 10000.0, 2.0); // Cairo
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('6'), 35.653099536061156, 139.74704060056993, 10000.0, 2.0); // Tokyo
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('7'), 37.38701052699002, -122.08555895549424, 10000.0, 2.0); // Mountain View, California
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('8'), 40.689618207006355, -74.04465595488215, 10000.0, 2.0); // Empire State Building
        _trackball->addKeyViewpoint(static_cast<vsg::KeySymbol>('9'), 25.997055873649554, -97.15543476551771, 1000.0, 2.0);  // Boca Chica, Taxas
        // osgEarthStyleMouseButtons
        _trackball->panButtonMask = vsg::BUTTON_MASK_1;
        _trackball->rotateButtonMask = vsg::BUTTON_MASK_2;
        _trackball->zoomButtonMask = vsg::BUTTON_MASK_3;
    }

    viewer->addEventHandler(_trackball);

    return true;
}

bool UI::createUI2(const vsg::ref_ptr<vsg::Window>& window,
                  const vsg::ref_ptr<vsg::Viewer>& viewer,
                  const vsg::ref_ptr<vsg::Camera>& camera,
                  const vsg::ref_ptr<vsg::Options>&)
{
    createImGui(window);
    // Add the ImGui event handler first to handle events early
    viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());
    viewer->addEventHandler(vsg::CloseHandler::create(viewer));

    _trackball = vsg::Trackball::create(camera);
    {
        // osgEarthStyleMouseButtons
        _trackball->panButtonMask = vsg::BUTTON_MASK_1;
        _trackball->rotateButtonMask = vsg::BUTTON_MASK_2;
        _trackball->zoomButtonMask = vsg::BUTTON_MASK_3;
    }
    viewer->addEventHandler(_trackball);

    return true;
}

vsg::ref_ptr<vsgImGui::RenderImGui> UI::createImGui(const vsg::ref_ptr<vsg::Window>& window)
{
    _ionIconComponent = CsApp::CreditComponent::create();
    _renderImGui = vsgImGui::RenderImGui::create(window, _ionIconComponent);
    return _renderImGui;
}

void UI::setViewpoint(const vsg::ref_ptr<vsg::LookAt>& lookAt, float duration)
{
    _trackball->setViewpoint(lookAt, duration);
}
