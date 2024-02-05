#pragma once

#include <vsg/all.h>
#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include "CsApp/CreditComponent.h"

namespace vsgCs
{
    class UI : public vsg::Inherit<vsg::Object, UI>
    {
        public:
        bool createUI(const vsg::ref_ptr<vsg::Window>& window,
                      const vsg::ref_ptr<vsg::Viewer>& viewer,
                      const vsg::ref_ptr<vsg::Camera>& camera,
                      const vsg::ref_ptr<vsg::EllipsoidModel>& ellipsoidModel,
                      const vsg::ref_ptr<vsg::Options>& options);

        bool createUI2(const vsg::ref_ptr<vsg::Window>& window,
                      const vsg::ref_ptr<vsg::Viewer>& viewer,
                      const vsg::ref_ptr<vsg::Camera>& camera,
                      const vsg::ref_ptr<vsg::Options>& options);

        vsg::ref_ptr<vsgImGui::RenderImGui> getImGui()
        {
            return _renderImGui;
        }
        void setViewpoint(const vsg::ref_ptr<vsg::LookAt>& lookAt, float duration);
        protected:
        vsg::ref_ptr<vsgImGui::RenderImGui> createImGui(const vsg::ref_ptr<vsg::Window>& window);

        vsg::ref_ptr<vsgImGui::RenderImGui> _renderImGui;
        vsg::ref_ptr<CsApp::CreditComponent> _ionIconComponent;
        vsg::ref_ptr<vsg::Trackball> _trackball;
    };
}
