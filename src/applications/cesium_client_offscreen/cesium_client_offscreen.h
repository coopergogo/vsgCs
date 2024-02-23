#pragma once

#include <vsg/all.h>
#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include "CsApp/CreditComponent.h"

namespace vsgCs
{
    class CommandOptions : public vsg::Inherit<vsg::Object, CommandOptions>
    {
        public:
        vsg::CommandLine *arguments;

        vsg::Path file_path;
        int numFrames;
        std::string pathFilename;
        double horizonMountainHeight;
        bool useEllipsoidPerspective;

        double poi_latitude;
        double poi_longitude;
        double poi_distance;
        int log_level;

        long ionAsset;
        long ionOverlay;
        std::string tilesetUrl;
        std::string ionEndpointUrl;
        bool useHeadlight;

    };

    class OffscreenControl: public vsg::Inherit<vsg::Object, OffscreenControl>
    {
        public:
        OffscreenControl();

        int setLookAt(long epsgCode, vsg::dvec3 eye,  vsg::dvec3 center,  vsg::dvec3 up);
        int setViewport(float x, float y, float width, float height);

        vsg::ref_ptr<vsg::LookAt> getLookAt() { return _lookAt ? _lookAt : vsg::LookAt::create(); }
        VkViewport getViewport() { return _viewportState ? _viewportState->getViewport() : VkViewport{}; }

        protected:
        vsg::ref_ptr<vsg::LookAt> _lookAt;  // ecef
        vsg::dvec3 _eye;
        vsg::dvec3 _center;
        vsg::dvec3 _up;
        long _epsgCode; // epsg for eye/centre/up

        vsg::ref_ptr<vsg::ViewportState> _viewportState;
        float _x;
        float _y;
        float _width;
        float _height;
    };

    class CesiumClient : public vsg::Inherit<vsg::Object, CesiumClient>
    {
        public:
        int renderCesium(const vsg::ref_ptr<vsgCs::CommandOptions> &options);

        void setOffscreenControl(vsg::ref_ptr<OffscreenControl> &offscreenControl) { _offscreenControl = offscreenControl; }
        vsg::ref_ptr<OffscreenControl> getOffscreenControl() { return _offscreenControl; }

        void setTrackball(vsg::ref_ptr<vsg::Trackball> &trackball) { _trackball = trackball; }

        void setDisplayCamera(vsg::ref_ptr<vsg::Camera> &displayCamera) { _displayCamera = displayCamera; }
        void setOffscreenCamera(vsg::ref_ptr<vsg::Camera> &offscreenCamera) { _offscreenCamera = offscreenCamera; }

        void setDisplayRenderGraph(vsg::ref_ptr<vsg::RenderGraph> &displayRenderGraph) { _displayRenderGraph = displayRenderGraph; }
        void setOffscreenRenderGraph(vsg::ref_ptr<vsg::RenderGraph> &offscreenRenderGraph) { _offscreenRenderGraph = offscreenRenderGraph; }

        // int setLookAt(long epsgCode, vsg::dvec3 eye,  vsg::dvec3 center,  vsg::dvec3 up);
        // int setViewport(float x, float y, float width, float height);
        int applyLookAt();
        int applyViewport();

        protected:
        vsg::ref_ptr<CommandOptions> _options;

        vsg::ref_ptr<OffscreenControl> _offscreenControl;
        vsg::ref_ptr<vsg::Trackball> _trackball;

        vsg::ref_ptr<vsg::Camera> _displayCamera;
        vsg::ref_ptr<vsg::Camera> _offscreenCamera;
        vsg::ref_ptr<vsg::RenderGraph> _displayRenderGraph;
        vsg::ref_ptr<vsg::RenderGraph> _offscreenRenderGraph;
    };
}
