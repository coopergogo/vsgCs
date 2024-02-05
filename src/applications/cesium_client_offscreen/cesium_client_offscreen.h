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

        int applyLookAt();
        int applyViewport();

        protected:
        vsg::ref_ptr<CommandOptions> _options;

        vsg::ref_ptr<vsg::Trackball> _trackball;
        vsg::ref_ptr<OffscreenControl> _offscreenControl;

        vsg::ref_ptr<vsg::Camera> _displayCamera;
        vsg::ref_ptr<vsg::Camera> _offscreenCamera;
    };
}
