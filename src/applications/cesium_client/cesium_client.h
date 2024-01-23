/* <editor-fold desc="MIT License">

Copyright(c) 2023 Timothy Moore

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

</editor-fold> */

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

    class CesiumClient : public vsg::Inherit<vsg::Object, CesiumClient>
    {
        public:
        int renderCesium(const vsg::ref_ptr<vsgCs::CommandOptions> &options);

        protected:
        vsg::ref_ptr<CommandOptions> _options;
    };
}
