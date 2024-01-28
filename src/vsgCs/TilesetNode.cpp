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

#include "TilesetNode.h"

#include "CsOverlay.h"
#include "jsonUtils.h"
#include "OpThreadTaskProcessor.h"
#include "RuntimeEnvironment.h"
#include "Tracing.h"
#include "UrlAssetAccessor.h"

#include <CesiumUtility/JsonHelpers.h>
#include <CesiumGltf/AccessorView.h>
#include <CesiumGltf/Accessor.h>
#include <CesiumGltf/Buffer.h>
#include <CesiumGltf/ExtensionCesiumRTC.h>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtx/color_encoding.hpp>
#include <glm/gtc/color_space.hpp>

#include <optional>
#include <cmath>
#include <vsg/core/ref_ptr.h>
#include <vsg/io/Options.h>

using namespace vsgCs;
using namespace CesiumGltf;

template<typename F>
void for_each_view(const vsg::ref_ptr<vsg::Viewer>& viewer, const F& f)
{
    for (auto& recordAndSubmitTask : viewer->recordAndSubmitTasks)
    {
        for (auto& commandGraph : recordAndSubmitTask->commandGraphs)
        {
            for (auto& child : commandGraph->children)
            {
                auto rg = ref_ptr_cast<vsg::RenderGraph>(child);
                if (rg)
                {
                    for (auto& rgChild : rg->children)
                    {
                        auto view = ref_ptr_cast<vsg::View>(rgChild);
                        if (view)
                        {
                            f(view, rg);
                        }
                    }
                }
            }
        }
    }
}

TilesetNode::TilesetNode(const DeviceFeatures& deviceFeatures, const TilesetSource& source,
                         const Cesium3DTilesSelection::TilesetOptions& tilesetOptions,
                         const vsg::ref_ptr<vsg::Options>&)
    : _viewUpdateResult(nullptr), _tilesetsBeingDestroyed(0)
{
    Cesium3DTilesSelection::TilesetOptions options(tilesetOptions);
    // turn off all the unsupported stuff
    options.enableOcclusionCulling = false;
    // Generous per-frame time limits for loading / unloading on main thread.
    options.mainThreadLoadingTimeLimit = 5.0;
    options.tileCacheUnloadTimeLimit = 5.0;
    options.contentOptions.enableWaterMask = false;
    options.loadErrorCallback =
        [](const Cesium3DTilesSelection::TilesetLoadFailureDetails& details)
        {
            if (details.statusCode != 200)
            {
                vsg::warn("status code = ", details.statusCode);
            }
            if (!details.message.empty())
            {
                vsg::warn(details.message);
            }
        };
    auto externals = RuntimeEnvironment::get()->getTilesetExternals();
    options.contentOptions.ktx2TranscodeTargets = deviceFeatures.ktx2TranscodeTargets;

    if (source.url)
    {
        _tileset = std::make_unique<Cesium3DTilesSelection::Tileset>(*externals, source.url.value(), options);
    }
    else
    {
        if (source.ionAssetEndpointUrl)
        {
            _tileset
                = std::make_unique<Cesium3DTilesSelection::Tileset>(*externals,
                                                                    source.ionAssetID.value(),
                                                                    source.ionAccessToken.value(),
                                                                    options,
                                                                    source.ionAssetEndpointUrl.value());
        }
        else
        {
            _tileset
                = std::make_unique<Cesium3DTilesSelection::Tileset>(*externals,
                                                                    source.ionAssetID.value(),
                                                                    source.ionAccessToken.value(),
                                                                    options);
        }
    }
}

// From Cesium Unreal:
// Don't allow this Cesium3DTileset to be fully destroyed until
// any cesium-native Tilesets it created have wrapped up any async
// operations in progress and have been fully destroyed.
// See IsReadyForFinishDestroy.

void TilesetNode::shutdown()
{
    if (_tileset)
    {
        // Kind of gross, but the overlay is going to call TilesetNode::removeOverlay, which mutates
        // the _overlays vector.
        vsg::ref_ptr<TilesetNode> ref_this(this);
        while (!_overlays.empty())
        {
            (*(_overlays.end() - 1))->removeFromTileset(ref_this);
        }
        ++_tilesetsBeingDestroyed;
        _tileset->getAsyncDestructionCompleteEvent().thenInMainThread(
            [this]()
            {
                --this->_tilesetsBeingDestroyed;
            });
        _tileset.reset();
    }
}

TilesetNode::~TilesetNode()
{
    shutdown();
}

std::vector<Cesium3DTilesSelection::Tile*> TilesetNode::getRenderTiles()
{
    std::vector<Cesium3DTilesSelection::Tile*> renderTiles = std::vector<Cesium3DTilesSelection::Tile*>{};
    if (_viewUpdateResult)
    {
        long long tileCount = 0;
        for (auto* tile : _viewUpdateResult->tilesToRenderThisFrame)
        {
            const auto& tileContent = tile->getContent();
            if (tileContent.isRenderContent())
            {
                tileCount++;
#if 0
                auto timestampMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                auto tileIDStr =  Cesium3DTilesSelection::TileIdUtilities::createTileIdString(tile->getTileID());
                vsg::warn("tileContentRender: ", timestampMs, " - ", tileIDStr);
#endif
                renderTiles.push_back(tile);
            }
        }

# if 0
        vsg::warn("tileContentRender: ", " - render-tile-count:", tileCount);
#endif

    }

    return renderTiles;
}


std::vector<Cesium3DTilesSelection::Tile*> TilesetNode::getRenderTilesContent()
{
    std::vector<Cesium3DTilesSelection::Tile *> renderTiles = std::vector<Cesium3DTilesSelection::Tile *>{};
    if (_viewUpdateResult)
    {
        // get all render-tiles
        long long tileCount = 0;
        for (auto* tile : _viewUpdateResult->tilesToRenderThisFrame)
        {
            const auto& tileContent = tile->getContent();
            if (tileContent.isRenderContent())
            {
                tileCount++;
                renderTiles.push_back(tile);
            }
        }
        // vsg::warn("tileContentRender: ", " - render-tile-count:", tileCount);

#if 0
        // get mergedGltf for all gltf-models
        CesiumGltf::Model &mergedModel = CesiumGltf::Model();
        for (auto* tile : renderTiles)
        {
            const auto& tileContent = tile->getContent();
            CesiumGltf::Model gltfModel = tileContent.getRenderContent()->getModel();

            mergedModel.merge(std::move(gltfModel));
        }

        vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
            ", gltf.meshes:", mergedModel.meshes.size(),
            ", gltf.accessors:", mergedModel.accessors.size(),
            ", gltf.buffers:", mergedModel.buffers.size()
            );

        // get mergedGltf buffer0 point-count
        if (mergedModel.buffers.size() > 0 ) {
            const CesiumGltf::Buffer& buffer = mergedModel.buffers.front();

            // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
            //     // ", gltf.meshes:", mergedModel.meshes.size(),
            //     // ", gltf.accessors:", mergedModel.accessors.size(),
            //     ", gltf.buffers:", mergedModel.buffers.size(),
            //     // ", gltf.buffer0.byteLength:", buffer.byteLength,
            //     // ", gltf.buffer0.data.size:", buffer.cesium.data.size(),
            //     ", gltf.buffer0.data.count:", buffer.cesium.data.size()/sizeof(glm::vec3)
            //     );

        }

#endif

#if 0
        // get all gltf-models with points
        long long pointCount = 0;
        for (auto* tile : renderTiles)
        {
            const auto& tileContent = tile->getContent();
            CesiumGltf::Model gltfModel = tileContent.getRenderContent()->getModel();

            CesiumGltf::ExtensionCesiumRTC *rtcExt = NULL;
            if (gltfModel.hasExtension<CesiumGltf::ExtensionCesiumRTC>()) {
                rtcExt = gltfModel.getExtension<CesiumGltf::ExtensionCesiumRTC>();
                vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    ", gltf.meshes:", gltfModel.meshes.size(),
                    ", gltf.accessors:", gltfModel.accessors.size(),
                    ", gltf.buffers:", gltfModel.buffers.size(),
                    ", center[0]:", rtcExt->center[0], ", center[1]:", rtcExt->center[1], ", center[2]:", rtcExt->center[2]
                    );
            }

            glm::vec3 point;
            for (auto &buffer : gltfModel.buffers)
            {
                auto &bytesData = buffer.cesium.data;
                pointCount += bytesData.size()/sizeof(glm::vec3);
                const int32_t byteStride = sizeof(glm::vec3);
                const glm::vec3& point0 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 0 * byteStride);
                const glm::vec3& point1 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 1 * byteStride);

                point = point0;
                if (rtcExt) {
                    point += glm::vec3(rtcExt->center[0], rtcExt->center[1], rtcExt->center[2]);
                }
                vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    ", gltf.meshes:", gltfModel.meshes.size(),
                    ", gltf.accessors:", gltfModel.accessors.size(),
                    ", gltf.buffers:", gltfModel.buffers.size(),
                    ", point0 x:", point.x, ", y:", point.y, ", z:", point.z
                    );

                point = point1;
                if (rtcExt) {
                    point += glm::vec3(rtcExt->center[0], rtcExt->center[1], rtcExt->center[2]);
                }
                vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    ", gltf.meshes:", gltfModel.meshes.size(),
                    ", gltf.accessors:", gltfModel.accessors.size(),
                    ", gltf.buffers:", gltfModel.buffers.size(),
                    ", point1 x:", point.x, ", y:", point.y, ", z:", point.z
                    );
            }

        }

        vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
            ", gltf.pointCount:", pointCount
            );

#endif

# if 0
        // buffers total point-count
        long long pointCount = 0;
        for (auto &buffer : mergedModel.buffers)
        {
            auto &bytesData = buffer.cesium.data;
            pointCount += bytesData.size()/sizeof(glm::vec3);
            const int32_t byteStride = sizeof(glm::vec3);
            const glm::vec3& point = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 0 * byteStride);
            const glm::vec3& point1 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 1 * byteStride);
            // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
            //     ", gltf.buffers:", mergedModel.buffers.size(),
            //     ", gltf.pointCount:", pointCount,
            //     ", x:", point.x, ", y:", point.y, ", z:", point.z
            //     );
            // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
            //     ", gltf.buffers:", mergedModel.buffers.size(),
            //     ", gltf.pointCount:", pointCount,
            //     ", 1x:", point1.x, ", 1y:", point1.y, ", 1z:", point1.z
            //     );
        }

        vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
            ", gltf.buffers:", mergedModel.buffers.size(),
            ", gltf.pointCount:", pointCount
            );
#endif

#if 0
        // try failed --- get points from gltf-model accessor
        if (mergedModel.accessors.size() > 0 ) {
            const CesiumGltf::Accessor& accessor = mergedModel.accessors.front();

            vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                ", gltfModel.accessors:", mergedModel.accessors.size(),
                ", gltfModel.accessor0.count:", accessor.count
                );

        }
#endif

# if 1
        // try failed --- get points from gltf-model primitives/meshes/accessor
        long long pointCount = 0;

        for (auto* tile : renderTiles)
        {
            const auto& tileContent = tile->getContent();
            CesiumGltf::Model gltfModel = tileContent.getRenderContent()->getModel();

            // CHECK(gltf.hasExtension<CesiumGltf::ExtensionCesiumRTC>());
            // CHECK(gltf.nodes.size() == 1);

            long long positionCount = 0;
            long long colorCount = 0;

            CesiumGltf::ExtensionCesiumRTC *rtcExt = NULL;
            if (gltfModel.hasExtension<CesiumGltf::ExtensionCesiumRTC>()) {
                rtcExt = gltfModel.getExtension<CesiumGltf::ExtensionCesiumRTC>();
                // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                //     ", gltf.meshes:", gltfModel.meshes.size(),
                //     ", gltf.accessors:", gltfModel.accessors.size(),
                //     ", gltf.buffers:", gltfModel.buffers.size(),
                //     ", center[0]:", rtcExt->center[0], ", center[1]:", rtcExt->center[1], ", center[2]:", rtcExt->center[2]
                //     );
            }

            if (gltfModel.meshes.size() > 0 ) {
                if(gltfModel.meshes.size() != 1) {vsg::error("bad gltfModel.meshes.size(): ", gltfModel.meshes.size());}

                // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                //     ", gltf.meshes:", gltfModel.meshes.size(),
                //     ", gltf.meshes0.primitives:", gltfModel.meshes[0].primitives.size()
                //     );

                const CesiumGltf::Mesh& mesh = gltfModel.meshes.front();
                // CHECK(mesh.primitives.size() == 1);
                if(mesh.primitives.size() != 1) {vsg::error("bad mesh.primitives.size()", mesh.primitives.size());}

                const CesiumGltf::MeshPrimitive& primitive = mesh.primitives.front();
                // CHECK(primitive.attributes.size() == 2);
                if(primitive.attributes.size() != 2) {vsg::error("bad primitive.attributes.size(): ", primitive.attributes.size());}
                if(primitive.mode != MeshPrimitive::Mode::POINTS) {vsg::error("bad primitive.mode: ", primitive.mode);}

                // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                //     ", gltf.meshes:", gltfModel.meshes.size(),
                //     ", meshe0.primitives:", mesh.primitives.size(),
                //     ", primitive.attributes:", primitive.attributes.size()
                //     );

                const auto attributes = primitive.attributes;

                glm::vec3 position;
                glm::vec3 color;

                // // Check that position and color attributes are present
                // checkAttribute<glm::vec3>(gltf, primitive, "POSITION", pointsLength);
                // checkAttribute<glm::vec3>(gltf, primitive, "COLOR_0", pointsLength);

                {
                    // Check position attribute more thoroughly
                    uint32_t positionAccessorId = static_cast<uint32_t>(attributes.at("POSITION"));
                    Accessor& positionAccessor = gltfModel.accessors[positionAccessorId];
                    // CHECK(!positionAccessor.normalized);
                    // if(positionAccessor.normalized) {vsg::error("bad positionAccessor.normalized: ", positionAccessor.normalized);}

                    uint32_t positionBufferViewId = static_cast<uint32_t>(positionAccessor.bufferView);
                    BufferView& positionBufferView = gltfModel.bufferViews[positionBufferViewId];

                    uint32_t positionBufferId = static_cast<uint32_t>(positionBufferView.buffer);
                    Buffer& positionBuffer = gltfModel.buffers[positionBufferId];

                    auto &bytesData = positionBuffer.cesium.data;
                    positionCount = bytesData.size()/sizeof(glm::vec3);
                    const int32_t byteStride = sizeof(glm::vec3);
                    const glm::vec3& position0 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 0 * byteStride);
                    const glm::vec3& position1 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 1 * byteStride);

                    position = position0;
                    if (rtcExt) {
                        position += glm::vec3(rtcExt->center[0], rtcExt->center[1], rtcExt->center[2]);
                    }
                    // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    //     ", gltf.meshes:", gltfModel.meshes.size(),
                    //     ", gltf.accessors:", gltfModel.accessors.size(),
                    //     ", gltf.buffers:", gltfModel.buffers.size(),
                    //     ", gltf.bufferIndex:", positionBufferId,
                    //     ", positionCount:", positionCount,
                    //     ", position0 x:", position.x, ", y:", position.y, ", z:", position.z
                    //     );

                }

                {
                    // Check color attribute more thoroughly
                    uint32_t colorAccessorId = static_cast<uint32_t>(attributes.at("COLOR_0"));
                    Accessor& colorAccessor = gltfModel.accessors[colorAccessorId];
                    // CHECK(!colorAccessor.normalized);
                    if(colorAccessor.normalized) {vsg::error("bad colorAccessor.normalized: ", colorAccessor.normalized);}

                    uint32_t colorBufferViewId = static_cast<uint32_t>(colorAccessor.bufferView);
                    BufferView& colorBufferView = gltfModel.bufferViews[colorBufferViewId];

                    uint32_t colorBufferId = static_cast<uint32_t>(colorBufferView.buffer);
                    Buffer& colorBuffer = gltfModel.buffers[colorBufferId];

                    auto &bytesData = colorBuffer.cesium.data;
                    colorCount = bytesData.size()/sizeof(glm::vec3);
                    const int32_t byteStride = sizeof(glm::vec3);
                    const glm::vec3& color0 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 0 * byteStride);
                    const glm::vec3& color1 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 1 * byteStride);

                    // color = glm::convertLinearToSRGB(color);
                    color = glm::pow(color, glm::vec3(1.0 / 2.2));
                    color *= 255;

                    // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    //     ", gltf.meshes:", gltfModel.meshes.size(),
                    //     ", gltf.accessors:", gltfModel.accessors.size(),
                    //     ", gltf.buffers:", gltfModel.buffers.size(),
                    //     ", gltf.bufferIndex:", colorBufferId,
                    //     ", colorCount:", colorCount,
                    //     ", color0 x:", color.x, ", y:", color.y, ", z:", color.z
                    //     );

                }

                if (positionCount != colorCount) {
                    vsg::error("mismatch positionCount and colorCount.", " positionCount:", positionCount, ", colorCount:", colorCount);
                }

                pointCount += positionCount;
            }

        }

        vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
            ", gltf.pointCount:", pointCount
            );
# endif

    }

    return renderTiles;
}

RenderContent* TilesetNode::getRenderContent()
{
    auto ellipsoidModel = vsg::EllipsoidModel::create();

    RenderContent* renderContent =  new RenderContent();
    std::vector<Cesium3DTilesSelection::Tile *> renderTiles = std::vector<Cesium3DTilesSelection::Tile *>{};

    if (_viewUpdateResult)
    {
        auto beginTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        // get all render-tiles
        long long tileCount = 0;
        for (auto* tile : _viewUpdateResult->tilesToRenderThisFrame)
        {
            const auto& tileContent = tile->getContent();
            if (tileContent.isRenderContent())
            {
                tileCount++;
                renderTiles.push_back(tile);
            }
        }
        // vsg::warn("tileContentRender: ", " - render-tile-count:", tileCount);

# if 1
        // try failed --- get points from gltf-model primitives/meshes/accessor
        long long pointCount = 0;

        for (auto* tile : renderTiles)
        {
            const auto& tileContent = tile->getContent();
            CesiumGltf::Model gltfModel = tileContent.getRenderContent()->getModel();

            // CHECK(gltf.hasExtension<CesiumGltf::ExtensionCesiumRTC>());
            // CHECK(gltf.nodes.size() == 1);

            long long positionCount = 0;
            long long colorCount = 0;

            CesiumGltf::ExtensionCesiumRTC *rtcExt = NULL;
            if (gltfModel.hasExtension<CesiumGltf::ExtensionCesiumRTC>()) {
                rtcExt = gltfModel.getExtension<CesiumGltf::ExtensionCesiumRTC>();
                // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                //     ", gltf.meshes:", gltfModel.meshes.size(),
                //     ", gltf.accessors:", gltfModel.accessors.size(),
                //     ", gltf.buffers:", gltfModel.buffers.size(),
                //     ", center[0]:", rtcExt->center[0], ", center[1]:", rtcExt->center[1], ", center[2]:", rtcExt->center[2]
                //     );
            }

            if (gltfModel.meshes.size() > 0 ) {
                if(gltfModel.meshes.size() != 1) {vsg::error("bad gltfModel.meshes.size(): ", gltfModel.meshes.size());}

                // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                //     ", gltf.meshes:", gltfModel.meshes.size(),
                //     ", gltf.meshes0.primitives:", gltfModel.meshes[0].primitives.size()
                //     );

                const CesiumGltf::Mesh& mesh = gltfModel.meshes.front();
                // CHECK(mesh.primitives.size() == 1);
                if(mesh.primitives.size() != 1) {vsg::error("bad mesh.primitives.size()", mesh.primitives.size());}

                const CesiumGltf::MeshPrimitive& primitive = mesh.primitives.front();
                // CHECK(primitive.attributes.size() == 2);
                if(primitive.attributes.size() != 2) {vsg::error("bad primitive.attributes.size(): ", primitive.attributes.size());}
                if(primitive.mode != MeshPrimitive::Mode::POINTS) {vsg::error("bad primitive.mode: ", primitive.mode);}

                // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                //     ", gltf.meshes:", gltfModel.meshes.size(),
                //     ", meshe0.primitives:", mesh.primitives.size(),
                //     ", primitive.attributes:", primitive.attributes.size()
                //     );

                const auto attributes = primitive.attributes;

                glm::vec3 position;
                glm::vec3 color;

                // // Check that position and color attributes are present
                // checkAttribute<glm::vec3>(gltf, primitive, "POSITION", pointsLength);
                // checkAttribute<glm::vec3>(gltf, primitive, "COLOR_0", pointsLength);

                {
                    // Check position attribute more thoroughly
                    uint32_t positionAccessorId = static_cast<uint32_t>(attributes.at("POSITION"));
                    Accessor& positionAccessor = gltfModel.accessors[positionAccessorId];
                    // CHECK(!positionAccessor.normalized);
                    if(positionAccessor.normalized) {vsg::error("bad positionAccessor.normalized: ", positionAccessor.normalized);}
                    if(positionAccessor.type != "VEC3") {vsg::error("bad positionAccessor.type: ", positionAccessor.type);}


                    uint32_t positionBufferViewId = static_cast<uint32_t>(positionAccessor.bufferView);
                    BufferView& positionBufferView = gltfModel.bufferViews[positionBufferViewId];

                    uint32_t positionBufferId = static_cast<uint32_t>(positionBufferView.buffer);
                    Buffer& positionBuffer = gltfModel.buffers[positionBufferId];

                    auto &bytesData = positionBuffer.cesium.data;
                    positionCount = bytesData.size()/sizeof(glm::vec3);
                    const int32_t byteStride = sizeof(glm::vec3);
                    const glm::vec3& position0 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 0 * byteStride);
                    const glm::vec3& position1 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 1 * byteStride);

                    position = position0;
                    if (rtcExt) {
                        position += glm::vec3(rtcExt->center[0], rtcExt->center[1], rtcExt->center[2]);
                    }
                    // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    //     ", gltf.meshes:", gltfModel.meshes.size(),
                    //     ", gltf.accessors:", gltfModel.accessors.size(),
                    //     ", gltf.buffers:", gltfModel.buffers.size(),
                    //     ", gltf.bufferIndex:", positionBufferId,
                    //     ", positionCount:", positionCount,
                    //     ", position0 x:", position.x, ", y:", position.y, ", z:", position.z
                    //     );

                    // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    //     ", positionCount:", positionCount,
                    //     ", position0 x:", position.x, ", y:", position.y, ", z:", position.z
                    //     );

                    for (auto i = 0; i < positionCount; i++) {
                        const glm::vec3& position0 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + i * byteStride);
                        position = position0;
                        if (rtcExt) {
                            position += glm::vec3(rtcExt->center[0], rtcExt->center[1], rtcExt->center[2]);
                        }

                        // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                        //     ", positionCount:", positionCount, ", index:", i,
                        //     ", position0 x:", position.x, ", y:", position.y, ", z:", position.z
                        //     );

                        renderContent->positions.push_back(position);
                    }

                }

                {
                    // Check color attribute more thoroughly
                    uint32_t colorAccessorId = static_cast<uint32_t>(attributes.at("COLOR_0"));
                    Accessor& colorAccessor = gltfModel.accessors[colorAccessorId];
                    // CHECK(!colorAccessor.normalized);
                    if(colorAccessor.normalized) {vsg::error("bad colorAccessor.normalized: ", colorAccessor.normalized);}
                    if(colorAccessor.type != "VEC3") {vsg::error("bad colorAccessor.type: ", colorAccessor.type);}

                    uint32_t colorBufferViewId = static_cast<uint32_t>(colorAccessor.bufferView);
                    BufferView& colorBufferView = gltfModel.bufferViews[colorBufferViewId];

                    uint32_t colorBufferId = static_cast<uint32_t>(colorBufferView.buffer);
                    Buffer& colorBuffer = gltfModel.buffers[colorBufferId];

                    auto &bytesData = colorBuffer.cesium.data;
                    colorCount = bytesData.size()/sizeof(glm::vec3);
                    const int32_t byteStride = sizeof(glm::vec3);
                    const glm::vec3& color0 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 0 * byteStride);
                    const glm::vec3& color1 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + 1 * byteStride);

                    color = color0;
                    // color = glm::convertLinearToSRGB(color);
                    color = glm::pow(color, glm::vec3(1.0 / 2.2));
                    color *= 255;

                    // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    //     ", gltf.meshes:", gltfModel.meshes.size(),
                    //     ", gltf.accessors:", gltfModel.accessors.size(),
                    //     ", gltf.buffers:", gltfModel.buffers.size(),
                    //     ", gltf.bufferIndex:", colorBufferId,
                    //     ", colorCount:", colorCount,
                    //     ", color0 r:", color.x, ", g:", color.y, ", b:", color.z
                    //     );

                    // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                    //     ", colorCount:", colorCount, ", index:", i,
                    //     ", color0 r:", color.x, ", g:", color.y, ", b:", color.z
                    //     );

                    for (auto i = 0; i < colorCount; i++) {
                        const glm::vec3& color0 = *reinterpret_cast<const glm::vec3*>(bytesData.data() + i * byteStride);
                        color = color0;
                        color = glm::pow(color, glm::vec3(1.0 / 2.2));
                        color *= 255;

                        // vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
                        //     ", colorCount:", colorCount, ", index:", i,
                        //     ", color0 r:", color.x, ", g:", color.y, ", b:", color.z
                        //     );

                        renderContent->colors.push_back(color);
                    }

                }

                if (positionCount != colorCount) {
                    vsg::error("mismatch positionCount and colorCount.", " positionCount:", positionCount, ", colorCount:", colorCount);
                }

                pointCount += positionCount;
            }

        }
        auto endTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        long long elaspeTimeMs = endTimeMs - beginTimeMs;

        vsg::warn("tileContentRenderModel: ", " - render-tile-count:", tileCount,
            ", gltf.pointCount:", pointCount,
            ", elaspeTimeMs:", elaspeTimeMs
            );
# endif

    }

    renderContent->renderTiles = renderTiles;
    return renderContent;
}

template<class V>
void TilesetNode::t_traverse(V& visitor) const
{
    if (_viewUpdateResult)
    {
        long long tileCount = 0;
        for (auto* tile : _viewUpdateResult->tilesToRenderThisFrame)
        {
            const auto& tileContent = tile->getContent();
            if (tileContent.isRenderContent())
            {
                tileCount++;
#if 0
                std::time_t now = std::time(nullptr);
                auto timestampMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                auto tileIDStr =  Cesium3DTilesSelection::TileIdUtilities::createTileIdString(tile->getTileID());
                vsg::warn("tileContentRender: ", timestampMs, " - ", tileIDStr);
#endif
                const auto* renderResources
                    = reinterpret_cast<const RenderResources*>(tileContent.getRenderContent()
                                                               ->getRenderResources());
                renderResources->model->accept(visitor);
            }
        }

# if 0
        vsg::warn("tileContentRender: ", " - render-tile-count:", tileCount);
#endif

    }
}

void TilesetNode::traverse(vsg::Visitor& visitor)
{
    t_traverse(visitor);
}

void TilesetNode::traverse(vsg::ConstVisitor& visitor) const
{
    t_traverse(visitor);
}

void TilesetNode::traverse(vsg::RecordTraversal& visitor) const
{
    t_traverse(visitor);
}

// We need to supply our cameras' poses (position, direction, up) to Cesium in its coordinate
// system i.e., Z up ECEF. The Cesium terrain may be attached to a VSG scenegraph with arbitrary
// transformations; at the very least, it should have a transformation to VSG Y up coordinates in
// its parents. If we follow the kinematic chain, The pose Pcs of the Cesium camera is
// 	Pcs = inv(Pw) * Pvsg * Tyup
// where Pw is the chain of transforms in the earth's parents, Pvsg is the pose of the VSG camera,
// and Tyup is the Z up to Y up transform.
// We actually have in hand the inverses of these matrices (pose is the inverse of a camera's view
// matrix), so we will calculate inv(Pcs) and invert it.

vsg::dmat4 TilesetNode::zUp2yUp(1.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, -1.0, 0.0,
                                0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1.0);

vsg::dmat4 TilesetNode::yUp2zUp(1.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 1.0, 0.0,
                                0.0, -1.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1.0);

class FindNodeVisitor : public vsg::Inherit<vsg::Visitor, FindNodeVisitor>
{
public:
    explicit FindNodeVisitor(const vsg::ref_ptr<vsg::Node>& node)
        : _node(node)
    {}
    explicit FindNodeVisitor(vsg::Node* node)
        : _node(vsg::ref_ptr<vsg::Node>(node))
    {}
    void apply(vsg::Node& node) override
    {
        if (!resultPath.empty())
        {
            return;
        }
        _objectPath.push_back(&node);
        if (&node == _node.get())
        {
            resultPath.insert(resultPath.begin(), _objectPath.begin(), _objectPath.end());
        }
        else
        {
            node.traverse(*this);
        }
        _objectPath.pop_back();
    }

    vsg::RefObjectPath resultPath;
private:
    vsg::ref_ptr<vsg::Node> _node;
    vsg::ObjectPath _objectPath;
};

struct ViewData : public vsg::Inherit<vsg::Object, ViewData>
{
    explicit ViewData(vsg::RefObjectPath& in_tilesetPath)
        : tilesetPath(in_tilesetPath)
    {}
    vsg::RefObjectPath tilesetPath;
};

namespace
{
    std::optional<Cesium3DTilesSelection::ViewState>
    createViewState(const vsg::ref_ptr<vsg::View>& view, const vsg::ref_ptr<vsg::RenderGraph>& renderGraph)
    {
        auto* viewData = dynamic_cast<ViewData*>(view->getObject("vsgCsViewData"));
        if (!viewData)
        {
            return {};
        }
        vsg::dmat4 Pw = vsg::computeTransform(viewData->tilesetPath);
        vsg::dmat4 PcsInv = TilesetNode::yUp2zUp * view->camera->viewMatrix->transform() * Pw;
        vsg::dmat4 Pcs = vsg::inverse(PcsInv);
        glm::dvec3 position(Pcs[3][0], Pcs[3][1], Pcs[3][2]);
        glm::dvec3 direction(Pcs[1][0], Pcs[1][1], Pcs[1][2]);
        glm::dvec3 up(Pcs[2][0], Pcs[2][1], Pcs[2][2]);
        // Have to assume that we have a perspective projection
        double fovy = 0.0;
        double fovx = 0.0;
        vsg::ref_ptr<vsg::ProjectionMatrix> projMat = view->camera->projectionMatrix;
        auto* persp = dynamic_cast<vsg::Perspective*>(projMat.get());
        if (persp)
        {
            fovy = vsg::radians(persp->fieldOfViewY);
            fovx = std::atan(std::tan(fovy / 2.0) * persp->aspectRatio) * 2.0;
        }
        else
        {
            vsg::dmat4 perspMat = projMat->transform();
            fovy = 2.0 * std::atan(-1.0 / perspMat[1][1]);
            fovx = 2.0 * std::atan(1.0 / perspMat[0][0]);
        }
        glm::dvec2 viewportSize;
        if (view->camera->viewportState)
        {
            VkViewport viewport = view->camera->viewportState->getViewport();
            viewportSize[0] = viewport.width;
            viewportSize[1] = viewport.height;
        }
        else
        {
            viewportSize[0] = renderGraph->renderArea.extent.width;
            viewportSize[1] = renderGraph->renderArea.extent.height;
        }
        Cesium3DTilesSelection::ViewState result =
            Cesium3DTilesSelection::ViewState::create(position, direction, up, viewportSize,
                                                      fovx, fovy);
        return {result};
    }
}



void TilesetNode::updateViews(const vsg::ref_ptr<vsg::Viewer>& viewer)
{
    for_each_view(viewer,
                  [this](const vsg::ref_ptr<vsg::View>& view, const vsg::ref_ptr<vsg::RenderGraph>&)
                  {
                      FindNodeVisitor visitor(this);
                      view->accept(visitor);
                      if (visitor.resultPath.empty())
                      {
                          view->removeObject("vsgCsViewData");
                      }
                      else
                      {
                          view->setObject("vsgCsViewData", ViewData::create(visitor.resultPath));
                      }
                  });
}

void TilesetNode::UpdateTileset::run()
{
    vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
    vsg::ref_ptr<TilesetNode> ref_tileset = tilesetNode;

    if (!(ref_viewer && ref_tileset))
    {
        return;
    }
    if (!ref_tileset->_tileset)
    {
        return;
    }
    VSGCS_ZONESCOPEDN("update view");
    std::vector<Cesium3DTilesSelection::ViewState> viewStates;
    for_each_view(viewer,
                  [&viewStates](const vsg::ref_ptr<vsg::View>& view, const vsg::ref_ptr<vsg::RenderGraph>& rg)
                  {
                      auto viewState = createViewState(view, rg);
                      if (viewState)
                      {
                          viewStates.push_back(viewState.value());
                      }
                  });
    ref_tileset->_viewUpdateResult = &ref_tileset->_tileset->updateView(viewStates);
}

bool TilesetNode::initialize(const vsg::ref_ptr<vsg::Viewer>& viewer)
{
    updateViews(viewer);
    // Making a ref_ptr from this is gross. If the caller doesn't hold a ref, then this will be
    // deleted at the end of the function! We could do unref_nodelete, but UpdateTileset holds
    // observer_ptrs... Anyway, keeping this "alive" for the whole function avoids a compiler /
    // clang-tidy error.
    vsg::ref_ptr<TilesetNode> ref(this);
    viewer->addUpdateOperation(UpdateTileset::create(ref, viewer), vsg::UpdateOperations::ALL_FRAMES);
    return true;
}

void TilesetNode::addOverlay(const vsg::ref_ptr<CsOverlay>& overlay)
{
    _overlays.push_back(overlay);
    _tileset->getOverlays().add(overlay->getOverlay());
}

void TilesetNode::removeOverlay(const vsg::ref_ptr<CsOverlay>& overlay)
{
    _tileset->getOverlays().remove(overlay->getOverlay());
    _overlays.erase(std::remove(_overlays.begin(), _overlays.end(), overlay), _overlays.end());
}

namespace
{
    vsg::ref_ptr<vsg::Object> buildTilesetNode(const rapidjson::Value& json,
                                               JSONObjectFactory* factory,
                                               const vsg::ref_ptr<vsg::Object>&)
    {
        auto env = RuntimeEnvironment::get();
        // Current implementation of TilesetNode constructs Cesium tileset in the constructor, using
        // its arguments... so the object argument is not useful.
        // auto tilesetNode = create_or<TilesetNode>(object);
        auto ionAsset = CesiumUtility::JsonHelpers::getInt64OrDefault(json, "ionAssetID", -1);
        auto tilesetUrl = CesiumUtility::JsonHelpers::getStringOrDefault(json, "tilesetUrl", "");
        auto ionAccessToken = CesiumUtility::JsonHelpers::getStringOrDefault(json, "ionAccessToken", "");
        auto ionEndpointUrl = CesiumUtility::JsonHelpers::getStringOrDefault(json, "ionEndpointUrl", "");
        vsgCs::TilesetSource source;
        if (!tilesetUrl.empty())
        {
            source.url = tilesetUrl;
        }
        else
        {
            if (ionAsset < 0)
            {
                vsg::error("No valid Ion asset\n");
                return {};
            }
            source.ionAssetID = ionAsset;
            if (ionAccessToken.empty())
            {
                ionAccessToken = env->ionAccessToken;
            }
            source.ionAccessToken = ionAccessToken;
            if (!ionEndpointUrl.empty())
            {
                source.ionAssetEndpointUrl = ionEndpointUrl;
            }
        }
        Cesium3DTilesSelection::TilesetOptions tileOptions;
        tileOptions.enableOcclusionCulling = false;
        tileOptions.forbidHoles = true;
        const auto stylingItr = json.FindMember("styling");
        if (stylingItr != json.MemberEnd() && stylingItr->value.IsObject())
        {
            auto styling = ref_ptr_cast<Styling>(factory->build(stylingItr->value, "Styling"));
            tileOptions.rendererOptions = styling;
        }
        else
        {
            tileOptions.rendererOptions = Styling::create();
        }
        auto tilesetNode = vsgCs::TilesetNode::create(env->features, source, tileOptions, env->options);
        const auto itr = json.FindMember("overlays");
        if (itr != json.MemberEnd() && itr->value.IsArray())
        {
            const auto& valueJson = itr->value;
            for (rapidjson::SizeType i = 0; i < valueJson.Size(); ++i)
            {
                const auto& element = valueJson[i];
                auto built = factory->build(element);
                vsg::ref_ptr<CsOverlay> overlay = ref_ptr_cast<CsOverlay>(built);
                if (!overlay)
                {
                    vsg::error("expected CSOverly, got ", built->className());
                    break;
                }
                overlay->layerNumber = i;
                overlay->addToTileset(tilesetNode);
            }
        }
        return tilesetNode;
    }

    JSONObjectFactory::Registrar r("Tileset", buildTilesetNode);
}
