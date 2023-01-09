#include "vsgResourcePreparer.h"
#include "CompilableImage.h"

#include <Cesium3DTilesSelection/GltfUtilities.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <CesiumAsync/AsyncSystem.h>

#include <limits>

using namespace vsgCs;
using namespace CesiumGltf;

// This deletion queue is a hack to not delete VSG objects (like buffers) while
// they might still be in use. The problem is that VSG recycles descriptors...

DeletionQueue::DeletionQueue()
    : lastFrameRun(std::numeric_limits<uint64_t>::max())
{
}

void DeletionQueue::add(vsg::ref_ptr<vsg::Viewer> viewer,
                        vsg::ref_ptr<vsg::Object> object)
{
    queue.push_back(Deletion{viewer->getFrameStamp()->frameCount, object});
}

void DeletionQueue::run()
{
    queue.clear();
    lastFrameRun = std::numeric_limits<uint64_t>::max();
}

void DeletionQueue::run(vsg::ref_ptr<vsg::Viewer> viewer)
{
    const auto frameStamp = viewer->getFrameStamp();
    if (lastFrameRun == std::numeric_limits<uint64_t>::max())
    {
        queue.clear();
    }
    else
    {
        if (frameStamp->frameCount <= lastFrameRun)
            return;
        auto itr = queue.begin();
        while (itr != queue.end())
        {
            if (itr->frameRemoved + 2 <= frameStamp->frameCount)
            {
                itr = queue.erase(itr);
            }
            else
            {
                break;
            }
        }
    }
    lastFrameRun = frameStamp->frameCount;
}

LoadModelResult*
vsgResourcePreparer::readAndCompile(Cesium3DTilesSelection::TileLoadResult &&tileLoadResult,
                                    const glm::dmat4& transform,
                                    const CreateModelOptions& options)
{
    vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
    if (!ref_viewer)
        return nullptr;
    auto resultNode = _builder->loadTile(std::move(tileLoadResult), transform, options);
    LoadModelResult* result = new LoadModelResult;
    result->modelResult = resultNode;
    result->compileResult = ref_viewer->compileManager->compile(resultNode);
    return result;
}

RenderResources* merge(vsgResourcePreparer* preparer, LoadModelResult& result)
{
    vsg::ref_ptr<vsg::Viewer> ref_viewer = preparer->viewer;
    if (ref_viewer)
    {
        updateViewer(*ref_viewer, result.compileResult);
        return new RenderResources{result.modelResult};
    }
    return nullptr;
}

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
vsgResourcePreparer::prepareInLoadThread(const CesiumAsync::AsyncSystem& asyncSystem,
                                         Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
                                         const glm::dmat4& transform,
                                         const std::any&)
{
    CesiumGltf::Model* pModel = std::get_if<CesiumGltf::Model>(&tileLoadResult.contentKind);
    if (!pModel)
    {
        return asyncSystem.createResolvedFuture(
            Cesium3DTilesSelection::TileLoadResultAndRenderResources{
                std::move(tileLoadResult),
                nullptr});
    }

    CreateModelOptions options;
    options.renderOverlays
        = (tileLoadResult.rasterOverlayDetails
           && !tileLoadResult.rasterOverlayDetails.value().rasterOverlayProjections.empty());
    LoadModelResult* result = readAndCompile(std::move(tileLoadResult), transform, options);
    return asyncSystem.createResolvedFuture(
        Cesium3DTilesSelection::TileLoadResultAndRenderResources{
            std::move(tileLoadResult),
            result});
}

void*
vsgResourcePreparer::prepareInMainThread(Cesium3DTilesSelection::Tile& tile,
                                         void* pLoadThreadResult)
{
    const Cesium3DTilesSelection::TileContent& content = tile.getContent();
    if (content.isRenderContent())
    {
        LoadModelResult* loadModelResult = reinterpret_cast<LoadModelResult*>(pLoadThreadResult);
        return merge(this, *loadModelResult);
    }
    return nullptr;
}

void vsgResourcePreparer::free(Cesium3DTilesSelection::Tile&,
                               void* pLoadThreadResult,
                               void* pMainThreadResult) noexcept
{
    vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
    LoadModelResult* loadModelResult = reinterpret_cast<LoadModelResult*>(pLoadThreadResult);
    RenderResources* renderResources = reinterpret_cast<RenderResources*>(pMainThreadResult);

    if (ref_viewer)
    {
        _deletionQueue.run(ref_viewer);
        if (pLoadThreadResult)
        {
            _deletionQueue.add(ref_viewer, loadModelResult->modelResult);
        }
            if (pMainThreadResult)
        if (ref_viewer)
        {
            _deletionQueue.add(ref_viewer, renderResources->model);
        }

    }
    delete loadModelResult;
    delete renderResources;
}

void*
vsgResourcePreparer::prepareRasterInLoadThread(CesiumGltf::ImageCesium& image,
                                               const std::any&)
{
    vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
    if (!ref_viewer)
        return nullptr;
    auto result = _builder->loadTexture(image,
                                        VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
                                        VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
                                        VK_FILTER_LINEAR,
                                        VK_FILTER_LINEAR,
                                        true,
                                        true);
    auto compilable = CompilableImage::create(result);
    return new LoadRasterResult{compilable->imageInfo, ref_viewer->compileManager->compile(compilable)};
}

void*
vsgResourcePreparer::prepareRasterInMainThread(Cesium3DTilesSelection::RasterOverlayTile&,
                                               void* rawLoadResult)
{
    LoadRasterResult* loadRasterResult = static_cast<LoadRasterResult*>(rawLoadResult);
    vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
    if (ref_viewer)
    {
        vsg::updateViewer(*ref_viewer, loadRasterResult->compileResult);
    }

    return new RasterResources{loadRasterResult->rasterResult};
}

void
vsgResourcePreparer::freeRaster(const Cesium3DTilesSelection::RasterOverlayTile&,
                                void* loadThreadResult,
                                void* mainThreadResult) noexcept
{
    vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
    LoadRasterResult* loadRasterResult = static_cast<LoadRasterResult*>(loadThreadResult);
    RasterResources* rasterResources = static_cast<RasterResources*>(mainThreadResult);
    if (ref_viewer)
    {
        _deletionQueue.run(ref_viewer);
        if (loadThreadResult)
        {
            _deletionQueue.add(ref_viewer, loadRasterResult->rasterResult);
        }
            if (mainThreadResult)
        if (ref_viewer)
        {
            _deletionQueue.add(ref_viewer, rasterResources->raster);
        }
    }

    delete loadRasterResult;
    delete rasterResources;
}

void
vsgResourcePreparer::attachRasterInMainThread(const Cesium3DTilesSelection::Tile& tile,
                                  int32_t overlayTextureCoordinateID,
                                  const Cesium3DTilesSelection::RasterOverlayTile& rasterTile,
                                  void* pMainThreadRendererResources,
                                  const glm::dvec2& translation,
                                  const glm::dvec2& scale)
{
    vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
    if (!ref_viewer)
        return;
    const Cesium3DTilesSelection::TileContent& content = tile.getContent();
    const Cesium3DTilesSelection::TileRenderContent* renderContent = content.getRenderContent();
    if (renderContent)
    {
        RenderResources* resources = static_cast<RenderResources*>(renderContent->getRenderResources());

        auto object = _builder->attachRaster(tile, resources->model,
                                             overlayTextureCoordinateID, rasterTile,
                                             pMainThreadRendererResources, translation, scale);
        if (object.valid())
        {
            auto compileResult = ref_viewer->compileManager->compile(object);
            vsg::updateViewer(*ref_viewer, compileResult);
        }
    }
}

void
vsgResourcePreparer::detachRasterInMainThread(const Cesium3DTilesSelection::Tile& tile,
                                              int32_t overlayTextureCoordinateID,
                                              const Cesium3DTilesSelection::RasterOverlayTile& rasterTile,
                                              void*) noexcept
{
    vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
    if (!ref_viewer)
        return;

    const Cesium3DTilesSelection::TileContent& content = tile.getContent();
    const Cesium3DTilesSelection::TileRenderContent* renderContent = content.getRenderContent();
    if (renderContent)
    {
        RenderResources* resources = static_cast<RenderResources*>(renderContent->getRenderResources());
        auto objects = _builder->detachRaster(tile, resources->model, overlayTextureCoordinateID, rasterTile);
        if (objects.first.valid())
        {
            auto compileResult = ref_viewer->compileManager->compile(objects.first);
            vsg::updateViewer(*ref_viewer, compileResult);
        }
        if (objects.second.valid())
        {
            _deletionQueue.run(ref_viewer);
            _deletionQueue.add(ref_viewer, objects.second);
        }
    }
}
