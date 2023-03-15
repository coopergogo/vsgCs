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

#include <glm/gtc/type_ptr.hpp>

#include <vsg/all.h>
#include <Cesium3DTilesSelection/Tile.h>

#include "Export.h"
#include "vsgCs/Config.h"

// Various random helper functions

namespace vsgCs
{
    void VSGCS_EXPORT startup();
    void VSGCS_EXPORT shutdown();
    vsg::ref_ptr<vsg::LookAt> VSGCS_EXPORT makeLookAtFromTile(const Cesium3DTilesSelection::Tile* tile,
                                                              double distance);

    inline void setdmat4(vsg::dmat4& vmat, const glm::dmat4x4& glmmat)
    {
        std::memcpy(vmat.data(), glm::value_ptr(glmmat), sizeof(double) * 16);
    }

    inline vsg::dmat4 glm2vsg(const glm::dmat4x4& glmmat)
    {
        vsg::dmat4 result;
        setdmat4(result, glmmat);
        return result;
    }

    inline vsg::dvec2 glm2vsg(const glm::dvec2& vec2)
    {
        return vsg::dvec2(vec2.x, vec2.y);
    }

    inline vsg::dvec3 glm2vsg(const glm::dvec3& vec3)
    {
        return vsg::dvec3(vec3.x, vec3.y, vec3.z);
    }

    inline bool isIdentity(const glm::dmat4x4& mat)
    {
        for (int c = 0; c < 4; ++c)
        {
            for (int r = 0; r < 4; ++r)
            {
                if (c == r)
                {
                    if (mat[c][r] != 1.0)
                        return false;
                }
                else
                {
                    if (mat[c][r] != 0.0)
                        return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Read an entire file as a string.
     */
    std::string readFile(const vsg::Path& filename, vsg::ref_ptr<const vsg::Options> options = {});

    std::vector<std::byte> readBinaryFile(const vsg::Path& filename,
                                          vsg::ref_ptr<const vsg::Options> options = {});

    vsg::ref_ptr<vsg::Data> readImageFile(const vsg::Path& filename,
                                          vsg::ref_ptr<const vsg::Options> options);


    /**
     * @brief Create an image from binary data.
     */
    vsg::ref_ptr<vsg::ImageInfo> VSGCS_EXPORT
    makeImage(gsl::span<const std::byte> data, bool useMipMaps, bool sRGB,
              VkSamplerAddressMode addressX = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
              VkSamplerAddressMode addressY = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
              VkFilter minFilter = VK_FILTER_LINEAR,
              VkFilter maxFilter = VK_FILTER_LINEAR);

    /**
     * @brief Load an image as vsg::Data.
     *
     * This returns vsg::Data because the vsg::Array2D template class does not have a more specific
     * superclass.
     */
    vsg::ref_ptr<vsg::Data> VSGCS_EXPORT loadImage(CesiumGltf::ImageCesium& image, bool useMipMaps, bool sRGB);

    int samplerLOD(const vsg::ref_ptr<vsg::Data>& data, bool generateMipMaps);

    /**
     * @brief create a VSG sampler.
     */
    vsg::ref_ptr<vsg::Sampler> VSGCS_EXPORT makeSampler(VkSamplerAddressMode addressX,
                                                        VkSamplerAddressMode addressY,
                                                        VkFilter minFilter,
                                                        VkFilter maxFilter,
                                                        int maxNumMipMaps);

    struct ReadRemoteImageResult
    {
        vsg::ref_ptr<vsg::ImageInfo> info;
        std::vector<std::string> errors{};
    };

    /**
     * @brief Read an image from a URL asynchronously.
     */
    CesiumAsync::Future<ReadRemoteImageResult>
    readRemoteImage(const std::string& url, bool compile = true);

    /**
     * @brief Creates a vsg::ref_ptr to a subclass using dynamic_cast from another ref_ptr.
     *
     * The VSG cast() member function templates use typeid, so they won't work if the actual object
     * is a sublcass of the desired cast.
     * @param TSubclass the target subclass
     * @param p the source ref_ptr
     * @return a ref_ptr that is valid if the cast succeeds, otherwise not.
     */
    template<typename TSubclass, typename TParent>
    vsg::ref_ptr<TSubclass> ref_ptr_cast(const vsg::ref_ptr<TParent>& p)
    {
        return vsg::ref_ptr<TSubclass>(dynamic_cast<TSubclass*>(p.get()));
    }

    template<typename T, typename TSource, typename... Args>
    vsg::ref_ptr<T> create_or(vsg::ref_ptr<TSource> source, Args&&... args)
    {
        auto downcast = ref_ptr_cast<T>(source);
        if (downcast.valid())
        {
            return downcast;
        }
        return T::create(args...);
    }

    // Templates for calling a function with the vertex indices of a triangle in the various
    // formats.

    template<typename F>
    void mapTriangleList(uint32_t numVertices, F&& f)
    {
        for (uint32_t i = 0; i < numVertices; i += 3)
        {
            f(i, i + 1, i + 2);
        }
    }

    template<typename F>
    void mapTriangleStrip(uint32_t numVertices, F&& f)
    {
        for (uint32_t i = 0; i < numVertices - 2; ++i)
        {
            f(i, i + 1 + (i % 2), i + 2 - (i % 2));
        }
    }

    template<typename F>
    void mapTriangleFan(uint32_t numVertices, F&& f)
    {
        for (uint32_t i = 0; i < numVertices - 2; ++i)
        {
            f(i + 1, i + 2, 0);
        }
    }

    std::optional<uint32_t> getUintSuffix(const std::string& prefix, const std::string& data);
}
