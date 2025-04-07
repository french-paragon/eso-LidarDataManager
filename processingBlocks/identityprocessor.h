#ifndef INDENTITYPROCESSOR_H
#define INDENTITYPROCESSOR_H

#include <StereoVision/io/pointcloud_io.h>

/*!
 * \brief The IdentityProcessor class represent a processor block which return the point cloud as is.
 *
 * Many processor will impact only the geometry, or only the color, or only the attributes, or only skip points.
 * To avoid having to rewrite a bunch of identity functions everytime, they can inherit this class.
 *
 * Note that the functions assume that the source is a valid interface, and not nullptr.
 *
 * To deal with potential nullptr source, it is recommanded to use a factory pattern for the child classes,
 * and not return a processor in case the source is not valid.
 */
class IdentityProcessor : public StereoVision::IO::PointCloudPointAccessInterface
{
public:
    IdentityProcessor(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source);

    virtual StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> getPointPosition() const override;
    virtual std::optional<StereoVision::IO::PtColor<StereoVision::IO::PointCloudGenericAttribute>> getPointColor() const override;

    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeById(int id) const override;
    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeByName(const char* attributeName) const override;

    virtual std::vector<std::string> attributeList() const override;

    virtual bool gotoNext() override;
    virtual bool hasData() const override;

protected:
    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> _src;
};

/*!
 * \brief The BufferedIdentityProcessor class represent a processor block which return the point cloud as is, but buffers the points.
 *
 * Some processing function could benefits from a speedup by processing points in chunks
 * (e.g. if they want to multithread, or use SIMD instructions). This class automatize the buffering process.
 *
 * The class is templatized to select the underlying storage type for the point geometry.
 * Other attributes are stored as PointCloudGenericAttribute.
 */
template <typename GeometryT = float>
class BufferedIdentityProcessor : public StereoVision::IO::PointCloudPointAccessInterface
{
public:
    BufferedIdentityProcessor(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source, int bufferSize = 1024) :
        _src(std::move(source)),
        _maxBufferSize(bufferSize)
    {

    }

    virtual StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> getPointPosition() const override {
        StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> ret;
        ret.x = _xyz[_currentSubIndex].x;
        ret.y = _xyz[_currentSubIndex].y;
        ret.z = _xyz[_currentSubIndex].z;
        return ret;
    }
    virtual std::optional<StereoVision::IO::PtColor<StereoVision::IO::PointCloudGenericAttribute>> getPointColor() const override {
        if (_rgba.empty()) {
            return std::nullopt;
        }
        return _rgba[_currentSubIndex];
    }

    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeById(int id) const override {
        std::vector<std::string> keys = attributeList();
        return getAttributeByName(keys[id].c_str());
    }
    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeByName(const char* attributeName) const override {

        if (_attributes[_currentSubIndex].count(attributeName) <= 0) {
            return std::nullopt;
        }

        return _attributes[_currentSubIndex].at(std::string(attributeName));
    }

    virtual std::vector<std::string> attributeList() const override {
        return _src->attributeList();
    }

    virtual bool gotoNext() override {
        _currentSubIndex++;
        if (_currentSubIndex >= _xyz.size()) {
            return loadNextChunk();
        }
        return true;
    }
    virtual bool hasData() const override {
        if (_currentSubIndex >= _xyz.size()) {
            return _src->hasData();
        }
        return true;
    }

protected:
    /*!
     * \brief loadNextChunk load the data in the buffer
     * \return true in case of sucess, false otherwise.
     *
     * This function needs to be called in the constructors of child classes to load the first chunk of data.
     */
    bool loadNextChunk() {

        if (!_src->hasData()) {
            return false; //the end was already reached
        }

        _currentSubIndex = 0;
        _xyz.clear();
        _rgba.clear();
        _attributes.clear();

        _xyz.reserve(_maxBufferSize);
        _rgba.reserve(_maxBufferSize);
        _attributes.reserve(_maxBufferSize);

        bool hasColor = true;

        for (int i = 0; i < _maxBufferSize; i++) {
            _xyz.push_back(_src->castedPointGeometry<GeometryT>());

            if (hasColor) {
                auto optColor = _src->getPointColor();
                if (!optColor.has_value()) {
                    hasColor = false;
                } else {
                    _rgba.push_back(optColor.value());
                }
            }

            std::vector<std::string> attributeNames = _src->attributeList();
            _attributes.emplace_back();

            for (std::string const& name : attributeNames) {
                auto optVal = _src->getAttributeByName(name.c_str());
                if (optVal.has_value()) {
                    _attributes.back()[name] = optVal.value();
                }
            }

            bool ok = _src->gotoNext();

            if (!ok) {
                break;
            }
        }

        return afterChunkLoaded();
    }

    /*!
     * \brief afterChunkLoaded is a virtual callback after a chunk gets loaded
     * \return true in case of success, false otherwise.
     * default implementation does nothing and return true.
     */
    virtual bool afterChunkLoaded() {
        return true;
    }


protected:
    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> _src;

    int _maxBufferSize;
    int _currentSubIndex;
    std::vector<StereoVision::IO::PtGeometry<GeometryT>> _xyz;
    std::vector<StereoVision::IO::PtColor<StereoVision::IO::PointCloudGenericAttribute>> _rgba;
    std::vector<std::map<std::string, StereoVision::IO::PointCloudGenericAttribute>> _attributes;

};

#endif // INDENTITYPROCESSOR_H
