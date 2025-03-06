#ifndef ATTRIBUTESETBASEDSELECTOR_H
#define ATTRIBUTESETBASEDSELECTOR_H

#include <StereoVision/io/pointcloud_io.h>

#include "identityprocessor.h"

#include <set>
#include <vector>

template<typename T>
static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupAttributeSetBasedSelectorImpl(
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
        std::string const& attributeName,
        int mode,
        std::set<T> const& valsset);

class AttributeSetBasedSelector : public IdentityProcessor
{
public:

    enum Mode {
        InSet,
        NotInSet
    };

    template<typename T>
    static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupAttributeSetBasedSelector(
            std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
            std::string const& attributeName,
            Mode mode,
            T const& container) {

        using ItemT = std::decay_t<decltype (container[0])>;

        std::set<ItemT> set;

        for (auto const& val : container) {
            set.insert(val);
        }

        if (set.empty()) {
            return nullptr;
        }

        return setupAttributeSetBasedSelectorImpl<ItemT>(source, attributeName, mode, set);
    }

    virtual bool gotoNext() override = 0;

protected:

    AttributeSetBasedSelector(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                              std::string const& attributeName);

    std::string _attributeName;
};

template<typename AttrT, AttributeSetBasedSelector::Mode mode>
class AttributeSetBasedSelectorImpl : public AttributeSetBasedSelector
{
public:
    AttributeSetBasedSelectorImpl(std::unique_ptr<PointCloudPointAccessInterface> && source,
                                  std::string const& attributeName,
                                  std::set<AttrT> const& valsset) :
        AttributeSetBasedSelector(std::move(source), attributeName),
        _comparisonVals(valsset)
    {

    }

    template<typename T>
    bool isAlikeImpl(T const& val, AttrT const& comp) {
    }

    virtual bool gotoNext() override {

        bool nextIsIn = false;
        bool sourceHasNotEnded = false;

        do {
            sourceHasNotEnded = _src->gotoNext();

            if (!sourceHasNotEnded) {
                break;
            }

            std::optional<StereoVision::IO::PointCloudGenericAttribute> attributeOpt =
                    getAttributeByName(_attributeName.c_str());


            if (mode == Mode::InSet and !attributeOpt.has_value()) {
                nextIsIn = true;
                break;
            }

            if (!attributeOpt.has_value()) {
                continue;
            }

            StereoVision::IO::PointCloudGenericAttribute& attribute = attributeOpt.value();

            if (mode == Mode::NotInSet) {
                nextIsIn = true;
            }

            using CompT = std::conditional_t<std::is_arithmetic_v<AttrT>, double, std::string>; //ensure the comparison type is a type that can holds all possible alternatives

            CompT attributeVal;
            CompT comparisonVal;

            if (std::holds_alternative<CompT>(attribute)) {
                attributeVal = std::get<CompT>(attribute);
            } else {
                attributeVal = StereoVision::IO::castedPointCloudAttribute<CompT>(attribute);
            }

            for (AttrT const& compVal : _comparisonVals) {

                comparisonVal = compVal;

                bool isAlike = comparisonVal == attributeVal;

                if (isAlike) {

                    if constexpr (mode == Mode::InSet) {
                        nextIsIn = true;
                        break;
                    }

                    if constexpr (mode == Mode::NotInSet) {
                        nextIsIn = false;
                        break;
                    }
                }
            }


        } while (!nextIsIn);

        return sourceHasNotEnded;
    }

protected:

    std::set<AttrT> _comparisonVals;


};

template<typename T>
static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupAttributeSetBasedSelectorImpl(
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
        std::string const& attributeName,
        int mode,
        std::set<T> const& valsset) {


    if (source == nullptr) {
        return nullptr;
    }

    if (valsset.empty()) {
        return nullptr;
    }

    if (attributeName.empty()) {
        return nullptr;
    }

    StereoVision::IO::PointCloudPointAccessInterface* ret = nullptr;

    switch (mode) {
    case AttributeSetBasedSelector::InSet:
        ret = new AttributeSetBasedSelectorImpl<T,AttributeSetBasedSelector::InSet>(std::move(source), attributeName, valsset);
        break;
    case AttributeSetBasedSelector::NotInSet:
        ret = new AttributeSetBasedSelectorImpl<T,AttributeSetBasedSelector::NotInSet>(std::move(source), attributeName, valsset);
        break;
    default:
        break;
    }

    return std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface>(ret);
}

#endif // ATTRIBUTESETBASEDSELECTOR_H
