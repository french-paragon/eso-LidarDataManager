/*
 * This file is part of the LidarDataManager tool.
 * Copyright (c) 2025 Laurent Valentin Jospin <laurent.jospin@epfl.ch>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "attributebasedselector.h"

#include <type_traits>

#include <StereoVision/utils/types_manipulations.h>

template<bool cond>
struct ConditionalRef {

    template<typename Ttrue, typename Tfalse>
    static Ttrue& val(Ttrue & t, Tfalse & f) { return t; }

    template<typename Ttrue, typename Tfalse>
    static Ttrue const& val(Ttrue const& t, Tfalse const& f) { return t; }
};

template<>
struct ConditionalRef<false> {

    template<typename Ttrue, typename Tfalse>
    static Tfalse& val(Ttrue & t, Tfalse & f) { return f; }

    template<typename Ttrue, typename Tfalse>
    static Tfalse const& val(Ttrue const& t, Tfalse const& f) { return f; }
};

template<AttributeBasedSelector::Comparator comparator>
class AttributeBasedSelectorImpl : public AttributeBasedSelector
{
public:
    AttributeBasedSelectorImpl(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                               std::string const& attributeName,
                               StereoVision::IO::PointCloudGenericAttribute const& val) :
        AttributeBasedSelector(std::move(source), attributeName, val)
    {

        bool currentIsValid = std::visit([this] (auto const& param) {
            return isCurrentAttributeValid(param);
        }, _comparisonVal);

        if (!currentIsValid) {
            gotoNext();
        }

    }

    template<typename T>
    bool isCurrentAttributeValid(T const& val) {

        std::optional<StereoVision::IO::PointCloudGenericAttribute> attributeOpt =
                getAttributeByName(_attributeName.c_str());

        if (comparator == Comparator::Different and !attributeOpt.has_value()) {
            return true;
        }

        if (!attributeOpt.has_value()) {
            return false;
        }

        StereoVision::IO::PointCloudGenericAttribute& attribute = attributeOpt.value();

        using CompT = std::conditional_t<std::is_arithmetic_v<T>, double, std::string>; //ensure the comparison type is a type that can holds all possible alternatives

        CompT attributeVal;
        CompT comparisonVal;

        if (std::holds_alternative<CompT>(attribute)) {
            attributeVal = std::get<CompT>(attribute);
        } else {
            attributeVal = StereoVision::IO::castedPointCloudAttribute<CompT>(attribute);
        }

        comparisonVal = ConditionalRef<std::is_arithmetic_v<T> or std::is_same_v<T, std::string>>::val(val,CompT());

        if (comparator == Comparator::Equal) {
            if (attributeVal == comparisonVal) {
                return true;
            }
        }

        if (comparator == Comparator::Different) {
            if (attributeVal != comparisonVal) {
                return true;
            }
        }

        if (comparator == Comparator::Greather) {
            if (attributeVal > comparisonVal) {
                return true;
            }
        }

        if (comparator == Comparator::GreatherOrEqual) {
            if (attributeVal >= comparisonVal) {
                return true;
            }
        }

        if (comparator == Comparator::Smaller) {
            if (attributeVal < comparisonVal) {
                return true;
            }
        }

        if (comparator == Comparator::SmallerOrEqual) {
            if (attributeVal <= comparisonVal) {
                return true;
            }
        }

        return false;

    }

    template<typename T>
    bool gotoNextImpl(T const& val) {

        if (!std::is_arithmetic_v<T> and !std::is_same_v<T, std::string>) {
            return false;
        }

        bool nextIsIn = false;
        bool sourceHasNotEnded = false;

        do {
            sourceHasNotEnded = _src->gotoNext();

            if (!sourceHasNotEnded) {
                break;
            }

            nextIsIn = isCurrentAttributeValid(val);

        } while (!nextIsIn);

        return sourceHasNotEnded;
    }

    virtual bool gotoNext() override {
        return std::visit([this] (auto const& param) {
            return gotoNextImpl(param);
        },
        _comparisonVal);
    }
};

std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> AttributeBasedSelector::setupAttributeBasedSelector(
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
        std::string const& attributeName,
        Comparator comparator,
        StereoVision::IO::PointCloudGenericAttribute const& val) {

    if (source == nullptr) {
        return nullptr;
    }

    if(!std::visit([] (auto const& val) {
                   using T = std::decay_t<decltype (val)>;
                   return std::is_arithmetic_v<T> or
                   std::is_same_v<T, std::string>;}, val)) {
        return nullptr;
    }

    if (attributeName.empty()) {
        return nullptr;
    }

    StereoVision::IO::PointCloudPointAccessInterface* ret = nullptr;

    switch (comparator) {
    case Equal:
        ret = new AttributeBasedSelectorImpl<Equal>(std::move(source), attributeName, val);
        break;
    case Different:
        ret = new AttributeBasedSelectorImpl<Different>(std::move(source), attributeName, val);
        break;
    case Greather:
        ret = new AttributeBasedSelectorImpl<Greather>(std::move(source), attributeName, val);
        break;
    case GreatherOrEqual:
        ret = new AttributeBasedSelectorImpl<GreatherOrEqual>(std::move(source), attributeName, val);
        break;
    case Smaller:
        ret = new AttributeBasedSelectorImpl<Smaller>(std::move(source), attributeName, val);
        break;
    case SmallerOrEqual:
        ret = new AttributeBasedSelectorImpl<SmallerOrEqual>(std::move(source), attributeName, val);
        break;
    default:
        break;
    }

    return std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface>(ret);

}

AttributeBasedSelector::AttributeBasedSelector(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                                               std::string const& attributeName,
                                               StereoVision::IO::PointCloudGenericAttribute const& val) :
    IdentityProcessor(std::move(source)),
    _attributeName(attributeName),
    _comparisonVal(val)
{

}
