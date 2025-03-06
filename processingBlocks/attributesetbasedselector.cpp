#include "attributesetbasedselector.h"

AttributeSetBasedSelector::AttributeSetBasedSelector(std::unique_ptr<PointCloudPointAccessInterface> && source,
                                                     std::string const& attributeName) :
    IdentityProcessor(std::move(source)),
    _attributeName(attributeName)
{

}
