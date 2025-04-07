#include <gtest/gtest.h>

#include <StereoVision/io/pointcloud_io.h>
#include <StereoVision/io/pcd_pointcloud_io.h>

#include "../processingBlocks/attributebasedselector.h"

#include <random>

using GenericCloud = StereoVision::IO::GenericPointCloud<float, float>;
using GenericCloudHeaderInterface = StereoVision::IO::GenericPointCloudHeaderInterface<float, float>;
using GenericCloudInterface = StereoVision::IO::GenericPointCloudPointAccessInterface<float, float>;

GenericCloud getRandomPointCloud(int nPoints) {

    using Point = GenericCloud::Point;

    std::random_device rd;
    std::default_random_engine re;
    re.seed(rd());

    constexpr float ptRange = 1000;
    constexpr float colorRange = 1;

    std::uniform_real_distribution<float> points_dist(-ptRange, ptRange);
    std::uniform_real_distribution<float> color_dist(0, colorRange);

    GenericCloud ret;

    for (int i = 0; i < nPoints; i++) {
        Point point;

        point.xyz.x = points_dist(re);
        point.xyz.y = points_dist(re);
        point.xyz.z = points_dist(re);

        point.rgba.r = color_dist(re);
        point.rgba.g = color_dist(re);
        point.rgba.b = color_dist(re);
        point.rgba.a = color_dist(re);

        ret.addPoint(point);
    }

    return ret;

}

class PointCloudTest : public testing::Test {
protected:
    static constexpr int nPoints = 1024;
    static constexpr char const* filter_attribute_name = "number";
    static constexpr std::array<int,2> filter_attribute_options = {42, 69};

    PointCloudTest() :
      testCloud(getRandomPointCloud(nPoints))
    {

        testCloud.addAttribute(filter_attribute_name);

        for (int i = 0; i < nPoints; i++) {
            testCloud[i].attributes[filter_attribute_name] = filter_attribute_options[i%2];
        }

    }

    GenericCloud testCloud;
};


TEST_F(PointCloudTest, TestAttributeBasedSelector) {

    AttributeBasedSelector::Comparator comparator = AttributeBasedSelector::Equal;
    StereoVision::IO::PointCloudGenericAttribute val0 = filter_attribute_options[0];
    StereoVision::IO::PointCloudGenericAttribute val1 = filter_attribute_options[1];

    for (StereoVision::IO::PointCloudGenericAttribute const& val : {val0, val1}) {
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> baseInterface =
                std::make_unique<GenericCloudInterface>(testCloud);

        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> selector =
                AttributeBasedSelector::setupAttributeBasedSelector(baseInterface,
                                                                    filter_attribute_name,
                                                                    comparator,
                                                                    val);

        int count = 0;

        bool hasMore = true;

        do {

            auto point = selector->castedPointGeometry<float>();
            auto color = selector->castedPointColor<float>();

            ASSERT_TRUE(color.has_value());

            auto attr = selector->getAttributeByName(filter_attribute_name);

            ASSERT_TRUE(attr.has_value());
            ASSERT_EQ(StereoVision::IO::castedPointCloudAttribute<int>(attr.value()),
                      StereoVision::IO::castedPointCloudAttribute<int>(val));

            count++;

            hasMore = selector->gotoNext();

        } while (hasMore);
    }

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
