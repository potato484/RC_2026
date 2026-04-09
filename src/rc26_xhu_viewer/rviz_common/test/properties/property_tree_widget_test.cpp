/*
 * Copyright (c) 2026
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <QApplication>

#include "rviz_common/config.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/property_tree_widget.hpp"

using rviz_common::Config;
using rviz_common::properties::Property;
using rviz_common::properties::PropertyTreeModel;
using rviz_common::properties::PropertyTreeWidget;

namespace
{

class SiblingRemovingProperty : public Property
{
public:
  explicit SiblingRemovingProperty(const QString & name, Property * parent = nullptr)
  : Property(name, QVariant(), "", parent)
  {}

  void setSiblingToRemove(Property * sibling)
  {
    sibling_to_remove_ = sibling;
  }

  QString getName() const override
  {
    if (!removed_ && sibling_to_remove_ && getParent()) {
      removed_ = true;
      Property * sibling = sibling_to_remove_;
      sibling_to_remove_ = nullptr;
      Property * detached = getParent()->takeChild(sibling);
      delete detached;
    }
    return Property::getName();
  }

private:
  mutable bool removed_{false};
  mutable Property * sibling_to_remove_{nullptr};
};

TEST(PropertyTreeWidget, loadToleratesChildListShrinkingDuringTraversal)
{
  auto * root = new Property("root");
  auto * first = new SiblingRemovingProperty("First", root);
  auto * second = new Property("Second", QVariant(), "", root);
  first->setSiblingToRemove(second);

  PropertyTreeModel model(root);
  PropertyTreeWidget widget;
  widget.setModel(&model);

  Config config;
  config.mapMakeChild("Expanded").listAppendNew().setValue("/First1");

  widget.load(config);

  EXPECT_EQ(1, root->numChildren());
  EXPECT_EQ("First", root->childAt(0)->getName().toStdString());
}

}  // namespace

int main(int argc, char ** argv)
{
  qputenv("QT_QPA_PLATFORM", QByteArray("offscreen"));
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
