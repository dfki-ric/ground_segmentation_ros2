#ifndef GRID_MAP_DISPLAY_HPP
#define GRID_MAP_DISPLAY_HPP

#include <rviz_common/display.hpp>
#include <ground_segmentation/msg/grid_map.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <memory>

#include <OgreSceneManager.h>
#include <OgreManualObject.h>

#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <Ogre.h>

namespace ground_segmentation {

namespace grid_map_plugin{

class GridMapDisplay : public rviz_common::MessageFilterDisplay<ground_segmentation::msg::GridMap>
{
  Q_OBJECT
public:
  GridMapDisplay();
  virtual ~GridMapDisplay();

protected:
  virtual void onInitialize() override;
  virtual void reset() override;

private:
  void processMessage(ground_segmentation::msg::GridMap::ConstSharedPtr msg) override;
  void createTransparentMaterial();

  Ogre::MaterialPtr transparentMaterial_;
};

} // grid_map_plugin

} // ground_segmentation

#endif // GRID_MAP_DISPLAY_HPP
