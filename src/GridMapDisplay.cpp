#include "GridMapDisplay.hpp"
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace ground_segmentation {

namespace grid_map_plugin {

GridMapDisplay::GridMapDisplay()
{
  // Constructor implementation
  createTransparentMaterial();
  
}

GridMapDisplay::~GridMapDisplay()
{
  // Destructor implementation
}

void GridMapDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void GridMapDisplay::reset()
{
  MFDClass::reset();
}

// Function to create a transparent material
void GridMapDisplay::createTransparentMaterial() {
    transparentMaterial_ = Ogre::MaterialManager::getSingleton().create(
        "TransparentWireframe", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    
    transparentMaterial_->setReceiveShadows(false);
    
    auto technique = transparentMaterial_->getTechnique(0);
    if (!technique) {
        technique = transparentMaterial_->createTechnique();
    }
    
    auto pass = technique->getPass(0);
    if (!pass) {
        pass = technique->createPass();
    }
    
    pass->setLightingEnabled(true);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setDiffuse(1, 0, 0, 1);  // Fully transparent
    pass->setAmbient(1, 0, 0);
    pass->setSelfIllumination(1, 0, 0);
    pass->setPolygonMode(Ogre::PM_WIREFRAME);
}

void GridMapDisplay::processMessage(ground_segmentation::msg::GridMap::ConstSharedPtr msg)
{
  // Clear any previous visuals
  scene_manager_->destroyAllManualObjects();
 
  Ogre::Vector3 halfSize(msg->cell_size_x / 2, msg->cell_size_y / 2, msg->cell_size_z / 2);

  for (const auto& cell : msg->cells) {
    // Convert the box parameters to a visual representation
    Ogre::Vector3 position(cell.position.x, cell.position.y, cell.position.z);

    auto pass = transparentMaterial_->getTechnique(0)->getPass(0);
    pass->setDiffuse(cell.color.r, cell.color.g, cell.color.b, cell.color.a);  
    pass->setAmbient(cell.color.r, cell.color.g, cell.color.b);
    pass->setSelfIllumination(cell.color.r, cell.color.g, cell.color.b);

    // Create a box visual for each cell
    auto box = scene_manager_->createManualObject();
    box->begin("TransparentWireframe", Ogre::RenderOperation::OT_LINE_LIST);

    // Define the vertices of the box based on the center position of the cell
    Ogre::Vector3 corners[8] = {
    position + Ogre::Vector3(-halfSize.x, -halfSize.y, -halfSize.z),
    position + Ogre::Vector3(halfSize.x, -halfSize.y, -halfSize.z),
    position + Ogre::Vector3(halfSize.x, halfSize.y, -halfSize.z),
    position + Ogre::Vector3(-halfSize.x, halfSize.y, -halfSize.z),
    position + Ogre::Vector3(-halfSize.x, -halfSize.y, halfSize.z),
    position + Ogre::Vector3(halfSize.x, -halfSize.y, halfSize.z),
    position + Ogre::Vector3(halfSize.x, halfSize.y, halfSize.z),
    position + Ogre::Vector3(-halfSize.x, halfSize.y, halfSize.z)
    };

    // Define the indices for the lines that make up the box edges
    unsigned short indices[24] = {
      // Front face
      0, 1, 1, 2, 2, 3, 3, 0,
      // Back face
      4, 5, 5, 6, 6, 7, 7, 4,
      // Side faces
      0, 4, 1, 5, 2, 6, 3, 7
    };

    // Add vertices to the manual object
    for (int i = 0; i < 8; ++i) {
        box->position(corners[i]);
    }

    // Add indices to the manual object
    for (int i = 0; i < 24; ++i) {
        box->index(indices[i]);
    }

    box->end();
    scene_node_->attachObject(box);
  }
}

} // namespace grid_map_plugin
} // namespace ground_segmentation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ground_segmentation::grid_map_plugin::GridMapDisplay, rviz_common::Display)
