/*
 * =====================================================================================
 *
 *       Filename:  footstep_visualizer.cpp
 *
*    Description:   Machinery to visualize footsteps in point cloud data
 *
 *        Version:  1.0
 *        Created:  10/07/2012 07:29:06 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Peyton Randolph,
 *   Organization:  Carnegie Mellon University
 *
 * =====================================================================================
 */

#include "footstep_visualizer.h"

// VTK includes
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCellData.h>
#include <vtkWorldPointPicker.h>
#include <vtkPropPicker.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkTriangle.h>
#include <vtkTransform.h>

#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>4)
#include <vtkHardwareSelector.h>
#include <vtkSelectionNode.h>
#else
#include <vtkVisibleCellSelector.h>
#endif

#include <vtkSelection.h>
#include <vtkPointPicker.h>

#include <pcl/visualization/common/actor_map.h>


// Static helper functions
static std::string _rand_string(int size);

using namespace footsteps;

/*
 * Footstep
 */
Footstep::Footstep (pcl::PointNormal point, float rotation, Chirality::Kind chirality)
{
  // Initialize members
  point_ = point;
  rotation_ = rotation;
  chirality_ = chirality;
}

Footstep::Footstep (float x, float y, float z, pcl::Normal normal, float rotation, Chirality::Kind chirality)
{
  // Create a footstep point out of the passed-in xyz-normal
  pcl::PointNormal p;
  p.x = x; p.y = y; p.z = z;
  p.normal_x = normal.normal_x; p.normal_y = normal.normal_y; p.normal_z = normal.normal_z;

  Footstep (p, rotation, chirality);
}

bool
Footstep::operator< (const Footstep& other) const
{
  // Order footsteps by x, then y, then z, then chirality
  pcl::PointNormal p = getPoint();
  pcl::PointNormal op = other.getPoint();
  if (p.x == op.x)
  {
    if (p.y == op.y)
    {
      if (p.z == op.z)
        return getChirality() < other.getChirality();
      return p.z < op.z;
    }
    return p.y < op.y;
  }
  return p.x < op.x;
}

/*
 * Footstep Style
 */

DefaultFootstepStyle::DefaultFootstepStyle()
{
  // set dimensions to 12x30x5cm WxHxD
  width = .12f;
  height = .30f;
  r = .05f;

  // map default color to left -> red, right -> blue
  pcl::RGB red; red.r = 1.0f; red.g = red.b = 0.5f;
  pcl::RGB blue; blue.b = 1.0f; blue.r = blue.g = 0.5f;
  color[Chirality::left] = red;
  color[Chirality::right] = blue;

  // Set default shape to sphere
  shape = Shape::box;
}


/*
 * Footstep Visualizer
 */

bool
FootstepVisualizer::addFootsteps (const FootstepVector &footsteps, const std::string &id, int viewport, FootstepStyle style)
{
  // add footsteps one-by-one
  for (FootstepVector::const_iterator it = footsteps.begin(); it != footsteps.end(); it++)
    if (!addFootstep (*it, id, viewport, style))
      return false;

  return true;
}

bool
FootstepVisualizer::addFootstep (Footstep footstep, const std::string &id, int viewport, FootstepStyle style)
{
  // get id map for viewport, creating if necessary
  if (!footstep_viewport_map_.count(viewport))
    footstep_viewport_map_[viewport] = FootstepIDMap ();
  FootstepIDMap id_map = footstep_viewport_map_[viewport];

  // get footstep vector for id, creating if necessary
  if (!id_map.count(id))
    id_map[id] = FootstepVector ();
  FootstepVector footsteps = id_map[id];

  // add footstep to footsteps
  footsteps.push_back(footstep);

  // Update footsteps in id_map and id_map in footstep_viewport_map_
  id_map[id] = footsteps;
  footstep_viewport_map_[viewport] = id_map;

  // generate and store random shape id
  std::string sid = _rand_string(16);
  footstep_shape_ids_.insert(std::pair<Footstep, std::string>(footstep, sid));


  // add footstep in visualizer
  if (!footstep_draw_functions_.count(style.shape))
  {
    std::cerr << "No draw function for given shape" << std::endl;
    return false;
  }

  // Choose draw function based on style
  FootstepDrawFunction drawFunction = footstep_draw_functions_[style.shape];

  // Draw shape
  (this->*(drawFunction))(footstep, sid, viewport, style);

  return true;
}

bool
FootstepVisualizer::removeFootsteps (const std::string &id, int viewport)
{
  // check whether viewport exists
  if (!footstep_viewport_map_.count(viewport))
    return false;

  FootstepIDMap id_map = footstep_viewport_map_[viewport];

  // check whether id exists
  if (!id_map.count(id))
    return false;

  // get footsteps
  FootstepVector footsteps = id_map[id];

  // remove footsteps in visualizer
  for (FootstepVector::const_iterator it = footsteps.begin(); it != footsteps.end(); it++)
  {
    std::string sid = footstep_shape_ids_[*it];
    footstep_shape_ids_.erase(*it);
    removeShape(sid, viewport);
  }

  // delete footsteps in data model
  id_map.erase(id_map.find(id));
  // delete id map if necessary
  if (id_map.empty())
    footstep_viewport_map_.erase(footstep_viewport_map_.find(viewport));

  return true;
}

// Draw functions

void
FootstepVisualizer::drawBox (Footstep footstep, const std::string &id, int viewport, const FootstepStyle style)
{
  // Grab box color from style
  pcl::RGB color = footstep_style_.color[footstep.getChirality()];

  // Get footstep target point and turn into vector
  pcl::PointNormal pt = footstep.getPoint();
  Eigen::Vector3f location = Eigen::Vector3f(pt.x, pt.y, pt.z);

  // Set up default normal ([0, 1, 0], the default orientation of cubes in the visualizer),
  // and footstep normal
  const Eigen::Vector3f default_normal = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f normal = Eigen::Vector3f(pt.normal_x, pt.normal_y, pt.normal_z);

  // Produce a rotation from the default normal to the footstep's normal
  Eigen::Quaternionf rotation_to_normal;
  rotation_to_normal.setFromTwoVectors(default_normal, normal);

  // Produce a rotation about the footstep's normal's axis
  Eigen::AngleAxisf axis_angle = Eigen::AngleAxisf(footstep.getRotation(), normal);
  Eigen::Quaternionf rotation_about_normal = Eigen::Quaternionf(axis_angle);

  // The above operations rotated around `pt` at the center
  // We offset the cube so that it rests on top of the point.
  // To do this, we move the cube style.r/2 in the direction
  // of its normal vector
  Eigen::Vector3f offset;
  if(sqrt(normal.dot(normal)) != 0.0f)
    offset = normal * (1 / sqrt(normal.dot(normal)) * style.r / 2.0f);
  else
    offset = default_normal * style.r / 2.0f;


  // Add cube to the visualizer
  addCube(location + offset, (rotation_about_normal * rotation_to_normal).normalized(), style.width, style.r, style.height, color.r, color.g, color.b, id, viewport);

}

void
FootstepVisualizer::drawSphere (Footstep footstep, const std::string &id, int viewport, const FootstepStyle style)
{
  // Grab sphere color from footstep style
  pcl::RGB color = footstep_style_.color[footstep.getChirality()];

  // Add sphere to visualizer
  addSphere<pcl::PointNormal> (footstep.getPoint(), footstep_style_.r, color.r, color.g, color.b, id, viewport);
}

bool
FootstepVisualizer::addCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation, double width, double height, double depth, float r, float g, float b, const std::string &id, int viewport)
{
  /*
   * Note: this function copied from pcl_visualizer.cpp
   * with modifications to allow colors
   */

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  pcl::visualization::ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addCube] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = pcl::visualization::createCube (translation, rotation, width, height, depth);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  actor->GetProperty ()->SetLighting (true);
  actor->GetProperty ()->SetColor(r, g, b);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

/*
 * Static helper functions
 */

// Generate a random alphanumeric string of a given size
static std::string _rand_string(int size)
{
  char sid[size];
  static const char alphanum[] =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz";

  for (int i = 0; i < size; ++i)
  {
    sid[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
  }

 return (std::string)sid;
}
