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
 *         Author:  Peyton Randolph (), 
 *   Organization:  Carnegie Mellon University
 *
 * =====================================================================================
 */

#include "footstep_visualizer.h"

using namespace footsteps;

bool
FootstepVisualizer::addFootsteps (const FootstepVector &footsteps, const std::string &id, int viewport)
{
  // add footsteps one-by-one
  for (FootstepVector::const_iterator it = footsteps.begin(); it != footsteps.end(); it++)
    if (!addFootstep (*it, id, viewport))
      return false;

  return true;
}

bool
FootstepVisualizer::addFootstep (Footstep footstep, const std::string &id, int viewport)
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

  // generate shape id
  char sid[16];
  static const char alphanum[] =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz";

  for (int i = 0; i < 16; ++i) {
    sid[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
  }
  footstep_shape_ids_.insert(std::pair<Footstep, std::string>(footstep, (std::string)sid));

  // add footstep in visualizer
  pcl::RGB color = footstep_style_.color[footstep.getChirality()];
  addSphere<pcl::PointNormal> (footstep.getPoint(), footstep_style_.r, color.r, color.g, color.b, (std::string)sid, viewport);

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
