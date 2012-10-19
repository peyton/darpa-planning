/*
 * Footstep Visualizer
 */

#ifndef FOOTSTEP_VISUALIZER_H
#define FOOTSTEP_VISUALIZER_H

#include <iostream>
#include <map>
#include <utility>
#include <vector>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace footsteps
{
  /*
   * Constants
   */
  namespace Chirality {
    enum Kind{ left, right };
  };

  namespace Shape {
    enum Kind { sphere, box };
  }

  /*
   * Footstep
   */
  class Footstep
  {
    protected:
      pcl::PointNormal point_;
      float rotation_;
      Chirality::Kind chirality_;

    public:
      /*
       * Constructors/destructors
       */
      Footstep (pcl::PointNormal point, float rotation=0.0f, Chirality::Kind chirality=Chirality::left);

      Footstep (float x, float y, float z, pcl::Normal normal, float rotation, Chirality::Kind chirality=Chirality::left);

      virtual ~Footstep(){}

      /*
       * Getters/setters
       */
      inline pcl::PointNormal
      const getPoint() const { return point_; }

      inline float
      const getRotation() const { return rotation_; }

      inline Chirality::Kind
      const getChirality() const { return chirality_; }

      /*
       * Comparators
       */
      bool
        operator< (const Footstep& other) const;
  };
  typedef std::vector<Footstep> FootstepVector;

  /*
   * Style
   */

  class FootstepStyle
  {
    public:
      union{
        float dimensions[3];
        struct
        {
          float width;
          float height;
          float r;
        };
      };

      std::map<Chirality::Kind, pcl::RGB> color;
      Shape::Kind shape;
  };

  class DefaultFootstepStyle: public FootstepStyle
  {
    public:
      DefaultFootstepStyle ();
  };


  /*
   * Draw function type
   */
  class FootstepVisualizer;
  typedef void (FootstepVisualizer::*FootstepDrawFunction)(Footstep footstep, const std::string &id, int viewport, const FootstepStyle style);

  /*
   * Visualizer
   */
  typedef std::map<std::string, FootstepVector> FootstepIDMap;
  typedef std::map<int, FootstepIDMap> FootstepViewportMap;
  class FootstepVisualizer: public pcl::visualization::PCLVisualizer
  {
    protected:
      std::map<Shape::Kind, FootstepDrawFunction>footstep_draw_functions_;
      std::map<Footstep, std::string> footstep_shape_ids_;
      FootstepStyle footstep_style_;
      FootstepViewportMap footstep_viewport_map_;
      /*
       * Draw functions
       */

      void
      drawBox (Footstep footstep, const std::string &id, int viewport, const FootstepStyle style);

      void
      drawSphere (Footstep footstep, const std::string &id, int viewport, const FootstepStyle style);

      bool
      addCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation, double width, double height, double depth, float r, float g, float b, const std::string &id, int viewport);

    public:
      /*
       * Constructors/destructors
       */
      FootstepVisualizer (const std::string &name = "", const bool create_interactor = true): pcl::visualization::PCLVisualizer::PCLVisualizer (name, create_interactor) {
        // Initialize draw functions with defaults
        footstep_draw_functions_[Shape::box] = &footsteps::FootstepVisualizer::drawBox;
        footstep_draw_functions_[Shape::sphere] = &footsteps::FootstepVisualizer::drawSphere;

        // Initialize style
        footstep_style_ = DefaultFootstepStyle ();
      };

      FootstepVisualizer (int &argc, char **argv, const std::string &name = "",
          pcl::visualization::PCLVisualizerInteractorStyle* style = pcl::visualization::PCLVisualizerInteractorStyle::New (), const bool create_interactor = true): pcl::visualization::PCLVisualizer::PCLVisualizer (argc, argv, name, style, create_interactor) {
      };

      virtual ~FootstepVisualizer ()
      {

      }

      /*
       * Footstep handling
       */
      bool
      addFootsteps (const FootstepVector &footsteps, const std::string &id="footsteps", int viewport=0, FootstepStyle style=DefaultFootstepStyle());

      bool addFootstep (const Footstep footstep, const std::string &id="footsteps", int viewport=0, FootstepStyle style=DefaultFootstepStyle());

      bool removeFootsteps (const std::string &id="footsteps", int viewport=0);
  };

}
#endif
