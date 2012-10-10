/*
 * Footstep Visualizer
 */

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

  /* 
   * Footstep
   */
  class Footstep
  {
    protected:
      pcl::PointNormal point_;
      Chirality::Kind chirality_;

    public:
      /*
       * Constructors/destructors
       */
      Footstep (pcl::PointNormal point, Chirality::Kind chirality=Chirality::left)
      {
        point_ = point;
        chirality_ = chirality;
      }

      Footstep (float x, float y, float z, pcl::Normal normal, Chirality::Kind chirality=Chirality::left)
      {
        pcl::PointNormal p;
        p.x = x; p.y = y; p.z = z;
        p.normal_x = normal.normal_x; p.normal_y = normal.normal_y; p.normal_z = normal.normal_z;
        Footstep (p, chirality);
      }

      virtual ~Footstep(){}

      /*
       * Getters/setters
       */
      inline pcl::PointNormal 
      getPoint() const {return point_;}

      inline Chirality::Kind
      getChirality() const {return chirality_;}

      /*
       * Comparators
       */
      bool
        operator< (const Footstep& other) const
        {
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
          float w;
          float h;
          float r;
        };
      };

      std::map<Chirality::Kind, pcl::RGB> color;
  };

  class DefaultFootstepStyle: public FootstepStyle
  {
    public:
      DefaultFootstepStyle ()
      {
        w = h = r = .1f;
        pcl::RGB red; red.r = 1.0f; red.g = red.b = 0.5f;
        pcl::RGB blue; blue.b = 1.0f; blue.r = blue.g = 0.5f;
        color[Chirality::left] = red;
        color[Chirality::right] = blue;
      }
  };

  /*
   * Visualizer
   */
  typedef std::map<std::string, FootstepVector> FootstepIDMap;
  typedef std::map<int, FootstepIDMap> FootstepViewportMap;
  class FootstepVisualizer: public pcl::visualization::PCLVisualizer
  {
    protected:
      FootstepViewportMap footstep_viewport_map_;
      FootstepStyle footstep_style_;
      std::map<Footstep, std::string> footstep_shape_ids_;

    public:
      /*
       * Constructors/destructors
       */
      FootstepVisualizer (const std::string &name = "", const bool create_interactor = true): pcl::visualization::PCLVisualizer::PCLVisualizer (name, create_interactor) {
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
      addFootsteps (const FootstepVector &footsteps, const std::string &id="footsteps", int viewport=0);

      bool addFootstep (const Footstep footstep, const std::string &id="footsteps", int viewport=0);

      bool removeFootsteps (const std::string &id="footsteps", int viewport=0);
  };
}
